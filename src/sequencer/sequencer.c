#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <math.h>
#include "sequencer.h"
#include "can_driver.h"
#include "mcc_driver.h"
#include "logger.h"
#include "lcm_interface.h"
#include "encoding.h"
#include "dyno.h"
#include "motor_sim.h"

/* ---- Sequencer globals ---- */
int                abortTest       = FALSE;
int                abortS1         = FALSE;
sem_t              semS1;
struct timespec    start_time_val;
double             start_realtime;
unsigned long long sequencePeriods;

static timer_t            timer_1;
static struct itimerspec  itime      = {{1,0},{1,0}};
static struct itimerspec  last_itime;
static unsigned long long seqCnt    = 0;

/* ---- sequencer_init ---- */
void sequencer_init(void)
{
    if (sem_init(&semS1, 0, 0)) {
        printf("sequencer_init: failed to initialize semS1\n");
        exit(-1);
    }
}

/* ---- sequencer_start ---- */
void sequencer_start(unsigned long long periods)
{
    int flags = 0;
    sequencePeriods = periods;

    timer_create(CLOCK_MONOTONIC, NULL, &timer_1);
    signal(SIGALRM, (void(*)()) Sequencer);

    itime.it_interval.tv_sec  = 0;
    itime.it_interval.tv_nsec = 2500000; /* 2.5 ms → 400 Hz sequencer */
    itime.it_value.tv_sec     = 0;
    itime.it_value.tv_nsec    = 2500000;
    timer_settime(timer_1, flags, &itime, &last_itime);
}

/* ---- Sequencer (SIGALRM handler) ---- */
void Sequencer(int id)
{
    int flags = 0;

    seqCnt++;
    if (seqCnt >= 400)
        seqCnt = 1;

    /* Release Service_1 every tick → 1000 Hz */
    sem_post(&semS1);

    if (abortTest || (seqCnt >= sequencePeriods)) {
        itime.it_interval.tv_sec  = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec     = 0;
        itime.it_value.tv_nsec    = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
        printf("Disabling sequencer interval timer with abort=%d and %llu of %llu\n",
               abortTest, seqCnt, sequencePeriods);
        sem_post(&semS1);
        abortS1 = TRUE;
    }
}

/* ---- Service_1 (1000 Hz RT thread) ---- */
void *Service_1(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S1Cnt = 0;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    printf("S1 thread started @ sec=%6.9lf\n", current_realtime - start_realtime);

    while (!abortS1) {
        sem_wait(&semS1);
        S1Cnt++;

        /* Snapshot: lock only long enough to copy — never hold during I/O */
        dyno_t cmd;
        pthread_mutex_lock(&dyno_mutex);
        cmd = dyno;
        pthread_mutex_unlock(&dyno_mutex);

        unsigned char data[8];

        /* ============================================================
         * VIRTUAL (SIM) PATH
         * Runs RK4 motor model instead of CAN I/O for motor-driving
         * modes.  Other states (CALIBRATE, TOGGLE_LED, DYNO_TEST…)
         * fall through to the hardware switch below so they still
         * work — just note DYNO_TEST will attempt CAN in sim mode
         * which is harmless (CAN frames are sent but motor is off).
         * ============================================================ */
        int did_sim = 0;
        if (cmd.sim_mode) {
            switch (cmd.state) {
                case CMD_VOLTAGE_FOC:
                case CMD_CURRENT:
                case CMD_TORQUE:
                case CMD_VELOCITY:
                case CMD_POSITION:
                case CMD_NONE:
                case CMD_OFF: {
                    motor_sim_input_t in = {
                        .vq_cmd     = (float)cmd.vq_cmd,
                        .iq_cmd     = (float)cmd.iq_cmd,
                        .torque_cmd = (float)cmd.torque_cmd,
                        .vel_cmd    = (float)cmd.vel_cmd,
                        .pos_cmd    = (float)cmd.pos_cmd,
                        .kp         = (float)cmd.kp,
                        .kd         = (float)cmd.kd
                    };
                    float applied_vq = motor_sim_step(cmd.state, &in, SIM_H);

                    motor_data.iq       = sim_state.iq;
                    motor_data.velocity = sim_state.omega;
                    motor_data.position = sim_state.theta;
                    motor_data.vq       = applied_vq;
                    motor_data.cmd_id   = cmd.state;

                    float torque_sim = SIM_Kt * sim_state.iq;
                    float elec_power = sim_state.iq * applied_vq;
                    float mech_power = torque_sim * sim_state.omega;
                    float efficiency = (elec_power != 0.0f)
                                       ? mech_power / elec_power : 0.0f;

                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);

                    pthread_mutex_lock(&dyno_mutex);
                    dyno.torque_dyno = torque_sim;
                    dyno.elec_power  = elec_power;
                    dyno.mech_power  = mech_power;
                    dyno.efficiency  = efficiency;
                    pthread_mutex_unlock(&dyno_mutex);

                    did_sim = 1;
                    break;
                }
                default:
                    break; /* fall through to hardware switch */
            }
        }

        if (!did_sim)
        switch (cmd.state) {

            case CMD_CALIBRATE:
                memset(data, 0, sizeof(data));
                data[1] = CMD_CALIBRATE;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_NONE;
                FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            /* --------------------------------------------------------
             * CMD_CAL_MOTOR — automated parameter identification.
             *
             * Five sequential phases (each driven by cal_tick):
             *   0  Resistance  R   — DC Vq, steady-state Iq
             *   1  Inductance  L   — sinusoidal Vq, RMS impedance
             *   2  Torque cst  Kt  — current steps, MCC torque
             *   3  Damping     B   — velocity sweep, steady Iq
             *   4  Inertia     J   — Iq step, velocity ramp rate
             *
             * Works in both VIRTUAL (RK4) and HARDWARE (CAN) mode.
             * Results are printed to stdout and saved to calibration.csv.
             * -------------------------------------------------------- */
            case CMD_CAL_MOTOR: {
                /* Persistent accumulators — valid only during calibration */
                static float  R_est  = 0, L_est = 0, Kt_est = 0, B_est = 0;
                static double sum_iq = 0, sum_vq2 = 0, sum_iq2 = 0, sum_n = 0;
                static double kt_iq[4],  kt_tau[4];
                static double b_omega[4], b_iq[4];
                static float  omega0 = 0;

                int phase = cmd.cal_phase;
                int tick  = cmd.cal_tick;

                /* Reset everything at the very first tick */
                if (phase == 0 && tick == 0) {
                    R_est = L_est = Kt_est = B_est = 0;
                    sum_iq = sum_vq2 = sum_iq2 = sum_n = 0;
                    omega0 = 0;
                    for (int i = 0; i < 4; i++)
                        kt_iq[i] = kt_tau[i] = b_omega[i] = b_iq[i] = 0;
                    if (cmd.sim_mode) {
                        motor_sim_init();
                        /* Phases 0 & 1 measure electrical parameters with rotor
                         * locked.  Use a very large inertia so dω/dt ≈ 0 — no
                         * back-EMF contamination, no NaN from force-zeroing ω. */
                        sim_params.J = 1e6f;
                    }
                    printf("\n[CAL] ===== Motor calibration started (%s) =====\n",
                           cmd.sim_mode ? "VIRTUAL" : "HARDWARE");
                }

                /* ── Determine stimulus ── */
                static const float iq_levels[4]  = {0.5f, 1.0f, 1.5f, CAL_IQ_MAX};
                static const float vel_levels[4] = {5.0f, 10.0f, 20.0f, CAL_VEL_MAX};

                int   cal_cmd     = CMD_VOLTAGE_FOC;
                float cal_vq_now  = 0.0f;
                float cal_iq_now  = 0.0f;
                float cal_vel_now = 0.0f;
                int   step        = 0;

                switch (phase) {
                    case 0:
                        cal_vq_now = CAL_VQ_DC;
                        break;
                    case 1:
                        cal_vq_now = CAL_VQ_AC *
                            sinf(2.0f * (float)M_PI * CAL_FREQ * tick * SIM_H);
                        break;
                    case 2:
                        step = (tick / 1000 < 4) ? tick / 1000 : 3;
                        cal_cmd    = CMD_CURRENT;
                        cal_iq_now = iq_levels[step];
                        break;
                    case 3:
                        cal_cmd    = CMD_CURRENT;
                        cal_iq_now = CAL_B_IQ;
                        break;
                    case 4:
                        cal_cmd    = CMD_CURRENT;
                        cal_iq_now = CAL_J_IQ;
                        break;
                    default:
                        break;
                }

                /* ── Drive motor: virtual or hardware ── */
                float meas_iq = 0, meas_omega = 0, meas_theta = 0;
                float meas_torque = 0, applied_vq = 0;

                if (cmd.sim_mode) {
                    motor_sim_input_t in = {
                        .vq_cmd     = cal_vq_now,
                        .iq_cmd     = cal_iq_now,
                        .vel_cmd    = cal_vel_now,
                        .kp         = CAL_VEL_KP,
                        .kd         = CAL_VEL_KD
                    };
                    applied_vq  = motor_sim_step(cal_cmd, &in, SIM_H);
                    meas_iq     = sim_state.iq;
                    meas_omega  = sim_state.omega;
                    meas_theta  = sim_state.theta;
                    meas_torque = SIM_Kt * sim_state.iq;

                    motor_data.iq       = meas_iq;
                    motor_data.velocity = meas_omega;
                    motor_data.position = meas_theta;
                    motor_data.vq       = applied_vq;
                    motor_data.cmd_id   = cal_cmd;
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                } else {
                    memset(data, 0, sizeof(data));
                    switch (cal_cmd) {
                        case CMD_VOLTAGE_FOC: {
                            int vq_int = float_to_uint16(cal_vq_now, -V_MAX, V_MAX);
                            data[1] = CMD_VOLTAGE_FOC;
                            data[2] = vq_int >> 8; data[3] = vq_int & 0xFF;
                            break;
                        }
                        case CMD_CURRENT: {
                            int iq_int = float_to_uint16(cal_iq_now, -I_MAX, I_MAX);
                            data[1] = CMD_CURRENT;
                            data[2] = iq_int >> 8; data[3] = iq_int & 0xFF;
                            break;
                        }
                        case CMD_VELOCITY: {
                            int vel_int = float_to_uint16(cal_vel_now,-SPEED_MAX,SPEED_MAX);
                            int kp_int  = float_to_uint8(CAL_VEL_KP, 0.0f, KP_MAX);
                            int kd_int  = float_to_uint8(CAL_VEL_KD, 0.0f, 0.1f);
                            data[1] = CMD_VELOCITY;
                            data[2] = vel_int >> 8; data[3] = vel_int & 0xFF;
                            data[4] = kp_int;       data[5] = kd_int;
                            break;
                        }
                    }
                    sendCANFrame(CAN_ID, data, 8);
                    if (receiveCANFrame(&motor_data) == 0) {
                        applied_vq  = (cal_cmd == CMD_VOLTAGE_FOC) ? cal_vq_now
                                                                    : motor_data.vq;
                        meas_iq     = motor_data.iq;
                        meas_omega  = motor_data.velocity;
                        meas_theta  = motor_data.position;
                        meas_torque = mcc_read_torque();
                    }
                    motor_data.cmd_id = cal_cmd;
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                }

                /* ── Log raw data every tick ── */
                clock_gettime(MY_CLOCK_TYPE, &current_time_val);
                current_realtime = realtime(&current_time_val);
                float cmd_input = (cal_cmd == CMD_VELOCITY) ? cal_vel_now
                                : (cal_cmd == CMD_CURRENT)  ? cal_iq_now
                                :                             cal_vq_now;
                log_push(&(log_entry_t){
                    .log_type    = LOG_TYPE_CAL,
                    .time_s      = current_realtime - start_realtime,
                    .vq_cmd      = cmd_input,
                    .vq_msr      = applied_vq,
                    .iq          = meas_iq,
                    .torque_dyno = meas_torque,
                    .velocity    = meas_omega,
                    .position    = meas_theta,
                    .elec_power  = meas_iq * applied_vq,
                    .mech_power  = meas_torque * meas_omega,
                    .efficiency  = (float)phase   /* phase ID in efficiency col */
                });

                /* ── Accumulate and advance phases ── */
                int next_phase = phase;
                int next_tick  = tick + 1;

                switch (phase) {

                    /* Phase 0 — Resistance (800 ticks = 800 ms) */
                    case 0:
                        if (tick >= 200) { sum_iq += meas_iq; sum_n++; }
                        if (tick == 799) {
                            R_est = (sum_n > 0)
                                    ? (float)(CAL_VQ_DC / (sum_iq / sum_n)) : 0;
                            printf("[CAL] Phase 0 — R = %.4f Ohm\n", R_est);
                            sum_iq = 0; sum_n = 0;
                            next_phase = 1; next_tick = 0;
                        }
                        break;

                    /* Phase 1 — Inductance (1200 ticks = 1.2 s) */
                    case 1:
                        if (tick >= 600) {
                            sum_vq2 += cal_vq_now * cal_vq_now;
                            sum_iq2 += meas_iq    * meas_iq;
                            sum_n++;
                        }
                        if (tick == 1199) {
                            float Vq_rms = sqrtf((float)(sum_vq2 / sum_n));
                            float Iq_rms = sqrtf((float)(sum_iq2 / sum_n));
                            float Z      = (Iq_rms > 1e-6f) ? Vq_rms / Iq_rms : 0;
                            float Z2R2   = Z*Z - R_est*R_est;
                            L_est = (Z2R2 > 0)
                                    ? sqrtf(Z2R2) / (2.0f * (float)M_PI * CAL_FREQ)
                                    : 0;
                            printf("[CAL] Phase 1 — L = %.4f mH\n", L_est * 1000.0f);
                            sum_vq2 = 0; sum_iq2 = 0; sum_n = 0;
                            if (cmd.sim_mode) motor_sim_init(); /* Kt test starts from rest */
                            next_phase = 2; next_tick = 0;
                        }
                        break;

                    /* Phase 2 — Kt (4000 ticks: 4 steps × 1000) */
                    case 2:
                        step = (tick / 1000 < 4) ? tick / 1000 : 3;
                        if (tick % 1000 >= 800) {  /* last 200 ticks of each step */
                            kt_iq[step]  += meas_iq;
                            kt_tau[step] += meas_torque;
                        }
                        if (tick == 3999) {
                            double num = 0, den = 0;
                            for (int i = 0; i < 4; i++) {
                                double ia = kt_iq[i]  / 200.0;
                                double ta = kt_tau[i] / 200.0;
                                num += ia * ta;
                                den += ia * ia;
                            }
                            Kt_est = (den > 1e-9) ? (float)(num / den) : 0;
                            printf("[CAL] Phase 2 — Kt = %.6f Nm/A\n", Kt_est);
                            if (cmd.sim_mode) motor_sim_init(); /* B sweep starts from rest */
                            next_phase = 3; next_tick = 0;
                        }
                        break;

                    /* Phase 3 — Damping B (4000 ticks = 4 s, 2-point exponential fit).
                     *
                     * CMD_CURRENT (rk4_mech) bypasses the electrical equation and is
                     * unconditionally stable at any step size.
                     *
                     * Physics: ω(t) = ω_ss·(1 − e^(−t/τm))
                     *   ω_ss = Kt·Iq / B ,  τm = J / B
                     * Sampling at T/2 (tick 2000 = 2 s) and T (tick 3999 = 4 s):
                     *   ω_ss = ω_a² / (2·ω_a − ω_b)
                     *   B    = Kt·Iq / ω_ss
                     * When ω_a ≈ ω_b (near SS), falls back to B = Kt·Iq / ω_a.
                     */
                    case 3:
                        if (tick == 2000) omega0 = meas_omega;  /* reuse omega0 */
                        if (tick == 3999) {
                            float wa    = omega0;
                            float wb    = meas_omega;
                            float x_val = (wa > 1e-4f) ? (wb / wa - 1.0f) : -1.0f;
                            float omega_ss_est;
                            if (x_val < 0.05f && wa > 1e-4f) {
                                /* Motor near or at SS — direct measurement */
                                omega_ss_est = wa;
                            } else if (x_val > 0.0f && x_val < 1.0f) {
                                /* 2-point fit */
                                float denom = 2.0f * wa - wb;
                                omega_ss_est = (denom > 1e-4f) ? (wa * wa / denom) : wa;
                            } else {
                                omega_ss_est = 0.0f;
                            }
                            B_est = (omega_ss_est > 1e-4f)
                                    ? Kt_est * CAL_B_IQ / omega_ss_est : 0;
                            printf("[CAL] Phase 3 — B = %.6f Nm*s/rad\n", B_est);
                            if (cmd.sim_mode) motor_sim_init(); /* J test starts from rest */
                            next_phase = 4; next_tick = 0;
                        }
                        break;

                    /* Phase 4 — Inertia J (800 ticks = 800 ms) */
                    case 4:
                        if (tick == 80)  omega0 = meas_omega;
                        if (tick == 680) {
                            float omega1    = meas_omega;
                            float alpha     = (omega1 - omega0) / (600.0f * SIM_H);
                            float omega_avg = (omega0 + omega1) * 0.5f;
                            float J_est     = (fabsf(alpha) > 0.1f)
                                ? (Kt_est * CAL_J_IQ - B_est * omega_avg) / alpha
                                : 0;
                            printf("[CAL] Phase 4 — J = %.6f kg*m^2\n\n", J_est);
                            printf("[CAL] ===== RESULTS =====\n");
                            printf("[CAL]   R  = %.4f  Ohm      (ref: %.4f)\n",
                                   R_est,  SIM_R);
                            printf("[CAL]   L  = %.4f  mH       (ref: %.4f)\n",
                                   L_est * 1000.0f, SIM_L * 1000.0f);
                            printf("[CAL]   Kt = %.6f  Nm/A     (ref: %.6f)\n",
                                   Kt_est, (float)SIM_Kt);
                            printf("[CAL]   B  = %.6f  Nm*s/rad (ref: %.6f)\n",
                                   B_est,  SIM_B);
                            printf("[CAL]   J  = %.6f  kg*m^2   (ref: %.6f)\n",
                                   J_est,  SIM_J);
                            printf("[CAL] ===================\n\n");
                            log_flush_cal();
                            next_phase = 5; next_tick = 0;
                        }
                        break;

                    /* Phase 5 — Done */
                    default:
                        pthread_mutex_lock(&dyno_mutex);
                        dyno.state     = CMD_NONE;
                        dyno.cal_phase = 0;
                        dyno.cal_tick  = 0;
                        pthread_mutex_unlock(&dyno_mutex);
                        break;
                }

                /* Advance counters (skip if phase 5 already reset state) */
                if (phase < 5) {
                    pthread_mutex_lock(&dyno_mutex);
                    dyno.cal_phase = next_phase;
                    dyno.cal_tick  = next_tick;
                    pthread_mutex_unlock(&dyno_mutex);
                }
                break;
            }

            case CMD_COULOMB_FRICTION: {
                int   tick      = cmd.cf_tick;
                int   test_done = (tick >= CF_TOTAL_TICKS - 1);
                float vel_target;

                if (tick < CF_TEST_TICKS) {
                    int step = tick / CF_STEP_TICKS;
                    if (step < CF_N_UP_STEPS)
                        vel_target = -CF_VEL_MAX + step * CF_VEL_STEP;
                    else
                        vel_target = CF_VEL_MAX - CF_VEL_STEP
                                     - (step - CF_N_UP_STEPS) * CF_VEL_STEP;
                } else {
                    vel_target = 0.0f;
                }

                float meas_omega = 0, meas_theta = 0, meas_iq = 0;
                float meas_torque = 0, applied_vq = 0;

                if (cmd.sim_mode) {
                    motor_sim_input_t in = {
                        .vel_cmd = vel_target,
                        .kp      = CF_KP,
                        .kd      = CF_KD
                    };
                    applied_vq  = motor_sim_step(CMD_VELOCITY, &in, SIM_H);
                    meas_iq     = sim_state.iq;
                    meas_omega  = sim_state.omega;
                    meas_theta  = sim_state.theta;
                    meas_torque = SIM_Kt * sim_state.iq;
                    motor_data.iq       = meas_iq;
                    motor_data.velocity = meas_omega;
                    motor_data.position = meas_theta;
                    motor_data.vq       = applied_vq;
                    motor_data.cmd_id   = CMD_VELOCITY;
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                } else {
                    int vel_int = float_to_uint16(vel_target, -SPEED_MAX, SPEED_MAX);
                    int kp_int  = float_to_uint8(CF_KP, 0.0f, KP_MAX / GAIN);
                    int kd_int  = float_to_uint8(CF_KD, 0.0f, 0.1f / GAIN);
                    memset(data, 0, sizeof(data));
                    data[1] = CMD_VELOCITY;
                    data[2] = vel_int >> 8;
                    data[3] = vel_int & 0xFF;
                    data[4] = kp_int;
                    data[5] = kd_int;
                    sendCANFrame(CAN_ID, data, 8);
                    motor_data.cmd_id = CMD_VELOCITY;
                    if (receiveCANFrame(&motor_data) == 0) {
                        meas_iq     = motor_data.iq;
                        meas_omega  = motor_data.velocity;
                        meas_theta  = motor_data.position;
                        applied_vq  = motor_data.vq;
                        meas_torque = mcc_read_torque();
                    }
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                }

                /* Inline log push — avoids a second mcc_read_torque() call in
                 * the generic manual log block below. */
                if (cmd.log_active) {
                    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
                    current_realtime = realtime(&current_time_val);
                    log_push(&(log_entry_t){
                        .log_type    = LOG_TYPE_MANUAL,
                        .time_s      = current_realtime - start_realtime,
                        .vq_cmd      = vel_target,
                        .vq_msr      = applied_vq,
                        .iq          = meas_iq,
                        .torque_dyno = meas_torque,
                        .velocity    = meas_omega,
                        .position    = meas_theta,
                        .elec_power  = meas_iq * applied_vq,
                        .mech_power  = meas_torque * meas_omega,
                        .efficiency  = 0.0f
                    });
                }

                pthread_mutex_lock(&dyno_mutex);
                dyno.torque_dyno = meas_torque;
                if (test_done) {
                    dyno.log_active = 0;
                    dyno.cf_tick    = 0;
                    dyno.state      = CMD_OFF;
                } else {
                    dyno.cf_tick = tick + 1;
                }
                pthread_mutex_unlock(&dyno_mutex);

                if (test_done)
                    log_flush_manual();

                break;
            }

            case CMD_OFF:
                memset(data, 0, sizeof(data));
                data[1] = CMD_OFF;
                sendCANFrame(CAN_ID, data, 8);
                if (receiveCANFrame(&motor_data) == 0) {
                    motor_data.cmd_id = CMD_NONE;
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                }
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            case CMD_VOLTAGE_FOC: {
                int vq_int = float_to_uint16(cmd.vq_cmd, -V_MAX, V_MAX);
                memset(data, 0, sizeof(data));
                data[1] = CMD_VOLTAGE_FOC;
                data[2] = vq_int >> 8;
                data[3] = vq_int & 0xFF;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_VOLTAGE_FOC;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);

                float torque_dyno = mcc_read_torque();
                float elec_power  = motor_data.iq * cmd.vq_cmd;
                float mech_power  = torque_dyno * motor_data.velocity;
                float efficiency  = (elec_power != 0.0f) ? mech_power / elec_power : 0.0f;
                pthread_mutex_lock(&dyno_mutex);
                dyno.torque_dyno = torque_dyno;
                dyno.elec_power  = elec_power;
                dyno.mech_power  = mech_power;
                dyno.efficiency  = efficiency;
                pthread_mutex_unlock(&dyno_mutex);
                break;
            }

            case CMD_POSITION: {
                int pos_int = float_to_uint16(cmd.pos_cmd, -ONE_REV * GR, ONE_REV * GR);
                int kp_int  = float_to_uint8(cmd.kp, 0.0f, KP_MAX / GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0f, 0.1f / GAIN);
                memset(data, 0, sizeof(data));
                data[0] = CMD_POSITION;
                data[1] = CMD_POSITION;
                data[2] = pos_int >> 8;
                data[3] = pos_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_POSITION;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                break;
            }

            case CMD_VELOCITY: {
                int vel_int = float_to_uint16(cmd.vel_cmd, -SPEED_MAX, SPEED_MAX);
                int kp_int  = float_to_uint8(cmd.kp, 0.0f, KP_MAX / GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0f, 0.1f / GAIN);
                memset(data, 0, sizeof(data));
                data[1] = CMD_VELOCITY;
                data[2] = vel_int >> 8;
                data[3] = vel_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_VELOCITY;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                break;
            }

            case CMD_DYNO_TEST: {
                double vq_cmd = cmd.vq_cmd;
                int    cycle  = cmd.cycle_counter;
                int    total  = cmd.total_test_counter;

                if (total <= cmd.N - cmd.N_ramp_down) {
                    if (cycle <= (int)(cmd.ramp_time / cmd.h))
                        vq_cmd = vq_cmd - cmd.dv;
                    cycle++;
                    if (cycle > (int)(cmd.ramp_time / cmd.h) + (int)(cmd.meas_time / cmd.h))
                        cycle = 1;
                } else {
                    vq_cmd = vq_cmd + cmd.dv;
                }
                total++;

                int test_done = (total >= cmd.N);
                if (test_done) {
                    vq_cmd = 0.0;
                    total  = 1;
                    cycle  = 0;
                }

                int vq_int = float_to_uint16(vq_cmd, -V_MAX, V_MAX);
                memset(data, 0, sizeof(data));
                data[1] = test_done ? CMD_OFF : CMD_DYNO_TEST;
                data[2] = vq_int >> 8;
                data[3] = vq_int & 0xFF;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = test_done ? CMD_OFF : CMD_DYNO_TEST;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);

                float torque_dyno = mcc_read_torque();
                float elec_power  = motor_data.iq * vq_cmd;
                float mech_power  = torque_dyno * motor_data.velocity;
                float efficiency  = (elec_power != 0.0f) ? mech_power / elec_power : 0.0f;
                clock_gettime(MY_CLOCK_TYPE, &current_time_val);
                current_realtime = realtime(&current_time_val);
                log_push(&(log_entry_t){
                    .log_type    = LOG_TYPE_DYNO,
                    .time_s      = current_realtime - start_realtime,
                    .vq_cmd      = vq_cmd,
                    .vq_msr      = motor_data.vq,
                    .iq          = motor_data.iq,
                    .torque_dyno = torque_dyno,
                    .velocity    = motor_data.velocity,
                    .position    = motor_data.position,
                    .elec_power  = elec_power,
                    .mech_power  = mech_power,
                    .efficiency  = efficiency
                });

                if (test_done)
                    log_flush_dyno(); /* flush ring, wait 10 ms, close dyno_test */

                pthread_mutex_lock(&dyno_mutex);
                dyno.vq_cmd             = vq_cmd;
                dyno.cycle_counter      = cycle;
                dyno.total_test_counter = total;
                dyno.torque_dyno        = torque_dyno;
                dyno.elec_power         = elec_power;
                dyno.mech_power         = mech_power;
                dyno.efficiency         = efficiency;
                if (test_done) dyno.state = CMD_OFF;
                pthread_mutex_unlock(&dyno_mutex);
                break;
            }

            case CMD_TOGGLE_LED:
                memset(data, 0, sizeof(data));
                data[0] = cmd.led_flag;
                data[1] = CMD_TOGGLE_LED;
                sendCANFrame(CAN_ID, data, 8);
                if (receiveCANFrame(&motor_data) == 0) {
                    motor_data.cmd_id = CMD_NONE;
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                }
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            case CMD_CURRENT: {
                int iq_int = float_to_uint16(cmd.iq_cmd, -I_MAX, I_MAX);
                memset(data, 0, sizeof(data));
                data[1] = CMD_CURRENT;
                data[2] = iq_int >> 8;
                data[3] = iq_int & 0xFF;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_CURRENT;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                break;
            }

            case CMD_TORQUE: {
                int tau_int = float_to_uint16(cmd.torque_cmd, -TAU_MAX, TAU_MAX);
                int kp_int  = float_to_uint8(cmd.kp, 0.0f, KP_MAX / GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0f, 0.1f / GAIN);
                memset(data, 0, sizeof(data));
                data[1] = CMD_TORQUE;
                data[2] = tau_int >> 8;
                data[3] = tau_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(CAN_ID, data, 8);
                motor_data.cmd_id = CMD_TORQUE;
                if (receiveCANFrame(&motor_data) == 0)
                    FOC_motor_t_publish(lcm, LCM_CHAN_GUI, &motor_data);
                break;
            }

            default:
                break;
        }

        /* Manual logging — runs every tick when active, except CMD_COULOMB_FRICTION
         * which does its own inline push to avoid a redundant mcc_read_torque() call.
         * Reads MCC torque and computes derived quantities independently of the
         * per-mode reads above so all control modes are covered. */
        if (cmd.log_active && cmd.state != CMD_COULOMB_FRICTION) {
            float torque_dyno = cmd.sim_mode
                                ? SIM_Kt * sim_state.iq
                                : mcc_read_torque();
            float elec_power  = motor_data.iq * motor_data.vq;
            float mech_power  = torque_dyno * motor_data.velocity;
            float efficiency  = (elec_power != 0.0f) ? mech_power / elec_power : 0.0f;
            clock_gettime(MY_CLOCK_TYPE, &current_time_val);
            current_realtime = realtime(&current_time_val);
            log_push(&(log_entry_t){
                .log_type    = LOG_TYPE_MANUAL,
                .time_s      = current_realtime - start_realtime,
                .vq_cmd      = cmd.vq_cmd,
                .vq_msr      = motor_data.vq,
                .iq          = motor_data.iq,
                .torque_dyno = torque_dyno,
                .velocity    = motor_data.velocity,
                .position    = motor_data.position,
                .elec_power  = elec_power,
                .mech_power  = mech_power,
                .efficiency  = efficiency
            });
        }
    }

    pthread_exit((void *)0);
}

/* ---- Timing utilities ---- */
double getTimeMsec(void)
{
    struct timespec event_ts = {0, 0};
    clock_gettime(MY_CLOCK_TYPE, &event_ts);
    return ((event_ts.tv_sec) * 1000.0) + ((event_ts.tv_nsec) / 1000000.0);
}

double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec) / 1000000000.0));
}

void print_scheduler(void)
{
    int schedType = sched_getscheduler(getpid());
    switch (schedType) {
        case SCHED_FIFO:
            printf("Pthread Policy is SCHED_FIFO\n");
            break;
        case SCHED_OTHER:
            printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
        case SCHED_RR:
            printf("Pthread Policy is SCHED_RR\n"); exit(-1);
        default:
            printf("Pthread Policy is UNKNOWN\n"); exit(-1);
    }
}
