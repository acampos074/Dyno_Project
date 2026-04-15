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

#include "sequencer.h"
#include "can_driver.h"
#include "mcc_driver.h"
#include "logger.h"
#include "lcm_interface.h"
#include "encoding.h"
#include "dyno.h"

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
    itime.it_interval.tv_nsec = 10000000; /* 10 ms → 100 Hz sequencer */
    itime.it_value.tv_sec     = 0;
    itime.it_value.tv_nsec    = 10000000;
    timer_settime(timer_1, flags, &itime, &last_itime);
}

/* ---- Sequencer (SIGALRM handler) ---- */
void Sequencer(int id)
{
    int flags = 0;

    seqCnt++;
    if (seqCnt >= 1000)
        seqCnt = 1;

    /* Release Service_1 at half the sequencer rate → 50 Hz */
    if ((seqCnt % 2) == 0)
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

/* ---- Service_1 (50 Hz RT thread) ---- */
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
                clock_gettime(MY_CLOCK_TYPE, &current_time_val);
                current_realtime = realtime(&current_time_val);
                log_push(&(log_entry_t){
                    .log_type    = LOG_TYPE_FOC,
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

        /* Manual logging — runs every tick when active, regardless of control mode.
         * Reads MCC torque and computes derived quantities independently of the
         * per-mode reads above so all control modes are covered. */
        if (cmd.log_active) {
            float torque_dyno = mcc_read_torque();
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
