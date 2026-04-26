#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "lcm_interface.h"
#include "logger.h"
#include "motor_sim.h"

/* Directory where all run-specific CSV files are created.
 * Set by lcm_interface_set_data_dir() before the listener thread starts. */
static char g_data_dir[512] = "";

/* ---- Definitions of shared state declared extern in lcm_interface.h ---- */
lcm_t          *lcm        = NULL;
lcm_t          *lcm2       = NULL;
FOC_motor_t     motor_data;
dyno_t          dyno;
pthread_mutex_t dyno_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Forward declaration — handler is file-local */
static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                        const FOC_motor_t *msg, void *user);

/* ---- lcm_interface_set_data_dir ---- */
void lcm_interface_set_data_dir(const char *dir)
{
    if (dir && *dir)
        strncpy(g_data_dir, dir, sizeof(g_data_dir) - 1);
    else
        g_data_dir[0] = '\0';
}

/* Build a full path under g_data_dir.  buf must be at least PATH_MAX bytes. */
static void make_path(char *buf, size_t buflen, const char *filename)
{
    if (g_data_dir[0])
        snprintf(buf, buflen, "%s/%s", g_data_dir, filename);
    else
        snprintf(buf, buflen, "%s", filename);
}

/* ---- dyno_init ---- */
void dyno_init(void)
{
    dyno = (dyno_t){
        .state             = 0,
        .led_flag          = 0,
        .log_active        = 0,
        .sim_mode          = 0,
        .cal_phase         = 0,
        .cal_tick          = 0,
        .cf_tick           = 0,
        .pos_cmd           = 0.0,
        .vel_cmd           = 0.0,
        .vq_cmd            = 0.0,
        .iq_cmd            = 0.0,
        .torque_cmd        = 0.0,
        .kp                = 0.0,
        .kd                = 0.0,
        .vq_int            = 0,
        .iq_int            = 0,
        .pos_cmd_int       = 0,
        .vel_cmd_int       = 0,
        .torque_cmd_int    = 0,
        .kp_int            = 0,
        .kd_int            = 0,
        .sample_waveform   = 0.0,
        .torque_dyno       = 0.0,
        .elec_power        = 0.0,
        .mech_power        = 0.0,
        .efficiency        = 0.0,
        .total_test_counter= 1,
        .cycle_counter     = 1,
        .Fs                = 400.0,
        .Vmax              = 2,
        .num_cycles        = 10,
        .ramp_time         = 1.0,
        .meas_time         = 6.0,
        .ramp_down_time    = 10.0
    };
    dyno.h           = 1.0 / dyno.Fs;
    dyno.ramp_step   = (int)(dyno.ramp_time   / dyno.h);
    dyno.dv          = dyno.Vmax / dyno.ramp_step / dyno.num_cycles;
    dyno.N_ramp_down = (int)(dyno.ramp_down_time / dyno.h);
    dyno.T           = (dyno.ramp_time + dyno.meas_time) * dyno.num_cycles
                       + dyno.ramp_down_time;
    dyno.N           = (int)(dyno.T / dyno.h);
}

/* ---- LCM init ---- */
int lcm_interface_init(void)
{
    lcm = lcm_create(NULL);
    if (!lcm) {
        printf("lcm_interface_init: lcm_create failed\n");
        return -1;
    }
    lcm2 = lcm_create(NULL);
    if (!lcm2) {
        printf("lcm_interface_init: lcm2_create failed\n");
        return -1;
    }
    FOC_motor_t_subscribe(lcm2, LCM_CHAN_MOTOR, &my_handler, NULL);
    return 0;
}

/* ---- Listener thread ---- */
void *listener(void *unused)
{
    while (1)
        lcm_handle(lcm2);
    return NULL;
}

/* ---- LCM message handler ---- */
static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                        const FOC_motor_t *msg, void *user)
{
    /* Pre-lock file I/O — determine files to open/close before acquiring mutex */
    FILE *new_dyno_test  = NULL;
    FILE *new_manual_log = NULL;
    int   stop_manual_log = 0;

    if ((MotorCmd)msg->cmd_id == CMD_LOG_TOGGLE) {
        /* Read current log_active state to decide what to do */
        pthread_mutex_lock(&dyno_mutex);
        int currently_active = dyno.log_active;
        pthread_mutex_unlock(&dyno_mutex);

        if (!currently_active) {
            /* Start logging: open a uniquely named file inside the run dir */
            time_t now = time(NULL);
            struct tm *tm_info = localtime(&now);
            char basename[64];
            strftime(basename, sizeof(basename), "log_%Y%m%d_%H%M%S.csv", tm_info);
            char path[576];
            make_path(path, sizeof(path), basename);
            new_manual_log = fopen(path, "w");
            if (new_manual_log) {
                fprintf(new_manual_log,
                        "Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),"
                        "Torque (Nm),Speed (rad/s),Pos (rad),"
                        "Elec Power (W),Mech Power (W),Efficiency\n");
                log_set_manual_file(new_manual_log);
                printf("Manual logging started: %s\n", path);
            } else {
                printf("my_handler: could not open %s\n", path);
            }
        } else {
            stop_manual_log = 1;
        }
    }

    if ((MotorCmd)msg->cmd_id == CMD_COULOMB_FRICTION) {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        char basename[64];
        strftime(basename, sizeof(basename), "coulomb_%Y%m%d_%H%M%S.csv", tm_info);
        char path[576];
        make_path(path, sizeof(path), basename);
        new_manual_log = fopen(path, "w");
        if (new_manual_log) {
            fprintf(new_manual_log,
                    "Time (s),Vel CMD(rad/s),Voltage MSR(V),Current (A),"
                    "Torque (Nm),Speed (rad/s),Pos (rad),"
                    "Elec Power (W),Mech Power (W),Efficiency\n");
            log_set_manual_file(new_manual_log);
            printf("Coulomb friction logging started: %s\n", path);
        } else {
            printf("my_handler: could not open %s\n", path);
        }
    }

    if ((MotorCmd)msg->cmd_id == CMD_CAL_MOTOR) {
        char cal_path[576];
        make_path(cal_path, sizeof(cal_path), "calibration.csv");
        FILE *new_cal = fopen(cal_path, "w");
        if (!new_cal) {
            printf("my_handler: could not open %s\n", cal_path);
            return;
        }
        fprintf(new_cal,
                "Time (s),CMD Input,Voltage MSR(V),Current (A),"
                "Torque (Nm),Speed (rad/s),Pos (rad),"
                "Elec Power (W),Mech Power (W),Phase\n");
        log_set_cal_file(new_cal);
    }

    if ((MotorCmd)msg->cmd_id == CMD_DYNO_TEST) {
        char dyno_path[576];
        make_path(dyno_path, sizeof(dyno_path), "dyno_test.csv");
        new_dyno_test = fopen(dyno_path, "w");
        if (!new_dyno_test) {
            printf("my_handler: could not open %s\n", dyno_path);
            return;
        }
        fprintf(new_dyno_test,
                "Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),"
                "Torque (Nm),Speed (rad/s),Pos (rad),"
                "Elec Power (W),Mech Power (W),Efficiency\n");
        log_set_dyno_file(new_dyno_test);
    }

    /* Lock only while writing to dyno — no I/O inside the lock */
    pthread_mutex_lock(&dyno_mutex);
    switch ((MotorCmd)msg->cmd_id) {
        case CMD_CALIBRATE:
            dyno.state = CMD_CALIBRATE;
            break;
        case CMD_OFF:
            dyno.state = CMD_OFF;
            break;
        case CMD_VOLTAGE_FOC:
            dyno.state  = CMD_VOLTAGE_FOC;
            dyno.vq_cmd = msg->vq_cmd;
            break;
        case CMD_POSITION:
            dyno.state   = CMD_POSITION;
            dyno.pos_cmd = msg->position_cmd;
            dyno.kp      = msg->kp;
            dyno.kd      = msg->kd;
            break;
        case CMD_VELOCITY:
            dyno.state   = CMD_VELOCITY;
            dyno.vel_cmd = msg->velocity_cmd;
            dyno.kp      = msg->kp;
            dyno.kd      = msg->kd;
            break;
        case CMD_DYNO_TEST:
            dyno.state  = CMD_DYNO_TEST;
            dyno.vq_cmd = -dyno.dv; /* start from 0 − dv on first tick */
            break;
        case CMD_TOGGLE_LED:
            dyno.led_flag = msg->led;
            dyno.state    = CMD_TOGGLE_LED;
            break;
        case CMD_CURRENT:
            dyno.iq_cmd = msg->iq_cmd;
            dyno.state  = CMD_CURRENT;
            break;
        case CMD_TORQUE:
            dyno.torque_cmd = msg->torque_cmd;
            dyno.state      = CMD_TORQUE;
            dyno.kp         = msg->kp;
            dyno.kd         = msg->kd;
            break;
        case CMD_LOG_TOGGLE:
            if (new_manual_log)
                dyno.log_active = 1;
            else if (stop_manual_log)
                dyno.log_active = 0;
            break;
        case CMD_SIM_TOGGLE:
            dyno.sim_mode = !dyno.sim_mode;
            if (dyno.sim_mode)
                motor_sim_init(); /* reset virtual motor state when entering sim */
            printf("Sim mode: %s\n", dyno.sim_mode ? "VIRTUAL" : "HARDWARE");
            break;
        case CMD_CAL_MOTOR:
            dyno.state     = CMD_CAL_MOTOR;
            dyno.cal_phase = 0;
            dyno.cal_tick  = 0;
            break;
        case CMD_COULOMB_FRICTION:
            dyno.state      = CMD_COULOMB_FRICTION;
            dyno.log_active = (new_manual_log != NULL);
            dyno.cf_tick    = 0;
            break;
        default:
            break;
    }
    pthread_mutex_unlock(&dyno_mutex);

    /* Post-lock: close the manual log file if stopping */
    if (stop_manual_log) {
        log_close_manual_file();
        printf("Manual logging stopped\n");
    }
}
