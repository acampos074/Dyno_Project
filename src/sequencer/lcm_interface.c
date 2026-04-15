#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "lcm_interface.h"
#include "logger.h"   /* log_set_dyno_file */

/* ---- Definitions of shared state declared extern in lcm_interface.h ---- */
lcm_t          *lcm        = NULL;
lcm_t          *lcm2       = NULL;
FOC_motor_t     motor_data;
dyno_t          dyno;
pthread_mutex_t dyno_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Forward declaration — handler is file-local */
static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                        const FOC_motor_t *msg, void *user);

/* ---- dyno_init ---- */
void dyno_init(void)
{
    dyno = (dyno_t){
        .state             = 0,
        .led_flag          = 0,
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
        .Fs                = 50.0,
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
    /* Open dyno_test file outside the lock — file I/O must not hold dyno_mutex */
    FILE *new_dyno_test = NULL;
    if ((MotorCmd)msg->cmd_id == CMD_DYNO_TEST) {
        new_dyno_test = fopen("dyno_test.csv", "w");
        if (!new_dyno_test) {
            printf("my_handler: could not open dyno_test.csv\n");
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
        default:
            break;
    }
    pthread_mutex_unlock(&dyno_mutex);
}
