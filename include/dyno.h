#ifndef DYNO_H
#define DYNO_H

#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

// ========== SEQUENCER DEFINES ========
#define USEC_PER_MSEC       (1000)
#define NANOSEC_PER_MSEC    (1000000)
#define NANOSEC_PER_SEC     (1000000000)
#define NUM_CPU_CORES       (12)
#define TRUE                (1)
#define FALSE               (0)
#define NUM_THREADS         (1)
#define MY_CLOCK_TYPE       CLOCK_MONOTONIC  // was CLOCK_MONOTONIC_RAW; using MONOTONIC for timer compat

// ========= CANBUS DEFINES =========
#define CAN_EFF_FLAG        0x80000000U
#define CAN_RTR_FLAG        0x40000000U
#define CAN_ERR_FLAG        0x20000000U
#define CAN_INTERFACE       "can0"
#define CAN_ID              0x123
#define CAN_RECV_TIMEOUT_MS 5       // max ms to wait for a CAN reply

// ========= MOTOR CONSTANTS =========
#define I_MAX       40.0f   // 40 amps max
#define V_MAX       24.0f   // 24 volts max
#define TAU_MAX     2.0f    // 2 Nm max
#define KT          0.0217f // torque constant (Nm/A)
#define GR          12.0f   // gear ratio
#define ONE_REV     6.2831f // 2*pi radians
#define SPEED_MAX   300.0f  // rad/s
#define GAIN        1.0f    // command scaling factor
#define KP_MAX      0.2f    // max proportional gain

// ========= STATE MACHINE COMMAND IDs =========
// Shared between GUI (main.cpp) and sequencer (seqgen.c).
// Use these names instead of bare integers everywhere.
typedef enum {
    CMD_NONE         = 0,
    CMD_CALIBRATE    = 1,
    CMD_OFF          = 2,
    CMD_VOLTAGE_FOC  = 3,
    CMD_POSITION     = 4,
    CMD_VELOCITY     = 5,
    CMD_DYNO_TEST    = 6,
    CMD_TOGGLE_LED   = 7,
    CMD_CURRENT      = 8,
    CMD_TORQUE       = 9,
    CMD_LOG_TOGGLE   = 10   /* start / stop manual CSV logging */
} MotorCmd;

// ========== DYNO STRUCTURE ========
typedef struct {
    int state;
    int led_flag;
    double pos_cmd;
    double vel_cmd;
    double vq_cmd;
    double iq_cmd;
    double torque_cmd;
    double kp;
    double kd;
    int vq_int;
    int iq_int;
    int pos_cmd_int;
    int vel_cmd_int;
    int torque_cmd_int;
    int kp_int;
    int kd_int;
    double sample_waveform;
    float torque_dyno;
    float elec_power;
    float mech_power;
    float efficiency;
    int log_active;
    int total_test_counter;
    double Fs;
    double h;
    double Vmax;
    int num_cycles;
    int cycle_counter;
    double ramp_time;
    double meas_time;
    int ramp_step;
    double dv;
    double ramp_down_time;
    double T;
    int N;
    int N_ramp_down;
} dyno_t;

#endif /* DYNO_H */
