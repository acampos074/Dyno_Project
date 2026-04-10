#ifndef dyno_h
#define dyno_h

#include <stdint.h>
#include <stdlib.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

//#include <lcm/lcm.h>
//#include "/home/campo074/Documents/ImGuiApps/MyApplication/FOC_GUI_V3/lib/lcm/FOC_motor_t.h"

// ========== SEQUENCER DEFINES ========
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (12)
#define TRUE (1)
#define FALSE (0)
#define NUM_THREADS (1)
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW

// ========= CANBUS DEFINES =========
/*Special address description flags for CAN_ID*/
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_INTERFACE_1 "vcan0"
#define CAN_INTERFACE_2 "vcan1"

// ========= FLOAT2INT DEFINES =========
/* Constants needed to convert float to int */
#define I_MAX 40.0f // 40 amps max
#define V_MAX 24.0f // 24 volts max
#define TAU_MAX 2.0f // 2 Nm max
//#define KT 0.037352f // torque constant (Nm/Amps)
#define KT 0.0217f // 0.023f
#define GR 12.0f // gear ratio
#define ONE_REV 6.2831f // 2*pi radians
#define SPEED_MAX 300.0f // 300 rad/sec

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
    int total_test_counter; // counter for dyno test waveform
    double Fs; // sampling frequency (Hz)
    double h; // sampling period (seconds)
    double Vmax; // max test voltage (V)
    int num_cycles; // number of cycles (ramp+measure)
    int cycle_counter; // counter for ramp+measure cycles
    double ramp_time; // ramp time between voltage steps (sec)
    double meas_time; // time used for data collection at fixed voltage (sec)
    int ramp_step; // ramp step size
    double dv; // voltage step size during ramp (V)
    double ramp_down_time; // time to ramp down voltage (sec)
    double T; // total dyno test time (sec)
    int N; // total number of time steps
    int N_ramp_down;
}dyno_t;


#endif