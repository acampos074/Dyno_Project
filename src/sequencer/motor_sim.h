#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#include "dyno.h"  /* MotorCmd enum, V_MAX, SIM_* constants */

/* =====================================================================
 * motor_sim — RK4 virtual brushless motor
 *
 * Integrates the d-q frame equations of motion at each 50 Hz tick:
 *
 *   Electrical:   L · dIq/dt  = Vq − R·Iq − Ke·ω
 *   Mechanical:   J · dω/dt   = Kt·Iq − B·ω
 *   Kinematic:    dθ/dt       = ω
 *
 * Current, torque, and PD modes bypass the electrical equation and
 * assume ideal inner-loop control so the mechanical response is the
 * dominant observable in those modes.
 *
 * Usage (Service_1):
 *   1. Call motor_sim_init() once at startup (seqgen_main.c).
 *   2. Each tick, fill a motor_sim_input_t from the dyno snapshot.
 *   3. Call motor_sim_step() — it writes sim_state and returns Vq.
 *   4. Copy sim_state fields into motor_data for LCM publish / logging.
 * ===================================================================== */

/* ---- simulation state (written each tick by motor_sim_step) ---- */
typedef struct {
    float iq;         /* q-axis current      (A)     */
    float omega;      /* angular velocity    (rad/s) */
    float theta;      /* angular position    (rad)   */
    float omega_prev; /* previous ω — used for D-term in velocity control */
} motor_sim_state_t;

/* ---- physical parameters — set once in motor_sim_init() ---- */
typedef struct {
    float R;   /* winding resistance      (Ω)        */
    float L;   /* phase inductance        (H)        */
    float Kt;  /* torque constant         (Nm/A)     */
    float Ke;  /* back-EMF constant       (V·s/rad)  */
    float B;   /* viscous damping         (Nm·s/rad) */
    float J;   /* rotor inertia           (kg·m²)    */
} motor_sim_params_t;

/* ---- per-tick input — fill only fields relevant to cmd_id ---- */
typedef struct {
    float vq_cmd;     /* CMD_VOLTAGE_FOC: open-loop voltage (V)   */
    float iq_cmd;     /* CMD_CURRENT:     current setpoint  (A)   */
    float torque_cmd; /* CMD_TORQUE:      torque setpoint   (Nm)  */
    float vel_cmd;    /* CMD_VELOCITY:    velocity setpoint (rad/s)*/
    float pos_cmd;    /* CMD_POSITION:    position setpoint (rad) */
    float kp;         /* PD proportional gain                     */
    float kd;         /* PD derivative gain                       */
} motor_sim_input_t;

/* Globals — written by motor_sim_step(), read by Service_1 */
extern motor_sim_state_t  sim_state;
extern motor_sim_params_t sim_params;

/*
 * Reset sim_state to zero and load default parameters from SIM_* defines.
 * Call once before the Service_1 thread starts.
 */
void motor_sim_init(void);

/*
 * Advance the simulation by one timestep h.
 *
 *   cmd_id — the active MotorCmd; selects which dynamics to integrate.
 *   in     — pointer to the filled motor_sim_input_t.
 *   h      — timestep in seconds (use SIM_H = 1/50 s).
 *
 * Returns the Vq actually applied to the windings.  Assign this to
 * motor_data.vq so the GUI plots the correct voltage signal.
 */
float motor_sim_step(int cmd_id, const motor_sim_input_t *in, float h);

#endif /* MOTOR_SIM_H */
