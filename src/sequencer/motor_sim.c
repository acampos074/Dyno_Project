#include <string.h>
#include "motor_sim.h"
#include "dyno.h"

/* ---- global state ------------------------------------------------ */
motor_sim_state_t  sim_state;
motor_sim_params_t sim_params;

/*
 * Maximum sub-step size for rk4_vq.
 *
 * The electrical time constant is τ_e = L/R.  For this motor
 * τ_e ≈ 1.57 ms, so the RK4 stability limit is h < 2.8·τ_e ≈ 4.4 ms.
 * The outer tick is 20 ms — well outside the stability region — so
 * rk4_vq internally slices h into sub-steps of at most 1 ms.
 *
 * rk4_mech has no electrical equation; its characteristic root is
 * −B/J ≈ −0.35 s⁻¹, so it is unconditionally stable at h = 20 ms.
 */
#define RK4_MAX_H  0.001f   /* 1 ms — keeps |z| = (R/L)·h_sub ≈ 0.64 < 2.8 */

/* ---- init --------------------------------------------------------- */
void motor_sim_init(void)
{
    memset(&sim_state, 0, sizeof(sim_state));
    sim_params = (motor_sim_params_t){
        .R  = SIM_R,
        .L  = SIM_L,
        .Kt = SIM_Kt,
        .Ke = SIM_Ke,
        .B  = SIM_B,
        .J  = SIM_J
    };
}

/* ===================================================================
 * Internal helpers
 * =================================================================== */

/* Three-component state vector used inside the RK4 loops */
typedef struct { float iq, omega, theta; } sv_t;

/* Continuous-time derivatives: full electrical + mechanical model */
static sv_t sv_f(sv_t s, float Vq)
{
    sv_t d;
    d.iq    = (Vq - sim_params.R * s.iq - sim_params.Ke * s.omega) / sim_params.L;
    d.omega = (sim_params.Kt * s.iq - sim_params.B * s.omega) / sim_params.J;
    d.theta = s.omega;
    return d;
}

/* a + scale * b */
static sv_t sv_axpy(sv_t a, sv_t b, float scale)
{
    return (sv_t){
        a.iq    + scale * b.iq,
        a.omega + scale * b.omega,
        a.theta + scale * b.theta
    };
}

/*
 * Full RK4 step — integrates electrical (Iq) + mechanical (ω, θ).
 * Use for: CMD_VOLTAGE_FOC, CMD_VELOCITY (Vq from PD), CMD_POSITION (Vq from PD).
 */
static void rk4_vq(float Vq, float h)
{
    int   n     = (int)(h / RK4_MAX_H);
    if (n < 1) n = 1;
    float h_sub = h / (float)n;

    for (int i = 0; i < n; i++) {
        sv_t x  = { sim_state.iq, sim_state.omega, sim_state.theta };
        sv_t k1 = sv_f(x,                          Vq);
        sv_t k2 = sv_f(sv_axpy(x, k1, h_sub*0.5f), Vq);
        sv_t k3 = sv_f(sv_axpy(x, k2, h_sub*0.5f), Vq);
        sv_t k4 = sv_f(sv_axpy(x, k3, h_sub),      Vq);

        sim_state.iq    = x.iq    + (h_sub/6.0f)*(k1.iq    + 2*k2.iq    + 2*k3.iq    + k4.iq);
        sim_state.omega = x.omega + (h_sub/6.0f)*(k1.omega + 2*k2.omega + 2*k3.omega + k4.omega);
        sim_state.theta = x.theta + (h_sub/6.0f)*(k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);
    }
}

/*
 * Mechanical-only RK4 — ideal current forcing, electrical dynamics bypassed.
 * Use for: CMD_CURRENT, CMD_TORQUE.
 * Iq is set directly; only ω and θ are integrated.
 *
 *   dω/dt = (Kt·Iq − B·ω) / J
 *   dθ/dt = ω
 */
static void rk4_mech(float Iq, float h)
{
    float w0 = sim_state.omega;
    float t0 = sim_state.theta;

    /* RK4 on ω */
    float dw1 = (sim_params.Kt*Iq - sim_params.B*(w0              )) / sim_params.J;
    float dw2 = (sim_params.Kt*Iq - sim_params.B*(w0 + dw1*h*0.5f )) / sim_params.J;
    float dw3 = (sim_params.Kt*Iq - sim_params.B*(w0 + dw2*h*0.5f )) / sim_params.J;
    float dw4 = (sim_params.Kt*Iq - sim_params.B*(w0 + dw3*h      )) / sim_params.J;

    /* RK4 on θ using the same intermediate ω values */
    float dt1 = w0;
    float dt2 = w0 + dw1*h*0.5f;
    float dt3 = w0 + dw2*h*0.5f;
    float dt4 = w0 + dw3*h;

    sim_state.iq    = Iq;
    sim_state.omega = w0 + (h/6.0f)*(dw1 + 2*dw2 + 2*dw3 + dw4);
    sim_state.theta = t0 + (h/6.0f)*(dt1 + 2*dt2 + 2*dt3 + dt4);
}

/* ===================================================================
 * Public API
 * =================================================================== */
float motor_sim_step(int cmd_id, const motor_sim_input_t *in, float h)
{
    float Vq = 0.0f;

    /* Save previous ω for velocity D-term before any integration */
    sim_state.omega_prev = sim_state.omega;

    switch (cmd_id) {

        /* ----------------------------------------------------------
         * Open-loop voltage: Vq directly commanded, full model.
         * ---------------------------------------------------------- */
        case CMD_VOLTAGE_FOC:
            rk4_vq(in->vq_cmd, h);
            return in->vq_cmd;

        /* ----------------------------------------------------------
         * Ideal current control: bypass electrical dynamics.
         * Back-compute Vq = R·Iq + Ke·ω for consistent logging.
         * ---------------------------------------------------------- */
        case CMD_CURRENT:
            rk4_mech(in->iq_cmd, h);
            return sim_params.R * sim_state.iq + sim_params.Ke * sim_state.omega;

        /* ----------------------------------------------------------
         * Ideal torque control: convert torque → equivalent Iq.
         * ---------------------------------------------------------- */
        case CMD_TORQUE: {
            float iq_eq = (sim_params.Kt > 0.0f)
                          ? in->torque_cmd / sim_params.Kt : 0.0f;
            rk4_mech(iq_eq, h);
            return sim_params.R * sim_state.iq + sim_params.Ke * sim_state.omega;
        }

        /* ----------------------------------------------------------
         * PD velocity control: mirrors motor firmware.
         *   Vq = kp·(ωcmd − ω) + kd·(−dω/dt)
         * dω/dt is approximated with the back-difference from the
         * previous tick (omega_prev was saved above).
         * ---------------------------------------------------------- */
        case CMD_VELOCITY: {
            float vel_err  = in->vel_cmd - sim_state.omega;
            float omega_dot = (sim_state.omega - sim_state.omega_prev) / h;
            Vq = in->kp * vel_err - in->kd * omega_dot;
            if (Vq >  V_MAX) Vq =  V_MAX;
            if (Vq < -V_MAX) Vq = -V_MAX;
            rk4_vq(Vq, h);
            return Vq;
        }

        /* ----------------------------------------------------------
         * PD position control: mirrors motor firmware.
         *   Vq = kp·(θcmd − θ) − kd·ω
         * ---------------------------------------------------------- */
        case CMD_POSITION: {
            Vq = in->kp * (in->pos_cmd - sim_state.theta) - in->kd * sim_state.omega;
            if (Vq >  V_MAX) Vq =  V_MAX;
            if (Vq < -V_MAX) Vq = -V_MAX;
            rk4_vq(Vq, h);
            return Vq;
        }

        /* ----------------------------------------------------------
         * All other states (OFF, NONE, CALIBRATE, …): coast to stop.
         * ---------------------------------------------------------- */
        default:
            rk4_vq(0.0f, h);
            return 0.0f;
    }
}
