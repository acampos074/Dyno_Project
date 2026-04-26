"""
foc_torque_speed.py

Generates a torque-speed envelope for the BLDC motor using a FOC simulation.
Algorithm mirrors FOC_Refactored/FOC/Core/Src/FOC.c:
  - Separate PI loops for Id (= 0) and Iq
  - Elliptic voltage saturation  (two-stage, matching commutate())
  - Integrator anti-windup clamped to v_max
  - Speed swept on a locked-rotor (dynamometer) basis at 1 kHz
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal

# ── Motor parameters ──────────────────────────────────────────────────────────
J   = 4.0102e-7  # kg·m²   (spin-down test)
B   = 8.4e-5     # N·m·s/rad
Km  = 0.025      # N·m/A   (torque const = back-EMF const, Ke = Km)
R   = 0.362      # Ω       (phase resistance)
L   = 567e-6     # H       (Ld = Lq)

# ── Drive parameters  (FOC.h) ─────────────────────────────────────────────────
V_BUS   = 24.0            # V   bus voltage
OVERMOD = 1.15            # overmodulation factor
DC_MAX  = 0.94            # max SVM duty cycle
SQRT1_3 = 0.57735026919   # 1/√3
I_MAX   = 40.0            # A   phase current limit

# Maximum DQ voltage (SVM limit, matches FOC.c line 148)
v_max = OVERMOD * V_BUS * DC_MAX * SQRT1_3   # ≈ 14.98 V

# PI current loop gains  (pqGain / pdGain / iqGain / idGain from FOC.h)
Kp = 0.27005
Ki = 0.021057

# ── Simulation settings ───────────────────────────────────────────────────────
fs     = 30000.0  # Hz  (30 kHz, matches FOC PWM frequency)
dt     = 1.0 / fs
T_elec = 0.05     # s   settling window per point  (>> L/R ≈ 1.6 ms)

# ── Continuous state-space (linearised at ω=0) ───────────────────────────────
# States: [id, iq, ω],  inputs: [Vd, Vq]
Ac = np.array([[-R/L,    0,      0    ],
               [  0,   -R/L,  -Km/L  ],
               [  0,   Km/J,  -B/J   ]])
Bc = np.array([[1/L,  0  ],
               [ 0,   1/L],
               [ 0,   0  ]])

Ad, Bd, _, _, _ = scipy.signal.cont2discrete(
    (Ac, Bc, np.eye(3), np.zeros((3, 2))), dt, method="zoh"
)

print(f"v_max  = {v_max:.3f} V")
print(f"ω_nl   = {v_max/Km:.1f} rad/s  ({v_max/Km*60/(2*np.pi):.0f} RPM)  (no-load estimate)")
print(f"τ_peak = {Km*I_MAX:.4f} N·m  (Km · I_MAX)")
print(f"\nDiscrete Ad (ZOH, dt={dt*1e3:.1f} ms):\n{Ad}")
print(f"\nDiscrete Bd:\n{Bd}")

# ── FOC simulation: electrical dynamics at fixed speed ────────────────────────
def foc_steady_state(omega, iq_ref):
    """
    Lock rotor at speed `omega`, run FOC (Id=0 / Iq=iq_ref) until electrical
    steady state.  Returns (torque, iq_ss, id_ss).

    Implements the PI + elliptic saturation logic from FOC.c commutate().
    """
    N      = int(T_elec / dt)
    id_    = 0.0
    iq_    = 0.0
    sum_id = 0.0
    sum_iq = 0.0

    for _ in range(N):
        # ── PI controllers ────────────────────────────────────────────────────
        id_err = 0.0 - id_
        iq_err = iq_ref - iq_

        vd_cmd = Kp * id_err + sum_id
        vq_cmd = Kp * iq_err + sum_iq

        # Stage 1: elliptic saturation (FOC.c lines 193-195)
        vd_cmd = np.clip(vd_cmd, -v_max, v_max)
        vq_lim = np.sqrt(max(0.0, v_max**2 - vd_cmd**2))
        vq_cmd = np.clip(vq_cmd, -vq_lim, vq_lim)

        # Stage 2: vector magnitude clamp (FOC.c lines 216-221)
        mag = np.hypot(vd_cmd, vq_cmd)
        if mag > v_max:
            vd_cmd *= v_max / mag
            vq_cmd *= v_max / mag

        # Integrator update with anti-windup
        sum_id = np.clip(sum_id + Kp * Ki * id_err, -v_max, v_max)
        sum_iq = np.clip(sum_iq + Kp * Ki * iq_err, -v_max, v_max)

        # ── Electrical dynamics (ω fixed, forward Euler) ──────────────────────
        did = (-R * id_ + omega * L * iq_ + vd_cmd) / L
        diq = (-R * iq_ - omega * L * id_ - Km * omega + vq_cmd) / L
        id_ += dt * did
        iq_ += dt * diq

    return Km * iq_, iq_, id_

# ── Speed sweep  (dynamometer-style) ─────────────────────────────────────────
omega_nl  = v_max / Km                             # theoretical no-load speed
omegas    = np.linspace(0, omega_nl * 0.99, 400)

iq_refs   = [I_MAX, I_MAX * 0.5, I_MAX * 0.25]   # three Iq setpoints
labels    = [f"Iq_ref = {I_MAX:.0f} A (max)",
             f"Iq_ref = {I_MAX*0.5:.0f} A",
             f"Iq_ref = {I_MAX*0.25:.0f} A"]
colors    = ["steelblue", "darkorange", "seagreen"]

results = []
for iq_ref in iq_refs:
    torques = []
    for w in omegas:
        tau, _, _ = foc_steady_state(w, iq_ref)
        torques.append(max(0.0, tau))
    results.append(np.array(torques))

# ── Plots ──────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

for torques, label, color in zip(results, labels, colors):
    ax1.plot(omegas, torques, label=label, color=color, linewidth=2)
    ax2.plot(omegas, torques * omegas, label=label, color=color, linewidth=2)

# Annotate current and voltage limits on torque plot
ax1.axhline(Km * I_MAX,      color="gray", linestyle="--", linewidth=1,
            label=f"Current limit  τ = {Km*I_MAX:.3f} N·m")

ax1.set_ylabel("Torque (N·m)")
ax1.set_title(f"BLDC Torque-Speed Envelope — FOC, Vbus = {V_BUS} V, Id = 0")
ax1.legend(fontsize=9)
ax1.grid(True)

ax2.set_ylabel("Power (W)")
ax2.set_xlabel("Speed (rad/s)")
ax2.legend(fontsize=9)
ax2.grid(True)

# Secondary x-axis in RPM
ax_rpm = ax2.twiny()
ax_rpm.set_xlim(np.array(ax2.get_xlim()) * 60 / (2 * np.pi))
ax_rpm.set_xlabel("Speed (RPM)")

plt.tight_layout()
plt.savefig("torque_speed.png", dpi=150)
plt.show()
