"""
efficiency_map.py

Test protocol: dynamometer-style sweep over a (speed, torque) grid.
For each operating point the motor is speed-locked (dyno holds ω),
FOC commands Id=0 / Iq=τ/Km, and steady-state powers are recorded.

Losses modelled:
  - Copper losses : P_cu  = R · Iq²
  - Friction      : P_fric = B · ω²
  (iron/eddy losses omitted — no core-loss parameters available)

Efficiency: η = P_shaft / P_elec
  P_shaft = τ · ω − P_fric     (net output at shaft)
  P_elec  = (R·Iq + Km·ω) · Iq  (DQ input, Id=0 ⟹ Vd·Id=0)
"""

import matplotlib.pyplot as plt
import numpy as np

# ── Motor parameters ──────────────────────────────────────────────────────────
B   = 8.4e-5     # N·m·s/rad  (viscous damping)
Km  = 0.025      # N·m/A      (torque const = back-EMF const)
R   = 0.362      # Ω
L   = 567e-6     # H  (Ld = Lq)

# ── Drive parameters (FOC.h) ──────────────────────────────────────────────────
V_BUS   = 24.0
OVERMOD = 1.15
DC_MAX  = 0.94
SQRT1_3 = 0.57735026919
v_max   = OVERMOD * V_BUS * DC_MAX * SQRT1_3   # ≈ 14.98 V
I_MAX   = 40.0                                  # A

# ── Operating envelope ────────────────────────────────────────────────────────
omega_max = v_max / Km          # no-load speed  [rad/s]
tau_max   = Km * I_MAX          # peak torque    [N·m]

# ── Voltage-limit: max Iq at each speed (quadratic solve, Id=0) ───────────────
# (ω·L·Iq)² + (R·Iq + Km·ω)² = v_max²
# → (R²+ω²L²)·Iq² + 2R·Km·ω·Iq + (Km·ω)²−v_max² = 0
def iq_voltage_limit(omega):
    a    = R**2 + (omega * L)**2
    b    = 2.0 * R * Km * omega
    c    = (Km * omega)**2 - v_max**2
    disc = b**2 - 4.0 * a * c
    if disc < 0:
        return 0.0
    return (-b + np.sqrt(disc)) / (2.0 * a)

# ── Test grid ─────────────────────────────────────────────────────────────────
N_speed  = 300
N_torque = 300

omegas = np.linspace(0, omega_max,  N_speed  + 1)[1:]   # exclude 0
taus   = np.linspace(0, tau_max,    N_torque + 1)[1:]   # exclude 0

# Precompute envelope limit at each speed point
iq_lim   = np.array([min(I_MAX, iq_voltage_limit(w)) for w in omegas])
tau_lim  = Km * iq_lim

# ── Sweep: record efficiency at every (ω, τ) grid point ──────────────────────
ETA      = np.full((N_torque, N_speed), np.nan)
P_SHAFT  = np.full((N_torque, N_speed), np.nan)

print(f"Sweeping {N_speed} speed × {N_torque} torque points ...")

for j, (omega, t_lim) in enumerate(zip(omegas, tau_lim)):
    P_fric = B * omega**2           # friction power at this speed

    for i, tau in enumerate(taus):
        if tau > t_lim:
            break                   # outside voltage / current envelope

        iq      = tau / Km
        P_shaft = tau * omega - P_fric
        P_elec  = (R * iq + Km * omega) * iq

        if P_shaft <= 0 or P_elec <= 0:
            continue                # friction-dominated — skip

        ETA[i, j]     = min(100.0, P_shaft / P_elec * 100.0)
        P_SHAFT[i, j] = P_shaft

print("Done.")

# Peak efficiency operating point
peak_idx = np.unravel_index(np.nanargmax(ETA), ETA.shape)
omega_peak = omegas[peak_idx[1]]
tau_peak   = taus[peak_idx[0]]
eta_peak   = ETA[peak_idx]
print(f"\nPeak efficiency : {eta_peak:.2f}%")
print(f"  at ω = {omega_peak:.1f} rad/s  ({omega_peak*60/(2*np.pi):.0f} RPM)")
print(f"  at τ = {tau_peak:.4f} N·m")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 8))

levels = np.arange(10, 101, 5)
cf = ax.contourf(omegas, taus, ETA,
                 levels=levels, cmap="RdYlGn",
                 extend="min", vmin=10, vmax=100)
cl = ax.contour(omegas, taus, ETA,
                levels=levels, colors="black",
                linewidths=0.4, alpha=0.5)
ax.clabel(cl, fmt="%d%%", fontsize=7, inline=True)

# Constant power curves
for P_kw in [0.1, 0.2, 0.5, 1.0, 2.0, 5.0]:
    P_w = P_kw * 1000 if P_kw >= 1.0 else P_kw * 1000
    tau_pwr = np.where(omegas > 0, P_w / omegas, np.inf)
    mask = (tau_pwr <= tau_lim) & (tau_pwr <= tau_max) & (tau_pwr > 0)
    if mask.any():
        label = f"{P_kw:.1f} W" if P_kw < 1.0 else f"{P_kw:.0f} W" if P_kw < 1000 else f"{P_kw/1000:.0f} kW"
        ax.plot(omegas[mask], tau_pwr[mask],
                "b--", linewidth=0.8, alpha=0.6,
                label=label if P_kw == 0.1 else "_")
        ax.text(omegas[mask][-1], tau_pwr[mask][-1],
                f" {P_w:.0f} W", fontsize=7, color="blue", va="center")

# Operating envelope boundary
ax.plot(omegas, tau_lim, "k-", linewidth=2, label="Envelope limit")
ax.fill_between(omegas, tau_lim, tau_max, color="white", alpha=1.0, zorder=2)

# Peak efficiency marker
ax.plot(omega_peak, tau_peak, "k*", markersize=12, zorder=5,
        label=f"Peak η = {eta_peak:.1f}%")

cbar = fig.colorbar(cf, ax=ax, label="Efficiency (%)")

ax.set_xlim(0, omega_max)
ax.set_ylim(0, tau_max)
ax.set_xlabel("Speed (rad/s)")
ax.set_ylabel("Torque (N·m)")
ax.set_title(f"BLDC Efficiency Map — FOC, Vbus = {V_BUS} V, Id = 0")
ax.legend(loc="upper right", fontsize=8)
ax.grid(True, alpha=0.2)

# Secondary RPM axis
ax_rpm = ax.twiny()
ax_rpm.set_xlim(np.array(ax.get_xlim()) * 60.0 / (2.0 * np.pi))
ax_rpm.set_xlabel("Speed (RPM)")

plt.tight_layout()
plt.savefig("efficiency_map.png", dpi=150)
plt.show()
