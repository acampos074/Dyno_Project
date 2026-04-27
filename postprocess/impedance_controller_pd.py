#!/usr/bin/env python3
"""
Discrete PD impedance torque controller design.
Mechanical plant:  J*θ'' + B*θ' = τ
  J = 4.0102e-7 kg.m²,  B = 8.4e-5 Nm.s/rad,  fs = 1 kHz,  BW = 10 Hz.

Firmware structure:
  τ[k] = Kp*(θd[k] - θ[k]) + Kd*(ωd[k] - ω[k])

Since both θ and ω are measured directly (not ω from numerical diff),
the correct discrete loop gain is:
  L_d(z) = Kp*G_pos_d(z) + Kd*G_vel_d(z)
where G_pos_d and G_vel_d are ZOH-discretised separately.

Design via pole placement (ζ = 1/√2, Butterworth):
  Kp = J·ωn²
  Kd = 2ζωn·J − B   (negative → reduces natural over-damping; stable iff B+Kd > 0)
"""

import os, subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete

pi = np.pi

# ── Parameters ───────────────────────────────────────────────────────────────
J   = 4.0102e-7    # kg.m²
B   = 8.4e-5       # Nm.s/rad
fs  = 1_000.0      # Hz
Ts  = 1.0 / fs
BW  = 100.0        # Hz — desired closed-loop -3 dB bandwidth

wn   = 2 * pi * BW               # rad/s  (= ωBW for ζ = 1/√2)
zeta = 1.0 / np.sqrt(2)          # 0.7071 — Butterworth / maximally flat

# ── PD gain design (continuous-time pole placement) ──────────────────────────
Kp    = J * wn**2
Kd    = 2.0 * zeta * wn * J - B
B_eff = B + Kd                    # effective damping after control

# ── Print results ─────────────────────────────────────────────────────────────
print("── PD Impedance Torque Controller Design ────────────────────────────────")
print(f"  J = {J:.4e} kg.m²,  B = {B:.2e} Nm.s/rad,  fs = {fs:.0f} Hz")
print(f"  Natural velocity pole:  B/J = {B/J:.2f} rad/s  ({B/J/(2*pi):.2f} Hz)")
print()
print(f"  Design:  BW = {BW} Hz,  ζ = {zeta:.4f},  ωn = {wn:.4f} rad/s")
print()
note = "reduces natural damping — active negative damping" if Kd < 0 else "adds damping"
stab = "stable ✓" if B_eff > 0 else "UNSTABLE ✗"
print(f"  Kp = {Kp:.6e} Nm/rad      (virtual stiffness)")
print(f"  Kd = {Kd:.6e} Nm.s/rad   ({note})")
print(f"  Effective damping:  B_eff = B + Kd = {B_eff:.4e} Nm.s/rad  ({stab})")
print()
print("  Firmware:  τ[k] = Kp*(θd[k] - θ[k]) + Kd*(ωd[k] - ω[k])")
print(f"    Kp = {Kp:.6e}    // Nm/rad")
print(f"    Kd = {Kd:.6e}    // Nm.s/rad")

# ── Discretise both plants using ZOH ─────────────────────────────────────────
# G_pos(s) = 1/(Js²+Bs)  — position response to torque
(num_pos_d, den_pos_d, _) = cont2discrete(([1.0], [J, B, 0.0]), Ts, method='zoh')
num_pos_d = num_pos_d.flatten()
den_pos_d = den_pos_d.flatten()

# G_vel(s) = 1/(Js+B)  — velocity response to torque (direct measurement)
(num_vel_d, den_vel_d, _) = cont2discrete(([1.0], [J, B]),      Ts, method='zoh')
num_vel_d = num_vel_d.flatten()
den_vel_d = den_vel_d.flatten()

# ── TF helpers ────────────────────────────────────────────────────────────────
def eval_tf(num, den, z_vals):
    return np.polyval(num, z_vals) / np.polyval(den, z_vals)

def mag_dB(H):     return 20.0 * np.log10(np.abs(H) + 1e-300)
def phase_deg(H):  return np.degrees(np.unwrap(np.angle(H)))

# ── Frequency grid: 0.1 Hz → Nyquist ─────────────────────────────────────────
freq = np.logspace(-1, np.log10(fs / 2.0), 3000)
w    = 2.0 * pi * freq
z    = np.exp(1j * w * Ts)
s    = 1j * w

# ── Continuous TFs ────────────────────────────────────────────────────────────
G_pos_c = 1.0 / (J*s**2 + B*s)       # position plant
G_vel_c = 1.0 / (J*s   + B)          # velocity plant
L_c     = Kp * G_pos_c + Kd * G_vel_c   # open loop (= (Kp+Kd·s)/(Js²+Bs))
T_c     = Kp * G_pos_c / (1.0 + L_c)   # closed loop (position reference → position)

# ── Discrete TFs (direct velocity measurement — correct loop gain) ─────────────
G_pos_d = eval_tf(num_pos_d, den_pos_d, z)
G_vel_d = eval_tf(num_vel_d, den_vel_d, z)
L_d     = Kp * G_pos_d + Kd * G_vel_d
T_d     = Kp * G_pos_d / (1.0 + L_d)

# ── Stability margins (discrete open loop) ────────────────────────────────────
mag_Ld = mag_dB(L_d)
ph_Ld  = phase_deg(L_d)

gc_idx = np.argmin(np.abs(mag_Ld))
f_gc   = freq[gc_idx]
PM     = 180.0 + ph_Ld[gc_idx]

bw_idx = np.argmin(np.abs(mag_dB(T_d) + 3.0))
f_bw   = freq[bw_idx]

# Discrete closed-loop poles via state-space
a_mech = B / J
b_mech = np.exp(-a_mech * Ts)
Ad = np.array([[1.0,                  (1.0-b_mech)/a_mech],
               [0.0,                   b_mech             ]])
Bd = np.array([[(Ts - (1.0-b_mech)/a_mech) / (J*a_mech)],
               [(1.0-b_mech)          / (J*a_mech)       ]])
K      = np.array([[Kp, Kd]])
A_cl   = Ad - Bd @ K
cl_poles = np.linalg.eigvals(A_cl)
cl_fn  = np.abs(np.angle(cl_poles)) / (2*pi*Ts)   # damped natural freq (Hz)
cl_mag = np.abs(cl_poles)

print(f"\n  Discrete closed-loop poles:  z = {cl_poles[0]:.5f},  {cl_poles[1]:.5f}")
print(f"  Pole magnitude:              |z| = {cl_mag[0]:.5f}  (< 1 → stable)")
print(f"  Pole frequency:              {cl_fn[0]:.2f} Hz")
print(f"\n  Gain crossover (discrete OL):  f_gc = {f_gc:.2f} Hz")
print(f"  Phase margin:                  PM   = {PM:.1f}°")
print(f"  Closed-loop -3 dB (discrete):  f_bw = {f_bw:.2f} Hz  (target {BW} Hz)")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

kc = dict(lw=1.5, ls='--', alpha=0.65)
kd = dict(lw=2.0)

# Magnitude
ax_m.semilogx(freq, mag_dB(G_pos_c), color='steelblue',  **kc, label=r'Plant $G_{pos}(s)$ [cont.]')
ax_m.semilogx(freq, mag_dB(G_pos_d), color='steelblue',  **kd, label=r'Plant $G_{pos,d}(z)$ [disc.]')
ax_m.semilogx(freq, mag_dB(L_c),     color='darkorange', **kc, label=r'Open loop $L(s)$ [cont.]')
ax_m.semilogx(freq, mag_dB(L_d),     color='darkorange', **kd, label=r'Open loop $L_d(z)$ [disc.]')
ax_m.semilogx(freq, mag_dB(T_c),     color='seagreen',   **kc, label=r'Closed loop $T(s)$ [cont.]')
ax_m.semilogx(freq, mag_dB(T_d),     color='seagreen',   **kd, label=r'Closed loop $T_d(z)$ [disc.]')
ax_m.axhline(0,  color='black', lw=0.7, ls=':', alpha=0.6)
ax_m.axhline(-3, color='gray',  lw=0.8, ls=':', label='−3 dB')
ax_m.axvline(BW, color='red',   lw=0.9, ls='--', alpha=0.7, label=f'Target BW = {BW} Hz')
ax_m.set_ylabel('Magnitude (dB)', fontsize=11)
ax_m.set_title(
    f'PD Impedance Torque Controller — Bode Plot  (fs = {fs:.0f} Hz)\n'
    f'Kp = {Kp:.3e} Nm/rad    Kd = {Kd:.3e} Nm·s/rad    '
    f'PM = {PM:.0f}°    BW = {f_bw:.1f} Hz',
    fontsize=10
)
ax_m.legend(fontsize=8, loc='lower left', ncol=2)
ax_m.grid(True, which='both', alpha=0.35)
ax_m.set_ylim(-120, 100)

# Phase
ax_p.semilogx(freq, phase_deg(G_pos_c), color='steelblue',  **kc, label='Plant [cont.]')
ax_p.semilogx(freq, phase_deg(G_pos_d), color='steelblue',  **kd, label='Plant [disc.]')
ax_p.semilogx(freq, phase_deg(L_c),     color='darkorange', **kc, label='Open loop [cont.]')
ax_p.semilogx(freq, phase_deg(L_d),     color='darkorange', **kd, label='Open loop [disc.]')
ax_p.semilogx(freq, phase_deg(T_c),     color='seagreen',   **kc, label='Closed loop [cont.]')
ax_p.semilogx(freq, phase_deg(T_d),     color='seagreen',   **kd, label='Closed loop [disc.]')
ax_p.axhline(-180, color='black', lw=0.7, ls=':', alpha=0.6, label='−180°')
ax_p.axvline(BW,   color='red',   lw=0.9, ls='--', alpha=0.7)
ax_p.set_xlabel('Frequency (Hz)', fontsize=11)
ax_p.set_ylabel('Phase (deg)', fontsize=11)
ax_p.legend(fontsize=8, loc='lower left', ncol=2)
ax_p.grid(True, which='both', alpha=0.35)
ax_p.set_ylim(-270, 30)

ax_m.set_xlim(freq[0], freq[-1])
plt.tight_layout()

script_dir = os.path.dirname(os.path.abspath(__file__))
out_path   = os.path.join(script_dir, 'impedance_controller_bode.png')
plt.savefig(out_path, dpi=150)
print(f'\nSaved: {out_path}')
subprocess.Popen(['eog', out_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
