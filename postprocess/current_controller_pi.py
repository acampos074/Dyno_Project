#!/usr/bin/env python3
"""
Discrete PI current controller design — pole-zero cancellation.
Plant: RL circuit  R = 0.362 Ω, L = 567 µH, fs = 30 kHz, BW = 30 Hz.

Strategy
--------
ZOH-discretised plant:  G_d(z) = K_p / (z - b),  b = exp(-R·Ts/L)
PI controller zero placed at plant pole:  C(z) = Kp·(z - b)/(z - 1)
After cancellation:  L(z) = α/(z - 1),  α = 1 - exp(-2π·BW·Ts)
Kp = α·R / (1 - b),   Ki = Kp·(1 - b) / Ts   [V/(A·s)]
Closed loop: T(z) = α / (z - (1 - α))  — first-order LP at ωBW.
"""

import os
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')          # non-interactive backend — no fontconfig, no GUI
import matplotlib.pyplot as plt

# ── Parameters ──────────────────────────────────────────────────────────────
R  = 0.362       # Ω
L  = 567e-6      # H
fs = 30_000      # Hz
Ts = 1.0 / fs
BW = 30.0        # Hz  desired closed-loop -3 dB bandwidth

# ── Continuous pole → discrete pole (ZOH) ───────────────────────────────────
a       = R / L                      # rad/s
fp_cont = a / (2.0 * np.pi)         # Hz
b       = np.exp(-a * Ts)           # discrete plant pole
K_plant = (1.0 - b) / R            # ZOH gain  (A/V per sample)

# ── Pole-zero cancellation design ────────────────────────────────────────────
alpha   = 1.0 - np.exp(-2.0 * np.pi * BW * Ts)   # integrator coefficient
Kp      = alpha * R / (1.0 - b)                    # V/A
Ki      = Kp * (1.0 - b) / Ts                      # V/(A·s)

# ── Gains in firmware format ──────────────────────────────────────────────────
# u[k]   = pqGain * e[k] + I[k]
# I[k+1] = I[k] + pqGain * iqGain * e[k]
pqGain = Kp               # V/A
iqGain = 1.0 - b          # = Ki*Ts/Kp  (dimensionless)

print("── Discrete PI current controller design ───────────────────────────────")
print(f"  R = {R} Ω,  L = {L*1e6:.0f} µH,  fs = {fs/1e3:.0f} kHz,  BW = {BW} Hz")
print(f"  Continuous pole:  a      = {a:.2f} rad/s  ({fp_cont:.2f} Hz)")
print(f"  Discrete pole:    b      = {b:.7f}")
print(f"  α                      = {alpha:.7f}")
print(f"  Kp                     = {Kp:.6f} V/A")
print(f"  Ki                     = {Ki:.4f}  V/(A·s)")
print()
print("  Firmware gains:")
print(f"    pqGain  = {pqGain:.6f}   // Kp  (V/A)")
print(f"    iqGain  = {iqGain:.6f}   // 1 - b = Ki*Ts/Kp  (dimensionless)")

# ── Transfer functions  (num, den) polynomial arrays in z ────────────────────
# G_d(z) = K_plant / (z - b)
num_G = np.array([K_plant])
den_G = np.array([1.0, -b])

# L(z)   = α       / (z - 1)
num_L = np.array([alpha])
den_L = np.array([1.0, -1.0])

# T(z)   = α       / (z - (1 - α))
num_T = np.array([alpha])
den_T = np.array([1.0, -(1.0 - alpha)])

def eval_tf(num, den, z_vals):
    """Evaluate a z-domain TF at an array of z values."""
    return np.polyval(num, z_vals) / np.polyval(den, z_vals)

# ── Frequency axis: 1 Hz → Nyquist ──────────────────────────────────────────
freq = np.logspace(0, np.log10(fs / 2.0), 3000)
z    = np.exp(1j * 2.0 * np.pi * freq * Ts)

H_G = eval_tf(num_G, den_G, z)
H_L = eval_tf(num_L, den_L, z)
H_T = eval_tf(num_T, den_T, z)

mag_G = 20.0 * np.log10(np.abs(H_G))
mag_L = 20.0 * np.log10(np.abs(H_L))
mag_T = 20.0 * np.log10(np.abs(H_T))

ph_G  = np.degrees(np.unwrap(np.angle(H_G)))
ph_L  = np.degrees(np.unwrap(np.angle(H_L)))
ph_T  = np.degrees(np.unwrap(np.angle(H_T)))

# ── Stability margins ────────────────────────────────────────────────────────
# Gain crossover: |L| = 0 dB
gc_idx    = np.argmin(np.abs(mag_L))
f_gc      = freq[gc_idx]
PM        = 180.0 + ph_L[gc_idx]

# Phase crossover: ∠L = -180°
pc_idx    = np.argmin(np.abs(ph_L + 180.0))
f_pc      = freq[pc_idx]
GM        = -mag_L[pc_idx]

# Closed-loop -3 dB point
bw_idx    = np.argmin(np.abs(mag_T + 3.0))
f_bw_meas = freq[bw_idx]

print(f"\n  Gain crossover:   f_gc  = {f_gc:.1f} Hz")
print(f"  Phase margin:     PM    = {PM:.1f}°")
print(f"  Phase crossover:  f_pc  = {f_pc:.0f} Hz")
print(f"  Gain margin:      GM    = {GM:.1f} dB")
print(f"  Closed-loop BW:   f_bw  = {f_bw_meas:.1f} Hz  (target {BW} Hz)")

# ── Plot ─────────────────────────────────────────────────────────────────────
fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

# Magnitude
ax_m.semilogx(freq, mag_G, color="steelblue",  lw=2.0, label=r"Plant $G_d(z)$")
ax_m.semilogx(freq, mag_L, color="darkorange", lw=2.0, label=r"Open loop $L(z) = C(z)\,G_d(z)$")
ax_m.semilogx(freq, mag_T, color="seagreen",   lw=2.0, label=r"Closed loop $T(z)$")
ax_m.axhline(0,  color="black", lw=0.7, ls="--", alpha=0.6)
ax_m.axhline(-3, color="gray",  lw=0.8, ls=":",  label="−3 dB")
ax_m.axvline(BW, color="red",   lw=0.9, ls="--", alpha=0.7, label=f"Target BW = {BW} Hz")
ax_m.set_ylabel("Magnitude (dB)", fontsize=11)
ax_m.set_title(
    f"Discrete PI Current Controller — Bode Plot\n"
    f"pqGain = {pqGain:.4f} V/A    iqGain = {iqGain:.5f}    "
    f"PM = {PM:.0f}°    GM = {GM:.1f} dB",
    fontsize=11
)
ax_m.legend(fontsize=9, loc="lower left")
ax_m.grid(True, which="both", alpha=0.35)
ax_m.set_ylim(-80, 50)

# Phase
ax_p.semilogx(freq, ph_G, color="steelblue",  lw=2.0, label=r"Plant $G_d(z)$")
ax_p.semilogx(freq, ph_L, color="darkorange", lw=2.0, label=r"Open loop $L(z)$")
ax_p.semilogx(freq, ph_T, color="seagreen",   lw=2.0, label=r"Closed loop $T(z)$")
ax_p.axhline(-180, color="black", lw=0.7, ls="--", alpha=0.6, label="−180°")
ax_p.axvline(BW,   color="red",   lw=0.9, ls="--", alpha=0.7)
ax_p.set_xlabel("Frequency (Hz)", fontsize=11)
ax_p.set_ylabel("Phase (deg)", fontsize=11)
ax_p.legend(fontsize=9, loc="lower left")
ax_p.grid(True, which="both", alpha=0.35)
ax_p.set_ylim(-200, 30)

ax_m.set_xlim(freq[0], freq[-1])
plt.tight_layout()

script_dir = os.path.dirname(os.path.abspath(__file__))
out_path = os.path.join(script_dir, "current_controller_bode.png")
plt.savefig(out_path, dpi=150)
print(f"\nSaved: {out_path}")
subprocess.Popen(['eog', out_path],
                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
