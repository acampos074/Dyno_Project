import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from collections import defaultdict
import glob
import os

# ── Config ─────────────────────────────────────────────────────────────────────
KT        = 0.0217   # Motor torque constant  (Nm/A)
SKIP_FRAC = 0.5      # Skip first 50 % of each step to allow velocity settling

# ── Load data ──────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_files  = sorted(glob.glob(os.path.join(script_dir, "coulomb_*.csv")))
if not csv_files:
    raise FileNotFoundError("No coulomb_*.csv found in postprocess/")
csv_path = csv_files[-1]
print(f"Loading: {csv_path}")

# Header: Time,Vel CMD,Voltage MSR,Current,Torque,Speed,Pos,Elec Pwr,Mech Pwr,Eff
data    = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
vel_cmd = data[:, 1]   # Vel CMD (rad/s)
iq      = data[:, 3]   # Current (A)
omega   = data[:, 5]   # Speed   (rad/s)
torque  = iq * KT

print(f"Rows: {len(data)}")

# ── Segment by velocity command (run-length encoding) ─────────────────────────
edges = np.where(np.diff(vel_cmd) != 0)[0] + 1
edges = np.concatenate([[0], edges, [len(vel_cmd)]])

segments = []
for i in range(len(edges) - 1):
    s, e  = int(edges[i]), int(edges[i + 1])
    v_cmd = vel_cmd[s]
    n     = e - s
    skip  = max(1, int(n * SKIP_FRAC))
    segments.append(dict(
        v_cmd      = float(v_cmd),
        avg_omega  = float(np.mean(omega [s + skip : e])),
        avg_torque = float(np.mean(torque[s + skip : e])),
        n_samples  = n
    ))

print(f"Detected {len(segments)} segments")

# Drop trailing zero-vel shutdown coast (longer than 1.5× any normal step)
if segments and segments[-1]["v_cmd"] == 0.0:
    max_normal = max(s["n_samples"] for s in segments[:-1])
    if segments[-1]["n_samples"] > 1.5 * max_normal:
        print(f"  → removing shutdown coast "
              f"({segments[-1]['n_samples']} samples at 0 rad/s)")
        segments = segments[:-1]

print(f"Using {len(segments)} velocity steps after trimming")

# ── Combine forward and backward passes at matching velocities ─────────────────
grouped = defaultdict(list)
for seg in segments:
    grouped[seg["v_cmd"]].append((seg["avg_omega"], seg["avg_torque"]))

vel_vec    = np.array(sorted(grouped.keys()))
omega_vec  = np.array([np.mean([e[0] for e in grouped[v]]) for v in vel_vec])
torque_vec = np.array([np.mean([e[1] for e in grouped[v]]) for v in vel_vec])

print(f"\n{'Vel CMD':>10}  {'n passes':>8}  {'Avg Speed':>10}  {'Avg Torque (mNm)':>17}")
for v in vel_vec:
    entries = grouped[v]
    print(f"{v:>10.1f}  {len(entries):>8d}  "
          f"{np.mean([e[0] for e in entries]):>10.3f}  "
          f"{np.mean([e[1] for e in entries])*1000:>17.4f}")

# ── Split into forward / backward passes for individual markers ────────────────
n_fwd    = min(31, len(segments))
fwd_segs = segments[:n_fwd]
bwd_segs = segments[n_fwd:]

# ── Regression: cmd velocity vs measured velocity ──────────────────────────────
coeffs   = np.polyfit(vel_vec, omega_vec, 1)
slope, intercept = coeffs
omega_fit = np.polyval(coeffs, vel_vec)

ss_res = np.sum((omega_vec - omega_fit) ** 2)
ss_tot = np.sum((omega_vec - omega_vec.mean()) ** 2)
r2     = 1.0 - ss_res / ss_tot

sign   = "+" if intercept >= 0 else "-"
eq_str = f"y = {slope:.4f}x {sign} {abs(intercept):.4f}\n$R^2$ = {r2:.6f}"
print(f"\nVelocity tracking regression:")
print(f"  slope     = {slope:.4f}")
print(f"  intercept = {intercept:.4f}")
print(f"  R²        = {r2:.6f}")

# ── Plot 1: Torque vs Velocity ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))

ax.plot([s["avg_omega"] for s in fwd_segs],
        [s["avg_torque"] * 1000 for s in fwd_segs],
        'o', color="steelblue", alpha=0.55, markersize=7,
        label="Forward pass  (−30 → +30 rad/s)")

if bwd_segs:
    ax.plot([s["avg_omega"] for s in bwd_segs],
            [s["avg_torque"] * 1000 for s in bwd_segs],
            's', color="tomato", alpha=0.55, markersize=7,
            label="Backward pass (+28 → −30 rad/s)")

ax.plot(omega_vec, torque_vec * 1000,
        'k-o', linewidth=1.8, markersize=4, label="Combined average")

ax.axhline(0, color="black", linewidth=0.7, linestyle="--")
ax.axvline(0, color="black", linewidth=0.7, linestyle="--")
ax.set_xlabel("Measured Velocity (rad/s)")
ax.set_ylabel("Torque  Iq × Kt  (mNm)")
ax.set_title("Coulomb Friction: Torque vs Velocity")
ax.legend()
ax.grid(True, alpha=0.4)
plt.tight_layout()

out_path = os.path.join(script_dir, "coulomb_friction.png")
plt.savefig(out_path, dpi=150)
print(f"\nSaved: {out_path}")

# ── Plot 2: Cmd velocity vs measured velocity (correlation) ────────────────────
fig2, ax2 = plt.subplots(figsize=(7, 6))

ax2.scatter(vel_vec, omega_vec, color="steelblue", s=40, zorder=3,
            label="Step averages (fwd+bwd combined)")
ax2.plot(vel_vec, omega_fit, color="tomato", linewidth=1.8,
         label=eq_str)
ax2.plot(vel_vec, vel_vec, color="gray", linewidth=1.0, linestyle="--",
         label="Ideal (y = x)")

ax2.set_xlabel("Commanded Velocity (rad/s)")
ax2.set_ylabel("Measured Velocity (rad/s)")
ax2.set_title("Velocity Tracking: Cmd vs Measured")
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.4)
plt.tight_layout()

out_path2 = os.path.join(script_dir, "coulomb_velocity_correlation.png")
plt.savefig(out_path2, dpi=150)
print(f"Saved: {out_path2}")

# ── Stribeck model fit (|ω| ≤ 6 rad/s) ───────────────────────────────────────
# T(ω) = (Fc + Fs·exp(-(ω/vs)²)) · tanh(ω/ωc) + Fv·ω
# Fc : Coulomb friction          (Nm)
# Fs : Stribeck extra amplitude  (Nm)   — friction peak above Fc near zero speed
# vs : Stribeck velocity         (rad/s)
# Fv : Viscous coefficient       (Nm·s/rad)
# ωc = 0.5 rad/s fixed — smooths the sign discontinuity at zero
OMEGA_C  = 0.5        # transition smoothing at zero  (rad/s, fixed)
FV_FIXED = 8.4e-5    # viscous coefficient from spin-down test (Nm·s/rad, fixed)

def stribeck_model(omega, Fc, Fs, vs):
    """T(ω) = (Fc + Fs·exp(-(ω/vs)²)) · tanh(ω/ωc) + Fv·ω
       Fc : Coulomb friction amplitude (Nm)       — plateau at high speed
       Fs : Stribeck amplitude above Fc (Nm)      — extra friction near standstill
       vs : Stribeck velocity (rad/s)              — decay rate of stiction peak
       Fv : viscous coefficient (Nm·s/rad)         — fixed from spin-down test
       ωc : smoothing constant (rad/s)             — fixed at 0.5"""
    return (Fc + Fs * np.exp(-(omega / vs)**2)) * np.tanh(omega / OMEGA_C) \
           + FV_FIXED * omega

mask   = np.abs(vel_vec) <= 6.0
x_data = omega_vec[mask]      # measured velocity as independent variable
y_data = torque_vec[mask]

p0     = [1.5e-3, 0.8e-3, 3.0]
bounds = ([0, 0, 0.5], [5e-3, 5e-3, 30.0])
popt, pcov = curve_fit(stribeck_model, x_data, y_data,
                       p0=p0, bounds=bounds, maxfev=10000)
perr = np.sqrt(np.diag(pcov))
Fc_fit, Fs_fit, vs_fit = popt

y_pred   = stribeck_model(x_data, *popt)
ss_res   = np.sum((y_data - y_pred) ** 2)
ss_tot   = np.sum((y_data - y_data.mean()) ** 2)
r2_strib = 1.0 - ss_res / ss_tot

print(f"\nStribeck model fit  (|ω| ≤ 6 rad/s):")
print(f"  Fc = {Fc_fit*1e3:.4f} ± {perr[0]*1e3:.4f} mNm   (Coulomb)")
print(f"  Fs = {Fs_fit*1e3:.4f} ± {perr[1]*1e3:.4f} mNm   (Stribeck peak above Coulomb)")
print(f"  vs = {vs_fit:.4f}  ± {perr[2]:.4f}  rad/s (Stribeck velocity)")
print(f"  Fv = {FV_FIXED*1e6:.1f} μNm·s/rad          (fixed from spin-down test)")
print(f"  R² = {r2_strib:.6f}")

# Fine grid for smooth model curve
omega_smooth  = np.linspace(-7, 7, 400)
torque_smooth = stribeck_model(omega_smooth, *popt)

legend_str = (f"Stribeck fit\n"
              f"$F_c$ = {Fc_fit*1e3:.3f} mNm\n"
              f"$F_s$ = {Fs_fit*1e3:.3f} mNm\n"
              f"$v_s$ = {vs_fit:.3f} rad/s\n"
              f"$F_v$ = {FV_FIXED*1e6:.1f} μNm·s/rad (fixed)\n"
              f"$R^2$ = {r2_strib:.5f}")

# ── Plot 3: Stribeck fit ───────────────────────────────────────────────────────
fig3, ax3 = plt.subplots(figsize=(8, 5))

ax3.scatter(x_data, y_data * 1e3, color="steelblue", s=70, zorder=5,
            label="Step averages (fwd+bwd, |ω| ≤ 6 rad/s)")
ax3.plot(omega_smooth, torque_smooth * 1e3, color="tomato",
         linewidth=2.0, label=legend_str)
ax3.axhline(0, color="black", linewidth=0.7, linestyle="--")
ax3.axvline(0, color="black", linewidth=0.7, linestyle="--")
ax3.set_xlabel("Measured Velocity (rad/s)")
ax3.set_ylabel("Torque  Iq × Kt  (mNm)")
ax3.set_title("Stribeck Friction Model Fit  (|ω| ≤ 6 rad/s)")
ax3.legend(fontsize=9, loc="upper left")
ax3.grid(True, alpha=0.4)
plt.tight_layout()

out_path3 = os.path.join(script_dir, "coulomb_stribeck_fit.png")
plt.savefig(out_path3, dpi=150)
print(f"Saved: {out_path3}")
subprocess.Popen(['eog', out_path, out_path2, out_path3],
                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
