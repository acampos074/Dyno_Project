import matplotlib
matplotlib.use('Agg')
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.ndimage import median_filter
import matplotlib.pyplot as plt
from itertools import groupby
import os

# ── Config ────────────────────────────────────────────────────────────────
ENC_COUNTS     = 16384  # 14-bit encoder full-scale
CSV_FILE       = "foc_open_loop.csv" # Ensure this matches your file name
TWO_PI         = 2.0 * np.pi
N_grid         = 512    # Grid points for interpolation
N_POLE_PAIRS   = 7      #
N_SLOTS        = 12     #
ANG_LP_CUTOFF  = 20     # cycles / revolution
theta          = np.linspace(0, TWO_PI, N_grid, endpoint=False)

def analyze_direction_and_split(df):
    """
    Automatically detects the forward and backward motion blocks.
    Uses 'Pos (rad)' since 'raw_enc' is missing from the CSV.
    """
    # Use 'Pos (rad)' as the position source
    pos_signal = df["Pos (rad)"].to_numpy()
    
    # Calculate delta between samples
    deltas = np.diff(pos_signal)
    
    # Handle wrap-around for Radians (-PI to PI or 0 to 2PI)
    # This keeps the velocity signal smooth even when the position wraps
    deltas = (deltas + np.pi) % (2 * np.pi) - np.pi
    
    # Smooth the velocity signal
    smooth_deltas = median_filter(deltas, size=11)

    # Threshold: Moving forward if velocity > 0.001 rad/sample 
    # (Adjust this threshold if your motor spins very slowly)
    is_moving_fwd = smooth_deltas > 0.001
    is_moving_bwd = smooth_deltas < -0.001

    def get_largest_block(mask):
        blocks = []
        for val, group in groupby(enumerate(mask), key=lambda x: x[1]):
            if val:
                g = list(group)
                blocks.append((g[0][0], g[-1][0]))
        return max(blocks, key=lambda x: x[1] - x[0]) if blocks else (0, 0)

    fwd_range = get_largest_block(is_moving_fwd)
    bwd_range = get_largest_block(is_moving_bwd)
    
    return fwd_range, bwd_range

def extract_revolutions(pos, torque, direction):
    """
    Interpolates each complete revolution onto the common angular grid.
    Friction is neutralized by negating backward torque before averaging.
    """
    pos_mapped = pos % TWO_PI
    diff = np.diff(pos_mapped)
    
    if direction == 'fwd':
        wrap_idx = np.where(diff < -np.pi)[0]
    else:
        wrap_idx = np.where(diff > np.pi)[0]

    # Need at least two wraps to define a complete revolution
    if len(wrap_idx) < 2:
        return np.empty((0, N_grid)), [], []

    rev_starts = wrap_idx[:-1] + 1
    rev_ends   = wrap_idx[1:]
    sign       = -1.0 if direction == 'bwd' else 1.0 # Reverse torque sign for backward

    mat = np.zeros((len(rev_starts), N_grid))
    for k, (s, e) in enumerate(zip(rev_starts, rev_ends)):
        p = pos_mapped[s:e]
        t = sign * torque[s:e]
        order = np.argsort(p)
        mat[k] = np.interp(theta, p[order], t[order])
    return mat, rev_starts, rev_ends

# 1. Load Data
if not os.path.exists(CSV_FILE):
    raise FileNotFoundError(f"Could not find {CSV_FILE}")
df = pd.read_csv(CSV_FILE)

# 2. Automatically find motion ranges
fwd_r, bwd_r = analyze_direction_and_split(df)
print(f"Detected Forward Motion:  Indices {fwd_r[0]} to {fwd_r[1]}")
print(f"Detected Backward Motion: Indices {bwd_r[0]} to {bwd_r[1]}")

# 3. Extract and Process Revolutions
pos_all    = df["Pos (rad)"].to_numpy()
torque_all = df["Torque (Nm)"].to_numpy()

# Forward direction
pos_fwd    = pos_all[fwd_r[0]:fwd_r[1]]
torque_fwd = torque_all[fwd_r[0]:fwd_r[1]]
torque_fwd -= torque_fwd.mean() #
mat_fwd, rs_fwd, re_fwd = extract_revolutions(pos_fwd, torque_fwd, 'fwd')
print(f"Forward : {len(rs_fwd)} complete revolutions")

# Backward direction
pos_bwd    = pos_all[bwd_r[0]:bwd_r[1]]
torque_bwd = torque_all[bwd_r[0]:bwd_r[1]]
torque_bwd -= torque_bwd.mean() #
mat_bwd, rs_bwd, re_bwd = extract_revolutions(pos_bwd, torque_bwd, 'bwd')
print(f"Backward: {len(rs_bwd)} complete revolutions")

# 4. Average all data
rev_mat     = np.vstack([mat_fwd, mat_bwd])
cogging_avg = rev_mat.mean(axis=0)
print(f"Total revolutions averaged: {len(rev_mat)}")

# 5. Filter in Angular Domain
ang_nyquist    = N_grid / 2.0
b_ang, a_ang   = butter(4, ANG_LP_CUTOFF / ang_nyquist, btype='low')
cogging_filt   = filtfilt(b_ang, a_ang, cogging_avg)

# 6. Plot Time Domain Results
fig, ax = plt.subplots(figsize=(10, 5))
for k in range(len(rev_mat)):
    ax.plot(theta, rev_mat[k], color="steelblue", alpha=0.2, linewidth=0.6)

ax.plot(theta, cogging_avg,  color="tomato",  linewidth=1.0, alpha=0.5, label="Average")
ax.plot(theta, cogging_filt, color="darkred", linewidth=2.0, label=f"LP {ANG_LP_CUTOFF} cyc/rev")
ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
ax.set_xlabel("Rotor position (rad)")
ax.set_ylabel("Torque (Nm)")
ax.set_title("Cogging Torque vs Rotor Position (Automated Direction Detection)")
ax.set_xlim(0, TWO_PI)
ax.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2, TWO_PI])
ax.set_xticklabels(["0", "π/2", "π", "3π/2", "2π"])
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.savefig("cogging_torque_automated.png", dpi=150)
print("Saved: cogging_torque_automated.png")

# 7. FFT Analysis
fft_vals  = np.fft.rfft(cogging_avg)
fft_freqs = np.fft.rfftfreq(N_grid, d=1.0/N_grid)
fft_mag   = np.abs(fft_vals) / (N_grid / 2)

fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle(f"Cogging Torque FFT Spectrum", fontsize=13)

# Left: Zoomed Linear
ax = axes[0]
ax.stem(fft_freqs, fft_mag * 1e3, linefmt="steelblue", basefmt="black")
ax.set_xlim(0, 50)
ax.set_xlabel("Frequency (cycles / revolution)")
ax.set_ylabel("Amplitude (mNm)")
ax.grid(True, alpha=0.4)

# Right: Full Log
ax = axes[1]
ax.semilogy(fft_freqs, fft_mag * 1e3 + 1e-6, color="steelblue")
ax.set_xlim(0, N_grid // 2)
ax.set_xlabel("Frequency (cycles / revolution)")
ax.set_ylabel("Amplitude (mNm, log)")
ax.grid(True, alpha=0.4, which="both")

plt.tight_layout()
plt.savefig("cogging_fft_automated.png", dpi=150)
print("Saved: cogging_fft_automated.png")

# Scale Nm to micro-Newton-meters (uNm) to preserve resolution
# 0.00032 Nm becomes 320 uNm
cogging_uNm_fixed = (cogging_filt * 1000000).astype(int)

print(f"\n// Cogging Torque LUT (Values in uNm - micro-Newton-meters)")
print(f"const int32_t cogging_torque_lut[512] = {{")
for i in range(0, 512, 8):
    chunk = cogging_uNm_fixed[i:i+8]
    line = ", ".join(f"{val:>6}" for val in chunk)
    suffix = "," if i + 8 < 512 else ""
    print(f"    {line}{suffix}")
print("};")
