import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import glob
import os

# ── Load data ──────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_files  = glob.glob(os.path.join(script_dir, "log_*.csv"))
if not csv_files:
    raise FileNotFoundError("No log_*.csv found in postprocess/")
csv_path = sorted(csv_files)[-1]          # most recent
print(f"Loading: {csv_path}")

df = pd.read_csv(csv_path)
torque = df["Torque (Nm)"].to_numpy()
pos    = df["Pos (rad)"].to_numpy()

# ── Filter design ──────────────────────────────────────────────────────────────
Fs     = 1.0 / 0.003          # 333.3 Hz
Fn     = Fs / 2.0             # Nyquist

hp_freq = 3.0                 # Hz  — remove DC bias
lp_freq = 100.0               # Hz  — remove high-freq noise

b_hp, a_hp = butter(2, hp_freq / Fn, btype='high')
b_lp, a_lp = butter(4, lp_freq / Fn, btype='low')

torque_hp   = filtfilt(b_hp, a_hp, torque)   # remove DC offset
torque_filt = filtfilt(b_lp, a_lp, torque_hp) # smooth

# ── Plot ───────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("Torque vs Position", fontsize=14)

# Raw
axes[0].plot(pos, torque, color="steelblue", alpha=0.6, linewidth=0.5)
axes[0].axhline(0, color="black", linewidth=0.8, linestyle="--")
axes[0].set_xlabel("Position (rad)")
axes[0].set_ylabel("Torque (Nm)")
axes[0].set_title("Raw")
axes[0].grid(True)

# Filtered
axes[1].plot(pos, torque_filt, color="tomato", linewidth=0.8)
axes[1].axhline(0, color="black", linewidth=0.8, linestyle="--")
axes[1].set_xlabel("Position (rad)")
axes[1].set_ylabel("Torque (Nm)")
axes[1].set_title(f"HP {hp_freq} Hz + LP {lp_freq} Hz (filtfilt)")
axes[1].grid(True)

plt.tight_layout()
out_path = os.path.join(script_dir, "torque_position.png")
plt.savefig(out_path, dpi=150)
print(f"Saved: {out_path}")
plt.show()
