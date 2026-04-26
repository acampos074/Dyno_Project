"""
analyze_eccentricity.py

Loads encoder error data collected by calibrate_HES2(), unwraps the
modular encoder counts, removes the DC offset, and plots:
  1. Raw error signal vs. mechanical angle
  2. FFT spectrum — shows which harmonics are present
  3. Reconstructed harmonic fit (1x + 2x + 3x + 7x/rev)

Usage:
    python analyze_eccentricity.py [path/to/eccentricity.csv]
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive — works without a display
import matplotlib.pyplot as plt

# ── config ────────────────────────────────────────────────────────────────
ENC_COUNTS       = 16384    # 14-bit encoder full-scale
CSV_FILE         = sys.argv[1] if len(sys.argv) > 1 else "eccentricity.csv"
N_HARMONICS_PLOT = 20       # how many harmonics to show in FFT bar chart
LUT_SIZE         = 128      # LUT entries — must be power of 2, matches SAMPLES_PER_PPAIR
# ──────────────────────────────────────────────────────────────────────────

# 1. Load ─────────────────────────────────────────────────────────────────
data = np.loadtxt(CSV_FILE, delimiter=",", skiprows=1)
idx   = data[:, 0].astype(int)
error = data[:, 1].astype(float)
N     = len(error)

# 2. Convert to signed errors ────────────────────────────────────────────
# calibrate_HES2 stores errors as (raw - ref) mod ENC_COUNTS (always >= 0).
# Values > ENC_COUNTS/2 are actually small negative errors that wrapped.
error_signed = np.where(error > ENC_COUNTS / 2, error - ENC_COUNTS, error)

# 3. Remove outliers (> 3 sigma from median) ─────────────────────────────
# The first few samples can be noisy before the motor stabilises.
median = np.median(error_signed)
sigma  = np.std(error_signed)
mask   = np.abs(error_signed - median) < 3.0 * sigma
idx_clean   = idx[mask]
error_clean = error_signed[mask]
n_removed   = N - mask.sum()
if n_removed:
    print(f"Removed {n_removed} outlier(s) (>3σ from median={median:.1f})")

# 4. Remove DC + linear trend ────────────────────────────────────────────
# A linear trend can appear if the motor speed wasn't perfectly constant.
trend   = np.polyfit(idx_clean, error_clean, 1)
detrend = error_clean - np.polyval(trend, idx_clean)

# Rebuild full-length arrays (fill removed samples with interpolation)
idx    = idx_clean
error  = error_clean

# Convert to degrees for readability
N          = len(detrend)
deg_per_count = 360.0 / ENC_COUNTS
error_deg     = detrend * deg_per_count

# 4. FFT ──────────────────────────────────────────────────────────────────
fft_coeffs = np.fft.rfft(detrend) / N
freqs      = np.fft.rfftfreq(N, d=1.0 / N)   # harmonic number (1 = 1x/rev)
amplitudes = 2.0 * np.abs(fft_coeffs)         # two-sided → one-sided amplitude
amplitudes[0] /= 2                             # DC bin is single-sided
amp_deg    = amplitudes * deg_per_count

# Print top harmonics
print(f"\nLoaded {N} samples from '{CSV_FILE}'")
print(f"Peak-to-peak error: {error_deg.max() - error_deg.min():.4f} deg")
print(f"\nTop {N_HARMONICS_PLOT} harmonics (mechanical cycles/rev):")
print(f"  {'harmonic':>10}  {'amplitude_deg':>14}  {'amplitude_counts':>16}")
top_idx = np.argsort(amp_deg[1:])[::-1][:N_HARMONICS_PLOT] + 1  # skip DC
for i in top_idx:
    print(f"  {freqs[i]:>10.1f}  {amp_deg[i]:>14.4f}  {amplitudes[i]:>16.2f}")

# 5. Reconstructed fit using dominant harmonics ──────────────────────────
# Keep harmonics above 1% of the strongest harmonic
theta_deg = np.linspace(0, 360, N, endpoint=False)
theta_fit = 2.0 * np.pi * np.arange(N) / N
threshold  = amp_deg[top_idx[0]] * 0.01
keep       = [i for i in range(1, len(amp_deg)) if amp_deg[i] >= threshold]
fit        = np.zeros(N)
for k in keep:
    phase = np.angle(fft_coeffs[k])
    fit  += amplitudes[k] * np.cos(k * theta_fit + phase)
fit_deg = fit * deg_per_count

# 6. Build LUT ────────────────────────────────────────────────────────────
# The LUT correction is the NEGATIVE of the measured error, downsampled to
# LUT_SIZE bins.  At runtime: corrected_raw = raw + lut[raw * LUT_SIZE / ENC_COUNTS]
#
# Each sample i maps to mechanical angle i/N * 360°, which maps to LUT bin
# round(i * LUT_SIZE / N).  We accumulate into bins then average.
lut_sum   = np.zeros(LUT_SIZE)
lut_count = np.zeros(LUT_SIZE, dtype=int)
for i, err in enumerate(detrend):
    bin_idx = int(round(i * LUT_SIZE / N)) % LUT_SIZE
    lut_sum[bin_idx]   += -err          # correction = -error
    lut_count[bin_idx] += 1

# Average and round to int16 (counts)
lut_counts = np.where(lut_count > 0, lut_sum / lut_count, 0.0)
lut_int16  = np.round(lut_counts).astype(np.int16)
lut_deg    = lut_int16 * deg_per_count

# Residual after LUT correction (interpolate LUT back to full resolution)
lut_applied = np.array([lut_int16[int(i * LUT_SIZE / N) % LUT_SIZE] for i in range(N)],
                        dtype=float)
residual_lut     = detrend + lut_applied          # error + correction
residual_lut_deg = residual_lut * deg_per_count

print(f"\nLUT ({LUT_SIZE} entries, int16):")
print(f"  Range  : [{lut_int16.min()}, {lut_int16.max()}] counts  "
      f"= [{lut_deg.min():.4f}, {lut_deg.max():.4f}] deg")
print(f"  RMS before correction : {np.std(detrend):.2f} counts  ({np.std(error_deg):.4f} deg)")
print(f"  RMS after  correction : {np.std(residual_lut):.2f} counts  ({np.std(residual_lut_deg):.4f} deg)")

# 7. Plot ─────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(4, 1, figsize=(12, 14))
fig.suptitle("Encoder Eccentricity Analysis", fontsize=14)

# Panel 1: error signal
ax = axes[0]
ax.plot(theta_deg, error_deg, lw=0.8, label="measured error")
ax.plot(theta_deg, fit_deg,   lw=1.5, color="red", label="harmonic fit")
ax.set_xlabel("Mechanical angle (deg)")
ax.set_ylabel("Error (deg)")
ax.set_title("Encoder positional error (detrended)")
ax.legend()
ax.grid(True, alpha=0.3)

# Panel 2: FFT spectrum
ax = axes[1]
plot_n = min(N_HARMONICS_PLOT + 1, len(freqs))
ax.bar(freqs[1:plot_n], amp_deg[1:plot_n], width=0.4)
ax.set_xlabel("Harmonic (cycles/rev)")
ax.set_ylabel("Amplitude (deg)")
ax.set_title("Harmonic spectrum")
ax.set_xticks(freqs[1:plot_n])
ax.grid(True, alpha=0.3, axis="y")

# Panel 3: residual after harmonic fit
residual = error_deg - fit_deg
ax = axes[2]
ax.plot(theta_deg, residual, lw=0.8, color="grey")
ax.set_xlabel("Mechanical angle (deg)")
ax.set_ylabel("Residual (deg)")
ax.set_title(f"Residual after harmonic fit  (RMS = {np.std(residual):.4f} deg)")
ax.grid(True, alpha=0.3)

# Panel 4: LUT
lut_angle = np.linspace(0, 360, LUT_SIZE, endpoint=False)
ax = axes[3]
ax.bar(lut_angle, lut_deg, width=360.0 / LUT_SIZE * 0.8,
       color="steelblue", label="LUT correction")
ax.plot(theta_deg, residual_lut_deg, lw=0.8, color="red",
        alpha=0.7, label=f"residual after LUT (RMS={np.std(residual_lut_deg):.4f}°)")
ax.axhline(0, color="black", lw=0.5)
ax.set_xlabel("Mechanical angle (deg)")
ax.set_ylabel("Correction (deg)")
ax.set_title(f"LUT ({LUT_SIZE} entries, int16)  —  range [{lut_int16.min()}, {lut_int16.max()}] counts")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("eccentricity_analysis.png", dpi=150)
print("\nPlot saved to eccentricity_analysis.png")

# 8. Print C-formatted LUT ────────────────────────────────────────────────
print("\n" + "="*60)
print("C-ARRAY FOR STM32 COMPENSATOR")
print("="*60)
print(f"const int16_t eccentricity_lut[{LUT_SIZE}] = {{")

# Print values in rows of 8 for readability
for i in range(0, LUT_SIZE, 8):
    chunk = lut_int16[i:i+8]
    line = ", ".join(f"{val:>4}" for val in chunk)
    # Add a comma if it's not the very last chunk
    suffix = "," if i + 8 < LUT_SIZE else ""
    print(f"    {line}{suffix}")

print("};")
print("="*60)
