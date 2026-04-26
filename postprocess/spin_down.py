import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

# Load data
df = pd.read_csv("spin_down_test.csv")
t_ms = df["time (ms)"].to_numpy()
omega = df["velocity (rad/sec)"].to_numpy()

# Convert time to seconds
t = t_ms / 1000.0

# Known viscous damping coefficient
B = 8.4e-5  # N·m·s/rad

# Exponential model: ω(t) = ω0 · exp(-t / τ)
def exp_decay(t, omega0, tau):
    return omega0 * np.exp(-t / tau)

p0 = [omega[0], 0.01]  # initial guess: [ω0, τ]
popt, _ = curve_fit(exp_decay, t, omega, p0=p0)
omega0_fit, tau_fit = popt

# Estimate inertia from time constant: τ = J/B → J = τ·B
J_est = tau_fit * B

# Compute R²
omega_fit = exp_decay(t, *popt)
ss_res = np.sum((omega - omega_fit) ** 2)
ss_tot = np.sum((omega - np.mean(omega)) ** 2)
r2 = 1.0 - ss_res / ss_tot

print(f"Fitted ω₀  = {omega0_fit:.4f} rad/s")
print(f"Time constant τ = {tau_fit*1000:.4f} ms")
print(f"Estimated J     = {J_est:.4e} kg·m²")
print(f"R²              = {r2:.6f}")

# Plot
t_dense = np.linspace(t[0], t[-1], 1000)
omega_dense = exp_decay(t_dense, *popt)

fig, ax = plt.subplots(figsize=(7, 7))
ax.scatter(t * 1000, omega,          label="Measured", color="steelblue", zorder=3)
ax.plot(t_dense * 1000, omega_dense, label="Model fit", color="tomato", linewidth=2)

equation = (f"$\\omega(t) = {omega0_fit:.3f}\\,e^{{-t/{tau_fit*1000:.2f}\\,\\mathrm{{ms}}}}$\n"
            f"$R^2 = {r2:.4f}$\n"
            f"$\\tau = {tau_fit*1000:.2f}$ ms\n"
            f"$J = \\tau \\cdot B = {J_est:.3e}$ kg·m²")
ax.text(0.97, 0.60, equation, transform=ax.transAxes,
        fontsize=11, verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.8))

ax.set_xlabel("Time (ms)")
ax.set_ylabel("Velocity (rad/s)")
ax.set_title("Spin-Down Test — Exponential Fit")
ax.legend(loc="upper right")
ax.grid(True)
plt.tight_layout()
plt.savefig("spin_down_fit.png", dpi=150)
plt.show()
