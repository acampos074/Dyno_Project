import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.signal
import sys

def load_log(filepath):
    df = pd.read_csv(filepath)

    time        = df["Time (s)"].to_numpy()
    voltage_cmd = df["Voltage CMD(V)"].to_numpy()
    voltage_msr = df["Voltage MSR(V)"].to_numpy()
    current     = df["Current (A)"].to_numpy()
    torque      = df["Torque (Nm)"].to_numpy()
    speed       = df["Speed (rad/s)"].to_numpy()
    position    = df["Pos (rad)"].to_numpy()
    elec_power  = df["Elec Power (W)"].to_numpy()
    mech_power  = df["Mech Power (W)"].to_numpy()
    efficiency  = df["Efficiency"].to_numpy()

    time = time - time[0]

    Km = 0.0217  # N·m/A
    torque_calc = (3.0 / 2.0) * current * Km

    fs = 50.0
    b, a = scipy.signal.butter(4, 10.0 / (fs / 2), btype="low")
    speed_filt   = scipy.signal.filtfilt(b, a, speed)
    torque_filt  = scipy.signal.filtfilt(b, a, torque_calc)
    torque_calc  = torque_filt

    accel = np.gradient(speed_filt, time)  # differentiate filtered velocity

    speed_from_pos = np.gradient(position, time)  # velocity estimated from position

    return (time, voltage_cmd, voltage_msr, current, torque,
            speed, speed_filt, position, elec_power, mech_power, efficiency, torque_calc, accel, speed_from_pos)


if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else "log_20260420_211030.csv"
    (time, voltage_cmd, voltage_msr, current, torque,
     speed, speed_filt, position, elec_power, mech_power, efficiency, torque_calc, accel, speed_from_pos) = load_log(filepath)

    labels = ["time", "voltage_cmd", "voltage_msr", "current", "torque",
              "speed", "speed_filt", "position", "elec_power", "mech_power", "efficiency", "torque_calc", "accel", "speed_from_pos"]
    for label, arr in zip(labels, (time, voltage_cmd, voltage_msr, current, torque,
                                   speed, speed_filt, position, elec_power, mech_power, efficiency, torque_calc, accel, speed_from_pos)):
        print(f"{label}: {len(arr)} samples, first={arr[0]:.6f}, last={arr[-1]:.6f}")

    # Integrated equation of motion: ∫τ dt = J·ω + B·θ + C
    # Avoids noisy differentiation by integrating both sides instead
    tau_integral = scipy.integrate.cumulative_trapezoid(torque_calc, time, initial=0)

    A = np.column_stack((speed_filt, position, np.ones(len(time))))
    y = tau_integral

    theta, residuals, rank, s = np.linalg.lstsq(A, y, rcond=None)
    J_est, B_est, C_est = theta

    print(f"\nLeast-squares estimates (integrated EOM):")
    print(f"  J (rotor inertia)   = {J_est:.6e} kg·m²")
    print(f"  B (viscous damping) = {B_est:.6e} N·m·s/rad")
    print(f"  C (initial cond.)   = {C_est:.6e}")

    # Continuous state-space: x = [θ, ω], u = τ
    #   dx/dt = Ac*x + Bc*u
    Ac = np.array([[0.0,        1.0      ],
                   [0.0, -B_est / J_est  ]])
    Bc = np.array([[0.0     ],
                   [1.0/J_est]])

    # ZOH discretization at 1 kHz simulation rate
    dt_sim = 1.0 / 1000.0
    Ad, Bd, _, _, _ = scipy.signal.cont2discrete((Ac, Bc, np.eye(2), np.zeros((2, 1))), dt_sim, method="zoh")

    print(f"\nDiscrete A matrix (ZOH, dt={dt_sim} s):")
    print(Ad)
    print(f"Discrete B matrix:")
    print(Bd)

    # Upsample torque input from 50 Hz to 1 kHz via linear interpolation
    time_sim = np.arange(time[0], time[-1], dt_sim)
    torque_sim = np.interp(time_sim, time, torque_calc)

    # Simulate at 1 kHz: x[k+1] = Ad @ x[k] + Bd @ u[k]
    sim_pos = np.empty(len(time_sim))
    sim_vel = np.empty(len(time_sim))
    x = np.array([position[0], speed_filt[0]])
    for k in range(len(time_sim)):
        sim_pos[k] = x[0]
        sim_vel[k] = x[1]
        if k < len(time_sim) - 1:
            x = Ad @ x + Bd.flatten() * torque_sim[k]

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(time,     position, label="Measured",  linewidth=1.5)
    ax1.plot(time_sim, sim_pos,  label="Simulated", linewidth=1.5, linestyle="--")
    ax1.set_ylabel("Position (rad)")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time,     speed,      label="Measured (raw)",      linewidth=1.0, alpha=0.5)
    ax2.plot(time,     speed_filt, label="Measured (filtered)", linewidth=1.5)
    ax2.plot(time_sim, sim_vel,    label="Simulated",           linewidth=1.5, linestyle="--")
    ax2.set_ylabel("Velocity (rad/s)")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True)

    fig.suptitle(f"J = {J_est:.3e} kg·m²,  B = {B_est:.3e} N·m·s/rad")
    plt.tight_layout()
    plt.savefig("simulation_results.png", dpi=150)
    plt.show()

    # Third plot: measured states only
    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax3.plot(time, position, linewidth=1.5)
    ax3.set_ylabel("Position (rad)")
    ax3.grid(True)

    ax4.plot(time, speed,      label="Raw",      linewidth=1.0, alpha=0.5)
    ax4.plot(time, speed_filt, label="Filtered", linewidth=1.5)
    ax4.set_ylabel("Velocity (rad/s)")
    ax4.set_xlabel("Time (s)")
    ax4.legend()
    ax4.grid(True)

    fig2.suptitle("Measured States")
    plt.tight_layout()
    plt.savefig("measured_states.png", dpi=150)
    plt.show()

    # Fourth plot: measured velocity vs velocity derived from position
    fig3, ax5 = plt.subplots(figsize=(10, 4))
    ax5.plot(time, speed,          label="Measured (sensor)",        linewidth=1.0, alpha=0.6)
    ax5.plot(time, speed_from_pos, label="Computed (d/dt position)", linewidth=1.5, linestyle="--")
    ax5.set_ylabel("Velocity (rad/s)")
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Velocity: Sensor vs Numerical Derivative of Position")
    ax5.legend()
    ax5.grid(True)
    plt.tight_layout()
    plt.savefig("velocity_comparison.png", dpi=150)
    plt.show()
