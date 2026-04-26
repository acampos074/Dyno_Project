import matplotlib.pyplot as plt
import numpy as np
import scipy.signal

# Motor parameters
J  = 4.0102e-7  # kg·m²  (from spin-down test)
B  = 8.4e-5     # N·m·s/rad
Km = 0.025      # N·m/A  (torque constant = back-EMF constant, Ke = Km)
R  = 0.362      # Ω
L  = 567e-6     # H  (Ld = Lq = L)

# Simulation settings
fs = 1000.0     # Hz
dt = 1.0 / fs
T  = 0.5        # total time (s)
t  = np.arange(0, T, dt)
N  = len(t)

# Input: Vd = 0, Vq = 1V step
Vq_step = 1.0
Vd = np.zeros(N)
Vq = Vq_step * np.ones(N)

# ── Continuous state-space (linearised around x = 0) ───────────────────────
# States: x = [id, iq, ω]
# Nonlinear terms (ω·L·iq, ω·L·id) vanish at the operating point, giving:
#
#   d/dt [id]   [-R/L    0     0   ] [id]   [1/L  0  ] [Vd]
#       [iq] = [  0    -R/L  -Km/L] [iq] + [ 0   1/L] [Vq]
#       [ω ]   [  0    Km/J  -B/J ] [ω ]   [ 0    0 ]
#
Ac = np.array([
    [-R/L,    0,      0    ],
    [  0,   -R/L,  -Km/L  ],
    [  0,   Km/J,  -B/J   ]
])
Bc = np.array([
    [1/L,  0  ],
    [ 0,   1/L],
    [ 0,   0  ]
])

# ZOH discretisation at 1 kHz
Ad, Bd, _, _, _ = scipy.signal.cont2discrete(
    (Ac, Bc, np.eye(3), np.zeros((3, 2))), dt, method="zoh"
)

print("Continuous Ac:")
print(Ac)
print("\nContinuous Bc:")
print(Bc)
print(f"\nDiscrete Ad (ZOH, dt={dt*1000:.1f} ms):")
print(Ad)
print("\nDiscrete Bd:")
print(Bd)

# ── Nonlinear simulation via RK4 ────────────────────────────────────────────
# Full nonlinear DQ dynamics (cross-coupling and back-EMF included):
#   did/dt = (Vd - R·id + ω·L·iq) / L
#   diq/dt = (Vq - R·iq - ω·L·id - Km·ω) / L
#   dω/dt  = (Km·iq - B·ω) / J

def dynamics(x, Vd_k, Vq_k):
    id_, iq_, w = x
    return np.array([
        (-R*id_ + w*L*iq_          + Vd_k) / L,
        (-R*iq_ - w*L*id_ - Km*w  + Vq_k) / L,
        ( Km*iq_ - B*w)                    / J
    ])

def rk4(x, Vd_k, Vq_k):
    k1 = dynamics(x,            Vd_k, Vq_k)
    k2 = dynamics(x + dt/2*k1, Vd_k, Vq_k)
    k3 = dynamics(x + dt/2*k2, Vd_k, Vq_k)
    k4 = dynamics(x + dt   *k3, Vd_k, Vq_k)
    return x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

x      = np.zeros(3)
id_sim = np.empty(N)
iq_sim = np.empty(N)
w_sim  = np.empty(N)

for k in range(N):
    id_sim[k] = x[0]
    iq_sim[k] = x[1]
    w_sim[k]  = x[2]
    x = rk4(x, Vd[k], Vq[k])

torque_sim = Km * iq_sim

# ── Plots ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

axes[0].plot(t * 1000, id_sim, color="steelblue")
axes[0].set_ylabel("Id (A)")
axes[0].set_title(f"BLDC Open-Loop Simulation — Vq = {Vq_step} V, Vd = 0")
axes[0].grid(True)

axes[1].plot(t * 1000, iq_sim, color="darkorange")
axes[1].set_ylabel("Iq (A)")
axes[1].grid(True)

axes[2].plot(t * 1000, w_sim, color="seagreen")
axes[2].set_ylabel("ω (rad/s)")
axes[2].grid(True)

axes[3].plot(t * 1000, torque_sim, color="tomato")
axes[3].set_ylabel("Torque (N·m)")
axes[3].set_xlabel("Time (ms)")
axes[3].grid(True)

plt.tight_layout()
plt.savefig("motor_simulation.png", dpi=150)
plt.show()
