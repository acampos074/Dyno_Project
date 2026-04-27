import streamlit as st
import numpy as np
import plotly.graph_objects as go
from scipy.signal import cont2discrete

# --- CORE CONTROL LOGIC ---
def design_gains(J, B, BW, zeta=0.7071):
    wn = 2.0 * np.pi * BW
    Kp = J * wn**2
    Kd = 2.0 * zeta * wn * J - B
    return Kp, Kd

def compute_bode(J, B, fs, Kp, Kd):
    Ts = 1.0 / fs
    # Frequency range: 0.1 Hz to Nyquist (500 Hz for fs=1000)
    freq = np.logspace(-1, np.log10(fs / 2.0), 1000)
    w = 2.0 * np.pi * freq
    z = np.exp(1j * w * Ts)
    
    # Position plant: G_pos(s) = 1/(Js^2 + Bs)
    num_p_d, den_p_d, _ = cont2discrete(([1.0], [J, B, 0.0]), Ts, method='zoh')
    # Velocity plant: G_vel(s) = 1/(Js + B)
    num_v_d, den_v_d, _ = cont2discrete(([1.0], [J, B]), Ts, method='zoh')
    
    G_pos_d = np.polyval(num_p_d.flatten(), z) / np.polyval(den_p_d.flatten(), z)
    G_vel_d = np.polyval(num_v_d.flatten(), z) / np.polyval(den_v_d.flatten(), z)
    
    L_d = Kp * G_pos_d + Kd * G_vel_d
    T_d = (Kp * G_pos_d) / (1.0 + L_d)
    
    return freq, G_pos_d, L_d, T_d

# --- STREAMLIT UI SETUP ---
st.set_page_config(page_title="PD Impedance Designer", layout="wide")

if 'kp' not in st.session_state:
    st.session_state.kp, st.session_state.kd = design_gains(4.0102e-7, 8.4e-5, 100.0)

with st.sidebar:
    st.header("Mechanical Plant")
    J = st.number_input("Inertia J (kg·m²)", value=4.0102e-7, format="%.4e")
    B = st.number_input("Damping B (Nm·s/rad)", value=8.4e-5, format="%.2e")
    fs = st.number_input("Sampling Freq (Hz)", value=1000)
    
    st.divider()
    st.header("Auto-Design")
    target_bw = st.number_input("Target BW (Hz)", value=100.0)
    zeta = st.slider("Damping Ratio (ζ)", 0.1, 1.5, 0.7071)
    
    if st.button("Apply Target Bandwidth"):
        new_kp, new_kd = design_gains(J, B, target_bw, zeta)
        st.session_state.kp, st.session_state.kd = new_kp, new_kd

    st.divider()
    st.header("Manual Tuning")
    kp_val = st.number_input("Kp (Nm/rad)", value=float(st.session_state.kp), format="%.4e")
    kd_val = st.number_input("Kd (Nm·s/rad)", value=float(st.session_state.kd), format="%.4e")
    st.session_state.kp, st.session_state.kd = kp_val, kd_val

# Calculations
freq, G_d, L_d, T_d = compute_bode(J, B, fs, st.session_state.kp, st.session_state.kd)
def get_mag(h): return 20 * np.log10(np.abs(h) + 1e-12)
def get_phs(h): return np.degrees(np.unwrap(np.angle(h)))

# --- VISUALIZATION (SYNCED AXES) ---
# Shared configuration for the x-axis
shared_x_axis = dict(type="log", title="Frequency (Hz)", gridcolor='lightgray', exponentformat="power")

# Magnitude Plot
fig_mag = go.Figure()
fig_mag.add_trace(go.Scatter(x=freq, y=get_mag(G_d), name="Plant G(z)", line=dict(color='gray', dash='dot')))
fig_mag.add_trace(go.Scatter(x=freq, y=get_mag(L_d), name="Open Loop L(z)", line=dict(color='orange', width=3)))
fig_mag.add_trace(go.Scatter(x=freq, y=get_mag(T_d), name="Closed Loop T(z)", line=dict(color='green', width=2)))
fig_mag.add_vline(x=target_bw, line_dash="dash", line_color="red")

# Sync this x-axis to fig_phs by using matches='x'
fig_mag.update_xaxes(shared_x_axis, matches='x', showticklabels=False, title="")
fig_mag.update_yaxes(title="Magnitude (dB)", range=[-100, 60], gridcolor='lightgray')
fig_mag.update_layout(height=350, margin=dict(t=30, b=10), template="plotly_white", legend=dict(orientation="h", y=1.1, x=1))

# Phase Plot (The Baseline)
fig_phs = go.Figure()
fig_phs.add_trace(go.Scatter(x=freq, y=get_phs(G_d), name="Plant G(z)", line=dict(color='gray', dash='dot')))
fig_phs.add_trace(go.Scatter(x=freq, y=get_phs(L_d), name="Open Loop L(z)", line=dict(color='orange', width=3)))
fig_phs.add_trace(go.Scatter(x=freq, y=get_phs(T_d), name="Closed Loop T(z)", line=dict(color='green', width=2)))
fig_phs.add_hline(y=-180, line_dash="dash", line_color="black")
fig_phs.add_vline(x=target_bw, line_dash="dash", line_color="red")

fig_phs.update_xaxes(shared_x_axis)
fig_phs.update_yaxes(title="Phase (deg)", range=[-270, 30], gridcolor='lightgray')
fig_phs.update_layout(height=400, margin=dict(t=10, b=10), template="plotly_white", showlegend=False)

# --- MAIN LAYOUT ---
col1, col2 = st.columns([1, 3])

with col1:
    st.subheader("Firmware Output")
    st.code(f"Kp = {st.session_state.kp:.6e};\nKd = {st.session_state.kd:.6e};", language='c')
    
    idx_0db = np.argmin(np.abs(get_mag(L_d)))
    pm = 180 + get_phs(L_d)[idx_0db]
    beff = B + st.session_state.kd
    
    st.metric("Phase Margin", f"{pm:.1f}°")
    st.metric("Eff. Damping", f"{beff:.2e}")
    
    if pm > 0 and beff > 0:
        st.success("Status: STABLE ✅")
    else:
        st.error("Status: UNSTABLE ❌")

with col2:
    st.plotly_chart(fig_mag, use_container_width=True)
    st.plotly_chart(fig_phs, use_container_width=True)