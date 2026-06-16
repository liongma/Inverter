import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

SHOW_Q = False  # Hide q-axis plots

# ==== Motor parameters ====
R_s = 0.15
L_d = 240e-6
L_q = 520e-6

# ==== PI parameters (Direct Axis) ====
Kp_d = 1.2
Ki_d = 800.0  # rad/s

# ==== PI parameters (Quadrature Axis) ====
Kp_q = 2.4
Ki_q = 1000.0  # rad/s

# ==== Delay ====
T_plant = 30e-6
T_ctrl  = 20e-6
delay_total = T_plant + T_ctrl
pade_num, pade_den = ctrl.pade(delay_total, 1)
Delay_tf = ctrl.tf(pade_num, pade_den)

# ==== Plant ====
Gd_tf = ctrl.tf([1], [L_d, R_s])
Gq_tf = ctrl.tf([1], [L_q, R_s])

# --- Desired compensator settings ---
K_lead = 1.5
f_lead_zero, f_lead_pole = 600, 2400   # Hz
K_lag  = 1.0
f_lag_zero,  f_lag_pole  = 50, 5    # Hz

# --- Convert Hz → rad/s ---
w_lead_z, w_lead_p = 2*np.pi*f_lead_zero, 2*np.pi*f_lead_pole
w_lag_z,  w_lag_p  = 2*np.pi*f_lag_zero,  2*np.pi*f_lag_pole

# --- Compensators ---
Lead = K_lead * ctrl.tf([1, w_lead_z], [1, w_lead_p])
Lag  = K_lag  * ctrl.tf([1, w_lag_z],  [1, w_lag_p])
print(Lead)
print(Lag)

# ==== PI controllers ====
PI_d = ctrl.tf([Kp_d, Ki_d], [1, 0])
#print(PI_d)
PI_q = ctrl.tf([Kp_q, Ki_q], [1, 0])

# ==== Second-order LPF ====
f_f = 5000.0                # Hz
omega_c = 2*np.pi*f_f
zeta = 0.707
LPF2_tf = ctrl.tf([omega_c**2], [1, 2*zeta*omega_c, omega_c**2])

# ==== Combine ====
Gd_delay = Gd_tf * Delay_tf
Gq_delay = Gq_tf * Delay_tf

#C_d = Lead * Lag
C_d = PI_d
C_q = PI_q

L_d_tf = C_d * Gd_delay
L_q_tf = C_q * Gq_delay

# ==== Frequency range ====
w = np.logspace(1, 6, 2000)  # rad/s
f = w / (2*np.pi)            # Hz for plotting

# ==== Stability margins ====
gm, pm, wg, wp = ctrl.margin(L_d_tf)
print(f"\nDirect-axis stability margins:")
print(f"  Gain margin: {20*np.log10(gm):.2f} dB at {wg/(2*np.pi):.1f} Hz")
print(f"  Phase margin: {pm:.2f} deg at {wp/(2*np.pi):.1f} Hz")

# ==== Bode data ====
mag_c, phase_c, _ = ctrl.bode(C_d, w, plot=False)
mag_g, phase_g, _ = ctrl.bode(Gd_delay, w, plot=False)
mag_l, phase_l, _ = ctrl.bode(L_d_tf, w, plot=False)

# ==== Bode Plot - Direct Axis ====
plt.figure(figsize=(10, 8))
# Magnitude
plt.subplot(2, 1, 1)
plt.semilogx(f, 20*np.log10(mag_c), label='C_d(s)')
plt.semilogx(f, 20*np.log10(mag_g), label='G_d(s)')
plt.semilogx(f, 20*np.log10(mag_l), label='L_d(s)')

# Mark gain margin
if gm != np.inf and wg > 0:
    plt.axvline(wg/(2*np.pi), color='red', linestyle='--', alpha=0.7)
    plt.text(wg/(2*np.pi), 0, f"GM = {20*np.log10(gm):.1f} dB", 
             rotation=90, va='bottom', ha='right', color='red')

plt.ylabel('Magnitude (dB)')
plt.title('Bode Plot - Direct Axis')
plt.grid(True, which='both', ls=':')
plt.legend()

# Phase
plt.subplot(2, 1, 2)
plt.semilogx(f, np.degrees(phase_c), label='C_d(s)')
plt.semilogx(f, np.degrees(phase_g), label='G_d(s)')
plt.semilogx(f, np.degrees(phase_l), label='L_d(s)')

# Mark phase margin
if pm != np.inf and wp > 0:
    plt.axvline(wp/(2*np.pi), color='green', linestyle='--', alpha=0.7)
    plt.text(wp/(2*np.pi), -180+pm, f"PM = {pm:.1f}°", 
             rotation=90, va='bottom', ha='right', color='green')

plt.ylabel('Phase (deg)')
plt.xlabel('Frequency (Hz)')
plt.grid(True, which='both', ls=':')
plt.legend()
plt.tight_layout()

## ==== Root Locus ====
##plt.figure()
##ctrl.root_locus(L_d_tf)
##plt.title('Root Locus - Direct Axis')
##
##if SHOW_Q:
##    plt.figure()
##    ctrl.root_locus(L_q_tf)
##    plt.title('Root Locus - Quadrature Axis')

# ==== Time-domain step response ====
T_cl_d = ctrl.feedback(L_d_tf, 1)
t = np.linspace(0, 0.01, 1000)
t_out, y_out = ctrl.step_response(T_cl_d, t)

plt.figure()
plt.plot(t_out, y_out)
plt.title('Direct Axis Closed-loop Step Response')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.grid(True)

plt.show()
