import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import matplotlib.colors as colors

@dataclass(frozen=True)
class FETDevice:
    name: str
    R_DS_ON: float
    V_F0: float
    T_FALL: float
    T_RISE: float
    C_OSS: float
    P_GATE: float
    color: str = None


def compute_losses(V_ph_rms, I_ph_rms, pf, dev, V_DC, F_SW, T_DEAD=200e-9, N_PHASES=3):
    V_ph_rms = np.asarray(V_ph_rms, dtype=float)
    I_ph_rms = np.asarray(I_ph_rms, dtype=float)
    F_SW = np.asarray(F_SW, dtype=float)

    V_ph_pk = V_ph_rms * np.sqrt(2.0)
    m = np.clip(V_ph_pk / (V_DC / np.sqrt(3.0)), 0.0, 1.0)

    I_pk = I_ph_rms * np.sqrt(2.0)
    phi = np.arccos(np.clip(pf, -1.0, 1.0))

    I_avg_sw  = I_pk * (1.0/(2.0*np.pi) + m * np.cos(phi) / 8.0)
    I_rms_sw2 = I_pk**2 * (1.0/8.0       + m * np.cos(phi) / (3.0*np.pi))

    I_avg_d   = I_pk * (1.0/(2.0*np.pi) - m * np.cos(phi) / 8.0)
    I_rms_d2  = I_pk**2 * (1.0/8.0       - m * np.cos(phi) / (3.0*np.pi))

    I_avg_sw  = np.maximum(I_avg_sw, 0.0)
    I_rms_sw2 = np.maximum(I_rms_sw2, 0.0)
    I_avg_d   = np.maximum(I_avg_d, 0.0)
    I_rms_d2  = np.maximum(I_rms_d2, 0.0)

    P_cond_sw = dev.R_DS_ON * I_rms_sw2
    P_cond_d  = dev.V_F0 * I_avg_d
    P_cond_total = 6.0 * (P_cond_sw + P_cond_d)

    P_sw_leg = 2.0 * V_DC * I_pk * (2.0 / np.pi) * 0.5 * (dev.T_FALL + dev.T_RISE) * F_SW
    P_sw_total = N_PHASES * P_sw_leg

    P_coss_leg = 0.5 * dev.C_OSS * V_DC**2
    P_coss_total = N_PHASES * P_coss_leg * F_SW

    P_deadtime = N_PHASES * 2.0 * T_DEAD * F_SW * dev.V_F0 * I_ph_rms
    P_gate_total = dev.P_GATE

    return P_cond_total + P_sw_total + P_deadtime + P_gate_total + P_coss_total


devices = [
    FETDevice("IGBT", 1.0e-3,  1.00,  1e-6,  1e-6, 50e-9, 15, "#1f77b4"),
    FETDevice("SiC",  15.0e-3, 0.95, 25e-9, 25e-9,  4e-9,  6, "#ff7f0e"),
    FETDevice("GaN",  75.0e-3, 8.0,  12e-9, 10e-9,  4e-9,  6, "#2ca02c"),
]

V_DC = 800.0
pf = 0.98
V_ph_rms_fixed = 0.7 * (V_DC / np.sqrt(3.0) / np.sqrt(2.0))

f_sw  = np.linspace(5e3, 20e3, 180)
i_out = np.linspace(5.0, 80.0, 140)
F, I  = np.meshgrid(f_sw, i_out)

P_LIMIT = 300.0

fig = plt.figure(figsize=(12, 8))
ax  = fig.add_subplot(111, projection="3d")

# Gray constant-loss plane
Z_plane = np.full_like(F, P_LIMIT)
ax.plot_surface(F / 1000.0, I, Z_plane, alpha=0.18, color="gray", edgecolor="none")

# Label anchor: place text near the high-frequency, low-current corner of each
# surface so the labels fan out and don't overlap each other.
label_fi = [
    (19.0, 10.0),   # IGBT  – top-right-front corner
    (19.0, 25.0),   # SiC   – mid current
    (19.0, 45.0),   # GaN   – higher current
]

surf = None
for dev, (lf, li) in zip(devices, label_fi):
    Z = compute_losses(V_ph_rms_fixed, I, pf, dev, V_DC, F)

    surf = ax.plot_surface(
        F / 1000.0, I, Z,
        alpha=0.5,
        linewidth=0.15,
        antialiased=True,
        edgecolors="black",
        norm=colors.Normalize(vmin=0, vmax=1000),
        cmap="viridis",
    )

    # Intersection contour
    ax.contour(
        F / 1000.0, I, Z,
        levels=[P_LIMIT],
        colors=[dev.color],
        linewidths=3.0,
        linestyles="solid",
    )

    # ── Surface label ────────────────────────────────────────────────────────
    # Evaluate loss at the chosen (f, i) anchor point
    lz = float(compute_losses(V_ph_rms_fixed, li, pf, dev, V_DC, lf * 1e3))

    ax.text(
        lf, li, lz,
        f" {dev.name}",
        color=dev.color,
        fontsize=13,
        fontweight="bold",
        ha="left",
        va="bottom",
        # White outline so the text is readable against any surface colour
        bbox=dict(boxstyle="round,pad=0.15", fc="white", ec=dev.color,
                  lw=1.2, alpha=0.75),
    )

fig.colorbar(surf, ax=ax, shrink=0.6, aspect=12, pad=0.08, label="Loss (W)")

ax.set_xlabel("Switching frequency (kHz)")
ax.set_ylabel("Output current (A)")
ax.set_zlabel("Loss (W)")
ax.set_title(f"Device loss surfaces with intersection at P_LIMIT = {P_LIMIT:.0f} W")
plt.tight_layout()
plt.show()