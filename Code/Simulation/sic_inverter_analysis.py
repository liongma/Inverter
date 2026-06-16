import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from scipy.spatial import ConvexHull

# ═══════════════════════════════════════════════════════════════════════════════
#  Device & system parameters
# ═══════════════════════════════════════════════════════════════════════════════

# Electrical operating point (fixed across sweeps)
V_DC        = 600.0     # V     DC bus voltage
V_PH_RMS    = 350.0     # V     phase-to-neutral RMS voltage (operating parameter)
PF          = 0.85      # –     output power factor
N_PHASES    = 3         # –     number of phases

# Device parameters
R_DS_ON     = 15e-3      # Ω     MOSFET on-state resistance
V_F0        = 2.0       # V     body-diode forward voltage
T_RISE      = 25e-9     # s     current rise time
T_FALL      = 25e-9     # s     current fall time
T_DEAD      = 100e-9    # s     dead time per switching event
P_GATE      = 6.0       # W     total gate-drive power (fixed overhead, all 6)
C_OSS       = 4e-9
# Thermal
T_MAX       = 120.0     # °C   max junction temperature
R_TH        = 0.5      # K/W  junction-to-ambient thermal resistance (per device)
T_AMB       = 65.0      # °C   ambient / coolant temperature

# DC-link / volume
RIPPLE_PCT  = 0.05      # –    max voltage ripple fraction
K_CAP       = 2e-6    # L/(V·µF)  capacitor volume proportionality
V_COOLING   = 0.15      # L    fixed cooling / structural volume offset

# Sweep ranges
POWER_KW    = np.linspace(5, 50, 100)      # kW  output power
FREQ_HZ     = np.linspace(5e3, 50e3, 60)  # Hz  switching frequency
FREQ_LINES  = [10e3, 20e3, 30e3, 40e3, 50e3]

CMAP = plt.cm.jet


# ═══════════════════════════════════════════════════════════════════════════════
#  Core loss model
# ═══════════════════════════════════════════════════════════════════════════════

def compute_losses(V_ph_rms, I_ph_rms, F_SW, pf=PF):
    """
    Total inverter losses (W).

    Parameters
    ----------
    V_ph_rms : scalar or array  – phase-to-neutral RMS voltage  (V)
    I_ph_rms : scalar or array  – RMS phase current              (A)
    F_SW     : scalar           – switching frequency            (Hz)
    pf       : float            – output power factor

    Returns
    -------
    dict with keys: cond, sw, deadtime, gate, total  (all in W)
    """
    V_ph_pk = V_ph_rms * np.sqrt(2.0)
    I_pk    = I_ph_rms * np.sqrt(2.0)
    phi     = np.arccos(pf)

    # Modulation index  m = V_ph_pk / (V_DC / √3)
    m = np.clip(V_ph_pk / (V_DC / np.sqrt(3.0)), 0.0, 1.0)

    # ── Conduction losses (SPWM, per switch / per diode) ─────────────────────
    I_avg_sw  = I_pk * (1.0 / (2.0 * np.pi) + m * np.cos(phi) / 8.0)
    I_rms_sw2 = I_pk**2 * (1.0 / 8.0        + m * np.cos(phi) / (3.0 * np.pi))

    I_avg_d   = I_pk * (1.0 / (2.0 * np.pi) - m * np.cos(phi) / 8.0)
    I_rms_d2  = I_pk**2 * (1.0 / 8.0        - m * np.cos(phi) / (3.0 * np.pi))

    # Clamp (low-m / leading-pf edge cases)
    I_avg_sw  = np.maximum(I_avg_sw,  0.0)
    I_rms_sw2 = np.maximum(I_rms_sw2, 0.0)
    I_avg_d   = np.maximum(I_avg_d,   0.0)
    I_rms_d2  = np.maximum(I_rms_d2,  0.0)

    P_cond_sw    = R_DS_ON * I_rms_sw2          # per MOSFET
    P_cond_d     = V_F0    * I_avg_d            # per body diode
    P_cond_total = 6.0 * (P_cond_sw + P_cond_d)

    # ── Switching losses (linear rise/fall model, per leg) ────────────────────
    P_sw_leg = 2 * V_DC * I_pk * 2/np.pi * 0.5 * (T_FALL+ T_RISE) * F_SW
    P_sw_total = N_PHASES * P_sw_leg

    P_coss_leg = 0.5 * C_OSS * V_DC**2 * F_SW
    P_coss_total = N_PHASES * P_coss_leg

    # ── Dead-time losses ──────────────────────────────────────────────────────
    P_deadtime = N_PHASES * 2.0 * T_DEAD * F_SW * V_F0 * I_ph_rms

    # ── Gate-drive (fixed overhead) ───────────────────────────────────────────
    P_gate_total = P_GATE

    total = P_cond_total + P_sw_total + P_deadtime + P_gate_total + P_coss_total

    return dict(cond=P_cond_total, sw=P_sw_total,
                deadtime=P_deadtime, gate=P_gate_total, coss = P_coss_total, total=total)


def phase_current_from_power(Pout_W, V_ph_rms=V_PH_RMS, pf=PF):
    """I_ph_rms [A] from 3-phase output power."""
    return Pout_W / (np.sqrt(3) * V_ph_rms * pf + 1e-12)


def dc_link_cap(Pout_W, F_SW):
    """Required DC-link capacitance [F]."""
    I_dc    = Pout_W / (V_DC + 1e-12)
    delta_V = RIPPLE_PCT * V_DC
    return I_dc / (2.0 * F_SW * delta_V)


def inverter_volume(C_F,LOSS):
    """Estimated inverter volume [L]."""

    return K_CAP * V_DC * (C_F * 1e6) + 0.045 * (100/LOSS["total"])**(-0.8)




def is_thermally_feasible(L_total):
    """True if per-device junction temperature ≤ T_MAX."""
    T_j = T_AMB + (L_total / 6.0) * R_TH
    return T_j <= T_MAX


# ═══════════════════════════════════════════════════════════════════════════════
#  Pre-compute full 2-D grid  (POWER_KW × FREQ_HZ)
# ═══════════════════════════════════════════════════════════════════════════════

def compute_grid():
    nP, nF = len(POWER_KW), len(FREQ_HZ)
    eff2d   = np.zeros((nP, nF))
    pd2d    = np.zeros((nP, nF))
    cap2d   = np.zeros((nP, nF))
    vol2d   = np.zeros((nP, nF))
    pcond   = np.zeros((nP, nF))
    psw     = np.zeros((nP, nF))
    pdt     = np.zeros((nP, nF))
    pcoss   = np.zeros((nP, nF))

    feas    = np.zeros((nP, nF), dtype=bool)

    for j, f in enumerate(FREQ_HZ):
        for i, pk in enumerate(POWER_KW):
            Pout     = pk * 1e3
            I_rms    = phase_current_from_power(Pout)
            L        = compute_losses(V_PH_RMS, I_rms, f)
            C        = dc_link_cap(Pout, f)
            V        = inverter_volume(C,L)
            P_in     = Pout + L['total']

            eff2d[i, j] = 100.0 * Pout / P_in
            pd2d[i, j]  = (Pout / 1e3) / V
            cap2d[i, j] = C * 1e6
            vol2d[i, j] = V
            pcond[i, j] = L['cond']
            psw[i, j]   = L['sw']
            pdt[i, j]   = L['deadtime']
            pcoss[i, j]   = L['coss']

            feas[i, j]  = is_thermally_feasible(L['total'])

    return dict(eff=eff2d, pd=pd2d, cap=cap2d, vol=vol2d,
                pcond=pcond, psw=psw, pcoss = pcoss, pdt=pdt, feas=feas)


# ═══════════════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════════════════════
#  WINDOW 1 – Efficiency vs Power Density  (Pareto plot)
# ═══════════════════════════════════════════════════════════════════════════════

def plot_pareto(grid):
    fig, ax = plt.subplots(figsize=(9, 7))
    fig.patch.set_facecolor("#FFFFFF")
    ax.set_facecolor("#FFFFFF")
    fig.canvas.manager.set_window_title('Window 1 – Efficiency vs Power Density')

    nP, nF = len(POWER_KW), len(FREQ_HZ)
    norm = Normalize(vmin=FREQ_HZ.min()/1e3, vmax=FREQ_HZ.max()/1e3)

    all_pd, all_eff, all_freq, all_feas = [], [], [], []
    for j, f in enumerate(FREQ_HZ):
        for i in range(nP):
            e = grid['eff'][i, j]
            if e > 50:
                all_pd.append(grid['pd'][i, j])
                all_eff.append(e)
                all_freq.append(f / 1e3)
                all_feas.append(grid['feas'][i, j])

    all_pd   = np.array(all_pd)
    all_eff  = np.array(all_eff)
    all_freq = np.array(all_freq)
    all_feas = np.array(all_feas)
    
    # Pre-compute ranges for normalization (across all feasible points)
    feas_mask = grid['feas'] & (grid['eff'] > 50)
    pd_min, pd_max = grid['pd'][feas_mask].min(), grid['pd'][feas_mask].max()
    eff_min, eff_max = grid['eff'][feas_mask].min(), grid['eff'][feas_mask].max()
    pd_range  = pd_max  - pd_min  + 1e-9
    eff_range = eff_max - eff_min + 1e-9

    # Scatter – thermally feasible
    sc = ax.scatter(all_pd[all_feas], all_eff[all_feas],
                    c=all_freq[all_feas], cmap=CMAP, norm=norm,
                    s=18, alpha=0.78, linewidths=0, zorder=3)
    # Scatter – infeasible (faded)
    if (~all_feas).any():
        ax.scatter(all_pd[~all_feas], all_eff[~all_feas],
                   c=all_freq[~all_feas], cmap=CMAP, norm=norm,
                   s=14, alpha=0.15, linewidths=0, zorder=2)

    # ── New frontier: max-efficiency ridge swept low → high frequency ─────────
    # For each frequency slice, find the feasible point with highest efficiency.
    # Walk from low to high frequency; terminate when power density would decrease.
    frontier_pd  = []
    frontier_eff = []
    frontier_freq = []
    prev_pd = -np.inf

    for j, f in enumerate(FREQ_HZ):
        col_feas = grid['feas'][:, j]
        col_eff  = grid['eff'][:, j]
        col_pd   = grid['pd'][:, j]

        mask = col_feas & (col_eff > 50)
        if not mask.any():
            continue

        # Score = normalised PD + normalised efficiency (equal weight)
        score = ((col_pd  - pd_min)  / pd_range +
                (col_eff - eff_min) / eff_range) * mask

        best_i   = np.argmax(score)
        peak_eff = col_eff[best_i]
        peak_pd  = col_pd[best_i]

        if peak_pd < prev_pd:
            break

        frontier_pd.append(peak_pd)
        frontier_eff.append(peak_eff)
        frontier_freq.append(f / 1e3)
        prev_pd = peak_pd

    if len(frontier_pd) > 1:
        fp  = np.array(frontier_pd)
        fe  = np.array(frontier_eff)
        fqf = np.array(frontier_freq)

        ax.plot(fp, fe, 'm--', linewidth=2.5, zorder=5)

        # Knee point = last point on the frontier (where we stopped)
        ax.plot(fp[-1], fe[-1], marker='*', markersize=18,
                color='#FF69B4', markeredgecolor='white', markeredgewidth=0.8,
                zorder=8)
        ax.annotate('Knee Point',
                    xy=(fp[-1], fe[-1]),
                    xytext=(fp[-1] - 0.5, fe[-1] - 0.4),
                    fontsize=9, color='#00BFFF', fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color='#00BFFF', lw=1.4))

        mid = len(fp) // 2
        ax.annotate('Max-Eff Frontier',
                    xy=(fp[mid], fe[mid]),
                    xytext=(fp[mid] - 1.0, fe[mid] + 0.15),
                    fontsize=9, color='magenta', fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color='magenta', lw=1.2))

    # ── Thermal constraint boundary ───────────────────────────────────────────
    th_pd, th_eff = [], []
    for j, f in enumerate(FREQ_HZ):
        for i in range(nP):
            if not grid['feas'][i, j] and i > 0 and grid['feas'][i-1, j]:
                # Crossed the boundary between index i-1 (feasible) and i (not)
                th_pd.append(grid['pd'][i-1, j])
                th_eff.append(grid['eff'][i-1, j])
                break
        else:
            # Never hit the thermal limit at this frequency — take the max power point
            if grid['feas'][:, j].any():
                last_i = np.where(grid['feas'][:, j])[0][-1]
                th_pd.append(grid['pd'][last_i, j])
                th_eff.append(grid['eff'][last_i, j])

    if th_pd:
        ax.plot(th_pd, th_eff,
                color='#4444CC', linestyle='--', linewidth=2.2, zorder=6)
        mid2 = len(th_pd) // 2
        ax.annotate('Thermal\nConstraint',
                    xy=(th_pd[mid2], th_eff[mid2]),
                    xytext=(th_pd[mid2] + 0.3, th_eff[mid2] - 0.5),
                    fontsize=9, color='#4444CC', fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color='#4444CC', lw=1.2))
    

    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label('Switching Frequency (kHz)', fontsize=9)
    cb.ax.tick_params(labelsize=8)
    style(ax, 'Efficiency vs Power Density  (SiC 3-phase VSI)',
          'Power Density (kW/L)', 'Efficiency (%)')
    ax.set_xlim(left=80)
    fig.tight_layout()
    return fig


def style(ax, title='', xlabel='', ylabel=''):
    ax.set_title(title, fontsize=10, fontweight='bold', pad=7)
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.tick_params(labelsize=8)
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.set_axisbelow(True)



# ═══════════════════════════════════════════════════════════════════════════════
#  WINDOW 2 – Efficiency vs Output Power  (multi-frequency lines)
# ═══════════════════════════════════════════════════════════════════════════════

def plot_efficiency_vs_power(grid):
    fig, ax = plt.subplots(figsize=(9, 5))
    fig.canvas.manager.set_window_title('Window 2 – Efficiency vs Output Power')

    norm   = Normalize(vmin=FREQ_HZ.min()/1e3, vmax=FREQ_HZ.max()/1e3)
    dashes = ['-', '--', '-.', (0,(5,2)), (0,(3,1,1,1))]

    for k, f in enumerate(FREQ_LINES):
        j   = np.argmin(np.abs(FREQ_HZ - f))
        eff = grid['eff'][:, j]
        ax.plot(POWER_KW, eff,
                color=CMAP(norm(f/1e3)), linewidth=2,
                linestyle=dashes[k], label=f'{f/1e3:.0f} kHz')

    style(ax, 'Efficiency vs Output Power', 'Output Power (kW)', 'Efficiency (%)')
    ax.legend(fontsize=8, framealpha=0.8, ncol=5,
              loc='lower center', bbox_to_anchor=(0.5, -0.18))
    ax.set_xlim(POWER_KW[0], POWER_KW[-1])
    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  WINDOW 3 – Efficiency heatmap
# ═══════════════════════════════════════════════════════════════════════════════

def plot_efficiency_heatmap(grid):
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.canvas.manager.set_window_title('Window 3 – Efficiency Heatmap')

    eff_masked = np.where(grid['feas'], grid['eff'], np.nan)
    im = ax.imshow(eff_masked.T, aspect='auto', origin='lower',
                   extent=[POWER_KW[0], POWER_KW[-1],
                           FREQ_HZ[0]/1e3, FREQ_HZ[-1]/1e3],
                   cmap='RdYlGn',
                   vmin=np.nanmin(eff_masked), vmax=np.nanmax(eff_masked))
    cb = fig.colorbar(im, ax=ax, pad=0.02)
    cb.set_label('Efficiency (%)', fontsize=9)

    # thermal boundary contour
    ax.contour(POWER_KW, FREQ_HZ/1e3, grid['feas'].T.astype(float),
               levels=[0.5], colors=['navy'], linewidths=1.8, linestyles='--')

    style(ax, 'Efficiency Map  (Trise = 60C)',
          'Output Power (kW)', 'Switching Frequency (kHz)')
    fig.tight_layout()
    return fig


def plot_power_density_heatmap(grid):
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.canvas.manager.set_window_title('Power Density Heatmap')

    pd_masked = np.where(grid['feas'], grid['pd'], np.nan)
    im = ax.imshow(pd_masked.T, aspect='auto', origin='lower',
                   extent=[POWER_KW[0], POWER_KW[-1],
                           FREQ_HZ[0]/1e3, FREQ_HZ[-1]/1e3],
                   cmap='viridis',
                   vmin=np.nanmin(pd_masked), vmax=np.nanmax(pd_masked))
    cb = fig.colorbar(im, ax=ax, pad=0.02)
    cb.set_label('Power Density (kW/L)', fontsize=9)

    # thermal boundary contour
    ax.contour(POWER_KW, FREQ_HZ/1e3, grid['feas'].T.astype(float),
               levels=[0.5], colors=['navy'], linewidths=1.8, linestyles='--')

    style(ax, 'Power Density (kW/L) (Trise = 60C)',
          'Output Power (kW)', 'Switching Frequency (kHz)')
    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  WINDOW 4 – Power density, capacitance, volume vs switching frequency
# ═══════════════════════════════════════════════════════════════════════════════

def plot_freq_sweeps(grid):
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.canvas.manager.set_window_title('Window 4 – Frequency Sweep Metrics')

    freq_khz = FREQ_HZ / 1e3
    pw_levels = [50, 100, 150, 200]
    norm   = Normalize(vmin=min(pw_levels), vmax=max(pw_levels))
    cmap_p = plt.cm.plasma

    for pw in pw_levels:
        pi  = np.argmin(np.abs(POWER_KW - pw))
        c   = cmap_p(norm(pw))
        lbl = f'{pw} kW'
        axes[0].plot(freq_khz, grid['pd'][pi, :],  color=c, linewidth=1.8, label=lbl)
        axes[1].plot(freq_khz, grid['cap'][pi, :], color=c, linewidth=1.8, label=lbl)
        axes[2].plot(freq_khz, grid['vol'][pi, :], color=c, linewidth=1.8, label=lbl)

    style(axes[0], 'Power Density vs f_sw',      'f_sw (kHz)', 'Power Density (kW/L)')
    style(axes[1], 'DC-Link Capacitance vs f_sw', 'f_sw (kHz)', 'Capacitance (µF)')
    style(axes[2], 'Inverter Volume vs f_sw',     'f_sw (kHz)', 'Volume (L)')
    for ax in axes:
        ax.legend(fontsize=7, framealpha=0.8)
        ax.set_xlim(freq_khz[0], freq_khz[-1])

    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  WINDOW 5 – Loss breakdown
# ═══════════════════════════════════════════════════════════════════════════════

def plot_loss_breakdown(grid):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.canvas.manager.set_window_title('Window 5 – Loss Breakdown')

    freq_khz = FREQ_HZ / 1e3
    pi_100   = np.argmin(np.abs(POWER_KW - 100))

    # ── Left: loss components vs frequency @ 100 kW ───────────────────────────
    # Recompute deadtime separately (not stored on grid at init — add it)
    axes[0].plot(freq_khz, grid['pcond'][pi_100, :],
                 color='#378ADD', linewidth=2, label='Conduction')
    axes[0].plot(freq_khz, grid['psw'][pi_100, :],
                 color='#D85A30', linewidth=2, linestyle='--', label='Switching')
    axes[0].plot(freq_khz, grid['pdt'][pi_100, :],
                 color='#1D9E75', linewidth=2, linestyle='-.', label='Dead-time')
    axes[0].plot(freq_khz, grid['pcoss'][pi_100, :],
                 color="#9E1D7A", linewidth=2, linestyle='-.', label='Coss-loss ')
    total_loss = (grid['pcond'][pi_100, :] + grid['psw'][pi_100, :]
                  + grid['pdt'][pi_100, :] + P_GATE)
    axes[0].plot(freq_khz, total_loss,
                 color='#2C2C2A', linewidth=2, linestyle=':', label='Total')
    axes[0].axvline(20, color='gray', linewidth=0.9, linestyle=':', alpha=0.7)
    axes[0].legend(fontsize=8)
    style(axes[0], 'Loss Components vs f_sw  @ 100 kW',
          'f_sw (kHz)', 'Power Loss (W)')

    # ── Right: stacked bar @ 20 kHz across power levels ───────────────────────
    j20     = np.argmin(np.abs(FREQ_HZ - 20e3))
    pw_vals = POWER_KW[::5]
    pc_v    = grid['pcond'][::5, j20]
    ps_v    = grid['psw'][::5,   j20]
    pd_v    = grid['pdt'][::5,   j20]
    pcoss_v    = grid['pcoss'][::5,   j20]

    x       = np.arange(len(pw_vals))

    axes[1].bar(x, pc_v, color='#378ADD', alpha=0.85, label='Conduction')
    axes[1].bar(x, ps_v, bottom=pc_v, color='#D85A30', alpha=0.85, label='Switching')
    axes[1].bar(x, pd_v, bottom=pc_v + ps_v, color='#1D9E75', alpha=0.85, label='Dead-time')
    axes[1].bar(x, pcoss_v, bottom=pc_v + ps_v + pd_v, color="#821D9E", alpha=0.85, label='Coss')

    axes[1].set_xticks(x[::2])
    axes[1].set_xticklabels([f'{v:.0f}' for v in pw_vals[::2]], fontsize=7)
    axes[1].legend(fontsize=8)
    style(axes[1], 'Loss Split vs Output Power  @ 20 kHz',
          'Output Power (kW)', 'Power Loss (W)')

    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  Console summary
# ═══════════════════════════════════════════════════════════════════════════════

def print_summary(power_kw=100.0, fsw_khz=20.0):
    Pout  = power_kw * 1e3
    F_SW  = fsw_khz * 1e3
    I_rms = phase_current_from_power(Pout)
    L     = compute_losses(V_PH_RMS, I_rms, F_SW)
    eff   = 100.0 * Pout / (Pout + L['total'])
    C     = dc_link_cap(Pout, F_SW)
    V     = inverter_volume(C,L)
    pd    = (Pout / 1e3) / V
    T_j   = T_AMB + (L['total'] / 6.0) * R_TH
    m     = np.clip(V_PH_RMS * np.sqrt(2) / (V_DC / np.sqrt(3)), 0, 1)

    print("=" * 62)
    print("  SiC VSI  ·  Operating Point Summary")
    print("=" * 62)
    rows = [
        ('Output power',      f'{power_kw:.1f} kW'),
        ('Switching freq',    f'{fsw_khz:.1f} kHz'),
        ('DC bus voltage',    f'{V_DC:.0f} V'),
        ('V_ph_rms (fixed)',  f'{V_PH_RMS:.1f} V'),
        ('Power factor',      f'{PF:.2f}'),
        ('Modulation index',  f'{m:.3f}'),
        ('R_DS(on)',          f'{R_DS_ON*1e3:.1f} mΩ'),
        ('V_F0',              f'{V_F0:.2f} V'),
        ('T_rise / T_fall',   f'{T_RISE*1e9:.0f} / {T_FALL*1e9:.0f} ns'),
        ('Dead time',         f'{T_DEAD*1e9:.0f} ns'),
        ('-', '-'),
        ('Phase RMS current', f'{I_rms:.1f} A'),
        ('Conduction loss',   f'{L["cond"]:.1f} W  ({100*L["cond"]/L["total"]:.1f}%)'),
        ('Switching loss',    f'{L["sw"]:.1f} W  ({100*L["sw"]/L["total"]:.1f}%)'),
        ('Dead-time loss',    f'{L["deadtime"]:.2f} W  ({100*L["deadtime"]/L["total"]:.1f}%)'),
        ('Gate drive loss',   f'{L["gate"]:.2f} W  ({100*L["gate"]/L["total"]:.1f}%)'),
        ('Total loss',        f'{L["total"]:.1f} W'),
        ('Efficiency',        f'{eff:.4f} %'),
        ('Junction temp',     f'{T_j:.1f} °C  (limit {T_MAX:.0f} °C)'),
        ('DC-link cap',       f'{C*1e6:.1f} µF'),
        ('Inverter volume',   f'{V:.3f} L'),
        ('Power density',     f'{pd:.2f} kW/L'),
    ]
    for k, v in rows:
        if k == '-':
            print('-' * 62)
        else:
            print(f'  {k:<24} {v}')
    print("=" * 62)


# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print_summary()

    print('\nComputing grid... ', end='', flush=True)
    grid = compute_grid()
    print('done.')

    plot_pareto(grid)
    plot_efficiency_vs_power(grid)
    plot_efficiency_heatmap(grid)
    plot_freq_sweeps(grid)
    plot_loss_breakdown(grid)
    plot_power_density_heatmap(grid)

    print('\n5 windows opened. Close any window to continue.')
    plt.show()
