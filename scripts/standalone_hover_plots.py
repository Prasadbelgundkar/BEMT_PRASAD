"""
scripts/standalone_hover_plots.py
----------------------------------
Hover performance maps for the tiltrotor design defined in parameters.py.
  R=3.8 m, B=3, sigma~0.148, 550 RPM hover, MTOW=7200 kg (2 rotors)

CHECKLIST COVERAGE
------------------
  [1] Thrust vs Collective           --> hover_map_thrust.png
                                         (+ power-limited and stall-limited regions shaded)
  [2] Torque vs Collective           --> hover_map_power_torque.png
  [3] Power vs Collective            --> hover_map_power_torque.png
                                         (+ power-limited region shaded)
  [4] Blade AOA vs Collective        --> hover_map_aoa.png  (NEW)
  [5] Sea-level & high-altitude      --> all plots show SL + 3000 m
  [6] Stall-limited region           --> shaded on thrust + power plots; stall fraction plot
  [7] Power-limited region           --> shaded on thrust + power plots
  [8] Hover ceiling / max hover wt   --> hover_ceiling.png  (NEW)
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, '..', 'src'))
sys.path.insert(0, _SRC_DIR)
_OUT_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, '..', 'outputs'))

from environment import isa
from bemt import run_bemt
import parameters as p

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
RHO_SL    = 1.225     # kg/m^3
LAPSE_EXP = 0.9       # turboshaft density-ratio exponent
ETA_DT    = 0.97      # drivetrain efficiency

HOVER_RPM   = 550.0
HOVER_OMEGA = HOVER_RPM * 2.0 * np.pi / 60.0   # rad/s
NUM_ROTORS  = p.NUM_ENGINES
G           = 9.80665

MTOW_KG  = p.EMPTY_MASS_KG + p.PAYLOAD_MASS_KG + p.FUEL_MASS_KG   # 7200 kg
T_REQ_N  = MTOW_KG * G / NUM_ROTORS
T_REQ_kN = T_REQ_N / 1e3

HOVER_STALL_LIMIT_PCT = 10.0          # % blade span; 10% is standard hover limit
MIN_POWER_MARGIN_PCT  = 5.0           # % power margin required

COLLS_DEG = np.linspace(10.0, 30.0, 60)
COLLS_RAD = np.radians(COLLS_DEG)

ALTITUDES  = [0.0, 3000.0]
ALT_LABELS = ['Sea Level (ISA+0)', '3000 m (ISA+0)']
COLORS     = ['royalblue', 'tomato']

# Spanwise stations to track AOA (non-dimensional r/R)
AOA_STATIONS    = [0.20, 0.40, 0.60, 0.80, 0.95]
AOA_STATION_LBL = ['r/R=0.20 (root)', 'r/R=0.40', 'r/R=0.60 (mid)',
                   'r/R=0.80', 'r/R=0.95 (tip)']
AOA_COLORS      = ['#d62728', '#ff7f0e', '#2ca02c', '#9467bd', '#1f77b4']


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _power_available_W(rho: float) -> float:
    """Turboshaft power available: P = P_SL*(rho/rho_SL)^0.9 * eta_dt."""
    return p.ENGINE_POWER_W * (rho / RHO_SL) ** LAPSE_EXP * ETA_DT


def _rotor_info_str() -> str:
    rotor = p.get_configured_rotor()
    sigma = rotor.solidity()
    Vtip  = HOVER_OMEGA * p.RADIUS_M
    Mtip  = Vtip / 340.29
    return (f"R={p.RADIUS_M} m, B={p.NUM_BLADES}, sigma={sigma:.4f}, "
            f"{HOVER_RPM:.0f} RPM  (Vtip={Vtip:.1f} m/s, M_tip={Mtip:.3f})")


def _save(fig, filename: str):
    path = os.path.join(_OUT_DIR, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
def _run_sweep(altitude_m: float) -> dict:
    """
    Run BEMT over the full collective range at one altitude.
    Returns per-collective arrays including spanwise AOA at key stations.
    """
    rotor      = p.get_configured_rotor()
    atmo       = isa(altitude_m, p.DISA_K)
    P_avail_kW = _power_available_W(atmo.density_kg_m3) / 1e3

    thrust_kN, power_kW, torque_kNm, fm, stall_pct = [], [], [], [], []
    aoa_data = {x: [] for x in AOA_STATIONS}   # alpha_deg at each r/R station

    for c_rad in COLLS_RAD:
        perf = run_bemt(
            rotor, p.AIRFOIL_PROVIDER, HOVER_OMEGA, c_rad,
            atmo.density_kg_m3, atmo.speed_of_sound_mps, v_axial=0.0,
        )
        thrust_kN .append(perf.thrust_N / 1e3)
        power_kW  .append(perf.power_W  / 1e3)
        torque_kNm.append((perf.power_W / HOVER_OMEGA) / 1e3)
        fm        .append(perf.figure_of_merit if perf.figure_of_merit else float('nan'))
        stall_pct .append(perf.stalled_fraction * 100.0)

        # Spanwise AOA at key stations (interpolate from element results)
        el_x     = np.array([e.x         for e in perf.elements])
        el_alpha = np.degrees(np.array([e.alpha_rad for e in perf.elements]))
        for tx in AOA_STATIONS:
            idx = int(np.argmin(np.abs(el_x - tx)))
            aoa_data[tx].append(el_alpha[idx])

    return {
        'thrust_kN'  : np.array(thrust_kN),
        'power_kW'   : np.array(power_kW),
        'torque_kNm' : np.array(torque_kNm),
        'fm'         : np.array(fm),
        'stall_pct'  : np.array(stall_pct),
        'P_avail_kW' : P_avail_kW,
        'aoa'        : {x: np.array(v) for x, v in aoa_data.items()},
    }


# ---------------------------------------------------------------------------
def _compute_hover_ceiling(n_alt: int = 20) -> tuple:
    """
    Sweep altitude from SL to 6000 m. At each altitude find:
      - Power-limited max hover weight (P_req <= P_avail with 5% margin)
      - Stall-limited max hover weight (stall_frac <= 10%)
    Returns (altitudes_m, mass_power_kg, mass_stall_kg).
    """
    altitudes  = np.linspace(0.0, 6000.0, n_alt)
    mass_power = []
    mass_stall = []

    for h in altitudes:
        sweep      = _run_sweep(h)
        T_arr      = sweep['thrust_kN']      # kN
        P_arr      = sweep['power_kW']       # kW
        S_arr      = sweep['stall_pct']      # %
        P_avail    = sweep['P_avail_kW']     # kW

        # Power-limited: P_req must not exceed P_avail*(1 - margin)
        P_limit    = P_avail * (1.0 - MIN_POWER_MARGIN_PCT / 100.0)
        power_ok   = P_arr <= P_limit
        stall_ok   = S_arr <= HOVER_STALL_LIMIT_PCT

        T_p = float(T_arr[power_ok].max()) if power_ok.any() else float('nan')
        T_s = float(T_arr[stall_ok].max()) if stall_ok.any() else float('nan')

        # Convert per-rotor thrust (kN) to total MTOW (kg)
        mass_power.append(T_p * 1e3 * NUM_ROTORS / G if not np.isnan(T_p) else float('nan'))
        mass_stall.append(T_s * 1e3 * NUM_ROTORS / G if not np.isnan(T_s) else float('nan'))

    return altitudes, np.array(mass_power), np.array(mass_stall)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def generate_hover_maps():
    os.makedirs(_OUT_DIR, exist_ok=True)
    plt.style.use('ggplot')
    info = _rotor_info_str()

    # ---- get stall angle from airfoil ----
    _af = p.AIRFOIL_PROVIDER(0.5)
    alpha_stall_deg = np.degrees(_af.stall_alpha_rad) if hasattr(_af, 'stall_alpha_rad') else 15.0

    # ---- collective sweeps ----
    print("Calculating hover performance maps ...")
    results = {}
    for alt, lbl in zip(ALTITUDES, ALT_LABELS):
        print(f"  Sweeping collective at h = {alt:.0f} m ...")
        results[alt] = _run_sweep(alt)

    # ==================================================================
    # PLOT 1 – Thrust vs Collective
    #          Checklist: [1] Thrust, [5] SL+altitude, [6] stall-limited,
    #                     [7] power-limited
    # ==================================================================
    fig1, ax1 = plt.subplots(figsize=(9, 6))

    # Shade stall-limited region (use SL result; 3000m nearly identical)
    r0      = results[ALTITUDES[0]]
    stall_mask = r0['stall_pct'] > HOVER_STALL_LIMIT_PCT
    if stall_mask.any():
        ax1.axvspan(COLLS_DEG[stall_mask][0], COLLS_DEG[-1],
                    color='orange', alpha=0.18, zorder=0, label='Stall-limited region')

    # Shade power-limited region for each altitude (hatched)
    shade_labels_done = set()
    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        r         = results[alt]
        plim_mask = r['power_kW'] > r['P_avail_kW']
        if plim_mask.any():
            x0 = COLLS_DEG[plim_mask][0]
            patch_lbl = f'Power-limited ({lbl})'
            ax1.axvspan(x0, COLLS_DEG[-1],
                        color=color, alpha=0.12, zorder=0, label=patch_lbl)

    # Thrust curves
    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        ax1.plot(COLLS_DEG, results[alt]['thrust_kN'],
                 color=color, linewidth=2, label=lbl)

    # Hover requirement line
    ax1.axhline(T_REQ_kN, color='green', linestyle='--', linewidth=1.8,
                label=f'Hover req  MTOW={MTOW_KG:.0f} kg  ({T_REQ_kN:.1f} kN/rotor)')

    ax1.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
    ax1.set_ylabel('Thrust per Rotor (kN)', fontweight='bold')
    ax1.set_title(f'Thrust vs Collective\n{info}', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=8, loc='upper left')
    ax1.grid(True, alpha=0.4)
    _save(fig1, 'hover_map_thrust.png')

    # ==================================================================
    # PLOT 2 – Torque & Power vs Collective
    #          Checklist: [2] Torque, [3] Power, [5] SL+altitude,
    #                     [7] power-limited region on power panel
    # ==================================================================
    fig2, (ax2a, ax2b) = plt.subplots(1, 2, figsize=(14, 6))

    # Torque
    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        ax2a.plot(COLLS_DEG, results[alt]['torque_kNm'],
                  color=color, label=lbl, linewidth=2)

    ax2a.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
    ax2a.set_ylabel('Torque per Rotor (kNm)', fontweight='bold')
    ax2a.set_title('Torque vs Collective', fontsize=12, fontweight='bold')
    ax2a.legend(fontsize=9); ax2a.grid(True, alpha=0.4)

    # Power – shade power-limited regions first (background)
    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        r         = results[alt]
        plim_mask = r['power_kW'] > r['P_avail_kW']
        if plim_mask.any():
            ax2b.axvspan(COLLS_DEG[plim_mask][0], COLLS_DEG[-1],
                         color=color, alpha=0.12, zorder=0,
                         label=f'Power-limited ({lbl})')

    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        r        = results[alt]
        P_avail  = r['P_avail_kW']
        ax2b.plot(COLLS_DEG, r['power_kW'],
                  color=color, label=lbl, linewidth=2)
        ax2b.axhline(P_avail, color=color, linestyle='--', alpha=0.80, linewidth=1.6,
                     label=f'P avail – {lbl}  ({P_avail:.0f} kW)')

    p_max = max(r['P_avail_kW'] for r in results.values())
    q_max = max(r['power_kW'].max() for r in results.values())
    ax2b.set_ylim(0, max(p_max * 1.25, q_max * 1.10))
    ax2b.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
    ax2b.set_ylabel('Power per Rotor (kW)', fontweight='bold')
    ax2b.set_title('Power vs Collective', fontsize=12, fontweight='bold')
    ax2b.legend(fontsize=8); ax2b.grid(True, alpha=0.4)

    fig2.suptitle(
        f'Power & Torque vs Collective  |  {info}\n'
        f'P_avail = P_SL x (rho/rho0)^{LAPSE_EXP} x eta_dt={ETA_DT}  (turboshaft lapse)',
        fontsize=10, fontweight='bold',
    )
    _save(fig2, 'hover_map_power_torque.png')

    # ==================================================================
    # PLOT 3 – Stall Fraction vs Collective
    #          Checklist: [6] stall-limited region identified
    # ==================================================================
    fig3, ax3 = plt.subplots(figsize=(8, 6))

    # Shade stall-limited zone
    ax3.axhspan(HOVER_STALL_LIMIT_PCT, 100.0,
                color='orange', alpha=0.15, zorder=0, label='Stall-limited zone')

    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        ax3.plot(COLLS_DEG, results[alt]['stall_pct'],
                 color=color, label=lbl, linewidth=2)

    ax3.axhline(HOVER_STALL_LIMIT_PCT, color='red', linestyle='--', linewidth=1.8,
                label=f'Hover stall limit ({HOVER_STALL_LIMIT_PCT:.0f}%)')

    ax3.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
    ax3.set_ylabel('Blade Stalled Fraction (%)', fontweight='bold')
    ax3.set_title(
        f'Stall Fraction vs Collective\n'
        f'alpha_stall ~ {alpha_stall_deg:.1f} deg  |  '
        f'Twist: {p.TWIST_ROOT_DEG:.0f} deg root / '
        f'{p.TWIST_ROOT_DEG + p.TWIST_RATE_DEG:.0f} deg tip',
        fontsize=12, fontweight='bold',
    )
    ax3.set_ylim(bottom=0)
    ax3.legend(fontsize=9); ax3.grid(True, alpha=0.4)
    _save(fig3, 'hover_map_stall.png')

    # ==================================================================
    # PLOT 4 – Figure of Merit vs Collective
    # ==================================================================
    fig4, ax4 = plt.subplots(figsize=(8, 6))
    for alt, color, lbl in zip(ALTITUDES, COLORS, ALT_LABELS):
        ax4.plot(COLLS_DEG, results[alt]['fm'],
                 color=color, label=lbl, linewidth=2)

    ax4.axhline(0.75, color='green',  linestyle='--', linewidth=1.5,
                label='FM = 0.75  (good rotor benchmark)')
    ax4.axhline(0.65, color='orange', linestyle=':',  linewidth=1.5,
                label='FM = 0.65  (lower acceptable bound)')

    ax4.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
    ax4.set_ylabel('Figure of Merit  FM = CT^1.5 / (sqrt(2) x CP)', fontweight='bold')
    ax4.set_ylim(0.0, 1.0)
    ax4.set_title(f'Figure of Merit vs Collective\n{info}',
                  fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9); ax4.grid(True, alpha=0.4)
    _save(fig4, 'hover_map_FM.png')

    # ==================================================================
    # PLOT 5 – Blade AOA vs Collective  (NEW – Checklist item 4)
    #          Shows local angle-of-attack at root/mid/tip vs collective
    #          for SL and 3000 m, with alpha_stall reference.
    # ==================================================================
    fig5, axes5 = plt.subplots(1, 2, figsize=(14, 6), sharey=True)

    for ax5, alt, alt_lbl in zip(axes5, ALTITUDES, ALT_LABELS):
        r = results[alt]

        # Shade stall region
        ax5.axhspan(alpha_stall_deg, alpha_stall_deg + 15,
                    color='red', alpha=0.10, zorder=0)

        # Plot AOA for each station
        for tx, tlbl, tcol in zip(AOA_STATIONS, AOA_STATION_LBL, AOA_COLORS):
            ax5.plot(COLLS_DEG, r['aoa'][tx],
                     color=tcol, label=tlbl, linewidth=2)

        # Alpha stall line
        ax5.axhline(alpha_stall_deg, color='red', linestyle='--', linewidth=1.8,
                    label=f'alpha_stall = {alpha_stall_deg:.1f} deg')
        # alpha = 0 reference
        ax5.axhline(0.0, color='grey', linestyle=':', linewidth=1.0)

        ax5.set_xlabel('Root Collective Pitch (deg)', fontweight='bold')
        ax5.set_ylabel('Local Angle of Attack (deg)', fontweight='bold')
        ax5.set_title(f'Blade AOA vs Collective\n{alt_lbl}',
                      fontsize=12, fontweight='bold')
        ax5.legend(fontsize=8, loc='upper left')
        ax5.grid(True, alpha=0.4)

    fig5.suptitle(
        f'Blade Angle of Attack at Key Radial Stations  |  {info}\n'
        f'Root has highest AOA (low tangential speed); tip has lowest (Prandtl tip loss + high U_T)',
        fontsize=10, fontweight='bold',
    )
    plt.tight_layout()
    _save(fig5, 'hover_map_aoa.png')

    # ==================================================================
    # PLOT 6 – Max Hover Weight vs Altitude  (NEW – Checklist item 8)
    #          Power-limited and stall-limited envelopes
    # ==================================================================
    print("\n  Computing hover ceiling envelope (sweeping 20 altitudes) ...")
    alt_arr, mass_pwr, mass_stl = _compute_hover_ceiling(n_alt=20)

    fig6, ax6 = plt.subplots(figsize=(9, 6))

    ax6.plot(alt_arr / 1000, mass_pwr / 1000, 'r-o', ms=5, lw=2,
             label='Power-limited max MTOW')
    ax6.plot(alt_arr / 1000, mass_stl / 1000, 'b--s', ms=5, lw=2,
             label='Stall-limited max MTOW')

    # Actual MTOW line
    ax6.axhline(MTOW_KG / 1000, color='green', linestyle=':', linewidth=1.8,
                label=f'Design MTOW = {MTOW_KG:.0f} kg')

    # Shade: where power curve < MTOW → aircraft cannot hover at MTOW
    ax6.fill_betweenx([0, MTOW_KG / 1000 * 1.2],
                      alt_arr[~np.isnan(mass_pwr) & (mass_pwr < MTOW_KG)][-1] / 1000
                      if (~np.isnan(mass_pwr) & (mass_pwr < MTOW_KG)).any()
                      else alt_arr[-1] / 1000,
                      alt_arr[-1] / 1000,
                      color='red', alpha=0.10, zorder=0, label='Cannot hover at MTOW')

    ax6.set_xlabel('Altitude (km)', fontweight='bold')
    ax6.set_ylabel('Max Hover Gross Weight (tonnes)', fontweight='bold')
    ax6.set_title(
        f'Hover Ceiling Envelope  |  {info}\n'
        f'Power margin >= {MIN_POWER_MARGIN_PCT:.0f}%  |  '
        f'Stall limit {HOVER_STALL_LIMIT_PCT:.0f}%  |  '
        f'Collective range 10-30 deg',
        fontsize=11, fontweight='bold',
    )
    ax6.legend(fontsize=9)
    ax6.grid(True, alpha=0.4)
    _save(fig6, 'hover_ceiling.png')

    print("\nAll hover plots saved successfully.")
    _summarise(results)


# ---------------------------------------------------------------------------
def _summarise(results: dict):
    """Print trim-point numerical summary for both altitudes."""
    from bemt import trim_hover_collective
    rotor = p.get_configured_rotor()

    print("\n--- Numerical Summary -------------------------------------------")
    print(f"  MTOW         : {MTOW_KG:.0f} kg "
          f"(OEW {p.EMPTY_MASS_KG:.0f} + PL {p.PAYLOAD_MASS_KG:.0f} "
          f"+ Fuel {p.FUEL_MASS_KG:.0f})")
    print(f"  T_req/rotor  : {T_REQ_N:.1f} N  ({T_REQ_kN:.2f} kN)")
    print(f"  Rotor        : R={p.RADIUS_M} m, B={p.NUM_BLADES}, "
          f"sigma={rotor.solidity():.4f}")
    print(f"  Hover RPM    : {HOVER_RPM:.0f}  ->  "
          f"Vtip = {HOVER_OMEGA*p.RADIUS_M:.2f} m/s  "
          f"(M_tip = {HOVER_OMEGA*p.RADIUS_M/340.29:.3f})")

    for alt, lbl in zip(ALTITUDES, ALT_LABELS):
        atmo       = isa(alt, p.DISA_K)
        P_avail_kW = _power_available_W(atmo.density_kg_m3) / 1e3
        coll_trim  = trim_hover_collective(
            rotor, p.AIRFOIL_PROVIDER, HOVER_OMEGA, T_REQ_N,
            atmo.density_kg_m3, atmo.speed_of_sound_mps,
            coll_range_deg=(10.0, 30.0),
        )
        print(f"\n  [{lbl}]")
        if coll_trim is not None:
            perf   = run_bemt(rotor, p.AIRFOIL_PROVIDER, HOVER_OMEGA,
                              np.radians(coll_trim),
                              atmo.density_kg_m3, atmo.speed_of_sound_mps)
            fm_str = f"{perf.figure_of_merit:.3f}" if perf.figure_of_merit else "N/A"
            margin = (P_avail_kW - perf.power_W / 1e3) / P_avail_kW * 100.0
            print(f"    Trim collective  : {coll_trim:.2f} deg")
            print(f"    Power required   : {perf.power_W/1e3:.1f} kW  "
                  f"| available {P_avail_kW:.1f} kW  | margin {margin:.1f}%")
            print(f"    Figure of Merit  : {fm_str}")
            print(f"    Stall fraction   : {perf.stalled_fraction*100:.1f}%")
        else:
            print(f"    Could not trim at MTOW in collective range 10-30 deg!")
    print("-----------------------------------------------------------------")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    generate_hover_maps()
