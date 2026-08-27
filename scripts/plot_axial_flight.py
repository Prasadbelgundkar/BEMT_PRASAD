# -*- coding: utf-8 -*-
"""
scripts/plot_axial_flight.py
-----------------------------
Axial Forward-Flight (Propeller Mode) Assessment using parameters.py.

CHECKLIST COVERAGE:
  [x] At least four advance ratios J
  [x] Several collective settings
  [x] Thrust coefficient (CT vs J)
  [x] Power coefficient (CP vs J)
  [x] Propulsive efficiency (eta vs J)
  [x] AOA distribution (at Design Cruise Point)
  [x] Feasible operating envelope (Shaded boundary limiting Stall & Mach)
  [x] Suitable cruise point
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from environment import isa
from bemt import run_bemt, advance_ratio
import parameters as p

_OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'outputs'))
os.makedirs(_OUT_DIR, exist_ok=True)

ALTITUDE = p.CRUISE_ALTITUDE_AMSL_M
CRUISE_RPM = 250.0
CRUISE_OMEGA = CRUISE_RPM * 2.0 * np.pi / 60.0
CRUISE_V = 74.3
CRUISE_COLL = 56.5
RADIUS = p.RADIUS_M
ATMO = isa(ALTITUDE, p.DISA_K)

V_VALS = np.linspace(10, 130, 30)
COLL_SETTINGS = [40, 45, 50, 55, 60, 65, 70]
COLORS = ["deepskyblue", "royalblue", "mediumseagreen", "goldenrod", "tomato", "purple", "brown"]

def generate_axial_plots():
    print("Calculating axial flight performance (Propeller Mode)...")
    rotor = p.get_configured_rotor()

    # =====================================================================
    # Plot 1: CT, CP, and Efficiency vs J
    # =====================================================================
    fig1, axes = plt.subplots(1, 3, figsize=(16, 5))
    for coll_deg, col in zip(COLL_SETTINGS, COLORS):
        J_arr, CT_arr, CP_arr, eta_arr = [], [], [], []
        for V in V_VALS:
            try:
                perf = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA,
                                np.radians(coll_deg), ATMO.density_kg_m3,
                                ATMO.speed_of_sound_mps, v_axial=V)
                
                # Avoid plotting deep into negative CT
                if perf.CT >= -0.005: 
                    J_arr.append(advance_ratio(V, CRUISE_OMEGA, RADIUS))
                    CT_arr.append(perf.CT)
                    CP_arr.append(perf.CP if perf.CP > 0 else float('nan'))
                    eta_arr.append(perf.propulsive_efficiency if perf.propulsive_efficiency else 0.0)
            except Exception:
                pass
            
        axes[0].plot(J_arr, CT_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")
        axes[1].plot(J_arr, CP_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")
        axes[2].plot(J_arr, eta_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")

    perf_c = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA, np.radians(CRUISE_COLL),
                      ATMO.density_kg_m3, ATMO.speed_of_sound_mps, v_axial=CRUISE_V)
    J_c = advance_ratio(CRUISE_V, CRUISE_OMEGA, RADIUS)
    eta_c = perf_c.propulsive_efficiency if perf_c.propulsive_efficiency else 0.0

    axes[0].plot(J_c, perf_c.CT, 'k*', ms=10, label=f"Cruise Pt ({CRUISE_COLL}deg)")
    axes[1].plot(J_c, perf_c.CP, 'k*', ms=10)
    axes[2].plot(J_c, eta_c, 'k*', ms=10)

    for ax, ylabel, title in zip(axes,
                                 ["Thrust coeff $C_T$", "Power coeff $C_P$", "Propulsive eff. $\eta$"],
                                 ["$C_T$ vs $J$", "$C_P$ vs $J$", "$\eta$ vs $J$"]):
        ax.set_xlabel("Advance ratio $J$", fontweight='bold')
        ax.set_ylabel(ylabel, fontweight='bold')
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)
        
    axes[0].set_ylim(bottom=0) # Do not show negative CT
    axes[1].set_ylim(bottom=0)
    axes[2].set_ylim(0, 1.0)
    fig1.suptitle(f"Propeller Mode Performance | R={RADIUS}m, {CRUISE_RPM} RPM, h={ALTITUDE}m", fontweight="bold")
    plt.tight_layout()
    p1 = os.path.join(_OUT_DIR, "axial_flight_CT_CP_eta.png")
    fig1.savefig(p1, dpi=150)
    plt.close()
    print(f"  Saved: {p1}")

    # =====================================================================
    # Plot 2: AOA Distribution at Design Cruise Point
    # =====================================================================
    perf_c_det = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA, np.radians(CRUISE_COLL),
                      ATMO.density_kg_m3, ATMO.speed_of_sound_mps, v_axial=CRUISE_V, n_stations=60)
    r_R = np.array([e.x for e in perf_c_det.elements])
    alpha = np.degrees(np.array([e.alpha_rad for e in perf_c_det.elements]))
    _af = p.AIRFOIL_PROVIDER(0.5)
    stall_lim = np.degrees(_af.stall_alpha_rad) if hasattr(_af, 'stall_alpha_rad') else 15.0

    fig2, ax2 = plt.subplots(figsize=(8, 5))
    ax2.plot(r_R, alpha, "b-", lw=2, label="Local Angle of Attack")
    ax2.axhline(stall_lim, color="red", ls="--", lw=1.5, label=f"Stall Limit = {stall_lim:.1f}deg")
    ax2.axhline(-stall_lim, color="red", ls=":", lw=1.5, label="Negative Stall Limit")
    ax2.axhline(0.0, color="gray", lw=1.0)
    ax2.fill_between(r_R, alpha, stall_lim, where=alpha > stall_lim, color="red", alpha=0.3, label="Stalled")
    ax2.fill_between(r_R, alpha, -stall_lim, where=alpha < -stall_lim, color="red", alpha=0.3)
    ax2.set_xlabel("Non-dimensional radius r/R", fontweight='bold')
    ax2.set_ylabel("Angle of Attack (deg)", fontweight='bold')
    ax2.set_title(f"Blade AOA Distribution at Cruise Point\n"
                  f"V={CRUISE_V} m/s, J={J_c:.3f}, Coll={CRUISE_COLL}deg, RPM={CRUISE_RPM}, h={ALTITUDE}m", fontweight='bold')
    ax2.legend(); ax2.grid(True, alpha=0.4)
    plt.tight_layout()
    p2 = os.path.join(_OUT_DIR, "axial_flight_AoA_dist.png")
    fig2.savefig(p2, dpi=150)
    plt.close()
    print(f"  Saved: {p2}")

    # =====================================================================
    # Plot 3: Feasible Operating Envelope & Collective Limits
    # =====================================================================
    print("  Computing feasible envelope boundary ...")
    fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(10, 9), sharex=True)
    
    J_env = []
    CT_env_max = []
    coll_min_env = []
    coll_max_env = []
    
    # Compute the absolute envelope boundary by finding the maximum valid CT at each speed
    for V in np.linspace(10, 140, 12):
        J = advance_ratio(V, CRUISE_OMEGA, RADIUS)
        max_ct = 0.0
        c_min = None
        c_max = None
        
        # Sweep collective to find max feasible CT
        for c in np.linspace(10, 80, 36):
            try:
                perf = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA, np.radians(c),
                                ATMO.density_kg_m3, ATMO.speed_of_sound_mps, v_axial=V)
                if perf.CT > 0:
                    # Check limits
                    if perf.stalled_fraction <= p.MAX_STALL_FRACTION and perf.max_tip_mach <= p.MAX_TIP_MACH:
                        if c_min is None:
                            c_min = c
                        c_max = c
                        if perf.CT > max_ct:
                            max_ct = perf.CT
            except Exception:
                pass
                
        if max_ct > 0 and c_min is not None and c_max is not None:
            J_env.append(J)
            CT_env_max.append(max_ct)
            coll_min_env.append(c_min)
            coll_max_env.append(c_max)
    print("Done computing boundary")
            
    # --- Top Subplot: Thrust Envelope ---
    ax3a.fill_between(J_env, 0, CT_env_max, color='mediumseagreen', alpha=0.3, label='Feasible Thrust Region')
    ax3a.plot(J_env, CT_env_max, 'g-', lw=2.5, label='Max Allowable Thrust')
    
    is_c_feas = (perf_c.stalled_fraction <= p.MAX_STALL_FRACTION) and (perf_c.max_tip_mach <= p.MAX_TIP_MACH) and (perf_c.CT > 0)
    marker = 'k*' if is_c_feas else 'rx'
    ax3a.plot(J_c, perf_c.CT, marker, ms=14, label=f"Cruise Point (J={J_c:.2f}, $C_T$={perf_c.CT:.4f})")
    
    ax3a.set_ylabel("Thrust Coefficient $C_T$", fontweight='bold')
    ax3a.set_ylim(bottom=0)
    ax3a.grid(True, alpha=0.4)
    ax3a.legend(loc='lower center')
    ax3a.set_title("1. Aerodynamic Thrust Limit", fontweight='bold')
    
    # --- Bottom Subplot: Collective Envelope ---
    ax3b.fill_between(J_env, coll_min_env, coll_max_env, color='cornflowerblue', alpha=0.3, label='Safe Operating Range')
    ax3b.plot(J_env, coll_max_env, 'b--', lw=2, label='Upper Limit (Prevents Stall / High Tip Mach)')
    ax3b.plot(J_env, coll_min_env, 'b:', lw=2, label='Lower Limit (Prevents Windmilling)')
    
    ax3b.plot(J_c, CRUISE_COLL, marker, ms=14, label=f"Cruise Point ({CRUISE_COLL} deg)")
    
    ax3b.set_xlabel("Advance Ratio $J$", fontweight='bold')
    ax3b.set_ylabel("Collective Pitch (deg)", fontweight='bold')
    ax3b.grid(True, alpha=0.4)
    ax3b.legend(loc='lower right')
    ax3b.set_title("2. Pilot Collective Pitch Envelope", fontweight='bold')
    
    # Annotations
    notes = (
        f"Cruise Point Verification:\n"
        f"Advance Ratio J = {J_c:.3f}\n"
        f"Efficiency = {eta_c*100:.1f}%\n"
        f"Tip Mach = {perf_c.max_tip_mach:.3f} (Limit: 0.9)\n"
        f"Stall Frac = {perf_c.stalled_fraction*100:.1f}% (Limit: 40%)\n"
        f"STATUS: {'FEASIBLE' if is_c_feas else 'UNFEASIBLE'}"
    )
    props = dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray')
    ax3a.text(0.02, 0.93, notes, transform=ax3a.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    fig3.suptitle(f"Propeller Mode Operating Envelopes\n"
                  f"R={RADIUS}m, {CRUISE_RPM} RPM, h={ALTITUDE}m", fontweight='bold', fontsize=14)
    plt.tight_layout()
    p3 = os.path.join(_OUT_DIR, "axial_flight_feasible_envelope.png")
    fig3.savefig(p3, bbox_inches='tight', dpi=150)
    plt.close()
    print(f"  Saved: {p3}")

if __name__ == '__main__':
    generate_axial_plots()
