
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from environment import isa
from bemt import run_bemt, advance_ratio
import parameters as p

_OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'outputs'))

def generate_efficiency_map():
    print('Generating efficiency map...')
    rotor = p.get_configured_rotor()
    ATMO = isa(p.CRUISE_ALTITUDE_AMSL_M, p.DISA_K)
    CRUISE_OMEGA = 250.0 * 2.0 * np.pi / 60.0
    RADIUS = p.RADIUS_M

    V_vals = np.linspace(20, 150, 18)
    C_vals = np.linspace(20, 75, 25)
    
    J_list = []
    CT_list = []
    Eta_list = []

    for V in V_vals:
        J = advance_ratio(V, CRUISE_OMEGA, RADIUS)
        for c in C_vals:
            try:
                perf = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA, np.radians(c),
                                ATMO.density_kg_m3, ATMO.speed_of_sound_mps, v_axial=V)
                if perf.CT > 0 and perf.propulsive_efficiency is not None:
                    if perf.stalled_fraction <= 0.4 and perf.max_tip_mach <= p.MAX_TIP_MACH:
                        J_list.append(J)
                        CT_list.append(perf.CT)
                        Eta_list.append(perf.propulsive_efficiency)
            except Exception:
                pass

    fig, ax = plt.subplots(figsize=(10, 7))
    
    cntr = ax.tricontourf(J_list, CT_list, Eta_list, levels=np.linspace(0.4, 0.95, 12), cmap='viridis')
    cbar = fig.colorbar(cntr, ax=ax)
    cbar.set_label('Propulsive Efficiency $\eta$', fontweight='bold')
    
    lines = ax.tricontour(J_list, CT_list, Eta_list, levels=[0.5, 0.6, 0.7, 0.8, 0.85, 0.9], colors='k', linewidths=0.5, alpha=0.5)
    ax.clabel(lines, inline=True, fmt='%.2f', fontsize=9)

    CRUISE_V = 74.3
    J_c = advance_ratio(CRUISE_V, CRUISE_OMEGA, RADIUS)
    CRUISE_COLL = 56.5
    perf_c = run_bemt(rotor, p.AIRFOIL_PROVIDER, CRUISE_OMEGA, np.radians(CRUISE_COLL),
                      ATMO.density_kg_m3, ATMO.speed_of_sound_mps, v_axial=CRUISE_V)
    
    ax.plot(J_c, perf_c.CT, 'r*', ms=15, markeredgecolor='white', label=f'Design Cruise Point\n(J={J_c:.2f}, eta={perf_c.propulsive_efficiency*100:.1f}%)')

    ax.set_xlabel('Advance Ratio $J$', fontweight='bold')
    ax.set_ylabel('Thrust Coefficient $C_T$', fontweight='bold')
    ax.set_title('Propeller Efficiency Map ($\eta$)\n$R=3.8$m, $250$ RPM, $h=7000$m', fontweight='bold', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right')
    
    plt.tight_layout()
    out_path = os.path.join(_OUT_DIR, 'axial_efficiency_map.png')
    fig.savefig(out_path, dpi=150)
    print(f'Saved: {out_path}')

if __name__ == '__main__':
    generate_efficiency_map()
