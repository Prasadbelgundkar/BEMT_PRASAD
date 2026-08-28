import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, linear_twist, linear_taper_chord
from bemt import run_bemt

airfoil = LinearAirfoil()
atmo = isa(0.0)
OMEGA = 2 * np.pi * 1000.0 / 60.0
R = 0.762
rc = 0.125
B = 2
c_avg = 0.0508 # Baseline constant chord

def make_tapered_rotor(taper_ratio):
    # Calculate root chord required to maintain constant mean chord (constant solidity)
    # c_mean = c_root * [1 - 0.5 * (1 - TR) * (1 + rc/R)]
    factor = 1.0 - 0.5 * (1.0 - taper_ratio) * (1.0 + rc / R)
    c_root = c_avg / factor
    
    return Rotor(R, rc, B,
                 linear_taper_chord(c_root, taper_ratio),
                 linear_twist(0.0, 0.0))

def main():
    taper_ratios = [0.4, 0.6, 0.8, 1.0] # tip_chord / root_chord
    collectives = np.linspace(2, 16, 15)
    
    outdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(outdir, exist_ok=True)
    
    fig1, ax1 = plt.subplots(figsize=(6, 4))
    fig2, ax2 = plt.subplots(figsize=(6, 4))
    
    for tr in taper_ratios:
        rotor = make_tapered_rotor(tr)
        
        T, P, Stall, FM = [], [], [], []
        for c in collectives:
            perf = run_bemt(rotor, lambda x: airfoil, OMEGA, np.radians(c),
                            atmo.density_kg_m3, atmo.speed_of_sound_mps)
            T.append(perf.thrust_N)
            P.append(perf.power_W / 1000.0) # kW
            Stall.append(perf.stalled_fraction)
            FM.append(perf.figure_of_merit if perf.figure_of_merit else 0)
            
        label = f"TR = {tr:.1f} ($\sigma$={rotor.solidity():.3f})"
        ax1.plot(collectives, T, lw=2, label=label)
        ax2.plot(collectives, P, lw=2, label=label)
        
        if 'fig3' not in locals():
            fig3, ax3 = plt.subplots(figsize=(6, 4))
        ax3.plot(collectives, FM, lw=2, label=label)
        
    ax1.set_xlabel("Collective [deg]")
    ax1.set_ylabel("Thrust [N]")
    ax1.set_title("Thrust vs Collective for Varying Taper Ratio")
    ax1.grid(True)
    ax1.legend()
    path1 = os.path.join(outdir, "taper_study_thrust.png")
    fig1.tight_layout()
    fig1.savefig(path1, dpi=150)
    plt.close(fig1)
    
    ax2.set_xlabel("Collective [deg]")
    ax2.set_ylabel("Power [kW]")
    ax2.set_title("Power vs Collective for Varying Taper Ratio")
    ax2.grid(True)
    ax2.legend()
    path2 = os.path.join(outdir, "taper_study_power.png")
    fig2.tight_layout()
    fig2.savefig(path2, dpi=150)
    plt.close(fig2)
    
    ax3.set_xlabel("Collective [deg]")
    ax3.set_ylabel("Figure of Merit")
    ax3.set_title("Efficiency (FM) vs Collective for Varying Taper Ratio")
    ax3.grid(True)
    ax3.legend()
    path3 = os.path.join(outdir, "taper_study_efficiency.png")
    fig3.tight_layout()
    fig3.savefig(path3, dpi=150)
    plt.close(fig3)

    print(f"Saved: {path1}")
    print(f"Saved: {path2}")
    print(f"Saved: {path3}")

if __name__ == '__main__':
    main()
