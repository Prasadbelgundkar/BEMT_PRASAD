import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist, linear_twist, linear_taper_chord
from bemt import run_bemt

airfoil = LinearAirfoil()
atmo = isa(0.0)
OMEGA = 2 * np.pi * 1000.0 / 60.0   # 1000 RPM baseline

def make_rotor(chord=0.0508, B=2, taper=1.0, twist_rate=0.0, rc=0.125, R=0.762):
    return Rotor(R, rc, B,
                 linear_taper_chord(chord, taper),
                 linear_twist(0.0, np.radians(twist_rate)))

def main():
    blade_numbers = [2, 3, 4, 5, 6]
    solidities = []
    thrusts = []
    powers = []
    fms = []

    # Choose a fixed collective for this comparison, e.g., 10 degrees
    collective = np.radians(10)

    for B in blade_numbers:
        rotor = make_rotor(B=B)
        perf = run_bemt(rotor, lambda x: airfoil, OMEGA, collective,
                        atmo.density_kg_m3, atmo.speed_of_sound_mps)
        solidities.append(rotor.solidity())
        thrusts.append(perf.thrust_N)
        powers.append(perf.power_W / 1000.0) # kW
        fms.append(perf.figure_of_merit if perf.figure_of_merit else 0)

    outdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(outdir, exist_ok=True)

    # 1. Thrust vs Solidity
    fig1, ax1 = plt.subplots(figsize=(6, 4))
    ax1.plot(solidities, thrusts, 'o-', lw=2)
    ax1.set_xlabel("Solidity (via Blade Number)")
    ax1.set_ylabel("Thrust [N]")
    ax1.set_title("Thrust vs Solidity")
    ax1.grid(True)
    for b, s in zip(blade_numbers, solidities):
        ax1.annotate(f"B={b}", (s, ax1.lines[0].get_ydata()[solidities.index(s)]), textcoords="offset points", xytext=(0,10), ha='center')
    plt.tight_layout()
    path1 = os.path.join(outdir, "solidity_physics_thrust.png")
    fig1.savefig(path1, dpi=150)
    plt.close(fig1)
    print(f"Saved: {path1}")

    # 2. Power vs Solidity
    fig2, ax2 = plt.subplots(figsize=(6, 4))
    ax2.plot(solidities, powers, 'o-', lw=2, color='orange')
    ax2.set_xlabel("Solidity (via Blade Number)")
    ax2.set_ylabel("Power [kW]")
    ax2.set_title("Power vs Solidity")
    ax2.grid(True)
    for b, s in zip(blade_numbers, solidities):
        ax2.annotate(f"B={b}", (s, ax2.lines[0].get_ydata()[solidities.index(s)]), textcoords="offset points", xytext=(0,10), ha='center')
    plt.tight_layout()
    path2 = os.path.join(outdir, "solidity_physics_power.png")
    fig2.savefig(path2, dpi=150)
    plt.close(fig2)
    print(f"Saved: {path2}")

    # 3. Efficiency vs Solidity
    fig3, ax3 = plt.subplots(figsize=(6, 4))
    ax3.plot(solidities, fms, 'o-', lw=2, color='green')
    ax3.set_xlabel("Solidity (via Blade Number)")
    ax3.set_ylabel("Figure of Merit")
    ax3.set_title("Efficiency (FM) vs Solidity")
    ax3.grid(True)
    for b, s in zip(blade_numbers, solidities):
        ax3.annotate(f"B={b}", (s, ax3.lines[0].get_ydata()[solidities.index(s)]), textcoords="offset points", xytext=(0,10), ha='center')
    plt.tight_layout()
    path3 = os.path.join(outdir, "solidity_physics_efficiency.png")
    fig3.savefig(path3, dpi=150)
    plt.close(fig3)
    print(f"Saved: {path3}")

if __name__ == '__main__':
    main()
