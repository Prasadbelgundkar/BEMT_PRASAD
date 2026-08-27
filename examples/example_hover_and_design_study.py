"""
Task 4 & 6 demo: hover performance sweep vs. collective, and a solidity
design-variable study on the validation rotor. Extend this pattern for
taper, twist, blade number, root cutout, and RPM sweeps (Section 4).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt

airfoil = LinearAirfoil()
atmo = isa(0.0, 0.0)
OMEGA = 2 * np.pi * 1000.0 / 60.0


def make_rotor(chord_m, num_blades=2, radius_m=0.762, root_cutout_m=0.125):
    return Rotor(radius_m=radius_m, root_cutout_m=root_cutout_m,
                 num_blades=num_blades, chord_fn=constant_chord(chord_m),
                 twist_fn=constant_twist(0.0))


def hover_sweep(rotor, collectives_deg):
    T, Q, P, FM, stall = [], [], [], [], []
    for c in collectives_deg:
        perf = run_bemt(rotor, lambda x: airfoil, OMEGA, np.radians(c),
                         atmo.density_kg_m3, atmo.speed_of_sound_mps, v_axial=0.0)
        T.append(perf.thrust_N); Q.append(perf.torque_Nm); P.append(perf.power_W)
        FM.append(perf.figure_of_merit); stall.append(perf.stalled_fraction)
    return map(np.array, (T, Q, P, FM, stall))


if __name__ == "__main__":
    collectives = np.linspace(2, 16, 15)

    # ---- Task 6.1: hover performance map for the baseline rotor ----------
    base_rotor = make_rotor(chord_m=0.0508)
    T, Q, P, FM, stall = hover_sweep(base_rotor, collectives)

    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    ax[0, 0].plot(collectives, T); ax[0, 0].set_xlabel("Collective [deg]")
    ax[0, 0].set_ylabel("Thrust [N]"); ax[0, 0].grid(alpha=0.3)
    ax[0, 1].plot(collectives, Q); ax[0, 1].set_xlabel("Collective [deg]")
    ax[0, 1].set_ylabel("Torque [N.m]"); ax[0, 1].grid(alpha=0.3)
    ax[1, 0].plot(collectives, P / 1000.0); ax[1, 0].set_xlabel("Collective [deg]")
    ax[1, 0].set_ylabel("Power [kW]"); ax[1, 0].grid(alpha=0.3)
    ax[1, 1].plot(collectives, stall * 100)
    ax[1, 1].set_xlabel("Collective [deg]"); ax[1, 1].set_ylabel("Stalled blade span [%]")
    ax[1, 1].grid(alpha=0.3)
    fig.suptitle("Hover performance vs. collective, sea level")
    fig.tight_layout()
    fig.savefig("outputs/hover_performance_map.png", dpi=150)

    # ---- Task 4.1: solidity variation study (via chord) -------------------
    chords = np.linspace(0.03, 0.09, 5)
    plt.figure(figsize=(6, 4.5))
    for c in chords:
        rotor = make_rotor(chord_m=c)
        T, Q, P, FM, stall = hover_sweep(rotor, collectives)
        sigma = rotor.solidity()
        plt.plot(collectives, T, label=f"sigma={sigma:.3f}")
    plt.xlabel("Collective [deg]"); plt.ylabel("Thrust [N]")
    plt.title("Effect of solidity on hover thrust")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig("outputs/solidity_study.png", dpi=150)

    print("Saved hover_performance_map.png and solidity_study.png to outputs/")
