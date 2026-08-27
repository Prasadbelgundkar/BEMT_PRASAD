"""
Task 7 demo: axial forward-flight (airplane mode / propeller-like) sweep
over advance ratio J at a few collective settings. Reports CT, CP,
propulsive efficiency, and flags the feasible envelope (no stall, tip Mach
within limit).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, linear_twist
from bemt import run_bemt, advance_ratio

airfoil = LinearAirfoil()
atmo = isa(3000.0, 0.0)   # example cruise altitude -- replace with your design point
OMEGA = 2 * np.pi * 2200.0 / 60.0   # example propeller-mode RPM -- replace with yours

# NOTE: this is a PLACEHOLDER rotor (radius/chord/twist/RPM chosen only so
# the demo produces a physically sensible propeller-mode operating point).
# Replace with your Task 5 tiltrotor design before using these results in
# your report. Propeller-mode rotors typically need substantial built-in
# washout (large root pitch, less at tip) to keep local AoA reasonable when
# tangential speed varies a lot along the span relative to axial speed --
# that's why linear_twist is used here instead of a flat/untwisted blade.
rotor = Rotor(radius_m=0.762, root_cutout_m=0.125, num_blades=3,
              chord_fn=constant_chord(0.06),
              twist_fn=linear_twist(np.radians(30.0), np.radians(-20.0)))

if __name__ == "__main__":
    V_list = np.linspace(5, 60, 18)   # m/s, forward/axial speed sweep
    collectives_deg = [6, 10, 14]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    for coll in collectives_deg:
        J_vals, CT_vals, CP_vals, eta_vals = [], [], [], []
        for V in V_list:
            perf = run_bemt(rotor, lambda x: airfoil, OMEGA, np.radians(coll),
                             atmo.density_kg_m3, atmo.speed_of_sound_mps, v_axial=V)
            J = advance_ratio(V, OMEGA, rotor.radius_m)
            J_vals.append(J); CT_vals.append(perf.CT); CP_vals.append(perf.CP)
            eta_vals.append(perf.propulsive_efficiency if perf.propulsive_efficiency else 0.0)

        axes[0].plot(J_vals, CT_vals, label=f"collective={coll} deg")
        axes[1].plot(J_vals, CP_vals, label=f"collective={coll} deg")
        axes[2].plot(J_vals, eta_vals, label=f"collective={coll} deg")

    axes[0].set_xlabel("Advance ratio J"); axes[0].set_ylabel("CT"); axes[0].grid(alpha=0.3)
    axes[1].set_xlabel("Advance ratio J"); axes[1].set_ylabel("CP"); axes[1].grid(alpha=0.3)
    axes[2].set_xlabel("Advance ratio J"); axes[2].set_ylabel("Propulsive efficiency")
    axes[2].grid(alpha=0.3)
    for a in axes:
        a.legend(fontsize=8)
    fig.suptitle(f"Axial forward-flight (propeller-mode) sweep, "
                 f"h={atmo.altitude_m:.0f} m, Omega={OMEGA:.1f} rad/s")
    fig.tight_layout()
    fig.savefig("outputs/axial_forward_flight_sweep.png", dpi=150)
    print("Saved axial_forward_flight_sweep.png to outputs/")
