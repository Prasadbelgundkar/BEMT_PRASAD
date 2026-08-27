"""
scripts/plot_design_study.py
-----------------------------
Section 4: Rotor Design-Variable Study.

Uses the K&H validation rotor as baseline and independently varies:
  4.1 Solidity -- chord variation (B fixed) + blade-number variation
  4.2 Taper ratio (4 values)
  4.3 Linear twist rate (5 values)
  4.4 Root cut-out (5 values)

All plots saved to outputs/
"""
import sys, os
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
COLLS = np.linspace(2, 14, 13)       # collective deg
OUTDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")

def sweep(rotor, omega=OMEGA):
    T, P, FM = [], [], []
    for c in COLLS:
        perf = run_bemt(rotor, lambda x: airfoil, omega, np.radians(c),
                        atmo.density_kg_m3, atmo.speed_of_sound_mps)
        T.append(perf.thrust_N)
        P.append(perf.power_W / 1000)
        FM.append(perf.figure_of_merit if perf.figure_of_merit else np.nan)
    return np.array(T), np.array(P), np.array(FM)

def make_rotor(chord=0.0508, B=2, taper=1.0, twist_rate=0.0, rc=0.125, R=0.762):
    return Rotor(R, rc, B,
                 linear_taper_chord(chord, taper),
                 linear_twist(0.0, np.radians(twist_rate)))

def trio_plot(variants, labels, title_prefix, fname):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    for rotor, label in zip(variants, labels):
        T, P, FM = sweep(rotor)
        axes[0].plot(COLLS, T, lw=2, label=label)
        axes[1].plot(COLLS, P, lw=2, label=label)
        axes[2].plot(COLLS, FM, lw=2, label=label)
    for ax, ylabel, title in zip(axes,
        ["Thrust [N]", "Power [kW]", "Figure of Merit"],
        [title_prefix+" -- Thrust", title_prefix+" -- Power", title_prefix+" -- FM"]):
        ax.set_xlabel("Collective [deg]"); ax.set_ylabel(ylabel)
        ax.set_title(title); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    plt.tight_layout()
    path = os.path.join(OUTDIR, fname)
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {path}")

# 4.1a Chord (solidity) variation
chords = [0.03, 0.04, 0.051, 0.065, 0.08]
rotors = [make_rotor(chord=c) for c in chords]
labels = [f"c={c:.3f}m, sigma={r.solidity():.3f}" for c, r in zip(chords, rotors)]
trio_plot(rotors, labels, "Fig 4.1 -- Chord/Solidity", "design_study_solidity_chord.png")

# 4.1b Blade-number variation
rotors_b = [make_rotor(B=b) for b in [2, 3, 4, 5]]
labels_b  = [f"B={b}, sigma={r.solidity():.3f}" for b, r in zip([2,3,4,5], rotors_b)]
trio_plot(rotors_b, labels_b, "Fig 4.1 -- Blade Number", "design_study_blade_number.png")

# 4.2 Taper ratio
tapers = [0.4, 0.6, 0.8, 1.0]
rotors_t = [make_rotor(taper=t) for t in tapers]
labels_t  = [f"TR={t:.1f}" for t in tapers]
trio_plot(rotors_t, labels_t, "Fig 4.2 -- Taper Ratio", "design_study_taper.png")

# 4.3 Linear twist
twists = [0.0, -5.0, -10.0, -15.0, -20.0]
rotors_tw = [make_rotor(twist_rate=tw) for tw in twists]
labels_tw  = [f"twist={tw:.0f} deg/R" for tw in twists]
trio_plot(rotors_tw, labels_tw, "Fig 4.3 -- Twist Rate", "design_study_twist.png")

# 4.4 Root cut-out
rcs = [0.05, 0.10, 0.15, 0.20, 0.25]
rotors_rc = [make_rotor(rc=r*0.762) for r in rcs]
labels_rc  = [f"rc/R={r:.2f}" for r in rcs]
trio_plot(rotors_rc, labels_rc, "Fig 4.4 -- Root Cut-Out", "design_study_root_cutout.png")

print("All design-study plots saved.")
