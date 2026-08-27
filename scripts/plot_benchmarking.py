"""
scripts/plot_benchmarking.py
-----------------------------
Section 6.3: TW-1500 Proprotor vs Comparable Rotors.

Compares the TW-1500 proprotor against reference data for:
  - Bell XV-15 proprotor (approximate, Johnson 1994)
  - Sikorsky CH-47D tandem rotor (approximate, Leishman 2006)

Nondimensional metrics: CT/sigma vs CP/sigma, FM vs CT/sigma.

Note: Reference values are approximate from open literature; cite properly.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from bemt import run_bemt
import aircraft_input as ac

OUTDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
ATMO_SL = isa(0.0)
COLLS = np.linspace(2, 20, 20)
sigma = ac.ROTOR.solidity()

# -- TW-1500 sweep --
CT_s, CP_s, FM = [], [], []
for c in COLLS:
    perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                    np.radians(c), ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps)
    CT_s.append(perf.CT / sigma)
    CP_s.append(perf.CP / sigma)
    FM.append(perf.figure_of_merit if perf.figure_of_merit else np.nan)
CT_s, CP_s, FM = np.array(CT_s), np.array(CP_s), np.array(FM)

# -- Reference data (approximate, open literature) --
xv15_sigma = 0.089
xv15_CTs = np.array([0.04, 0.06, 0.08, 0.10, 0.12, 0.14])
xv15_FM  = np.array([0.55, 0.68, 0.74, 0.75, 0.74, 0.70])
xv15_CPs = (xv15_CTs**1.5) / (np.sqrt(2) * np.where(xv15_FM>0, xv15_FM, 0.01) * xv15_sigma)

ch47_sigma = 0.118
ch47_CTs = np.array([0.05, 0.07, 0.09, 0.11, 0.13])
ch47_FM  = np.array([0.60, 0.70, 0.74, 0.73, 0.69])
ch47_CPs = (ch47_CTs**1.5) / (np.sqrt(2) * ch47_FM * ch47_sigma)

fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

# Fig 6.3a CT/sigma vs CP/sigma
axes[0].plot(CP_s, CT_s, "b-", lw=2.5, label=f"TW-1500 (sigma={sigma:.3f})")
axes[0].plot(xv15_CPs, xv15_CTs, "r--o", lw=1.5, ms=6,
             label=f"Bell XV-15 (sigma={xv15_sigma:.3f})")
axes[0].plot(ch47_CPs, ch47_CTs, "g:s", lw=1.5, ms=6,
             label=f"CH-47D (sigma={ch47_sigma:.3f})")
axes[0].set_xlabel("$C_P/\sigma$"); axes[0].set_ylabel("$C_T/\sigma$")
axes[0].set_title("Fig 6.3a -- Rotor Benchmarking\n$C_T/\sigma$ vs $C_P/\sigma$ (hover, SL)")
axes[0].legend(); axes[0].grid(alpha=0.3)

# Fig 6.3b FM vs CT/sigma
axes[1].plot(CT_s, FM, "b-", lw=2.5, label=f"TW-1500 (sigma={sigma:.3f})")
axes[1].plot(xv15_CTs, xv15_FM, "r--o", lw=1.5, ms=6,
             label=f"Bell XV-15 (sigma={xv15_sigma:.3f})")
axes[1].plot(ch47_CTs, ch47_FM, "g:s", lw=1.5, ms=6,
             label=f"CH-47D (sigma={ch47_sigma:.3f})")
axes[1].set_xlabel("$C_T/\sigma$"); axes[1].set_ylabel("Figure of Merit FM")
axes[1].set_ylim(0, 1.0)
axes[1].set_title("Fig 6.3b -- FM Benchmarking\nFM vs $C_T/\sigma$ (hover, SL)")
axes[1].legend(); axes[1].grid(alpha=0.3)

fig.suptitle("Section 6.3: TW-1500 vs Comparable Rotors\n"
             "(XV-15, CH-47D reference data: approximate from open literature)",
             fontsize=11, fontweight="bold")
plt.tight_layout()
path = os.path.join(OUTDIR, "benchmarking_CT_CP_FM.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")
print("Benchmarking plot done.")
