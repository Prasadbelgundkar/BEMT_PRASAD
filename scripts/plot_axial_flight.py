"""
scripts/plot_axial_flight.py
-----------------------------
Section 6.2: Axial Forward-Flight (Propeller Mode) Assessment.

All aircraft parameters from aircraft_input.py.

Plots:
  Fig 6.2a -- CT vs advance ratio J at 4 collective settings
  Fig 6.2b -- CP vs J at 4 collective settings
  Fig 6.2c -- Propulsive efficiency vs J at 4 collective settings
  Fig 6.2d -- Blade AoA spanwise distribution at design cruise point
  Fig 6.2e -- Feasible operating envelope (stall & Mach limited)
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from bemt import run_bemt, advance_ratio
import aircraft_input as ac

OUTDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
ATMO   = isa(ac.CRUISE_ALTITUDE_M, ac.dISA_K)
V_VALS = np.linspace(5, 90, 22)
COLL_SETTINGS = [2, 5, 8, 12]   # 4 collective angles [deg]
COLORS = ["b", "g", "r", "m"]

# ---- Figs 6.2a/b/c ----
fig, axes = plt.subplots(1, 3, figsize=(16, 5))
for coll_deg, col in zip(COLL_SETTINGS, COLORS):
    J_arr, CT_arr, CP_arr, eta_arr = [], [], [], []
    for V in V_VALS:
        perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.CRUISE_OMEGA,
                        np.radians(coll_deg), ATMO.density_kg_m3,
                        ATMO.speed_of_sound_mps, v_axial=V)
        J_arr.append(advance_ratio(V, ac.CRUISE_OMEGA, ac.ROTOR_RADIUS_M))
        CT_arr.append(perf.CT)
        CP_arr.append(perf.CP)
        eta_arr.append(perf.propulsive_efficiency if perf.propulsive_efficiency else 0.0)
    axes[0].plot(J_arr, CT_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")
    axes[1].plot(J_arr, CP_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")
    axes[2].plot(J_arr, eta_arr, color=col, lw=2, label=f"Coll={coll_deg}deg")

for ax, ylabel, title in zip(axes,
    ["Thrust coeff $C_T$", "Power coeff $C_P$", "Propulsive eff. eta"],
    [f"Fig 6.2a -- CT vs J", f"Fig 6.2b -- CP vs J", f"Fig 6.2c -- eta vs J"]):
    ax.set_xlabel("Advance ratio J"); ax.set_ylabel(ylabel)
    ax.set_title(title + f"\n{ac.CRUISE_RPM:.0f} RPM, h={ac.CRUISE_ALTITUDE_M:.0f} m")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
axes[2].set_ylim(0, 1.0)
fig.suptitle("Section 6.2: Axial Forward-Flight Performance", fontweight="bold")
plt.tight_layout()
path = os.path.join(OUTDIR, "axial_flight_CT_CP_eta.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")

# ---- Fig 6.2d: AoA distribution at cruise design point ----
V_c = ac.CRUISE_SPEED_MPS
coll_c = ac.CRUISE_COLLECTIVE_DEG
perf_c = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.CRUISE_OMEGA,
                   np.radians(coll_c), ATMO.density_kg_m3,
                   ATMO.speed_of_sound_mps, v_axial=V_c, n_stations=60)
J_c = advance_ratio(V_c, ac.CRUISE_OMEGA, ac.ROTOR_RADIUS_M)

r_R   = np.array([e.x for e in perf_c.elements])
alpha = np.degrees(np.array([e.alpha_rad for e in perf_c.elements]))
stall_lim = np.degrees(ac.AIRFOIL.stall_alpha_rad)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(r_R, alpha, "b-", lw=2, label="AoA distribution")
ax.axhline(stall_lim, color="r", ls="--", lw=1.5, label=f"Stall AoA={stall_lim:.0f}deg")
ax.fill_between(r_R, alpha, stall_lim, where=alpha > stall_lim,
                 alpha=0.3, color="red", label="Stalled region")
ax.set_xlabel("Non-dimensional radius r/R"); ax.set_ylabel("AoA [deg]")
ax.set_title(f"Fig 6.2d -- AoA Spanwise Distribution at Cruise\n"
             f"V={V_c:.0f} m/s, J={J_c:.3f}, Coll={coll_c:.0f}deg, "
             f"{ac.CRUISE_RPM:.0f} RPM, h={ac.CRUISE_ALTITUDE_M:.0f} m")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "axial_flight_AoA_dist.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")

# ---- Fig 6.2e: Feasible operating envelope ----
fig, ax = plt.subplots(figsize=(9, 6))
for coll_deg, col in zip(COLL_SETTINGS, COLORS):
    J_f, CT_f = [], []
    for V in V_VALS:
        perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.CRUISE_OMEGA,
                        np.radians(coll_deg), ATMO.density_kg_m3,
                        ATMO.speed_of_sound_mps, v_axial=V)
        ok = (perf.max_tip_mach <= ac.MAX_TIP_MACH and
              perf.stalled_fraction <= ac.MAX_STALL_FRACTION and
              perf.CT > 0)
        J = advance_ratio(V, ac.CRUISE_OMEGA, ac.ROTOR_RADIUS_M)
        if ok:
            J_f.append(J); CT_f.append(perf.CT)
    if J_f:
        ax.plot(J_f, CT_f, "o-", color=col, lw=2, ms=5, label=f"Coll={coll_deg}deg")
ax.set_xlabel("Advance ratio J"); ax.set_ylabel("Thrust coefficient $C_T$")
ax.set_title(f"Fig 6.2e -- Feasible Operating Envelope\n"
             f"Tip Mach<={ac.MAX_TIP_MACH}, Stall<={ac.MAX_STALL_FRACTION*100:.0f}%, "
             f"{ac.CRUISE_RPM:.0f} RPM, h={ac.CRUISE_ALTITUDE_M:.0f} m")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "axial_flight_feasible_envelope.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")
print("Axial-flight plots done.")
