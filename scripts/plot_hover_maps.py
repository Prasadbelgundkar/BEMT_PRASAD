"""
scripts/plot_hover_maps.py
---------------------------
Section 6.1: Hover Performance Maps for the TW-1500 Tiltrotor Design.

All aircraft parameters come from aircraft_input.py.

Plots:
  hover_map_h0m.png    -- T, Q, P, FM, Stall%, Power-margin vs collective at SL
  hover_map_h3000m.png -- same at SERVICE_CEILING_M
  hover_max_weight_vs_altitude.png -- max hover GW vs altitude (power & stall limited)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from bemt import run_bemt, trim_hover_collective
import aircraft_input as ac

OUTDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
COLLS  = np.linspace(ac.MIN_COLLECTIVE_DEG, ac.MAX_COLLECTIVE_DEG, 30)

def hover_sweep(altitude_m):
    atmo = isa(altitude_m, ac.dISA_K)
    P_avail_kW = ac.POWER_MODEL.power_available_W(atmo) / 1000
    T, Q, P, FM, stall = [], [], [], [], []
    for c in COLLS:
        perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                        np.radians(c), atmo.density_kg_m3, atmo.speed_of_sound_mps)
        T.append(perf.thrust_N); Q.append(perf.torque_Nm)
        P.append(perf.power_W / 1000)
        FM.append(perf.figure_of_merit if perf.figure_of_merit else np.nan)
        stall.append(perf.stalled_fraction * 100)
    return (np.array(T), np.array(Q), np.array(P),
            np.array(FM), np.array(stall), P_avail_kW)

def plot_hover_map(altitude_m):
    T, Q, P, FM, stall, P_avail = hover_sweep(altitude_m)
    W_N = ac.weight_per_rotor_N(ac.GROSS_MASS_KG)
    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    ax = axes.flatten()

    ax[0].plot(COLLS, T, "b", lw=2)
    ax[0].axhline(W_N, color="r", ls="--", lw=1.5, label=f"W/rotor={W_N:.0f} N")
    ax[0].set_xlabel("Collective [deg]"); ax[0].set_ylabel("Thrust per rotor [N]")
    ax[0].set_title(f"Thrust vs Collective  h={altitude_m:.0f} m")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    ax[1].plot(COLLS, Q, "g", lw=2)
    ax[1].set_xlabel("Collective [deg]"); ax[1].set_ylabel("Torque per rotor [N·m]")
    ax[1].set_title(f"Torque vs Collective  h={altitude_m:.0f} m"); ax[1].grid(alpha=0.3)

    ax[2].plot(COLLS, P, "r", lw=2, label="P required")
    ax[2].axhline(P_avail, color="k", ls="--", lw=1.5, label=f"P avail={P_avail:.1f} kW")
    plim_mask = P > P_avail * (1 - ac.MIN_POWER_MARGIN_FRAC)
    if plim_mask.any():
        ax[2].fill_between(COLLS, P, P_avail, where=plim_mask,
                           alpha=0.25, color="red", label="Power limited")
    ax[2].set_xlabel("Collective [deg]"); ax[2].set_ylabel("Power per rotor [kW]")
    ax[2].set_title(f"Power vs Collective  h={altitude_m:.0f} m")
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    ax[3].plot(COLLS, FM, "m", lw=2)
    ax[3].axhline(0.75, color="g", ls="--", lw=1, label="FM=0.75 ref")
    ax[3].set_xlabel("Collective [deg]"); ax[3].set_ylabel("Figure of Merit")
    ax[3].set_ylim(0, 1.0); ax[3].set_title(f"FM vs Collective  h={altitude_m:.0f} m")
    ax[3].legend(fontsize=8); ax[3].grid(alpha=0.3)

    ax[4].plot(COLLS, stall, color="orange", lw=2)
    slim = ac.MAX_STALL_FRACTION * 100
    ax[4].axhline(slim, color="r", ls="--", lw=1.5, label=f"Stall limit {slim:.0f}%")
    ax[4].fill_between(COLLS, stall, slim, where=stall > slim,
                        alpha=0.25, color="orange", label="Stall limited")
    ax[4].set_xlabel("Collective [deg]"); ax[4].set_ylabel("Stalled blade span [%]")
    ax[4].set_title(f"Stall Fraction vs Collective  h={altitude_m:.0f} m")
    ax[4].legend(fontsize=8); ax[4].grid(alpha=0.3)

    margin = (P_avail - P) / P_avail * 100
    ax[5].plot(COLLS, margin, "c", lw=2)
    ax[5].axhline(ac.MIN_POWER_MARGIN_FRAC * 100, color="r", ls="--",
                  lw=1.5, label=f"Min margin {ac.MIN_POWER_MARGIN_FRAC*100:.0f}%")
    ax[5].set_xlabel("Collective [deg]"); ax[5].set_ylabel("Power margin [%]")
    ax[5].set_title(f"Power Margin vs Collective  h={altitude_m:.0f} m")
    ax[5].legend(fontsize=8); ax[5].grid(alpha=0.3)

    sigma = ac.ROTOR.solidity()
    fig.suptitle(
        f"Section 6.1: Hover Performance Map  |  {ac.HOVER_RPM:.0f} RPM, h={altitude_m:.0f} m\n"
        f"B={ac.NUM_BLADES}, R={ac.ROTOR_RADIUS_M} m, sigma={sigma:.4f}, "
        f"MTOW={ac.GROSS_MASS_KG:.0f} kg, P_avail/rotor={P_avail:.0f} kW",
        fontsize=11, fontweight="bold")
    plt.tight_layout()
    path = os.path.join(OUTDIR, f"hover_map_h{int(altitude_m)}m.png")
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {path}")

plot_hover_map(ac.TAKEOFF_ALTITUDE_M)
plot_hover_map(ac.SERVICE_CEILING_M)

# -- Max hover weight vs altitude --
print("Computing max hover weight envelope...")
altitudes = np.linspace(0, ac.SERVICE_CEILING_M, 18)
mass_pwr, mass_stl = [], []
for h in altitudes:
    atmo = isa(h, ac.dISA_K)
    P_avail = ac.POWER_MODEL.power_available_W(atmo)
    T_cands = np.linspace(500, 12000, 50)
    Tp, Ts = None, None
    for Tc in reversed(T_cands):
        coll = trim_hover_collective(
            ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA, Tc,
            atmo.density_kg_m3, atmo.speed_of_sound_mps,
            coll_range_deg=(ac.MIN_COLLECTIVE_DEG, ac.MAX_COLLECTIVE_DEG))
        if coll is None: continue
        perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                        np.radians(coll), atmo.density_kg_m3, atmo.speed_of_sound_mps)
        if Tp is None and perf.power_W <= P_avail * (1 - ac.MIN_POWER_MARGIN_FRAC):
            Tp = Tc
        if Ts is None and perf.stalled_fraction <= ac.MAX_STALL_FRACTION:
            Ts = Tc
        if Tp and Ts: break
    mass_pwr.append(Tp * ac.NUM_ROTORS / 9.80665 if Tp else np.nan)
    mass_stl.append(Ts * ac.NUM_ROTORS / 9.80665 if Ts else np.nan)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(altitudes/1000, mass_pwr, "r-o", ms=5, lw=2, label="Power limited")
ax.plot(altitudes/1000, mass_stl, "b--s", ms=5, lw=2, label="Stall limited")
ax.axhline(ac.GROSS_MASS_KG, color="k", ls=":", lw=1.5,
           label=f"Design MTOW={ac.GROSS_MASS_KG:.0f} kg")
ax.set_xlabel("Altitude [km]"); ax.set_ylabel("Max hover gross mass [kg]")
ax.set_title(f"Fig 6.1d -- Max Hover Gross Weight vs Altitude\n"
             f"{ac.HOVER_RPM:.0f} RPM, B={ac.NUM_BLADES}, R={ac.ROTOR_RADIUS_M} m, std ISA")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "hover_max_weight_vs_altitude.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")
print("Hover map plots done.")
