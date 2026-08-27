"""
scripts/plot_mission_analysis.py
---------------------------------
Section 7: Mission Planner v1 Tests.

All aircraft parameters from aircraft_input.py.

7.1: Feasible + infeasible mission demonstration (printout + log)
7.2: Fuel-burn rate vs gross weight (hover)
7.3: Hover endurance vs takeoff weight
7.4: Cruise range vs cruise speed
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from bemt import run_bemt, trim_hover_collective, advance_ratio
from mission import (MissionPlanner, MissionSegment, SegmentType, MissionInfeasibleError)
import aircraft_input as ac

OUTDIR   = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
ATMO_SL  = isa(ac.TAKEOFF_ALTITUDE_M, ac.dISA_K)
ATMO_CRU = isa(ac.CRUISE_ALTITUDE_M,  ac.dISA_K)
G        = ac.G

# Auto-trim hover collective at MTOW, sea level
coll_hover = trim_hover_collective(
    ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
    ac.weight_per_rotor_N(ac.GROSS_MASS_KG),
    ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps,
    coll_range_deg=(ac.MIN_COLLECTIVE_DEG, ac.MAX_COLLECTIVE_DEG),
)
if coll_hover is None:
    coll_hover = ac.HOVER_COLLECTIVE_DEG
    print(f"[WARN] Auto-trim failed; using default hover collective = {coll_hover:.1f} deg")
else:
    print(f"Auto-trimmed hover collective = {coll_hover:.2f} deg  (T=W at MTOW, SL)")

def make_planner(fuel_kg):
    return MissionPlanner(
        rotor=ac.ROTOR, airfoil_provider=ac.airfoil_provider,
        num_rotors=ac.NUM_ROTORS, empty_mass_kg=ac.EMPTY_MASS_KG,
        fuel_mass_kg=fuel_kg, power_model=ac.POWER_MODEL,
        fuel_model=ac.FUEL_MODEL, limits=ac.LIMITS,
    )

SEGMENTS = [
    MissionSegment("Takeoff hover", SegmentType.HOVER, duration_s=60,
                   altitude_m=ac.TAKEOFF_ALTITUDE_M, dISA_K=ac.dISA_K,
                   rpm=ac.HOVER_RPM, collective_deg=coll_hover, dt_s=5),
    MissionSegment("Climb", SegmentType.VERTICAL_CLIMB, duration_s=600,
                   altitude_m=ac.SERVICE_CEILING_M/2, dISA_K=ac.dISA_K,
                   rpm=ac.HOVER_RPM, collective_deg=coll_hover,
                   vertical_speed_mps=4.0, dt_s=30),
    MissionSegment("Cruise out", SegmentType.CRUISE, duration_s=1500,
                   altitude_m=ac.CRUISE_ALTITUDE_M, dISA_K=ac.dISA_K,
                   rpm=ac.CRUISE_RPM, collective_deg=ac.CRUISE_COLLECTIVE_DEG,
                   cruise_speed_mps=ac.CRUISE_SPEED_MPS, wind_mps=-5.0, dt_s=60),
    MissionSegment("Payload drop", SegmentType.PAYLOAD_EVENT, duration_s=0,
                   altitude_m=ac.CRUISE_ALTITUDE_M, payload_delta_kg=-ac.PAYLOAD_KG*0.5),
    MissionSegment("Loiter", SegmentType.LOITER, duration_s=300,
                   altitude_m=ac.CRUISE_ALTITUDE_M, dISA_K=ac.dISA_K,
                   rpm=ac.CRUISE_RPM, collective_deg=ac.CRUISE_COLLECTIVE_DEG+2, dt_s=30),
    MissionSegment("Cruise return", SegmentType.CRUISE, duration_s=1500,
                   altitude_m=ac.CRUISE_ALTITUDE_M, dISA_K=ac.dISA_K,
                   rpm=ac.CRUISE_RPM, collective_deg=ac.CRUISE_COLLECTIVE_DEG,
                   cruise_speed_mps=ac.CRUISE_SPEED_MPS, wind_mps=5.0, dt_s=60),
    MissionSegment("Landing hover", SegmentType.HOVER, duration_s=60,
                   altitude_m=ac.TAKEOFF_ALTITUDE_M, dISA_K=ac.dISA_K,
                   rpm=ac.HOVER_RPM, collective_deg=coll_hover, dt_s=5),
]

# ---- 7.1 Feasible mission ----
print("\n" + "="*60)
print("Section 7.1 -- Feasible Mission")
print("="*60)
p1 = make_planner(ac.FUEL_MASS_KG)
try:
    s1 = p1.run_mission(SEGMENTS)
    print(f"  STATUS   : COMPLETED")
    print(f"  Duration : {s1.time_s/60:.1f} min")
    print(f"  Fuel left: {s1.fuel_mass_kg:.1f} kg  (reserve={ac.RESERVE_FUEL_KG:.0f} kg)")
    print(f"  Mass left: {s1.gross_mass_kg:.1f} kg")
    print(f"  Log steps: {len(s1.log)}")
except MissionInfeasibleError as e:
    print(f"  STATUS   : FAILED -- {e}")

# ---- Infeasible mission (too little fuel) ----
print("\n" + "="*60)
print("Section 7.1 -- Deliberately Infeasible Mission (insufficient fuel)")
print("="*60)
p2 = make_planner(ac.RESERVE_FUEL_KG + 1.0)
try:
    s2 = p2.run_mission(SEGMENTS)
    print("  (Mission completed -- reduce fuel or duration to trigger failure)")
except MissionInfeasibleError as e:
    print(f"  STATUS   : CORRECTLY FLAGGED")
    print(f"  Segment  : {e.segment_name}")
    print(f"  Time     : {e.time_s:.1f} s")
    print(f"  Reason   : {e.reason}")

# ---- 7.2 Fuel-burn rate vs gross weight ----
print("\nComputing 7.2: fuel-burn rate vs gross weight...")
masses = np.linspace(ac.EMPTY_MASS_KG, ac.GROSS_MASS_KG, 20)
burns  = []
for m in masses:
    coll = trim_hover_collective(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                                  ac.weight_per_rotor_N(m),
                                  ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps)
    if coll is None:
        burns.append(np.nan); continue
    perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                    np.radians(coll), ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps)
    P_tot = ac.NUM_ROTORS * perf.power_W
    burns.append(ac.FUEL_MODEL.burn_rate_kg_s(P_tot) * 3600)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(masses, burns, "b-o", lw=2, ms=5)
ax.set_xlabel("Gross mass [kg]"); ax.set_ylabel("Fuel burn rate [kg/hr]")
ax.set_title(f"Fig 7.2 -- Hover Fuel-Burn Rate vs Gross Mass\n"
             f"{ac.HOVER_RPM:.0f} RPM, SL ISA, {ac.NUM_ROTORS} rotors, "
             f"SFC={ac.SFC_KG_PER_J:.1e} kg/J")
ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "mission_fuel_burn_vs_mass.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")

# ---- 7.3 Hover endurance vs TOW ----
print("Computing 7.3: hover endurance vs TOW...")
endurance = []
for m in masses:
    coll = trim_hover_collective(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                                  ac.weight_per_rotor_N(m),
                                  ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps)
    if coll is None:
        endurance.append(np.nan); continue
    perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.HOVER_OMEGA,
                    np.radians(coll), ATMO_SL.density_kg_m3, ATMO_SL.speed_of_sound_mps)
    P_tot = ac.NUM_ROTORS * perf.power_W
    br = ac.FUEL_MODEL.burn_rate_kg_s(P_tot)
    usable = ac.FUEL_MASS_KG - ac.RESERVE_FUEL_KG
    endurance.append((usable / br) / 3600 if br > 0 else np.nan)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(masses, endurance, "g-o", lw=2, ms=5)
ax.axhline(ac.HOVER_ENDURANCE_MIN/60, color="orange", ls="--", lw=1.5,
           label=f"Target={ac.HOVER_ENDURANCE_MIN:.0f} min")
ax.axvline(ac.GROSS_MASS_KG, color="r", ls="--", lw=1.5,
           label=f"MTOW={ac.GROSS_MASS_KG:.0f} kg")
ax.set_xlabel("Takeoff gross mass [kg]"); ax.set_ylabel("Hover endurance [hr]")
ax.set_title(f"Fig 7.3 -- Hover Endurance vs Takeoff Mass\n"
             f"Reserve={ac.RESERVE_FUEL_KG:.0f} kg, "
             f"Usable fuel={ac.FUEL_MASS_KG-ac.RESERVE_FUEL_KG:.0f} kg, "
             f"{ac.HOVER_RPM:.0f} RPM, SL ISA")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "mission_hover_endurance_vs_TOW.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")

# ---- 7.4 Cruise range vs speed ----
print("Computing 7.4: cruise range vs speed...")
V_speeds = np.linspace(20, 90, 20)
ranges_km = []
for V in V_speeds:
    perf = run_bemt(ac.ROTOR, ac.airfoil_provider, ac.CRUISE_OMEGA,
                    np.radians(ac.CRUISE_COLLECTIVE_DEG),
                    ATMO_CRU.density_kg_m3, ATMO_CRU.speed_of_sound_mps, v_axial=V)
    if perf.power_W > 0:
        P_tot   = ac.NUM_ROTORS * perf.power_W
        br      = ac.FUEL_MODEL.burn_rate_kg_s(P_tot)
        usable  = ac.FUEL_MASS_KG - ac.RESERVE_FUEL_KG
        t_s     = usable / br if br > 0 else 0
        ranges_km.append(V * t_s / 1000)
    else:
        ranges_km.append(np.nan)

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(V_speeds, ranges_km, "m-o", lw=2, ms=5)
ax.axhline(ac.RANGE_TARGET_KM, color="r", ls="--", lw=1.5,
           label=f"Range target={ac.RANGE_TARGET_KM:.0f} km")
ax.axvline(ac.CRUISE_SPEED_MPS, color="g", ls="--", lw=1.5,
           label=f"Design cruise={ac.CRUISE_SPEED_MPS:.0f} m/s")
ax.set_xlabel("Cruise speed [m/s]"); ax.set_ylabel("Range [km]")
ax.set_title(f"Fig 7.4 -- Cruise Range vs Cruise Speed\n"
             f"Reserve={ac.RESERVE_FUEL_KG:.0f} kg, "
             f"Usable={ac.FUEL_MASS_KG-ac.RESERVE_FUEL_KG:.0f} kg, "
             f"h={ac.CRUISE_ALTITUDE_M:.0f} m, Coll={ac.CRUISE_COLLECTIVE_DEG:.0f}deg, "
             f"{ac.CRUISE_RPM:.0f} RPM")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
path = os.path.join(OUTDIR, "mission_cruise_range_vs_speed.png")
fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {path}")
print("All mission analysis plots done.")
