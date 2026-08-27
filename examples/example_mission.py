"""
Task 9 & 10 / Demonstration Cases demo: one feasible mission and one
deliberately infeasible mission (e.g. insufficient fuel loaded), showing
that MissionInfeasibleError correctly identifies the failure point.

Replace the placeholder rotor / mass / power numbers with your team's
actual Task 5 tiltrotor design before using this for your report.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from rotor import Rotor, constant_chord, constant_twist, linear_twist
from airfoil import LinearAirfoil
from mission import (MissionPlanner, MissionSegment, SegmentType,
                      PowerAvailableModel, FuelModel, DesignLimits,
                      MissionInfeasibleError)

# TILTROTOR TROOP VEHICLE DESIGN
# Specs: 2 pilots + 10 passengers (1200kg payload)
# Max Speed: 450 km/h (125 m/s)
# Service Ceiling: 7000 m
# Range: 1000 km (at 125 m/s, duration = 8000 seconds)

airfoil = LinearAirfoil()
# Realistic rotor for a 7,200 kg MTOW tiltrotor (similar to a scaled-down V-22)
rotor = Rotor(radius_m=3.8, root_cutout_m=0.5, num_blades=3,
              chord_fn=constant_chord(0.45),
              twist_fn=linear_twist(np.radians(15.0), np.radians(-15.0)))

# Realistic 1,880 kW (2,520 shp) turboshaft engines (x2) based on the GE CT7-8A (used on Sikorsky S-92)
power_model = PowerAvailableModel(sea_level_power_W=1_880_000.0) 
fuel_model = FuelModel(sfc_kg_per_J=7.60e-8)  # GE CT7-8A SFC (0.45 lb/shp-hr)
limits = DesignLimits(max_tip_mach=0.9, max_stall_fraction=0.40,
                       min_power_margin_frac=0.05, min_rpm=200, max_rpm=1000,
                       min_collective_deg=-10, max_collective_deg=65,
                       reserve_fuel_kg=100.0)

def build_planner(fuel_kg):
    # Empty mass = 4500 kg, Fuel = 1500 kg, Payload = 1200 kg. Total = 7200 kg.
    return MissionPlanner(rotor=rotor, airfoil_provider=lambda x: airfoil,
                           num_rotors=2, empty_mass_kg=4500.0 + 1200.0, fuel_mass_kg=fuel_kg,
                           power_model=power_model, fuel_model=fuel_model, limits=limits)

def mission_profile():
    return [
        MissionSegment("Takeoff hover", SegmentType.HOVER, duration_s=60,
                        altitude_m=0, rpm=550, collective_deg=8, dt_s=10),
        MissionSegment("Climb to Ceiling", SegmentType.VERTICAL_CLIMB, duration_s=600,
                        altitude_m=3500, rpm=550, collective_deg=10,
                        vertical_speed_mps=5.0, dt_s=60),
        MissionSegment("High-Alt Cruise (1000 km)", SegmentType.CRUISE, duration_s=8000,
                        altitude_m=7000, rpm=350, collective_deg=45,
                        cruise_speed_mps=125.0, wind_mps=0.0, dt_s=100),
        MissionSegment("Troop Drop Hover", SegmentType.HOVER, duration_s=120,
                        altitude_m=1000, rpm=550, collective_deg=5, dt_s=20),
    ]

if __name__ == "__main__":
    print("=== Feasible mission (Full Fuel: 1000km Range at 7000m) ===")
    planner = build_planner(fuel_kg=1500.0)
    try:
        final_state = planner.run_mission(mission_profile())
        print(f"Mission complete. Final fuel: {final_state.fuel_mass_kg:.2f} kg, "
              f"final gross mass: {final_state.gross_mass_kg:.2f} kg, "
              f"total time: {final_state.time_s/3600:.2f} hours")
    except MissionInfeasibleError as e:
        print(f"UNEXPECTED FAILURE: {e}")

    print("\n=== Deliberately infeasible mission (Insufficient Fuel) ===")
    planner2 = build_planner(fuel_kg=500.0)  # Only 500kg fuel for a 1000km trip
    try:
        planner2.run_mission(mission_profile())
        print("Mission unexpectedly completed -- adjust the infeasible test case.")
    except MissionInfeasibleError as e:
        print(f"Mission correctly flagged as infeasible:")
        print(f"  Segment : {e.segment_name}")
        print(f"  Time    : {e.time_s/60:.1f} min")
        print(f"  Reason  : {e.reason}")
