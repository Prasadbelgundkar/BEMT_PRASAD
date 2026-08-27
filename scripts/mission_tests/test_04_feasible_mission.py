import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))
from mission import MissionPlanner, MissionSegment, SegmentType, DesignLimits, MissionInfeasibleError, PowerAvailableModel, FuelModel
import parameters as p

def get_planner(reserve=100.0, fuel=1000.0):
    return MissionPlanner(
        rotor=p.get_configured_rotor(), airfoil_provider=p.AIRFOIL_PROVIDER, num_rotors=2,
        empty_mass_kg=6200.0, fuel_mass_kg=fuel,
        power_model=PowerAvailableModel(sea_level_power_W=p.ENGINE_POWER_W), 
        fuel_model=FuelModel(sfc_kg_per_J=p.ENGINE_SFC_KG_J),
        limits=DesignLimits(
            max_tip_mach=p.MAX_TIP_MACH, max_stall_fraction=p.MAX_STALL_FRACTION,
            min_power_margin_frac=p.MIN_POWER_MARGIN_FRAC, min_rpm=200, max_rpm=1000,
            min_collective_deg=-10, max_collective_deg=85, reserve_fuel_kg=reserve
        ), 
        flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
    )

print('--- Running Test 04: Feasible Mission ---')
planner = get_planner()
segs = [
    MissionSegment('Takeoff', SegmentType.HOVER, duration_s=10, dt_s=10, rpm=550, altitude_m=0),
    MissionSegment('Climb', SegmentType.VERTICAL_CLIMB, duration_s=60, dt_s=30, rpm=550, altitude_m=500, vertical_speed_mps=5),
    MissionSegment('Cruise', SegmentType.CRUISE, duration_s=600, dt_s=60, rpm=250, altitude_m=500, cruise_speed_mps=74)
]
try:
    planner.run_mission(segs)
    print('[PASS] Full feasible mission completed smoothly without violating any constraints.')
except Exception as e:
    print(f'[FAIL] Feasible mission crashed: {e}')
