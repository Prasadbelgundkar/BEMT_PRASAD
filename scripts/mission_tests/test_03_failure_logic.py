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

print('--- Running Test 03: Failure Logic & Constraints ---')
planner = get_planner(reserve=150.0, fuel=155.0)
segs = [MissionSegment('Long', SegmentType.HOVER, duration_s=3600, dt_s=60, rpm=550, altitude_m=0)]
try:
    planner.run_mission(segs)
    print('[FAIL] Reserve Fuel test failed (did not abort)')
except MissionInfeasibleError as e:
    if 'reserve' in str(e).lower():
        print('[PASS] Reserve Fuel failure logic verified.')
    else:
        print(f'[FAIL] Wrong error: {e}')

planner2 = get_planner()
segs2 = [MissionSegment('Fail', SegmentType.HOVER, duration_s=10, dt_s=5, rpm=550, altitude_m=16000)]
try:
    planner2.run_mission(segs2)
    print('[FAIL] Power Limit test failed (did not abort)')
except MissionInfeasibleError as e:
    print('[PASS] Power/Aerodynamic Limit failure logic verified.')
