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

print('--- Running Test 02: Atmospheric & Wind Effects ---')
planner = get_planner()
segs = [
    MissionSegment('SL', SegmentType.HOVER, duration_s=2, dt_s=2, rpm=550, altitude_m=0),
    MissionSegment('High', SegmentType.HOVER, duration_s=2, dt_s=2, rpm=550, altitude_m=3000)
]
planner.run_mission(segs)
p_sl = planner.state.log[0]['power_req_W']
p_high = planner.state.log[1]['power_req_W']
if p_high > p_sl:
    print(f'[PASS] Atmosphere Variation verified (Power SL: {p_sl/1000:.0f}kW, Power 3km: {p_high/1000:.0f}kW)')
else:
    print('[FAIL] Atmosphere Variation failed')

planner2 = get_planner()
segs2 = [
    MissionSegment('NoWind', SegmentType.CRUISE, duration_s=2, dt_s=2, rpm=250, altitude_m=0, cruise_speed_mps=70, wind_mps=0),
    MissionSegment('Tailwind', SegmentType.CRUISE, duration_s=2, dt_s=2, rpm=250, altitude_m=0, cruise_speed_mps=70, wind_mps=20)
]
planner2.run_mission(segs2)
if planner2.state.log[0]['power_req_W'] != planner2.state.log[1]['power_req_W']:
    print('[PASS] Wind Treatment verified (Wind alters power required)')
else:
    print('[FAIL] Wind Treatment failed')
