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

print('--- Running Test 01: Mass Continuity, Payload, and Fuel ---')
planner = get_planner()
segs = [
    MissionSegment('Hover1', SegmentType.HOVER, duration_s=10, dt_s=5, rpm=550, altitude_m=0),
    MissionSegment('Drop', SegmentType.PAYLOAD_EVENT, duration_s=1, dt_s=1, rpm=550, altitude_m=0, payload_delta_kg=-500),
    MissionSegment('Hover2', SegmentType.HOVER, duration_s=10, dt_s=5, rpm=550, altitude_m=0)
]
state = planner.run_mission(segs)

mass_start = state.log[0]['gross_mass_kg']
mass_end = state.log[-1]['gross_mass_kg']
payload_drop_event = next(l for l in state.log if l['segment'] == 'Drop')

if payload_drop_event['gross_mass_kg'] < mass_start - 490:
    print('[PASS] Payload Drop verified.')
else:
    print('[FAIL] Payload Drop failed.')

if state.log[-1]['fuel_mass_kg'] < state.log[0]['fuel_mass_kg']:
    print('[PASS] Fuel strictly decreases over time.')
else:
    print('[FAIL] Fuel decrease failed.')
