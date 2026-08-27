from mission import MissionPlanner, MissionSegment, SegmentType, PowerAvailableModel, FuelModel, DesignLimits, MissionInfeasibleError
import parameters as p

def get_mission_profile():
    profile = []
    for seg_data in p.MISSION_PLAN:
        name, s_type, dur, alt, rpm, coll, speed, dt = seg_data
        enum_type = getattr(SegmentType, s_type)
        v_speed = speed if 'CLIMB' in s_type or 'DESCENT' in s_type else 0.0
        c_speed = speed if 'CRUISE' in s_type else 0.0
        profile.append(MissionSegment(
            name=name, seg_type=enum_type, duration_s=dur,
            altitude_m=alt, rpm=rpm, collective_deg=coll,
            vertical_speed_mps=v_speed, cruise_speed_mps=c_speed,
            dt_s=dt
        ))
    return profile

rotor = p.get_configured_rotor()
power_model = PowerAvailableModel(sea_level_power_W=p.ENGINE_POWER_W)
fuel_model = FuelModel(sfc_kg_per_J=p.ENGINE_SFC_KG_J)
limits = DesignLimits(
    max_tip_mach=p.MAX_TIP_MACH, 
    max_stall_fraction=p.MAX_STALL_FRACTION,
    min_power_margin_frac=p.MIN_POWER_MARGIN_FRAC, 
    min_rpm=200, max_rpm=1000,
    min_collective_deg=-10, max_collective_deg=65,
    reserve_fuel_kg=p.RESERVE_FUEL_KG
)

planner = MissionPlanner(
    rotor=rotor, 
    airfoil_provider=p.AIRFOIL_PROVIDER,
    num_rotors=p.NUM_ENGINES, 
    empty_mass_kg=p.EMPTY_MASS_KG + p.PAYLOAD_MASS_KG, 
    fuel_mass_kg=p.FUEL_MASS_KG,
    power_model=power_model, 
    fuel_model=fuel_model, 
    limits=limits,
    flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
)

print("Starting run_mission...")
try:
    final_state = planner.run_mission(get_mission_profile())
    print(f'Done! Fuel remaining: {final_state.fuel_mass_kg}')
except Exception as e:
    print(f"Python Caught Exception: {e}")
    import traceback
    traceback.print_exc()
print("Script finished normally.")
