import matplotlib.pyplot as plt
import numpy as np
from mission import MissionPlanner, MissionSegment, SegmentType, PowerAvailableModel, FuelModel, DesignLimits, MissionInfeasibleError
import parameters as p

def build_planner():
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

    gross_mass_empty = p.EMPTY_MASS_KG + p.PAYLOAD_MASS_KG
    return MissionPlanner(
        rotor=rotor, 
        airfoil_provider=p.AIRFOIL_PROVIDER,
        num_rotors=p.NUM_ENGINES, 
        empty_mass_kg=gross_mass_empty, 
        fuel_mass_kg=p.FUEL_MASS_KG,
        power_model=power_model, 
        fuel_model=fuel_model, 
        limits=limits,
        flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
    )

def get_mission_profile():
    profile = []
    for seg_data in p.MISSION_PLAN:
        name, s_type, dur, alt, rpm, coll, speed, dt = seg_data
        
        # Map string to Enum
        enum_type = getattr(SegmentType, s_type)
        
        # Assign speed to the correct variable depending on segment type
        v_speed = speed if "CLIMB" in s_type or "DESCENT" in s_type else 0.0
        c_speed = speed if "CRUISE" in s_type else 0.0
        
        profile.append(MissionSegment(
            name=name, seg_type=enum_type, duration_s=dur,
            altitude_m=alt, rpm=rpm, collective_deg=coll,
            vertical_speed_mps=v_speed, cruise_speed_mps=c_speed,
            dt_s=dt
        ))
    return profile

if __name__ == "__main__":
    planner = build_planner()
    profile = get_mission_profile()
    
    print("Starting Mission Simulation...")
    try:
        final_state = planner.run_mission(profile)
        print(f"\nMission complete. Final fuel: {final_state.fuel_mass_kg:.2f} kg, "
              f"total time: {final_state.time_s/3600:.2f} hours")
    except MissionInfeasibleError as e:
        print(f"\nMISSION FAILED: {e}")
        # Even if it fails, we plot the data up to the failure point
        final_state = planner.state
        
    log = final_state.log
    times = [entry['time_s']/3600 for entry in log]
    fuels = [entry['fuel_mass_kg'] for entry in log]
    gross = [entry['gross_mass_kg'] for entry in log]
    segments = [entry['segment'] for entry in log]

    # Map segment names to numeric IDs for plotting a "flight phase" graph
    unique_segments = list(dict.fromkeys(segments))
    segment_ids = [unique_segments.index(s) for s in segments]

    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot 1: Fuel Burn over Time
    ax1.plot(times, fuels, color='#00ffcc', linewidth=2.5, label="Fuel Mass (kg)")
    ax1.fill_between(times, fuels, min(fuels)*0.9, color='#00ffcc', alpha=0.2)
    ax1.set_title("Real-Time Fuel Burn Tracking", fontsize=14, fontweight='bold', color='white')
    ax1.set_ylabel("Fuel Remaining (kg)", fontsize=12)
    ax1.grid(color='gray', linestyle='--', alpha=0.4)
    ax1.legend()
    
    # Plot 2: Mission Segment Tracker
    ax2.step(times, segment_ids, color='#ff00ff', linewidth=2.5, where='post')
    ax2.set_title("Aircraft Mission Segment Tracker", fontsize=14, fontweight='bold', color='white')
    ax2.set_xlabel("Mission Time (Hours)", fontsize=12)
    ax2.set_ylabel("Flight Phase", fontsize=12)
    ax2.set_yticks(range(len(unique_segments)))
    ax2.set_yticklabels(unique_segments)
    ax2.grid(color='gray', linestyle='--', alpha=0.4)
    
    plt.tight_layout()
    plt.savefig('mission_telemetry.png')
    print("\nPlot saved as 'mission_telemetry.png' and opening interactively...")
    plt.show()
