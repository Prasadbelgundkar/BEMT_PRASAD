import matplotlib.pyplot as plt
import numpy as np
import os
from mission import MissionPlanner, MissionSegment, SegmentType, PowerAvailableModel, FuelModel, DesignLimits, MissionInfeasibleError
import parameters as p

def get_mission_profile():
    profile = []
    for seg_data in p.MISSION_PLAN:
        name, s_type, dur, alt, rpm, coll, speed, dt = seg_data
        enum_type = getattr(SegmentType, s_type)
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
    profile = get_mission_profile()
    
    # Precompute unique segments for the tracker y-axis
    unique_segments = list(dict.fromkeys([seg.name for seg in profile]))

    # --- SETUP LIVE PLOTTING ---
    plt.style.use('dark_background')
    plt.ion()  # Turn on interactive mode for live updates
    fig, (ax2, ax1, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # Initialize empty line objects that we will update live
    line_phase, = ax2.step([], [], color='#ff00ff', linewidth=2.5, where='post')
    ax2.set_title("Aircraft Mission Segment Tracker", fontsize=14, fontweight='bold', color='white')
    ax2.set_ylabel("Flight Phase", fontsize=12)
    ax2.set_yticks(range(len(unique_segments)))
    ax2.set_yticklabels(unique_segments)
    ax2.grid(color='gray', linestyle='--', alpha=0.4)
    ax2.set_ylim(-0.5, len(unique_segments) - 0.5)
    
    line_fuel, = ax1.plot([], [], color='#00ffcc', linewidth=2.5, label="Fuel Mass (kg)")
    ax1.set_title("Real-Time Fuel Burn Tracking", fontsize=14, fontweight='bold', color='white')
    ax1.set_ylabel("Fuel Remaining (kg)", fontsize=12)
    ax1.grid(color='gray', linestyle='--', alpha=0.4)
    ax1.legend()
    
    line_coll, = ax3.plot([], [], color='#ffcc00', linewidth=2.5, label="Collective Pitch (°)")
    ax3.set_title("Auto-Trimmed Collective Pitch (Optimizer in Action)", fontsize=14, fontweight='bold', color='white')
    ax3.set_xlabel("Mission Time (Hours)", fontsize=12)
    ax3.set_ylabel("Pitch (Degrees)", fontsize=12)
    ax3.grid(color='gray', linestyle='--', alpha=0.4)
    ax3.legend()
    
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)

    # Callback function to update the plots during the simulation
    def live_plot_callback(state):
        # Update roughly every 2 calculations to save UI drawing time
        if len(state.log) % 2 != 0: return 
        
        log = state.log
        times = [entry['time_s']/3600 for entry in log]
        fuels = [entry['fuel_mass_kg'] for entry in log]
        segments = [entry['segment'] for entry in log]
        collectives = [entry.get('collective_deg', 0.0) for entry in log]
        segment_ids = [unique_segments.index(s) for s in segments]
        
        # Update data arrays
        line_phase.set_data(times, segment_ids)
        line_fuel.set_data(times, fuels)
        line_coll.set_data(times, collectives)
        
        # Dynamically scale axes
        max_t = max(times[-1] * 1.05, 0.05) if times else 0.05
        ax1.set_xlim(0, max_t)
        
        if fuels:
            ax1.set_ylim(min(fuels)*0.9, max(fuels)*1.05)
        if collectives:
            c_min, c_max = min(collectives), max(collectives)
            if c_max > c_min:
                ax3.set_ylim(c_min - 2, c_max + 2)
                
        # Force a safe GUI redraw
        fig.canvas.draw()
        plt.pause(0.01)

    # --- BUILD PLANNER WITH CALLBACK ---
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
    planner = MissionPlanner(
        rotor=rotor, 
        airfoil_provider=p.AIRFOIL_PROVIDER,
        num_rotors=p.NUM_ENGINES, 
        empty_mass_kg=gross_mass_empty, 
        fuel_mass_kg=p.FUEL_MASS_KG,
        power_model=power_model, 
        fuel_model=fuel_model, 
        limits=limits,
        flat_plate_area_m2=p.FLAT_PLATE_AREA_M2,
        step_callback=live_plot_callback   # <--- PASS CALLBACK HERE
    )

    print("Starting Live Mission Simulation...")
    try:
        final_state = planner.run_mission(profile)
        print(f"\nMission complete. Final fuel: {final_state.fuel_mass_kg:.2f} kg, "
              f"total time: {final_state.time_s/3600:.2f} hours")
    except MissionInfeasibleError as e:
        print(f"\nMISSION FAILED: {e}")
        final_state = planner.state
    except Exception as e:
        import traceback
        with open('crash.log', 'w') as f:
            f.write("CRASHED WITH TRACEBACK:\n")
            f.write(traceback.format_exc())
        print("\nSIMULATION CRASHED - See crash.log")
        final_state = planner.state
        
    # Final plot update just to ensure the absolute last frame is drawn
    live_plot_callback(final_state)
    
    # Save the final image to outputs
    plt.ioff()
    os.makedirs('outputs', exist_ok=True)
    out_path = os.path.join('outputs', 'mission_telemetry_high_res.png')
    plt.savefig(out_path, dpi=150)
    print(f"\nFinal plot saved as '{out_path}'")
