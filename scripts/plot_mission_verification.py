import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.getcwd(), 'src')))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mission import MissionPlanner, MissionSegment, SegmentType, DesignLimits, MissionInfeasibleError, PowerAvailableModel, FuelModel
import parameters as p

def make_planner(reserve=100.0, fuel=1000.0):
    rotor = p.get_configured_rotor()
    power_model = PowerAvailableModel(sea_level_power_W=p.ENGINE_POWER_W)
    fuel_model = FuelModel(sfc_kg_per_J=p.ENGINE_SFC_KG_J)
    lim = DesignLimits(
        max_tip_mach=p.MAX_TIP_MACH, max_stall_fraction=p.MAX_STALL_FRACTION,
        min_power_margin_frac=p.MIN_POWER_MARGIN_FRAC, min_rpm=200, max_rpm=1000,
        min_collective_deg=-10, max_collective_deg=85, reserve_fuel_kg=reserve
    )
    return MissionPlanner(
        rotor=rotor, airfoil_provider=p.AIRFOIL_PROVIDER, num_rotors=2,
        empty_mass_kg=6200.0, fuel_mass_kg=fuel,
        power_model=power_model, fuel_model=fuel_model,
        limits=lim, flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
    )

def plot_visuals():
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # ---------------------------------------------------------
    # TEST 1: Mass Continuity & Payload Drop
    # ---------------------------------------------------------
    planner1 = make_planner(fuel=1000.0)
    segs1 = [
        MissionSegment("Hover1", SegmentType.HOVER, duration_s=600, dt_s=100, rpm=550, altitude_m=0),
        MissionSegment("Drop", SegmentType.PAYLOAD_EVENT, duration_s=1, dt_s=1, rpm=550, altitude_m=0, payload_delta_kg=-500),
        MissionSegment("Hover2", SegmentType.HOVER, duration_s=600, dt_s=100, rpm=550, altitude_m=0)
    ]
    state1 = planner1.run_mission(segs1)
    
    t1 = [l['time_s']/60 for l in state1.log]
    mass = [l['gross_mass_kg'] for l in state1.log]
    fuel = []
    last_fuel = 1000.0
    for l in state1.log:
        if 'fuel_mass_kg' in l:
            last_fuel = l['fuel_mass_kg']
        fuel.append(last_fuel + 6200)
    
    axes[0].plot(t1, mass, 'b-', lw=3, label='Gross Mass (kg)')
    axes[0].plot(t1, fuel, 'g--', lw=2, label='Empty Mass + Fuel (kg)')
    axes[0].axvline(10, color='r', linestyle=':', label='500kg Payload Drop')
    axes[0].set_title('Test: Mass Continuity & Payload Drop', fontweight='bold')
    axes[0].set_xlabel('Time (Minutes)')
    axes[0].set_ylabel('Mass (kg)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.4)

    # ---------------------------------------------------------
    # TEST 2: Environment (Atmo & Wind) Effect on Power
    # ---------------------------------------------------------
    planner2 = make_planner()
    segs2 = [
        MissionSegment("SL Hover", SegmentType.HOVER, duration_s=120, dt_s=60, rpm=550, altitude_m=0),
        MissionSegment("3km Hover", SegmentType.HOVER, duration_s=120, dt_s=60, rpm=550, altitude_m=3000),
        MissionSegment("Cruise (No Wind)", SegmentType.CRUISE, duration_s=120, dt_s=60, rpm=250, altitude_m=3000, cruise_speed_mps=74, wind_mps=0),
        MissionSegment("Cruise (Tailwind)", SegmentType.CRUISE, duration_s=120, dt_s=60, rpm=250, altitude_m=3000, cruise_speed_mps=74, wind_mps=20)
    ]
    state2 = planner2.run_mission(segs2)
    
    t2 = [l['time_s']/60 for l in state2.log]
    pwr = [l['power_req_W']/1000 for l in state2.log]
    segs_labels = [l['segment'] for l in state2.log]
    
    for seg_name in ["SL Hover", "3km Hover", "Cruise (No Wind)", "Cruise (Tailwind)"]:
        idx = [i for i, name in enumerate(segs_labels) if name == seg_name]
        if idx:
            axes[1].plot([t2[i] for i in idx], [pwr[i] for i in idx], lw=3, label=seg_name)
        
    axes[1].set_title('Test: Altitude & Wind Effects on Power', fontweight='bold')
    axes[1].set_xlabel('Time (Minutes)')
    axes[1].set_ylabel('Power Required (kW)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.4)

    # ---------------------------------------------------------
    # TEST 3: Infeasible Mission / Reserve Fuel Abort
    # ---------------------------------------------------------
    planner3 = make_planner(reserve=150.0, fuel=200.0) # Start with 200kg, reserve is 150kg
    segs3 = [ MissionSegment("Long Hover", SegmentType.HOVER, duration_s=7200, dt_s=60, rpm=550, altitude_m=0) ]
    
    try:
        planner3.run_mission(segs3)
    except MissionInfeasibleError as e:
        pass # Expected to fail!
        
    t3 = [l['time_s']/60 for l in planner3.state.log]
    fuel3 = [l['fuel_mass_kg'] for l in planner3.state.log]
    
    axes[2].plot(t3, fuel3, 'r-', lw=3, label='Fuel Mass')
    axes[2].axhline(150, color='k', linestyle='--', lw=2, label='Reserve Limit (Abort trigger)')
    if t3:
        axes[2].plot(t3[-1], fuel3[-1], 'ko', markersize=8, label='Mission Aborted')
    
    axes[2].set_title('Test: Failure Logic (Reserve Fuel)', fontweight='bold')
    axes[2].set_xlabel('Time (Minutes)')
    axes[2].set_ylabel('Fuel Remaining (kg)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.4)

    plt.tight_layout()
    out_dir = os.path.abspath(os.path.join(os.getcwd(), 'outputs'))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'mission_verification_plots.png')
    fig.savefig(out_path, dpi=150)
    print(f'Saved visuals to {out_path}')

plot_visuals()
