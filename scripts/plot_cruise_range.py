import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

curr_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
src_path = os.path.abspath(os.path.join(curr_dir, '..', 'src'))
sys.path.insert(0, src_path)

from mission import MissionPlanner, MissionSegment, SegmentType, DesignLimits, PowerAvailableModel, FuelModel
import parameters as p
from environment import isa

def main():
    rotor = p.get_configured_rotor()
    power_model = PowerAvailableModel(sea_level_power_W=p.ENGINE_POWER_W)
    fuel_model = FuelModel(sfc_kg_per_J=p.ENGINE_SFC_KG_J)
    limits = DesignLimits(
        max_tip_mach=p.MAX_TIP_MACH, max_stall_fraction=p.MAX_STALL_FRACTION,
        min_power_margin_frac=0.0, min_rpm=200, max_rpm=1000,
        min_collective_deg=-10, max_collective_deg=85, reserve_fuel_kg=150.0
    )
    
    planner = MissionPlanner(
        rotor=rotor, airfoil_provider=p.AIRFOIL_PROVIDER, num_rotors=p.NUM_ENGINES,
        empty_mass_kg=4500.0, fuel_mass_kg=1500.0,
        power_model=power_model, fuel_model=fuel_model,
        limits=limits, flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
    )
    
    # Set to MTOW (7200 kg) for worst-case conservative range estimate
    planner.state.gross_mass_kg = 7200.0
    
    altitude_m = 7000.0 # High altitude cruise for efficiency
    atmo = isa(altitude_m, 0)
    omega = 2 * np.pi * 250 / 60.0  # Airplane mode RPM
    usable_fuel = 1500.0 - 150.0 # 1350 kg usable
    
    speeds = np.linspace(40, 120, 40)
    ranges_km = []
    
    for v in speeds:
        # Create a dummy segment just to let the planner calculate airplane-mode drag
        seg = MissionSegment("Cruise", SegmentType.CRUISE, duration_s=1, altitude_m=altitude_m, cruise_speed_mps=v, wind_mps=0, rpm=250)
        drag = planner._required_thrust_N(seg)
        target_thrust = drag / 2.0 # Split between two rotors
        
        coll, perf = planner._optimize_trim(target_thrust, omega, atmo, v_axial=v)
        if perf is not None and perf.converged:
            total_power = 2.0 * perf.power_W
            burn_rate = fuel_model.burn_rate_kg_s(total_power)
            
            # Endurance in seconds = fuel mass / mass flow rate
            endurance_s = usable_fuel / burn_rate
            
            # Range in km = (seconds * m/s) / 1000
            range_km = (endurance_s * v) / 1000.0
            ranges_km.append(range_km)
        else:
            ranges_km.append(np.nan)
            
    plt.figure(figsize=(9, 6))
    plt.plot(speeds, ranges_km, 'b-', linewidth=3, label='Maximum Cruise Range')
    
    # Identify the best range speed
    valid_idx = ~np.isnan(ranges_km)
    if any(valid_idx):
        valid_speeds = speeds[valid_idx]
        valid_ranges = np.array(ranges_km)[valid_idx]
        best_idx = np.argmax(valid_ranges)
        best_v = valid_speeds[best_idx]
        best_r = valid_ranges[best_idx]
        
        # Shade the best range region
        plt.axvspan(best_v - 4, best_v + 4, color='gold', alpha=0.3, label='Optimal Best-Range Region')
        plt.plot(best_v, best_r, 'r*', markersize=14, label=f'Peak Range ({best_r:.0f} km at {best_v:.1f} m/s)')
        
    plt.title('Cruise Range vs. Cruise Speed', fontsize=15, fontweight='bold')
    plt.suptitle(f'Conditions: {altitude_m:.0f}m Altitude, 0 m/s Wind | MTOW (7200 kg) | Usable Fuel: 1350 kg', fontsize=11, color='gray')
    plt.xlabel('Cruise Speed (m/s)', fontsize=13)
    plt.ylabel('Maximum Range (km)', fontsize=13)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='lower center', fontsize=11)
    
    plt.tight_layout()
    out_dir = os.path.abspath(os.path.join(curr_dir, '..', 'outputs'))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'cruise_range_vs_speed.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved cruise range plot to {out_path}')

if __name__ == '__main__':
    main()
