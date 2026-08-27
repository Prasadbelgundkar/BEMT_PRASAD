import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

curr_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
src_path = os.path.abspath(os.path.join(curr_dir, '..', 'src'))
sys.path.insert(0, src_path)

from mission import MissionPlanner, DesignLimits, PowerAvailableModel, FuelModel
import parameters as p
from environment import isa

def get_burn_rate(planner, fuel_model, weight, omega, atmo):
    target_thrust = (weight * 9.81) / 2.0
    coll, perf = planner._optimize_trim(target_thrust, omega, atmo, v_axial=0.0)
    if perf is not None and perf.converged:
        return fuel_model.burn_rate_kg_s(2.0 * perf.power_W)
    return None

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
    
    atmo = isa(0, 0)
    omega = 2 * np.pi * 550 / 60.0
    usable_fuel = 1500.0 - 150.0
    
    # Evaluate endurance from Empty+Fuel (6000kg) to Overweight (7500kg)
    weights = np.linspace(6000, 7500, 30)
    endurances = []
    
    for w in weights:
        br = get_burn_rate(planner, fuel_model, w, omega, atmo)
        if br is not None:
            # Endurance = Usable Fuel / Average Burn Rate (using initial burn rate as standard estimate)
            # A more precise integration would account for aircraft getting lighter, but this is a standard plot.
            time_s = usable_fuel / br
            endurances.append(time_s / 3600.0)
        else:
            endurances.append(np.nan)
            
    plt.figure(figsize=(9, 6))
    
    # Identify feasible range (6000 to 7200)
    plt.axvspan(6000, 7200, color='lightgreen', alpha=0.3, label='Feasible Payload Range')
    plt.axvspan(7200, 7500, color='lightcoral', alpha=0.3, label='Overweight (Infeasible)')
    
    plt.plot(weights, endurances, 'b-', linewidth=3, label='Hover Endurance')
    plt.axvline(7200, color='red', linestyle='--', linewidth=2, label='Design MTOW (7200 kg)')
    
    plt.title('Hover Endurance vs. Takeoff Weight', fontsize=15, fontweight='bold')
    plt.suptitle('Conditions: Sea Level (ISA), 550 RPM | Full Fuel (1500 kg) with 150 kg Reserve', fontsize=11, color='gray')
    
    plt.xlabel('Takeoff Gross Weight (kg)', fontsize=13)
    plt.ylabel('Hover Endurance (Hours)', fontsize=13)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlim(6000, 7500)
    plt.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    out_dir = os.path.abspath(os.path.join(curr_dir, '..', 'outputs'))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'hover_endurance_vs_weight.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved endurance plot to {out_path}')

if __name__ == '__main__':
    main()
