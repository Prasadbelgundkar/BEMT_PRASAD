import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Ensure src is in the path safely
curr_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
src_path = os.path.abspath(os.path.join(curr_dir, '..', 'src'))
sys.path.insert(0, src_path)

from mission import MissionPlanner, DesignLimits, PowerAvailableModel, FuelModel
import parameters as p
from environment import isa

def plot_fuel_burn():
    # Load configuration
    rotor = p.get_configured_rotor()
    power_model = PowerAvailableModel(sea_level_power_W=p.ENGINE_POWER_W)
    fuel_model = FuelModel(sfc_kg_per_J=p.ENGINE_SFC_KG_J)
    limits = DesignLimits(
        max_tip_mach=p.MAX_TIP_MACH, max_stall_fraction=p.MAX_STALL_FRACTION,
        min_power_margin_frac=0.0, min_rpm=200, max_rpm=1000,
        min_collective_deg=-10, max_collective_deg=85, reserve_fuel_kg=150.0
    )
    
    # We instantiate a planner just to use its handy _optimize_trim method
    planner = MissionPlanner(
        rotor=rotor, airfoil_provider=p.AIRFOIL_PROVIDER, num_rotors=p.NUM_ENGINES,
        empty_mass_kg=6000.0, fuel_mass_kg=1000.0,
        power_model=power_model, fuel_model=fuel_model,
        limits=limits, flat_plate_area_m2=p.FLAT_PLATE_AREA_M2
    )
    
    # Sweep Gross Weight from 5000 kg to 7200 kg (MTOW limit)
    weights = np.linspace(5000, 7200, 30)
    burn_rates_kgh = []
    
    atmo = isa(0, 0) # Sea Level (ISA) condition
    omega = 2 * np.pi * 550 / 60.0 # Standard Hover RPM
    
    for w in weights:
        target_thrust = (w * 9.81) / 2.0  # Thrust per rotor
        coll, perf = planner._optimize_trim(target_thrust, omega, atmo, v_axial=0.0)
        
        if perf is not None and perf.converged:
            total_power_W = 2.0 * perf.power_W
            burn_kg_s = fuel_model.burn_rate_kg_s(total_power_W)
            burn_rates_kgh.append(burn_kg_s * 3600.0) # Convert kg/s to kg/hour
        else:
            burn_rates_kgh.append(np.nan)

    # -----------------------------
    # CREATE THE PLOT
    # -----------------------------
    plt.figure(figsize=(9, 6))
    plt.plot(weights, burn_rates_kgh, 'b-', linewidth=3, label="Fuel Burn Rate")
    plt.axvline(7200, color='r', linestyle='--', linewidth=2, label="Design MTOW (7200 kg)")
    
    plt.title("Hover Fuel-Burn Rate vs. Gross Weight", fontsize=15, fontweight='bold')
    plt.xlabel("Gross Weight (kg)", fontsize=13)
    plt.ylabel("Fuel Burn Rate (kg/hr)", fontsize=13)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlim(5000, 7250) # Just enough padding to see the MTOW line clearly
             
    plt.legend(loc='lower right', fontsize=11)
    plt.tight_layout()
    
    # Save output
    out_dir = os.path.abspath(os.path.join(curr_dir, '..', 'outputs'))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'hover_fuel_burn_rate.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved hover fuel burn plot to {out_path}')

if __name__ == '__main__':
    plot_fuel_burn()
