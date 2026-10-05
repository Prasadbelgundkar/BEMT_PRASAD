import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

import m2.aircraft_input_m2 as CFG
from environment import isa
from m2.aircraft_input_m2 import get_default_aircraft
from m2.trim_solver import compute_aircraft_residual

def main():
    aircraft = get_default_aircraft()
    rotor = CFG.ROTOR
    
    airfoil_provider = CFG.airfoil_provider
    
    atmo = isa(0)
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps
    
    # We will simulate a mid-transition state:
    V_inf = 30.0          # 30 m/s airspeed
    nacelle_deg = 60.0    # 60 degrees tilt
    gamma_rad = 0.0       # level flight path
    omega_rad_s = CFG.HOVER_OMEGA
    
    # Baseline trim state guess (alpha=5 deg, collective=15 deg, elevator=0)
    alpha_body_rad = np.radians(5.0)
    collective_rad = np.radians(15.0)
    
    # Sweep elevator from -20 to +20 degrees
    elevator_sweep_deg = np.linspace(-20, 20, 21)
    
    res_X_list = []
    res_Z_list = []
    res_M_list = []
    
    print(f"Sweeping Elevator Deflection at V={V_inf} m/s, Nacelle={nacelle_deg} deg...")
    
    for delta_e_deg in elevator_sweep_deg:
        x = np.array([alpha_body_rad, collective_rad, np.radians(delta_e_deg)])
        
        # We pass 'elevator' so that the 3rd state variable acts as delta_e
        res = compute_aircraft_residual(
            x, V_inf, gamma_rad, nacelle_deg, omega_rad_s,
            aircraft, rotor, airfoil_provider, rho, a_sound,
            trim_pitch_with='elevator'
        )
        
        # Un-normalize residuals to get actual force/moment values
        res_X_list.append(res[0] * aircraft.W_MTOW_N)
        res_Z_list.append(res[1] * aircraft.W_MTOW_N)
        res_M_list.append(res[2] * aircraft.W_MTOW_N)
        
    # Plotting
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    
    fig, ax1 = plt.subplots(figsize=(8, 5))
    
    color = 'tab:blue'
    ax1.set_xlabel('Elevator Deflection (deg)')
    ax1.set_ylabel('Pitching Moment Residual (Nm)', color=color)
    ax1.plot(elevator_sweep_deg, res_M_list, color=color, linewidth=2, label='Pitching Moment')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True)
    
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Z Force Residual (N)', color=color)
    ax2.plot(elevator_sweep_deg, res_Z_list, color=color, linestyle='--', label='Z Force (Lift)')
    ax2.tick_params(axis='y', labelcolor=color)
    
    plt.title('Aircraft Pitch Moment vs Elevator Deflection\n(Demonstrating Control Authority)')
    fig.tight_layout()
    
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/control_sweep.png')
    plt.savefig(out_path, dpi=300)
    print(f"Saved control sweep plot to {out_path}")

if __name__ == "__main__":
    main()
