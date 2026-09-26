import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from environment import isa
from m2.aircraft_input_m2 import get_default_aircraft
from m2.conversion_corridor import compute_conversion_corridor

def main():
    aircraft = get_default_aircraft()
    rotor = Rotor(
        radius_m=3.8,
        root_cutout_m=0.5,
        num_blades=3,
        chord_fn=linear_taper_chord(0.90, 0.3888),
        twist_fn=linear_twist(np.radians(25), np.radians(-45))
    )
    
    airfoil_provider = BlendedLinearAirfoilProvider(
        stations=[0.0, 1.0],
        airfoils=[LinearAirfoil(), LinearAirfoil()]
    )
    
    atmo = isa(0)
    
    # Coarse grid for fast demonstration
    # Nacelle: 90 (helicopter) down to 0 (airplane)
    nacelle_sweep = np.linspace(90, 0, 10)
    # Airspeed: 0 (hover) up to 80 m/s (~155 knots)
    V_sweep = np.linspace(0.1, 80.0, 15)
    
    omega_rad_s = 500 * (2 * np.pi / 60)
    
    print("Mapping Conversion Corridor...")
    print(f"Grid: {len(nacelle_sweep)} Nacelle angles x {len(V_sweep)} Airspeeds = {len(nacelle_sweep) * len(V_sweep)} points")
    
    results = compute_conversion_corridor(
        V_sweep=V_sweep,
        nacelle_sweep=nacelle_sweep,
        aircraft=aircraft,
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        omega_rad_s=omega_rad_s
    )
    
    # Process results into grids for plotting
    V_grid, N_grid = np.meshgrid(V_sweep, nacelle_sweep)
    
    success_grid = np.zeros_like(V_grid, dtype=bool)
    power_grid = np.zeros_like(V_grid)
    alpha_grid = np.zeros_like(V_grid)
    rotor_stall_grid = np.zeros_like(V_grid)
    wing_stall_grid = np.zeros_like(V_grid, dtype=bool)
    
    MAX_POWER_KW = 4000.0  # arbitrary limit for the corridor
    
    idx = 0
    for i in range(len(nacelle_sweep)):
        for j in range(len(V_sweep)):
            r = results[idx]
            
            # Criteria for valid corridor:
            # 1. Solved for trim
            # 2. Power within limits
            # 3. Alpha not totally insane (-15 to +20)
            # 4. Wing not stalled
            
            if r.success:
                valid = (r.power_kW < MAX_POWER_KW) and (-15 < r.alpha_deg < 20) and (not r.wing_stalled)
                success_grid[i, j] = valid
                power_grid[i, j] = r.power_kW
                alpha_grid[i, j] = r.alpha_deg
                rotor_stall_grid[i, j] = r.rotor_stalled_fraction
                wing_stall_grid[i, j] = r.wing_stalled
            else:
                success_grid[i, j] = False
                
            idx += 1

    # Plotting
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot background points
    # Red X = Invalid/Failed Trim. Green dot = Valid Corridor
    for i in range(len(nacelle_sweep)):
        for j in range(len(V_sweep)):
            if success_grid[i, j]:
                ax.scatter(V_grid[i, j], N_grid[i, j], color='green', marker='o', s=50)
            else:
                ax.scatter(V_grid[i, j], N_grid[i, j], color='red', marker='x', s=30)
                
    # Contour lines for Power
    # Mask power grid where it failed
    power_masked = np.ma.masked_where(~success_grid, power_grid)
    if np.any(success_grid):
        CS = ax.contour(V_grid, N_grid, power_masked, levels=[500, 1000, 1500, 2000, 3000], colors='blue', alpha=0.5)
        ax.clabel(CS, inline=True, fontsize=8, fmt='%d kW')
    
    # Add an ideal transition path (example)
    # V_stall is approx 40 m/s for this weight. 
    # Schedule: i_n = 90 at V=0, i_n = 0 at V=60.
    V_path = np.linspace(0, 60, 20)
    N_path = 90 * (1 - (V_path/60.0)**1.5)
    ax.plot(V_path, N_path, 'k--', linewidth=2, label='Ideal Transition Schedule')
    
    ax.set_xlabel('Airspeed (m/s)')
    ax.set_ylabel('Nacelle Angle (deg)')
    ax.set_title('Tiltrotor Conversion Corridor Map\n(Green = Safe Trim, Red = Stall/Overpower/Failed Trim)')
    ax.grid(True, linestyle=':')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/corridor_map.png')
    plt.savefig(out_path, dpi=300)
    print(f"\nSaved corridor map to {out_path}")

if __name__ == "__main__":
    main()
