import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from environment import isa
from m2.edgewise_bemt import run_edgewise_bemt

def main():
    # Setup V-22 like rotor
    R = 3.8
    rotor = Rotor(
        radius_m=R,
        root_cutout_m=0.5,
        num_blades=3,
        chord_fn=linear_taper_chord(0.90, 0.3888),
        twist_fn=linear_twist(np.radians(25), np.radians(-45))
    )
    
    airfoil_provider = BlendedLinearAirfoilProvider(
        stations=[0.0, 1.0],
        airfoils=[LinearAirfoil(), LinearAirfoil()]
    )
    
    atmo = isa(0) # Sea level
    
    # Forward flight case: V = 50 m/s (approx 100 kts) in airplane mode
    V_inf = 50.0
    omega = 500 * (2 * np.pi / 60) # 500 RPM
    
    print("Running Azimuth-Resolved BEMT...")
    result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=V_inf,
        omega_rad_s=omega,
        theta0_rad=np.radians(12),
        theta1c_rad=np.radians(-2),  # Small cyclic to offset rolling moment
        theta1s_rad=np.radians(3),   # Small cyclic to offset pitching moment
        alpha_shaft_rad=np.radians(5), # 5 deg nose up
        nacelle_angle_deg=0.0, # Airplane mode
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        n_r=40,
        n_psi=72
    )
    
    print(f"Converged: {result.converged}")
    print(f"Thrust: {result.T_N:.2f} N")
    print(f"Torque: {result.Q_Nm:.2f} Nm")
    print(f"Power:  {result.power_W/1000:.2f} kW")
    
    # Plotting
    r_array = np.linspace(0.5, R, 40)
    psi_array = np.linspace(0, 360, 72, endpoint=False)
    
    R_grid, PSI_grid = np.meshgrid(r_array, psi_array, indexing='ij')
    
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8, 6))
    
    # Convert psi to radians for polar plot (0 is right/aft depending on convention, we use 0=aft)
    # We want 0 degrees at bottom (aft), 90 deg right (advancing)
    ax.set_theta_zero_location("S")
    ax.set_theta_direction(-1) # Clockwise? No, 90 deg is right, so CCW is positive
    ax.set_theta_direction(1)  # CCW from South
    
    contour = ax.contourf(np.radians(PSI_grid), R_grid, result.dT_dr_dpsi, levels=20, cmap='viridis')
    plt.colorbar(contour, label='Sectional Thrust (N/m)')
    
    ax.set_title(f'Azimuthal Loading (V={V_inf} m/s)\nAdvancing Side is Right (90 deg)')
    
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/azimuthal_loading.png')
    plt.savefig(out_path, dpi=300)
    print(f"Saved plot to {out_path}")

if __name__ == "__main__":
    main()
