import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

import m2.aircraft_input_m2 as CFG
from environment import isa
from m2.edgewise_bemt import run_edgewise_bemt

def main():
    # Setup V-22 like rotor
    R = CFG.ROTOR.radius_m
    rotor = CFG.ROTOR
    
    airfoil_provider = CFG.airfoil_provider
    
    atmo = isa(0) # Sea level
    
    # Edgewise (helicopter-mode) forward flight: V = 50 m/s, mu ~ 0.25.
    # alpha_shaft = 5 deg means the disk is tilted 5 deg nose-down into the wind
    # (flow passes DOWN through the disk); nacelle = 90 deg (helicopter mode).
    V_inf = 50.0
    omega = CFG.HOVER_OMEGA # 500 RPM
    
    print("Running Azimuth-Resolved BEMT...")
    result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=V_inf,
        omega_rad_s=omega,
        theta0_rad=np.radians(20),
        theta1c_rad=np.radians(-2),  # cos(psi) cyclic: fore/aft loading (pitch moment, rigid disk)
        theta1s_rad=np.radians(-3),  # sin(psi) cyclic: unload advancing side (roll moment, rigid disk)
        alpha_shaft_rad=np.radians(5), # disk 5 deg nose-down
        nacelle_angle_deg=90.0, # Helicopter mode
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
    plt.colorbar(contour, label='Sectional thrust dT/dr, all 3 blades (N/m)')
    
    ax.set_title(f'Azimuthal Loading (V={V_inf} m/s)\nAdvancing Side is Right (90 deg)')
    
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/azimuthal_loading.png')
    plt.savefig(out_path, dpi=300)
    print(f"Saved plot to {out_path}")

if __name__ == "__main__":
    main()
