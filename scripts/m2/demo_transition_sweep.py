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
    # Aircraft Parameters
    W_MTOW = 7200 * 9.81  # Newtons
    S_wing = 39.24        # m^2 (from M1 context)
    CL_max = 1.4
    
    # Rotor Parameters (from M1)
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
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps
    
    # 1. Calculate Stall Speed (upper bound for transition sweep)
    # V_stall is the speed where the wing can carry the FULL aircraft weight.
    V_stall = np.sqrt(2.0 * W_MTOW / (rho * S_wing * CL_max))
    print(f"Calculated Wing Stall Speed: {V_stall:.2f} m/s")
    
    # We will sweep up to 1.1 * V_stall to show completion of transition
    V_sweep = np.linspace(0.0, V_stall * 1.1, 20)
    
    # Nacelle angle schedule (90 deg at hover, 0 deg at 1.1*V_stall)
    # A typical tiltrotor uses a roughly linear or quadratic schedule vs speed.
    i_n_schedule = 90.0 * (1.0 - (V_sweep / (V_stall * 1.1))**1.5)
    
    # Rotor RPM schedule (constant 500 RPM for transition)
    omega = 500 * (2 * np.pi / 60)
    
    # Logs
    T_list = []
    H_list = []
    Q_list = []
    P_list = []
    mu_list = []
    stall_list = []
    rev_flow_list = []
    mach_list = []
    
    print("\nRunning Transition Sweep...")
    print(f"{'V (m/s)':>8} | {'Nacelle':>8} | {'mu':>6} | {'T (N)':>8} | {'H (N)':>8} | {'P (kW)':>8} | {'Stall%':>6} | {'Rev%':>6}")
    print("-" * 75)
    
    for i, V in enumerate(V_sweep):
        nacelle_deg = i_n_schedule[i]
        
        # In a real transition, collective and cyclic are actively trimmed.
        # As the nacelle tilts forward, the axial inflow increases drastically.
        # To maintain positive thrust and not stall negatively, the collective must INCREASE 
        # heavily (acting like a high-pitch airplane propeller).
        # We start at ~12 deg in hover and increase to ~35 deg in airplane mode.
        collective_deg = 12.0 + 23.0 * (V / (V_stall * 1.1))
        
        # alpha_shaft is the aerodynamic angle of attack of the rotor disk relative to freestream.
        # If aircraft is level (alpha_body = 0), then when nacelle is 90 deg (helicopter), 
        # the disk is horizontal, so alpha_shaft = 0 (edgewise).
        # When nacelle is 0 deg (airplane), the disk is vertical, so alpha_shaft = 90 (axial).
        alpha_shaft_deg = 90.0 - nacelle_deg
        alpha_shaft_rad = np.radians(alpha_shaft_deg)
        
        # Advance ratio
        mu = (V * np.cos(alpha_shaft_rad)) / (omega * R)
        
        result = run_edgewise_bemt(
            rotor=rotor,
            airfoil_provider=airfoil_provider,
            V_inf=V,
            omega_rad_s=omega,
            theta0_rad=np.radians(collective_deg),
            theta1c_rad=np.radians(0), # No cyclic for this basic sweep
            theta1s_rad=np.radians(0),
            alpha_shaft_rad=alpha_shaft_rad,
            nacelle_angle_deg=nacelle_deg,
            rho=rho,
            a_sound=a_sound,
            n_r=40,
            n_psi=72,
            max_glauert_iter=15
        )
        
        T_list.append(result.T_N)
        H_list.append(result.H_N)
        Q_list.append(result.Q_Nm)
        P_list.append(result.power_W)
        mu_list.append(mu)
        stall_list.append(result.stalled_fraction * 100)
        rev_flow_list.append(result.reverse_flow_fraction * 100)
        mach_list.append(result.adv_tip_mach)
        
        print(f"{V:8.2f} | {nacelle_deg:8.2f} | {mu:6.3f} | {result.T_N:8.0f} | {result.H_N:8.0f} | {result.power_W/1000:8.1f} | {result.stalled_fraction*100:6.1f} | {result.reverse_flow_fraction*100:6.1f}")
        
    # Plotting
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    
    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    
    # 1. Forces
    axs[0].plot(V_sweep, T_list, 'b-', label='Thrust (Z-axis)')
    axs[0].plot(V_sweep, H_list, 'r-', label='H-Force (X-axis drag)')
    axs[0].set_ylabel('Force (N)')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].set_title('Rotor Forces during Transition (Untrimmed)')
    
    # 2. Power
    P_kW = np.array(P_list) / 1000.0
    axs[1].plot(V_sweep, P_kW, 'g-', label='Rotor Power')
    axs[1].set_ylabel('Power (kW)')
    axs[1].grid(True)
    
    # 3. Aerodynamic conditions (Stall & Reverse Flow)
    axs[2].plot(V_sweep, stall_list, 'm-', label='Blade Stalled %')
    axs[2].plot(V_sweep, rev_flow_list, 'c-', label='Reverse Flow %')
    axs[2].set_xlabel('Airspeed V (m/s)')
    axs[2].set_ylabel('Disk Area Fraction (%)')
    axs[2].legend()
    axs[2].grid(True)
    
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/transition_sweep.png')
    plt.savefig(out_path, dpi=300)
    print(f"\nSaved transition plots to {out_path}")

if __name__ == "__main__":
    main()
