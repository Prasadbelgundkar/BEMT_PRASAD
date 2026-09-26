import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from environment import isa
from m2.aircraft_input_m2 import get_default_aircraft
from m2.trim_solver import trim_aircraft

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
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps
    
    # We will pick a few combinations of V and Nacelle
    test_points = [
        (0.1, 90.0, 'cyclic'),    # Hover
        (15.0, 75.0, 'cyclic'),   # Early transition
        (30.0, 45.0, 'elevator'), # Mid transition (wind is fast enough for elevator)
        (50.0, 0.0, 'elevator')   # Airplane mode
    ]
    
    omega_rad_s = 500 * (2 * np.pi / 60)
    
    # Initial guess: alpha=0, coll=15 deg, ctrl=0
    x0 = np.array([0.0, np.radians(15.0), 0.0])
    
    print("Running Trim Matrix...")
    print(f"{'V (m/s)':>8} | {'Nacelle':>8} | {'Alpha(deg)':>10} | {'Coll(deg)':>10} | {'Ctrl(deg)':>10} | {'Ctrl Type'}")
    print("-" * 75)
    
    for (V, nacelle, ctrl_type) in test_points:
        try:
            x_trim = trim_aircraft(
                V_inf=V,
                gamma_rad=0.0,
                nacelle_deg=nacelle,
                omega_rad_s=omega_rad_s,
                aircraft=aircraft,
                rotor=rotor,
                airfoil_provider=airfoil_provider,
                rho=rho,
                a_sound=a_sound,
                x0=x0,
                trim_pitch_with=ctrl_type
            )
            
            alpha_deg = np.degrees(x_trim[0])
            coll_deg = np.degrees(x_trim[1])
            ctrl_deg = np.degrees(x_trim[2])
            
            print(f"{V:8.1f} | {nacelle:8.1f} | {alpha_deg:10.2f} | {coll_deg:10.2f} | {ctrl_deg:10.2f} | {ctrl_type}")
            
            # Use this trim state as the seed for the next iteration to speed up convergence
            x0 = x_trim
            
        except Exception as e:
            print(f"{V:8.1f} | {nacelle:8.1f} | FAILED TO TRIM: {e}")

if __name__ == "__main__":
    main()
