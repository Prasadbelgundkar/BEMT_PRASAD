import sys
import os
import numpy as np

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
    omega_rad_s = 500 * (2 * np.pi / 60)
    
    print("Attempting to trim in an IMPOSSIBLE state: V = 10 m/s, Airplane Mode (Nacelle = 0)")
    print("The wings cannot generate enough lift at 10 m/s, and the rotors are pointing forward.")
    print("Expect the solver to either fail to converge, or require massive impossible alpha angles.\n")
    
    # Baseline trim state guess
    x0 = np.array([np.radians(5.0), np.radians(15.0), 0.0])
    
    # Warning message will be printed by the trim_solver if it fails.
    x_trim = trim_aircraft(
        V_inf=10.0, 
        gamma_rad=0.0,
        nacelle_deg=0.0, # Airplane mode
        omega_rad_s=omega_rad_s,
        aircraft=aircraft,
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        rho=rho,
        a_sound=a_sound,
        x0=x0,
        trim_pitch_with='elevator'
    )
    
    print("\nResult:")
    print(f"Alpha: {np.degrees(x_trim[0]):.2f} deg")
    print(f"Collective: {np.degrees(x_trim[1]):.2f} deg")
    print(f"Elevator: {np.degrees(x_trim[2]):.2f} deg")
    
    if np.degrees(x_trim[0]) > 40.0:
        print("-> Notice the huge Alpha! The aircraft is trying to pitch its nose up 40+ degrees to point the rotors upwards and generate vertical thrust.")

if __name__ == "__main__":
    main()
