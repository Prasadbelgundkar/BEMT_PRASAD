import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from environment import isa
from m2.aircraft_input_m2 import get_default_aircraft
from m2.trim_solver import compute_aircraft_residual, trim_aircraft

def test_trim_hover():
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
    
    # Realistic CG
    aircraft.cg_to_wing_ac_m[0] = 0.0
    aircraft.cg_to_htail_ac_m[0] = -6.0
    aircraft.cg_to_right_rotor_m[0] = 0.5
    aircraft.cg_to_left_rotor_m[0] = 0.5
    
    atmo = isa(0)
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps
    
    # Trim in hover (V=0)
    # Pitch should be trimmed using cyclic (theta1s) because elevator has no q in hover
    x0 = np.array([0.0, np.radians(12.0), 0.0]) # alpha, coll, cyclic
    
    x_trim = trim_aircraft(
        V_inf=0.01, # slightly > 0 to avoid singularity in mu
        gamma_rad=0.0,
        nacelle_deg=90.0,
        omega_rad_s=500 * (2 * np.pi / 60),
        aircraft=aircraft,
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        rho=rho,
        a_sound=a_sound,
        x0=x0,
        trim_pitch_with='cyclic'
    )
    
    alpha_deg = np.degrees(x_trim[0])
    coll_deg = np.degrees(x_trim[1])
    cyclic_deg = np.degrees(x_trim[2])
    
    print(f"Hover Trim: Alpha={alpha_deg:.2f} deg, Coll={coll_deg:.2f} deg, Cyclic={cyclic_deg:.2f} deg")
    
    res = compute_aircraft_residual(
        x_trim, 0.01, 0.0, 90.0, 500 * (2 * np.pi / 60),
        aircraft, rotor, airfoil_provider, rho, a_sound, 'cyclic'
    )
    
    assert np.all(np.abs(res) < 1e-3), f"Residuals not zero: {res}"

if __name__ == "__main__":
    test_trim_hover()
