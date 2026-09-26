import sys
import os
import numpy as np
import pytest

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_twist, constant_chord, constant_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from bemt import run_bemt as run_bemt_m1
from environment import isa

from m2.edgewise_bemt import run_edgewise_bemt

def test_m1_recovery_hover():
    """
    Tests that the Phase 2 edgewise BEMT matches the Phase 1 axisymmetric BEMT
    under hover conditions (V=0, cyclic=0).
    """
    # 1. Setup aircraft parameters
    R = 3.8
    root_cutout = 0.5
    B = 3
    omega = 500 * (2 * np.pi / 60)
    theta0_deg = 12.0
    theta0_rad = np.radians(theta0_deg)
    
    rotor = Rotor(
        radius_m=R,
        root_cutout_m=root_cutout,
        num_blades=B,
        chord_fn=constant_chord(0.35),
        twist_fn=constant_twist(0.0)
    )
    
    airfoil_provider = BlendedLinearAirfoilProvider(
        stations=[0.0, 1.0],
        airfoils=[LinearAirfoil(), LinearAirfoil()]
    )
    
    atmo = isa(0)
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps
    
    # 2. Run M1 BEMT
    m1_result = run_bemt_m1(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        omega_rad_s=omega,
        collective_rad=theta0_rad,
        rho=rho,
        a_sound=a_sound,
        v_axial=0.0,
        n_stations=40
    )
    
    # 3. Run M2 Edgewise BEMT
    m2_result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=0.0,
        omega_rad_s=omega,
        theta0_rad=theta0_rad,
        theta1c_rad=0.0,
        theta1s_rad=0.0,
        alpha_shaft_rad=0.0,
        nacelle_angle_deg=90.0,
        rho=rho,
        a_sound=a_sound,
        n_r=40,
        n_psi=72
    )
    
    # 4. Assertions
    # Check that M2 converged
    assert m2_result.converged, "M2 BEMT did not converge!"
    
    # Check Thrust
    thrust_err = abs(m2_result.T_N - m1_result.thrust_N) / m1_result.thrust_N
    assert thrust_err < 0.10, f"Thrust mismatch! M1: {m1_result.thrust_N:.2f}, M2: {m2_result.T_N:.2f}"
    
    # Check Torque
    torque_err = abs(m2_result.Q_Nm - m1_result.torque_Nm) / m1_result.torque_Nm
    assert torque_err < 0.10, f"Torque mismatch! M1: {m1_result.torque_Nm:.2f}, M2: {m2_result.Q_Nm:.2f}"
    
    # Check Power
    power_err = abs(m2_result.power_W - m1_result.power_W) / m1_result.power_W
    assert power_err < 0.10, f"Power mismatch! M1: {m1_result.power_W:.2f}, M2: {m2_result.power_W:.2f}"
    
    # In pure hover, H and Y forces should be effectively zero
    assert abs(m2_result.H_N) < 1.0, f"H force not zero in hover: {m2_result.H_N}"
    assert abs(m2_result.Y_N) < 1.0, f"Y force not zero in hover: {m2_result.Y_N}"

if __name__ == "__main__":
    pytest.main([__file__])
