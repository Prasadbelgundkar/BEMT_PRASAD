import sys
import os
import numpy as np
import pytest

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from m2.frames import (
    R_body_to_inertial,
    R_shaft_to_body,
    R_blade_to_hub,
    transform_force_shaft_to_body,
    transform_moment_about_cg
)

def test_R_body_to_inertial_identity():
    # Zero roll, pitch, yaw -> Identity matrix
    R = R_body_to_inertial(0, 0, 0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

def test_R_shaft_to_body_helicopter_mode():
    # Helicopter mode (i_n = 90 deg)
    R = R_shaft_to_body(90)
    # X_H (aft) -> -X_b (forward is positive, so aft is -X_b) -> X_b = -X_H
    # Y_H (right) -> Y_b (right) -> Y_b = Y_H
    # Z_H (up) -> -Z_b (down is positive, so up is -Z_b) -> Z_b = -Z_H
    expected = np.array([
        [-1,  0,  0],
        [ 0,  1,  0],
        [ 0,  0, -1]
    ])
    np.testing.assert_allclose(R, expected, atol=1e-10)

def test_R_shaft_to_body_airplane_mode():
    # Airplane mode (i_n = 0 deg)
    R = R_shaft_to_body(0)
    # Z_H (thrust) points forward (+X_b)
    # X_H (aft) points down (+Z_b)
    # Y_H (right) points right (+Y_b)
    expected = np.array([
        [ 0,  0,  1],
        [ 0,  1,  0],
        [-1,  0,  0]
    ])
    np.testing.assert_allclose(R, expected, atol=1e-10)

def test_rotation_matrix_orthogonality():
    for angle in [0, 30, 45, 90, 120, -45]:
        R = R_shaft_to_body(angle)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        
        R2 = R_blade_to_hub(np.radians(angle))
        np.testing.assert_allclose(R2 @ R2.T, np.eye(3), atol=1e-10)

def test_moment_about_cg():
    F_body = np.array([100, 0, -500])  # Force in body frame
    M_hub = np.array([0, 10, 0])       # Pure pitch moment at hub
    r_hub = np.array([5, 0, -2])       # Hub is 5m forward, 2m up from CG
    
    # r x F = [ (0*-500 - -2*0), (-2*100 - 5*-500), (5*0 - 0*100) ]
    # = [0, -200 + 2500, 0] = [0, 2300, 0]
    # Total M = [0, 10, 0] + [0, 2300, 0] = [0, 2310, 0]
    
    M_cg = transform_moment_about_cg(F_body, M_hub, r_hub)
    np.testing.assert_allclose(M_cg, np.array([0, 2310, 0]))

if __name__ == "__main__":
    pytest.main([__file__])
