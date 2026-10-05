"""Regression tests for the Milestone-2 sign-convention and solver fixes."""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor
import m2.aircraft_input_m2 as CFG
from airfoil import LinearAirfoil, prandtl_glauert_correct
from environment import isa
from bemt import run_bemt
from m2.edgewise_bemt import run_edgewise_bemt
from m2.aircraft_input_m2 import get_default_aircraft
from m2.trim_solver import (shaft_angle_of_attack, compute_aircraft_residual,
                            trim_aircraft, trim_aircraft_detailed)

ROTOR = CFG.ROTOR
provider = CFG.airfoil_provider
ATM = isa(0)
RHO, A = ATM.density_kg_m3, ATM.speed_of_sound_mps
OMEGA = CFG.HOVER_OMEGA


def _edge(V=0.0, coll=22.0, t1c=0.0, t1s=0.0, a_s=0.0, nac=90.0):
    return run_edgewise_bemt(ROTOR, provider, V, OMEGA, np.radians(coll), np.radians(t1c),
                             np.radians(t1s), np.radians(a_s), nac, RHO, A, n_r=25, n_psi=36)


def test_shaft_angle_convention():
    # helicopter mode, nose UP -> freestream comes up through the disk (lam_c < 0)
    assert np.sin(shaft_angle_of_attack(np.radians(5.0), 90.0)) < 0
    # helicopter mode, nose DOWN -> flow goes down through the disk
    assert np.sin(shaft_angle_of_attack(np.radians(-5.0), 90.0)) > 0
    # airplane mode -> essentially axial
    assert abs(shaft_angle_of_attack(0.0, 0.0) - np.pi / 2) < 1e-12


def test_thrust_ahead_of_cg_pitches_nose_up():
    ac = get_default_aircraft()           # rotors 0.5 m ahead of the CG
    x = np.array([0.0, np.radians(22.0), 0.0])
    res = compute_aircraft_residual(x, 0.01, 0.0, 90.0, OMEGA, ac, ROTOR, provider, RHO, A, 'cyclic')
    assert res[2] > 0, "upward thrust ahead of the CG must give a nose-up (+My) moment"


def test_hub_moment_signs_rigid_disk():
    # more pitch on the advancing side (psi=90, +Y_hub) -> M = r x F gives +Mx_hub,
    # which is -Mx in the body frame for helicopter mode (x_b = -x_hub)
    assert _edge(t1s=2.0).moments_body_Nm[0] < -1000
    # more pitch aft (psi=0, +X_hub) -> My_hub < 0 (nose-down)
    assert _edge(t1c=2.0).moments_body_Nm[1] < -1000
    # and the sin component must NOT be a pitch control in the rigid-disk model
    assert abs(_edge(t1s=2.0).moments_body_Nm[1]) < 100


def test_hover_trim_is_physical():
    ac = get_default_aircraft()
    x = trim_aircraft(0.1, 0.0, 90.0, OMEGA, ac, ROTOR, provider, RHO, A,
                      np.array([0.0, np.radians(15.0), 0.0]), 'cyclic')
    assert x is not None
    assert abs(np.degrees(x[0])) < 2.0          # level attitude
    assert 15.0 < np.degrees(x[1]) < 30.0
    assert abs(np.degrees(x[2])) < 10.0


def test_airplane_mode_trim_matches_hand_calc():
    ac = get_default_aircraft()
    x = trim_aircraft(60.0, 0.0, 0.0, OMEGA, ac, ROTOR, provider, RHO, A,
                      np.array([np.radians(5.0), np.radians(30.0), 0.0]), 'elevator')
    assert x is not None
    # wing CL = W/(qS) = 0.82 -> alpha_wing ~ 9.5 deg -> alpha_body ~ 5.5 deg
    assert 4.0 < np.degrees(x[0]) < 7.0


def test_out_of_bounds_root_is_not_a_trim():
    ac = get_default_aircraft()
    x, status, _ = trim_aircraft_detailed(10.0, 0.0, 0.0, OMEGA, ac, ROTOR, provider, RHO, A,
                                          np.array([np.radians(5.0), np.radians(15.0), 0.0]),
                                          'elevator', verbose=False)
    assert status != 'ok'


def test_m1_descent_and_negative_loading_converge():
    for V in (0.0, -5.0, -15.0, -45.0):
        for coll in (5.0, 15.0, 22.0):
            p = run_bemt(ROTOR, provider, OMEGA, np.radians(coll), RHO, A, v_axial=V)
            assert p.converged, f"unconverged elements at V={V}, coll={coll}"
    # descent at fixed collective: thrust stays in a sane band, no negative power blow-up
    p = run_bemt(ROTOR, provider, OMEGA, np.radians(22.0), RHO, A, v_axial=-5.0)
    assert 30e3 < p.thrust_N < 45e3 and p.power_W > 0


def test_prandtl_glauert_continuous_through_mach_0p7():
    lo, _ = prandtl_glauert_correct(1.0, 0.02, 0.699)
    hi, _ = prandtl_glauert_correct(1.0, 0.02, 0.701)
    assert abs(lo - hi) < 0.01
    # and the solvers must not switch it off above M=0.7: thrust rises smoothly with RPM
    kh = Rotor(radius_m=3.8, root_cutout_m=0.5, num_blades=3,
               chord_fn=lambda x: 0.3, twist_fn=lambda x: 0.0)
    T = [run_bemt(kh, provider, 0.7 * A / 3.8 * f, np.radians(8.0), RHO, A).thrust_N / f**2
         for f in (0.97, 1.03)]
    assert abs(T[1] / T[0] - 1.0) < 0.08


def test_reverse_flow_region_carries_download():
    # high advance ratio, positive pitch: inside the reverse-flow circle the
    # onset flow hits the upper surface -> negative sectional thrust
    e = _edge(V=120.0, coll=22.0, a_s=0.0)
    r = np.linspace(0.5, 3.8, 25)
    psi = np.linspace(0, 2 * np.pi, 36, endpoint=False)
    j = int(np.argmin(abs(psi - 1.5 * np.pi)))            # psi = 270 deg
    mu = 120.0 / (OMEGA * 3.8)
    i = int(np.argmin(abs(r - 0.5 * mu * 3.8)))            # well inside the circle
    assert r[i] < mu * 3.8
    assert e.dT_dr_dpsi[i, j] < 0
    assert e.reverse_flow_fraction > 0


if __name__ == "__main__":
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v(); print("ok", k)
