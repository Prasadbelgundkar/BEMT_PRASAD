"""Tests for the 6-DOF trim (WP5): convergence, symmetric lateral residuals,
status classification and the failure diagnosis."""
import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

import m2.aircraft_input_m2 as CFG
from m2.trim_6dof import trim_6dof, make_condition, evaluate, diagnose, RES_TOL

AC = CFG.get_default_aircraft()


def _trim(V, nac, **kw):
    rpm = CFG.AIRPLANE_RPM if nac == 0 else CFG.HOVER_RPM
    return trim_6dof(AC, CFG.ROTOR, CFG.airfoil_provider, make_condition(V, nac, rpm=rpm, **kw))


@pytest.mark.parametrize("V,nac", [(0.0, 90.0), (45.0, 60.0), (85.0, 0.0)])
def test_trim_converges_with_all_six_residuals(V, nac):
    tr = _trim(V, nac)
    assert tr.status.startswith('ok'), tr.status
    W = AC.W_MTOW_N
    assert np.all(np.abs(tr.F_res_N) < 10.0 * RES_TOL * W)
    assert np.all(np.abs(tr.M_res_Nm) < 10.0 * RES_TOL * W)


def test_symmetric_flight_has_zero_lateral_controls():
    tr = _trim(45.0, 60.0)
    assert abs(tr.phi_deg) < 0.05 and abs(tr.dcoll_deg) < 0.05 and abs(tr.dcyc_deg) < 0.05
    assert abs(tr.aileron_deg) < 0.5 and abs(tr.rudder_deg) < 0.5


def test_lateral_controls_have_authority_in_helicopter_mode():
    """Differential collective -> roll, differential theta1s -> yaw (both nonzero)."""
    cond = make_condition(20.0, 90.0)
    x = np.array([np.radians(-3.0), 0.0, np.radians(24.0), 0.3, 0.0, 0.0])
    r0 = evaluate(x, AC, CFG.ROTOR, CFG.airfoil_provider, cond)
    r_lat = evaluate(x + [0, 0, 0, 0, 0.2, 0], AC, CFG.ROTOR, CFG.airfoil_provider, cond)
    r_ped = evaluate(x + [0, 0, 0, 0, 0, 0.2], AC, CFG.ROTOR, CFG.airfoil_provider, cond)
    assert r_lat[3] - r0[3] < -1e-3, "+d_lat must roll left (MX < 0)"
    assert r_ped[5] - r0[5] < -1e-3, "+d_ped must yaw nose-left (MZ < 0)"


def test_airplane_mode_too_slow_is_not_a_trim():
    tr = _trim(40.0, 0.0)
    assert not tr.status.startswith('ok')
    assert 'wing' in diagnose(tr)


def test_collective_saturation_is_reported():
    tr = _trim(140.0, 0.0)   # beyond the 125 m/s design dash at airplane-mode cruise RPM
    assert tr.status == 'control_saturation' and 'theta0' in tr.at_bound


def test_power_flag_hot_and_high():
    tr = _trim(0.0, 90.0, altitude_m=4000.0, dISA_K=20.0)   # above the ISA+20 hover ceiling (~3900 m)
    assert tr.status.startswith('ok') and 'power' in tr.flags and not tr.feasible
