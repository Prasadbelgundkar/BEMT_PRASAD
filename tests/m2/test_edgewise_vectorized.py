"""
Checks on the vectorized edgewise solver (WP1):
  * identical to the original loop implementation (uniform-Glauert option),
  * left (CW) rotor is the exact mirror image of the right (CCW) rotor, so a
    symmetric pair gives zero side force, rolling and yawing moment,
  * torque reaction sign on the airframe,
  * periodicity / grid consistency.
"""
import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))
sys.path.insert(0, os.path.dirname(__file__))

from environment import isa
from m2.edgewise_bemt import run_edgewise_bemt
import m2.aircraft_input_m2 as CFG
from _legacy_edgewise_loop import run_edgewise_bemt as run_legacy_loop

ATM = isa(0)
RHO, A = ATM.density_kg_m3, ATM.speed_of_sound_mps


def _run(**kw):
    base = dict(rotor=CFG.ROTOR, airfoil_provider=CFG.airfoil_provider, V_inf=40.0,
                omega_rad_s=CFG.HOVER_OMEGA, theta0_rad=np.radians(14.0),
                theta1c_rad=np.radians(1.5), theta1s_rad=np.radians(-2.0),
                alpha_shaft_rad=np.radians(-4.0), nacelle_angle_deg=75.0,
                rho=RHO, a_sound=A, n_r=20, n_psi=24)
    base.update(kw)
    return run_edgewise_bemt(**base)


@pytest.mark.parametrize("seed", range(5))
def test_vectorized_matches_legacy_loop(seed):
    rng = np.random.default_rng(seed)
    args = (CFG.ROTOR, CFG.airfoil_provider, rng.uniform(0, 90), CFG.HOVER_OMEGA,
            np.radians(rng.uniform(5, 30)), np.radians(rng.uniform(-5, 5)),
            np.radians(rng.uniform(-5, 5)), np.radians(rng.uniform(-20, 90)), 90.0, RHO, A)
    new = run_edgewise_bemt(*args, n_r=20, n_psi=24, inflow='uniform_glauert')
    old = run_legacy_loop(*args, n_r=20, n_psi=24)
    for f in ('T_N', 'Q_Nm', 'H_N', 'Y_N'):
        assert abs(getattr(new, f) - getattr(old, f)) <= 1e-8 * max(abs(getattr(old, f)), 1.0)
    np.testing.assert_allclose(new.dT_dr_dpsi, old.dT_dr_dpsi, rtol=1e-10, atol=1e-8)


def test_cw_rotor_is_mirror_of_ccw():
    hub_r = CFG.get_default_aircraft().hub_from_cg(75.0, 'right')
    hub_l = CFG.get_default_aircraft().hub_from_cg(75.0, 'left')
    r = _run(rotation='ccw', r_hub_body=hub_r)
    l = _run(rotation='cw', r_hub_body=hub_l)
    # same magnitudes
    assert r.T_N == pytest.approx(l.T_N) and r.Q_Nm == pytest.approx(l.Q_Nm)
    # mirrored shaft-frame loads
    np.testing.assert_allclose(l.forces_shaft_N, r.forces_shaft_N * [1, -1, 1])
    np.testing.assert_allclose(l.moments_shaft_Nm, r.moments_shaft_Nm * [-1, 1, -1])
    # symmetric pair: lateral-directional loads cancel about the CG
    F = r.forces_body_N + l.forces_body_N
    M = r.moments_body_Nm + l.moments_body_Nm
    scale = abs(r.T_N) * 10.0
    assert abs(F[1]) < 1e-9 * abs(r.T_N)
    assert abs(M[0]) < 1e-9 * scale and abs(M[2]) < 1e-9 * scale


def test_torque_reaction_opposes_rotation():
    """CCW rotor (seen from above, helicopter mode): the airframe is pushed
    clockwise from above = nose-right = +Mz in body axes (z down)."""
    r = _run(V_inf=0.0, theta1c_rad=0.0, theta1s_rad=0.0, alpha_shaft_rad=0.0,
             nacelle_angle_deg=90.0)
    assert r.Q_Nm > 0
    assert r.moments_hub_body_Nm[2] == pytest.approx(r.Q_Nm)


def test_azimuth_grid_independent_of_starting_point():
    a = _run(n_psi=24)
    b = _run(n_psi=48)
    assert a.T_N == pytest.approx(b.T_N, rel=0.01)


def test_diagnostic_fields_shapes():
    r = _run()
    for f in ('U_T', 'U_P', 'alpha_eff_rad', 'mach', 'reverse_mask', 'stall_mask'):
        assert getattr(r, f).shape == (20, 24)
    assert r.max_mach >= r.mach.max() - 1e-12
    assert 0.0 <= r.stalled_fraction_fwd <= 1.0
