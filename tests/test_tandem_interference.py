"""
test_tandem_interference.py

Unit tests for the tandem rotor interference module.
Covers geometric overlap limits, Coleman wake limits, solver baseline
recovery, physical bounds, grid convergence, and hover auto-trim.
"""

import os
import sys
import pytest
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist, linear_twist, linear_taper_chord
from bemt import run_bemt
from tandem_interference import (
    compute_overlap_fraction,
    coleman_inflow_augmentation,
    solve_tandem_bemt,
    trim_tandem_hover,
    TandemPerformance,
)
from tandem_rotor_config import (
    build_fwd_rotor,
    build_aft_rotor,
    FWD_ROTOR_RADIUS_M,
    AFT_ROTOR_RADIUS_M,
    LONGITUDINAL_SEP_M,
    LATERAL_OFFSET_M,
    HOVER_OMEGA,
    GROSS_MASS_KG,
    G,
)


@pytest.fixture
def linear_airfoil():
    return LinearAirfoil(
        a0=5.73,
        Cd_min=0.0095,
        eps=1.15,
        stall_alpha_rad=np.radians(14.0),
        Cl_max_clip=1.45,
    )


@pytest.fixture
def test_rotors():
    return build_fwd_rotor(), build_aft_rotor()


# --- Geometric overlap ---

def test_zero_overlap_separated_disks():
    R1, R2 = 9.14, 9.14
    assert compute_overlap_fraction(R1, R2, longitudinal_sep_m=R1 + R2) == 0.0
    assert compute_overlap_fraction(R1, R2, longitudinal_sep_m=30.0) == 0.0
    assert compute_overlap_fraction(R1, R2, lateral_offset_m=25.0) == 0.0
    assert compute_overlap_fraction(R1, R2, longitudinal_sep_m=15.0, lateral_offset_m=15.0) == 0.0


def test_full_overlap_concentric_disks():
    R = 9.14
    kappa = compute_overlap_fraction(R, R, longitudinal_sep_m=0.0, lateral_offset_m=0.0)
    assert pytest.approx(kappa, abs=1e-8) == 1.0


def test_analytical_circle_intersection_hand_calc():
    # Two unit circles at d=1.0:
    # A_overlap = 2*arccos(0.5) - 0.5*sqrt(3) = 2*pi/3 - sqrt(3)/2
    # kappa = A_overlap / pi = 0.39100...
    R = 1.0
    d = 1.0
    exact_kappa = (2.0 * np.pi / 3.0 - 0.5 * np.sqrt(3.0)) / np.pi
    assert pytest.approx(compute_overlap_fraction(R, R, center_distance_m=d), abs=1e-7) == exact_kappa


def test_asymmetric_radii_overlap_geometry():
    R_fwd = 10.0
    # Smaller aft fully inside forward
    assert compute_overlap_fraction(R_fwd, 5.0, center_distance_m=0.0) == 1.0
    # Larger aft: overlap = pi*R_fwd^2 / pi*R_aft^2 = (R_fwd/R_aft)^2
    R_aft_large = 20.0
    expected = (R_fwd / R_aft_large) ** 2
    assert pytest.approx(compute_overlap_fraction(R_fwd, R_aft_large, center_distance_m=0.0), abs=1e-7) == expected


def test_geometric_overlap_bounds():
    np.random.seed(42)
    for _ in range(50):
        r1 = np.random.uniform(1.0, 15.0)
        r2 = np.random.uniform(1.0, 15.0)
        d  = np.random.uniform(0.0, 40.0)
        assert 0.0 <= compute_overlap_fraction(r1, r2, center_distance_m=d) <= 1.0


def test_invalid_radius_raises():
    with pytest.raises(ValueError):
        compute_overlap_fraction(-5.0, 9.14)
    with pytest.raises(ValueError):
        compute_overlap_fraction(9.14, 0.0)


# --- Coleman wake limits ---

def test_coleman_inflow_limits():
    R_fwd = 9.14
    v_i   = 12.0

    # Zero overlap -> no augmentation
    assert coleman_inflow_augmentation(v_i, kappa=0.0, z_sep_m=5.0, R_fwd=R_fwd) == 0.0

    # Disk plane (z=0, kappa=1): delta_v = v_i
    assert pytest.approx(coleman_inflow_augmentation(v_i, kappa=1.0, z_sep_m=0.0, R_fwd=R_fwd), rel=1e-5) == v_i

    # Far downstream (z >> R): delta_v -> 2*v_i
    assert pytest.approx(coleman_inflow_augmentation(v_i, kappa=1.0, z_sep_m=1000.0 * R_fwd, R_fwd=R_fwd), rel=1e-3) == 2.0 * v_i

    # At z=R: factor = 1 + 1/sqrt(2)
    dv_mid = coleman_inflow_augmentation(v_i, kappa=1.0, z_sep_m=R_fwd, R_fwd=R_fwd)
    assert v_i < dv_mid < 2.0 * v_i
    assert pytest.approx(dv_mid, rel=1e-4) == v_i * (1.0 + 1.0 / np.sqrt(2.0))


# --- Solver integration ---

def test_baseline_switch_recovery(test_rotors, linear_airfoil):
    """include_interference=False must exactly match a clean single-rotor solve."""
    r_fwd, r_aft = test_rotors
    atmo     = isa(0.0)
    coll_rad = np.radians(8.0)

    tandem_off = solve_tandem_bemt(
        rotor_fwd=r_fwd,
        rotor_aft=r_aft,
        airfoil_provider_fwd=lambda x: linear_airfoil,
        airfoil_provider_aft=lambda x: linear_airfoil,
        omega_fwd_rad_s=HOVER_OMEGA,
        collective_fwd_rad=coll_rad,
        collective_aft_rad=coll_rad,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        longitudinal_sep_m=LONGITUDINAL_SEP_M,
        lateral_offset_m=LATERAL_OFFSET_M,
        include_interference=False,
    )

    p_iso = run_bemt(
        rotor=r_aft,
        airfoil_provider=lambda x: linear_airfoil,
        omega_rad_s=HOVER_OMEGA,
        collective_rad=coll_rad,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        v_axial=0.0,
    )

    assert tandem_off.delta_v_axial_mps == 0.0
    assert pytest.approx(tandem_off.aft.thrust_N, rel=1e-8) == p_iso.thrust_N
    assert pytest.approx(tandem_off.aft.power_W,  rel=1e-8) == p_iso.power_W
    assert pytest.approx(tandem_off.aft.CT,        rel=1e-8) == p_iso.CT
    assert pytest.approx(tandem_off.aft.CP,        rel=1e-8) == p_iso.CP


def test_zero_overlap_recovers_isolated_aft(test_rotors, linear_airfoil):
    """Rotors separated beyond R1+R2 should give zero interference even with flag on."""
    r_fwd, r_aft = test_rotors
    atmo     = isa(0.0)
    coll_rad = np.radians(8.0)

    tandem_far = solve_tandem_bemt(
        rotor_fwd=r_fwd,
        rotor_aft=r_aft,
        airfoil_provider_fwd=lambda x: linear_airfoil,
        airfoil_provider_aft=lambda x: linear_airfoil,
        omega_fwd_rad_s=HOVER_OMEGA,
        collective_fwd_rad=coll_rad,
        collective_aft_rad=coll_rad,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        longitudinal_sep_m=40.0,   # >> 2*R
        lateral_offset_m=0.0,
        include_interference=True,
    )

    p_clean = run_bemt(
        rotor=r_aft,
        airfoil_provider=lambda x: linear_airfoil,
        omega_rad_s=HOVER_OMEGA,
        collective_rad=coll_rad,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        v_axial=0.0,
    )

    assert tandem_far.overlap_fraction == 0.0
    assert tandem_far.delta_v_axial_mps == 0.0
    assert pytest.approx(tandem_far.aft.thrust_N, rel=1e-8) == p_clean.thrust_N
    assert pytest.approx(tandem_far.aft.power_W,  rel=1e-8) == p_clean.power_W


# --- Physics trends ---

def test_interference_physics_penalty(test_rotors, linear_airfoil):
    """At equal collective, aft rotor must produce less thrust than forward due to downwash."""
    r_fwd, r_aft = test_rotors
    atmo = isa(0.0)

    res = solve_tandem_bemt(
        rotor_fwd=r_fwd,
        rotor_aft=r_aft,
        airfoil_provider_fwd=lambda x: linear_airfoil,
        airfoil_provider_aft=lambda x: linear_airfoil,
        omega_fwd_rad_s=HOVER_OMEGA,
        collective_fwd_rad=np.radians(9.0),
        collective_aft_rad=np.radians(9.0),
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        longitudinal_sep_m=LONGITUDINAL_SEP_M,
        lateral_offset_m=LATERAL_OFFSET_M,
        include_interference=True,
    )

    assert res.delta_v_axial_mps > 0.0
    assert res.aft.thrust_N < res.fwd.thrust_N
    assert res.aft.CT < res.fwd.CT


def test_dimensional_and_physical_bounds(test_rotors, linear_airfoil):
    """CT, CP, and FM should stay within plausible helicopter ranges."""
    r_fwd, r_aft = test_rotors
    atmo = isa(0.0)

    for coll_deg in [4.0, 8.0, 12.0]:
        res = solve_tandem_bemt(
            rotor_fwd=r_fwd,
            rotor_aft=r_aft,
            airfoil_provider_fwd=lambda x: linear_airfoil,
            airfoil_provider_aft=lambda x: linear_airfoil,
            omega_fwd_rad_s=HOVER_OMEGA,
            collective_fwd_rad=np.radians(coll_deg),
            collective_aft_rad=np.radians(coll_deg),
            rho=atmo.density_kg_m3,
            a_sound=atmo.speed_of_sound_mps,
            longitudinal_sep_m=LONGITUDINAL_SEP_M,
            lateral_offset_m=LATERAL_OFFSET_M,
            include_interference=True,
        )

        assert res.total_thrust_N > 0.0
        assert res.total_power_W  > 0.0
        assert 0.0 < res.total_CT < 0.015
        assert 0.0 < res.total_CP < 0.003
        if res.figure_of_merit_system is not None:
            assert 0.25 <= res.figure_of_merit_system <= 0.85


# --- Grid convergence ---

def test_spanwise_station_convergence(test_rotors, linear_airfoil):
    """N=80 vs N=120 should differ by less than 0.5% on total thrust."""
    r_fwd, r_aft = test_rotors
    atmo     = isa(0.0)
    coll_rad = np.radians(8.0)

    common = dict(
        rotor_fwd=r_fwd, rotor_aft=r_aft,
        airfoil_provider_fwd=lambda x: linear_airfoil,
        airfoil_provider_aft=lambda x: linear_airfoil,
        omega_fwd_rad_s=HOVER_OMEGA,
        collective_fwd_rad=coll_rad, collective_aft_rad=coll_rad,
        rho=atmo.density_kg_m3, a_sound=atmo.speed_of_sound_mps,
        longitudinal_sep_m=LONGITUDINAL_SEP_M, lateral_offset_m=LATERAL_OFFSET_M,
    )

    res_40  = solve_tandem_bemt(**common, n_stations=40)
    res_80  = solve_tandem_bemt(**common, n_stations=80)
    res_120 = solve_tandem_bemt(**common, n_stations=120)

    diff_40_80  = abs(res_80.total_thrust_N  - res_40.total_thrust_N)  / res_80.total_thrust_N
    diff_80_120 = abs(res_120.total_thrust_N - res_80.total_thrust_N)  / res_120.total_thrust_N

    assert diff_40_80  < 0.015
    assert diff_80_120 < 0.005


# --- Auto-trim ---

def test_trim_tandem_hover(test_rotors, linear_airfoil):
    """Trim should hit MTOW within 0.05% and require higher aft collective."""
    r_fwd, r_aft   = test_rotors
    atmo           = isa(0.0)
    target_thrust  = GROSS_MASS_KG * G

    trim_result = trim_tandem_hover(
        rotor_fwd=r_fwd,
        rotor_aft=r_aft,
        airfoil_provider_fwd=lambda x: linear_airfoil,
        airfoil_provider_aft=lambda x: linear_airfoil,
        omega_fwd_rad_s=HOVER_OMEGA,
        target_thrust_total_N=target_thrust,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        thrust_split_fwd=0.5,
        longitudinal_sep_m=LONGITUDINAL_SEP_M,
        lateral_offset_m=LATERAL_OFFSET_M,
        include_interference=True,
    )

    assert trim_result is not None
    coll_fwd_deg, coll_aft_deg, perf = trim_result

    rel_error = abs(perf.total_thrust_N - target_thrust) / target_thrust
    assert rel_error < 0.0005

    # Aft rotor must demand more collective to overcome downwash
    assert coll_aft_deg > coll_fwd_deg
    print(f"\nTrim: Fwd = {coll_fwd_deg:.2f}°, Aft = {coll_aft_deg:.2f}° (diff = {coll_aft_deg - coll_fwd_deg:.2f}°)")
