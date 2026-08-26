"""
test_bemt.py
------------
Comprehensive pytest test suite for the tiltrotor BEMT tool.

Run with:
    cd "f:\Rotory Wing\tiltrotor_bemt\tiltrotor_bemt"
    python -m pytest test_bemt.py -v

Coverage:
  - ISA atmosphere accuracy vs. ICAO standard values
  - LinearAirfoil and TableAirfoil (incl. stall, boundary edges)
  - Prandtl-Glauert correction (Cl AND Cd)
  - Prandtl tip-loss and root-loss factors
  - BEMT hover convergence and physics sanity (FM < 1, CT > 0, CP > 0)
  - BEMT axial (propeller) forward-flight
  - Element-level and rotor-level convergence flags
  - Advance-ratio helper
  - Mission planner: feasible mission, fuel reserve breach, power margin,
    tip-Mach limit, stall limit, RPM limit, collective limit, payload event
  - Rotor geometry: solidity, disk area, tip speed, tip Mach
"""
import sys
import os
import pytest
import numpy as np

# Ensure the package root is on the path when running from any directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment import isa, AtmoState
from airfoil import LinearAirfoil, TableAirfoil, prandtl_glauert_correct
from rotor import Rotor, constant_chord, constant_twist, linear_twist, linear_taper_chord
from bemt import run_bemt, advance_ratio, prandtl_tip_loss, ElementResult, RotorPerformance
from mission import (
    MissionPlanner, MissionSegment, SegmentType,
    PowerAvailableModel, FuelModel, DesignLimits, MissionInfeasibleError,
)

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def linear_airfoil():
    return LinearAirfoil()


@pytest.fixture
def kh_rotor():
    return Rotor(
        radius_m=0.762, root_cutout_m=0.125, num_blades=2,
        chord_fn=constant_chord(0.0508),
        twist_fn=constant_twist(0.0),
        name="Knight-Hefner",
    )


@pytest.fixture
def kh_omega():
    return 2 * np.pi * 1000.0 / 60.0


@pytest.fixture
def sea_level():
    return isa(0.0, 0.0)


# ---------------------------------------------------------------------------
# 1. ISA atmosphere
# ---------------------------------------------------------------------------

class TestISA:
    def test_sea_level_standard_values(self):
        s = isa(0.0, 0.0)
        assert abs(s.temperature_K - 288.15) < 0.01
        assert abs(s.pressure_Pa - 101325.0) < 1.0
        assert abs(s.density_kg_m3 - 1.225) < 0.001
        assert abs(s.speed_of_sound_mps - 340.29) < 0.1

    def test_tropopause_temperature(self):
        s = isa(11000.0, 0.0)
        assert abs(s.temperature_K - 216.65) < 0.1

    def test_isothermal_stratosphere(self):
        s11 = isa(11000.0, 0.0)
        s15 = isa(15000.0, 0.0)
        assert abs(s15.temperature_K - s11.temperature_K) < 0.01
        assert s15.pressure_Pa < s11.pressure_Pa

    def test_hot_day_offset(self):
        std = isa(0.0, 0.0)
        hot = isa(0.0, 20.0)
        assert abs(hot.temperature_K - (std.temperature_K + 20.0)) < 0.01
        assert hot.density_kg_m3 < std.density_kg_m3

    def test_density_decreases_with_altitude(self):
        rhos = [isa(h).density_kg_m3 for h in [0, 1000, 3000, 5000, 8000, 11000]]
        assert all(rhos[i] > rhos[i + 1] for i in range(len(rhos) - 1))

    def test_speed_of_sound_decreases_in_troposphere(self):
        a_vals = [isa(h).speed_of_sound_mps for h in [0, 3000, 8000]]
        assert all(a_vals[i] > a_vals[i + 1] for i in range(len(a_vals) - 1))

    def test_negative_altitude_raises(self):
        with pytest.raises(ValueError, match="Altitude must be"):
            isa(-1.0)

    def test_above_limit_raises(self):
        with pytest.raises(ValueError, match="20,000"):
            isa(25000.0)

    def test_boundary_altitudes_valid(self):
        isa(0.0)
        isa(11000.0)
        isa(20000.0)

    def test_atmo_state_fields(self):
        s = isa(5000.0)
        assert isinstance(s, AtmoState)
        assert s.altitude_m == 5000.0
        assert s.dynamic_viscosity_Pa_s > 0


# ---------------------------------------------------------------------------
# 2. Airfoil models
# ---------------------------------------------------------------------------

class TestLinearAirfoil:
    def test_zero_alpha(self, linear_airfoil):
        Cl, Cd, stalled = linear_airfoil.get_coeffs(0.0)
        assert abs(Cl) < 1e-9
        assert abs(Cd - linear_airfoil.Cd_min) < 1e-9
        assert not stalled

    def test_positive_alpha(self, linear_airfoil):
        alpha = np.radians(5.0)
        Cl, Cd, stalled = linear_airfoil.get_coeffs(alpha)
        assert abs(Cl - linear_airfoil.a0 * alpha) < 1e-9
        assert not stalled

    def test_stall_flag(self, linear_airfoil):
        alpha_stall = linear_airfoil.stall_alpha_rad
        _, _, not_stalled = linear_airfoil.get_coeffs(alpha_stall - 0.001)
        _, _, stalled = linear_airfoil.get_coeffs(alpha_stall)
        assert not not_stalled
        assert stalled

    def test_post_stall_cl_clipped(self, linear_airfoil):
        Cl, Cd, stalled = linear_airfoil.get_coeffs(np.radians(25.0))
        assert stalled
        assert abs(Cl) <= linear_airfoil.Cl_max_clip + 1e-9
        assert Cd >= 0.05

    def test_negative_alpha_symmetry(self, linear_airfoil):
        Cl_pos, _, _ = linear_airfoil.get_coeffs(np.radians(5.0))
        Cl_neg, _, _ = linear_airfoil.get_coeffs(np.radians(-5.0))
        assert abs(Cl_pos + Cl_neg) < 1e-9


class TestTableAirfoil:
    @pytest.fixture
    def table_airfoil(self):
        return TableAirfoil(
            alpha_deg=[-10, -5, 0, 5, 10, 15],
            Cl=[-0.9, -0.4, 0.1, 0.6, 1.1, 1.5],
            Cd=[0.05, 0.02, 0.01, 0.02, 0.05, 0.12],
        )

    def test_interpolation_midpoint(self, table_airfoil):
        Cl, Cd, stalled = table_airfoil.get_coeffs(np.radians(7.5))
        expected_Cl = 0.6 + 0.5 * (1.1 - 0.6)
        assert abs(Cl - expected_Cl) < 0.01
        assert not stalled

    def test_stall_at_boundary(self, table_airfoil):
        _, _, s_lo = table_airfoil.get_coeffs(np.radians(-10.0))
        _, _, s_hi = table_airfoil.get_coeffs(np.radians(15.0))
        assert s_lo
        assert s_hi

    def test_not_stalled_inside(self, table_airfoil):
        _, _, s = table_airfoil.get_coeffs(np.radians(0.0))
        assert not s

    def test_extrapolation_clips_to_boundary(self, table_airfoil):
        Cl_clip, _, _ = table_airfoil.get_coeffs(np.radians(30.0))
        Cl_hi, _, _ = table_airfoil.get_coeffs(np.radians(15.0))
        assert abs(Cl_clip - Cl_hi) < 1e-9

    def test_unsorted_input_is_sorted(self):
        af = TableAirfoil(
            alpha_deg=[10, 0, -10],
            Cl=[1.0, 0.0, -1.0],
            Cd=[0.05, 0.01, 0.05],
        )
        Cl, _, _ = af.get_coeffs(np.radians(5.0))
        assert abs(Cl - 0.5) < 0.01


class TestPrandtlGlauert:
    def test_zero_mach_no_correction(self):
        Cl_c, Cd_c = prandtl_glauert_correct(1.0, 0.02, 0.0)
        assert abs(Cl_c - 1.0) < 1e-9
        assert abs(Cd_c - 0.02) < 1e-9

    def test_mach_0_5_factor(self):
        Cl_c, Cd_c = prandtl_glauert_correct(1.0, 0.02, 0.5)
        expected = 1.0 / (1.0 - 0.5 ** 2) ** 0.5
        assert abs(Cl_c - expected) < 1e-4
        assert abs(Cd_c - 0.02 * expected) < 1e-6

    def test_above_mach_limit_frozen(self):
        Cl_at_limit, Cd_at_limit = prandtl_glauert_correct(1.0, 0.02, 0.7)
        Cl_above, Cd_above = prandtl_glauert_correct(1.0, 0.02, 0.9)
        assert abs(Cl_above - Cl_at_limit) < 1e-9
        assert abs(Cd_above - Cd_at_limit) < 1e-9

    def test_both_coefficients_corrected(self):
        Cl_in, Cd_in = 1.2, 0.04
        Cl_c, Cd_c = prandtl_glauert_correct(Cl_in, Cd_in, 0.5)
        factor = 1.0 / (1.0 - 0.5 ** 2) ** 0.5
        assert abs(Cl_c - Cl_in * factor) < 1e-9
        assert abs(Cd_c - Cd_in * factor) < 1e-9

    def test_correction_increases_coefficients(self):
        Cl_c, Cd_c = prandtl_glauert_correct(1.0, 0.02, 0.5)
        assert Cl_c > 1.0
        assert Cd_c > 0.02


# ---------------------------------------------------------------------------
# 3. Prandtl tip-loss and root-loss
# ---------------------------------------------------------------------------

class TestTipLoss:
    def test_at_tip_goes_to_zero(self):
        F = prandtl_tip_loss(B=3, R=1.0, r=0.999, phi=np.radians(5.0))
        assert F < 0.2

    def test_well_inboard_close_to_one(self):
        F = prandtl_tip_loss(B=3, R=1.0, r=0.5, phi=np.radians(8.0))
        assert F > 0.85

    def test_more_blades_less_loss(self):
        F2 = prandtl_tip_loss(B=2, R=1.0, r=0.9, phi=np.radians(5.0))
        F4 = prandtl_tip_loss(B=4, R=1.0, r=0.9, phi=np.radians(5.0))
        assert F4 > F2

    def test_phi_zero_safe(self):
        F = prandtl_tip_loss(B=3, R=1.0, r=0.9, phi=0.0)
        assert np.isfinite(F)
        assert 0 < F <= 1.0

    def test_root_loss_always_le_tip_loss(self):
        F_tip = prandtl_tip_loss(B=3, R=1.0, r=0.6, phi=np.radians(5.0))
        F_both = prandtl_tip_loss(B=3, R=1.0, r=0.6, phi=np.radians(5.0),
                                  root_cutout=0.15, include_root_loss=True)
        assert F_both <= F_tip


# ---------------------------------------------------------------------------
# 4. BEMT — hover physics sanity
# ---------------------------------------------------------------------------

class TestBEMTHover:
    def test_positive_thrust_positive_collective(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3,
                        sea_level.speed_of_sound_mps, v_axial=0.0)
        assert perf.thrust_N > 0
        assert perf.torque_Nm > 0
        assert perf.power_W > 0

    def test_ct_cq_cp_positive(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3,
                        sea_level.speed_of_sound_mps)
        assert perf.CT > 0
        assert perf.CQ > 0
        assert perf.CP > 0

    def test_figure_of_merit_physical(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3,
                        sea_level.speed_of_sound_mps)
        assert perf.figure_of_merit is not None
        assert 0 < perf.figure_of_merit <= 1.0

    def test_fm_is_none_in_forward_flight(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3,
                        sea_level.speed_of_sound_mps, v_axial=20.0)
        assert perf.figure_of_merit is None

    def test_thrust_increases_with_collective(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perfs = [run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                          np.radians(c), sea_level.density_kg_m3,
                          sea_level.speed_of_sound_mps)
                 for c in [4, 6, 8, 10]]
        thrusts = [p.thrust_N for p in perfs]
        assert all(thrusts[i] < thrusts[i + 1] for i in range(len(thrusts) - 1))

    def test_higher_density_higher_thrust(self, kh_rotor, linear_airfoil, kh_omega):
        atmo_sl = isa(0.0)
        atmo_hi = isa(5000.0)
        perf_sl = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                           np.radians(8.0), atmo_sl.density_kg_m3, atmo_sl.speed_of_sound_mps)
        perf_hi = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                           np.radians(8.0), atmo_hi.density_kg_m3, atmo_hi.speed_of_sound_mps)
        assert perf_sl.thrust_N > perf_hi.thrust_N

    def test_more_blades_more_thrust(self, linear_airfoil, sea_level, kh_omega):
        def make(b):
            return Rotor(radius_m=0.762, root_cutout_m=0.125, num_blades=b,
                         chord_fn=constant_chord(0.0508), twist_fn=constant_twist(0.0))
        perf2 = run_bemt(make(2), lambda x: linear_airfoil, kh_omega, np.radians(8.0),
                         sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        perf4 = run_bemt(make(4), lambda x: linear_airfoil, kh_omega, np.radians(8.0),
                         sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        assert perf4.thrust_N > perf2.thrust_N

    def test_converged_flag_true_for_normal_case(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        assert perf.converged

    def test_element_converged_fields_present(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        for el in perf.elements:
            assert hasattr(el, "converged")
            assert isinstance(el.converged, bool)

    def test_stalled_fraction_range(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        assert 0.0 <= perf.stalled_fraction <= 1.0

    def test_max_tip_mach_physical(self, kh_rotor, linear_airfoil, sea_level, kh_omega):
        perf = run_bemt(kh_rotor, lambda x: linear_airfoil, kh_omega,
                        np.radians(8.0), sea_level.density_kg_m3, sea_level.speed_of_sound_mps)
        assert perf.max_tip_mach > 0
        assert perf.max_tip_mach < 0.5


# ---------------------------------------------------------------------------
# 5. BEMT — axial forward flight
# ---------------------------------------------------------------------------

class TestBEMTAxialFlight:
    @pytest.fixture
    def prop_rotor(self):
        return Rotor(radius_m=0.762, root_cutout_m=0.125, num_blades=3,
                     chord_fn=constant_chord(0.06),
                     twist_fn=linear_twist(np.radians(30.0), np.radians(-20.0)))

    @pytest.fixture
    def prop_omega(self):
        return 2 * np.pi * 2200.0 / 60.0

    @pytest.fixture
    def cruise_atmo(self):
        return isa(3000.0, 0.0)

    def test_propulsive_efficiency_physical(self, prop_rotor, linear_airfoil, prop_omega, cruise_atmo):
        perf = run_bemt(prop_rotor, lambda x: linear_airfoil, prop_omega,
                        np.radians(10.0), cruise_atmo.density_kg_m3,
                        cruise_atmo.speed_of_sound_mps, v_axial=40.0)
        assert perf.propulsive_efficiency is not None
        assert 0 < perf.propulsive_efficiency < 1.0

    def test_eta_none_in_hover(self, prop_rotor, linear_airfoil, prop_omega, cruise_atmo):
        perf = run_bemt(prop_rotor, lambda x: linear_airfoil, prop_omega,
                        np.radians(10.0), cruise_atmo.density_kg_m3,
                        cruise_atmo.speed_of_sound_mps, v_axial=0.0)
        assert perf.propulsive_efficiency is None

    def test_advance_ratio_proportional_to_speed(self, prop_rotor, prop_omega):
        J1 = advance_ratio(20.0, prop_omega, prop_rotor.radius_m)
        J2 = advance_ratio(40.0, prop_omega, prop_rotor.radius_m)
        assert abs(J2 / J1 - 2.0) < 1e-9

    def test_advance_ratio_zero_speed(self, prop_rotor, prop_omega):
        assert advance_ratio(0.0, prop_omega, prop_rotor.radius_m) == 0.0

    def test_advance_ratio_zero_omega(self, prop_rotor):
        assert advance_ratio(40.0, 0.0, prop_rotor.radius_m) == 0.0

    def test_ct_decreases_with_forward_speed(self, prop_rotor, linear_airfoil,
                                              prop_omega, cruise_atmo):
        speeds = [10.0, 30.0, 50.0]
        cts = [run_bemt(prop_rotor, lambda x: linear_airfoil, prop_omega,
                        np.radians(10.0), cruise_atmo.density_kg_m3,
                        cruise_atmo.speed_of_sound_mps, v_axial=v).CT
               for v in speeds]
        assert cts[0] > cts[-1]


# ---------------------------------------------------------------------------
# 6. Rotor geometry
# ---------------------------------------------------------------------------

class TestRotorGeometry:
    def test_disk_area(self, kh_rotor):
        assert abs(kh_rotor.disk_area_m2() - np.pi * 0.762 ** 2) < 1e-9

    def test_tip_speed(self, kh_rotor, kh_omega):
        assert abs(kh_rotor.tip_speed_mps(kh_omega) - kh_omega * 0.762) < 1e-9

    def test_tip_mach_hover(self, kh_rotor, kh_omega):
        a = isa(0.0).speed_of_sound_mps
        mach = kh_rotor.tip_mach(kh_omega, a, axial_velocity_mps=0.0)
        assert abs(mach - kh_rotor.tip_speed_mps(kh_omega) / a) < 1e-9

    def test_tip_mach_forward_flight_higher(self, kh_rotor, kh_omega):
        a = isa(0.0).speed_of_sound_mps
        assert kh_rotor.tip_mach(kh_omega, a, 50.0) > kh_rotor.tip_mach(kh_omega, a, 0.0)

    def test_solidity_constant_chord(self):
        R, c, B = 1.0, 0.1, 3
        r = Rotor(radius_m=R, root_cutout_m=0.1, num_blades=B,
                  chord_fn=constant_chord(c), twist_fn=constant_twist(0.0))
        assert abs(r.solidity() - B * c / (np.pi * R)) < 0.001

    def test_linear_taper_chord(self):
        c_fn = linear_taper_chord(root_chord=0.1, taper_ratio=0.5)
        assert abs(c_fn(0.0) - 0.1) < 1e-9
        assert abs(c_fn(1.0) - 0.05) < 1e-9


# ---------------------------------------------------------------------------
# 7. Mission Planner
# ---------------------------------------------------------------------------

@pytest.fixture
def mission_airfoil():
    return LinearAirfoil()


@pytest.fixture
def mission_rotor():
    return Rotor(radius_m=0.762, root_cutout_m=0.125, num_blades=3,
                 chord_fn=constant_chord(0.06),
                 twist_fn=linear_twist(np.radians(30.0), np.radians(-20.0)))


@pytest.fixture
def mission_limits():
    return DesignLimits(
        max_tip_mach=0.9, max_stall_fraction=0.10,
        min_power_margin_frac=0.05, min_rpm=200, max_rpm=2600,
        min_collective_deg=-20, max_collective_deg=30,
        reserve_fuel_kg=5.0,
    )


@pytest.fixture
def mission_power_model():
    return PowerAvailableModel(sea_level_power_W=90_000.0)


@pytest.fixture
def mission_fuel_model():
    return FuelModel(sfc_kg_per_J=1.8e-8)


def _make_planner(mission_rotor, mission_airfoil, mission_power_model,
                  mission_fuel_model, mission_limits, fuel_kg=30.0):
    return MissionPlanner(
        rotor=mission_rotor, airfoil_provider=lambda x: mission_airfoil,
        num_rotors=2, empty_mass_kg=80.0, fuel_mass_kg=fuel_kg,
        power_model=mission_power_model, fuel_model=mission_fuel_model,
        limits=mission_limits,
    )


class TestMissionFeasible:
    def test_short_hover_completes(self, mission_rotor, mission_airfoil,
                                   mission_power_model, mission_fuel_model, mission_limits):
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, mission_limits)
        state = planner.run_mission([
            MissionSegment("Hover", SegmentType.HOVER, duration_s=30,
                           altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)
        ])
        assert state.time_s == pytest.approx(30.0)
        assert state.fuel_mass_kg > 0

    def test_log_entries_populated(self, mission_rotor, mission_airfoil,
                                   mission_power_model, mission_fuel_model, mission_limits):
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, mission_limits)
        state = planner.run_mission([
            MissionSegment("Hover", SegmentType.HOVER, duration_s=10,
                           altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)
        ])
        assert len(state.log) == 2
        assert "thrust_N" in state.log[0]

    def test_payload_event_changes_mass(self, mission_rotor, mission_airfoil,
                                        mission_power_model, mission_fuel_model, mission_limits):
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, mission_limits)
        initial_mass = planner.state.gross_mass_kg
        planner.run_mission([
            MissionSegment("Drop", SegmentType.PAYLOAD_EVENT, duration_s=0,
                           altitude_m=0, payload_delta_kg=-5.0)
        ])
        assert abs(planner.state.gross_mass_kg - (initial_mass - 5.0)) < 1e-9

    def test_fuel_decreases_during_flight(self, mission_rotor, mission_airfoil,
                                           mission_power_model, mission_fuel_model, mission_limits):
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, mission_limits)
        fuel_before = planner.state.fuel_mass_kg
        planner.run_mission([
            MissionSegment("Hover", SegmentType.HOVER, duration_s=30,
                           altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)
        ])
        assert planner.state.fuel_mass_kg < fuel_before


class TestMissionInfeasible:
    def test_fuel_reserve_breach_detected(self, mission_rotor, mission_airfoil,
                                           mission_power_model, mission_fuel_model):
        limits = DesignLimits(
            max_tip_mach=0.9, max_stall_fraction=0.10,
            min_power_margin_frac=0.0, min_rpm=0, max_rpm=9999,
            min_collective_deg=-90, max_collective_deg=90,
            reserve_fuel_kg=5.0,
        )
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, limits, fuel_kg=5.02)
        with pytest.raises(MissionInfeasibleError, match="reserve requirement"):
            planner.run_mission([
                MissionSegment("Long hover", SegmentType.HOVER, duration_s=60,
                               altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5),
            ])

    def test_tip_mach_limit(self, mission_rotor, mission_airfoil,
                             mission_power_model, mission_fuel_model):
        limits = DesignLimits(max_tip_mach=0.05, max_stall_fraction=1.0,
                              min_power_margin_frac=-1.0, min_rpm=0, max_rpm=9999,
                              min_collective_deg=-90, max_collective_deg=90)
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, limits)
        with pytest.raises(MissionInfeasibleError, match="Tip Mach"):
            planner.run_mission([MissionSegment("Hover", SegmentType.HOVER, duration_s=5,
                                                altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)])

    def test_stall_limit(self, mission_rotor, mission_airfoil,
                          mission_power_model, mission_fuel_model):
        limits = DesignLimits(max_tip_mach=1.0, max_stall_fraction=0.0,
                              min_power_margin_frac=-1.0, min_rpm=0, max_rpm=9999,
                              min_collective_deg=-90, max_collective_deg=90)
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, limits)
        with pytest.raises(MissionInfeasibleError, match="Stalled"):
            planner.run_mission([MissionSegment("Hover", SegmentType.HOVER, duration_s=5,
                                                altitude_m=0, rpm=2200, collective_deg=28, dt_s=5)])

    def test_rpm_limit_low(self, mission_rotor, mission_airfoil,
                            mission_power_model, mission_fuel_model):
        limits = DesignLimits(max_tip_mach=1.0, max_stall_fraction=1.0,
                              min_power_margin_frac=-1.0, min_rpm=5000, max_rpm=9999,
                              min_collective_deg=-90, max_collective_deg=90)
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, limits)
        with pytest.raises(MissionInfeasibleError, match="RPM"):
            planner.run_mission([MissionSegment("Hover", SegmentType.HOVER, duration_s=5,
                                                altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)])

    def test_collective_limit(self, mission_rotor, mission_airfoil,
                               mission_power_model, mission_fuel_model):
        limits = DesignLimits(max_tip_mach=1.0, max_stall_fraction=1.0,
                              min_power_margin_frac=-1.0, min_rpm=0, max_rpm=9999,
                              min_collective_deg=0, max_collective_deg=10)
        planner = _make_planner(mission_rotor, mission_airfoil,
                                mission_power_model, mission_fuel_model, limits)
        with pytest.raises(MissionInfeasibleError, match="Collective"):
            planner.run_mission([MissionSegment("Hover", SegmentType.HOVER, duration_s=5,
                                                altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)])

    def test_power_margin_limit(self, mission_rotor, mission_airfoil, mission_fuel_model):
        tiny_power = PowerAvailableModel(sea_level_power_W=100.0)
        limits = DesignLimits(max_tip_mach=1.0, max_stall_fraction=1.0,
                              min_power_margin_frac=0.05, min_rpm=0, max_rpm=9999,
                              min_collective_deg=-90, max_collective_deg=90)
        planner = _make_planner(mission_rotor, mission_airfoil,
                                tiny_power, mission_fuel_model, limits)
        with pytest.raises(MissionInfeasibleError, match="Power margin"):
            planner.run_mission([MissionSegment("Hover", SegmentType.HOVER, duration_s=5,
                                                altitude_m=0, rpm=2200, collective_deg=-2, dt_s=5)])


# ---------------------------------------------------------------------------
# 8. FuelModel and PowerAvailableModel
# ---------------------------------------------------------------------------

class TestSupportModels:
    def test_fuel_burn_rate_positive(self):
        fm = FuelModel(sfc_kg_per_J=1.8e-8)
        assert fm.burn_rate_kg_s(10000.0) > 0

    def test_fuel_burn_rate_zero_for_zero_power(self):
        assert FuelModel(sfc_kg_per_J=1.8e-8).burn_rate_kg_s(0.0) == 0.0

    def test_fuel_burn_rate_no_negative(self):
        assert FuelModel(sfc_kg_per_J=1.8e-8).burn_rate_kg_s(-100.0) == 0.0

    def test_power_available_decreases_with_altitude(self):
        pm = PowerAvailableModel(sea_level_power_W=100_000.0)
        assert pm.power_available_W(isa(0.0)) > pm.power_available_W(isa(5000.0))

    def test_drivetrain_efficiency_applied(self):
        pm = PowerAvailableModel(sea_level_power_W=100_000.0, drivetrain_efficiency=0.9)
        assert abs(pm.power_available_W(isa(0.0)) - 90_000.0) < 1.0
