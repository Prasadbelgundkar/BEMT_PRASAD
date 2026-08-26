"""
mission.py
----------
Mission Planner v1 (Tasks 9 & 10).

Executive controller that steps through user-defined mission segments,
uses the BEMT solver as its aerodynamic backend to get power required,
compares against a power-available model, updates mass/fuel each time
step, and raises a MissionInfeasibleError the first time any adopted
design limit is violated -- identifying segment, time, and reason, per
Task 10.

Segment types implemented: HOVER, VERTICAL_CLIMB, VERTICAL_DESCENT,
CRUISE (axial/airplane-mode), LOITER, PAYLOAD_EVENT.

This module intentionally does NOT hard-code an aircraft: everything
(rotor, engine model, mass, mission profile) is passed in by the caller,
per the "do not hard-code a single aircraft or mission" requirement.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, List, Optional
import numpy as np

from environment import isa
from rotor import Rotor
from bemt import run_bemt, advance_ratio


class SegmentType(Enum):
    HOVER = auto()
    VERTICAL_CLIMB = auto()
    VERTICAL_DESCENT = auto()
    CRUISE = auto()
    LOITER = auto()
    PAYLOAD_EVENT = auto()


class MissionInfeasibleError(Exception):
    """Raised the first time a mission-level constraint is violated.
    Carries the segment name, mission time, and human-readable reason,
    per Task 10's requirement to clearly identify all three."""
    def __init__(self, segment_name: str, time_s: float, reason: str):
        self.segment_name = segment_name
        self.time_s = time_s
        self.reason = reason
        super().__init__(f"[t={time_s:6.1f}s | segment='{segment_name}'] {reason}")


@dataclass
class PowerAvailableModel:
    """Simple engine/motor power-available model, degrading with altitude
    and temperature. Replace `sea_level_power_W` and `lapse` with your
    team's adopted propulsion data (Section 1.3)."""
    sea_level_power_W: float
    density_ratio_exponent: float = 1.0   # P_avail ~ P0 * (rho/rho0)^exponent
    drivetrain_efficiency: float = 0.95   # gearbox/transmission losses

    def power_available_W(self, atmo) -> float:
        rho0 = 1.225
        rho_ratio = atmo.density_kg_m3 / rho0
        P = self.sea_level_power_W * rho_ratio ** self.density_ratio_exponent
        return P * self.drivetrain_efficiency


@dataclass
class FuelModel:
    """Specific fuel consumption model: sfc in kg of fuel per (W * s), i.e.
    kg/J, so fuel_burn_rate = sfc * shaft_power. For electric propulsion,
    set sfc based on battery energy density instead and track energy, not
    mass -- swap `burn_rate_kg_s` accordingly."""
    sfc_kg_per_J: float

    def burn_rate_kg_s(self, shaft_power_W: float) -> float:
        return self.sfc_kg_per_J * max(shaft_power_W, 0.0)


@dataclass
class DesignLimits:
    max_tip_mach: float = 0.85
    max_stall_fraction: float = 0.05      # e.g. <=5% of blade span stalled
    min_power_margin_frac: float = 0.05   # P_avail must exceed P_req by >=5%
    min_rpm: float = 0.0
    max_rpm: float = 1e9
    min_collective_deg: float = -5.0
    max_collective_deg: float = 20.0
    reserve_fuel_kg: float = 0.0


@dataclass
class MissionSegment:
    name: str
    seg_type: SegmentType
    duration_s: float
    altitude_m: float
    dISA_K: float = 0.0
    rpm: float = 0.0
    collective_deg: float = 0.0
    vertical_speed_mps: float = 0.0     # climb(+)/descent(-) for VERTICAL_*
    cruise_speed_mps: float = 0.0       # true airspeed for CRUISE
    wind_mps: float = 0.0               # headwind(+)/tailwind(-) along cruise
    payload_delta_kg: float = 0.0       # applied instantaneously at segment start
                                          # (positive = pickup, negative = drop)
    dt_s: float = 5.0                   # user-defined time-step


@dataclass
class MissionState:
    time_s: float = 0.0
    gross_mass_kg: float = 0.0
    fuel_mass_kg: float = 0.0
    log: List[dict] = field(default_factory=list)


class MissionPlanner:
    def __init__(self, rotor: Rotor, airfoil_provider: Callable[[float], object],
                 num_rotors: int, empty_mass_kg: float, fuel_mass_kg: float,
                 power_model: PowerAvailableModel, fuel_model: FuelModel,
                 limits: DesignLimits, g: float = 9.80665,
                 flat_plate_area_m2: float = 1.7):
        self.rotor = rotor
        self.airfoil_provider = airfoil_provider
        self.num_rotors = num_rotors
        self.g = g
        self.power_model = power_model
        self.fuel_model = fuel_model
        self.limits = limits
        self.flat_plate_area_m2 = flat_plate_area_m2
        self.state = MissionState(gross_mass_kg=empty_mass_kg + fuel_mass_kg,
                                   fuel_mass_kg=fuel_mass_kg)
        self.empty_mass_kg = empty_mass_kg

    def _check_limits(self, seg: MissionSegment, perf, P_req_W: float, P_avail_W: float):
        if perf.max_tip_mach > self.limits.max_tip_mach:
            raise MissionInfeasibleError(
                seg.name, self.state.time_s,
                f"Tip Mach {perf.max_tip_mach:.3f} exceeds limit "
                f"{self.limits.max_tip_mach:.3f}.")
        if perf.stalled_fraction > self.limits.max_stall_fraction:
            raise MissionInfeasibleError(
                seg.name, self.state.time_s,
                f"Stalled blade fraction {perf.stalled_fraction:.1%} exceeds "
                f"limit {self.limits.max_stall_fraction:.1%}.")
        margin = (P_avail_W - P_req_W) / max(P_avail_W, 1e-9)
        if margin < self.limits.min_power_margin_frac:
            raise MissionInfeasibleError(
                seg.name, self.state.time_s,
                f"Power margin {margin:.1%} below required "
                f"{self.limits.min_power_margin_frac:.1%} "
                f"(P_req={P_req_W/1e3:.1f} kW, P_avail={P_avail_W/1e3:.1f} kW).")
        rpm = seg.rpm
        if not (self.limits.min_rpm <= rpm <= self.limits.max_rpm):
            raise MissionInfeasibleError(
                seg.name, self.state.time_s,
                f"RPM {rpm:.0f} outside allowed range "
                f"[{self.limits.min_rpm:.0f}, {self.limits.max_rpm:.0f}].")
        if not (self.limits.min_collective_deg <= seg.collective_deg <= self.limits.max_collective_deg):
            raise MissionInfeasibleError(
                seg.name, self.state.time_s,
                f"Collective {seg.collective_deg:.1f} deg outside allowed range "
                f"[{self.limits.min_collective_deg:.1f}, {self.limits.max_collective_deg:.1f}] deg.")

    def _required_thrust_N(self, seg: MissionSegment) -> float:
        """Required total thrust from ALL rotors combined for vertical
        equilibrium / climb, per current gross mass."""
        W = self.state.gross_mass_kg * self.g
        if seg.seg_type in (SegmentType.HOVER, SegmentType.LOITER):
            return W
        if seg.seg_type == SegmentType.VERTICAL_CLIMB:
            # Simple momentum-based climb thrust augmentation could be added;
            # first-order: still size for weight support (climb power comes
            # from the extra Vc term inside BEMT itself).
            return W
        if seg.seg_type == SegmentType.VERTICAL_DESCENT:
            return W
        if seg.seg_type == SegmentType.CRUISE:
            # Airplane mode: wing assumed to carry weight; rotors sized for
            # propulsive thrust to overcome drag. (D = 0.5*rho*V^2*f)
            # Using the V-22 scale equivalent flat plate area: f = 1.7 m^2
            atmo = isa(seg.altitude_m, seg.dISA_K)
            V = seg.cruise_speed_mps
            drag_N = 0.5 * atmo.density_kg_m3 * V**2 * self.flat_plate_area_m2
            return drag_N
        return W

    def run_segment(self, seg: MissionSegment):
        n_steps = max(int(round(seg.duration_s / seg.dt_s)), 1)
        omega = 2 * np.pi * seg.rpm / 60.0

        # Apply instantaneous payload event at segment start.
        if seg.seg_type == SegmentType.PAYLOAD_EVENT:
            self.state.gross_mass_kg += seg.payload_delta_kg
            self.state.log.append(dict(time_s=self.state.time_s, segment=seg.name,
                                        event="payload_change",
                                        gross_mass_kg=self.state.gross_mass_kg))
            return

        for _ in range(n_steps):
            atmo = isa(seg.altitude_m, seg.dISA_K)

            if seg.seg_type in (SegmentType.HOVER, SegmentType.LOITER):
                v_axial = 0.0
            elif seg.seg_type == SegmentType.VERTICAL_CLIMB:
                v_axial = seg.vertical_speed_mps
            elif seg.seg_type == SegmentType.VERTICAL_DESCENT:
                v_axial = -abs(seg.vertical_speed_mps)
            elif seg.seg_type == SegmentType.CRUISE:
                v_axial = seg.cruise_speed_mps - seg.wind_mps
            else:
                v_axial = 0.0

            perf = run_bemt(self.rotor, self.airfoil_provider, omega,
                             np.radians(seg.collective_deg), atmo.density_kg_m3,
                             atmo.speed_of_sound_mps, v_axial=v_axial)

            P_req_W = self.num_rotors * perf.power_W
            P_avail_W = self.num_rotors * self.power_model.power_available_W(atmo)  # total, all rotors

            self._check_limits(seg, perf, P_req_W, P_avail_W)

            burn_rate = self.fuel_model.burn_rate_kg_s(P_req_W)
            fuel_burned = burn_rate * seg.dt_s
            self.state.fuel_mass_kg -= fuel_burned
            self.state.gross_mass_kg -= fuel_burned
            self.state.time_s += seg.dt_s

            # Check fuel reserve AFTER deduction so violations are caught on the
            # exact step that causes the breach, not one step late.
            if self.state.fuel_mass_kg < self.limits.reserve_fuel_kg:
                raise MissionInfeasibleError(
                    seg.name, self.state.time_s,
                    f"Fuel {self.state.fuel_mass_kg:.2f} kg has dropped below the "
                    f"reserve requirement of {self.limits.reserve_fuel_kg:.2f} kg.")

            self.state.log.append(dict(
                time_s=self.state.time_s, segment=seg.name,
                gross_mass_kg=self.state.gross_mass_kg,
                fuel_mass_kg=self.state.fuel_mass_kg,
                thrust_N=self.num_rotors * perf.thrust_N,
                power_req_W=P_req_W, power_avail_W=P_avail_W,
                max_tip_mach=perf.max_tip_mach,
                stalled_fraction=perf.stalled_fraction,
            ))

    def run_mission(self, segments: List[MissionSegment]):
        for seg in segments:
            self.run_segment(seg)
        return self.state
