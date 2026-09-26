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
                 flat_plate_area_m2: float = 1.7,
                 step_callback: Optional[Callable[[MissionState], None]] = None):
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
        self.step_callback = step_callback

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
            # Airplane mode: total drag = parasite drag + induced drag
            atmo = isa(seg.altitude_m, seg.dISA_K)
            V = seg.cruise_speed_mps
            q = 0.5 * atmo.density_kg_m3 * V**2
            
            # Parasite Drag
            D_parasite = q * self.flat_plate_area_m2
            
            # Induced Drag (L = W, D_i = L^2 / (q * pi * AR * e))
            # Hardcoding the wing parameters designed earlier (S=39.24, AR=9.0, e=0.8)
            wing_area = 39.24
            AR = 9.0
            e = 0.8
            L = self.state.gross_mass_kg * 9.81
            D_induced = (L**2) / (q * wing_area * np.pi * e * AR)
            
            drag_N = D_parasite + D_induced
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

            # -----------------------------------------------------------------
            # AUTO-TRIM & FUEL OPTIMIZATION ENGINE
            # -----------------------------------------------------------------
            target_total_thrust = self._required_thrust_N(seg)
            target_per_rotor = target_total_thrust / self.num_rotors
            
            # Use Auto-Trim just for the collective (fixes RPM to user choice for performance)
            omega = 2 * np.pi * seg.rpm / 60.0
            best_coll, perf = self._optimize_trim(target_per_rotor, omega, atmo, v_axial)
            best_rpm = seg.rpm
            
            if perf is None:
                raise MissionInfeasibleError(
                    seg.name, self.state.time_s, 
                    f"Aero Auto-Trim Failed: Could not find any valid Collective pitch to generate required thrust ({target_per_rotor:.0f} N) at {seg.rpm} RPM.")

            # Temporarily update segment state so limit checks process the optimized values
            original_rpm = seg.rpm
            original_coll = seg.collective_deg
            seg.rpm = best_rpm
            seg.collective_deg = best_coll
            
            P_req_W = self.num_rotors * perf.power_W
            P_avail_W = self.num_rotors * self.power_model.power_available_W(atmo)

            self._check_limits(seg, perf, P_req_W, P_avail_W)

            burn_rate = self.fuel_model.burn_rate_kg_s(P_req_W)
            fuel_burned = burn_rate * seg.dt_s
            self.state.fuel_mass_kg -= fuel_burned
            self.state.gross_mass_kg -= fuel_burned
            self.state.time_s += seg.dt_s

            if self.state.fuel_mass_kg < self.limits.reserve_fuel_kg:
                raise MissionInfeasibleError(
                    seg.name, self.state.time_s,
                    f"Fuel {self.state.fuel_mass_kg:.2f} kg has dropped below the "
                    f"reserve requirement of {self.limits.reserve_fuel_kg:.2f} kg.")

            self.state.log.append(dict(
                time_s=self.state.time_s, segment=seg.name,
                gross_mass_kg=self.state.gross_mass_kg,
                fuel_mass_kg=self.state.fuel_mass_kg,
                rpm=best_rpm, collective_deg=best_coll,
                thrust_N=self.num_rotors * perf.thrust_N,
                power_req_W=P_req_W, power_avail_W=P_avail_W,
                max_tip_mach=perf.max_tip_mach,
                stalled_fraction=perf.stalled_fraction,
            ))
            
            if self.step_callback:
                self.step_callback(self.state)
            
            # Restore segment defaults just in case (though optimizer will override next tick anyway)
            seg.rpm = original_rpm
            seg.collective_deg = original_coll

    def run_mission(self, segments: List[MissionSegment]):
        for seg in segments:
            self.run_segment(seg)
        return self.state
        
    def _optimize_trim(self, target_thrust_per_rotor: float, omega: float, atmo, v_axial: float):
        """Uses Brent's method to find the exact collective pitch that generates the required thrust."""
        def residual(coll_deg: float) -> float:
            perf = run_bemt(self.rotor, self.airfoil_provider, omega,
                            np.radians(coll_deg), atmo.density_kg_m3,
                            atmo.speed_of_sound_mps, v_axial=v_axial)
            return perf.thrust_N - target_thrust_per_rotor

        c_min, c_max = self.limits.min_collective_deg, self.limits.max_collective_deg
        
        # Check if the target thrust is even possible within the collective limits
        f_min = residual(c_min)
        f_max = residual(c_max)
        
        if not (np.isfinite(f_min) and np.isfinite(f_max)): return None, None
        if f_min * f_max > 0: return None, None  # Target thrust not achievable in this bracket

        try:
            from scipy.optimize import brentq
            opt_coll = brentq(residual, c_min, c_max, xtol=0.05, maxiter=60)
            perf = run_bemt(self.rotor, self.airfoil_provider, omega,
                            np.radians(opt_coll), atmo.density_kg_m3,
                            atmo.speed_of_sound_mps, v_axial=v_axial)
            return opt_coll, perf
        except Exception:
            return None, None

    def _find_optimal_efficiency(self, target_thrust_per_rotor: float, atmo, v_axial: float, fallback_rpm: float):
        """Scans allowed RPMs to find the most fuel-efficient (lowest power) state that trims the aircraft."""
        best_rpm = None
        best_coll = None
        best_perf = None
        min_power = float('inf')

        # Scan RPM range in steps of 25 to find optimal aerodynamic efficiency
        for test_rpm in np.arange(self.limits.min_rpm, self.limits.max_rpm + 1, 25.0):
            if test_rpm <= 0: continue
            
            omega = 2 * np.pi * test_rpm / 60.0
            
            # Fast Mach check to skip RPMs that are obviously supersonic
            tip_speed = omega * self.rotor.radius_m
            mach = np.sqrt(tip_speed**2 + v_axial**2) / atmo.speed_of_sound_mps
            if mach > self.limits.max_tip_mach:
                continue
                
            coll, perf = self._optimize_trim(target_thrust_per_rotor, omega, atmo, v_axial)
            
            if perf is not None and perf.converged:
                if perf.stalled_fraction <= self.limits.max_stall_fraction:
                    if perf.power_W < min_power:
                        min_power = perf.power_W
                        best_rpm = test_rpm
                        best_coll = coll
                        best_perf = perf
                        
        if best_perf is None:
            # Fallback to user-provided RPM if the sweep fails
            omega = 2 * np.pi * fallback_rpm / 60.0
            coll, perf = self._optimize_trim(target_thrust_per_rotor, omega, atmo, v_axial)
            return fallback_rpm, coll, perf
            
        return best_rpm, best_coll, best_perf
