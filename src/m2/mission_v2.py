import numpy as np
from typing import Callable, List, Dict
from dataclasses import dataclass

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.trim_solver import trim_aircraft, compute_aircraft_residual, shaft_angle_of_attack
from m2.edgewise_bemt import run_edgewise_bemt

@dataclass
class MissionSegment:
    name: str
    duration_s: float
    distance_m: float
    energy_J: float
    avg_power_kW: float
    fuel_burn_kg: float

def compute_segment_power(
    V_inf: float, 
    gamma_rad: float, 
    nacelle_deg: float, 
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    omega_rad_s: float
) -> float:
    """
    Trims the aircraft and returns the required total power in Watts.
    """
    trim_pitch_with = 'cyclic' if nacelle_deg > 45.0 else 'elevator'
    x0 = np.array([np.radians(5.0), np.radians(15.0), 0.0])
    
    x_trim = trim_aircraft(
        V_inf=max(0.1, V_inf), # Avoid V=0 singularity
        gamma_rad=gamma_rad,
        nacelle_deg=nacelle_deg,
        omega_rad_s=omega_rad_s,
        aircraft=aircraft,
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        rho=rho,
        a_sound=a_sound,
        x0=x0,
        trim_pitch_with=trim_pitch_with
    )
    
    # Run BEMT one last time at the trim state to get power
    if x_trim is None:
        raise RuntimeError(
            f"Trim failed at V={V_inf:.1f} m/s, nacelle={nacelle_deg:.0f} deg; "
            "segment power cannot be evaluated at an untrimmed state.")
    alpha_shaft_rad = shaft_angle_of_attack(x_trim[0], nacelle_deg)
    rotor_result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=max(0.1, V_inf),
        omega_rad_s=omega_rad_s,
        theta0_rad=x_trim[1],
        theta1c_rad=x_trim[2] if trim_pitch_with == 'cyclic' else 0.0,
        theta1s_rad=0.0,
        alpha_shaft_rad=alpha_shaft_rad,
        nacelle_angle_deg=nacelle_deg,
        rho=rho,
        a_sound=a_sound,
        n_r=25,
        n_psi=36
    )
    
    # Return total power for both rotors
    return 2.0 * rotor_result.power_W

def run_tiltrotor_mission(
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    hover_omega_rad_s: float,
    cruise_omega_rad_s: float,
    cruise_distance_m: float,
    cruise_velocity_m_s: float,
    sfc_kg_J: float = 8.0e-8, # Specific fuel consumption roughly typical for turboshaft
    transition_schedule=None
) -> List[MissionSegment]:
    """
    Simulates a full mission profile: 
    Hover -> Transition -> Cruise -> Reconversion -> Hover
    """
    segments = []
    
    # 1. Hover Takeoff (2 minutes)
    hover_time = 120.0
    print("Simulating Takeoff Hover segment...")
    P_hover = compute_segment_power(0.0, 0.0, 90.0, aircraft, rotor, airfoil_provider, rho, a_sound, hover_omega_rad_s)
    segments.append(MissionSegment(
        name="Takeoff Hover",
        duration_s=hover_time,
        distance_m=0.0,
        energy_J=P_hover * hover_time,
        avg_power_kW=P_hover / 1000.0,
        fuel_burn_kg=P_hover * hover_time * sfc_kg_J
    ))
    
    # 2. Transition (Accelerate to cruise speed over 60 seconds)
    trans_time = 60.0
    print("Simulating Transition segment...")
    # (airspeed [m/s], nacelle angle [deg]) points flown during conversion.
    # A straight line from (0, 90) to (V_cruise, 0) crosses untrimmable
    # low-speed / low-nacelle-angle cells of the conversion corridor, so the
    # schedule is an input; the default holds the nacelles high until the
    # wing carries load (see the Section 7 corridor map).
    if transition_schedule is None:
        transition_schedule = [(0.1, 90.0), (25.0, 75.0), (45.0, 60.0),
                               (60.0, 30.0), (max(cruise_velocity_m_s, 65.0), 0.0)]
    V_pts = [p[0] for p in transition_schedule]
    N_pts = [p[1] for p in transition_schedule]
    P_trans_pts = []
    for v, n in zip(V_pts, N_pts):
        P = compute_segment_power(v, 0.0, n, aircraft, rotor, airfoil_provider, rho, a_sound, hover_omega_rad_s)
        P_trans_pts.append(P)
        
    avg_P_trans = np.mean(P_trans_pts)
    trans_dist = 0.5 * cruise_velocity_m_s * trans_time # basic kinematics
    segments.append(MissionSegment(
        name="Conversion",
        duration_s=trans_time,
        distance_m=trans_dist,
        energy_J=avg_P_trans * trans_time,
        avg_power_kW=avg_P_trans / 1000.0,
        fuel_burn_kg=avg_P_trans * trans_time * sfc_kg_J
    ))
    
    # 3. Cruise
    print("Simulating Cruise segment...")
    cruise_time = cruise_distance_m / cruise_velocity_m_s
    P_cruise = compute_segment_power(cruise_velocity_m_s, 0.0, 0.0, aircraft, rotor, airfoil_provider, rho, a_sound, cruise_omega_rad_s)
    segments.append(MissionSegment(
        name="Cruise",
        duration_s=cruise_time,
        distance_m=cruise_distance_m,
        energy_J=P_cruise * cruise_time,
        avg_power_kW=P_cruise / 1000.0,
        fuel_burn_kg=P_cruise * cruise_time * sfc_kg_J
    ))
    
    # 4. Reconversion (Decelerate over 60 seconds)
    print("Simulating Reconversion segment...")
    segments.append(MissionSegment(
        name="Reconversion",
        duration_s=trans_time,
        distance_m=trans_dist,
        energy_J=avg_P_trans * trans_time,
        avg_power_kW=avg_P_trans / 1000.0,
        fuel_burn_kg=avg_P_trans * trans_time * sfc_kg_J
    ))
    
    # 5. Landing Hover (2 minutes)
    print("Simulating Landing Hover segment...")
    segments.append(MissionSegment(
        name="Landing Hover",
        duration_s=hover_time,
        distance_m=0.0,
        energy_J=P_hover * hover_time,
        avg_power_kW=P_hover / 1000.0,
        fuel_burn_kg=P_hover * hover_time * sfc_kg_J
    ))

    return segments


# ===========================================================================
# Mission Planner v2 -- time-stepped transition missions (report Sections 2.3, 8)
# ===========================================================================
from typing import Optional as _Optional, Sequence as _Sequence


def schedule(points) -> Callable[[float], float]:
    """Piecewise-linear schedule of normalized segment time tau in [0, 1].
    `points` is a number (constant) or a list of (tau, value) pairs."""
    if np.isscalar(points):
        v = float(points)
        return lambda tau: v
    taus, vals = zip(*points)
    taus, vals = np.asarray(taus, float), np.asarray(vals, float)
    return lambda tau: float(np.interp(tau, taus, vals))


def ramp_schedule(v0: float, v1: float, ramp_frac: float = 0.15, n: int = 61) -> Callable[[float], float]:
    """Speed schedule v0 -> v1 with a TRAPEZOIDAL acceleration profile:
    acceleration ramps up over the first `ramp_frac` of the segment, stays
    constant, and ramps down over the last `ramp_frac` (no acceleration steps
    at segment boundaries)."""
    tau = np.linspace(0.0, 1.0, n)
    a = np.minimum(1.0, np.minimum(tau, 1.0 - tau) / ramp_frac)       # shape of dV/dtau
    v = np.concatenate([[0.0], np.cumsum(0.5 * (a[1:] + a[:-1]) * np.diff(tau))])
    v = v0 + (v1 - v0) * v / v[-1]
    return schedule(list(zip(tau, v)))


@dataclass
class M2Segment:
    """One Mission Planner v2 segment. All schedules are functions of tau = t/T.
      speed      horizontal true airspeed [m/s]
      climb      climb rate [m/s] (+ up)
      nacelle    nacelle angle [deg] (90 helicopter, 0 airplane)
      rpm        rotor speed [RPM]
      nacelle_of_speed  optional: nacelle angle as a function of airspeed (a
                 conversion path in the speed-nacelle plane); overrides `nacelle`
      headwind   along-track headwind [m/s] (ground speed = V_h - headwind);
                 a number or a schedule of tau
    Controls are not scheduled: they come from the online 6-DOF trim at every
    step (use_online_trim=True, the only mode implemented)."""
    name: str
    kind: str
    duration_s: float
    dt_s: float
    speed: Callable[[float], float]
    climb: Callable[[float], float]
    nacelle: Callable[[float], float]
    rpm: Callable[[float], float]
    nacelle_of_speed: _Optional[Callable[[float], float]] = None
    headwind_mps: object = 0.0
    use_online_trim: bool = True

    def headwind(self, tau: float) -> float:
        return float(self.headwind_mps(tau)) if callable(self.headwind_mps) else float(self.headwind_mps)

    def state(self, tau: float) -> dict:
        V = self.speed(tau)
        n = self.nacelle_of_speed(V) if self.nacelle_of_speed is not None else self.nacelle(tau)
        return dict(V_h=V, climb=self.climb(tau), nacelle=n, rpm=self.rpm(tau), wind=self.headwind(tau))


class MissionPlannerV2:
    """Time-stepped transition mission with online 6-DOF trim at every step.

    Per step: ISA atmosphere at the current altitude -> flight-path angle
    gamma = atan2(climb, V_h) and along-path acceleration from the speed
    schedule -> 6-DOF trim at the current gross mass and CG (fuel state) ->
    power required vs available -> limit checks (trim status, rotor stall,
    reverse flow, tip Mach, wing stall, power margin, nacelle tilt rate, RPM
    range, reserve fuel) -> fuel burn, mass, altitude and ground-distance
    update (explicit Euler). State continuity (airspeed, nacelle angle, RPM) is
    checked at every segment boundary. The first violation raises
    MissionInfeasibleError(segment, time, reason) unless stop_on_violation=False,
    in which case violations are logged and the mission continues."""

    def __init__(self, rotor, airfoil_provider, start_altitude_m, fuel_kg, payload_kg=None,
                 dISA_K=0.0, reserve_fuel_kg=0.0, stop_on_violation=True, n_r=25, n_psi=36,
                 continuity_tol=(0.5, 0.5, 1.0), verbose=True):
        import m2.aircraft_input_m2 as CFG
        self.CFG = CFG
        self.rotor, self.afp = rotor, airfoil_provider
        self.h = start_altitude_m
        self.fuel = fuel_kg
        self.payload = payload_kg
        self.dISA = dISA_K
        self.reserve = reserve_fuel_kg
        self.stop = stop_on_violation
        self.n_r, self.n_psi = n_r, n_psi
        self.tol = continuity_tol
        self.verbose = verbose
        self.t = 0.0
        self.x_ground = 0.0
        self.log = []
        self.violations = []
        self._x0 = None
        self._last_state = None

    def mass_kg(self):
        return self.CFG.get_default_aircraft().mass_kg(fuel_kg=self.fuel, payload_kg=self.payload)

    def _violate(self, seg, reason):
        from mission import MissionInfeasibleError
        self.violations.append((seg.name, self.t, reason))
        if self.stop:
            raise MissionInfeasibleError(seg.name, self.t, reason)
        if self.verbose:
            print(f"    VIOLATION t={self.t:.0f}s [{seg.name}]: {reason}")

    def _check_continuity(self, seg):
        if self._last_state is None:
            return
        s0, p = seg.state(0.0), self._last_state
        dv, dn, dr = abs(s0['V_h'] - p['V_h']), abs(s0['nacelle'] - p['nacelle']), abs(s0['rpm'] - p['rpm'])
        if dv > self.tol[0] or dn > self.tol[1] or dr > self.tol[2]:
            self._violate(seg, f"state discontinuity at segment start: dV={dv:.2f} m/s, "
                               f"d_nacelle={dn:.2f} deg, dRPM={dr:.1f}")

    def run_segment(self, seg: M2Segment):
        from environment import isa
        from m2.trim_6dof import trim_6dof, TrimCondition, diagnose
        CFG = self.CFG
        self._check_continuity(seg)
        lim = CFG.ControlLimits()
        n_steps = max(1, int(round(seg.duration_s / seg.dt_s)))
        dt = seg.duration_s / n_steps
        if self.verbose:
            print(f"  segment '{seg.name}' ({seg.kind}, {seg.duration_s:.0f} s, {n_steps} steps)")
        for k in range(n_steps + 1):
            tau = k / n_steps
            st = seg.state(tau)
            # along-path acceleration and nacelle tilt rate from the schedules
            d = 1e-3
            ta, tb = max(0.0, tau - d), min(1.0, tau + d)
            sa, sb = seg.state(ta), seg.state(tb)
            Va, Vb = np.hypot(sa['V_h'], sa['climb']), np.hypot(sb['V_h'], sb['climb'])
            accel = (Vb - Va) / ((tb - ta) * seg.duration_s)
            n_rate = (sb['nacelle'] - sa['nacelle']) / ((tb - ta) * seg.duration_s)
            V = float(np.hypot(st['V_h'], st['climb']))
            gamma = float(np.arctan2(st['climb'], st['V_h'])) if V > 1e-6 else 0.0

            atm = isa(self.h, self.dISA)
            m = self.mass_kg()
            ac = CFG.get_default_aircraft(m)
            cond = TrimCondition(V_mps=V, nacelle_deg=st['nacelle'], omega_rad_s=CFG.rpm_to_omega(st['rpm']),
                                 rho=atm.density_kg_m3, a_sound=atm.speed_of_sound_mps, gamma_rad=gamma,
                                 accel_mps2=accel, altitude_m=self.h, fuel_kg=self.fuel, payload_kg=self.payload,
                                 P_avail_per_rotor_W=CFG.POWER_MODEL.power_available_W(atm))
            tr = trim_6dof(ac, self.rotor, self.afp, cond, x0=self._x0, n_r=self.n_r, n_psi=self.n_psi)
            if tr.status in ('ok', 'ok_at_limit'):
                self._x0 = tr.x

            # ---- feasibility checks at this time step ----
            if not tr.feasible:
                self._violate(seg, diagnose(tr))
            if abs(n_rate) > lim.nacelle_rate_deg_s + 1e-6:
                self._violate(seg, f"nacelle tilt rate {n_rate:.1f} deg/s > {lim.nacelle_rate_deg_s} deg/s")
            if not (lim.rpm[0] <= st['rpm'] <= lim.rpm[1]):
                self._violate(seg, f"RPM {st['rpm']:.0f} outside {lim.rpm}")
            if self.fuel < self.reserve:
                self._violate(seg, f"fuel {self.fuel:.0f} kg below reserve {self.reserve:.0f} kg")

            P = tr.P_req_W if np.isfinite(tr.P_req_W) else 0.0
            a_st = ac.wing.alpha_stall_deg
            has_aero = tr.aero is not None
            entry = dict(
                t_s=self.t, segment=seg.name, kind=seg.kind, altitude_m=self.h,
                V_h_mps=st['V_h'], V_mps=V, ground_speed_mps=st['V_h'] - st['wind'], headwind_mps=st['wind'],
                climb_mps=st['climb'], gamma_deg=np.degrees(gamma), accel_mps2=accel,
                nacelle_deg=st['nacelle'], nacelle_rate_deg_s=n_rate, rpm=st['rpm'],
                mass_kg=m, fuel_kg=self.fuel, distance_km=self.x_ground / 1e3,
                theta_deg=tr.theta_deg, phi_deg=tr.phi_deg, alpha_deg=tr.alpha_deg,
                collective_deg=tr.collective_deg, theta1c_deg=tr.theta1c_deg, elevator_deg=tr.elevator_deg,
                dcoll_deg=tr.dcoll_deg, dcyc_deg=tr.dcyc_deg, aileron_deg=tr.aileron_deg,
                rudder_deg=tr.rudder_deg,
                P_req_kW=P / 1e3, P_avail_kW=tr.P_avail_W / 1e3,
                rotor_stall_pct=100 * tr.stalled_fraction, rotor_stall_margin_deg=tr.stall_margin_deg,
                wing_alpha_deg=tr.aero.alpha_wing_deg if has_aero else np.nan,
                wing_alpha_margin_deg=(a_st - abs(tr.aero.alpha_wing_deg)) if (has_aero and V > 10) else np.nan,
                tip_mach=tr.adv_tip_mach, reverse_pct=100 * tr.reverse_flow_fraction,
                rotor_lift_pct=100 * tr.rotor_lift_N / (m * CFG.G),
                wing_lift_pct=100 * tr.wing_lift_N / (m * CFG.G),
                trim_status=tr.status, residual=tr.residual_norm, flags=";".join(tr.flags),
                feasible=tr.feasible,
            )
            self.log.append(entry)
            if self.verbose and (k % max(1, n_steps // 6) == 0 or k == n_steps):
                print(f"    t={self.t:6.0f}s h={self.h:6.0f} V={V:5.1f} i_n={st['nacelle']:5.1f} "
                      f"rpm={st['rpm']:4.0f} P={P / 1e3:5.0f}/{tr.P_avail_W / 1e3:4.0f} kW "
                      f"coll={tr.collective_deg:5.1f} t1c={tr.theta1c_deg:5.2f} de={tr.elevator_deg:6.2f} "
                      f"fuel={self.fuel:6.1f} {tr.status}{' ' + str(tr.flags) if tr.flags else ''}")
            if k == n_steps:
                break
            # ---- state update (explicit Euler over dt) ----
            self.fuel -= CFG.FUEL_MODEL.burn_rate_kg_s(P) * dt
            self.h += st['climb'] * dt
            self.x_ground += (st['V_h'] - st['wind']) * dt
            self.t += dt
        self._last_state = seg.state(1.0)

    def run(self, segments):
        for seg in segments:
            self.run_segment(seg)
        return self.log
