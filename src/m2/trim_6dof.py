"""
trim_6dof.py -- six-degree-of-freedom steady trim of the twin-rotor tiltrotor
===========================================================================
(report Section 2.2 / 6)

Unknowns  x = [theta, phi, theta0, d_lon, d_lat, d_ped]
    theta   pitch attitude [rad]   (alpha = theta - gamma, wings level reference)
    phi     roll attitude [rad]
    theta0  symmetric collective [rad] (on top of the built-in twist)
    d_lon, d_lat, d_ped  normalized pilot inputs in [-1, 1], mapped to the
            rotor controls and surfaces by ControlMixing (aircraft_input_m2):
              pitch: theta1c (both rotors) + elevator
              roll : differential collective + flaperons (ailerons)
              yaw  : differential lateral cyclic theta1s + rudder
Fixed     airspeed V, flight-path angle gamma, longitudinal acceleration
          a_x (quasi-steady), nacelle angle i_n, rotor speed, altitude, mass,
          sideslip beta = 0 (coordinated).

Residuals  r = [sum FX, FY, FZ]/W  and  [sum MX, MY, MZ]/(W * 1 m) about the CG,
in body axes, including both rotors (right CCW, left CW = mirror image), the
wing/flaperons, H-tail/elevator, V-tail/rudder, fuselage drag, weight and the
inertial term m a_x along the flight path.

Method     scipy.optimize.least_squares, trust-region-reflective with bounds,
           several seeds, warm start from a neighbouring solution.
Converged  ||r||_2 < 1e-4 (|F| ~< 7 N, |M| ~< 7 N m at 7.2 t).

Status (first that applies):
  'ok'                   converged, no unknown on its bound
  'ok_at_limit'          converged with an unknown on its bound (marginal)
  'control_saturation'   not converged and an unknown is on its bound
  'excessive_residual'   not converged, 1e-4 <= ||r|| < 1e-2, no bound active
  'no_physical_solution' not converged, ||r|| >= 1e-2 from every seed
Constraint flags (evaluated on every converged point):
  rotor_stall, reverse_flow, tip_mach, wing_stall, power
A point is FEASIBLE if its status is 'ok' (or 'ok_at_limit') and no flag is set.
"""
from dataclasses import dataclass, field
from typing import Optional, List

import numpy as np
from scipy.optimize import least_squares

from m2.edgewise_bemt import run_edgewise_bemt, EdgewiseRotorResult
from m2.aero_models import airframe_loads, AirframeLoads
from m2.frames import R_body_to_inertial

RES_TOL = 1e-4
RES_NOPHYS = 1e-2
UNKNOWNS = ["theta", "phi", "theta0", "d_lon", "d_lat", "d_ped"]


def shaft_angle_of_attack(alpha_body_rad: float, nacelle_deg: float) -> float:
    """alpha_shaft = 90 deg - i_n - alpha (see trim_solver.shaft_angle_of_attack)."""
    return np.radians(90.0 - nacelle_deg) - alpha_body_rad


@dataclass
class TrimCondition:
    V_mps: float
    nacelle_deg: float
    omega_rad_s: float
    rho: float
    a_sound: float
    gamma_rad: float = 0.0
    accel_mps2: float = 0.0
    altitude_m: float = 0.0
    fuel_kg: Optional[float] = None
    payload_kg: Optional[float] = None
    P_avail_per_rotor_W: Optional[float] = None

    @property
    def rpm(self) -> float:
        return self.omega_rad_s * 60.0 / (2.0 * np.pi)


@dataclass
class TrimResult:
    cond: TrimCondition
    status: str
    feasible: bool
    flags: List[str]
    residual_norm: float
    x: np.ndarray                         # unknowns (rad for angles)
    at_bound: List[str]
    # attitude and controls [deg]
    theta_deg: float = 0.0
    phi_deg: float = 0.0
    alpha_deg: float = 0.0
    collective_deg: float = 0.0
    theta1c_deg: float = 0.0
    theta1s_deg: float = 0.0
    dcoll_deg: float = 0.0
    dcyc_deg: float = 0.0
    elevator_deg: float = 0.0
    aileron_deg: float = 0.0
    rudder_deg: float = 0.0
    d_lon: float = 0.0
    d_lat: float = 0.0
    d_ped: float = 0.0
    # residual loads (physical units), body axes about the CG
    F_res_N: np.ndarray = field(default_factory=lambda: np.zeros(3))
    M_res_Nm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # performance / sharing
    P_req_W: float = 0.0
    P_avail_W: float = 0.0
    rotor_lift_N: float = 0.0             # earth-vertical lift of both rotors
    wing_lift_N: float = 0.0              # earth-vertical lift of wing + tail
    rotor_propulsive_N: float = 0.0       # rotor force along the flight path
    stalled_fraction: float = 0.0         # max of the two rotors (loaded area)
    stall_margin_deg: float = 0.0
    reverse_flow_fraction: float = 0.0
    adv_tip_mach: float = 0.0
    nfev: int = 0
    right: Optional[EdgewiseRotorResult] = None
    left: Optional[EdgewiseRotorResult] = None
    aero: Optional[AirframeLoads] = None

    @property
    def power_margin_frac(self) -> float:
        return 1.0 - self.P_req_W / self.P_avail_W if self.P_avail_W > 0 else np.nan


# ---------------------------------------------------------------------------
def _bounds(ac):
    L = ac.limits
    lo = [np.radians(L.attitude_deg[0]), np.radians(L.roll_deg[0]), np.radians(L.collective_deg[0]), -1, -1, -1]
    hi = [np.radians(L.attitude_deg[1]), np.radians(L.roll_deg[1]), np.radians(L.collective_deg[1]), 1, 1, 1]
    return np.array(lo, float), np.array(hi, float)


def evaluate(x, ac, rotor, airfoil_provider, cond: TrimCondition, n_r=25, n_psi=36, full=False):
    """Residual vector (normalized) for unknowns x; with full=True also the
    component results."""
    theta, phi, th0, d_lon, d_lat, d_ped = x
    n = cond.nacelle_deg
    mass_kw = {k: v for k, v in (("fuel_kg", cond.fuel_kg), ("payload_kg", cond.payload_kg)) if v is not None}
    W = ac.W_MTOW_N
    m = W / 9.80665
    alpha = theta - cond.gamma_rad
    eff = ac.mixing.effectors(n, d_lon, d_lat, d_ped)
    t1c = np.radians(eff["theta1c_deg"])
    dcol = np.radians(eff["dcoll_deg"])
    dcyc = np.radians(eff["dcyc_deg"])
    a_s = shaft_angle_of_attack(alpha, n)
    V = max(cond.V_mps, 0.05)

    common = dict(rotor=rotor, airfoil_provider=airfoil_provider, V_inf=V,
                  omega_rad_s=cond.omega_rad_s, theta1c_rad=t1c, alpha_shaft_rad=a_s,
                  nacelle_angle_deg=n, rho=cond.rho, a_sound=cond.a_sound, n_r=n_r, n_psi=n_psi)
    right = run_edgewise_bemt(theta0_rad=th0 + dcol, theta1s_rad=-dcyc, rotation='ccw',
                              r_hub_body=ac.hub_from_cg(n, 'right', **mass_kw), **common)
    left = run_edgewise_bemt(theta0_rad=th0 - dcol, theta1s_rad=+dcyc, rotation='cw',
                             r_hub_body=ac.hub_from_cg(n, 'left', **mass_kw), **common)
    aero = airframe_loads(ac, cond.V_mps, alpha, cond.rho, n, elevator_deg=eff["elevator_deg"],
                          aileron_deg=eff["aileron_deg"], rudder_deg=eff["rudder_deg"], **mass_kw)

    G = W * np.array([-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)])
    v_hat = np.array([np.cos(alpha), 0.0, np.sin(alpha)])          # flight-path direction, body
    F = right.forces_body_N + left.forces_body_N + aero.F_body_N + G - m * cond.accel_mps2 * v_hat
    M = right.moments_body_Nm + left.moments_body_Nm + aero.M_cg_Nm
    r = np.concatenate([F / W, M / W])
    if not full:
        return r
    return r, F, M, right, left, aero, eff


def _seeds(ac, cond, rotor, x0):
    seeds = [] if x0 is None else [np.asarray(x0, float)]
    n = np.radians(cond.nacelle_deg)
    R = rotor.radius_m
    v_h = np.sqrt(0.5 * ac.W_MTOW_N / (2.0 * cond.rho * np.pi * R ** 2))
    for th_deg in (2.0, 6.0, -4.0, 12.0):
        th = np.radians(th_deg)
        alpha = th - cond.gamma_rad
        V_through = cond.V_mps * np.cos(n + alpha)
        phi75 = np.arctan2(max(V_through, 0.0) + v_h * np.sin(n) ** 2, cond.omega_rad_s * 0.75 * R)
        coll = phi75 - rotor.twist_fn(0.75) + np.radians(5.0)
        seeds.append(np.array([th, 0.0, coll, 0.0, 0.0, 0.0]))
    return seeds


def trim_6dof(ac, rotor, airfoil_provider, cond: TrimCondition, x0=None, n_r=25, n_psi=36,
              max_nfev=60, default_seeds=True) -> TrimResult:
    """Solve the 6-DOF trim. x0 (optional) is tried first; with
    default_seeds=False ONLY x0 is used (for demonstrating numerical failure)."""
    lo, hi = _bounds(ac)
    span = hi - lo
    best = None
    nfev = 0
    seeds = _seeds(ac, cond, rotor, x0) if default_seeds else [np.asarray(x0, float)]
    for s in seeds:
        s = np.clip(s, lo + 1e-6 * span, hi - 1e-6 * span)
        try:
            sol = least_squares(evaluate, s, bounds=(lo, hi), method='trf', x_scale=span / 10.0,
                                diff_step=1e-4, xtol=1e-10, ftol=1e-12, gtol=1e-12, max_nfev=max_nfev,
                                args=(ac, rotor, airfoil_provider, cond, n_r, n_psi))
        except Exception:
            continue
        nfev += sol.nfev
        norm = float(np.linalg.norm(sol.fun))
        if not np.isfinite(norm):
            continue
        if best is None or norm < best[1]:
            best = (sol.x, norm)
        if norm < RES_TOL:
            break
    if best is None:
        return TrimResult(cond=cond, status='no_physical_solution', feasible=False, flags=[],
                          residual_norm=np.inf, x=np.full(6, np.nan), at_bound=[], nfev=nfev)
    return _assemble(best[0], best[1], ac, rotor, airfoil_provider, cond, n_r, n_psi, lo, hi, nfev)


def _assemble(x, norm, ac, rotor, airfoil_provider, cond, n_r, n_psi, lo, hi, nfev) -> TrimResult:
    r, F, M, right, left, aero, eff = evaluate(x, ac, rotor, airfoil_provider, cond, n_r, n_psi, full=True)
    tol_b = 1e-3 * (hi - lo)
    at_bound = [UNKNOWNS[i] for i in range(6) if x[i] <= lo[i] + tol_b[i] or x[i] >= hi[i] - tol_b[i]]
    converged = norm < RES_TOL
    if converged:
        status = 'ok_at_limit' if at_bound else 'ok'
    elif at_bound:
        status = 'control_saturation'
    elif norm < RES_NOPHYS:
        status = 'excessive_residual'
    else:
        status = 'no_physical_solution'

    theta, phi = x[0], x[1]
    alpha = theta - cond.gamma_rad
    # earth-vertical lift sharing and propulsive force (wind axes, level reference)
    Rbi = R_body_to_inertial(np.degrees(phi), np.degrees(theta), 0.0)
    up = lambda Fb: -(Rbi @ Fb)[2]
    F_rot = right.forces_body_N + left.forces_body_N
    v_hat = np.array([np.cos(alpha), 0.0, np.sin(alpha)])
    P_req = right.power_W + left.power_W
    P_av = 2.0 * cond.P_avail_per_rotor_W if cond.P_avail_per_rotor_W else 0.0

    L = ac.limits
    flags = []
    if converged:
        if max(right.stalled_fraction_fwd, left.stalled_fraction_fwd) > L.max_stall_fraction:
            flags.append('rotor_stall')
        if max(right.reverse_flow_area_fraction, left.reverse_flow_area_fraction) > L.max_reverse_flow_fraction:
            flags.append('reverse_flow')
        if max(right.adv_tip_mach, left.adv_tip_mach) > L.max_tip_mach:
            flags.append('tip_mach')
        if aero.wing_stalled and cond.V_mps > 10.0:
            flags.append('wing_stall')
        if P_av > 0 and P_req > P_av * (1.0 - L.min_power_margin_frac):
            flags.append('power')
    feasible = status in ('ok', 'ok_at_limit') and not flags

    return TrimResult(
        cond=cond, status=status, feasible=feasible, flags=flags, residual_norm=norm, x=x,
        at_bound=at_bound,
        theta_deg=np.degrees(theta), phi_deg=np.degrees(phi), alpha_deg=np.degrees(alpha),
        collective_deg=np.degrees(x[2]), theta1c_deg=eff["theta1c_deg"], theta1s_deg=0.0,
        dcoll_deg=eff["dcoll_deg"], dcyc_deg=eff["dcyc_deg"], elevator_deg=eff["elevator_deg"],
        aileron_deg=eff["aileron_deg"], rudder_deg=eff["rudder_deg"],
        d_lon=x[3], d_lat=x[4], d_ped=x[5],
        F_res_N=F, M_res_Nm=M, P_req_W=P_req, P_avail_W=P_av,
        rotor_lift_N=up(F_rot), wing_lift_N=up(aero.F_body_N) - up(aero.D_fuselage_N * -v_hat),
        rotor_propulsive_N=float(F_rot @ v_hat),
        stalled_fraction=max(right.stalled_fraction_fwd, left.stalled_fraction_fwd),
        stall_margin_deg=min(right.stall_margin_deg, left.stall_margin_deg),
        reverse_flow_fraction=max(right.reverse_flow_area_fraction, left.reverse_flow_area_fraction),
        adv_tip_mach=max(right.adv_tip_mach, left.adv_tip_mach),
        nfev=nfev, right=right, left=left, aero=aero,
    )


def make_condition(V_mps, nacelle_deg, rpm=None, altitude_m=None, gamma_deg=0.0, accel_mps2=0.0,
                   fuel_kg=None, dISA_K=0.0):
    """Convenience constructor using the M2 configuration defaults."""
    import m2.aircraft_input_m2 as CFG
    from environment import isa
    h = CFG.REFERENCE_ALTITUDE_M if altitude_m is None else altitude_m
    atm = isa(h, dISA_K)
    rpm = CFG.HOVER_RPM if rpm is None else rpm
    return TrimCondition(V_mps=V_mps, nacelle_deg=nacelle_deg, omega_rad_s=CFG.rpm_to_omega(rpm),
                         rho=atm.density_kg_m3, a_sound=atm.speed_of_sound_mps,
                         gamma_rad=np.radians(gamma_deg), accel_mps2=accel_mps2, altitude_m=h,
                         fuel_kg=fuel_kg, P_avail_per_rotor_W=CFG.POWER_MODEL.power_available_W(atm))


def diagnose(tr: TrimResult) -> str:
    """Human-readable classification of a trim outcome (report Sections 6.3 / 7.2):
    numerical failure, insufficient control authority, rotor stall, wing stall,
    power limitation, tip-Mach limitation, reverse flow, or physical infeasibility."""
    if tr.feasible:
        return "feasible trim"
    if tr.status in ('ok', 'ok_at_limit'):
        names = {'rotor_stall': f"rotor stall ({100*tr.stalled_fraction:.0f} % of loaded disk)",
                 'reverse_flow': f"reverse flow ({100*tr.reverse_flow_fraction:.1f} % of disk)",
                 'tip_mach': f"advancing-tip Mach {tr.adv_tip_mach:.3f}",
                 'wing_stall': f"wing stall (alpha_w = {tr.aero.alpha_wing_deg:.1f} deg)",
                 'power': f"power limitation ({tr.P_req_W/1e3:.0f} kW required, {tr.P_avail_W/1e3:.0f} kW available)"}
        return "trimmed but limited by " + ", ".join(names[f] for f in tr.flags)
    if tr.status == 'control_saturation':
        eff = {'theta0': "collective", 'd_lon': "pitch control (elevator / theta1c)", 'd_lat': "roll control",
               'd_ped': "yaw control", 'theta': "pitch attitude bound", 'phi': "roll attitude bound"}
        return "insufficient control authority: " + ", ".join(eff[b] for b in tr.at_bound) + " at its limit"
    if (tr.aero is not None and tr.cond.V_mps > 10.0
            and abs(tr.aero.alpha_wing_deg) >= tr.aero.alpha_wing_stall_deg - 0.25):
        return (f"no physical solution: wing at/over stall (alpha_w = {tr.aero.alpha_wing_deg:.1f} deg) cannot "
                f"carry the weight; FZ residual {tr.F_res_N[2]/1e3:.1f} kN")
    if tr.P_avail_W > 0 and tr.P_req_W > tr.P_avail_W:
        return (f"no physical solution near the power/stall limit (best point needs {tr.P_req_W/1e3:.0f} kW "
                f"of {tr.P_avail_W/1e3:.0f} kW, rotor stall {100*tr.stalled_fraction:.0f} %)")
    if tr.status == 'excessive_residual':
        return f"numerical: residual stalled at {tr.residual_norm:.1e} (not a converged trim)"
    return f"no physical solution (residual {tr.residual_norm:.2e} from every seed)"
