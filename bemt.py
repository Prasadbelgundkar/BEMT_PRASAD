"""
bemt.py
-------
Blade Element Momentum Theory solver.

Handles both helicopter (hover/climb/descent, axial inflow, Vc along the
rotor axis with Omega*r tangential) and propeller/axial-forward-flight mode
(freestream aligned with the rotor axis -- this is exactly the tiltrotor
airplane-mode case). Because the inflow is axisymmetric in BOTH regimes for
an axial-flow rotor, one element-solve function serves both; only the axial
velocity term (climb/descent vs. forward flight true airspeed) changes,
which is exactly how Task 1 asks for it to be parameterized.

Per-element algorithm (matches the flow you described):
    1. Guess induced velocity v.
    2. U_T = Omega*r,  U_P = V_axial + v
    3. phi = atan2(U_P, U_T);  alpha = twist(r) + collective - phi
    4. Look up Cl, Cd (with optional stall flag) from the airfoil model.
    5. Apply Prandtl-Glauert compressibility correction to Cl based on
       local Mach number.
    6. Compute Prandtl tip-loss factor F (and optional root-loss factor).
    7. Residual = dT_BET(v) - dT_momentum(v); solve for v with Brent's method.
    8. Integrate sectional thrust/torque radially -> rotor T, Q, P.

Nondimensional outputs (CT, CQ, CP, FM) use helicopter convention:
    CT = T / (rho * A * (Omega R)^2)
    CQ = Q / (rho * A * (Omega R)^2 * R)
    CP = P / (rho * A * (Omega R)^3)
    FM = CT^1.5 / (sqrt(2) * CP)      [hover only, Vc=0]
"""

from dataclasses import dataclass
from typing import Callable, Optional, List
import numpy as np
from scipy.optimize import brentq

from rotor import Rotor
from airfoil import prandtl_glauert_correct

# numpy >= 2.0 renamed trapz -> trapezoid; support either version.
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


@dataclass
class ElementResult:
    r: float
    x: float                # r/R
    v_induced: float
    phi_rad: float
    alpha_rad: float
    Cl: float
    Cd: float
    mach: float
    tip_loss_F: float
    stalled: bool
    dT_dr: float
    dQ_dr: float
    converged: bool = True    # False if Brent solver failed to find a bracket


@dataclass
class RotorPerformance:
    thrust_N: float
    torque_Nm: float
    power_W: float
    CT: float
    CQ: float
    CP: float
    figure_of_merit: Optional[float]
    propulsive_efficiency: Optional[float]
    elements: List[ElementResult]
    stalled_fraction: float
    max_tip_mach: float
    converged: bool


def prandtl_tip_loss(B: int, R: float, r: float, phi: float,
                      root_cutout: float = 0.0, include_root_loss: bool = False) -> float:
    """Standard Prandtl tip-loss factor F, with an optional symmetric
    root-loss factor (Task 1: 'root loss may be included if justified')."""
    sin_phi = max(abs(np.sin(phi)), 1e-4)

    f_tip = (B / 2.0) * (R - r) / (r * sin_phi)
    F_tip = (2.0 / np.pi) * np.arccos(np.clip(np.exp(-f_tip), -1.0, 1.0))

    if not include_root_loss or root_cutout <= 0.0:
        return max(F_tip, 1e-3)

    f_root = (B / 2.0) * (r - root_cutout) / (r * sin_phi)
    F_root = (2.0 / np.pi) * np.arccos(np.clip(np.exp(-f_root), -1.0, 1.0))
    return max(F_tip * F_root, 1e-3)


def solve_element(r: float, R: float, B: int, chord: float, twist_total: float,
                   omega: float, v_axial: float, rho: float, a_sound: float,
                   airfoil, root_cutout: float = 0.0, include_root_loss: bool = False,
                   v_scan_range=(-80.0, 150.0), n_scan: int = 400) -> ElementResult:
    """Solve one blade element for induced velocity v via root-finding on the
    BET/momentum thrust residual, then evaluate all derived quantities.

    The residual dT_BET(v) - dT_momentum(v) is NOT monotonic in v (it can
    have a spurious sign change at large negative v as well as the physical
    root near the expected hover/climb induced velocity), so a blind
    two-point bracket is unreliable. Instead we scan v across a wide range,
    take the FIRST sign change encountered going from v=0 outward towards
    positive v (the physically expected branch for a rotor producing
    positive thrust; negative-thrust/autorotation cases are handled by also
    scanning towards negative v if no positive-side root is found), then
    refine with Brent's method inside that bracket.
    """

    def residual(v):
        U_T = omega * r
        U_P = v_axial + v
        phi = np.arctan2(U_P, U_T)
        alpha = twist_total - phi
        Cl, Cd, _ = airfoil.get_coeffs(alpha)

        U_res = (U_T ** 2 + U_P ** 2) ** 0.5
        mach = U_res / a_sound
        if mach < 0.7:
            Cl, Cd = prandtl_glauert_correct(Cl, Cd, mach)

        F = prandtl_tip_loss(B, R, r, phi, root_cutout, include_root_loss)

        dL = 0.5 * rho * U_res ** 2 * chord * Cl
        dD = 0.5 * rho * U_res ** 2 * chord * Cd
        dT_BET = B * (dL * np.cos(phi) - dD * np.sin(phi))

        # Differential-annulus momentum theory (per unit span), with tip loss.
        dT_mom = 4.0 * np.pi * r * rho * F * abs(v_axial + v) * v

        return dT_BET - dT_mom

    def find_bracket(v_start, v_end, n):
        vs = np.linspace(v_start, v_end, n)
        prev_v, prev_f = vs[0], residual(vs[0])
        for vv in vs[1:]:
            f = residual(vv)
            if np.isfinite(prev_f) and np.isfinite(f) and prev_f * f < 0:
                return prev_v, vv
            prev_v, prev_f = vv, f
        return None

    v_lo_scan, v_hi_scan = v_scan_range
    # Search outward from v=0 towards positive v first (typical thrusting rotor).
    bracket = find_bracket(0.0, v_hi_scan, n_scan // 2)
    if bracket is None:
        # Fall back to negative-v branch (e.g. windmilling/negative thrust).
        bracket = find_bracket(0.0, v_lo_scan, n_scan // 2)

    if bracket is not None:
        try:
            v = brentq(residual, bracket[0], bracket[1], xtol=1e-8, maxiter=200)
            converged = True
        except ValueError:
            v = 0.0
            converged = False
    else:
        v = 0.0
        converged = False

    U_T = omega * r
    U_P = v_axial + v
    phi = np.arctan2(U_P, U_T)
    alpha = twist_total - phi
    Cl, Cd, stalled = airfoil.get_coeffs(alpha)
    U_res = (U_T ** 2 + U_P ** 2) ** 0.5
    mach = U_res / a_sound
    if mach < 0.7:
        Cl, Cd = prandtl_glauert_correct(Cl, Cd, mach)
    F = prandtl_tip_loss(B, R, r, phi, root_cutout, include_root_loss)
    dL = 0.5 * rho * U_res ** 2 * chord * Cl
    dD = 0.5 * rho * U_res ** 2 * chord * Cd
    dT_dr = B * (dL * np.cos(phi) - dD * np.sin(phi))
    dQ_dr = B * r * (dL * np.sin(phi) + dD * np.cos(phi))

    return ElementResult(r=r, x=r / R, v_induced=v, phi_rad=phi, alpha_rad=alpha,
                          Cl=Cl, Cd=Cd, mach=mach, tip_loss_F=F, stalled=stalled,
                          dT_dr=dT_dr, dQ_dr=dQ_dr, converged=converged)


def run_bemt(rotor: Rotor, airfoil_provider: Callable[[float], object],
             omega_rad_s: float, collective_rad: float, rho: float, a_sound: float,
             v_axial: float = 0.0, n_stations: int = 60,
             include_root_loss: bool = False) -> RotorPerformance:
    """
    Run the full BEMT solve across the blade span and integrate.

    Parameters
    ----------
    rotor : Rotor
    airfoil_provider : callable(x) -> airfoil object with get_coeffs(alpha)
        Allows a single airfoil (lambda x: my_airfoil) or a radially blended
        set of airfoils.
    omega_rad_s : rotor speed
    collective_rad : collective pitch added uniformly to the built-in twist
    v_axial : climb/descent velocity (hover branch) OR forward/axial
        freestream velocity (propeller/axial-flight branch). Same parameter,
        different physical meaning depending on flight mode -- this is what
        lets one solver serve both regimes per Task 1.
    """
    R = rotor.radius_m
    x_stations = np.linspace(rotor.root_cutout_m / R, 1.0, n_stations)
    r_stations = x_stations * R

    elements: List[ElementResult] = []
    for x, r in zip(x_stations, r_stations):
        chord = rotor.chord_fn(x)
        twist_total = rotor.twist_fn(x) + collective_rad
        airfoil = airfoil_provider(x)
        el = solve_element(r, R, rotor.num_blades, chord, twist_total, omega_rad_s,
                            v_axial, rho, a_sound, airfoil,
                            root_cutout=rotor.root_cutout_m,
                            include_root_loss=include_root_loss)
        elements.append(el)

    dT = np.array([e.dT_dr for e in elements])
    dQ = np.array([e.dQ_dr for e in elements])
    T = _trapz(dT, r_stations)
    Q = _trapz(dQ, r_stations)
    P = Q * omega_rad_s

    A = rotor.disk_area_m2()
    vtip = rotor.tip_speed_mps(omega_rad_s)
    CT = T / (rho * A * vtip ** 2 + 1e-12)
    CQ = Q / (rho * A * vtip ** 2 * R + 1e-12)
    CP = P / (rho * A * vtip ** 3 + 1e-12)

    FM = None
    if abs(v_axial) < 1e-6 and CT > 0 and CP > 0:
        FM = CT ** 1.5 / (np.sqrt(2.0) * CP)

    eta_prop = None
    if v_axial > 1e-6 and P > 0:
        eta_prop = T * v_axial / P

    stalled_fraction = float(np.mean([e.stalled for e in elements]))
    max_tip_mach = max(e.mach for e in elements)
    all_converged = all(e.converged for e in elements)

    return RotorPerformance(
        thrust_N=T, torque_Nm=Q, power_W=P, CT=CT, CQ=CQ, CP=CP,
        figure_of_merit=FM, propulsive_efficiency=eta_prop,
        elements=elements, stalled_fraction=stalled_fraction,
        max_tip_mach=max_tip_mach, converged=all_converged,
    )


def advance_ratio(v_axial: float, omega_rad_s: float, R: float) -> float:
    """Propeller-convention advance ratio J = V / (n D) = pi*V/(Omega*R)."""
    n = omega_rad_s / (2 * np.pi)
    D = 2 * R
    return v_axial / (n * D) if n > 0 else 0.0


def trim_hover_collective(
    rotor: "Rotor",
    airfoil_provider: "Callable[[float], object]",
    omega_rad_s: float,
    target_thrust_N: float,
    rho: float,
    a_sound: float,
    coll_range_deg=(-5.0, 25.0),
) -> Optional[float]:
    """Find the collective pitch [deg] that produces *target_thrust_N* in hover
    (v_axial=0). Uses Brent's method inside *coll_range_deg*.

    Returns
    -------
    float or None
        Collective pitch in degrees, or None if the target thrust cannot be
        achieved inside the requested collective range.
    """
    def residual(coll_deg: float) -> float:
        perf = run_bemt(rotor, airfoil_provider, omega_rad_s,
                        np.radians(coll_deg), rho, a_sound, v_axial=0.0)
        return perf.thrust_N - target_thrust_N

    f_lo = residual(coll_range_deg[0])
    f_hi = residual(coll_range_deg[1])
    if not (np.isfinite(f_lo) and np.isfinite(f_hi)):
        return None
    if f_lo * f_hi > 0:
        return None  # target thrust not achievable in this collective range
    try:
        return brentq(residual, coll_range_deg[0], coll_range_deg[1],
                      xtol=0.01, maxiter=60)
    except ValueError:
        return None
