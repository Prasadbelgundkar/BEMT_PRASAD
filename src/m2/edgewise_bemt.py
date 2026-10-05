"""
edgewise_bemt.py -- azimuth-resolved blade-element / momentum rotor model
=========================================================================
Milestone 2 extension of the axisymmetric M1 solver (bemt.py) to edgewise and
conversion flight. Vectorized over the (r, psi) grid.

Conventions (report Section 1.1)
--------------------------------
Hub / shaft frame H (Venkatesan): +X_H aft (downstream of the in-plane
freestream), +Y_H towards the ADVANCING side, +Z_H along the thrust.
Blade azimuth psi is measured from +X_H IN THE DIRECTION OF ROTATION, so
psi = 90 deg is always the advancing blade. Each rotor uses its own azimuth.

  * rotation='ccw' (right rotor, counter-clockwise seen from above in
    helicopter mode): +Y_H is the aircraft's right.
  * rotation='cw'  (left rotor): the mirror image. Loads are computed in the
    rotor's own frame and then mirrored (Y, Mx and the torque reaction change
    sign) into the common right-handed frame (+Y_H = aircraft right).

Rigid disk: no flapping (beta = 0); hub moments are carried by the hub.

Inflow model (report Section 1.2 / 2.1)
---------------------------------------
    lambda_i(r, psi) = lambda_i0(r) * [1 + K (r/R) cos(psi)]
    K = (4/3 * mu/lambda_G) / (1.2 + mu/lambda_G)            (handout hint)

lambda_G is the total (freestream + induced) inflow ratio from Glauert's
rotor-level momentum equation. lambda_i0(r) is either
  * 'annular_glauert' (default): an annulus-by-annulus Glauert momentum balance
    with the Prandtl tip-loss factor,
        (1/2pi) Int dT_BET(r, psi) dpsi = 4 pi r rho F v0 sqrt(V_x^2 + (V_c + v0)^2)
    which is EXACTLY the M1 annulus equation 4 pi r rho F |V_c + v| v when
    V_x = 0 (hover, axial climb, airplane-mode axial flight), or
  * 'uniform_glauert': lambda_i0 = Glauert mean induced inflow, no tip loss
    (the original M2 model, kept for the Section 3.1 comparison).
"""
from dataclasses import dataclass, field
from typing import Callable, Any, Optional

import numpy as np
from scipy.optimize import brentq

from m2.frames import transform_force_shaft_to_body, transform_moment_about_cg

try:
    from airfoil import LinearAirfoil, TableAirfoil
except ImportError:  # imported from a top-level script without src on the path
    import os
    import sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from airfoil import LinearAirfoil, TableAirfoil


_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")

PG_MACH_LIMIT = 0.7          # Prandtl-Glauert factor frozen above this (as in M1)
STALL_EVAL_U_FRAC = 0.2      # stall margin evaluated where U > 0.2 Omega R


@dataclass
class EdgewiseRotorResult:
    T_N: float
    H_N: float
    Y_N: float
    Q_Nm: float
    power_W: float

    CT: float
    CH: float
    CY: float
    CQ: float
    CP: float

    forces_body_N: np.ndarray     # [Fx, Fy, Fz] on the aircraft, body axes
    moments_body_Nm: np.ndarray   # [Mx, My, Mz] on the aircraft about the CG, body axes

    dT_dr_dpsi: np.ndarray        # (n_r, n_psi) sectional thrust of all B blades [N/m]
    dQ_dr_dpsi: np.ndarray        # (n_r, n_psi) sectional torque [N m / m]

    stalled_fraction: float       # fraction of ALL (r, psi) cells beyond the stall angle
    reverse_flow_fraction: float
    adv_tip_mach: float
    converged: bool

    # ---- additional diagnostics (Milestone 2 report Sections 3-4) ----
    rotation: str = 'ccw'
    inflow_model: str = 'annular_glauert'
    forces_shaft_N: np.ndarray = None     # [H, Y, T] on the aircraft, common hub frame
    moments_shaft_Nm: np.ndarray = None   # [Mx, My, Mz] at the hub on the aircraft, hub frame
    moments_hub_body_Nm: np.ndarray = None  # hub moments in body axes (no r x F term)
    Mx_hub_Nm: float = 0.0                # rolling hub moment, rotor's own frame
    My_hub_Nm: float = 0.0                # pitching hub moment, rotor's own frame
    mu: float = 0.0
    lambda_c: float = 0.0
    lambda_i_G: float = 0.0               # Glauert induced inflow ratio
    lambda_G: float = 0.0                 # Glauert total inflow ratio
    K_inflow: float = 0.0
    r_m: np.ndarray = None                # (n_r,)
    psi_rad: np.ndarray = None            # (n_psi,)
    v0_r_mps: np.ndarray = None           # (n_r,) annulus-mean induced velocity
    U_T: np.ndarray = None                # (n_r, n_psi) signed in-plane velocity [m/s]
    U_P: np.ndarray = None                # (n_r, n_psi) through-disk velocity [m/s]
    alpha_eff_rad: np.ndarray = None      # (n_r, n_psi)
    mach: np.ndarray = None               # (n_r, n_psi)
    reverse_mask: np.ndarray = None
    stall_mask: np.ndarray = None
    alpha_stall_rad: np.ndarray = None    # (n_r,)
    reverse_flow_area_fraction: float = 0.0  # reverse-flow area / blade-swept annulus area
    stalled_fraction_fwd: float = 0.0     # stalled loaded area / loaded forward-flow area
    stall_margin_deg: float = 0.0         # min(alpha_stall - |alpha_eff|) over forward-flow cells
    max_mach: float = 0.0
    n_inflow_iter: int = 0


# ---------------------------------------------------------------------------
# Vectorized airfoil coefficients
# ---------------------------------------------------------------------------
class SectionPolars:
    """Vectorized wrapper around the M1 airfoil objects for a set of radial
    stations. Reproduces LinearAirfoil.get_coeffs / TableAirfoil.get_coeffs
    exactly; falls back to element-wise calls for any other airfoil type."""

    def __init__(self, airfoil_provider: Callable[[float], Any], x: np.ndarray):
        afs = [airfoil_provider(float(xi)) for xi in x]
        self._afs = afs
        if all(isinstance(a, LinearAirfoil) for a in afs):
            self.kind = 'linear'
            col = lambda name: np.array([getattr(a, name) for a in afs], float)[:, None]
            self.a0, self.Cd_min, self.eps = col('a0'), col('Cd_min'), col('eps')
            self.stall, self.clip = col('stall_alpha_rad'), col('Cl_max_clip')
            self.alpha_stall = self.stall[:, 0]
        elif all(isinstance(a, TableAirfoil) for a in afs):
            self.kind = 'table'
            self.alpha_stall = np.array([a.alpha_rad_arr[-1] for a in afs])
        else:
            self.kind = 'generic'
            self.alpha_stall = np.full(len(afs), np.radians(12.0))

    def __call__(self, alpha: np.ndarray):
        if self.kind == 'linear':
            stalled = np.abs(alpha) >= self.stall
            Cl = self.a0 * alpha
            Cd = self.Cd_min + self.eps * alpha ** 2
            Cl = np.where(stalled, np.sign(alpha) * np.minimum(np.abs(Cl), self.clip), Cl)
            Cd = np.where(stalled, np.maximum(Cd, 0.05), Cd)
            return Cl, Cd, stalled
        Cl = np.empty_like(alpha)
        Cd = np.empty_like(alpha)
        st = np.zeros(alpha.shape, bool)
        for i, a in enumerate(self._afs):
            if self.kind == 'table':
                lo, hi = a.alpha_rad_arr[0], a.alpha_rad_arr[-1]
                st[i] = (alpha[i] < lo) | (alpha[i] > hi)
                ac = np.clip(alpha[i], lo, hi)
                Cl[i] = np.interp(ac, a.alpha_rad_arr, a.Cl_arr)
                Cd[i] = np.interp(ac, a.alpha_rad_arr, a.Cd_arr)
            else:
                for j, aa in enumerate(alpha[i]):
                    Cl[i, j], Cd[i, j], st[i, j] = a.get_coeffs(float(aa))
        return Cl, Cd, st


def _prandtl_glauert_cl(Cl, mach):
    """Vectorized airfoil.prandtl_glauert_correct (Cl only, factor frozen at M=0.7)."""
    m = np.minimum(mach, PG_MACH_LIMIT)
    return Cl / np.maximum(np.sqrt(1.0 - m ** 2), 1e-3)


def _tip_loss(B, R, r, phi):
    """Vectorized bemt.prandtl_tip_loss (no root loss)."""
    sin_phi = np.maximum(np.abs(np.sin(phi)), 1e-4)
    f = (B / 2.0) * (R - r) / (r * sin_phi)
    F = (2.0 / np.pi) * np.arccos(np.clip(np.exp(-f), -1.0, 1.0))
    return np.maximum(F, 1e-3)


# ---------------------------------------------------------------------------
# Glauert rotor-level momentum
# ---------------------------------------------------------------------------
def solve_glauert(CT: float, mu: float, lam_c: float) -> float:
    """Mean induced inflow lambda_i from Glauert's momentum equation
        lambda_i = CT / (2 sqrt(mu^2 + (lam_c + lambda_i)^2)).
    Returns 0 for CT <= 0 (no momentum-theory induced inflow)."""
    if CT <= 0:
        return 0.0
    if mu == 0.0 and lam_c == 0.0:
        return np.sqrt(CT / 2.0)

    def residual(lam):
        return lam - CT / (2.0 * np.sqrt(mu ** 2 + (lam_c + lam) ** 2))

    try:
        lam_max = np.sqrt(max(CT, 1e-6) / 2.0) + 0.5
        return float(brentq(residual, 1e-6, lam_max))
    except ValueError:
        lam = np.sqrt(CT / 2.0)
        for _ in range(50):
            lam_new = CT / (2.0 * np.sqrt(mu ** 2 + (lam_c + lam) ** 2))
            if abs(lam_new - lam) < 1e-6:
                break
            lam = 0.5 * lam + 0.5 * lam_new
        return lam


def inflow_K(mu: float, lambda_G: float) -> float:
    """Longitudinal inflow-gradient factor from the handout hint."""
    if abs(lambda_G) > 1e-9:
        m = abs(mu / lambda_G)
        return (4.0 / 3.0) * m / (1.2 + m)
    return 4.0 / 3.0 if abs(mu) > 1e-9 else 0.0


# ---------------------------------------------------------------------------
# Main solver
# ---------------------------------------------------------------------------
def run_edgewise_bemt(
    rotor: Any,
    airfoil_provider: Callable[[float], Any],
    V_inf: float,
    omega_rad_s: float,
    theta0_rad: float,
    theta1c_rad: float,
    theta1s_rad: float,
    alpha_shaft_rad: float,
    nacelle_angle_deg: float,
    rho: float,
    a_sound: float,
    r_hub_body: np.ndarray = np.zeros(3),
    n_r: int = 40,
    n_psi: int = 72,
    max_glauert_iter: int = 20,
    tol: float = 1e-4,
    rotation: str = 'ccw',
    inflow: str = 'annular_glauert',
) -> EdgewiseRotorResult:
    """Azimuth-resolved BEMT with collective/cyclic pitch and nonuniform inflow.

    Blade pitch:  theta(r, psi) = twist(r) + theta0 + theta1c cos(psi) + theta1s sin(psi)
    Velocities :  U_T = Omega r + V_x sin(psi),   V_x = V cos(alpha_shaft)  (in-plane)
                  U_P = V_c + v_i(r, psi),        V_c = V sin(alpha_shaft)  (through disk)
    alpha_shaft > 0 means the freestream passes through the disk in the same
    sense as the induced flow (climb-like); see trim_solver.shaft_angle_of_attack.

    r_hub_body: hub position from the CG in body axes (for the r x F moment).
    """
    if rotation not in ('ccw', 'cw'):
        raise ValueError("rotation must be 'ccw' or 'cw'")
    if inflow not in ('annular_glauert', 'uniform_glauert'):
        raise ValueError("inflow must be 'annular_glauert' or 'uniform_glauert'")
    if omega_rad_s <= 0:
        raise ValueError("omega_rad_s must be positive")

    R = rotor.radius_m
    B = rotor.num_blades
    r = np.linspace(rotor.root_cutout_m, R, n_r)
    x = r / R
    psi = np.linspace(0.0, 2.0 * np.pi, n_psi, endpoint=False)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    rc = r[:, None]

    chord = np.array([rotor.chord_fn(xi) for xi in x])[:, None]
    twist = np.array([rotor.twist_fn(xi) for xi in x])[:, None]
    polars = SectionPolars(airfoil_provider, x)

    OR = omega_rad_s * R
    V_x = V_inf * np.cos(alpha_shaft_rad)          # in-plane
    V_c = V_inf * np.sin(alpha_shaft_rad)          # through the disk
    mu = V_x / OR
    lam_c = V_c / OR

    theta = twist + theta0_rad + theta1c_rad * cpsi + theta1s_rad * spsi
    U_T_signed = omega_rad_s * rc + V_x * spsi
    reverse = U_T_signed < 0.0
    U_T_abs = np.abs(U_T_signed)
    cos_shape = x[:, None] * cpsi                  # (r/R) cos(psi)

    def loads(v0, K, full=False):
        """Sectional loads for annulus-mean induced velocity v0 (n_r,)."""
        U_P = V_c + v0[:, None] * (1.0 + K * cos_shape)
        U2 = U_T_abs ** 2 + U_P ** 2
        phi = np.arctan2(U_P, U_T_abs)
        alpha_eff = np.where(reverse, -(theta + phi), theta - phi)
        Cl, Cd, stalled = polars(alpha_eff)
        mach = np.sqrt(U2) / a_sound
        Cl = _prandtl_glauert_cl(Cl, mach)
        q = 0.5 * rho * U2 * chord
        dL, dD = q * Cl, q * Cd
        cphi, sphi = np.cos(phi), np.sin(phi)
        dT = B * (dL * cphi - dD * sphi)
        if not full:
            return dT
        # in-plane force opposing rotation; in reverse flow it acts WITH rotation
        dFin = B * (dL * sphi + dD * cphi)
        dFin = np.where(reverse, -dFin, dFin)
        return dict(dT=dT, dFin=dFin, U_P=U_P, alpha_eff=alpha_eff, mach=mach, stalled=stalled)

    def annulus_residual(v0, K):
        dT_bet = loads(v0, K).mean(axis=1)
        phi_m = np.arctan2(V_c + v0, omega_rad_s * r)
        F = _tip_loss(B, R, r, phi_m)
        dT_mom = 4.0 * np.pi * r * rho * F * v0 * np.sqrt(V_x ** 2 + (V_c + v0) ** 2)
        return dT_bet - dT_mom

    # geometric scan grid for bracketing (both signs, like M1's scan)
    scan = np.concatenate([[0.0], 0.05 * 1.45 ** np.arange(18)])   # 0 .. ~27 * 0.05*... ~ 28 m/s
    scan = np.concatenate([scan, [40.0, 60.0, 90.0, 150.0]])

    def solve_annuli(K, v_prev=None):
        """Vectorized root of annulus_residual for every radial station.
        Returns (v0, ok_mask)."""
        lo = np.zeros(n_r)
        hi = np.zeros(n_r)
        found = np.zeros(n_r, bool)
        if v_prev is not None:          # warm start: narrow bracket around the last root
            d = 0.15 * np.abs(v_prev) + 0.05
            a, b = v_prev - d, v_prev + d
            fa, fb = annulus_residual(a, K), annulus_residual(b, K)
            ok = np.isfinite(fa) & np.isfinite(fb) & (fa * fb <= 0)
            lo[ok], hi[ok], found[ok] = a[ok], b[ok], True
        if not found.all():
            g0 = annulus_residual(np.zeros(n_r), K)
            sgn = np.where(g0 >= 0, 1.0, -1.0)         # positive loading -> v > 0
            prev_v = np.zeros(n_r)
            prev_g = g0
            exact0 = (g0 == 0) & ~found
            lo[exact0], hi[exact0], found[exact0] = 0.0, 0.0, True
            for s in scan[1:]:
                if found.all():
                    break
                v = sgn * s
                g = annulus_residual(v, K)
                hit = (~found) & np.isfinite(g) & np.isfinite(prev_g) & (prev_g * g <= 0)
                lo[hit] = np.minimum(prev_v[hit], v[hit])
                hi[hit] = np.maximum(prev_v[hit], v[hit])
                found |= hit
                prev_v, prev_g = v, g
        # bisection on all bracketed stations at once
        f_lo = annulus_residual(lo, K)
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            f_mid = annulus_residual(mid, K)
            left = f_lo * f_mid <= 0
            hi = np.where(left, mid, hi)
            lo = np.where(left, lo, mid)
            f_lo = np.where(left, f_lo, f_mid)
            if np.max(hi - lo) < 1e-7:
                break
        v0 = np.where(found, 0.5 * (lo + hi), 0.0)
        return v0, found

    # ---------------- inflow iteration ----------------
    A_disk = np.pi * R ** 2
    norm_T = rho * A_disk * OR ** 2
    converged = False
    stations_ok = np.ones(n_r, bool)
    CT_cur = 0.005
    v0 = None
    n_it = 0
    lam_i_G = 0.0
    K = 0.0
    for n_it in range(1, max_glauert_iter + 1):
        lam_i_G = solve_glauert(CT_cur, mu, lam_c)
        K = inflow_K(mu, lam_c + lam_i_G)
        if inflow == 'uniform_glauert':
            v0 = np.full(n_r, lam_i_G * OR)
            dT = loads(v0, K)
        else:
            v0, stations_ok = solve_annuli(K, v0)
            dT = loads(v0, K)
        CT_new = float(np.mean(_trapz(dT, x=r, axis=0))) / norm_T
        if inflow == 'annular_glauert' and abs(mu) < 1e-12:
            CT_cur = CT_new            # K = 0: the annulus solution does not depend on CT
            converged = True
            break
        if abs(CT_new - CT_cur) < tol:
            CT_cur = CT_new
            converged = True
            break
        # Annular: CT only enters through K (a first-harmonic shape factor), so
        # the plain fixed point is stable. Uniform: damped, as in the original.
        CT_cur = CT_new if inflow == 'annular_glauert' else 0.5 * CT_cur + 0.5 * CT_new
    if inflow == 'annular_glauert' and v0 is not None and stations_ok is not None:
        converged = converged and bool(stations_ok.all())

    # ---------------- final loads and integration ----------------
    L = loads(v0, K, full=True)
    dT_mat = L['dT']
    dQ_mat = L['dFin'] * rc
    dH_mat = L['dFin'] * spsi           # +X_H aft
    dY_mat = -L['dFin'] * cpsi          # +Y_H advancing side
    dMx_mat = dT_mat * rc * spsi        # M = r x F, r = r(cos psi, sin psi, 0)
    dMy_mat = -dT_mat * rc * cpsi

    integ = lambda m: float(np.mean(_trapz(m, x=r, axis=0)))
    T_N, Q_Nm, H_N, Y_N = integ(dT_mat), integ(dQ_mat), integ(dH_mat), integ(dY_mat)
    Mx_Nm, My_Nm = integ(dMx_mat), integ(dMy_mat)

    power_W = Q_Nm * omega_rad_s
    CT = T_N / norm_T
    CQ = Q_Nm / (norm_T * R)
    CH = H_N / norm_T
    CY = Y_N / norm_T
    CP = power_W / (norm_T * OR)

    # Loads ON THE AIRCRAFT in the common hub frame (+Y_H = aircraft right).
    # Aerodynamic torque on the blades is -Q about +Z of the rotor's own frame;
    # the shaft passes it to the airframe.
    if rotation == 'ccw':
        F_shaft = np.array([H_N, Y_N, T_N])
        M_shaft = np.array([Mx_Nm, My_Nm, -Q_Nm])
    else:   # mirror image about the X_H-Z_H plane
        F_shaft = np.array([H_N, -Y_N, T_N])
        M_shaft = np.array([-Mx_Nm, My_Nm, Q_Nm])

    F_body = transform_force_shaft_to_body(F_shaft, nacelle_angle_deg)
    M_hub_body = transform_force_shaft_to_body(M_shaft, nacelle_angle_deg)
    M_body_cg = transform_moment_about_cg(F_body, M_hub_body, np.asarray(r_hub_body, float))

    # Helical advancing-tip Mach (in-plane + through-disk components)
    lam_G = lam_c + lam_i_G
    ut_tip = OR + abs(V_x)             # V_x < 0 moves the advancing side to psi = 270
    up_tip = V_c + lam_i_G * OR
    adv_tip_mach = float(np.sqrt(ut_tip ** 2 + up_tip ** 2) / a_sound)

    # Stall margin / fraction are evaluated over the LOADED forward-flow region:
    # cells inside or just outside the reverse-flow circle have U -> 0 and
    # arbitrary angle of attack but carry no load (q < 4 % of tip q).
    fwd = (~reverse) & ((U_T_abs ** 2 + L['U_P'] ** 2) > (STALL_EVAL_U_FRAC * OR) ** 2)
    stalled = L['stalled']
    a_st = polars.alpha_stall[:, None]
    margin = np.where(fwd, a_st - np.abs(L['alpha_eff']), np.inf)
    n_fwd = max(int(fwd.sum()), 1)

    return EdgewiseRotorResult(
        T_N=T_N, H_N=H_N, Y_N=Y_N, Q_Nm=Q_Nm, power_W=power_W,
        CT=CT, CH=CH, CY=CY, CQ=CQ, CP=CP,
        forces_body_N=F_body, moments_body_Nm=M_body_cg,
        dT_dr_dpsi=dT_mat, dQ_dr_dpsi=dQ_mat,
        stalled_fraction=float(stalled.mean()),
        reverse_flow_fraction=float(reverse.mean()),
        adv_tip_mach=adv_tip_mach,
        converged=converged,
        rotation=rotation, inflow_model=inflow,
        forces_shaft_N=F_shaft, moments_shaft_Nm=M_shaft, moments_hub_body_Nm=M_hub_body,
        Mx_hub_Nm=Mx_Nm, My_hub_Nm=My_Nm,
        mu=mu, lambda_c=lam_c, lambda_i_G=lam_i_G, lambda_G=lam_G, K_inflow=K,
        r_m=r, psi_rad=psi, v0_r_mps=v0,
        U_T=U_T_signed, U_P=L['U_P'], alpha_eff_rad=L['alpha_eff'], mach=L['mach'],
        reverse_mask=reverse, stall_mask=stalled, alpha_stall_rad=polars.alpha_stall,
        reverse_flow_area_fraction=float((reverse * rc).sum() / (rc.sum() * n_psi)),
        stalled_fraction_fwd=float(((stalled & fwd) * rc).sum() / max(float((fwd * rc).sum()), 1e-12)),
        stall_margin_deg=float(np.degrees(np.min(margin))),
        max_mach=float(np.max(L['mach'])),
        n_inflow_iter=n_it,
    )
