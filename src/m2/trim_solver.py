import numpy as np
from scipy.optimize import root
from typing import Callable, Optional

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.aero_models import compute_wing_aero, compute_htail_aero
from m2.edgewise_bemt import run_edgewise_bemt

def shaft_angle_of_attack(alpha_body_rad: float, nacelle_deg: float) -> float:
    """Rotor-disk angle of attack used by run_edgewise_bemt, defined so that
        mu      = V cos(alpha_shaft) / (Omega R)   (in-plane, towards +X_hub = aft)
        lam_c   = V sin(alpha_shaft) / (Omega R)   (through the disk, same sense
                                                    as the induced velocity)
    The air velocity relative to the aircraft in body axes is
    V*(-cos(alpha), 0, -sin(alpha)); projecting it on the hub axes from
    frames.R_shaft_to_body gives  in-plane = V sin(i_n + alpha),
    through-disk = V cos(i_n + alpha), hence

        alpha_shaft = 90 deg - i_n - alpha_body.

    Check: helicopter mode (i_n = 90) with the nose UP (alpha > 0) puts the
    freestream UP through the disk (lam_c < 0), nose DOWN gives lam_c > 0.
    """
    return np.radians(90.0 - nacelle_deg) - alpha_body_rad


def compute_aircraft_residual(
    x: np.ndarray, 
    V_inf: float, 
    gamma_rad: float, 
    nacelle_deg: float, 
    omega_rad_s: float,
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    trim_pitch_with: str = 'elevator' # 'elevator' or 'cyclic'
) -> np.ndarray:
    """
    Computes the 3-DOF longitudinal residual forces/moments for a given state vector x.
    x = [alpha_body_rad, collective_rad, pitch_control_rad]
    """
    alpha_body_rad = x[0]
    collective_rad = x[1]
    pitch_ctrl = x[2]
    
    delta_e_rad = pitch_ctrl if trim_pitch_with == 'elevator' else 0.0
    # Rigid-disk model (no flapping): the airload responds IN PHASE with the
    # blade pitch, so a fore/aft loading difference -- i.e. a hub PITCHING
    # moment -- is produced by the cos(psi) cyclic component (psi = 0 aft).
    # The sin(psi) component produces a ROLLING moment here and has almost no
    # pitch authority, so it cannot be used to trim pitch in this model.
    # (On a real flapping rotor the ~90 deg phase lag moves this job to theta_1s.)
    theta1c_rad = pitch_ctrl if trim_pitch_with == 'cyclic' else 0.0
    
    # 1. Aerodynamics
    wing_aero = compute_wing_aero(aircraft.wing, V_inf, alpha_body_rad, rho)
    htail_aero = compute_htail_aero(aircraft.htail, V_inf, alpha_body_rad, rho, delta_e_rad=delta_e_rad)
    
    # Fuselage drag
    q_dyn = 0.5 * rho * V_inf**2
    D_fuse = q_dyn * aircraft.flat_plate_area_m2
    Fx_fuse_body = -D_fuse * np.cos(alpha_body_rad)
    Fz_fuse_body = -D_fuse * np.sin(alpha_body_rad)
    
    # 2. Rotors (2 rotors)
    # Nacelle angle 90 = Helicopter (shaft points UP, Z_body = -Z_shaft)
    # alpha_shaft is angle between rotor disk and freestream
    alpha_shaft_rad = shaft_angle_of_attack(alpha_body_rad, nacelle_deg)
    
    rotor_result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=V_inf,
        omega_rad_s=omega_rad_s,
        theta0_rad=collective_rad,
        theta1c_rad=theta1c_rad,
        theta1s_rad=0.0,
        alpha_shaft_rad=alpha_shaft_rad,
        nacelle_angle_deg=nacelle_deg,
        rho=rho,
        a_sound=a_sound,
        n_r=25, # lower resolution for trim speed
        n_psi=36
    )
    
    # Multiply by 2 for twin rotors
    Fx_rotors = 2.0 * rotor_result.forces_body_N[0]
    Fz_rotors = 2.0 * rotor_result.forces_body_N[2]
    
    # Pitching moment from rotors
    My_rotors = 2.0 * rotor_result.moments_body_Nm[1]
    
    # Add moment due to rotor CG offsets
    # M = r x F  ->  My = r_z * F_x - r_x * F_z   (body: x fwd, y right, z down).
    # Symmetric rotors: the y offsets cancel for the pitching moment.
    def _My_offset(r, Fx, Fz):
        return r[2] * Fx - r[0] * Fz

    My_rotors += 2.0 * _My_offset(aircraft.cg_to_right_rotor_m,
                                  rotor_result.forces_body_N[0],
                                  rotor_result.forces_body_N[2])

    # Wing and Tail pitching moments
    My_wing = wing_aero.M_Nm + _My_offset(aircraft.cg_to_wing_ac_m, wing_aero.Fx_body_N, wing_aero.Fz_body_N)
    My_htail = htail_aero.M_Nm + _My_offset(aircraft.cg_to_htail_ac_m, htail_aero.Fx_body_N, htail_aero.Fz_body_N)

    # Total Body Forces
    Fx_total = Fx_rotors + wing_aero.Fx_body_N + htail_aero.Fx_body_N + Fx_fuse_body
    Fz_total = Fz_rotors + wing_aero.Fz_body_N + htail_aero.Fz_body_N + Fz_fuse_body
    My_total = My_rotors + My_wing + My_htail
    
    # Gravity in Body Frame
    # Pitch attitude theta = alpha_body + gamma
    theta_rad = alpha_body_rad + gamma_rad
    W = aircraft.W_MTOW_N
    Gx_body = -W * np.sin(theta_rad)
    Gz_body = W * np.cos(theta_rad)
    
    # Residuals (Sum of forces = 0, Sum of moments = 0)
    res_X = Fx_total + Gx_body
    res_Z = Fz_total + Gz_body
    res_M = My_total
    
    # Normalize residuals for better solver behavior
    return np.array([res_X / W, res_Z / W, res_M / (W * 1.0)])

# Bounds on the trim unknowns [deg]. A converged root outside these is NOT a
# usable trim (e.g. an "elevator" of -150 deg satisfies the equations
# numerically but is meaningless), so it is reported as control saturation.
DEFAULT_TRIM_BOUNDS_DEG = {
    'alpha':      (-30.0, 30.0),   # body angle of attack
    'collective': (-5.0, 60.0),    # added to built-in twist (airplane mode needs ~30-45)
    'cyclic':     (-15.0, 15.0),
    'elevator':   (-25.0, 25.0),
}


def _initial_guesses(V_inf, nacelle_deg, omega_rad_s, aircraft, rotor, rho, x0):
    """User seed first, then physically motivated starts. The residual is
    strongly nonlinear (stall clipping, windmilling rotor at low collective in
    axial flow), so one seed is often not enough for hybr."""
    guesses = [np.asarray(x0, dtype=float)]
    n = np.radians(nacelle_deg)
    # collective that puts the 0.75R section a few degrees above its inflow angle
    R = rotor.radius_m
    v_h = np.sqrt(0.5 * aircraft.W_MTOW_N / (2.0 * rho * np.pi * R**2))
    for alpha in (np.radians(4.0), 0.0, np.radians(10.0), np.radians(-5.0)):
        V_through = V_inf * np.cos(n + alpha)
        phi75 = np.arctan2(max(V_through, 0.0) + v_h * np.sin(n) ** 2, omega_rad_s * 0.75 * R)
        coll = phi75 - rotor.twist_fn(0.75) + np.radians(6.0)
        guesses.append(np.array([alpha, coll, 0.0]))
    return guesses


def trim_aircraft_detailed(
    V_inf: float,
    gamma_rad: float,
    nacelle_deg: float,
    omega_rad_s: float,
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    x0: np.ndarray,
    trim_pitch_with: str = 'elevator',
    bounds_deg: Optional[dict] = None,
    residual_tol: float = 1e-4,
    verbose: bool = True,
):
    """Solve the 3-DOF trim. Returns (x, status, residual_norm) where status is
    'ok', 'control_saturation' (root found but outside bounds_deg) or
    'no_convergence'. x is the best point found in every case (may be None)."""
    bounds = dict(DEFAULT_TRIM_BOUNDS_DEG)
    if bounds_deg:
        bounds.update(bounds_deg)
    lims = [bounds['alpha'], bounds['collective'], bounds[trim_pitch_with]]

    def cost(x):
        return compute_aircraft_residual(
            x, V_inf, gamma_rad, nacelle_deg, omega_rad_s,
            aircraft, rotor, airfoil_provider, rho, a_sound, trim_pitch_with
        )

    def in_bounds(x):
        return all(lo - 1e-6 <= np.degrees(xi) <= hi + 1e-6 for xi, (lo, hi) in zip(x, lims))

    best = (None, 'no_convergence', np.inf)
    saturated = None
    for guess in _initial_guesses(V_inf, nacelle_deg, omega_rad_s, aircraft, rotor, rho, x0):
        try:
            res = root(cost, guess, method='hybr')
        except Exception:
            continue
        norm = float(np.linalg.norm(res.fun))
        if not np.isfinite(norm):
            continue
        if norm < residual_tol:
            if in_bounds(res.x):
                return res.x, 'ok', norm
            if saturated is None:
                saturated = (res.x, 'control_saturation', norm)
        elif norm < best[2]:
            best = (res.x, 'no_convergence', norm)

    out = saturated if saturated is not None else best
    if verbose:
        if out[1] == 'control_saturation':
            x = np.degrees(out[0])
            print(f"[trim_solver] WARNING: trim at V={V_inf:.1f} m/s, nacelle={nacelle_deg:.0f} deg needs "
                  f"alpha={x[0]:.1f}, collective={x[1]:.1f}, {trim_pitch_with}={x[2]:.1f} deg "
                  f"-- outside control bounds.")
        else:
            print(f"[trim_solver] WARNING: Trim did not converge at V={V_inf:.1f} m/s, "
                  f"nacelle={nacelle_deg:.0f} deg. Best residual norm: {out[2]:.4f}.")
    return out


def trim_aircraft(
    V_inf: float,
    gamma_rad: float,
    nacelle_deg: float,
    omega_rad_s: float,
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    x0: np.ndarray,
    trim_pitch_with: str = 'elevator',
    bounds_deg: Optional[dict] = None,
) -> Optional[np.ndarray]:
    """Returns the trim vector [alpha, collective, pitch control] in rad, or
    None if no in-bounds converged trim exists (see trim_aircraft_detailed for
    the reason)."""
    x, status, _ = trim_aircraft_detailed(
        V_inf, gamma_rad, nacelle_deg, omega_rad_s, aircraft, rotor,
        airfoil_provider, rho, a_sound, x0, trim_pitch_with, bounds_deg)
    return x if status == 'ok' else None
