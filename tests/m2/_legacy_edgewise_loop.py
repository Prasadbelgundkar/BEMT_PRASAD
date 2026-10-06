import numpy as np
from dataclasses import dataclass
from typing import Callable, Tuple, Any
from scipy.optimize import brentq

# Adjust imports to allow this to be used within the package
from m2.frames import transform_force_shaft_to_body, transform_moment_about_cg

try:
    from bemt import prandtl_tip_loss
    from airfoil import prandtl_glauert_correct
except ImportError:
    # Fallbacks if imported from top-level script
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from bemt import prandtl_tip_loss
    from airfoil import prandtl_glauert_correct


_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")

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
    
    forces_body_N: np.ndarray  # [Fx, Fy, Fz] in aircraft body frame
    moments_body_Nm: np.ndarray # [Mx, My, Mz] about CG
    
    dT_dr_dpsi: np.ndarray  # (n_r, n_psi)
    dQ_dr_dpsi: np.ndarray  # (n_r, n_psi)
    
    stalled_fraction: float
    reverse_flow_fraction: float
    adv_tip_mach: float
    converged: bool


def solve_glauert(CT: float, mu: float, lam_c: float) -> float:
    """
    Solves the Glauert momentum equation for mean induced velocity lambda_i:
    lambda_i = CT / (2 * sqrt(mu^2 + (lam_c + lambda_i)^2))
    """
    if CT <= 0:
        return 0.0
        
    if mu == 0.0 and lam_c == 0.0:
        return np.sqrt(CT / 2.0)
        
    def residual(lam):
        return lam - CT / (2.0 * np.sqrt(mu**2 + (lam_c + lam)**2))
        
    try:
        # Avoid starting at exactly 0.0 if mu is very small
        lam_max = np.sqrt(max(CT, 1e-6) / 2.0) + 0.5
        return float(brentq(residual, 1e-6, lam_max))
    except ValueError:
        # Fallback to relaxation iteration if brentq fails to bracket
        lam = np.sqrt(CT / 2.0)
        for _ in range(50):
            lam_new = CT / (2.0 * np.sqrt(mu**2 + (lam_c + lam)**2))
            if abs(lam_new - lam) < 1e-6:
                break
            lam = 0.5 * lam + 0.5 * lam_new
        return lam


def run_edgewise_bemt(
    rotor: Any,  # Rotor dataclass
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
    tol: float = 1e-4
) -> EdgewiseRotorResult:
    """
    Runs an azimuth-resolved BEMT analysis with cyclic pitch and non-uniform inflow.
    """
    R = rotor.radius_m
    B = rotor.num_blades
    r_array = np.linspace(rotor.root_cutout_m, R, n_r)
    psi_array = np.linspace(0, 2 * np.pi, n_psi, endpoint=False)
    
    dr = r_array[1] - r_array[0]
    dpsi = psi_array[1] - psi_array[0]
    
    # Pre-evaluate rotor geometry
    chord_array = np.array([rotor.chord_fn(r / R) for r in r_array])
    twist_array = np.array([rotor.twist_fn(r / R) for r in r_array])
    
    mu = (V_inf * np.cos(alpha_shaft_rad)) / (omega_rad_s * R) if omega_rad_s > 0 else 0.0
    lam_c = (V_inf * np.sin(alpha_shaft_rad)) / (omega_rad_s * R) if omega_rad_s > 0 else 0.0
    
    # Initialize CT for Glauert iteration
    # Estimate CT from uniform momentum if hover, or just start at 0.01
    CT_current = 0.005
    lambda_G = 0.0
    
    converged = False
    
    # Storage arrays
    dT_mat = np.zeros((n_r, n_psi))
    dQ_mat = np.zeros((n_r, n_psi))
    dH_mat = np.zeros((n_r, n_psi))
    dY_mat = np.zeros((n_r, n_psi))
    
    stalled_count = 0
    reverse_count = 0
    total_elements = n_r * n_psi
    
    # Iterative loop for Glauert inflow
    for iter_idx in range(max_glauert_iter):
        lambda_G = solve_glauert(CT_current, mu, lam_c)
        
        T_accum = 0.0
        H_accum = 0.0
        Y_accum = 0.0
        Q_accum = 0.0
        stalled_count = 0
        reverse_count = 0
        
        for i, r in enumerate(r_array):
            c = chord_array[i]
            theta_tw = twist_array[i]
            airfoil = airfoil_provider(r / R)
            
            for j, psi in enumerate(psi_array):
                # 1. Cyclic pitch
                theta = theta_tw + theta0_rad + theta1c_rad * np.cos(psi) + theta1s_rad * np.sin(psi)
                
                # 2. Local velocities (Venkatesan 3.46-3.48)
                U_T = omega_rad_s * r + mu * omega_rad_s * R * np.sin(psi)
                
                # Non-uniform inflow factor K
                # K uses mu / lambda_total, where lambda_total is the total
                # Glauert inflow (freestream component + induced).
                lam_total = lam_c + lambda_G
                if abs(lam_total) > 1e-9:
                    mu_over_lam = abs(mu / lam_total)
                    K = ( (4.0/3.0) * mu_over_lam ) / (1.2 + mu_over_lam)
                elif abs(mu) > 1e-9:
                    K = 4.0 / 3.0   # limit of K as mu/lambda -> infinity
                else:
                    K = 0.0
                    
                lambda_i_local = lambda_G * (1.0 + K * (r / R) * np.cos(psi))
                # Venkatesan (3.48): U_P = lambda Omega R + r beta_dot + mu Omega R beta cos(psi).
                # Rigid disk (beta = 0): U_P = lambda Omega R, with
                # lambda = V sin(alpha_shaft) / (Omega R) + lambda_i.
                U_P = V_inf * np.sin(alpha_shaft_rad) + lambda_i_local * omega_rad_s * R
                
                # Reverse flow handling (U_T < 0: air reaches the trailing edge first)
                is_reverse = U_T < 0
                if is_reverse:
                    reverse_count += 1
                    U_T = abs(U_T)

                U_mag = np.sqrt(U_T**2 + U_P**2)
                phi = np.arctan2(U_P, U_T)

                if is_reverse:
                    # Seen from the reversed onset flow the section is pitched
                    # "nose-down" by theta, and downward inflow also strikes the
                    # upper surface, so the angle of attack that produces UPWARD
                    # lift is -(theta + phi).
                    alpha_eff = -(theta + phi)
                else:
                    alpha_eff = theta - phi

                # 3. Aerodynamics
                Cl, Cd, stalled = airfoil.get_coeffs(alpha_eff)
                if stalled:
                    stalled_count += 1

                mach = U_mag / a_sound
                # Cl only; the factor is frozen at M=0.7 inside the function, so no
                # outer guard (a guard here switches the correction OFF above M=0.7
                # and makes Cl drop discontinuously by ~29%).
                Cl, _ = prandtl_glauert_correct(Cl, Cd, mach)

                # Prandtl tip/hub loss is not applied in M2 (prescribed Glauert
                # inflow, no local momentum balance to put it in).

                # 4. Sectional Forces (dL positive up, dD along the relative wind)
                q_dyn = 0.5 * rho * U_mag**2 * c
                dL = q_dyn * Cl
                dD = q_dyn * Cd

                dFz = B * (dL * np.cos(phi) - dD * np.sin(phi))
                # In-plane force OPPOSING rotation. In reverse flow the in-plane
                # onset flow runs TE -> LE, so both the drag and the tilt of the
                # lift vector act WITH the rotation: the sign flips.
                dF_inplane = B * (dL * np.sin(phi) + dD * np.cos(phi))
                if is_reverse:
                    dF_inplane = -dF_inplane

                # H (longitudinal drag) and Y (lateral force)
                # Venkatesan 3.68, 3.69
                # dFx1 = dF_inplane * sin(psi)
                # dFy1 = -dF_inplane * cos(psi)
                # Assuming dFx1 and dFy1 map to Hub frame H and Y
                dFx_hub = dF_inplane * np.sin(psi)
                dFy_hub = -dF_inplane * np.cos(psi)
                
                # Record
                dT_mat[i, j] = dFz
                dQ_mat[i, j] = dF_inplane * r
                dH_mat[i, j] = dFx_hub
                dY_mat[i, j] = dFy_hub
        
        # Integrate over r (trapezoidal) and average over psi
        T_az = _trapz(dT_mat, x=r_array, axis=0)
        Q_az = _trapz(dQ_mat, x=r_array, axis=0)
        H_az = _trapz(dH_mat, x=r_array, axis=0)
        Y_az = _trapz(dY_mat, x=r_array, axis=0)
        
        T_N = np.mean(T_az)
        Q_Nm = np.mean(Q_az)
        H_N = np.mean(H_az)
        Y_N = np.mean(Y_az)
        
        # Check convergence
        CT_new = T_N / (rho * (np.pi * R**2) * (omega_rad_s * R)**2)
        # Debug print removed for speed
        if abs(CT_new - CT_current) < tol:
            converged = True
            CT_current = CT_new
            break
            
        # Damped update for stability in transition zone
        CT_current = 0.5 * CT_current + 0.5 * CT_new

    # Final calculations
    # Report the CT that belongs to the returned loads (CT_current may still be
    # the damped iterate if the inflow loop stopped on max_glauert_iter).
    CT_current = T_N / (rho * (np.pi * R**2) * (omega_rad_s * R)**2)
    power_W = Q_Nm * omega_rad_s
    
    rho_A_vtip2 = rho * (np.pi * R**2) * (omega_rad_s * R)**2
    CQ = Q_Nm / (rho_A_vtip2 * R)
    CH = H_N / rho_A_vtip2
    CY = Y_N / rho_A_vtip2
    CP = power_W / (rho_A_vtip2 * omega_rad_s * R)
    
    # Hub moments (aerodynamic only, rigid blade), integrated from the thrust:
    # M = r x F with r = r*(cos psi, sin psi, 0), F = (0, 0, dT):
    # Mx = + T * r * sin(psi)
    # My = - T * r * cos(psi)
    dMx_mat = dT_mat * r_array[:, None] * np.sin(psi_array)
    dMy_mat = -dT_mat * r_array[:, None] * np.cos(psi_array)
    
    Mx_az = _trapz(dMx_mat, x=r_array, axis=0)
    My_az = _trapz(dMy_mat, x=r_array, axis=0)
    
    Mx_Nm = np.mean(Mx_az)
    My_Nm = np.mean(My_az)
    
    # 6-DOF Hub forces in shaft frame
    # Shaft frame (Venkatesan Hub): +X aft, +Y right, +Z up
    # T_N is up (+Z)
    # H_N is aft (+X)
    # Y_N is right (+Y)
    # Torque Q is around Z. If CCW, aerodynamic torque opposes, so it's acting on rotor. 
    # Moment acting ON AIRCRAFT from rotor is Q in +Z direction.
    F_shaft = np.array([H_N, Y_N, T_N])
    M_shaft = np.array([Mx_Nm, My_Nm, Q_Nm])
    
    # Transform to body frame
    F_body = transform_force_shaft_to_body(F_shaft, nacelle_angle_deg)
    # We must also transform moments! Rotation matrix applies to moments too.
    M_body_hub = transform_force_shaft_to_body(M_shaft, nacelle_angle_deg)
    
    # Moment about CG
    M_body_cg = transform_moment_about_cg(F_body, M_body_hub, r_hub_body)
    
    # Tip mach on advancing side (psi=90)
    # Helical: in-plane (Omega R + edgewise V) and through-disk (axial V + induced)
    # components. Without the through-disk part this under-reads badly in
    # airplane mode, where the whole flight speed is axial.
    _ut_tip = omega_rad_s * R + V_inf * np.cos(alpha_shaft_rad)
    _up_tip = V_inf * np.sin(alpha_shaft_rad) + lambda_G * omega_rad_s * R
    adv_tip_mach = np.sqrt(_ut_tip**2 + _up_tip**2) / a_sound
    
    return EdgewiseRotorResult(
        T_N=T_N, H_N=H_N, Y_N=Y_N, Q_Nm=Q_Nm, power_W=power_W,
        CT=CT_current, CH=CH, CY=CY, CQ=CQ, CP=CP,
        forces_body_N=F_body,
        moments_body_Nm=M_body_cg,
        dT_dr_dpsi=dT_mat, dQ_dr_dpsi=dQ_mat,
        stalled_fraction=stalled_count / total_elements,
        reverse_flow_fraction=reverse_count / total_elements,
        adv_tip_mach=adv_tip_mach,
        converged=converged
    )
