import numpy as np
from dataclasses import dataclass
from m2.aircraft_input_m2 import WingGeometry, TailGeometry

@dataclass
class AeroForces:
    L_N: float
    D_N: float
    M_Nm: float  # Pitching moment about AC
    
    # Body frame forces (X is fwd, Z is down)
    Fx_body_N: float
    Fz_body_N: float

def compute_wing_aero(wing: WingGeometry, V_inf: float, alpha_body_rad: float, rho: float) -> AeroForces:
    """
    Computes wing aerodynamic forces using simple lifting line theory approximations.
    """
    if V_inf < 1.0:
        return AeroForces(0.0, 0.0, 0.0, 0.0, 0.0)
        
    alpha_wing_rad = alpha_body_rad + np.radians(wing.i_w_deg)
    
    # CL slope ~ 2*pi for 2D, reduced by finite span effect
    a0 = 2.0 * np.pi
    a = a0 / (1.0 + a0 / (np.pi * wing.e_oswald * wing.AR))
    
    # Stall model (simple linear up to stall)
    alpha_stall = np.radians(15.0)
    if alpha_wing_rad > alpha_stall:
        CL = a * alpha_stall * (alpha_stall / alpha_wing_rad) # drops off post stall
    elif alpha_wing_rad < -alpha_stall:
        CL = -a * alpha_stall * (alpha_stall / abs(alpha_wing_rad))
    else:
        CL = a * alpha_wing_rad
        
    # Drag
    CD0 = 0.015
    CD_i = (CL**2) / (np.pi * wing.e_oswald * wing.AR)
    CD = CD0 + CD_i
    
    # Moment
    CM_ac = -0.1
    
    q = 0.5 * rho * V_inf**2
    L = q * wing.S_m2 * CL
    D = q * wing.S_m2 * CD
    M = q * wing.S_m2 * wing.chord_m * CM_ac
    
    # Convert Lift (wind Z) and Drag (wind X) to Body axes
    # Wind X is at alpha to Body X.
    # L is perpendicular to V, D is parallel to V.
    # Fx = -D * cos(alpha) + L * sin(alpha)
    # Fz = -D * sin(alpha) - L * cos(alpha)
    Fx = -D * np.cos(alpha_body_rad) + L * np.sin(alpha_body_rad)
    Fz = -D * np.sin(alpha_body_rad) - L * np.cos(alpha_body_rad)
    
    return AeroForces(L, D, M, Fx, Fz)

def compute_htail_aero(tail: TailGeometry, V_inf: float, alpha_body_rad: float, rho: float, 
                       downwash_angle_rad: float = 0.0, delta_e_rad: float = 0.0) -> AeroForces:
    """
    Computes horizontal tail aerodynamic forces.
    """
    if V_inf < 1.0:
        return AeroForces(0.0, 0.0, 0.0, 0.0, 0.0)
        
    alpha_tail_rad = alpha_body_rad + np.radians(tail.i_t_deg) - downwash_angle_rad
    
    a0 = 2.0 * np.pi
    a = a0 / (1.0 + a0 / (np.pi * tail.e_oswald * tail.AR))
    
    # Elevator effectiveness (tau) approx 0.4
    tau_e = 0.4
    
    alpha_stall = np.radians(12.0)
    alpha_eff = alpha_tail_rad + tau_e * delta_e_rad
    
    if abs(alpha_eff) > alpha_stall:
        CL = np.sign(alpha_eff) * a * alpha_stall * (alpha_stall / abs(alpha_eff))
    else:
        CL = a * alpha_eff
        
    CD0 = 0.015
    CD = CD0 + (CL**2) / (np.pi * tail.e_oswald * tail.AR)
    
    q = 0.5 * rho * V_inf**2
    L = q * tail.S_m2 * CL
    D = q * tail.S_m2 * CD
    M = 0.0
    
    Fx = -D * np.cos(alpha_body_rad) + L * np.sin(alpha_body_rad)
    Fz = -D * np.sin(alpha_body_rad) - L * np.cos(alpha_body_rad)
    
    return AeroForces(L, D, M, Fx, Fz)


# ===========================================================================
# 6-DOF airframe model (wing + flaperons, H-tail + elevator, V-tail + rudder,
# fuselage/nacelle drag). Used by trim_6dof.py.
# ===========================================================================
@dataclass
class AirframeLoads:
    F_body_N: np.ndarray          # total airframe force, body axes
    M_cg_Nm: np.ndarray           # total airframe moment about the CG, body axes
    alpha_wing_deg: float
    CL_wing: float
    L_wing_N: float
    D_wing_N: float
    alpha_htail_deg: float
    CL_htail: float
    L_htail_N: float
    downwash_deg: float
    D_fuselage_N: float
    wing_stalled: bool
    htail_stalled: bool
    q_Pa: float
    alpha_wing_stall_deg: float = 15.0


def _lift_curve(alpha, CL_alpha, alpha_stall):
    """Linear up to stall, then CL decays as alpha_stall/alpha (as the legacy model)."""
    if abs(alpha) <= alpha_stall:
        return CL_alpha * alpha, False
    return np.sign(alpha) * CL_alpha * alpha_stall * (alpha_stall / abs(alpha)), True


def _post_stall_drag(alpha, alpha_stall, CD90):
    s2, s2s = np.sin(alpha) ** 2, np.sin(alpha_stall) ** 2
    return CD90 * max(0.0, s2 - s2s) / (1.0 - s2s)


def airframe_loads(ac, V_inf: float, alpha_rad: float, rho: float, nacelle_deg: float,
                   elevator_deg: float = 0.0, aileron_deg: float = 0.0, rudder_deg: float = 0.0,
                   beta_rad: float = 0.0, **mass_kw) -> AirframeLoads:
    """Aerodynamic loads of the non-rotor airframe in body axes, moments about
    the CG (which depends on nacelle angle and fuel through **mass_kw).

    Assumptions (report Section 1.3): freestream velocity and angle of attack
    seen by every surface; NO rotor-wake/wing or rotor-wake/tail interference
    and no hover download; tail downwash from the wing only,
    eps = 2 CL_w / (pi AR_w); fuselage + nacelle drag as an equivalent flat
    plate acting at the CG (no fuselage pitching moment).
    Sign conventions: +elevator = trailing edge down (nose-down moment),
    +aileron = right flaperon down / left up (roll-left), +rudder = trailing
    edge left (nose-left)."""
    q = 0.5 * rho * V_inf ** 2
    zero = np.zeros(3)
    if V_inf < 1.0:
        return AirframeLoads(zero, zero.copy(), np.degrees(alpha_rad + np.radians(ac.wing.i_w_deg)),
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False, q)
    w, t, vt = ac.wing, ac.htail, ac.vtail
    ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
    # unit vectors (body axes) of drag (along the relative wind) and lift (normal, "up")
    e_D = -np.array([ca * np.cos(beta_rad), np.sin(beta_rad), sa * np.cos(beta_rad)])
    e_L = np.array([sa, 0.0, -ca])

    # ---- wing ----
    a_w = alpha_rad + np.radians(w.i_w_deg)
    a_st_w = np.radians(w.alpha_stall_deg)
    CL_w, st_w = _lift_curve(a_w, w.CL_alpha, a_st_w)
    CD_w = w.CD0 + CL_w ** 2 / (np.pi * w.e_oswald * w.AR) + _post_stall_drag(a_w, a_st_w, w.CD90)
    L_w, D_w = q * w.S_m2 * CL_w, q * w.S_m2 * CD_w
    F_w = L_w * e_L + D_w * e_D
    M_w = np.array([-q * w.S_m2 * w.span_m * w.Cl_delta_a * np.radians(aileron_deg),
                    q * w.S_m2 * w.chord_m * w.CM_ac, 0.0])
    M_w += np.cross(ac.wing_ac_from_cg(nacelle_deg, **mass_kw), F_w)

    # ---- horizontal tail ----
    eps = 2.0 * CL_w / (np.pi * w.AR)
    a_t = alpha_rad + np.radians(t.i_t_deg) - eps + t.tau_e * np.radians(elevator_deg)
    CLa_t = t.a0_per_rad / (1.0 + t.a0_per_rad / (np.pi * t.e_oswald * t.AR))
    a_st_t = np.radians(t.alpha_stall_deg)
    CL_t, st_t = _lift_curve(a_t, CLa_t, a_st_t)
    CD_t = t.CD0 + CL_t ** 2 / (np.pi * t.e_oswald * t.AR) + _post_stall_drag(a_t, a_st_t, w.CD90)
    L_t, D_t = q * t.S_m2 * CL_t, q * t.S_m2 * CD_t
    F_t = L_t * e_L + D_t * e_D
    M_t = np.cross(ac.htail_ac_from_cg(nacelle_deg, **mass_kw), F_t)

    # ---- vertical tail (side force from sideslip and rudder) ----
    F_v, M_v = np.zeros(3), np.zeros(3)
    if vt is not None:
        CY_v = vt.CL_alpha * (-beta_rad + vt.tau_r * np.radians(rudder_deg))
        F_v = q * vt.S_m2 * (CY_v * np.array([0.0, 1.0, 0.0]) + vt.CD0 * e_D)
        M_v = np.cross(ac.vtail_ac_from_cg(nacelle_deg, **mass_kw), F_v)

    # ---- fuselage + nacelles (flat plate, at the CG) ----
    D_f = q * ac.flat_plate_area_m2
    F_f = D_f * e_D

    return AirframeLoads(
        F_body_N=F_w + F_t + F_v + F_f, M_cg_Nm=M_w + M_t + M_v,
        alpha_wing_deg=np.degrees(a_w), CL_wing=CL_w, L_wing_N=L_w, D_wing_N=D_w,
        alpha_htail_deg=np.degrees(a_t), CL_htail=CL_t, L_htail_N=L_t, downwash_deg=np.degrees(eps),
        D_fuselage_N=D_f, wing_stalled=st_w, htail_stalled=st_t, q_Pa=q,
        alpha_wing_stall_deg=w.alpha_stall_deg)
