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
