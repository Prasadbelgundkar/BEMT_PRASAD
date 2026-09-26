import numpy as np
from scipy.optimize import root
from typing import Callable

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.aero_models import compute_wing_aero, compute_htail_aero
from m2.edgewise_bemt import run_edgewise_bemt

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
    theta1s_rad = pitch_ctrl if trim_pitch_with == 'cyclic' else 0.0
    
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
    alpha_shaft_rad = alpha_body_rad + np.radians(90.0 - nacelle_deg)
    
    rotor_result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=V_inf,
        omega_rad_s=omega_rad_s,
        theta0_rad=collective_rad,
        theta1c_rad=0.0,
        theta1s_rad=theta1s_rad,
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
    # M = r x F. Since symmetry, y offset cancels for pitch moment
    # My_offset = Fz * x_offset - Fx * z_offset
    My_rotors += 2.0 * (rotor_result.forces_body_N[2] * aircraft.cg_to_right_rotor_m[0] - 
                        rotor_result.forces_body_N[0] * aircraft.cg_to_right_rotor_m[2])
    
    # Wing and Tail pitching moments
    My_wing = wing_aero.M_Nm + (wing_aero.Fz_body_N * aircraft.cg_to_wing_ac_m[0] - wing_aero.Fx_body_N * aircraft.cg_to_wing_ac_m[2])
    My_htail = htail_aero.M_Nm + (htail_aero.Fz_body_N * aircraft.cg_to_htail_ac_m[0] - htail_aero.Fx_body_N * aircraft.cg_to_htail_ac_m[2])
    
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
    trim_pitch_with: str = 'elevator'
) -> np.ndarray:
    
    # Cost function for scipy.optimize.root
    def cost(x):
        return compute_aircraft_residual(
            x, V_inf, gamma_rad, nacelle_deg, omega_rad_s,
            aircraft, rotor, airfoil_provider, rho, a_sound, trim_pitch_with
        )
        
    res = root(cost, x0, method='hybr')
    if not res.success:
        print(f"Warning: Trim did not converge at V={V_inf}. Msg: {res.message}")
        
    return res.x
