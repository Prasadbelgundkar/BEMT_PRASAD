import numpy as np

def R_body_to_inertial(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Euler rotation matrix: Body frame to Inertial frame (NED).
    Standard aerospace 3-2-1 rotation sequence (Yaw, Pitch, Roll).
    
    Body Frame: +X forward, +Y right, +Z down
    Inertial Frame: +X North, +Y East, +Z Down
    """
    phi = np.radians(roll_deg)
    theta = np.radians(pitch_deg)
    psi = np.radians(yaw_deg)
    
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_th, s_th = np.cos(theta), np.sin(theta)
    c_psi, s_psi = np.cos(psi), np.sin(psi)
    
    R_x = np.array([
        [1, 0, 0],
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])
    
    R_y = np.array([
        [c_th, 0, s_th],
        [0, 1, 0],
        [-s_th, 0, c_th]
    ])
    
    R_z = np.array([
        [c_psi, -s_psi, 0],
        [s_psi, c_psi, 0],
        [0, 0, 1]
    ])
    
    return R_z @ R_y @ R_x

def R_shaft_to_body(i_n_deg: float) -> np.ndarray:
    """
    Rotation matrix from Rotor Shaft (Hub) frame to Aircraft Body frame.
    
    Venkatesan Hub Frame (H):
    +X_H = aft
    +Y_H = right (advancing side for CCW rotor at psi=90)
    +Z_H = up (thrust direction)
    
    Aircraft Body Frame (b):
    +X_b = forward
    +Y_b = right
    +Z_b = down
    
    Tiltrotor Nacelle Angle (i_n):
    i_n = 90 deg (Helicopter mode): +Z_H points up (-Z_b), +X_H points aft (-X_b)
    i_n = 0 deg (Airplane mode): +Z_H points forward (+X_b), +X_H points UP (-Z_b)
    (the shaft frame is pitched nose-down by 90 - i_n about Y; proper rotation, det = +1)
    """
    theta = np.radians(i_n_deg)
    c = np.cos(theta)
    s = np.sin(theta)
    
    # Transformation derived in PDR:
    # x_b = -x_s * sin(theta) + z_s * cos(theta)
    # y_b =  y_s
    # z_b = -x_s * cos(theta) - z_s * sin(theta)
    return np.array([
        [-s,  0,  c],
        [ 0,  1,  0],
        [-c,  0, -s]
    ])

def R_blade_to_hub(psi_rad: float) -> np.ndarray:
    """
    Transforms a vector from the rotating blade frame to the non-rotating hub frame.
    Blade at psi=0 is along +X_H (aft).
    Blade at psi=90 is along +Y_H (right).
    """
    c = np.cos(psi_rad)
    s = np.sin(psi_rad)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def transform_force_shaft_to_body(F_shaft: np.ndarray, i_n_deg: float) -> np.ndarray:
    """
    Transforms a force vector from the shaft frame to the body frame.
    """
    return R_shaft_to_body(i_n_deg) @ F_shaft

def transform_moment_about_cg(F_body: np.ndarray, M_hub_body: np.ndarray, r_hub_cg_body: np.ndarray) -> np.ndarray:
    """
    Computes total moment about the CG in the body frame.
    M_total = M_hub_body + (r_hub_cg_body x F_body)
    
    F_body: Force at the hub in body frame (3,)
    M_hub_body: Moment at the hub in body frame (3,)
    r_hub_cg_body: Position vector from CG to Hub in body frame (3,)
    """
    return M_hub_body + np.cross(r_hub_cg_body, F_body)
