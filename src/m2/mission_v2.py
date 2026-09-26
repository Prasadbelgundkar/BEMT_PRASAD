import numpy as np
from typing import Callable, List, Dict
from dataclasses import dataclass

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.trim_solver import trim_aircraft, compute_aircraft_residual
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
    alpha_shaft_rad = x_trim[0] + np.radians(90.0 - nacelle_deg)
    rotor_result = run_edgewise_bemt(
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        V_inf=max(0.1, V_inf),
        omega_rad_s=omega_rad_s,
        theta0_rad=x_trim[1],
        theta1c_rad=0.0,
        theta1s_rad=x_trim[2] if trim_pitch_with == 'cyclic' else 0.0,
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
    sfc_kg_J: float = 8.0e-8 # Specific fuel consumption roughly typical for turboshaft
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
    V_pts = np.linspace(0.1, cruise_velocity_m_s, 5)
    N_pts = np.linspace(90.0, 0.0, 5)
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
