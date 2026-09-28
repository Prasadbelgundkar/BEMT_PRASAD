import numpy as np
from typing import Callable, Tuple, List, Dict
from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.trim_solver import trim_aircraft, compute_aircraft_residual
from m2.edgewise_bemt import run_edgewise_bemt

@dataclass
class TrimPointResult:
    V_inf: float
    nacelle_deg: float
    success: bool
    alpha_deg: float
    collective_deg: float
    ctrl_deg: float
    power_kW: float
    wing_stalled: bool
    rotor_stalled_fraction: float
    message: str

def evaluate_trim_point(
    V_inf: float, 
    nacelle_deg: float, 
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    omega_rad_s: float,
    x0: np.ndarray,
    trim_pitch_with: str
) -> TrimPointResult:
    """
    Attempts to trim the aircraft at a specific point in the corridor.
    """
    try:
        x_trim = trim_aircraft(
            V_inf=V_inf,
            gamma_rad=0.0,
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
        if x_trim is None:
            return TrimPointResult(V_inf, nacelle_deg, False, 0, 0, 0, 0, False, 0.0, "Trim failed")
            
        # Verify residuals
        res = compute_aircraft_residual(
            x_trim, V_inf, 0.0, nacelle_deg, omega_rad_s,
            aircraft, rotor, airfoil_provider, rho, a_sound, trim_pitch_with
        )
        
        # If residuals are too high, trim failed
        if np.max(np.abs(res)) > 0.05:
            return TrimPointResult(V_inf, nacelle_deg, False, 0, 0, 0, 0, False, 0.0, "High residuals")
            
        alpha_rad = x_trim[0]
        alpha_wing = alpha_rad + np.radians(aircraft.wing.i_w_deg)
        wing_stalled = abs(alpha_wing) > np.radians(15.0)
        
        # Run BEMT one more time to get power and stall fraction
        alpha_shaft_rad = alpha_rad + np.radians(90.0 - nacelle_deg)
        rotor_result = run_edgewise_bemt(
            rotor=rotor,
            airfoil_provider=airfoil_provider,
            V_inf=V_inf,
            omega_rad_s=omega_rad_s,
            theta0_rad=x_trim[1],
            theta1c_rad=0.0,
            theta1s_rad=x_trim[2] if trim_pitch_with == 'cyclic' else 0.0,
            alpha_shaft_rad=alpha_shaft_rad,
            nacelle_angle_deg=nacelle_deg,
            rho=rho,
            a_sound=a_sound,
            n_r=30,
            n_psi=36
        )
        
        total_power_kW = 2.0 * rotor_result.power_W / 1000.0
        
        return TrimPointResult(
            V_inf=V_inf,
            nacelle_deg=nacelle_deg,
            success=True,
            alpha_deg=np.degrees(alpha_rad),
            collective_deg=np.degrees(x_trim[1]),
            ctrl_deg=np.degrees(x_trim[2]),
            power_kW=total_power_kW,
            wing_stalled=wing_stalled,
            rotor_stalled_fraction=rotor_result.stalled_fraction,
            message="OK"
        )
        
    except Exception as e:
        return TrimPointResult(V_inf, nacelle_deg, False, 0, 0, 0, 0, False, 0.0, str(e))

def compute_conversion_corridor(
    V_sweep: np.ndarray,
    nacelle_sweep: np.ndarray,
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    omega_rad_s: float
) -> List[TrimPointResult]:
    
    results = []
    
    # Simple nested loop. (Could be parallelized, but BEMT is fast enough for a coarse grid)
    # We use the previous airspeed's trim state as the seed for the next one to speed up convergence
    
    for nacelle in nacelle_sweep:
        print(f"Sweeping Nacelle = {nacelle} deg...")
        
        # Reset seed for each nacelle angle
        x0 = np.array([np.radians(5.0), np.radians(15.0), 0.0])
        
        # Decide control effector
        # Use cyclic at high nacelle (helicopter), elevator at low nacelle (airplane)
        trim_pitch_with = 'cyclic' if nacelle > 45.0 else 'elevator'
        
        for V in V_sweep:
            # Skip very low speeds if we are in airplane mode (can't fly at 5 m/s with nacelle 0)
            # This just saves computation time on points we know will fail
            if nacelle < 30.0 and V < 20.0:
                results.append(TrimPointResult(V, nacelle, False, 0, 0, 0, 0, True, 1.0, "Too slow for nacelle angle"))
                continue
                
            res = evaluate_trim_point(V, nacelle, aircraft, rotor, airfoil_provider, rho, a_sound, omega_rad_s, x0, trim_pitch_with)
            results.append(res)
            
            if res.success:
                x0 = np.array([np.radians(res.alpha_deg), np.radians(res.collective_deg), np.radians(res.ctrl_deg)])
                
    return results
