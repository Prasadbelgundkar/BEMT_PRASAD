from dataclasses import dataclass
import numpy as np

@dataclass
class WingGeometry:
    S_m2: float
    AR: float
    e_oswald: float
    i_w_deg: float  # Incidence angle of the wing relative to body
    
    @property
    def span_m(self) -> float:
        return np.sqrt(self.S_m2 * self.AR)
        
    @property
    def chord_m(self) -> float:
        return self.S_m2 / self.span_m

@dataclass
class TailGeometry:
    S_m2: float
    AR: float
    e_oswald: float
    i_t_deg: float # Incidence angle of horizontal tail

@dataclass
class AircraftGeometryM2:
    W_MTOW_N: float
    flat_plate_area_m2: float
    
    wing: WingGeometry
    htail: TailGeometry
    
    # CG locations (x is positive forward, z is positive down in our body frame)
    # Origin is at the CG. We need the coordinates of the aerodynamic centers
    # and the rotor hubs relative to the CG.
    
    cg_to_wing_ac_m: np.ndarray  # [x, y, z] body
    cg_to_htail_ac_m: np.ndarray # [x, y, z] body
    
    cg_to_right_rotor_m: np.ndarray # [x, y, z] body
    cg_to_left_rotor_m: np.ndarray  # [x, y, z] body

# Default V-22 / Generic Tiltrotor Class Scale
def get_default_aircraft() -> AircraftGeometryM2:
    return AircraftGeometryM2(
        W_MTOW_N=7200.0 * 9.81,
        flat_plate_area_m2=1.8,
        wing=WingGeometry(
            S_m2=39.24,
            AR=9.0,
            e_oswald=0.8,
            i_w_deg=4.0
        ),
        htail=TailGeometry(
            S_m2=8.0,
            AR=4.5,
            e_oswald=0.8,
            i_t_deg=0.0
        ),
        # Assuming CG is near the wing aerodynamic center
        cg_to_wing_ac_m=np.array([0.0, 0.0, -0.5]), 
        cg_to_htail_ac_m=np.array([-6.0, 0.0, -0.5]), 
        # Rotors are at the wingtips. Wing span is ~18.8m, so y is +/- 9.4m
        cg_to_right_rotor_m=np.array([0.5, 9.4, -0.5]),
        cg_to_left_rotor_m=np.array([0.5, -9.4, -0.5])
    )
