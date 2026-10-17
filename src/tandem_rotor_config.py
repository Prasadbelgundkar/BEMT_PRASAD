"""
tandem_rotor_config.py

Geometry and baseline parameters for a tandem-rotor helicopter (CH-47 Chinook class).
Forward and aft rotors are defined independently so unequal radii or asymmetric
blade designs can be tested without touching the interference model.
"""

import os
import sys
from typing import Callable, Optional
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from airfoil import LinearAirfoil
from rotor import Rotor, linear_twist, linear_taper_chord, constant_chord, constant_twist
from tandem_interference import compute_overlap_fraction


# --- Airfoil ---
TANDEM_AIRFOIL = LinearAirfoil(
    a0=5.73,
    Cd_min=0.0095,
    eps=1.15,
    stall_alpha_rad=np.radians(14.0),
    Cl_max_clip=1.45,
)

def default_airfoil_provider(x: float) -> LinearAirfoil:
    return TANDEM_AIRFOIL


# --- Forward rotor geometry ---
FWD_ROTOR_RADIUS_M  = 9.14    # tip radius [m] (30 ft)
FWD_ROOT_CUTOUT_M   = 1.10    # root cutout (~0.12 R)
FWD_NUM_BLADES      = 3
FWD_ROOT_CHORD_M    = 0.82
FWD_TAPER_RATIO     = 0.85
FWD_TWIST_ROOT_DEG  = 10.0
FWD_TWIST_RATE_DEG  = -8.0    # washout [deg per unit r/R]

# --- Aft rotor geometry (independent from forward) ---
AFT_ROTOR_RADIUS_M  = 9.14    # can differ from forward rotor
AFT_ROOT_CUTOUT_M   = 1.10
AFT_NUM_BLADES      = 3
AFT_ROOT_CHORD_M    = 0.82
AFT_TAPER_RATIO     = 0.85
AFT_TWIST_ROOT_DEG  = 10.0
AFT_TWIST_RATE_DEG  = -8.0

# --- Tandem arrangement ---
LONGITUDINAL_SEP_M  = 13.60   # center-to-center [m] (~1.49 R)
LATERAL_OFFSET_M    = 0.00    # in-line tandem
VERTICAL_STAGGER_M  = 1.30    # aft rotor pylon height above forward [m]

HOVER_RPM   = 225.0
HOVER_OMEGA = 2.0 * np.pi * HOVER_RPM / 60.0  # rad/s

# --- Mass & power ---
GROSS_MASS_KG           = 22_680.0   # MTOW [kg] (~50,000 lb)
EMPTY_MASS_KG           = 11_150.0
MAX_PAYLOAD_KG          = 10_880.0
FUEL_CAPACITY_KG        = 3_150.0
G                       = 9.80665
INSTALLED_SHAFT_POWER_W = 7_000_000.0   # 2 x 3500 kW
DRIVETRAIN_EFFICIENCY   = 0.96


def build_fwd_rotor(
    radius_m: float = FWD_ROTOR_RADIUS_M,
    root_cutout_m: float = FWD_ROOT_CUTOUT_M,
    num_blades: int = FWD_NUM_BLADES,
    root_chord_m: float = FWD_ROOT_CHORD_M,
    taper_ratio: float = FWD_TAPER_RATIO,
    twist_root_deg: float = FWD_TWIST_ROOT_DEG,
    twist_rate_deg: float = FWD_TWIST_RATE_DEG,
    name: str = "Forward Rotor",
) -> Rotor:
    chord_fn = linear_taper_chord(root_chord_m, taper_ratio)
    twist_fn = linear_twist(np.radians(twist_root_deg), np.radians(twist_rate_deg))
    return Rotor(
        radius_m=radius_m,
        root_cutout_m=root_cutout_m,
        num_blades=num_blades,
        chord_fn=chord_fn,
        twist_fn=twist_fn,
        name=name,
    )


def build_aft_rotor(
    radius_m: float = AFT_ROTOR_RADIUS_M,
    root_cutout_m: float = AFT_ROOT_CUTOUT_M,
    num_blades: int = AFT_NUM_BLADES,
    root_chord_m: float = AFT_ROOT_CHORD_M,
    taper_ratio: float = AFT_TAPER_RATIO,
    twist_root_deg: float = AFT_TWIST_ROOT_DEG,
    twist_rate_deg: float = AFT_TWIST_RATE_DEG,
    name: str = "Aft Rotor",
) -> Rotor:
    chord_fn = linear_taper_chord(root_chord_m, taper_ratio)
    twist_fn = linear_twist(np.radians(twist_root_deg), np.radians(twist_rate_deg))
    return Rotor(
        radius_m=radius_m,
        root_cutout_m=root_cutout_m,
        num_blades=num_blades,
        chord_fn=chord_fn,
        twist_fn=twist_fn,
        name=name,
    )


def get_geometric_overlap(
    R_fwd: float = FWD_ROTOR_RADIUS_M,
    R_aft: float = AFT_ROTOR_RADIUS_M,
    longitudinal_sep_m: float = LONGITUDINAL_SEP_M,
    lateral_offset_m: float = LATERAL_OFFSET_M,
) -> float:
    return compute_overlap_fraction(
        R_fwd=R_fwd,
        R_aft=R_aft,
        longitudinal_sep_m=longitudinal_sep_m,
        lateral_offset_m=lateral_offset_m,
    )


def hover_weight_per_rotor_N(gross_mass_kg: float = GROSS_MASS_KG, split: float = 0.5) -> float:
    return gross_mass_kg * G * split


if __name__ == "__main__":
    r_fwd = build_fwd_rotor()
    r_aft = build_aft_rotor()
    overlap = get_geometric_overlap(r_fwd.radius_m, r_aft.radius_m, LONGITUDINAL_SEP_M, LATERAL_OFFSET_M)
    print("Tandem Helicopter Baseline Configuration")
    print(f"  Forward Rotor Radius : {r_fwd.radius_m:.2f} m, Solidity: {r_fwd.solidity():.4f}")
    print(f"  Aft Rotor Radius     : {r_aft.radius_m:.2f} m, Solidity: {r_aft.solidity():.4f}")
    print(f"  Longitudinal Sep     : {LONGITUDINAL_SEP_M:.2f} m ({LONGITUDINAL_SEP_M / r_fwd.radius_m:.2f} R)")
    print(f"  Lateral Offset       : {LATERAL_OFFSET_M:.2f} m")
    print(f"  Disk Overlap Fraction: {overlap * 100.0:.2f}%")
    print(f"  Total MTOW           : {GROSS_MASS_KG:.1f} kg ({GROSS_MASS_KG * G / 1e3:.1f} kN)")
    print(f"  Hover Tip Speed      : {r_fwd.tip_speed_mps(HOVER_OMEGA):.1f} m/s")
