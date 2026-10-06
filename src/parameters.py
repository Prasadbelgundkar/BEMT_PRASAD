"""
parameters.py
-------------
Configuration used by the Milestone 1 axial-flight and hover-map scripts
(rotor geometry, operating point, engine model, mission plan).
"""

import numpy as np
from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil

# ==========================================
# 1. ENVIRONMENT SETTINGS
# ==========================================
ALTITUDE_M = 10000.0    # Operating altitude in meters
DISA_K = 0.0              # Temperature offset from ISA in Kelvin (+15 for hot day)

# ==========================================
# 2. ROTOR GEOMETRY (Task 5 Design Variables)
# ==========================================
RADIUS_M = 3.8            # Blade radius (meters)
ROOT_CUTOUT_M = 0.5       # Root cutout radius (meters)
NUM_BLADES = 3            # Number of blades per rotor

# Chord / Taper
ROOT_CHORD_M = 0.90       # Chord length at the root (meters)
TAPER_RATIO = 0.3888      # Tip chord / Root chord (0.35m / 0.90m)

# Twist
TWIST_ROOT_DEG = 25.0     # Built-in pitch at the root (r/R = 0)
TWIST_RATE_DEG = -45.0    # Linear washout per unit r/R (proprotor-type twist)

# Airfoils: spanwise-blended linear sections read from data/my_blended_airfoils.csv
import os
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
AIRFOIL_PROVIDER = BlendedLinearAirfoilProvider.from_csv(os.path.join(DATA_DIR, "my_blended_airfoils.csv"))
# Single linear section along the whole span:
# AIRFOIL_PROVIDER = lambda x: LinearAirfoil()

# ==========================================
# 3. OPERATING CONDITIONS
# ==========================================
OMEGA_RPM = 500.0         # Rotor speed in RPM
COLLECTIVE_DEG = 8.0      # Collective pitch angle (degrees)
V_AXIAL_MPS = 0.0         # Forward speed or climb speed (m/s). 0.0 = Hover.

# ==========================================
# AUTO-BUILDER
# ==========================================
def get_configured_rotor() -> Rotor:
    """Creates and returns the Rotor object based on the parameters above."""
    chord_fn = linear_taper_chord(ROOT_CHORD_M, TAPER_RATIO)
    twist_fn = linear_twist(np.radians(TWIST_ROOT_DEG), np.radians(TWIST_RATE_DEG))
    
    return Rotor(
        radius_m=RADIUS_M,
        root_cutout_m=ROOT_CUTOUT_M,
        num_blades=NUM_BLADES,
        chord_fn=chord_fn,
        twist_fn=twist_fn
    )

# ==========================================
# 4. AIRCRAFT MASS & GEOMETRY (Mission Planner)
# ==========================================
EMPTY_MASS_KG = 4500.0          # Aircraft empty weight
PAYLOAD_MASS_KG = 1200.0        # 2 pilots + 10 passengers (100kg each)
FUEL_MASS_KG = 1500.0           # Fuel weight for 1000km mission
FLAT_PLATE_AREA_M2 = 1.7        # Equivalent flat plate area for drag (f)

# ==========================================
# 5. ENGINE MODEL (GE CT7-8A)
# ==========================================
ENGINE_POWER_W = 1_880_000.0    # 2520 shp per engine
ENGINE_SFC_KG_J = 7.60e-8       # 0.45 lb/shp-hr
NUM_ENGINES = 2

# ==========================================
# 6. DESIGN LIMITS
# ==========================================
MAX_TIP_MACH = 0.90
MAX_STALL_FRACTION = 0.40       # Allow 40% stall during high-speed cruise
MIN_POWER_MARGIN_FRAC = 0.05    # 5% safety margin on power
RESERVE_FUEL_KG = 450.0         # Absolute minimum fuel allowed

# ==========================================
# 7. MISSION PROFILE (Flight Plan)
# ==========================================
# Mission altitudes (AMSL = above mean sea level):
TAKEOFF_ALTITUDE_AMSL_M = 0.0
CLIMB_ALTITUDE_AMSL_M = 3500.0
CRUISE_ALTITUDE_AMSL_M = 7000.0
DROP_ALTITUDE_AMSL_M = 1000.0
LANDING_ALTITUDE_AMSL_M = 0.0

# dt_s = time step between aerodynamic evaluations [s] (per segment)
MISSION_PLAN = [
    # (Name, Type, Duration[s], Alt[m], RPM, Collective[deg], Vertical/Cruise Speed[m/s], dt_s)
    ("Takeoff hover", "HOVER", 60, TAKEOFF_ALTITUDE_AMSL_M, 550, 8.0, 0.0, 10),
    ("Climb to Ceiling", "VERTICAL_CLIMB", 600, CLIMB_ALTITUDE_AMSL_M, 550, 10.0, 5.0, 30),
    ("Max Range Cruise (30% Reserve)", "CRUISE", 25500, CRUISE_ALTITUDE_AMSL_M, 250, 56.5, 74.3, 60),
    ("Troop Drop Hover", "HOVER", 120, DROP_ALTITUDE_AMSL_M, 550, 5.0, 0.0, 10),
    ("Landing hover", "HOVER", 60, LANDING_ALTITUDE_AMSL_M, 550, 8.0, 0.0, 10),
]
