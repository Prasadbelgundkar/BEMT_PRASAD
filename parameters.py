"""
parameters.py
-------------
Centralized configuration file for the BEMT Solver and Mission Planner.
Change your design variables here, and all other scripts will read from this file.
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
ROOT_CHORD_M = 0.45       # Chord length at the root (meters)
TAPER_RATIO = 1.0         # Tip chord / Root chord (1.0 = constant chord)

# Twist
TWIST_ROOT_DEG = 15.0     # Built-in twist at the root (degrees)
TWIST_RATE_DEG = -30.0    # Twist change from root to tip (degrees). (15 to -15 = -30)

# Airfoils
# If using the blended CSV approach we just built, you would uncomment this:
#AIRFOIL_PROVIDER = BlendedLinearAirfoilProvider.from_csv("data/my_blended_airfoils.csv")

# use a simple constant linear airfoil provider so it runs out-of-the-box:
AIRFOIL_PROVIDER = lambda x: LinearAirfoil()

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
RESERVE_FUEL_KG = 100.0         # Absolute minimum fuel allowed

# ==========================================
# 7. MISSION PROFILE (Flight Plan)
# ==========================================
# Define your mission altitudes here (AMSL = Above Mean Sea Level):
TAKEOFF_ALTITUDE_AMSL_M = 0.0
CLIMB_ALTITUDE_AMSL_M = 3500.0
CRUISE_ALTITUDE_AMSL_M = 7000.0
DROP_ALTITUDE_AMSL_M = 1000.0
LANDING_ALTITUDE_AMSL_M = 0.0

# You can change the time-step (dt_s) here to speed up or slow down the simulation!
# dt_s = Time between aerodynamic calculations (in seconds)
MISSION_PLAN = [
    # (Name, Type, Duration[s], Alt[m], RPM, Collective[deg], Vertical/Cruise Speed[m/s], dt_s)
    ("Takeoff hover", "HOVER", 60, TAKEOFF_ALTITUDE_AMSL_M, 550, 8.0, 0.0, 60),
    ("Climb to Ceiling", "VERTICAL_CLIMB", 600, CLIMB_ALTITUDE_AMSL_M, 550, 10.0, 5.0, 300),
    ("High-Alt Cruise", "CRUISE", 8000, CRUISE_ALTITUDE_AMSL_M, 450, 54.5, 125.0, 1000),
    ("Troop Drop Hover", "HOVER", 120, DROP_ALTITUDE_AMSL_M, 550, 5.0, 0.0, 120),
    ("Landing hover", "HOVER", 60, LANDING_ALTITUDE_AMSL_M, 550, 8.0, 0.0, 60),
]
