"""
aircraft_input.py  --  MILESTONE 1 AIRCRAFT CONFIGURATION
==========================================================
Single definition of the tiltrotor (rotor, airfoil, masses, engines, fuel
model, limits). The analysis scripts and the Milestone 2 configuration
(m2/aircraft_input_m2.py) import the design from here.
"""
import numpy as np, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from airfoil import LinearAirfoil, TableAirfoil
from rotor import Rotor, linear_twist, linear_taper_chord, constant_chord, constant_twist
from mission import PowerAvailableModel, FuelModel, DesignLimits

# ============================================================
# SECTION 1 -- AIRFOIL SELECTION
# ============================================================
# Option A: Linear model (matches Knight & Hefner validation airfoil)
AIRFOIL = LinearAirfoil(
    a0=5.75,                          # lift curve slope [1/rad]
    Cd_min=0.0113,                    # minimum profile drag
    eps=1.25,                         # quadratic drag factor
    stall_alpha_rad=np.radians(14.0), # stall angle [rad]
    Cl_max_clip=1.4,                  # post-stall Cl cap
)
AIRFOIL_NAME = "Linear (a0=5.75, Cd=0.0113+1.25*alpha^2)"

# Option B: tabulated polar (not used in the reported results)
# AIRFOIL = TableAirfoil(
#     alpha_deg=[-12, -8, -4, 0, 4, 8, 12, 14, 16],
#     Cl=      [-1.0, -0.6,-0.2, 0.2, 0.6, 1.0, 1.3, 1.35, 1.2],
#     Cd=      [0.030,0.015,0.009,0.008,0.009,0.015,0.030,0.045,0.080],
#     name="Custom-Airfoil",
# )
# AIRFOIL_NAME = "Custom Tabulated Airfoil"

def airfoil_provider(x):
    """Airfoil as a function of non-dimensional radius x=r/R.
    Returns the same airfoil across the full span (modify for spanwise variation)."""
    return AIRFOIL

# ============================================================
# SECTION 2 -- ROTOR GEOMETRY
# ============================================================
ROTOR_RADIUS_M  = 3.8     # tip radius [m]
ROOT_CUTOUT_M   = 0.50    # root cutout radius [m] (= 0.132 R)
NUM_BLADES      = 3       # number of blades

# Chord distribution (choose one and comment the other):
ROOT_CHORD_M = 0.90                              # root chord [m]
TAPER_RATIO  = 0.3888                            # tip_chord / root_chord  (tip = 0.35 m)
CHORD_FN     = linear_taper_chord(ROOT_CHORD_M, TAPER_RATIO)
# CHORD_FN   = constant_chord(0.28)              # OR constant chord

# Twist distribution -- built-in twist; collective is added on top:
TWIST_ROOT_DEG       = 25.0    # pitch at blade root [deg]
TWIST_RATE_DEG_PER_R = -45.0  # linear washout rate [deg per unit r/R]  (V-22 Osprey style)
TWIST_FN = linear_twist(np.radians(TWIST_ROOT_DEG), np.radians(TWIST_RATE_DEG_PER_R))
# TWIST_FN = constant_twist(0.0)                 # OR untwisted

# Assemble rotor object (used by all scripts)
ROTOR = Rotor(
    radius_m=ROTOR_RADIUS_M, root_cutout_m=ROOT_CUTOUT_M, num_blades=NUM_BLADES,
    chord_fn=CHORD_FN, twist_fn=TWIST_FN, name="TW-7200 Proprotor",
)

# ============================================================
# SECTION 3 -- RPM SCHEDULE
# ============================================================
HOVER_RPM  = 550.0   # rotor speed in hover / climb [RPM]
CRUISE_RPM = 250.0   # rotor speed in airplane-mode cruise [RPM] (74.3 m/s, 7000 m: J ~ 2.35)

HOVER_OMEGA  = 2 * np.pi * HOVER_RPM  / 60.0   # [rad/s]
CRUISE_OMEGA = 2 * np.pi * CRUISE_RPM / 60.0   # [rad/s]

# Collective pitch limits [deg] -- on top of built-in twist
MIN_COLLECTIVE_DEG = -10.0
MAX_COLLECTIVE_DEG = 65.0

# Default collective angles (auto-trim will refine hover collective)
HOVER_COLLECTIVE_DEG  = 8.0    # initial guess [deg]
CRUISE_COLLECTIVE_DEG = 56.5   # airplane-mode cruise trim, 74.3 m/s at 7000 m [deg]

# ============================================================
# SECTION 4 -- TWIN-ROTOR ARRANGEMENT
# ============================================================
NUM_ROTORS = 2  # total proprotors on the aircraft (each identical)

# ============================================================
# SECTION 5 -- AIRCRAFT MASSES [kg]
# ============================================================
GROSS_MASS_KG   = 7200.0   # maximum takeoff mass (empty + payload + fuel)
EMPTY_MASS_KG   = 4500.0   # operating empty mass (incl. crew)
PAYLOAD_KG      = 1200.0   # design payload (2 pilots + 10 pax @ 100 kg each)
FUEL_MASS_KG    = 1500.0   # usable fuel onboard
RESERVE_FUEL_KG =  450.0   # mandatory reserve fuel

# ============================================================
# SECTION 6 -- PERFORMANCE REQUIREMENTS & ENVIRONMENT
# ============================================================
TAKEOFF_ALTITUDE_M  =    0.0   # takeoff / hover altitude [m ASL]
SERVICE_CEILING_M   = 7000.0   # service ceiling [m]
CRUISE_ALTITUDE_M   = 7000.0   # cruise altitude [m]
dISA_K              =    0.0   # ISA temperature offset [K]; 0 = standard day
HOVER_REQ_ALTITUDE_M = 1500.0  # OGE hover requirement at MTOW: up to 1500 m ...
HOVER_REQ_DISA_K     =   15.0  # ... at ISA + 15 K

CRUISE_SPEED_MPS    =   74.3   # design cruise true airspeed [m/s]
MAX_SPEED_TARGET_MPS =  125.0  # 450 km/h speed target [m/s]
RANGE_TARGET_KM     = 1000.0   # design range target [km]
HOVER_ENDURANCE_MIN =   30.0   # minimum hover endurance [min]

# Equivalent flat-plate drag area of aircraft (fuselage + nacelles) in cruise [m^2]
# Propulsive thrust required = 0.5 * rho * V^2 * FLAT_PLATE_AREA_M2
# Typical V-22 class: 1.5 -- 3.0 m^2
FLAT_PLATE_AREA_M2 = 1.7

# ============================================================
# SECTION 7 -- POWER AVAILABLE MODEL
# ============================================================
SEA_LEVEL_TOTAL_POWER_W = 2 * 1_880_000.0  # total installed shaft power [W] (2 turboshafts)
DENSITY_RATIO_EXPONENT  = 1.0          # P ~ P_SL * (rho/rho_SL)^exponent
DRIVETRAIN_EFFICIENCY   = 0.95         # gearbox + shaft losses

SEA_LEVEL_POWER_PER_ROTOR_W = SEA_LEVEL_TOTAL_POWER_W / NUM_ROTORS

POWER_MODEL = PowerAvailableModel(
    sea_level_power_W=SEA_LEVEL_POWER_PER_ROTOR_W,
    density_ratio_exponent=DENSITY_RATIO_EXPONENT,
    drivetrain_efficiency=DRIVETRAIN_EFFICIENCY,
)

# ============================================================
# SECTION 8 -- FUEL MODEL (TURBOSHAFT)
# ============================================================
# SFC [kg per joule of shaft energy]: 8.33e-8 kg/J == 0.30 kg/kWh
SFC_KG_PER_J = 8.33e-8
FUEL_MODEL = FuelModel(sfc_kg_per_J=SFC_KG_PER_J)

# ============================================================
# SECTION 9 -- DESIGN LIMITS
# ============================================================
MAX_TIP_MACH          = 0.85
MAX_STALL_FRACTION    = 0.05   # 5% of blade span
MIN_POWER_MARGIN_FRAC = 0.05   # 5% power margin
MIN_RPM = 250.0
MAX_RPM = 900.0

LIMITS = DesignLimits(
    max_tip_mach=MAX_TIP_MACH, max_stall_fraction=MAX_STALL_FRACTION,
    min_power_margin_frac=MIN_POWER_MARGIN_FRAC,
    min_rpm=MIN_RPM, max_rpm=MAX_RPM,
    min_collective_deg=MIN_COLLECTIVE_DEG, max_collective_deg=MAX_COLLECTIVE_DEG,
    reserve_fuel_kg=RESERVE_FUEL_KG,
)

# ============================================================
# SECTION 10 -- CONVENIENCE HELPERS
# ============================================================
G = 9.80665  # gravitational acceleration [m/s^2]

def weight_per_rotor_N(gross_mass_kg=GROSS_MASS_KG):
    """Required vertical thrust per rotor for steady level flight [N]."""
    return gross_mass_kg * G / NUM_ROTORS

def disk_loading_N_m2(gross_mass_kg=GROSS_MASS_KG):
    """Total disk loading W/(N_rotor * pi * R^2) [N/m^2]."""
    import math
    return gross_mass_kg * G / (NUM_ROTORS * math.pi * ROTOR_RADIUS_M**2)

def tip_speed_mps(omega=HOVER_OMEGA):
    """Blade tip speed [m/s]."""
    return omega * ROTOR_RADIUS_M

if __name__ == "__main__":
    print("Aircraft configuration summary")
    print(f"  Rotor:         R={ROTOR_RADIUS_M} m, B={NUM_BLADES}, sigma={ROTOR.solidity():.4f}")
    print(f"  Hover RPM:     {HOVER_RPM:.0f},  Vtip={tip_speed_mps(HOVER_OMEGA):.1f} m/s")
    print(f"  Gross mass:    {GROSS_MASS_KG:.0f} kg")
    print(f"  Disk loading:  {disk_loading_N_m2():.1f} N/m^2")
    print(f"  T per rotor:   {weight_per_rotor_N():.1f} N  (at MTOW)")
    print(f"  Airfoil:       {AIRFOIL_NAME}")
