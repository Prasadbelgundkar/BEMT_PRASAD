"""
aircraft_input_m2.py  --  MILESTONE 2 AIRCRAFT CONFIGURATION (single source of truth)
=====================================================================================
Every module in src/m2 and every script in scripts/m2 takes the aircraft from
HERE. Rotor planform, airfoil, RPM, collective range, masses and the power-lapse
model come from the Milestone 1 file `aircraft_input.py`; this file adds what
edgewise flight and trim need (wing, empennage, component locations, mass items
/ CG, control limits), the selected engine, and records the Milestone 2 design
changes (report Section 5.2).

Body axes (report Section 1.1): origin at the REFERENCE POINT = wing root
quarter-chord, +x forward, +y right, +z down. Component positions below are
given from this reference point; positions "from the CG" are derived.

Nacelle angle i_n: 90 deg = helicopter mode (shaft vertical), 0 deg = airplane
mode (shaft pointing forward). The proprotor and its gearbox tilt about the
conversion-actuator pivot at the wing tip; the engines tilt with the nacelles
(Milestone 1 layout).
"""
from dataclasses import dataclass, field
from typing import List, Optional
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import aircraft_input as M1                      # noqa: E402  (Milestone 1 master file)
from mission import PowerAvailableModel, FuelModel  # noqa: E402

G = M1.G

# ============================================================
# SECTION 1 -- ROTOR, AIRFOIL, FUEL (unchanged from Milestone 1)
# ============================================================
from rotor import Rotor, linear_twist                     # noqa: E402

# Rotor variant (report Sections 5.2 / 5.3 / 9.2). Select with the environment
# variable M2_ROTOR = 'refined' (default, the Milestone 2 design) or 'M1', e.g.
#     set M2_ROTOR=M1   (Windows cmd)  /  $env:M2_ROTOR="M1" (PowerShell)
#   'M1'      : Milestone 1 blade, twist 25 deg at the root, -45 deg/R.
#   'refined' : same planform, twist 12 deg at the root, -30 deg/R (less inboard
#               pitch: removes the inboard hover stall of the M1 blade at MTOW).
# Both variants use the Milestone 1 helicopter-mode RPM (550).
ROTOR_M1 = M1.ROTOR
ROTOR_REFINED = Rotor(
    radius_m=M1.ROTOR_RADIUS_M, root_cutout_m=M1.ROOT_CUTOUT_M, num_blades=M1.NUM_BLADES,
    chord_fn=M1.CHORD_FN, twist_fn=linear_twist(np.radians(12.0), np.radians(-30.0)),
    name="TW-7200 Proprotor (M2 refined twist)",
)
ROTOR_VARIANTS = {"M1": ROTOR_M1, "refined": ROTOR_REFINED}
ROTOR_VARIANT = os.environ.get("M2_ROTOR", "refined")
if ROTOR_VARIANT not in ROTOR_VARIANTS:
    raise ValueError(f"M2_ROTOR must be one of {list(ROTOR_VARIANTS)}, got {ROTOR_VARIANT!r}")
ROTOR = ROTOR_VARIANTS[ROTOR_VARIANT]
TWIST_DESC = {"M1": "25 deg root, -45 deg/R", "refined": "12 deg root, -30 deg/R"}[ROTOR_VARIANT]
airfoil_provider = M1.airfoil_provider
AIRFOIL = M1.AIRFOIL
AIRFOIL_NAME = M1.AIRFOIL_NAME
NUM_ROTORS = M1.NUM_ROTORS

# ============================================================
# SECTION 2 -- RPM SCHEDULE
# ============================================================
# Airplane-mode RPM is scheduled with advance ratio (6-DOF trim RPM study,
# report Section 5.3): the lowest RPM that trims with collective margin.
#   250 RPM  Milestone 1 long-range cruise (74.3 m/s, 7000 m): least power
#   350 RPM  airplane-mode cruise up to 100 m/s (250 RPM saturates collective above ~85 m/s)
#   420 RPM  high-speed dash above 100 m/s (450 km/h target)
HOVER_RPM = M1.HOVER_RPM            # helicopter mode and conversion (550, Milestone 1)
CONVERSION_RPM = HOVER_RPM
LONG_RANGE_RPM = M1.CRUISE_RPM      # 250
AIRPLANE_RPM = 350.0
DASH_RPM = 420.0


def rpm_to_omega(rpm: float) -> float:
    return 2.0 * np.pi * rpm / 60.0


HOVER_OMEGA = rpm_to_omega(HOVER_RPM)
CONVERSION_OMEGA = rpm_to_omega(CONVERSION_RPM)
AIRPLANE_OMEGA = rpm_to_omega(AIRPLANE_RPM)

# ============================================================
# SECTION 3 -- ENGINES AND POWER AVAILABLE
# ============================================================
# Selected by the engine-sizing study (scripts/m2/engine_selection.py, report
# Section 5.2): 2 x GE CT7-8A turboshaft, take-off rating 1893 kW each, one per
# nacelle, cross-shafted through the interconnect drive (one engine can drive
# both rotors). Dry mass ~245 kg and SFC ~280 g/kWh are estimates from the
# CT7 / T700 engine family (not published for the -8A).
ENGINE_NAME = "GE CT7-8A"
POWER_PER_ENGINE_SL_W = 1893e3
ENGINE_DRY_MASS_KG = 245.0
ENGINE_SFC_KG_PER_J = 280.0 / 3.6e9          # 280 g/kWh
MCP_FRACTION = 0.86                          # max continuous / take-off rating
POWER_MODEL = PowerAvailableModel(           # PER ROTOR (one engine per nacelle)
    sea_level_power_W=POWER_PER_ENGINE_SL_W,
    density_ratio_exponent=M1.DENSITY_RATIO_EXPONENT,
    drivetrain_efficiency=M1.DRIVETRAIN_EFFICIENCY,
)
FUEL_MODEL = FuelModel(sfc_kg_per_J=ENGINE_SFC_KG_PER_J)

# ============================================================
# SECTION 4 -- REFERENCE CONDITIONS
# ============================================================
GROSS_MASS_KG = M1.GROSS_MASS_KG
REFERENCE_ALTITUDE_M = 2000.0        # conversion corridor / trim matrix altitude
AIRPLANE_CRUISE_SPEED_MPS = 85.0     # transition-mission cruise at 2000 m (M1 design cruise: 74.3 m/s at 7000 m)

# Operational conversion path in the airspeed-nacelle plane (report Sections
# 7.1 / 8.1), chosen through the middle of the feasible corridor of the
# Section 7.1 map: (V [m/s], i_n [deg]). Mission Planner v2 schedules the
# nacelle angle as a function of airspeed along this path.
CONVERSION_PATH = [(0.0, 90.0), (10.0, 90.0), (20.0, 85.0), (30.0, 80.0), (40.0, 75.0),
                   (50.0, 60.0), (55.0, 45.0), (60.0, 30.0), (65.0, 15.0), (70.0, 0.0)]
CONVERSION_TIME_S = 100.0            # 0 -> 70 m/s at ~0.7 m/s^2, max nacelle rate ~2.1 deg/s
# Reconversion (deceleration) path: the nacelles are tilted back EARLIER and
# beyond vertical at low speed so the rotors supply the decelerating force
# instead of a nose-up attitude (which stalls the wing at 30-45 m/s).
RECONVERSION_PATH = [(0.0, 90.0), (12.0, 90.0), (25.0, 92.0), (35.0, 90.0), (42.0, 85.0),
                     (48.0, 72.0), (54.0, 55.0), (60.0, 35.0), (65.0, 15.0), (70.0, 0.0)]
RECONVERSION_TIME_S = 130.0          # 70 -> 0 m/s at ~0.54 m/s^2


# ============================================================
# SECTION 5 -- COMPONENT GEOMETRY DATACLASSES
# ============================================================
@dataclass
class WingGeometry:
    S_m2: float
    AR: float
    e_oswald: float
    i_w_deg: float              # incidence relative to body x-axis
    a0_per_rad: float = 2.0 * np.pi
    alpha_stall_deg: float = 15.0
    CD0: float = 0.015
    CM_ac: float = -0.1
    CD90: float = 1.2              # flat-plate drag at 90 deg (post-stall, e.g. vertical climb)
    # Flaperons used as ailerons (roll control in airplane mode)
    aileron_eta: tuple = (0.45, 0.95)   # inner/outer edge as fraction of the semi-span
    aileron_tau: float = 0.45           # effectiveness d(alpha)/d(delta_a)
    aileron_limit_deg: float = 20.0

    @property
    def span_m(self) -> float:
        return np.sqrt(self.S_m2 * self.AR)

    @property
    def chord_m(self) -> float:
        return self.S_m2 / self.span_m

    @property
    def Cl_delta_a(self) -> float:
        """Rolling-moment derivative per rad (strip theory, both flaperons)."""
        s = 0.5 * self.span_m
        y1, y2 = self.aileron_eta[0] * s, self.aileron_eta[1] * s
        return self.CL_alpha * self.aileron_tau * self.chord_m * (y2 ** 2 - y1 ** 2) / (self.S_m2 * self.span_m)

    @property
    def CL_alpha(self) -> float:
        return self.a0_per_rad / (1.0 + self.a0_per_rad / (np.pi * self.e_oswald * self.AR))

    @property
    def CL_max(self) -> float:
        return self.CL_alpha * np.radians(self.alpha_stall_deg)


@dataclass
class TailGeometry:
    S_m2: float
    AR: float
    e_oswald: float
    i_t_deg: float              # horizontal-tail incidence
    a0_per_rad: float = 2.0 * np.pi
    alpha_stall_deg: float = 12.0
    CD0: float = 0.015
    tau_e: float = 0.4          # elevator effectiveness d(alpha)/d(delta_e)
    elevator_chord_frac: float = 0.30
    elevator_limit_deg: float = 25.0

    @property
    def span_m(self) -> float:
        return np.sqrt(self.S_m2 * self.AR)

    @property
    def chord_m(self) -> float:
        return self.S_m2 / self.span_m


@dataclass
class VTailGeometry:
    S_m2: float
    AR: float
    a0_per_rad: float = 2.0 * np.pi
    CD0: float = 0.015
    rudder_chord_frac: float = 0.30
    rudder_limit_deg: float = 20.0
    tau_r: float = 0.5             # rudder effectiveness

    @property
    def height_m(self) -> float:
        return np.sqrt(self.S_m2 * self.AR)

    @property
    def CL_alpha(self) -> float:
        AR_eff = 1.55 * self.AR        # end-plate effect of fuselage / H-tail
        return self.a0_per_rad / (1.0 + self.a0_per_rad / (np.pi * 0.8 * AR_eff))


@dataclass
class MassItem:
    name: str
    mass_kg: float
    x_m: float                  # from reference point, +fwd (unused for tilting items)
    z_m: float                  # from reference point, +down (unused for tilting items)
    tilts_with_nacelle: bool = False
    shaft_offset_m: float = 0.0  # tilting items: distance from the pivot along the shaft axis
    category: str = "empty"      # 'empty', 'payload' or 'fuel'


@dataclass
class ControlLimits:
    collective_deg: tuple = (M1.MIN_COLLECTIVE_DEG, M1.MAX_COLLECTIVE_DEG)   # -10..65, on top of twist
    theta_1c_deg: tuple = (-10.0, 10.0)      # longitudinal cyclic (rigid disk: pitch moment)
    theta_1s_deg: tuple = (-10.0, 10.0)      # lateral cyclic
    elevator_deg: tuple = (-25.0, 25.0)
    rudder_deg: tuple = (-20.0, 20.0)
    nacelle_deg: tuple = (0.0, 95.0)
    nacelle_rate_deg_s: float = 8.0
    rpm: tuple = (M1.MIN_RPM, M1.MAX_RPM)
    max_tip_mach: float = M1.MAX_TIP_MACH
    max_stall_fraction: float = M1.MAX_STALL_FRACTION
    min_power_margin_frac: float = M1.MIN_POWER_MARGIN_FRAC
    max_reverse_flow_fraction: float = 0.03  # reverse-flow AREA / swept annulus (~mu = 0.35-0.4)
    attitude_deg: tuple = (-20.0, 25.0)      # pitch attitude bounds used by trim
    roll_deg: tuple = (-30.0, 30.0)


@dataclass
class ControlMixing:
    """Pilot-stick to effector mixing (report Sections 1.3 / 2.2 / 5.5).
    Normalized pilot inputs delta_lon, delta_lat, delta_ped are in [-1, 1].
    Rotor-control gains are phased out with sin^2(i_n) (full in helicopter
    mode, zero in airplane mode, V-22 style); the aerodynamic surfaces are
    always connected and become effective with dynamic pressure.
        pitch : theta1c (both rotors) = K_cyc  sin^2(i_n) delta_lon,  delta_e = K_e delta_lon
        roll  : differential collective = K_dcol sin^2(i_n) delta_lat, delta_a = K_a delta_lat
        yaw   : differential theta1s    = K_dcyc sin^2(i_n) delta_ped, delta_r = K_r delta_ped
    Yaw uses DIFFERENTIAL LATERAL cyclic: on the rigid disk the airload
    responds in phase with pitch, so theta1s (max pitch on the advancing side)
    changes the rotor H-force; opposite H-forces at the +/-9.4 m hubs give the
    yawing moment (a flapping rotor does this with differential longitudinal
    TPP tilt). Applied as theta1s_right = -dcyc, theta1s_left = +dcyc (each in
    its own azimuth).
    Signs are chosen so each pair of effectors acts in the same sense
    (+delta_lon nose-down, +delta_lat roll-left, +delta_ped nose-left)."""
    K_cyc_deg: float = 10.0
    K_e_deg: float = 25.0
    K_dcol_deg: float = 3.0
    K_a_deg: float = 20.0
    K_dcyc_deg: float = 8.0
    K_r_deg: float = 20.0
    phase_out_exponent: float = 2.0

    def effectors(self, nacelle_deg: float, d_lon: float, d_lat: float, d_ped: float) -> dict:
        g = np.sin(np.radians(nacelle_deg)) ** self.phase_out_exponent
        return dict(
            theta1c_deg=self.K_cyc_deg * g * d_lon,
            elevator_deg=self.K_e_deg * d_lon,
            dcoll_deg=self.K_dcol_deg * g * d_lat,
            aileron_deg=self.K_a_deg * d_lat,
            dcyc_deg=self.K_dcyc_deg * g * d_ped,
            rudder_deg=self.K_r_deg * d_ped,
        )


@dataclass
class AircraftGeometryM2:
    W_MTOW_N: float
    flat_plate_area_m2: float

    wing: WingGeometry
    htail: TailGeometry

    # Legacy fixed offsets FROM THE CG [x, y, z] body, used by the 3-DOF trim
    # in trim_solver.py. New code should use hub_from_cg() / *_from_cg().
    cg_to_wing_ac_m: np.ndarray
    cg_to_htail_ac_m: np.ndarray
    cg_to_right_rotor_m: np.ndarray
    cg_to_left_rotor_m: np.ndarray

    vtail: Optional[VTailGeometry] = None
    # Positions from the REFERENCE POINT (wing root quarter chord), body axes.
    wing_ac_ref_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    htail_ac_ref_m: np.ndarray = field(default_factory=lambda: np.array([-6.3, 0.0, -0.5]))
    vtail_ac_ref_m: np.ndarray = field(default_factory=lambda: np.array([-6.6, 0.0, -1.6]))
    nacelle_pivot_ref_m: np.ndarray = field(default_factory=lambda: np.array([0.0, 9.4, -0.5]))
    mast_length_m: float = 1.6   # pivot -> hub along the shaft axis
    mass_items: List[MassItem] = field(default_factory=list)
    limits: ControlLimits = field(default_factory=ControlLimits)
    mixing: ControlMixing = field(default_factory=ControlMixing)

    # ---------------- mass properties ----------------
    @staticmethod
    def _item_mass(it: MassItem, fuel_kg, payload_kg) -> float:
        if it.category == 'fuel' and fuel_kg is not None:
            return fuel_kg
        if it.category == 'payload' and payload_kg is not None:
            return payload_kg
        return it.mass_kg

    def mass_kg(self, fuel_kg: Optional[float] = None, payload_kg: Optional[float] = None) -> float:
        return sum(self._item_mass(it, fuel_kg, payload_kg) for it in self.mass_items)

    @staticmethod
    def shaft_axis_body(nacelle_deg: float) -> np.ndarray:
        """Unit vector of +Z_shaft (thrust direction) in body axes
        (third column of frames.R_shaft_to_body)."""
        n = np.radians(nacelle_deg)
        return np.array([np.cos(n), 0.0, -np.sin(n)])

    def hub_ref_m(self, nacelle_deg: float, side: str = 'right') -> np.ndarray:
        p = np.array(self.nacelle_pivot_ref_m, dtype=float)
        if side == 'left':
            p[1] = -p[1]
        return p + self.mast_length_m * self.shaft_axis_body(nacelle_deg)

    def _item_position(self, it: MassItem, nacelle_deg: float) -> np.ndarray:
        if it.tilts_with_nacelle:
            p = np.array(self.nacelle_pivot_ref_m, dtype=float)
            p[1] = 0.0                      # items listed per left/right pair: y cancels
            return p + it.shaft_offset_m * self.shaft_axis_body(nacelle_deg)
        return np.array([it.x_m, 0.0, it.z_m])

    def cg_ref_m(self, nacelle_deg: float = 90.0, fuel_kg=None, payload_kg=None) -> np.ndarray:
        """CG position from the reference point (symmetric aircraft -> y = 0)."""
        m_tot, mom = 0.0, np.zeros(3)
        for it in self.mass_items:
            m = self._item_mass(it, fuel_kg, payload_kg)
            m_tot += m
            mom += m * self._item_position(it, nacelle_deg)
        return mom / m_tot

    def hub_from_cg(self, nacelle_deg: float, side: str = 'right', **mass_kw) -> np.ndarray:
        return self.hub_ref_m(nacelle_deg, side) - self.cg_ref_m(nacelle_deg, **mass_kw)

    def wing_ac_from_cg(self, nacelle_deg: float, **mass_kw) -> np.ndarray:
        return np.asarray(self.wing_ac_ref_m, float) - self.cg_ref_m(nacelle_deg, **mass_kw)

    def htail_ac_from_cg(self, nacelle_deg: float, **mass_kw) -> np.ndarray:
        return np.asarray(self.htail_ac_ref_m, float) - self.cg_ref_m(nacelle_deg, **mass_kw)

    def vtail_ac_from_cg(self, nacelle_deg: float, **mass_kw) -> np.ndarray:
        return np.asarray(self.vtail_ac_ref_m, float) - self.cg_ref_m(nacelle_deg, **mass_kw)


# ============================================================
# SECTION 6 -- MASS BREAKDOWN  (empty 4530 + payload 1200 + fuel 1470 = 7200 kg)
# ============================================================
# Empty-mass split follows typical tiltrotor group-weight fractions
# (wing ~12 %, fuselage ~20 %, rotors ~12 %, propulsion ~13 %, drive ~11 %).
# x/z are from the reference point (wing root quarter chord), +fwd / +down.
# Tilting items are listed for BOTH nacelles together. The engines sit in the
# tilting nacelles (Milestone 1 layout); 2 x CT7-8A = 490 kg (+30 kg on the
# previous 460 kg allowance), taken from the fuel to keep MTOW = 7200 kg.
ENGINE_PAIR_MASS_KG = 2 * ENGINE_DRY_MASS_KG
FUEL_CAPACITY_KG = M1.FUEL_MASS_KG - (ENGINE_PAIR_MASS_KG - 460.0)


def default_mass_items() -> List[MassItem]:
    return [
        MassItem("Wing structure",                520.0, -0.35,  0.00),
        MassItem("Fuselage structure",            900.0, -0.60,  1.10),
        MassItem("Empennage (H + V tail)",        160.0, -6.40, -0.60),
        MassItem("Landing gear",                  240.0, -0.40,  2.00),
        MassItem("Engines (2 x CT7-8A, tilting)", ENGINE_PAIR_MASS_KG, 0.0, 0.0, True, -0.3),
        MassItem("Nacelle structure (fixed)",     120.0, -0.40, -0.20),
        MassItem("Proprotor gearboxes (tilting)", 240.0,  0.0,   0.0, True, 0.6),
        MassItem("Proprotors, blades + hubs",     520.0,  0.0,   0.0, True, 1.6),
        MassItem("Interconnect drive system",     240.0, -0.30, -0.20),
        MassItem("Flight controls",               180.0, -1.00,  1.00),
        MassItem("Avionics, electrical, systems", 400.0,  2.50,  1.00),
        MassItem("Crew + furnishings",            520.0,  2.00,  1.00),
        MassItem("Payload (10 pax)",      M1.PAYLOAD_KG,    -0.50,  1.20, category='payload'),
        MassItem("Fuel (wing tanks)",     FUEL_CAPACITY_KG, -0.30,  0.10, category='fuel'),
    ]


# ============================================================
# SECTION 7 -- DESIGN CHANGES FROM MILESTONE 1 (report Section 5.2)
# ============================================================
DESIGN_CHANGES = [
    ("Engines: 1.88 MW-class turboshaft (assumed) -> 2 x GE CT7-8A, 1893 kW take-off (selected)",
     "Engine-sizing study with the 6-DOF trimmed aircraft: hover OGE 1500 m ISA+15 margin +35 %, OEI at the "
     "corridor minimum-power point +61 %, OEI 85 m/s cruise +21 % (lightest candidate meeting OEI cruise); "
     "see m2_5_engine_selection.md"),
    ("Airplane-mode RPM: 250 -> scheduled 250 / 350 / 420 RPM",
     "250 RPM kept for the 74.3 m/s / 7000 m cruise (least power) but the collective reaches its 65 deg limit "
     "between 80 and 85 m/s; 350 RPM up to 100 m/s, 420 RPM for the high-speed dash"),
    ("Blade twist 25 deg root, -45 deg/R -> 12 deg root, -30 deg/R ('refined')",
     "M1 blade stalls inboard in hover at MTOW / 2000 m (12 % of loaded disk, stall margin -2.5 deg); refined "
     "blade: 0 %, +2.0 deg, hover power -4 %; see m2_9_comparison.md"),
    ("Explicit mass breakdown, tilting nacelle mass (engines, gearboxes, rotors), CG(i_n)",
     "Trim needs component locations; the CG moves forward as the nacelles tilt down"),
    ("Wing, horizontal and vertical tail with locations and control surfaces",
     "Required for transition trim (M1 used a flat-plate drag area and wing induced drag only)"),
    ("Fuel capacity 1500 -> 1470 kg",
     "Selected engines are 30 kg heavier than the empty-mass allowance; MTOW kept at 7200 kg"),
    ("Rotor model: axisymmetric BEMT -> azimuth-resolved BEMT with cyclic, annular Glauert + tip loss + K-factor",
     "Edgewise and conversion flight; reduces exactly to the M1 solver at mu = 0"),
    ("Stall model stated as implemented: linear Cl with stall flag and Cl clip",
     "The M1 report described a Viterna-Corrigan post-stall model; the code (M1 and M2) uses the linear model"),
]


# ============================================================
# SECTION 8 -- DEFAULT AIRCRAFT
# ============================================================
def get_default_aircraft(gross_mass_kg: Optional[float] = None,
                         nacelle_deg_for_legacy: float = 90.0) -> AircraftGeometryM2:
    """Default design. `gross_mass_kg` (if given) overrides the weight used by
    trim (e.g. the current mission mass); the CG uses the full mass breakdown."""
    ac = AircraftGeometryM2(
        W_MTOW_N=(gross_mass_kg if gross_mass_kg is not None else GROSS_MASS_KG) * G,
        flat_plate_area_m2=M1.FLAT_PLATE_AREA_M2,
        wing=WingGeometry(S_m2=39.24, AR=9.0, e_oswald=0.8, i_w_deg=4.0),
        htail=TailGeometry(S_m2=8.0, AR=4.5, e_oswald=0.8, i_t_deg=0.0),
        vtail=VTailGeometry(S_m2=6.0, AR=1.5),
        cg_to_wing_ac_m=np.zeros(3),
        cg_to_htail_ac_m=np.zeros(3),
        cg_to_right_rotor_m=np.zeros(3),
        cg_to_left_rotor_m=np.zeros(3),
        mass_items=default_mass_items(),
    )
    # Legacy fixed offsets (3-DOF trim): evaluated at one nacelle angle.
    n = nacelle_deg_for_legacy
    ac.cg_to_wing_ac_m = ac.wing_ac_from_cg(n)
    ac.cg_to_htail_ac_m = ac.htail_ac_from_cg(n)
    ac.cg_to_right_rotor_m = ac.hub_from_cg(n, 'right')
    ac.cg_to_left_rotor_m = ac.hub_from_cg(n, 'left')
    return ac


def atmosphere(altitude_m: float = REFERENCE_ALTITUDE_M, dISA_K: float = 0.0):
    from environment import isa
    return isa(altitude_m, dISA_K)


if __name__ == "__main__":
    ac = get_default_aircraft()
    print(f"Mass: {ac.mass_kg():.0f} kg   (M1 gross mass {GROSS_MASS_KG:.0f} kg)")
    for n in (90.0, 45.0, 0.0):
        cg = ac.cg_ref_m(n)
        print(f"i_n={n:4.0f}: CG from ref = [{cg[0]:+.3f}, {cg[1]:+.3f}, {cg[2]:+.3f}] m,  "
              f"right hub from CG = {np.round(ac.hub_from_cg(n), 3)},  "
              f"wing ac from CG = {np.round(ac.wing_ac_from_cg(n), 3)}")
    print(f"Wing: span {ac.wing.span_m:.2f} m, chord {ac.wing.chord_m:.2f} m, CL_max {ac.wing.CL_max:.2f}")
