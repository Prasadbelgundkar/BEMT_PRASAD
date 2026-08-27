"""
airfoil.py
----------
Airfoil lift/drag models.

Two model types are supported:

1. LinearAirfoil - the closed-form linear model given in the assignment
   handout (Knight & Hefner validation rotor):
        Cl = a0 * alpha
        Cd = Cd_min + eps * alpha^2
   with alpha in RADIANS. This model has no built-in stall behaviour, so a
   separate `stall_alpha` cutoff is used purely for FLAGGING stalled
   sections (Task 2) -- the linear Cl/Cd values are still returned so the
   solver stays well-behaved, but `is_stalled` tells you where the model
   is no longer physically trustworthy. Document this limitation in
   Section 3.4 of your report.

2. TableAirfoil - a generic angle-of-attack lookup table (e.g. digitized
   XFOIL/experimental polar), linearly interpolated, with the last
   angle-of-attack in the table treated as the stall boundary. Use this
   for your own tiltrotor design airfoil(s) in Task 5 if you have polar
   data (recommended over the linear model once alpha gets large, e.g. in
   axial/propeller mode where blade AoA can vary a lot).

Both models share a common interface: get_coeffs(alpha_rad) -> (Cl, Cd, stalled)
"""

from dataclasses import dataclass, field
from typing import Sequence, Tuple
import numpy as np


@dataclass
class LinearAirfoil:
    a0: float = 5.75          # lift-curve slope, per radian
    Cd_min: float = 0.0113
    eps: float = 1.25         # quadratic drag coefficient
    stall_alpha_rad: float = np.radians(12.0)  # adopted stall criterion
    Cl_max_clip: float = 1.4  # physically sane ceiling once "stalled"

    def get_coeffs(self, alpha_rad: float) -> Tuple[float, float, bool]:
        stalled = abs(alpha_rad) >= self.stall_alpha_rad
        Cl = self.a0 * alpha_rad
        Cd = self.Cd_min + self.eps * alpha_rad ** 2
        if stalled:
            # Post-stall behaviour is not defined by the linear model.
            # Clip Cl to avoid the solver chasing a nonphysical value and
            # inflate Cd modestly to represent separated flow. This is a
            # simple engineering fix -- justify / replace it in your report.
            Cl = np.sign(alpha_rad) * min(abs(Cl), self.Cl_max_clip)
            Cd = max(Cd, 0.05)
        return Cl, Cd, stalled


@dataclass
class TableAirfoil:
    """Linear interpolation over a user-supplied polar table."""
    alpha_deg: Sequence[float]
    Cl: Sequence[float]
    Cd: Sequence[float]
    name: str = "custom_airfoil"

    def __post_init__(self):
        self.alpha_rad_arr = np.radians(np.asarray(self.alpha_deg, dtype=float))
        self.Cl_arr = np.asarray(self.Cl, dtype=float)
        self.Cd_arr = np.asarray(self.Cd, dtype=float)
        order = np.argsort(self.alpha_rad_arr)
        self.alpha_rad_arr = self.alpha_rad_arr[order]
        self.Cl_arr = self.Cl_arr[order]
        self.Cd_arr = self.Cd_arr[order]

    def get_coeffs(self, alpha_rad: float) -> Tuple[float, float, bool]:
        lo, hi = self.alpha_rad_arr[0], self.alpha_rad_arr[-1]
        stalled = alpha_rad <= lo or alpha_rad >= hi
        a_clip = min(max(alpha_rad, lo), hi)
        Cl = float(np.interp(a_clip, self.alpha_rad_arr, self.Cl_arr))
        Cd = float(np.interp(a_clip, self.alpha_rad_arr, self.Cd_arr))
        return Cl, Cd, stalled

    @classmethod
    def from_csv(cls, path: str, name: str = "custom_airfoil"):
        import csv
        alpha, cl, cd = [], [], []
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                alpha.append(float(row["alpha_deg"]))
                cl.append(float(row["Cl"]))
                cd.append(float(row["Cd"]))
        return cls(alpha_deg=alpha, Cl=cl, Cd=cd, name=name)


def prandtl_glauert_correct(Cl: float, Cd: float, mach: float,
                             mach_limit: float = 0.7) -> Tuple[float, float]:
    """
    Apply Prandtl-Glauert compressibility correction to both lift and drag
    coefficients.  The same factor 1/sqrt(1-M^2) applies to both Cl and Cd
    per standard subsonic compressibility theory (see readme_formula.md).
    Correction is frozen above mach_limit since P-G becomes invalid near/above
    M=0.7 -- flag high-Mach sections separately using tip Mach checks in the
    mission planner instead.

    Returns
    -------
    Cl_corrected, Cd_corrected : float, float
    """
    m = min(mach, mach_limit)
    beta = max((1.0 - m ** 2) ** 0.5, 1e-3)
    return Cl / beta, Cd / beta


class BlendedLinearAirfoilProvider:
    """
    Reads a CSV defining LinearAirfoil properties at different radial stations (r/R)
    and interpolates the aerodynamic parameters for any given station.
    
    The CSV must have the following headers:
    r_over_R, a0, Cd_min, eps, Cl_max_clip, stall_alpha_deg
    """
    def __init__(self, stations: Sequence[float], airfoils: Sequence[LinearAirfoil]):
        self.stations = np.array(stations, dtype=float)
        self.airfoils = list(airfoils)
        
        # Ensure they are sorted from root to tip
        order = np.argsort(self.stations)
        self.stations = self.stations[order]
        self.airfoils = [self.airfoils[i] for i in order]

    @classmethod
    def from_csv(cls, path: str):
        import csv
        stations = []
        airfoils = []
        with open(path, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                stations.append(float(row["r_over_R"]))
                # Parse the linear airfoil properties
                a0 = float(row["a0"])
                Cd_min = float(row["Cd_min"])
                eps = float(row["eps"])
                Cl_max = float(row["Cl_max_clip"])
                # Fallback to 12.0 degrees if stall_alpha_deg isn't provided
                stall_deg = float(row.get("stall_alpha_deg", 12.0))
                
                airfoil = LinearAirfoil(
                    a0=a0,
                    Cd_min=Cd_min,
                    eps=eps,
                    stall_alpha_rad=np.radians(stall_deg),
                    Cl_max_clip=Cl_max
                )
                airfoils.append(airfoil)
        return cls(stations, airfoils)

    def __call__(self, x: float) -> LinearAirfoil:
        """
        x is the non-dimensional radial station (r/R).
        Returns a new LinearAirfoil with interpolated properties.
        """
        # Interpolate each aerodynamic parameter independently
        a0 = float(np.interp(x, self.stations, [a.a0 for a in self.airfoils]))
        Cd_min = float(np.interp(x, self.stations, [a.Cd_min for a in self.airfoils]))
        eps = float(np.interp(x, self.stations, [a.eps for a in self.airfoils]))
        stall = float(np.interp(x, self.stations, [a.stall_alpha_rad for a in self.airfoils]))
        Cl_max = float(np.interp(x, self.stations, [a.Cl_max_clip for a in self.airfoils]))
        
        return LinearAirfoil(
            a0=a0, 
            Cd_min=Cd_min, 
            eps=eps, 
            stall_alpha_rad=stall, 
            Cl_max_clip=Cl_max
        )
