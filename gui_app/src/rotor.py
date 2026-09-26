"""
rotor.py
--------
Rotor / blade geometry container. Nothing here is hard-coded to a single
aircraft -- every field is a constructor argument so the same class serves
the validation rotor, the design-variable study, and your final tiltrotor
design.

Radial distributions (chord, twist) are given as callables of nondimensional
radial station r/R so you can swap in linear, ideal-twist, or arbitrary
tabulated distributions without touching the solver.
"""

from dataclasses import dataclass, field
from typing import Callable, Optional
import numpy as np

_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


def linear_taper_chord(root_chord: float, taper_ratio: float) -> Callable[[float], float]:
    """chord(r/R) linearly varying from root_chord at r/R=root_cutout_frac to
    taper_ratio*root_chord at the tip. taper_ratio = tip_chord/root_chord."""
    def c(x):
        return root_chord * (1.0 - (1.0 - taper_ratio) * x)
    return c


def linear_twist(theta_root_rad: float, twist_rate_rad_per_R: float) -> Callable[[float], float]:
    """twist(r/R) = theta_root + twist_rate * (r/R). twist_rate negative for
    typical washout (nose-down at tip)."""
    def th(x):
        return theta_root_rad + twist_rate_rad_per_R * x
    return th


def constant_chord(chord: float) -> Callable[[float], float]:
    return lambda x: chord


def constant_twist(theta_rad: float) -> Callable[[float], float]:
    return lambda x: theta_rad


@dataclass
class Rotor:
    radius_m: float
    root_cutout_m: float
    num_blades: int
    chord_fn: Callable[[float], float]      # chord(r/R) -> m
    twist_fn: Callable[[float], float]      # twist(r/R) -> rad (built-in twist,
                                             # collective is added separately)
    airfoil_fn: Callable[[float], "object"] = None  # optional: airfoil(r/R) -> airfoil obj
                                                       # for radially-blended airfoils;
                                                       # if None, a single airfoil is passed
                                                       # to the solver directly.
    name: str = "rotor"

    def solidity(self, n_stations: int = 200) -> float:
        """Rotor (thrust-weighted-ish) solidity: sigma = B * mean_chord / (pi * R)."""
        x = np.linspace(self.root_cutout_m / self.radius_m, 1.0, n_stations)
        chords = np.array([self.chord_fn(xi) for xi in x])
        mean_chord = _trapz(chords, x) / (1.0 - self.root_cutout_m / self.radius_m)
        return self.num_blades * mean_chord / (np.pi * self.radius_m)

    def disk_area_m2(self) -> float:
        return np.pi * self.radius_m ** 2

    def tip_speed_mps(self, omega_rad_s: float) -> float:
        return omega_rad_s * self.radius_m

    def tip_mach(self, omega_rad_s: float, speed_of_sound_mps: float,
                 axial_velocity_mps: float = 0.0) -> float:
        """Resultant tip Mach number including axial/forward-flight component."""
        v_tip = self.tip_speed_mps(omega_rad_s)
        v_res = (v_tip ** 2 + axial_velocity_mps ** 2) ** 0.5
        return v_res / speed_of_sound_mps
