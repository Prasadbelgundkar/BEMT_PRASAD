"""
environment.py
----------------
Simple International Standard Atmosphere (ISA) model with a user-selectable
temperature offset (dISA), valid through the troposphere and lower
stratosphere (0 - 20 km). This is the single source of truth for density,
temperature, pressure, and speed of sound used everywhere else in the tool.

Assumptions (document these in your report, Section 1.2):
    - Dry air, ideal gas law.
    - Standard ISA lapse rate of -6.5 K/km up to 11 km, isothermal 11-20 km.
    - A constant additive temperature offset (dISA, in K) is applied at all
      altitudes to represent hot-day / cold-day operation.
    - Gravity is treated as constant (9.80665 m/s^2); no altitude variation.
"""

import math
from dataclasses import dataclass

# Physical constants
R_AIR = 287.05287        # J/(kg K), specific gas constant for dry air
GAMMA = 1.4               # ratio of specific heats
G0 = 9.80665               # m/s^2
T0 = 288.15                # K, sea level standard temperature
P0 = 101325.0               # Pa, sea level standard pressure
RHO0 = 1.225                # kg/m^3, sea level standard density
LAPSE_RATE = -0.0065         # K/m, troposphere lapse rate
TROPOPAUSE_ALT = 11000.0      # m
STRATOSPHERE_TOP = 20000.0     # m (isothermal layer used here)


@dataclass
class AtmoState:
    altitude_m: float
    dISA_K: float
    temperature_K: float
    pressure_Pa: float
    density_kg_m3: float
    speed_of_sound_mps: float
    dynamic_viscosity_Pa_s: float


def sutherland_viscosity(T_K: float) -> float:
    """Sutherland's law for dynamic viscosity of air."""
    mu0 = 1.716e-5   # Pa.s at T0=273.15 K
    T0_suth = 273.15
    S = 110.4
    return mu0 * (T_K / T0_suth) ** 1.5 * (T0_suth + S) / (T_K + S)


def isa(altitude_m: float, dISA_K: float = 0.0) -> AtmoState:
    """
    Return atmospheric state at a given geopotential altitude, with a
    uniform ISA temperature offset applied.

    Parameters
    ----------
    altitude_m : float
        Geopotential altitude in meters (0 <= h <= 20000 supported).
    dISA_K : float
        Uniform temperature offset in Kelvin (e.g. +20 for a hot day).
    """
    if altitude_m < 0:
        raise ValueError("Altitude must be >= 0 m in this model.")
    if altitude_m > STRATOSPHERE_TOP:
        raise ValueError("Model only valid up to 20,000 m.")

    if altitude_m <= TROPOPAUSE_ALT:
        T_std = T0 + LAPSE_RATE * altitude_m
        P = P0 * (T_std / T0) ** (-G0 / (LAPSE_RATE * R_AIR))
    else:
        T11 = T0 + LAPSE_RATE * TROPOPAUSE_ALT
        P11 = P0 * (T11 / T0) ** (-G0 / (LAPSE_RATE * R_AIR))
        T_std = T11
        P = P11 * math.exp(
            -G0 * (altitude_m - TROPOPAUSE_ALT) / (R_AIR * T11)
        )

    T = T_std + dISA_K  # apply offset to actual (not used for pressure calc,
                         # consistent with common preliminary-design practice)
    rho = P / (R_AIR * T)
    a = (GAMMA * R_AIR * T) ** 0.5
    mu = sutherland_viscosity(T)

    return AtmoState(
        altitude_m=altitude_m,
        dISA_K=dISA_K,
        temperature_K=T,
        pressure_Pa=P,
        density_kg_m3=rho,
        speed_of_sound_mps=a,
        dynamic_viscosity_Pa_s=mu,
    )


if __name__ == "__main__":
    for h in [0, 1000, 3000, 5000]:
        s = isa(h, dISA_K=15.0)
        print(f"h={h:5.0f} m  T={s.temperature_K:6.2f} K  "
              f"rho={s.density_kg_m3:.4f} kg/m3  a={s.speed_of_sound_mps:.2f} m/s")
