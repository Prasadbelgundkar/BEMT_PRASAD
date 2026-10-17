"""
tandem_interference.py

Aerodynamic interference model for tandem (overlapping) rotor configurations.
The forward rotor's contracted wake washes over part of the aft rotor disk,
increasing inflow and power.  Two sub-models are combined:

  1. Two-circle geometric overlap: fraction kappa of the aft disk inside the
     forward disk boundary, for arbitrary radii and lateral/longitudinal offsets.

  2. Coleman (1947) wake velocity profile: axial downwash transitions smoothly
     from v_i at the rotor disk to 2*v_i far downstream.

`solve_tandem_bemt` does a forward-then-aft two-pass solve; setting
`include_interference=False` recovers the uncoupled single-rotor result.
"""

from dataclasses import dataclass
from typing import Callable, Optional, Tuple
import numpy as np

from bemt import run_bemt, RotorPerformance
from rotor import Rotor


@dataclass
class TandemPerformance:
    """Consolidated performance for a dual-rotor tandem system."""
    fwd: RotorPerformance
    aft: RotorPerformance
    total_thrust_N: float
    total_power_W: float
    total_torque_Nm: float
    total_CT: float
    total_CP: float
    figure_of_merit_system: Optional[float]
    overlap_fraction: float
    delta_v_axial_mps: float
    include_interference: bool


def compute_overlap_fraction(
    R_fwd: float,
    R_aft: float,
    longitudinal_sep_m: float = 0.0,
    lateral_offset_m: float = 0.0,
    center_distance_m: Optional[float] = None,
) -> float:
    """
    Geometric disk overlap fraction kappa = A_overlap / (pi * R_aft^2), in [0, 1].

    Pass `center_distance_m` directly, or let the function compute it from the
    longitudinal and lateral components via hypot.
    """
    if R_fwd <= 0.0 or R_aft <= 0.0:
        raise ValueError(f"Rotor radii must be positive (got R_fwd={R_fwd}, R_aft={R_aft})")

    if center_distance_m is not None:
        d = abs(float(center_distance_m))
    else:
        d = float(np.hypot(longitudinal_sep_m, lateral_offset_m))

    # Disks separated or just touching
    if d >= (R_fwd + R_aft):
        return 0.0

    # One disk fully enclosed in the other
    if d <= abs(R_fwd - R_aft):
        if R_aft <= R_fwd:
            return 1.0
        else:
            # forward disk entirely inside aft: A_overlap = pi * R_fwd^2
            return float((R_fwd / R_aft) ** 2)

    # Partial overlap - standard two-circle lens area
    r1, r2 = R_fwd, R_aft
    cos_alpha = np.clip((d**2 + r1**2 - r2**2) / (2.0 * d * r1), -1.0, 1.0)
    cos_beta  = np.clip((d**2 + r2**2 - r1**2) / (2.0 * d * r2), -1.0, 1.0)

    alpha = np.arccos(cos_alpha)
    beta  = np.arccos(cos_beta)

    heron = max((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2), 0.0)
    A_overlap = r1**2 * alpha + r2**2 * beta - 0.5 * np.sqrt(heron)

    kappa = A_overlap / (np.pi * r2**2)
    return float(np.clip(kappa, 0.0, 1.0))


def coleman_inflow_augmentation(
    v_induced_fwd: float,
    kappa: float,
    z_sep_m: float,
    R_fwd: float,
) -> float:
    """
    Additional axial inflow on the aft rotor from the forward rotor's wake.

    Coleman (1947) vortex-cylinder profile:
        v_wake(z) = v_i * (1 + z / sqrt(R^2 + z^2))

    Returns kappa * v_wake(z_sep).  Returns 0 for zero overlap or zero inflow.
    """
    if kappa <= 0.0 or v_induced_fwd <= 0.0 or R_fwd <= 0.0:
        return 0.0

    z = max(float(z_sep_m), 0.0)
    v_wake = v_induced_fwd * (1.0 + z / np.sqrt(R_fwd**2 + z**2))
    return float(kappa * v_wake)


def solve_tandem_bemt(
    rotor_fwd: Rotor,
    rotor_aft: Rotor,
    airfoil_provider_fwd: Callable[[float], object],
    airfoil_provider_aft: Callable[[float], object],
    omega_fwd_rad_s: float,
    collective_fwd_rad: float,
    collective_aft_rad: float,
    rho: float,
    a_sound: float,
    omega_aft_rad_s: Optional[float] = None,
    longitudinal_sep_m: float = 0.0,
    lateral_offset_m: float = 0.0,
    vertical_stagger_m: float = 0.0,
    v_axial_freestream: float = 0.0,
    include_interference: bool = True,
    n_stations: int = 60,
    include_root_loss: bool = False,
) -> TandemPerformance:
    """
    Two-pass coupled BEMT solve for a tandem rotor system.

    Forward rotor runs in clean air first; its mean induced velocity drives
    the Coleman wake model; the aft rotor then runs with the augmented axial
    inflow.  Set `include_interference=False` to run both rotors uncoupled.
    """
    if omega_aft_rad_s is None:
        omega_aft_rad_s = omega_fwd_rad_s

    # Forward rotor - clean inflow
    perf_fwd = run_bemt(
        rotor=rotor_fwd,
        airfoil_provider=airfoil_provider_fwd,
        omega_rad_s=omega_fwd_rad_s,
        collective_rad=collective_fwd_rad,
        rho=rho,
        a_sound=a_sound,
        v_axial=v_axial_freestream,
        n_stations=n_stations,
        include_root_loss=include_root_loss,
    )

    kappa = compute_overlap_fraction(
        R_fwd=rotor_fwd.radius_m,
        R_aft=rotor_aft.radius_m,
        longitudinal_sep_m=longitudinal_sep_m,
        lateral_offset_m=lateral_offset_m,
    )

    if include_interference:
        delta_v_axial = coleman_inflow_augmentation(
            v_induced_fwd=perf_fwd.mean_induced_velocity(),
            kappa=kappa,
            z_sep_m=vertical_stagger_m,
            R_fwd=rotor_fwd.radius_m,
        )
    else:
        delta_v_axial = 0.0

    # Aft rotor - augmented inflow
    perf_aft = run_bemt(
        rotor=rotor_aft,
        airfoil_provider=airfoil_provider_aft,
        omega_rad_s=omega_aft_rad_s,
        collective_rad=collective_aft_rad,
        rho=rho,
        a_sound=a_sound,
        v_axial=v_axial_freestream + delta_v_axial,
        n_stations=n_stations,
        include_root_loss=include_root_loss,
    )

    total_T = perf_fwd.thrust_N + perf_aft.thrust_N
    total_P = perf_fwd.power_W  + perf_aft.power_W
    total_Q = perf_fwd.torque_Nm + perf_aft.torque_Nm

    A_total  = rotor_fwd.disk_area_m2() + rotor_aft.disk_area_m2()
    vtip_ref = 0.5 * (rotor_fwd.tip_speed_mps(omega_fwd_rad_s) + rotor_aft.tip_speed_mps(omega_aft_rad_s))

    total_CT = total_T / (rho * A_total * vtip_ref**2 + 1e-12)
    total_CP = total_P / (rho * A_total * vtip_ref**3 + 1e-12)

    figure_of_merit_system = None
    if abs(v_axial_freestream) < 1e-5 and total_CT > 0 and total_CP > 0:
        figure_of_merit_system = total_CT**1.5 / (np.sqrt(2.0) * total_CP)

    return TandemPerformance(
        fwd=perf_fwd,
        aft=perf_aft,
        total_thrust_N=total_T,
        total_power_W=total_P,
        total_torque_Nm=total_Q,
        total_CT=total_CT,
        total_CP=total_CP,
        figure_of_merit_system=figure_of_merit_system,
        overlap_fraction=kappa,
        delta_v_axial_mps=delta_v_axial,
        include_interference=include_interference,
    )


def trim_tandem_hover(
    rotor_fwd: Rotor,
    rotor_aft: Rotor,
    airfoil_provider_fwd: Callable[[float], object],
    airfoil_provider_aft: Callable[[float], object],
    omega_fwd_rad_s: float,
    target_thrust_total_N: float,
    rho: float,
    a_sound: float,
    thrust_split_fwd: float = 0.5,
    omega_aft_rad_s: Optional[float] = None,
    longitudinal_sep_m: float = 0.0,
    lateral_offset_m: float = 0.0,
    vertical_stagger_m: float = 0.0,
    include_interference: bool = True,
    coll_range_deg: Tuple[float, float] = (-5.0, 25.0),
) -> Optional[Tuple[float, float, TandemPerformance]]:
    """
    Trim both rotors to a target total thrust and longitudinal split.

    Returns (coll_fwd_deg, coll_aft_deg, TandemPerformance), or None if
    the trim is outside the collective range.
    """
    from scipy.optimize import brentq
    from bemt import trim_hover_collective

    if omega_aft_rad_s is None:
        omega_aft_rad_s = omega_fwd_rad_s

    target_T_fwd = target_thrust_total_N * thrust_split_fwd
    target_T_aft = target_thrust_total_N * (1.0 - thrust_split_fwd)

    # Forward rotor collective - unaffected by aft wake
    coll_fwd_deg = trim_hover_collective(
        rotor=rotor_fwd,
        airfoil_provider=airfoil_provider_fwd,
        omega_rad_s=omega_fwd_rad_s,
        target_thrust_N=target_T_fwd,
        rho=rho,
        a_sound=a_sound,
        coll_range_deg=coll_range_deg,
    )
    if coll_fwd_deg is None:
        return None

    perf_fwd = run_bemt(
        rotor=rotor_fwd,
        airfoil_provider=airfoil_provider_fwd,
        omega_rad_s=omega_fwd_rad_s,
        collective_rad=np.radians(coll_fwd_deg),
        rho=rho,
        a_sound=a_sound,
        v_axial=0.0,
    )

    kappa = compute_overlap_fraction(
        R_fwd=rotor_fwd.radius_m,
        R_aft=rotor_aft.radius_m,
        longitudinal_sep_m=longitudinal_sep_m,
        lateral_offset_m=lateral_offset_m,
    )
    if include_interference:
        delta_v = coleman_inflow_augmentation(
            v_induced_fwd=perf_fwd.mean_induced_velocity(),
            kappa=kappa,
            z_sep_m=vertical_stagger_m,
            R_fwd=rotor_fwd.radius_m,
        )
    else:
        delta_v = 0.0

    def aft_residual(coll_deg: float) -> float:
        p = run_bemt(
            rotor=rotor_aft,
            airfoil_provider=airfoil_provider_aft,
            omega_rad_s=omega_aft_rad_s,
            collective_rad=np.radians(coll_deg),
            rho=rho,
            a_sound=a_sound,
            v_axial=delta_v,
        )
        return p.thrust_N - target_T_aft

    f_lo = aft_residual(coll_range_deg[0])
    f_hi = aft_residual(coll_range_deg[1])
    if not (np.isfinite(f_lo) and np.isfinite(f_hi)) or (f_lo * f_hi > 0):
        return None

    try:
        coll_aft_deg = brentq(aft_residual, coll_range_deg[0], coll_range_deg[1], xtol=0.01, maxiter=60)
    except ValueError:
        return None

    perf_tandem = solve_tandem_bemt(
        rotor_fwd=rotor_fwd,
        rotor_aft=rotor_aft,
        airfoil_provider_fwd=airfoil_provider_fwd,
        airfoil_provider_aft=airfoil_provider_aft,
        omega_fwd_rad_s=omega_fwd_rad_s,
        collective_fwd_rad=np.radians(coll_fwd_deg),
        collective_aft_rad=np.radians(coll_aft_deg),
        rho=rho,
        a_sound=a_sound,
        omega_aft_rad_s=omega_aft_rad_s,
        longitudinal_sep_m=longitudinal_sep_m,
        lateral_offset_m=lateral_offset_m,
        vertical_stagger_m=vertical_stagger_m,
        v_axial_freestream=0.0,
        include_interference=include_interference,
    )

    return coll_fwd_deg, coll_aft_deg, perf_tandem
