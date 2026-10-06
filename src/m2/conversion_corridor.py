import numpy as np
from typing import Callable, Tuple, List, Dict
from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor

from rotor import Rotor
from m2.aircraft_input_m2 import AircraftGeometryM2
from m2.trim_solver import trim_aircraft_detailed, compute_aircraft_residual, shaft_angle_of_attack
from m2.edgewise_bemt import run_edgewise_bemt

@dataclass
class TrimPointResult:
    V_inf: float
    nacelle_deg: float
    success: bool
    alpha_deg: float
    collective_deg: float
    ctrl_deg: float
    power_kW: float
    wing_stalled: bool
    rotor_stalled_fraction: float
    message: str

def evaluate_trim_point(
    V_inf: float, 
    nacelle_deg: float, 
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    omega_rad_s: float,
    x0: np.ndarray,
    trim_pitch_with: str
) -> TrimPointResult:
    """
    Attempts to trim the aircraft at a specific point in the corridor.
    """
    try:
        x_trim, status, res_norm = trim_aircraft_detailed(
            V_inf=V_inf,
            gamma_rad=0.0,
            nacelle_deg=nacelle_deg,
            omega_rad_s=omega_rad_s,
            aircraft=aircraft,
            rotor=rotor,
            airfoil_provider=airfoil_provider,
            rho=rho,
            a_sound=a_sound,
            x0=x0,
            trim_pitch_with=trim_pitch_with,
            verbose=False
        )
        if status == 'control_saturation':
            x = np.degrees(x_trim)
            return TrimPointResult(V_inf, nacelle_deg, False, x[0], x[1], x[2], 0, False, 0.0,
                                   "Control saturation (trim root outside control bounds)")
        if status != 'ok':
            return TrimPointResult(V_inf, nacelle_deg, False, 0, 0, 0, 0, False, 0.0,
                                   f"No trim solution (best residual norm {res_norm:.3g})")

        alpha_rad = x_trim[0]
        alpha_wing = alpha_rad + np.radians(aircraft.wing.i_w_deg)
        wing_stalled = abs(alpha_wing) > np.radians(15.0)
        
        # Run BEMT one more time to get power and stall fraction
        alpha_shaft_rad = shaft_angle_of_attack(alpha_rad, nacelle_deg)
        rotor_result = run_edgewise_bemt(
            rotor=rotor,
            airfoil_provider=airfoil_provider,
            V_inf=V_inf,
            omega_rad_s=omega_rad_s,
            theta0_rad=x_trim[1],
            theta1c_rad=x_trim[2] if trim_pitch_with == 'cyclic' else 0.0,
            theta1s_rad=0.0,
            alpha_shaft_rad=alpha_shaft_rad,
            nacelle_angle_deg=nacelle_deg,
            rho=rho,
            a_sound=a_sound,
            n_r=30,
            n_psi=36
        )
        
        total_power_kW = 2.0 * rotor_result.power_W / 1000.0
        
        return TrimPointResult(
            V_inf=V_inf,
            nacelle_deg=nacelle_deg,
            success=True,
            alpha_deg=np.degrees(alpha_rad),
            collective_deg=np.degrees(x_trim[1]),
            ctrl_deg=np.degrees(x_trim[2]),
            power_kW=total_power_kW,
            wing_stalled=wing_stalled,
            rotor_stalled_fraction=rotor_result.stalled_fraction,
            message="OK"
        )
        
    except Exception as e:
        return TrimPointResult(V_inf, nacelle_deg, False, 0, 0, 0, 0, False, 0.0, str(e))

def compute_conversion_corridor(
    V_sweep: np.ndarray,
    nacelle_sweep: np.ndarray,
    aircraft: AircraftGeometryM2,
    rotor: Rotor,
    airfoil_provider: Callable,
    rho: float,
    a_sound: float,
    omega_rad_s: float
) -> List[TrimPointResult]:
    
    results = []
    
    # Simple nested loop. (Could be parallelized, but BEMT is fast enough for a coarse grid)
    # We use the previous airspeed's trim state as the seed for the next one to speed up convergence
    
    for nacelle in nacelle_sweep:
        print(f"Sweeping Nacelle = {nacelle} deg...")
        
        # Reset seed for each nacelle angle
        x0 = np.array([np.radians(5.0), np.radians(15.0), 0.0])
        
        # Decide control effector
        # Use cyclic at high nacelle (helicopter), elevator at low nacelle (airplane)
        trim_pitch_with = 'cyclic' if nacelle > 45.0 else 'elevator'
        
        for V in V_sweep:
            # Skip very low speeds if we are in airplane mode (can't fly at 5 m/s with nacelle 0)
            # This just saves computation time on points we know will fail
            if nacelle < 30.0 and V < 20.0:
                results.append(TrimPointResult(V, nacelle, False, 0, 0, 0, 0, True, 1.0, "Too slow for nacelle angle"))
                continue
                
            res = evaluate_trim_point(V, nacelle, aircraft, rotor, airfoil_provider, rho, a_sound, omega_rad_s, x0, trim_pitch_with)
            results.append(res)
            
            if res.success:
                x0 = np.array([np.radians(res.alpha_deg), np.radians(res.collective_deg), np.radians(res.ctrl_deg)])
                
    return results


# ===========================================================================
# 6-DOF conversion corridor (report Section 7) -- uses trim_6dof
# ===========================================================================
# Primary-cause categories, in plotting order. A converged point with several
# limit flags gets the first flag in FLAG_PRIORITY as its primary cause.
CATEGORIES = ["feasible", "power", "wing_stall", "rotor_stall", "tip_mach", "reverse_flow",
              "control_saturation", "excessive_residual", "no_physical_solution"]
FLAG_PRIORITY = ["power", "wing_stall", "rotor_stall", "tip_mach", "reverse_flow"]
FIELDS = ["status_code", "category", "feasible", "residual", "theta_deg", "alpha_deg", "collective_deg",
          "theta1c_deg", "elevator_deg", "d_lon", "P_req_kW", "P_avail_kW", "power_margin",
          "stall_frac", "stall_margin_deg", "reverse_frac", "tip_mach", "alpha_wing_deg",
          "rotor_lift_frac", "control_margin"]
STATUS_CODES = {"ok": 0, "ok_at_limit": 1, "control_saturation": 2, "excessive_residual": 3,
                "no_physical_solution": 4}


def primary_category(tr) -> str:
    if tr.feasible:
        return "feasible"
    if tr.status in ("ok", "ok_at_limit"):
        for f in FLAG_PRIORITY:
            if f in tr.flags:
                return f
        return "feasible"
    return tr.status


def _row_task(args):
    """Trim one nacelle row with continuation in airspeed (runs in a worker)."""
    nacelle_deg, V_list, rpm, altitude_m, gross_mass_kg = args
    import m2.aircraft_input_m2 as CFG
    from m2.trim_6dof import trim_6dof, make_condition
    ac = CFG.get_default_aircraft(gross_mass_kg)
    lim = ac.limits
    out = []
    x0 = None
    for V in V_list:
        cond = make_condition(V, nacelle_deg, rpm=rpm, altitude_m=altitude_m)
        tr = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond, x0=x0)
        if tr.status in ("ok", "ok_at_limit"):
            x0 = tr.x
        conv = tr.status in ("ok", "ok_at_limit")
        W = ac.W_MTOW_N
        nan = np.nan
        coll_lo, coll_hi = lim.collective_deg
        ctrl_margin = nan
        if conv:
            stick = max(abs(tr.d_lon), abs(tr.d_lat), abs(tr.d_ped))
            coll_m = min(tr.collective_deg - coll_lo, coll_hi - tr.collective_deg) / (coll_hi - coll_lo)
            ctrl_margin = min(1.0 - stick, 2.0 * coll_m)
        out.append(dict(
            status_code=STATUS_CODES[tr.status], category=CATEGORIES.index(primary_category(tr)),
            feasible=float(tr.feasible), residual=tr.residual_norm,
            theta_deg=tr.theta_deg if conv else nan, alpha_deg=tr.alpha_deg if conv else nan,
            collective_deg=tr.collective_deg if conv else nan, theta1c_deg=tr.theta1c_deg if conv else nan,
            elevator_deg=tr.elevator_deg if conv else nan, d_lon=tr.d_lon if conv else nan,
            P_req_kW=tr.P_req_W / 1e3 if conv else nan, P_avail_kW=tr.P_avail_W / 1e3,
            power_margin=tr.power_margin_frac if conv else nan,
            stall_frac=tr.stalled_fraction if conv else nan,
            stall_margin_deg=tr.stall_margin_deg if conv else nan,
            reverse_frac=tr.reverse_flow_fraction if conv else nan,
            tip_mach=tr.adv_tip_mach if conv else nan,
            alpha_wing_deg=tr.aero.alpha_wing_deg if (conv and tr.aero is not None) else nan,
            rotor_lift_frac=tr.rotor_lift_N / W if conv else nan,
            control_margin=ctrl_margin,
        ))
    return nacelle_deg, out


def build_corridor_map(V_grid, nacelle_grid, rpm, altitude_m, gross_mass_kg, n_jobs=None, verbose=True):
    """Trim every (nacelle, V) point; returns dict of 2-D arrays [n_nacelle, n_V]."""
    import os
    import time
    from concurrent.futures import as_completed
    from concurrent.futures.process import BrokenProcessPool
    tasks = [(float(n), [float(v) for v in V_grid], rpm, altitude_m, gross_mass_kg) for n in nacelle_grid]
    # Default: at most 4 worker processes (a typical laptop), never more than the CPU count - 1.
    n_jobs = n_jobs or max(1, min(len(tasks), 4, (os.cpu_count() or 2) - 1))
    res = {}
    t0 = time.time()

    def done(n, out):
        res[n] = out
        if verbose:
            print(f"    row i_n = {n:5.1f} done ({time.time() - t0:.0f} s)", flush=True)

    if n_jobs > 1:
        try:
            with ProcessPoolExecutor(max_workers=n_jobs) as ex:
                futures = [ex.submit(_row_task, t) for t in tasks]
                for f in as_completed(futures):
                    done(*f.result())
        except BrokenProcessPool:
            print("    a worker process stopped unexpectedly -- finishing the remaining rows serially", flush=True)
    for t in tasks:                       # serial run, or rows lost with a broken pool
        if t[0] not in res:
            done(*_row_task(t))
    grid = {f: np.array([[p[f] for p in res[float(n)]] for n in nacelle_grid], float) for f in FIELDS}
    grid["V"] = np.asarray(V_grid, float)
    grid["nacelle"] = np.asarray(nacelle_grid, float)
    return grid
