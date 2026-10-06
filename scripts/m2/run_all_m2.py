"""
Milestone 2 -- reproduce every graded figure and table (report Section 10.1).

    python scripts/m2/run_all_m2.py                          # design rotor ('refined'), tests first
    python scripts/m2/run_all_m2.py --variants refined M1    # both rotor variants (Section 9.2 trade-off)
    python scripts/m2/run_all_m2.py --recompute-corridor     # re-solve the 273-point corridor map
    python scripts/m2/run_all_m2.py --jobs 2                 # worker processes for the corridor (default <= 4)
    python scripts/m2/run_all_m2.py --skip-tests

Each variant writes to outputs/m2/rotor_<variant>/ with a FIGURES.md index
(figure -> report section -> caption with flight condition and assumptions).
Measured runtime with the cached corridor: ~10 min per variant (the two
transition missions take ~6 min); --recompute-corridor adds ~5.5 min with
--jobs 8, about twice that with the default 4 worker processes. Every script
runs in its own process with one BLAS
thread; a script that stops abnormally is run once more before it is
reported as failed.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))

PIPELINE = [
    ("verify_edgewise.py", "Section 3: M1 recovery, azimuthal loading, reverse flow / Mach / stall, grid sensitivity"),
    ("demo_rotor_control_sweep.py", "Section 4: collective / theta1c / theta1s sweeps, control derivatives"),
    ("plot_aircraft_schematic.py", "Section 5.1: aircraft schematic"),
    ("make_design_tables.py", "Sections 5.2-5.5: design tables, blade distributions"),
    ("demo_trim_matrix.py", "Sections 6.1, 6.2, 6.4: trim matrix, table, trends"),
    ("demo_failed_trim.py", "Section 6.3: failed-trim cases"),
    ("demo_corridor_map.py", "Section 7: conversion corridor and constraint boundaries"),
    ("engine_selection.py", "Section 5.2: engine sizing and selection (uses the corridor map)"),
    ("demo_transition_mission.py", "Section 8: Mission Planner v2 outbound / inbound transitions"),
    ("compare_m1_m2.py", "Section 9: comparison with Milestone 1, rotor trade-off data"),
]


def run(cmd, env, retries=1):
    """Run one script; a non-zero exit is retried `retries` times."""
    t0 = time.time()
    for attempt in range(retries + 1):
        rc = subprocess.run(cmd, cwd=ROOT, env=env).returncode
        if rc == 0:
            break
        if attempt < retries:
            print(f"    exit code {rc} -- running it once more", flush=True)
    return rc, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["refined"], choices=["M1", "refined"])
    ap.add_argument("--recompute-corridor", action="store_true")
    ap.add_argument("--jobs", type=int, default=None, help="worker processes for the corridor map")
    ap.add_argument("--skip-tests", action="store_true")
    a = ap.parse_args()
    base = dict(os.environ, MPLBACKEND="Agg", PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8",
                OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    failures = []
    if not a.skip_tests:
        print("=== pytest (Milestone 1 + Milestone 2 regression tests)", flush=True)
        rc, dt = run([sys.executable, "-m", "pytest", "tests", "-q"], base)
        print(f"    pytest exit code {rc} ({dt:.0f} s)", flush=True)
        if rc:
            failures.append("pytest")
    for var in a.variants:
        env = dict(base, M2_ROTOR=var)
        for script, what in PIPELINE:
            cmd = [sys.executable, os.path.join(HERE, script)]
            if script == "demo_corridor_map.py":
                if a.recompute_corridor:
                    cmd.append("--recompute")
                if a.jobs:
                    cmd += ["--jobs", str(a.jobs)]
            if script == "engine_selection.py":
                cmd.append("--recompute")
            print(f"\n=== [{var}] {script}  --  {what}", flush=True)
            rc, dt = run(cmd, env)
            print(f"    exit code {rc} ({dt:.0f} s)", flush=True)
            if rc:
                failures.append(f"{var}:{script}")
    print("\nDone." if not failures else f"\nFAILED: {failures}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
