"""
Milestone 2 -- reproduce every graded figure and table (report Section 10.1).

    python scripts/m2/run_all_m2.py                      # both rotor variants, tests first
    python scripts/m2/run_all_m2.py --variants refined   # one variant
    python scripts/m2/run_all_m2.py --recompute-corridor # re-solve the corridor (~12-15 min / variant)
    python scripts/m2/run_all_m2.py --skip-tests

Each variant writes to outputs/m2/rotor_<variant>/ with a FIGURES.md index
(figure -> report section -> caption with flight condition and assumptions).
Typical runtime per variant with a cached corridor: ~10 min (the two
transition missions dominate); with --recompute-corridor add ~12-15 min
(uses all CPU cores but one).
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
    ("demo_transition_mission.py", "Section 8: Mission Planner v2 outbound / inbound transitions"),
    ("compare_m1_m2.py", "Section 9: comparison with Milestone 1, rotor trade-off data"),
]


def run(cmd, env):
    t0 = time.time()
    r = subprocess.run(cmd, cwd=ROOT, env=env)
    return r.returncode, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["refined", "M1"], choices=["M1", "refined"])
    ap.add_argument("--recompute-corridor", action="store_true")
    ap.add_argument("--skip-tests", action="store_true")
    a = ap.parse_args()
    failures = []
    if not a.skip_tests:
        print("=== pytest (Milestone 1 + Milestone 2 regression tests)")
        rc, dt = run([sys.executable, "-m", "pytest", "tests", "-q"], dict(os.environ, M2_ROTOR="M1"))
        print(f"    pytest exit code {rc} ({dt:.0f} s)")
        if rc:
            failures.append("pytest")
    for var in a.variants:
        env = dict(os.environ, M2_ROTOR=var, MPLBACKEND="Agg", PYTHONUNBUFFERED="1")
        for script, what in PIPELINE:
            cmd = [sys.executable, os.path.join(HERE, script)]
            if script == "demo_corridor_map.py" and a.recompute_corridor:
                cmd.append("--recompute")
            print(f"\n=== [{var}] {script}  --  {what}")
            rc, dt = run(cmd, env)
            print(f"    exit code {rc} ({dt:.0f} s)")
            if rc:
                failures.append(f"{var}:{script}")
    print("\nDone." if not failures else f"\nFAILED: {failures}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
