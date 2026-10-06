# Tiltrotor BEMT + Mission Planner — Milestone 1 & 2 Codebase

Modular Python implementation of a BEMT rotor-performance tool, a 6-DOF trim solver, a conversion-corridor
mapper and a time-stepped Mission Planner, used to evaluate the team's tiltrotor across hover, conversion and
airplane-mode flight.

---

## Repository layout

```
.
├── src/                    solver library (imported by every script)
│   ├── *.py                Milestone 1: ISA, airfoil, rotor, axial BEMT, mission planner v1, sizing
│   ├── data/               Knight & Hefner validation data, airfoil tables
│   └── m2/                 Milestone 2: edgewise BEMT, frames, airframe aero, 6-DOF trim, corridor, mission v2
├── scripts/
│   ├── m1/                 Milestone 1 figure scripts (hover, axial flight, design study, mission v1)
│   │   └── mission_tests/  Mission Planner v1 evaluation cases
│   └── m2/                 Milestone 2 figure/table scripts (run_all_m2.py runs everything)
├── outputs/
│   ├── m1/                 Milestone 1 figures
│   └── m2/rotor_<variant>/ Milestone 2 figures, tables and FIGURES.md index per rotor variant
├── tests/                  pytest suite (M1 solver + M2 regression tests)
├── examples/               short usage examples of the solver API
├── gui_app/                optional Streamlit GUI (uses src/)
├── docs/
│   ├── milestone1/         assignment brief, formulas, flow diagrams, validation/sizing reports
│   │   └── reports/        Milestone 1 analysis write-ups (Sections 6–7)
│   └── milestone2/         Milestone 2 algorithms and logic flow
├── requirements.txt
└── setup.bat / setup.sh    install the requirements (Windows / macOS-Linux)
```

All commands below are run from the repository root.

---

## Install

```bash
pip install -r requirements.txt               # numpy, scipy, matplotlib, pytest (Python 3.10+; tested on 3.13)
pip install -r gui_app/requirements_gui.txt   # optional: Streamlit GUI
```

**Hardware:** any 64-bit laptop or desktop with 2+ cores and 4 GB RAM. The solvers are vectorised on small
grids, run one BLAS thread per process (set automatically) and use at most 4 worker processes for the
corridor map (`--jobs N` to change). All result files are committed, so nothing has to be recomputed to read
the results; the default `run_all_m2.py` reuses the cached corridor map.

---

## Tests

```bash
python -m pytest                              # M1 + M2 (design rotor 'refined'), configured in pytest.ini
M2_ROTOR=M1 python -m pytest tests/m2         # M2 tests on the Milestone 1 blade
```

Key regression tests: `tests/m2/test_m1_recovery.py` (edgewise solver reproduces M1 hover / climb / airplane
mode to < 1 % on the design rotor), `test_edgewise_vectorized.py` (vectorized = original loop, CW/CCW mirror
symmetry), `test_trim_6dof.py` (convergence, lateral symmetry, saturation / power / wing-stall classification).

---

## Milestone 2 — reproduce every graded figure and table

```bash
python scripts/m2/run_all_m2.py                          # tests + design rotor ('refined')
python scripts/m2/run_all_m2.py --variants refined M1    # both rotor variants (Section 9.2 trade-off)
python scripts/m2/run_all_m2.py --recompute-corridor     # also re-solve the corridor map
python scripts/m2/run_all_m2.py --jobs 2                 # fewer worker processes for the corridor
```

Outputs go to `outputs/m2/rotor_<variant>/`. Each folder has a `FIGURES.md` index (figure → report section →
caption with flight condition and assumptions) plus markdown/CSV tables. Measured runtime (8 efficiency cores of
a desktop CPU, similar to a mid-range laptop): tests 70 s; each variant ~10 min with the cached corridor
(`corridor_grid.npz`), of which the two transition missions take ~6 min; re-solving the corridor adds ~5.5 min
with `--jobs 8`, about twice that with the default 4 worker processes. A script that stops abnormally is run
once more before it is reported as failed.

### Rotor variant switch

The aircraft is defined once in `src/m2/aircraft_input_m2.py`. It takes the Milestone 1 design of record
(`src/aircraft_input.py`: rotor planform, airfoil, 550 / 250 RPM, collective range, masses, power-lapse model)
and adds the airframe, mass items / CG, control limits, the selected engines (2 × GE CT7-8A, from
`engine_selection.py`) and the airplane-mode RPM schedule (250 RPM long-range cruise, 350 RPM to 100 m/s,
420 RPM dash). Two blades are available through an environment variable:

| `M2_ROTOR` | Blade twist | Notes |
|---|---|---|
| `refined` (default, M2 design) | 12° root, −30°/R | less inboard pitch: removes the inboard hover stall of the M1 blade |
| `M1` | 25° root, −45°/R (Milestone 1 blade) | kept for the Milestone 1 / Milestone 2 trade-off (Section 9.2) |

```bash
set M2_ROTOR=M1                 # Windows cmd
$env:M2_ROTOR="M1"              # PowerShell
export M2_ROTOR=M1              # bash
```

### Scripts (`scripts/m2/`)

| Script | Report section | Output |
|---|---|---|
| `verify_edgewise.py` | 3.1–3.4 | M1 recovery, sectional-load polar plots, U_T / reverse flow / Mach / stall map, grid sensitivity |
| `demo_rotor_control_sweep.py` | 4.1–4.3, 4.6 | single-rotor collective / θ1c / θ1s sweeps (FX…MZ, power, stall margin), control derivatives |
| `plot_aircraft_schematic.py` | 5.1 | dimensioned top/side views drawn from the config |
| `make_design_tables.py` | 5.2–5.5 | change log, rotor / wing / empennage / mass / CG / limit tables, blade distributions |
| `engine_selection.py` | 5.2 | engine sizing: trimmed power required at the sizing conditions vs five candidate turboshafts |
| `demo_trim_matrix.py` | 6.1, 6.2, 6.4 | 4 nacelle angles × 3 speeds, full 6-DOF trim table, trends, lift sharing |
| `demo_failed_trim.py` | 6.3 | seven failed cases, each classified (numerical, control, stall, power, tip Mach, physical) |
| `demo_corridor_map.py` | 7.1, 7.2 | 13 × 21 speed–nacelle map (6-DOF trim at every point), constraint fields |
| `demo_transition_mission.py` | 8.1, 8.2 | outbound (hover → airplane) and inbound (airplane → hover) Mission Planner v2 runs |
| `compare_m1_m2.py` | 9.1, 9.2 | M1 vs M2 power / stall / envelope comparison, rotor-variant trade table |

Early-draft demos (`demo_control_sweep.py`, `demo_azimuthal_loading.py`, `demo_transition_sweep.py`,
`demo_full_mission.py`) still run and write to `outputs/m2/`, but are superseded by the scripts above.

### Modules (`src/m2/`)

| File | Content |
|---|---|
| `aircraft_input_m2.py` | single source of truth: rotor variant, RPM schedule, installed power, wing / H-tail / V-tail, component locations, mass breakdown → CG(i_n, fuel), control limits, stick mixing, conversion paths, design-change log |
| `frames.py` | inertial / body / shaft (hub) / blade frames, rotations, moment transfer to the CG |
| `edgewise_bemt.py` | vectorized azimuth-resolved BEMT: cyclic pitch, annular-Glauert inflow with tip loss and the handout's K-factor, reverse flow, Prandtl–Glauert, stall and tip-Mach diagnostics, CW/CCW mirror |
| `aero_models.py` | wing + flaperons, H-tail + elevator (downwash), V-tail + rudder, fuselage flat-plate drag (`airframe_loads`) |
| `trim_6dof.py` | 6-DOF trim: unknowns θ, φ, θ0, δlon, δlat, δped; six residuals about the CG; bounded least squares; status + limit flags + `diagnose()` |
| `conversion_corridor.py` | parallel corridor map built on `trim_6dof` (legacy 3-DOF functions kept) |
| `mission_v2.py` | `MissionPlannerV2`: time-stepped segments with airspeed / climb / nacelle / RPM / wind schedules, online trim, fuel and mass update, continuity and limit checks (legacy function kept) |
| `trim_solver.py` | original 3-DOF longitudinal trim (kept for regression tests) |

### Conventions and assumptions (summary)

* **Frames:** body x fwd, y right, z down, origin at the CG; hub frame x aft, y advancing side, z thrust; blade
  azimuth ψ from aft in the direction of rotation. Nacelle angle i_n = 90° helicopter, 0° airplane. Right rotor
  counter-clockwise seen from above, left rotor clockwise (mirror image).
* **Rotor:** rigid disk (no flapping), quasi-steady linear airfoil (a0 = 5.75/rad, stall flag 14°, Cl clipped
  post-stall), Prandtl–Glauert on Cl only (frozen above M = 0.7), annular-Glauert + K-factor inflow, reverse-flow
  sections with reversed incidence and in-plane force. No dynamic stall, unsteady wake or blade elasticity.
* **Airframe:** freestream velocity and incidence at every surface; tail downwash from the wing only; fuselage +
  nacelle drag as a flat plate at the CG; **no rotor-wake/wing interference and no hover download**.
* **Controls:** pitch = θ1c + elevator, roll = differential collective + flaperons, yaw = differential θ1s +
  rudder; rotor terms faded out with sin²(i_n).
* **Limits:** rotor stalled loaded area ≤ 5 %, reverse-flow area ≤ 3 %, advancing-tip Mach ≤ 0.85, 5 % power
  margin, wing α below stall, controls within bounds, nacelle rate ≤ 8°/s.

Full description: [`docs/milestone2/Milestone_2_Architecture.md`](docs/milestone2/Milestone_2_Architecture.md).

---

## Milestone 1 — hover and axial flow

### Modules (`src/`)

| File | Content / assignment task |
|---|---|
| `environment.py` | ISA model — Section 1.2 |
| `airfoil.py` | airfoil Cl/Cd models + stall flagging — Task 2 |
| `rotor.py` | blade geometry (chord/twist distributions, solidity, tip Mach) |
| `bemt.py` | axial BEMT solver (iterative inflow, Prandtl tip loss) |
| `validation.py` | hover validation vs. Knight & Hefner — Task 3 |
| `mission.py` | Mission Planner v1 (segments, fuel burn, feasibility checks) |
| `aircraft_input.py` | Milestone 1 aircraft definition (also the base of the M2 configuration) |
| `aircraft_sizing.py` | conceptual sizing and constraint analysis |

### Scripts (`scripts/m1/`, figures to `outputs/m1/`)

| Script | Output |
|---|---|
| `plot_design_study.py`, `twist_study.py`, `taper_ratio_study.py`, `solidity_physics_study.py` | design-variable studies (blades, twist, taper, solidity, root cut-out) |
| `standalone_hover_plots.py`, `plot_hover_maps.py`, `plot_hover_envelope.py` | hover performance maps, ceiling and operating envelope |
| `fm_vs_ct_tiltrotor.py`, `plot_benchmarking.py` | figure of merit vs CT, comparison with other rotors |
| `plot_axial_flight.py`, `plot_efficiency_map.py` | airplane-mode (axial) performance and propulsive-efficiency map |
| `plot_hover_endurance.py`, `plot_hover_fuel_burn.py`, `plot_cruise_range.py`, `plot_mission_analysis.py` | Mission Planner v1 endurance, fuel burn and range studies |
| `verify_mission_planner.py`, `plot_mission_verification.py` | Mission Planner v1 verification table and plots |
| `mission_tests/run_all_evaluations.py` | Mission Planner v1 evaluation cases (mass/payload/fuel, atmosphere/wind, failure logic, feasible mission) |

```bash
python src/validation.py                  # validation vs Knight & Hefner (2-, 3- and 4-blade data)
python scripts/m1/plot_axial_flight.py    # axial forward-flight performance
streamlit run gui_app/app.py              # interactive GUI
```

Milestone 1 assumptions: no dynamic stall, no unsteady aerodynamics, no blade flexibility; the linear Cl–α model
has no physical post-stall behaviour; Prandtl–Glauert frozen above M = 0.7.

---

## Acknowledgement

Generative-AI assistance was used during development of parts of this code, as acknowledged in the project
report in line with the course policy on assistance and external code.
