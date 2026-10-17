# Tiltrotor BEMT + Mission Planner — Milestone 1, 2 & Modeling Project Codebase

Modular Python implementation of a BEMT rotor-performance tool, a 6-DOF trim solver, a conversion-corridor
mapper, a time-stepped Mission Planner, and a tandem-rotor aerodynamic interference model — used to evaluate
the team's tiltrotor across hover, conversion and airplane-mode flight, and to characterise tandem-rotor
interference for the Modeling Project.

---

## Repository layout

```
.
├── src/                         solver library (imported by every script)
│   ├── *.py                     Milestone 1: ISA, airfoil, rotor, axial BEMT, mission planner v1, sizing
│   ├── tandem_interference.py   Modeling Project: tandem rotor aerodynamic interference model
│   ├── tandem_rotor_config.py   Modeling Project: CH-47 Chinook class baseline configuration
│   ├── data/                    Knight & Hefner + Dingeldein (NACA TN-3236) validation datasets
│   └── m2/                      Milestone 2: edgewise BEMT, frames, airframe aero, 6-DOF trim, corridor, mission v2
├── scripts/
│   ├── m1/                      Milestone 1 figure scripts (hover, axial flight, design study, mission v1)
│   │   └── mission_tests/       Mission Planner v1 evaluation cases
│   ├── m2/                      Milestone 2 figure/table scripts (run_all_m2.py runs everything)
│   └── mp/                      Modeling Project figure scripts (tandem rotor interference)
├── bonus_simulation_transition_analysis/
│   ├── run_transition_simulation.py   ← RUN THIS for the bonus 3D transition simulation
│   └── output/                  generated interactive HTML animations (open in a web browser)
├── outputs/
│   ├── m1/                      Milestone 1 figures
│   ├── m2/rotor_<variant>/      Milestone 2 figures, tables and FIGURES.md index per rotor variant
│   └── mp/                      Modeling Project figures (validation, assessment, parameter study)
├── tests/                       pytest suite (M1 + M2 regression tests + tandem interference unit tests)
├── examples/                    short usage examples of the solver API
├── gui_app/                     optional Streamlit GUI (uses src/)
├── docs/
│   ├── milestone1/              assignment brief, formulas, flow diagrams, validation/sizing reports
│   │   └── reports/             Milestone 1 analysis write-ups (Sections 6–7)
│   └── milestone2/              Milestone 2 algorithms and logic flow
├── requirements.txt
└── setup.bat / setup.sh         install the requirements (Windows / macOS-Linux)
```

All commands below are run from the repository root.

---

## Install

```bash
pip install -r requirements.txt               # numpy, scipy, matplotlib, pytest, plotly + pandas (bonus 3D simulation); Python 3.10+, tested on 3.13
pip install -r gui_app/requirements_gui.txt   # optional: Streamlit GUI
```

**Hardware:** any 64-bit laptop or desktop with 2+ cores and 4 GB RAM. The solvers are vectorised on small
grids, run one BLAS thread per process (set automatically) and use at most 4 worker processes for the
corridor map (`--jobs N` to change). All result files are committed, so nothing has to be recomputed to read
the results; the default `run_all_m2.py` reuses the cached corridor map.

---

## Tests

```bash
python -m pytest                              # all 118 tests: M1 + M2 + tandem interference
M2_ROTOR=M1 python -m pytest tests/m2         # M2 tests on the Milestone 1 blade
python -m pytest tests/test_tandem_interference.py -v   # 13 tandem-rotor unit tests only
```

Key regression tests: `tests/m2/test_m1_recovery.py` (edgewise solver reproduces M1 hover / climb / airplane
mode to < 1 % on the design rotor), `test_edgewise_vectorized.py` (vectorized = original loop, CW/CCW mirror
symmetry), `test_trim_6dof.py` (convergence, lateral symmetry, saturation / power / wing-stall classification),
`test_tandem_interference.py` (zero/full overlap limits, asymmetric radii, Coleman wake progression, baseline
recovery, spanwise convergence, auto-trim — 13/13 passing).

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

## Modeling Project — Tandem rotor interference

### Aerodynamic interference model

In a tandem rotorcraft, longitudinal disk overlap places the aft rotor partially inside the downwash wake shed by the forward rotor, increasing aft rotor induced velocity and power required. The model combines:

* **Vortex-cylinder wake contraction (Coleman 1947, NACA TN-1218):**
  Axial downwash at vertical distance *z* downstream:

  $$v_{\text{wake}}(z) = v_i \left(1 + \dfrac{z}{\sqrt{R_{\text{fwd}}^2 + z^2}}\right)$$

  transitioning from $v_i$ at the disk plane ($z=0$) to $2 v_i$ in the far wake ($z \to \infty$).

* **Asymmetric two-circle overlap ($\kappa$):**
  Planform overlap area between two circular disks of radii $R_{\text{fwd}}$ and $R_{\text{aft}}$ separated by distance $d$:

  $$\kappa = \frac{A_{\text{overlap}}}{\pi R_{\text{aft}}^2}$$

  Handles separated ($d \ge R_1 + R_2$), tangent, concentric, and unequal-radii disks without singularities.

The ingested downwash increment applied across the aft rotor span is:

$$\Delta v_{\text{axial}} = \kappa \cdot v_{\text{wake}}(z_{\text{sep}})$$

Solved via a two-pass coupled BEMT approach: forward rotor solved in clean air $\to$ downwash increment computed $\to$ aft rotor solved with augmented axial inflow.

**Model selection rationale:** A closed-form analytical approach based on momentum and vortex cylinder theory was selected because it requires zero empirical calibration constants, evaluates in under a millisecond, and is directly compatible with the blade element framework. Higher-order methods (free-wake or CFD) would be computationally disproportionate for preliminary design and parameter sweeps.

### Modules (`src/`)

| File | Content |
|---|---|
| `tandem_interference.py` | Core model: `compute_overlap_fraction()`, `coleman_wake_velocity()`, `solve_tandem_bemt()`, `trim_tandem_hover()`, `TandemPerformance` |
| `tandem_rotor_config.py` | Boeing CH-47 Chinook baseline parameters and rotor builders |
| `data/tandem_validation_data.csv` | Digitised hover data from Dingeldein (1954, NACA TN-3236) |

### Baseline rotorcraft (CH-47 Chinook class)

| Parameter | Value |
|---|---|
| Rotor radius *R* | 9.14 m (30 ft) |
| Shaft separation *d* | 13.60 m (1.487 *R*) |
| Planform disk overlap | 14.94 % |
| Gross mass (MTOW) | 22 680 kg (50 000 lb) |
| Blade count (fwd / aft) | 3 / 3 |
| Tip speed | 220 m/s |

### Scripts & reproduction (`scripts/mp/`, figures to `outputs/mp/`)

```bash
# 1. Validation against Dingeldein (1954, NACA TN-3236) wind-tunnel data
python scripts/mp/plot_tandem_validation.py
# -> CT MAPE = 1.42%, CP MAPE = 4.54% (saved to outputs/mp/tandem_validation_dingeldein.png)

# 2. CH-47 hover assessment (SL ISA and Hot-and-High comparison)
python scripts/mp/plot_tandem_hover_assessment.py
# -> saved to outputs/mp/tandem_hover_assessment.png

# 3. Parameter sweeps (spacing, radius ratio, pylon stagger, thrust split)
python scripts/mp/plot_tandem_parameter_study.py
# -> saved to outputs/mp/tandem_parameter_study.png
```

### Assessment results summary

| Flight Condition | Baseline Power | Augmented Power | Penalty | Δ*FM* |
|---|---|---|---|---|
| Sea Level ISA | 4 346.6 kW | 4 502.8 kW | **+156.2 kW (+3.6%)** | −0.024 |
| Hot-and-High (1500 m, ISA+15 K) | 4 752.1 kW | 4 928.3 kW | **+176.2 kW (+3.7%)** | −0.024 |

* **Trim collective:** Aft rotor requires **+0.49°** (SL) / **+0.56°** (Hot-and-High) additional collective pitch to maintain equal thrust split under downwash immersion.
* **Separation sweep:** Increasing $d/R$ from 1.0 to 2.0 eliminates overlap (39.1% $\to$ 0%) and recovers 156 kW.
* **Pylon stagger:** Elevating the aft rotor ($z/R > 0.15$) reduces direct downwash ingestion.
* **Thrust split:** A slight forward bias (51% fwd / 49% aft) unloads the wake-immersed aft disk, reducing total power.

### Unit tests (`tests/test_tandem_interference.py`)

Thirteen unit tests cover limiting cases and numerical robustness (run via `pytest tests/test_tandem_interference.py`):
* Zero overlap ($d \ge R_1 + R_2 \to \kappa = 0$; identical to isolated rotor)
* Full overlap ($d = 0, R_1 = R_2 \to \kappa = 1$)
* Analytical circle intersection match at $d = R$ (error $< 10^{-7}$)
* Asymmetric radii bounds ($\kappa \in [0, 1]$)
* Coleman limits ($z=0 \to \Delta v = v_i$; $z \to \infty \to \Delta v = 2 v_i$)
* Baseline toggle recovery (`include_interference=False` matches isolated BEMT)
* Physical direction checks (power increases, FM decreases)
* Radial grid convergence ($N = 40, 80, 120$ differ $< 0.3\%$)
* Hover auto-trim convergence (target thrust within $0.05\%$)

### References

1. Coleman, R. P. & Feingold, A. M. (1947). *Theory of Self-Excited Mechanical Oscillations of Helicopter Rotors with Hinged Blades.* NACA TN-1351.
2. Coleman, R. P. (1947). *A Survey of the Ground-Vibration Characteristics of Helicopters.* NACA TN-1218.
3. Dingeldein, R. C. (1954). *Wind-Tunnel Studies of the Performance of Multirotor Configurations.* NACA TN-3236.
4. Johnson, W. (1980). *Helicopter Theory.* Princeton University Press. §3.5.
5. Stepniewski, W. Z. & Keys, C. N. (1984). *Rotary-Wing Aerodynamics.* Dover. Vol. II, Ch. 6.


---

## Bonus — 3D transition simulation

**File to run:** `bonus_simulation_transition_analysis/run_transition_simulation.py` (from the repository root,
after `pip install -r requirements.txt`):

```bash
python bonus_simulation_transition_analysis/run_transition_simulation.py                 # outbound: hover -> airplane
python bonus_simulation_transition_analysis/run_transition_simulation.py --leg inbound   # inbound: airplane -> hover
```

Runs in a few seconds and prints the path of the result:
`bonus_simulation_transition_analysis/output/transition_3d_<leg>_rotor_<variant>.html`. **Open that file in a
web browser** (Chrome, Edge, Firefox; works offline) and press **Play**. The HTML files (~10 MB each) are not
committed, so run the command above to create them.

| | What it shows |
|---|---|
| Left view | the aircraft to scale (fuselage, wing, H/V-tail, tilting nacelles, rotor disks), rotating about its CG with the trimmed pitch / roll; nacelles tilt 90° → 0° |
| Right view | flight path, ground distance vs altitude (height exaggerated); flown part in blue, red marker = current position |
| Data box | flight phase, time, airspeed, nacelle angle, pitch, altitude |
| Controls | Play (~16× real time), Slow (~5×), Pause, time slider; mouse to rotate / pan / zoom |

It does not solve anything itself: it replays the trimmed 6-DOF mission of Section 8
(`outputs/m2/rotor_<variant>/m2_8_<leg>_log.csv`, written by `scripts/m2/demo_transition_mission.py`; the logs
are committed, so no re-run is needed). Geometry is taken from `src/m2/aircraft_input_m2.py` and matches the
Section 5.1 schematic. Plot axes: x forward, y left, z up. The rotor variant follows `M2_ROTOR` as above.

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
