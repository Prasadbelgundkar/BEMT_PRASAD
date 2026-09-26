# Tiltrotor BEMT + Mission Planner — Milestone 1 & 2 Codebase

Modular Python implementation of a BEMT rotor-performance tool, 3-DOF trim solver, and Mission Planner, designed to evaluate tiltrotor aircraft configurations across hover, conversion, and forward flight.

This repository covers the complete architecture detailed in your assignments for **Milestone 1 (Hover & Axial Flow)** and **Milestone 2 (Forward Flight, Trim & Transition)**. 

---

## What's New in Milestone 2
- **Edgewise Forward Flight BEMT**: The rotor solver now discretizes the disk azimuthally and radially, applying Glauert's momentum equation for non-uniform inflow, handling reversed flow, and incorporating cyclic pitch.
- **3-DOF Longitudinal Trim Solver**: Uses optimization (`scipy.optimize.root`) to balance total aircraft forces ($F_x$, $F_z$) and pitching moment ($M_y$) by solving for angle of attack ($\alpha$), collective pitch ($\theta_0$), and longitudinal control.
- **Control Actuator Blending**: Automatically shifts longitudinal control authority from cyclic pitch (helicopter mode, nacelle > 45°) to elevator deflection (airplane mode, nacelle < 45°).
- **Conversion Corridor Mapping**: Sweeps airspeed and nacelle angles to map the feasible trim envelope for transition.
- **Interactive GUI**: A new interactive Streamlit application (`gui_app/app.py`) for rapid design parameter sweeping, airfoil comparison, and performance plotting.

---

## Install

```bash
# Core computational requirements
pip install numpy scipy matplotlib

# To run the Milestone 2 Interactive GUI
pip install -r gui_app/requirements_gui.txt
```

---

## Architecture & File Map

### Milestone 2: Forward Flight & Trim (`src/m2/`)
| File | Description |
|---|---|
| `edgewise_bemt.py` | Core rotor aerodynamic solver for edgewise forward flight |
| `aero_models.py` | Aerodynamic models for fixed-wing components (wing & h-tail) |
| `trim_solver.py` | 3-DOF longitudinal trim solver |
| `conversion_corridor.py` | Maps feasible trim states across nacelle and velocity sweeps |
| `frames.py` | Rigid body coordinate transformations |
| `scripts/m2/` | Demonstration scripts (transition sweep, mission profile, trim matrix) |
| `tests/m2/` | Pytest suite for the trim solver, frames, and recovery behaviors |

*(See [Milestone_2_Architecture.md](Milestone_2_Architecture.md) for a detailed architecture diagram and methodology breakdown).*

### Milestone 1: Hover & Axial Flow (`src/` and `scripts/`)
| File | Assignment task(s) |
|---|---|
| `environment.py` | ISA model — Section 1.2 |
| `airfoil.py` | Airfoil Cl/Cd model + stall flagging — Task 2 |
| `rotor.py` | Blade geometry (chord/twist distributions, solidity, tip Mach) |
| `bemt.py` | Core axial BEMT solver (iterative inflow, Prandtl tip loss) |
| `validation.py` | Hover validation vs. Knight & Hefner — Task 3 |

---

## Quick Start

### Milestone 2 Examples
```bash
# 1. Run a conversion corridor sweep
python scripts/m2/demo_corridor_map.py

# 2. Sweep cyclic/elevator control effectiveness
python scripts/m2/demo_control_sweep.py

# 3. Simulate a full tiltrotor mission profile
python scripts/m2/demo_full_mission.py
```

### Launch the Interactive GUI
```bash
cd gui_app
streamlit run app.py
```

### Milestone 1 Examples
```bash
# 1. Validation vs Knight & Hefner (requires filled CSV data)
python gui_app/src/validation.py

# 2. Axial forward-flight performance
python scripts/plot_axial_flight.py
```

---

## Known Modeling Limitations

**Milestone 2 Assumptions:**
- **Rigid Blades**: Flapping dynamics are ignored ($\beta = 0$). Hub moments are computed purely from aerodynamic force asymmetries.
- **Symmetric Flight**: Assumes purely longitudinal motion with no side-slip ($\beta_{yaw} = 0$), roll, or yaw. Lateral equations of motion are decoupled and ignored.
- **Interference Effects**: Rotor wake impingement on the wing and tail is currently neglected or simplified.

**Milestone 1 Assumptions:**
- No dynamic stall, no unsteady aerodynamics, no blade flexibility.
- The default linear Cl-alpha model has no physical post-stall behavior. You must use a tabulated airfoil (like the provided NACA profiles in the GUI app) for realistic high-alpha performance.
- Prandtl-Glauert correction is frozen (not applied) above M=0.7 rather than extrapolated.

---

## What is REAL vs. PLACEHOLDER (Action Required)

**Real:**
- The BEMT physics (both axial and edgewise), rigid body transformations, trim optimization logic, atmospheric models, and geometric structural mapping.
- The `tests/` directory verifying standard math and edge cases.

**Placeholder — YOU must replace before submitting:**
- The experimental CT/CQ data in `gui_app/src/data/knight_hefner_template.csv`. You must fill this with digitized experimental data for accurate Task 3 validation.
- The specific tiltrotor design variables inside the M2 demonstration scripts (`scripts/m2/*`). They are currently populated with toy numbers to ensure the trim solver loops run smoothly. You must replace them with your Task 5 design choices.

## Academic Integrity Note
Per the handout: discussion across teams is fine, copying code/analysis is not, and generative-AI assistance must be disclosed (Section 8.3). This codebase was produced with AI assistance — say so in your report, and ensure every team member can explain the core concepts (BEMT loop, trim solving, conversion assumptions) as they are graded aspects of your defense.
