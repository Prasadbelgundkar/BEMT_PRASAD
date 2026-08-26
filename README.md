# Tiltrotor BEMT + Mission Planner — Milestone 1 Starter Codebase

Modular Python implementation of a BEMT rotor-performance tool and Mission
Planner v1, built around the architecture in your notes (Environment /
Geometry / BEMT Solver / Mission Planner modules).

**Read this before you touch the report.** This is a working *foundation*,
not a finished submission. What's real vs. what's a placeholder is called
out explicitly below and in code comments — replace every placeholder with
your team's actual data and design choices, and disclose AI assistance per
Section 8.3 of the handout.

## Install

```bash
pip install numpy scipy matplotlib
```

## File map -> assignment tasks

| File | Assignment task(s) |
|---|---|
| `environment.py` | ISA model — Section 1.2 |
| `airfoil.py` | Airfoil Cl/Cd model + stall flagging — Task 2 |
| `rotor.py` | Blade geometry (chord/twist distributions, solidity, tip Mach) — Task 1, 5.4 |
| `bemt.py` | Core BEMT solver: iterative inflow, Prandtl tip loss, P-G compressibility correction — Task 1 |
| `validation.py` | Hover validation vs. Knight & Hefner — Task 3 |
| `mission.py` | Mission Planner v1 + feasibility checks — Task 9, 10 |
| `examples/example_hover_and_design_study.py` | Hover maps + design-variable study — Task 4, 6.1 |
| `examples/example_axial_forward_flight.py` | Propeller-mode / advance-ratio sweep — Task 7 |
| `examples/example_mission.py` | Feasible + deliberately infeasible mission — Demonstration Cases |
| `data/knight_hefner_template.csv` | **You must fill this in** with digitized experimental data |

## What is REAL vs. PLACEHOLDER

**Real / directly from the handout:**
- Validation rotor geometry (R=0.762 m, root cutout=0.125 m, chord=0.0508 m)
  and the linear airfoil model (Cl = 5.75*alpha, Cd = 0.0113 + 1.25*alpha^2)
  in `validation.py` / `airfoil.LinearAirfoil`.
- The BEMT physics: blade-element / momentum-theory equating, iterative
  induced-velocity solve, Prandtl tip loss, stall flagging, Prandtl-Glauert
  compressibility correction, CT/CQ/CP/FM/propulsive-efficiency definitions.
- The mission-planner mechanics: segment sequencing, time-stepping, mass/fuel
  update, feasibility checks that raise `MissionInfeasibleError` with
  segment/time/reason (Task 10).

**Placeholder — YOU must replace before submitting:**
- `NUM_BLADES` and the test RPM (`OMEGA_RAD_S`) in `validation.py` — set
  to your team's chosen blade count and the actual Knight & Hefner test
  condition.
- The experimental CT/CQ data in `data/knight_hefner_template.csv` — I did
  not have access to digitize the actual published figure; you need real
  numbers here, not invented ones.
- Every rotor/aircraft number in the `examples/` scripts (radius, chord,
  twist, RPM, collective, cruise speed, gross mass, power available, SFC)
  — these were tuned only to make the demos internally self-consistent
  (non-stalled, converged) using a small toy rotor, NOT your Task 5
  tiltrotor design.
- `mission.py`'s `_required_thrust_N` for CRUISE currently returns 0 (wing
  assumed to carry weight in airplane mode) — you'll want to add a real
  drag model (D = 0.5 rho V^2 S CD) for your aircraft if you want the
  mission planner to size cruise thrust automatically rather than take
  user-specified collective/RPM directly.
- Mission Planner v1 does **not** auto-trim collective to hold weight —
  you supply collective/RPM per segment. Consider adding a root-find
  ("solve for collective such that T = W") as a Milestone 2 improvement,
  since right now getting a physically consistent mission requires you to
  pre-sweep operating points yourself (as the examples do).

## Quick start

```bash
# 1. Sanity-check the solver
python3 -c "
from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt
import numpy as np
atmo = isa(0.0, 0.0)
rotor = Rotor(0.762, 0.125, 2, constant_chord(0.0508), constant_twist(0.0))
perf = run_bemt(rotor, lambda x: LinearAirfoil(), 2*np.pi*1000/60, np.radians(8), atmo.density_kg_m3, atmo.speed_of_sound_mps)
print(perf.thrust_N, perf.torque_Nm, perf.figure_of_merit)
"

# 2. Validation (after filling in data/knight_hefner_data.csv)
python3 validation.py

# 3. Design study + hover maps
python3 examples/example_hover_and_design_study.py

# 4. Axial forward-flight sweep
python3 examples/example_axial_forward_flight.py

# 5. Mission planner demo (feasible + infeasible)
python3 examples/example_mission.py
```

## Known modeling limitations (put these in Section 3.4 / 1.1)

- The linear Cl-alpha model has no physical post-stall behavior; past the
  adopted stall angle (12 deg, `LinearAirfoil.stall_alpha_rad`) Cl is
  clipped and Cd inflated as a simple engineering fix — replace with a
  measured/tabulated polar (`TableAirfoil`) if you need believable
  post-stall numbers, especially for your Task 5 design at high collective
  or high forward speed.
- No dynamic stall, no unsteady aerodynamics, no wake distortion/vortex
  interaction model, no blade flexibility — steady/quasi-steady BEMT only,
  axisymmetric inflow (no azimuthal variation), consistent with "axial
  flow rotor" scope of this milestone.
- Prandtl-Glauert correction is frozen (not applied) above M=0.7 rather
  than extrapolated, since P-G itself becomes invalid there — you rely on
  the tip-Mach feasibility check to flag those conditions instead.

## Academic integrity note

Per the handout: discussion across teams is fine, copying code/analysis is
not, and generative-AI assistance must be disclosed (Section 8.3). This
codebase was produced with AI assistance — say so in your report, and make
sure every team member can actually explain how the BEMT loop, tip-loss
correction, and mission feasibility checks work, since that's graded too.
