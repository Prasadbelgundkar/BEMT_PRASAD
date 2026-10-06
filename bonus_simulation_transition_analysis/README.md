# Bonus — 3D transition simulation

**Run this file:** `run_transition_simulation.py`

From the repository root (after `pip install -r requirements.txt`):

```bash
python bonus_simulation_transition_analysis/run_transition_simulation.py                 # hover -> airplane (outbound)
python bonus_simulation_transition_analysis/run_transition_simulation.py --leg inbound   # airplane -> hover (inbound)
```

It takes a few seconds and writes `output/transition_3d_<leg>_rotor_<variant>.html`. Open that file in a web
browser (works offline) and press **Play**. Drag to rotate, scroll to zoom, use the slider to scrub in time.

The animation replays the trimmed 6-DOF transition mission of report Section 8 (logs in
`outputs/m2/rotor_<variant>/`, produced by `scripts/m2/demo_transition_mission.py`). See the main
[README](../README.md#bonus--3d-transition-simulation) for details.
