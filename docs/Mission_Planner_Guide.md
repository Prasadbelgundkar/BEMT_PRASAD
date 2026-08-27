# Mission Planner Architecture & Execution Logic

## 1. Overview
The Mission Planner (`mission.py` and `run_mission.py`) is designed as a **discrete-time flight simulator**. Instead of evaluating the aircraft at a single steady-state snapshot, it mathematically "flies" the tiltrotor through the sky by breaking the flight plan down into small time steps (e.g., ticking the clock every 10 to 60 seconds). At every tick of the clock, it dynamically recalculates the exact aerodynamic physics, updates the fuel burn, and checks for structural limits.

## 2. Core Modules (The Building Blocks)
The planner is highly modular and does not hard-code a specific aircraft. Instead, it accepts external models:
* **`Rotor` & Airfoils:** Defines the blade geometry (3.8m radius, twist, chord, etc.).
* **`PowerAvailableModel`:** Simulates how the GE CT7-8A turboshaft engines degrade in power as the aircraft climbs into thinner atmosphere.
* **`FuelModel`:** Simulates fuel consumption using a constant Specific Fuel Consumption (SFC) metric in `kg/J`.
* **`DesignLimits`:** The safety monitor (Task 10 requirements). It enforces the maximum allowable blade stall, tip Mach limits, minimum power margins, and FAA reserve fuel requirements.
* **`MissionState`:** A dynamic data structure (the "flight recorder") that actively tracks `time_s`, `fuel_mass_kg`, and `gross_mass_kg`. As the flight progresses, fuel is burned and `gross_mass_kg` decreases, which physically lowers the power required for the subsequent time steps.

## 3. Flight Segments and Aerodynamic Conditions
A mission is defined as a sequence of `MissionSegment` blocks (e.g., Takeoff Hover, Climb, High-Alt Cruise). The planner handles each flight condition by dynamically altering the **Inflow Velocity ($V_{axial}$)** before sending the data to the BEMT solver:

1. **`HOVER` (Takeoff, Landing, Troop Drop):** 
   * $V_{axial} = 0.0$ m/s. 
   * The BEMT solver evaluates the rotor in pure helicopter mode.
2. **`VERTICAL_CLIMB`:**
   * $V_{axial} = V_{climb}$ (e.g., 5.0 m/s). 
   * The upward climb speed is added to the inflow velocity. The BEMT engine handles this via the Momentum Theory climb equation ($P_{climb} = T(V_c + v_i)$), resulting in a massive spike in required power.
3. **`CRUISE` (Airplane Mode):**
   * $V_{axial} = V_{cruise}$ (e.g., 125 m/s or 450 km/h). 
   * The planner automatically switches its drag model to $D = \frac{1}{2} \rho V^2 f$ (using our $f = 1.7 \text{ m}^2$ flat plate area). It passes the massive $125 \text{ m/s}$ inflow speed into the BEMT engine. The collective pitch and RPM must be adjusted by the user in `parameters.py` to prevent the blades from entering a windmill-brake stall state.

## 4. The Core Simulation Loop (`run_segment`)
When the simulation runs, it executes the following mathematical sequence for *every single time step* (`dt_s`) across the entire flight profile:

1. **Environmental Query:** It calls `environment.isa()` to get the precise air density ($\rho$) and speed of sound ($a$) at the aircraft's current altitude.
2. **Aerodynamic Solve:** It hands the RPM, collective pitch, air density, and $V_{axial}$ to the **BEMT Engine**. The BEMT engine slices the blade into aerodynamic elements, balances Momentum Theory with Blade Element Theory, and returns the exact Thrust ($T$) and Power ($P$) required for that specific second of flight.
3. **Task 10 Limit Check:** It passes the BEMT performance data into a `_check_limits()` function. If the power required exceeds the engine's capability, if the tips break the sound barrier, or if the blades stall, it instantly crashes the simulation and raises a custom `MissionInfeasibleError`, reporting the exact time and aerodynamic reason for the failure.
4. **Fuel Burn & Weight Update:** It asks the `FuelModel` how much fuel was burned over the `dt_s` window based on the actual power drawn. It subtracts that mass from `fuel_mass_kg` and `gross_mass_kg`.

## 5. Centralized Configuration
To allow for rapid conceptual iteration, all variables have been moved to `parameters.py`. The execution script (`run_mission.py`) dynamically reads this parameter file, builds the exact aircraft specified, runs the simulation loop, and automatically plots a telemetry dashboard showing real-time fuel burn curves and flight phase transitions using `matplotlib`.
