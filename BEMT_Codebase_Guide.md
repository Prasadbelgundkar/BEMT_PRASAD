# Tiltrotor BEMT Codebase: Logical Guide & Physics Breakdown

This document serves as a comprehensive guide to understanding the Blade Element Momentum Theory (BEMT) solver codebase. It explains the logical flow of the code, how the physics are implemented, and answers common aerodynamic questions regarding the solver's architecture.

---

## 1. The Environment & Airfoils

### `environment.py`
This file acts as the single source of truth for atmospheric properties. It implements the International Standard Atmosphere (ISA) model.
* **Logic:** It takes an altitude and a temperature offset (`dISA_K`). It calculates the standard temperature lapse rate and hydrostatic pressure, and returns an `AtmoState` object containing the air density ($\rho$) and speed of sound ($a$) which the solver needs for aerodynamic calculations.

### `airfoil.py`
This file defines the lift and drag characteristics of the blade cross-sections.
* **The Linear Model:** A simplified mathematical model ($C_l = a_0 \alpha$) used for basic validation. It includes a hardcoded stall angle to prevent the solver from calculating infinite lift.
* **The Blended CSV Model:** We added a `BlendedLinearAirfoilProvider` class. This allows you to define different airfoil properties at different stations along the blade (e.g., a thick root and a thin tip) via a `.csv` file. The solver interpolates these aerodynamic properties smoothly as it walks down the blade radius.
* **Compressibility:** The file also contains the `prandtl_glauert_correct` function, which scales lift and drag as the blade tip approaches the speed of sound, freezing the correction at Mach 0.7 to prevent mathematical singularities.

---

## 2. Rotor Geometry (`rotor.py`)

This file acts as a container (`Rotor` class) for the blade's physical dimensions.

### Rules vs. Numbers (Functions as Variables)
Instead of using fixed numbers for chord and twist, the codebase uses **Functions** (e.g., `chord_fn`, `twist_fn`). 
Because the width and twist change depending on where you are on the blade, the solver needs a "Rule" to follow. The solver steps along the non-dimensional blade radius $x$ (where $x = r/R$) and asks the rule: *"I am at 50% radius, what is the width here?"* This allows you to model highly complex tapers and twists without hardcoding arrays of numbers.

### The Coordinate System and Root Cutout
* The math is defined from $x=0$ (the absolute center of the spinning shaft) to $x=1.0$ (the tip).
* However, **the solver never evaluates $x=0$.** The aerodynamic blade doesn't start at the hub; it starts at the `root_cutout_m`. The solver intelligently begins its mathematical slices at the root cutout and walks to the tip. Defining equations from the center simply makes the geometry math easier to write.

---

## 3. The Control Panel (`parameters.py`)

To make running design studies easier, we centralized the inputs into a single file. 
* **Environment:** Altitude and Temperature.
* **Geometry:** Radius, Cutout, Taper ratio, and built-in structural twist.
* **Operating Conditions:** Engine RPM, forward flight velocity, and Collective Pitch.

**Why is it called `Collective` and not `Pilot Pitch`?**
A rotorcraft has two types of pitch controls: *Cyclic* (which tilts the disk for forward flight by changing pitch dynamically around the circle) and *Collective* (which changes the pitch of all blades uniformly at the same time to go straight up). Because our BEMT solver assumes symmetrical airflow (hover or pure axial forward flight), it only models Collective pitch.

**A Note on Units:** The solver itself (`bemt.py`) only accepts strict physics units (Radians, Radians/Second). The `parameters.py` file accepts human-readable units (RPM, Degrees) and the run scripts handle the mathematical conversion before passing them to the solver.

---

## 4. The Physics Engine (`bemt.py`)

This is the core of the codebase. It calculates thrust by forcing two different aerodynamic theories to agree with each other.

### Why do we guess? (Annular Momentum Theory)
Basic Momentum Theory (Actuator Disk) treats the rotor as a solid fan and calculates one average downwash velocity ($v_{avg}$) for the whole helicopter. We cannot use this global average because the tips of the blades suck down far more air than the root. 

BEMT uses **Annular Momentum Theory**, treating the rotor as a series of concentric rings. The lift at radius $r$ exactly dictates the downwash $v$ at radius $r$. 
* We can't know the Lift without knowing $v$ (because $v$ changes the Angle of Attack).
* We can't know $v$ without knowing the Lift.
Because they depend on each other, the solver guesses $v$, calculates thrust using both theories, and compares them at every single slice of the blade.

### The BEMT Loop (`solve_element`)
For a single slice of the blade:
1. **The Guess:** The solver guesses an induced velocity $v$.
2. **Blade Element Theory (BET):** It treats the slice as an airplane wing. It calculates the total airspeed and Angle of Attack, looks up $C_l$ and $C_d$, and calculates the Wing Thrust ($dT_{BET}$).
3. **Momentum Theory:** It treats the slice as a fan pushing a ring of air. It calculates the Momentum Thrust ($dT_{mom}$).
4. **The Balance:** It calculates the `Residual = dT_BET - dT_mom`. 

### The Root Finder (Brent's Method)
To force the `Residual` to exactly zero, the code uses Brent's Method (`brentq`):
1. **The Bracket:** It first scans velocities to find a "trap"—one guess where the residual is positive, and one where it is negative. The true answer must be inside this trap.
2. **The Correction:** It uses a hybrid approach. It tries to make a highly intelligent "Fast" guess by drawing a line between the data points to predict where it crosses zero (Secant method). If that guess is unsafe, it falls back to a "Safe" guess by just chopping the trap perfectly in half (Bisection). 
3. It repeats this, shrinking the trap until it finds the exact velocity $v$ within a tolerance of `1e-8`.

### Prandtl Tip Loss ($F$)
High-pressure air leaks around the tips of the blades, reducing lift. Momentum theory assumes infinite blades and ignores this. The Prandtl Tip Loss factor $F$ fixes it.
* The code calculates the exact trigonometric formula for $F$. (Note: Analytical class notes often show a simplified formula using the inflow ratio $\lambda$; the code does not use this small-angle approximation, making it more accurate for large angles).
* **Application:** $F$ is multiplied into the Momentum Theory thrust equation. By artificially shrinking the momentum thrust at the tip, the solver is forced to compensate by guessing a much higher downwash velocity ($v$). A higher downwash brutally reduces the Angle of Attack at the tip, accurately modeling the loss of real-world lift.

### Integration (`run_bemt`)
**Crucial Note:** The solver *never* integrates the velocity $v$. We want $v$ to remain distinct at every slice.
Instead, the solver integrates the **Forces**. 
Once `solve_element` finds the accurate $v$, it locks it in and calculates the final Thrust ($dT$) and Torque ($dQ$) for that tiny slice. The main `run_bemt` loop collects these tiny forces from all 60 slices of the blade and uses numerical calculus (the Trapezoidal Rule) to sum them up. This results in the Total Rotor Thrust and Total Rotor Torque.
