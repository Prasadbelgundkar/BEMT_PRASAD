# Tiltrotor BEMT Milestone 2: Architecture & Code Flow

## 1. Overview
Milestone 2 expands the foundational hover BEMT code into a comprehensive forward flight and transition analysis tool for tiltrotor aircraft. It introduces edgewise rotor aerodynamics, non-uniform inflow, reversed flow handling, full aircraft longitudinal trim, and conversion corridor mapping.

## 2. Code Flow Structure
The new capabilities are heavily compartmentalized within the `src/m2/` directory:

- `src/m2/edgewise_bemt.py`: The core rotor aerodynamic solver for forward flight. It discretizes the disk azimuthally and radially.
- `src/m2/aero_models.py`: Aerodynamic models for the fixed-wing components (wing and horizontal tail).
- `src/m2/trim_solver.py`: 3-DOF longitudinal trim solver using `scipy.optimize.root` to balance forces and moments.
- `src/m2/conversion_corridor.py`: Explores the trim envelope over a grid of nacelle angles and airspeeds to map the feasible transition corridor.
- `src/m2/mission_v2.py`: Mission analysis integrating trim solutions over time/distance profiles.
- `src/m2/frames.py`: Rigid body coordinate transformations (shaft to body frame, computing moments about the CG).
- `src/m2/aircraft_input_m2.py`: Expanded dataclasses representing the complete aircraft geometry (CG offsets, wing areas, etc.).

## 3. Workflow Diagram

```mermaid
flowchart TD
    subgraph Inputs
    A1[Aircraft Geometry & CG] --> Trim
    A2[Rotor & Airfoil Data] --> BEMT
    A3[Flight State: V_inf, Nacelle Angle, Altitude] --> Trim
    end

    subgraph Trim Solver [3-DOF Longitudinal Trim Optimizer]
    TrimState[State Guess: Alpha, Collective, Pitch Control]
    Residuals[Residual Evaluation: Fx, Fz, My]
    TrimState --> Residuals
    end

    subgraph Aerodynamic Models
    BEMT[Edgewise BEMT]
    Wing[Wing Aerodynamics]
    Tail[Tail Aerodynamics]
    end

    Inputs --> TrimState
    
    TrimState -->|V, Alpha, Omega, Nacelle, Collective, Cyclic| BEMT
    TrimState -->|V, Alpha| Wing
    TrimState -->|V, Alpha, Elevator| Tail
    
    BEMT -->|Forces & Moments| Residuals
    Wing -->|Forces & Moments| Residuals
    Tail -->|Forces & Moments| Residuals

    Residuals -->|Check Convergence| TrimState
    
    Residuals -->|Converged (Res < Tol)| Output
    
    subgraph Outputs
    Output[Trimmed State: Power, Pitch Attitude, Controls, Stall Margin]
    end
```

## 4. Methodology

### Rotor Aerodynamics (Edgewise BEMT)
- **Discretization**: The rotor disk is discretized both radially ($r$) and azimuthally ($\psi$).
- **Inflow Modeling**: Applies Glauert's momentum equation for mean inflow, modified by a non-uniform inflow factor ($K$) to account for forward flight asymmetry.
- **Blade Kinematics**: Accounts for cyclic pitch ($\theta_{1c}$, $\theta_{1s}$) and computes local blade velocity components ($U_T$, $U_P$) at every azimuth station.
- **Reversed Flow**: Explicitly identifies and handles reversed flow regions on the retreating blade, reversing lift orientation appropriately.
- **Integration**: Aerodynamic forces are computed at each element using lookup tables with Prandtl-Glauert compressibility corrections, then numerically integrated (using trapezoidal rules) to find total Thrust, H-force, Y-force, Torque, and Hub Moments.

### Aircraft Trim
- **Degrees of Freedom**: Solves a 3-DOF longitudinal trim problem (Sum of Forces X = 0, Sum of Forces Z = 0, Sum of Pitching Moments Y = 0).
- **Optimization States**: Solves for angle of attack ($\alpha$), collective pitch ($\theta_0$), and longitudinal control.
- **Control Blending**: Actuator logic shifts based on nacelle angle. At high nacelle angles (Helicopter mode, >45°), cyclic pitch is used for longitudinal trim. At low nacelle angles (Airplane mode, <45°), the elevator takes over.

### Conversion Corridor Mapping
- Sweeps a matrix of airspeeds and nacelle angles.
- Feeds the previous successfully trimmed state as the initial guess for the next adjacent state, which greatly accelerates the optimizer's convergence through the highly non-linear transition regime.

## 5. Key Assumptions
1. **Rigid Blades**: Flapping dynamics are currently ignored ($\beta = 0$). Hub moments are computed purely from aerodynamic force asymmetries rather than blade root structural constraints or flapping hinge offsets.
2. **Quasi-Steady Aerodynamics**: Unsteady aerodynamic effects (e.g., dynamic stall, wake lag) are not modeled. Airfoil coefficients are derived from static tables.
3. **Interference Effects**: Rotor wake impingement on the wing and tail is currently neglected or heavily simplified. Wing lift reduction due to the nacelle and rotor blockage is approximated.
4. **Glauert Inflow Validity**: Assumes a linear longitudinal variation of induced velocity across the rotor disk, which is a standard but simplified approximation for forward flight.
5. **Symmetric Flight**: Assumes purely longitudinal motion with no side-slip ($\beta = 0$), no roll, and no yaw, allowing lateral equations of motion to be decoupled and ignored.
