# BEMT Governing Equations and Logic

## 1. Core Kinematics and Validation Airfoil
Before calculating forces on a blade element, you must establish the local velocities and the validation baseline.

*   Tangential velocity: $U_T = \Omega r$[cite: 3]
*   Perpendicular velocity: $U_P = V + v$[cite: 3]
*   Validation Lift Coefficient: $C_l = 5.75 \alpha$[cite: 1]
*   Validation Drag Coefficient: $C_d = 0.0113 + 1.25 \alpha^2$[cite: 1]

## 2. Multi-Airfoil Blending and Compressibility
For practical tiltrotor blades featuring varying airfoils and high tip speeds, apply these intermediate transformations.

*   Interpolation Weighting: $W = \frac{r - r_{\text{inner}}}{r_{\text{outer}} - r_{\text{inner}}}$[cite: 5]
*   Blended Lift: $C_{l,\text{blended}} = C_{l,\text{inner}} + W \times (C_{l,\text{outer}} - C_{l,\text{inner}})$[cite: 5]
*   Prandtl-Glauert Lift Correction: $C_l = \frac{C_{l,\text{incompressible}}}{\sqrt{1-M_\infty^2}}$[cite: 2]
*   Prandtl-Glauert Drag Correction: $C_d = \frac{C_{d,\text{incompressible}}}{\sqrt{1-M_\infty^2}}$[cite: 2]

## 3. Force Balance (The BEMT Iteration)
The core of your code will loop through these formulas to find the converged induced velocity ($v$) for every blade element.

*   Momentum Theory Thrust: $dT = 4\pi\rho r(V+v)v dr$[cite: 2]
*   Blade Element Thrust: $dT = \frac{1}{2}\rho(U_T^2 + U_P^2)c(C_l \cos\phi - C_d \sin\phi)dr$[cite: 3]
*   Equate both $dT$ expressions iteratively to solve for $v$[cite: 2, 4].
*   Apply the Prandtl tip loss factor ($F$) by modifying the sectional inflow $(V+v)$ by $F$ within the Momentum Theory expression[cite: 2, 4].

## 4. Rotor Performance Integration
Once the induced velocity $v$ has converged for all radial strips, calculate the macroscopic rotor forces.

| Performance Metric | Governing Integral |
| :--- | :--- |
| **Total Thrust ($T$)** | $b \int_{R_C}^{R} dT$[cite: 3] |
| **Sectional In-Plane Force ($dF_x$)** | $\frac{1}{2}\rho(U_T^2 + U_P^2)c(C_d \cos\phi + C_l \sin\phi)dr$[cite: 3] |
| **Total Torque ($Q$)** | $b \int_{R_C}^{R} r dF_x$[cite: 3] |
| **Total Power ($P$)** | $Q \Omega$[cite: 3] |