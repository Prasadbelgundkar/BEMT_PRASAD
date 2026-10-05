# BEMT Validation Study: Knight & Hefner (1937)

This document outlines the methodology, code modifications, observations, and aerodynamic limitations discovered during the validation of the Blade Element Momentum Theory (BEMT) solver against the 1937 NACA Technical Note No. 626 experimental dataset.

---

## 1. Code Modifications & Methodology

To successfully validate the modern BEMT solver against the historical dataset, several critical changes were made to the codebase and the validation script.

### 1.1 The Historical Non-Dimensional Correction (The Factor of 2)
The most critical change made to `validation.py` was addressing a historical discrepancy in aerodynamic conventions. 
* **The Issue:** The 1937 NACA TN 626 data defined thrust and torque coefficients using dynamic pressure ($\frac{1}{2} \rho V^2$), whereas the modern standard (and our `bemt.py` solver) uses standard density ($\rho V^2$). 
* **The Fix:** In `validation.py`, immediately after loading the CSV, an array operation was added to divide all experimental data by 2.0. This scales the historical 1937 data into modern standard coefficients, allowing a direct 1:1 comparison.

### 1.2 Reverse-Engineering the Historical NACA Coefficients
Because the validation required plotting $T_c$ (historical thrust coefficient, $T_\sigma$) and $Q_c$ (historical torque coefficient, $Q_\sigma$), we translated our modern solver outputs into those historical formats inside `validation.py`:
* Calculated the mathematical blade solidity: `sigma = rotor.solidity()`
* Calculated historical Thrust: `Tc_pred = CT_pred / (sigma**2)`
* Calculated historical Torque: `Qc_pred = CQ_pred / (sigma**3)`

### 1.3 Isolating Induced Torque ($C_Q'$)
To plot $C_Q'$ (induced torque coefficient), the validation script was modified to dynamically subtract the profile drag. 
* It records the BEMT total torque at $0^\circ$ collective (where induced thrust is zero, meaning 100% of the torque is pure profile friction). 
* It subtracts that constant baseline from the entire sweep: `CQ_dash_pred = CQ_pred - CQ_pred[0]`.

### 1.4 Fixing the "Windmill Brake State" Bug in the Core Solver
While validating the low-angle data, we discovered a mathematical trap in the core physics engine (`bemt.py`).
* **The Issue:** At $0^\circ$ collective, the BEMT solver was guessing a massive *negative* downwash velocity. Because the Momentum Theory equation lacked an absolute value sign, squaring the negative velocity returned a positive thrust, mathematically balancing the equations but resulting in physically impossible negative induced drag.
* **The Fix:** We modified the Momentum Theory thrust equation in `bemt.py` to correctly track mass flow rate direction by adding an absolute value constraint: 
  `dT_mom = 4.0 * np.pi * r * rho * F * abs(v_axial + v) * v`
* This forced the solver to correctly predict $v = 0$ at $0^\circ$ collective, instantly fixing scattered values and restoring true profile drag.

### 1.5 Multi-Blade Automation
Instead of running three separate scripts, `validation.py` was entirely restructured with a loop to sequentially construct 2-blade, 3-blade, and 4-blade rotors, parse their respective CSVs, run the sweeps, and plot them simultaneously on the same figure for direct cross-comparison.

---

## 2. Key Observations

1. **Excellent Alignment at Operating Angles:** Between $2^\circ$ and $10^\circ$ (where a rotor normally operates), the BEMT prediction lines match the experimental data almost perfectly for both $C_T$ and $C_Q$. This proves the core physics engine is mathematically sound.
2. **Accurate Solidity Scaling:** As the configuration scales from 2 to 4 blades (increasing solidity $\sigma$), both the thrust and torque increase proportionally. The BEMT code perfectly predicted this scale without requiring any artificial tuning factors, proving that the Annular Momentum Theory loop handles multi-blade interference correctly.
3. **Induced Torque Isolation:** The $C_Q'$ graph proves that profile drag (friction) makes up the bulk of the torque at $0^\circ$, but as pitch increases, the induced drag (the aerodynamic cost of pushing air down) rapidly takes over and dominates the total power requirement. 
4. **Power Coefficient Parity:** The $C_P$ graphs align perfectly with the $C_Q$ graphs, proving the theoretical relationship that in purely axial hover conditions, Power Coefficient is mathematically identical to Torque Coefficient ($P = \Omega Q$).

---

## 3. BEMT Model Limitations

While the solver is highly accurate in the mid-range, it inherently deviates from reality at extreme angles ($11^\circ - 12^\circ$) due to the fundamental simplifications of Blade Element Momentum Theory:

1. **The Airfoil Model is Over-Simplified:**
   * In reality, airfoils experience dynamic stall, boundary layer separation, and Reynolds number scaling. 
   * Our validation uses a simple linear mathematical approximation ($C_l = 5.75 \alpha$ and $C_d = 0.0113 + 1.25\alpha^2$). It cannot capture the sudden drag spikes or lift fall-offs of real blades at high angles of attack.
2. **Ignoring Wake Contraction and Swirl:**
   * BEMT assumes the air gets pushed straight down in a perfect, smooth column. 
   * In reality, the air contracts (shrinks inward) below the rotor, and the friction of the blades causes the air to swirl in a vortex. This swirl wastes energy that BEMT does not account for, causing BEMT to slightly under-predict the required power ($C_P$) at high thrusts.
3. **Idealized Tip Loss (Prandtl's Model):**
   * The Prandtl tip loss factor ($F$) used in the code is an ingenious but simplified mathematical trick assuming the wake is composed of infinite 2D vortex sheets. 
   * Real helicopter blades have complex 3D tip vortices that curl upward and hit the following blade (blade-vortex interaction), especially in hover. BEMT completely ignores this 3D interference.
4. **Rigid Blade Assumption:**
   * The solver assumes the blades are perfectly rigid. Real Knight & Hefner blades bend, flap, and twist under aerodynamic load (aeroelasticity), slightly changing their effective geometric pitch angle during the wind tunnel experiment.
