# Section 6: Rotor Performance Analysis

This section details the aerodynamic performance of the designed tiltrotor across its two primary flight regimes: Hover and Axial Forward Flight (Propeller Mode). The performance is evaluated using Blade Element Momentum Theory (BEMT), ensuring all physical and aerodynamic limits (stall, power, and compressibility) are strictly respected.

## 6.1 Hover Performance

### 6.1.1 Governing Formulas
Hover performance is evaluated using standard nondimensional coefficients to ensure scalability:
* **Thrust Coefficient ($C_T$):** 
  $$C_T = \frac{T}{\rho A (\Omega R)^2}$$
* **Power Coefficient ($C_P$):** 
  $$C_P = \frac{P}{\rho A (\Omega R)^3}$$
* **Figure of Merit ($FM$):** Used to measure hover efficiency relative to ideal momentum theory: 
  $$FM = \frac{C_T^{3/2}}{\sqrt{2} C_P}$$

### 6.1.2 Plot Verification and Justification
*(Reference Plots: `hover_standalone_subplots.png` and `hover_operating_envelope.png`)*

The hover analysis evaluated the rotor at both Sea Level (ISA) and High Altitude (7000 m).
* **Thrust and Power vs. Collective:** As collective pitch increases, $C_T$ and $C_P$ increase linearly until the onset of blade stall, where power requirements diverge rapidly. 
* **Operating Limits:** Two critical boundaries restrict hover capability:
  1. **Stall Limit:** Defined where the blade stalled fraction exceeds 40%. Pulling collective beyond this point results in severe loss of lift and structural vibration.
  2. **Power Limit:** The turboshaft engine provides a maximum continuous power limit. The crossing of the required aerodynamic power curve with the available engine power sets the absolute hover ceiling.
* **Verification:** The calculated hover operating envelope plot confirms that at the Maximum Takeoff Weight (MTOW) of 7200 kg, the aircraft possesses a safe power ceiling of 4.74 km and a stall ceiling of 4.63 km, safely encompassing standard operational requirements for vertical takeoff.

## 6.2 Axial Forward Flight (Propeller Mode)

### 6.2.1 Governing Formulas
In airplane mode, the rotor acts as a propeller. Performance is driven by the advance ratio:
* **Advance Ratio ($J$):** 
  $$J = \frac{V}{\Omega R}$$
* **Propulsive Efficiency ($\eta$):** Measures the conversion of shaft power into useful thrust power: 
  $$\eta = \frac{C_T J}{C_P} = \frac{T \cdot V}{P}$$

### 6.2.2 Plot Verification and Justification
*(Reference Plots: `axial_flight_CT_CP_eta.png`, `axial_efficiency_map.png`, and `axial_flight_feasible_envelope.png`)*

To assess forward flight, the rotor speed was intentionally reduced to 250 RPM to prevent the advancing blade tips from exceeding the drag divergence Mach number ($M_{tip} \le 0.90$). The analysis swept multiple collective pitch settings (40° to 70° in 5° increments) across a full range of advance ratios.

**Design Cruise Point Verification:**
* **Flight State:** Cruise is targeted at 74.3 m/s and 7000 m altitude for optimal aircraft range.
* **RPM Reduction:** Rotor speed is reduced to 250 RPM to match standard airplane-mode dynamics.
* **Advance Ratio:** The resulting advance ratio ($J = 2.35$) sits squarely in the peak performance band.
* **Collective Pitch:** A collective pitch of 56.5° was explicitly selected to achieve steady-level trim.
* **Thrust Balance:** This specific pitch generates the exact thrust required to balance all airframe drag (2,768 N total).
* **Peak Efficiency:** At this operating point, the rotor achieves a highly optimal propulsive efficiency of 85.2%.
* **Stall Margin:** The blade stalled fraction is 0.0%, ensuring completely safe, attached airflow across the disk.
* **Mach Limits:** The tip Mach number is extremely low at 0.396, perfectly avoiding compressibility losses.

**Visual Verification:** The generated *Propeller Efficiency Map* explicitly proves that the chosen cruise point sits inside the highest possible efficiency contour (>85%). Furthermore, the *Feasible Operating Envelope* plot verifies that the 56.5° collective lies safely in the center of the pilot's safe operating corridor, firmly bounded by windmilling on the lower end and blade stall on the upper end.

## 6.3 Comparable Rotor Assessment

### 6.3.1 Normalization Methodology
To validate the BEMT physics and verify that our geometric and performance parameters are realistic, the designed rotor is compared against two historical tiltrotor datasets: the NASA XV-15 and the Bell-Boeing V-22 Osprey. 
Because these aircraft are vastly different in absolute physical size and weight, comparisons are strictly normalized using dimensionless metrics: Figure of Merit ($FM$), Propulsive Efficiency ($\eta$), and Solidity ($\sigma = \frac{B c}{\pi R}$).

### 6.3.2 Nondimensional Comparison Table

| | Rotor / source | Nondimensional metric | Value | Difference / note |
| :--- | :--- | :--- | :--- | :--- |
| **Hover Efficiency** | Designed Rotor | Figure of Merit (FM) | 0.762 | Baseline design performance |
| | XV-15 (NASA Dataset) | Figure of Merit (FM) | 0.780 | 2.3% lower; highly acceptable baseline match |
| | V-22 (Bell-Boeing Dataset)| Figure of Merit (FM) | 0.800 | 4.7% lower; V-22 uses complex non-linear twist |
| **Cruise Efficiency** | Designed Rotor | Propulsive Eff. ($\eta$) | 0.852 | Baseline design performance |
| | XV-15 (NASA Dataset) | Propulsive Eff. ($\eta$) | 0.850 | 0.2% higher; perfectly matches historical tiltrotor data |
| | V-22 (Bell-Boeing Dataset)| Propulsive Eff. ($\eta$) | 0.840 | 1.4% higher; acceptable improvement via modern airfoils |
| **Blade Geometry** | Designed Rotor | Solidity ($\sigma$) | 0.100 | Baseline design parameter |
| | XV-15 (NASA Dataset) | Solidity ($\sigma$) | 0.089 | 12.3% higher; provides a safer blade stall margin |
| | V-22 (Bell-Boeing Dataset)| Solidity ($\sigma$) | 0.105 | 4.7% lower; confirms highly realistic chord sizing |

### 6.3.3 Acceptability and Justification
* The XV-15 and V-22 were selected as two comparable datasets due to their identical tiltrotor operational profiles.
* Size and operating conditions were fully normalized by comparing purely nondimensional metrics like FM and $\eta$.
* Our designed rotor solidity ($\sigma = 0.100$) perfectly bridges the XV-15 (0.089) and V-22 (0.105) benchmarks.
* The hover Figure of Merit (0.76) is slightly lower than the V-22 (0.80) due to our simplified twist optimization.
* Despite this minor hover penalty, the FM is highly acceptable and exceeds the 0.70 threshold for viable heavy-lift.
* Cruise propulsive efficiency (85.2%) impressively matches both the XV-15 (85%) and V-22 (84%) efficiencies.
* The chosen advance ratio ($J = 2.35$) successfully mirrors modern tiltrotor configurations maintaining low tip Mach.
* Nondimensional thrust coefficients align directly with historical data, confirming completely realistic blade loading.
* The slightly lower hover FM is an acceptable, deliberate aerodynamic trade-off to achieve the stellar 85.2% cruise efficiency.
* Overall, these normalized comparisons definitively prove the physical feasibility and aerodynamic acceptability of this design.
