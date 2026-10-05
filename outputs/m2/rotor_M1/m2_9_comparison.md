# Section 9 -- Milestone 2 vs Milestone 1

Rotor 'M1' (25 deg root, -45 deg/R), 2000 m ISA, 7200 kg.

## 9.1 Effect of edgewise flight, trim and conversion

| Quantity | Milestone 1 (axial model) | Milestone 2 (trimmed) | Change |
|---|---|---|---|
| Hover power, both rotors [kW] | 1965 | 1980 | +0.8 % (trim cyclic, CG offset, 6-DOF balance) |
| Hover collective [deg] | 25.46 | 25.57 (+ theta1c 1.39) | +0.12 deg |
| Hover stall | 43 % of span | 24 % of loaded disk | - |
| Airplane mode 85 m/s, 420 RPM, power [kW] | 889 | 1160 | +30.5 % (wing profile + tail + trim drag, attitude) |
| Airplane mode collective [deg] | 45.9 | 46.3 | +0.5 deg |
| Helicopter mode 40 m/s | not modelled (axial only) | P = 1742 kW, stall 21 %, theta1c = 4.8 deg, attitude -6.3 deg | edgewise power bucket and retreating-blade stall appear |
| Control inputs | collective only | collective, theta1c, differential collective / theta1s, elevator, flaperons, rudder (stick mixing) | 6-DOF trim |
| Feasible steady operating points | hover/climb and axial airplane mode only | 66 of 273 corridor points; min-power speed 50 m/s (587 kW) | conversion corridor bounded by wing stall, rotor stall, tip Mach, power, control saturation |
| Lowest airplane-mode speed (i_n = 0) | not checked (M1 cruise 40 m/s) | 55 m/s (wing stall / no trim below) | - |

## 9.2 Trade-off data: rotor variants

| Metric | 'M1' rotor (25/-45, 500 RPM) | 'refined' rotor (12/-30, 540 RPM) |
|---|---|---|
| Hover power (2000 m, MTOW) [kW] | 1980 | 1903 |
| Hover stalled loaded area [%] | 24 | 0 |
| Hover stall margin [deg] | -6.2 | +1.5 |
| Helicopter mode 40 m/s stalled area [%] | 21 | 6 |
| Airplane mode 85 m/s power [kW] | 1160 | 1286 |
| Airplane mode 85 m/s collective [deg] | 46.3 | 49.4 |
| Feasible corridor points (of 273) | 66 | 105 |
| Max feasible speed at i_n = 90 [m/s] | n/a | 30 |
