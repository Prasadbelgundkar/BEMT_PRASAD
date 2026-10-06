# Section 9 -- Milestone 2 vs Milestone 1

Rotor 'refined' (12 deg root, -30 deg/R), 2000 m ISA, 7200 kg.

## 9.1 Effect of edgewise flight, trim and conversion

| Quantity | Milestone 1 (axial model) | Milestone 2 (trimmed) | Change |
|---|---|---|---|
| Hover power, both rotors [kW] | 1891 | 1903 | +0.6 % (trim cyclic, CG offset, 6-DOF balance) |
| Hover collective [deg] | 24.83 | 24.92 (+ theta1c 0.80) | +0.09 deg |
| Hover stall | 0 % of span | 0 % of loaded disk | - |
| M1 design cruise 74.3 m/s, 7000 m, 250 RPM, power [kW] | 544 | 620 | +14.1 % (wing profile + tail + trim drag) |
| M1 design cruise collective [deg] | 62.2 | 62.6 | +0.4 deg |
| Airplane mode 85 m/s, 2000 m, 350 RPM, power [kW] | 902 | 1171 | +29.8 % (wing profile + tail + trim drag, attitude) |
| Airplane mode collective [deg] | 54.3 | 55.0 | +0.7 deg |
| Helicopter mode 40 m/s | not modelled (axial only) | P = 1465 kW, stall 5 %, theta1c = 3.7 deg, attitude -5.4 deg | edgewise power bucket and retreating-blade stall appear |
| Control inputs | collective only | collective, theta1c, differential collective / theta1s, elevator, flaperons, rudder (stick mixing) | 6-DOF trim |
| Feasible steady operating points | hover/climb and axial airplane mode only | 103 of 273 corridor points; min-power speed 45 m/s (576 kW) | conversion corridor bounded by wing stall, rotor stall, tip Mach, power, control saturation |
| Lowest airplane-mode speed (i_n = 0) | not checked (M1 cruise 74.3 m/s at 7000 m) | 55 m/s (wing stall / no trim below) | - |

## 9.2 Trade-off data: rotor variants

| Metric | 'M1' rotor (25/-45, 550 RPM) | 'refined' rotor (12/-30, 550 RPM) |
|---|---|---|
| Hover power (2000 m, MTOW) [kW] | 1989 | 1903 |
| Hover stalled loaded area [%] | 12 | 0 |
| Hover stall margin [deg] | -2.5 | +2.0 |
| Helicopter mode 40 m/s stalled area [%] | 14 | 5 |
| Airplane mode 85 m/s, 350 RPM power [kW] | 1074 | 1171 |
| Airplane mode 85 m/s collective [deg] | 51.7 | 55.0 |
| Feasible corridor points (of 273) | 80 | 103 |
| Highest nacelle angle with a feasible point [deg] | 67.5 | 90.0 |
| Max feasible speed at i_n = 90 [m/s] | no feasible point | 35 |
