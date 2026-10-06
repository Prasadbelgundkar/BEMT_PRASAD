# Section 9 -- Milestone 2 vs Milestone 1

Rotor 'M1' (25 deg root, -45 deg/R), 2000 m ISA, 7200 kg.

## 9.1 Effect of edgewise flight, trim and conversion

| Quantity | Milestone 1 (axial model) | Milestone 2 (trimmed) | Change |
|---|---|---|---|
| Hover power, both rotors [kW] | 1982 | 1989 | +0.4 % (trim cyclic, CG offset, 6-DOF balance) |
| Hover collective [deg] | 22.49 | 22.53 (+ theta1c 0.83) | +0.04 deg |
| Hover stall | 27 % of span | 12 % of loaded disk | - |
| M1 design cruise 74.3 m/s, 7000 m, 250 RPM, power [kW] | 520 | 597 | +14.8 % (wing profile + tail + trim drag) |
| M1 design cruise collective [deg] | 58.8 | 59.2 | +0.4 deg |
| Airplane mode 85 m/s, 2000 m, 350 RPM, power [kW] | 801 | 1074 | +34.0 % (wing profile + tail + trim drag, attitude) |
| Airplane mode collective [deg] | 51.1 | 51.7 | +0.6 deg |
| Helicopter mode 40 m/s | not modelled (axial only) | P = 1729 kW, stall 14 %, theta1c = 3.6 deg, attitude -6.1 deg | edgewise power bucket and retreating-blade stall appear |
| Control inputs | collective only | collective, theta1c, differential collective / theta1s, elevator, flaperons, rudder (stick mixing) | 6-DOF trim |
| Feasible steady operating points | hover/climb and axial airplane mode only | 80 of 273 corridor points; min-power speed 50 m/s (659 kW) | conversion corridor bounded by wing stall, rotor stall, tip Mach, power, control saturation |
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
