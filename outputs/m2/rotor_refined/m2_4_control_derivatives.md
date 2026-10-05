# Section 4 -- control derivatives of one rotor (right rotor, CCW from above)

Central differences at the trim point. Forces in kN/deg, moments (about the CG) in kN m/deg, power in kW/deg; body axes x fwd, y right, z down.

## Condition A: helicopter-like edgewise flight

V=30 m/s, i_n=90 deg, h=2000 m ISA, 540 RPM, m=7200 kg; trim: theta0=22.45, theta1c=4.07, theta1s=0.00 deg, alpha_body=-4.17 deg, alpha_shaft=4.17 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | -0.166 | -0.092 | -2.953 | -30.130 | +2.593 | +3.136 | +90.259 |
| theta1c | -0.001 | -0.371 | +0.041 | -0.478 | -6.855 | +0.311 | +20.929 |
| theta1s | -0.370 | +0.004 | -0.599 | -12.458 | +1.380 | +3.740 | +14.946 |

At trim: T = 35.47 kN, H = 1.99 kN, Y = -1.31 kN, Q = 12.97 kN m, P = 733 kW of 1142 kW available, mu = 0.139, adv. tip Mach = 0.737, reverse-flow area = 0.3 %, stalled loaded area = 5.0 %, stall margin = -6.0 deg.

## Condition B: intermediate conversion

V=45 m/s, i_n=60 deg, h=2000 m ISA, 540 RPM, m=7200 kg; trim: theta0=19.19, theta1c=-0.71, theta1s=0.00 deg, alpha_body=8.73 deg, alpha_shaft=21.27 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | +1.524 | +0.069 | -3.156 | -33.106 | +1.270 | -14.839 | +73.455 |
| theta1c | +0.003 | -0.390 | -0.004 | -0.909 | -7.110 | -0.415 | -1.571 |
| theta1s | +0.203 | +0.023 | -1.181 | -17.518 | +1.131 | -5.277 | +16.986 |

At trim: T = 12.46 kN, H = 2.30 kN, Y = 0.49 kN, Q = 5.36 kN m, P = 303 kW of 1142 kW available, mu = 0.195, adv. tip Mach = 0.774, reverse-flow area = 1.7 %, stalled loaded area = 0.0 %, stall margin = +2.4 deg.

