# Section 4 -- control derivatives of one rotor (right rotor, CCW from above)

Central differences at the trim point. Forces in kN/deg, moments (about the CG) in kN m/deg, power in kW/deg; body axes x fwd, y right, z down.

## Condition A: helicopter-like edgewise flight

V=30 m/s, i_n=90 deg, h=2000 m ISA, 550 RPM, m=7200 kg; trim: theta0=22.02, theta1c=3.74, theta1s=0.00 deg, alpha_body=-4.10 deg, alpha_shaft=4.10 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | -0.170 | -0.089 | -3.078 | -31.393 | +2.587 | +3.189 | +92.330 |
| theta1c | -0.002 | -0.377 | +0.037 | -0.529 | -7.176 | +0.341 | +21.958 |
| theta1s | -0.377 | +0.004 | -0.632 | -13.103 | +1.382 | +3.814 | +15.339 |

At trim: T = 35.45 kN, H = 1.97 kN, Y = -1.20 kN, Q = 12.66 kN m, P = 729 kW of 1478 kW available, mu = 0.137, adv. tip Mach = 0.749, reverse-flow area = 0.3 %, stalled loaded area = 4.2 %, stall margin = -5.4 deg.

## Condition B: intermediate conversion

V=45 m/s, i_n=60 deg, h=2000 m ISA, 550 RPM, m=7200 kg; trim: theta0=18.94, theta1c=-0.82, theta1s=-0.00 deg, alpha_body=8.80 deg, alpha_shaft=21.20 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | +1.583 | +0.074 | -3.259 | -34.139 | +1.249 | -15.421 | +75.189 |
| theta1c | +0.003 | -0.396 | -0.004 | -0.918 | -7.419 | -0.414 | -1.823 |
| theta1s | +0.210 | +0.025 | -1.204 | -17.989 | +1.125 | -5.488 | +17.123 |

At trim: T = 12.35 kN, H = 2.29 kN, Y = 0.54 kN, Q = 5.34 kN m, P = 307 kW of 1478 kW available, mu = 0.192, adv. tip Mach = 0.786, reverse-flow area = 1.6 %, stalled loaded area = 0.0 %, stall margin = +2.5 deg.

