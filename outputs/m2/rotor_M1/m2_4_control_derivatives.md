# Section 4 -- control derivatives of one rotor (right rotor, CCW from above)

Central differences at the trim point. Forces in kN/deg, moments (about the CG) in kN m/deg, power in kW/deg; body axes x fwd, y right, z down.

## Condition A: helicopter-like edgewise flight

V=30 m/s, i_n=90 deg, h=2000 m ISA, 550 RPM, m=7200 kg; trim: theta0=20.29, theta1c=3.44, theta1s=0.00 deg, alpha_body=-4.57 deg, alpha_shaft=4.57 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | -0.129 | -0.094 | -2.867 | -29.078 | +2.414 | +2.723 | +87.688 |
| theta1c | -0.006 | -0.339 | -0.034 | -1.161 | -6.824 | +0.309 | +17.440 |
| theta1s | -0.344 | +0.000 | -0.449 | -11.030 | +1.248 | +3.447 | +12.391 |

At trim: T = 35.79 kN, H = 2.29 kN, Y = -1.12 kN, Q = 14.25 kN m, P = 821 kW of 1478 kW available, mu = 0.137, adv. tip Mach = 0.749, reverse-flow area = 0.3 %, stalled loaded area = 12.6 %, stall margin = -14.0 deg.

## Condition B: intermediate conversion

V=45 m/s, i_n=60 deg, h=2000 m ISA, 550 RPM, m=7200 kg; trim: theta0=17.21, theta1c=-0.84, theta1s=0.00 deg, alpha_body=7.91 deg, alpha_shaft=22.09 deg

| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |
|---|---|---|---|---|---|---|---|
| collective | +1.567 | +0.048 | -3.199 | -33.533 | +1.135 | -15.203 | +77.162 |
| theta1c | +0.013 | -0.406 | -0.043 | -1.328 | -7.359 | -0.551 | -3.014 |
| theta1s | +0.169 | +0.010 | -1.101 | -16.942 | +1.071 | -5.047 | +17.406 |

At trim: T = 13.74 kN, H = 3.26 kN, Y = 0.60 kN, Q = 7.25 kN m, P = 417 kW of 1478 kW available, mu = 0.191, adv. tip Mach = 0.786, reverse-flow area = 1.5 %, stalled loaded area = 2.7 %, stall margin = -4.7 deg.

