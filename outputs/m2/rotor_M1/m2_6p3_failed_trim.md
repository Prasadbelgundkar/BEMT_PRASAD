# Section 6.3 -- failed-trim cases

Rotor variant 'M1' (25 deg root, -45 deg/R), MTOW unless stated. Convergence: ||r|| < 0.0001. Residuals in N and N m (body axes, about the CG).

| Case | Condition | Status | norm r | Unknowns at bound | Limit flags | FX/FY/FZ [N] | MX/MY/MZ [N m] | theta [deg] | theta0 [deg] | elevator [deg] | P req/avail [kW] | adv tip Mach | rotor stall [%] | Diagnosis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Airplane mode too slow | V=40 m/s, i_n=0, 2000 m | no_physical_solution | 3.80e-01 | - | - | -2212 / 0 / 26627 | -0 / -2431 / 0 | 11.0 | 32.9 | -20.8 | 384 / 2955 | 0.458 | 1 | no physical solution: wing at/over stall (alpha_w = 15.0 deg) cannot carry the weight; FZ residual 26.6 kN |
| Helicopter mode too fast | V=70 m/s, i_n=90, 2000 m | ok | 3.43e-11 | - | rotor_stall, tip_mach, power | 0 / 0 / 0 | -0 / 0 / 0 | -11.9 | 31.2 | 20.0 | 4736 / 2955 | 0.867 | 39 | trimmed but limited by rotor stall (39 % of loaded disk), advancing-tip Mach 0.867, power limitation (4736 kW required, 2955 kW available) |
| Helicopter mode far beyond limits | V=85 m/s, i_n=90, 2000 m | no_physical_solution | 1.24e-01 | - | - | -8445 / 0 / 2223 | 4 / 76 / 6 | -10.5 | 31.1 | 19.4 | 4930 / 2955 | 0.913 | 38 | no physical solution near the power/stall limit (best point needs 4930 kW of 2955 kW, rotor stall 38 %) |
| Airplane mode overspeed | V=140 m/s, i_n=0, 2000 m | control_saturation | 1.04e-01 | theta0 | - | -7314 / -0 / 266 | 0 / -60 / -0 | -1.2 | 65.0 | -1.7 | 2991 / 2955 | 0.601 | 0 | insufficient control authority: collective at its limit |
| Hot-and-high hover | V=0, i_n=90, 4000 m ISA+20 | ok | 1.37e-09 | - | rotor_stall, power | 0 / 0 / -0 | -0 / 0 / 0 | -0.0 | 26.9 | 3.1 | 2271 / 2235 | 0.654 | 29 | trimmed but limited by rotor stall (29 % of loaded disk), power limitation (2271 kW required, 2235 kW available) |
| Forward CG (payload +5 m) | V=60 m/s, i_n=0, 2000 m | control_saturation | 2.63e-01 | d_lon | - | -8212 / 0 / 13420 | 0 / -9905 / -0 | 5.8 | 40.2 | -25.0 | 118 / 2955 | 0.473 | 1 | insufficient control authority: pitch control (elevator / theta1c) at its limit |
| Numerical: poor seed, 6 evaluations | V=45 m/s, i_n=60, 2000 m | control_saturation | 1.05e+00 | theta0, d_lon | - | 59401 / 85 / -42176 | 3 / 13071 / -9 | -11.3 | 65.0 | 25.0 | 16609 / 2955 | 0.771 | 100 | numerical failure (poor seed / iteration budget); standard seeds: ok, |r| = 2.1e-11, feasible = True -> numerical, not physical |
