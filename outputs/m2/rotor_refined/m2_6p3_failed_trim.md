# Section 6.3 -- failed-trim cases

Rotor variant 'refined' (12 deg root, -30 deg/R), MTOW unless stated. Convergence: ||r|| < 0.0001. Residuals in N and N m (body axes, about the CG).

| Case | Condition | Status | norm r | Unknowns at bound | Limit flags | FX/FY/FZ [N] | MX/MY/MZ [N m] | theta [deg] | theta0 [deg] | elevator [deg] | P req/avail [kW] | adv tip Mach | rotor stall [%] | Diagnosis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Airplane mode too slow | V=40 m/s, i_n=0, 2000 m | no_physical_solution | 3.89e-01 | - | - | -1270 / -0 / 27045 | 0 / -4476 / -0 | 11.0 | 31.6 | -19.7 | 471 / 2284 | 0.539 | 1 | no physical solution: wing at/over stall (alpha_w = 15.0 deg) cannot carry the weight; FZ residual 27.0 kN |
| Helicopter mode too fast | V=70 m/s, i_n=90, 2000 m | ok | 3.98e-11 | - | rotor_stall, tip_mach, power | 0 / 0 / 0 | -0 / 0 / -0 | -10.6 | 30.1 | 17.2 | 3596 / 2284 | 0.856 | 27 | trimmed but limited by rotor stall (27 % of loaded disk), advancing-tip Mach 0.856, power limitation (3596 kW required, 2284 kW available) |
| Helicopter mode far beyond limits | V=85 m/s, i_n=90, 2000 m | no_physical_solution | 9.25e-02 | - | - | -6329 / 0 / 1623 | -1 / 77 / -2 | -10.4 | 32.6 | 19.7 | 4501 / 2284 | 0.901 | 37 | no physical solution near the power/stall limit (best point needs 4501 kW of 2284 kW, rotor stall 37 %) |
| Airplane mode overspeed | V=120 m/s, i_n=0, 2000 m | control_saturation | 5.16e-01 | theta0 | - | -36410 / 0 / 1331 | -0 / -237 / 0 | -0.8 | 55.0 | 1.2 | -1468 / 2284 | 0.615 | 1 | insufficient control authority: collective at its limit |
| Hot-and-high hover | V=0, i_n=90, 3000 m ISA+20 | ok | 1.56e-11 | - | power | 0 / 0 / -0 | -0 / 0 / -0 | -0.0 | 27.6 | 3.1 | 2072 / 1954 | 0.634 | 0 | trimmed but limited by power limitation (2072 kW required, 1954 kW available) |
| Forward CG (payload +5 m) | V=60 m/s, i_n=0, 2000 m | control_saturation | 2.69e-01 | d_lon | - | -8453 / -0 / 13729 | -0 / -10001 / -0 | 5.9 | 38.5 | -25.0 | 204 / 2284 | 0.551 | 3 | insufficient control authority: pitch control (elevator / theta1c) at its limit |
| Numerical: poor seed, 6 evaluations | V=45 m/s, i_n=60, 2000 m | control_saturation | 1.07e+00 | theta0 | - | 63992 / -134 / -40669 | 61 / 3039 / 143 | -9.8 | 55.0 | 18.6 | 10827 / 2284 | 0.761 | 91 | numerical failure (poor seed / iteration budget); standard seeds: ok, |r| = 1.5e-11, feasible = True -> numerical, not physical |
