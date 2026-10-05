# Section 6.3 -- failed-trim cases

Rotor variant 'M1' (25 deg root, -45 deg/R), MTOW unless stated. Convergence: ||r|| < 0.0001. Residuals in N and N m (body axes, about the CG).

| Case | Condition | Status | norm r | Unknowns at bound | Limit flags | FX/FY/FZ [N] | MX/MY/MZ [N m] | theta [deg] | theta0 [deg] | elevator [deg] | P req/avail [kW] | adv tip Mach | rotor stall [%] | Diagnosis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Airplane mode too slow | V=40 m/s, i_n=0, 2000 m | no_physical_solution | 3.84e-01 | - | - | -2459 / 0 / 26921 | -0 / -2335 / 0 | 11.0 | 28.6 | -21.0 | 427 / 2284 | 0.539 | 0 | no physical solution: wing at/over stall (alpha_w = 15.0 deg) cannot carry the weight; FZ residual 26.9 kN |
| Helicopter mode too fast | V=70 m/s, i_n=90, 2000 m | no_physical_solution | 7.70e-02 | - | - | -5215 / -0 / 1531 | 0 / 74 / 0 | -10.0 | 32.0 | 21.1 | 3734 / 2284 | 0.808 | 44 | no physical solution near the power/stall limit (best point needs 3734 kW of 2284 kW, rotor stall 44 %) |
| Helicopter mode far beyond limits | V=85 m/s, i_n=90, 2000 m | no_physical_solution | 1.77e-01 | - | - | -12220 / 0 / 2481 | 4 / 6 / -4 | -7.9 | 27.2 | 15.0 | 2851 / 2284 | 0.853 | 30 | no physical solution near the power/stall limit (best point needs 2851 kW of 2284 kW, rotor stall 30 %) |
| Airplane mode overspeed | V=120 m/s, i_n=0, 2000 m | control_saturation | 1.45e-01 | theta0 | - | -10221 / -0 / 318 | 0 / -83 / -0 | -0.6 | 55.0 | -1.7 | 1476 / 2284 | 0.616 | 0 | insufficient control authority: collective at its limit |
| Hot-and-high hover | V=0, i_n=90, 3000 m ISA+20 | ok | 2.23e-11 | - | rotor_stall, power | -0 / -0 / 0 | -0 / -0 / 0 | -0.0 | 29.2 | 4.8 | 2224 / 1954 | 0.588 | 39 | trimmed but limited by rotor stall (39 % of loaded disk), power limitation (2224 kW required, 1954 kW available) |
| Forward CG (payload +5 m) | V=60 m/s, i_n=0, 2000 m | control_saturation | 2.63e-01 | d_lon | - | -8438 / -0 / 13334 | -0 / -9821 / -0 | 5.9 | 35.6 | -25.0 | 163 / 2284 | 0.551 | 1 | insufficient control authority: pitch control (elevator / theta1c) at its limit |
| Numerical: poor seed, 6 evaluations | V=45 m/s, i_n=60, 2000 m | control_saturation | 7.79e-01 | theta0 | - | 47660 / -7 / -27365 | 12 / 1899 / 5 | -8.1 | 55.0 | 23.8 | 9090 / 2284 | 0.715 | 92 | numerical failure (poor seed / iteration budget); standard seeds: ok, |r| = 2.4e-07, feasible = True -> numerical, not physical |
