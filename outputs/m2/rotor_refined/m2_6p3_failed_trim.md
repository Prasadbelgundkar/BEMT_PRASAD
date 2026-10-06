# Section 6.3 -- failed-trim cases

Rotor variant 'refined' (12 deg root, -30 deg/R), MTOW unless stated. Convergence: ||r|| < 0.0001. Residuals in N and N m (body axes, about the CG).

| Case | Condition | Status | norm r | Unknowns at bound | Limit flags | FX/FY/FZ [N] | MX/MY/MZ [N m] | theta [deg] | theta0 [deg] | elevator [deg] | P req/avail [kW] | adv tip Mach | rotor stall [%] | Diagnosis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Airplane mode too slow | V=40 m/s, i_n=0, 2000 m | no_physical_solution | 3.86e-01 | - | - | -2198 / 0 / 27038 | -0 / -2472 / 0 | 11.0 | 35.6 | -20.8 | 397 / 2955 | 0.458 | 2 | no physical solution: wing at/over stall (alpha_w = 15.0 deg) cannot carry the weight; FZ residual 27.0 kN |
| Helicopter mode too fast | V=70 m/s, i_n=90, 2000 m | ok | 4.04e-11 | - | rotor_stall, tip_mach, power | 0 / -0 / 0 | -0 / 0 / 0 | -10.2 | 28.5 | 15.2 | 3331 / 2955 | 0.868 | 21 | trimmed but limited by rotor stall (21 % of loaded disk), advancing-tip Mach 0.868, power limitation (3331 kW required, 2955 kW available) |
| Helicopter mode far beyond limits | V=85 m/s, i_n=90, 2000 m | no_physical_solution | 7.41e-02 | - | - | -5057 / 0 / 1352 | -1 / 77 / -2 | -11.0 | 33.4 | 20.5 | 4978 / 2955 | 0.912 | 38 | no physical solution near the power/stall limit (best point needs 4978 kW of 2955 kW, rotor stall 38 %) |
| Airplane mode overspeed | V=140 m/s, i_n=0, 2000 m | control_saturation | 4.57e-01 | theta0 | - | -32271 / -0 / 1265 | -0 / -224 / 0 | -1.3 | 65.0 | 0.2 | -498 / 2955 | 0.601 | 0 | insufficient control authority: collective at its limit |
| Hot-and-high hover | V=0, i_n=90, 4000 m ISA+20 | ok | 1.37e-06 | - | power | -0 / -0 / 0 | 0 / -0 / -0 | -0.0 | 28.5 | 2.7 | 2181 / 2235 | 0.654 | 3 | trimmed but limited by power limitation (2181 kW required, 2235 kW available) |
| Forward CG (payload +5 m) | V=60 m/s, i_n=0, 2000 m | control_saturation | 2.71e-01 | d_lon | - | -8170 / 0 / 13961 | 0 / -10169 / -0 | 5.8 | 43.2 | -25.0 | 166 / 2955 | 0.473 | 3 | insufficient control authority: pitch control (elevator / theta1c) at its limit |
| Numerical: poor seed, 6 evaluations | V=45 m/s, i_n=60, 2000 m | control_saturation | 1.10e+00 | theta0 | - | 63486 / -182 / -43925 | 19 / 4695 / -10 | -11.1 | 65.0 | 24.1 | 15805 / 2955 | 0.771 | 100 | numerical failure (poor seed / iteration budget); standard seeds: ok, |r| = 1.9e-08, feasible = True -> numerical, not physical |
