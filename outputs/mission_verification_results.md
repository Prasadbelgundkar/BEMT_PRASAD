| Feature | Verification Item | Test case / evidence | Pass? | Comments |
|---|---|---|---|---|
| Segment sequencing | Executes segments in defined order | Run Seg1 -> Drop -> Seg2 | **PASS** | Segments executed sequentially in simulation log |
| Mass continuity | Gross mass strictly follows fuel burn | Mass delta equals fuel delta across Seg1 | **PASS** | Aircraft mass strictly coupled to continuous fuel burn |
| Payload pickup/drop | Discrete mass jumps for payload events | Payload drop of 500kg event | **PASS** | Gross mass immediately decreased by 500kg correctly |
| Fuel update | Fuel mass monotonically decreases | Fuel burned in hover: 1.40 kg | **PASS** | Fuel burn updated per timestep based on required power |
| Atmospheric variation | Atmosphere updates automatically with altitude | Hover at SL vs 3000m | **PASS** | Power changed properly (SL: 1846kW -> 3000m: 2026kW) |
| Wind treatment | Wind velocity offsets axial inflow velocity | Cruise at 70m/s with 0m/s vs 20m/s tailwind | **PASS** | Rotor experiences different effective inflow and requires different power |
| Reserve fuel | Mission aborts if reserve fuel penetrated | Hover continuously until fuel < 150kg | **PASS** | Successfully caught reserve fuel threshold violation |
| Power required/available | Checks P_req vs P_avail and aero limits | Hover at 16,000m (beyond absolute ceiling) | **PASS** | Caught exceedance of available thrust/power limits |
| Failure-warning logic | Explicit errors thrown on constraint violation | Hover at 16,000m | **PASS** | MissionInfeasibleError raised immediately |
| infeasible mission: first violated constraint | Simulation halts exactly at failure point | Hover at 16,000m | **PASS** | Simulation stopped gracefully at the exact timestep of failure |
| Feasible payload mission test | Complete feasible mission runs smoothly | Takeoff -> Climb -> Cruise with payload | **PASS** | Multi-segment mission successfully trimmed without violating constraints |