# Section 8.1 -- transition-mission definition

Rotor 'refined' (12 deg root, -30 deg/R). ISA (dISA = 0). Wind: calm at the pad (hover/vertical segments), along-track headwind building to 5 m/s aloft during conversion. Speed changes use trapezoidal acceleration profiles (15 % ramps). Outbound start: 2000 m, MTOW 7200 kg (fuel 1500 kg). Inbound start (arbitrary point): 2250 m, fuel 900 kg. Reserve fuel 450 kg. Controls from the online 6-DOF trim at every step. Conversion path (V [m/s], i_n [deg]): [(0.0, 90.0), (10.0, 90.0), (20.0, 85.0), (30.0, 80.0), (40.0, 75.0), (50.0, 60.0), (55.0, 45.0), (60.0, 30.0), (65.0, 15.0), (70.0, 0.0)]; reconversion path: [(0.0, 90.0), (12.0, 90.0), (25.0, 92.0), (35.0, 90.0), (42.0, 85.0), (48.0, 72.0), (54.0, 55.0), (60.0, 35.0), (65.0, 15.0), (70.0, 0.0)].

| Leg | Segment | Type | Duration [s] | dt [s] | Airspeed schedule [m/s] | Climb rate [m/s] | Nacelle schedule [deg] | RPM schedule |
|---|---|---|---|---|---|---|---|---|
| Outbound | Take-off hover | hover | 20 | 5 | 0 -> 0 | +0.0 (ramped) | 90 -> 90 | 540 -> 540 |
| Outbound | Vertical climb | vertical | 60 | 5 | 0 -> 0 | +2.5 (ramped) | 90 -> 90 | 540 -> 540 |
| Outbound | Conversion (hover -> airplane) | conversion | 100 | 2 | 0 -> 70 | +0.4 (ramped) | conversion path n(V) | 540 -> 540 |
| Outbound | Airplane accel. + RPM reduction | airplane | 60 | 5 | 70 -> 85 | +0.8 (ramped) | 0 -> 0 | 540 -> 420 |
| Outbound | Airplane cruise | airplane | 60 | 10 | 85 -> 85 | +0.0 (ramped) | 0 -> 0 | 420 -> 420 |
| Inbound | Airplane cruise | airplane | 30 | 10 | 85 -> 85 | +0.0 (ramped) | 0 -> 0 | 420 -> 420 |
| Inbound | Decel. + RPM increase | airplane | 60 | 5 | 85 -> 70 | -0.8 (ramped) | 0 -> 0 | 420 -> 540 |
| Inbound | Reconversion (airplane -> hover) | conversion | 130 | 2 | 70 -> 0 | +0.0 (ramped) | reconversion path n(V) | 540 -> 540 |
| Inbound | Hover | hover | 20 | 5 | 0 -> 0 | +0.0 (ramped) | 90 -> 90 | 540 -> 540 |
| Inbound | Vertical descent | vertical | 60 | 5 | 0 -> 0 | -1.5 (ramped) | 90 -> 90 | 540 -> 540 |
| Inbound | Landing hover | hover | 15 | 5 | 0 -> 0 | +0.0 (ramped) | 90 -> 90 | 540 -> 540 |
