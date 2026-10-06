# Section 5 -- updated tiltrotor design (generated from src/m2/aircraft_input_m2.py)

Active rotor variant: 'M1' (25 deg root, -45 deg/R).

## 5.2 Changes from Milestone 1

| Change | Motivating issue |
|---|---|
| Engines: 1.88 MW-class turboshaft (assumed) -> 2 x GE CT7-8A, 1893 kW take-off (selected) | Engine-sizing study with the 6-DOF trimmed aircraft: hover OGE 1500 m ISA+15 margin +35 %, OEI at the corridor minimum-power point +61 %, OEI 85 m/s cruise +21 % (lightest candidate meeting OEI cruise); see m2_5_engine_selection.md |
| Airplane-mode RPM: 250 -> scheduled 250 / 350 / 420 RPM | 250 RPM kept for the 74.3 m/s / 7000 m cruise (least power) but the collective reaches its 65 deg limit between 80 and 85 m/s; 350 RPM up to 100 m/s, 420 RPM for the high-speed dash |
| Blade twist 25 deg root, -45 deg/R -> 12 deg root, -30 deg/R ('refined') | M1 blade stalls inboard in hover at MTOW / 2000 m (12 % of loaded disk, stall margin -2.5 deg); refined blade: 0 %, +2.0 deg, hover power -4 %; see m2_9_comparison.md |
| Explicit mass breakdown, tilting nacelle mass (engines, gearboxes, rotors), CG(i_n) | Trim needs component locations; the CG moves forward as the nacelles tilt down |
| Wing, horizontal and vertical tail with locations and control surfaces | Required for transition trim (M1 used a flat-plate drag area and wing induced drag only) |
| Fuel capacity 1500 -> 1470 kg | Selected engines are 30 kg heavier than the empty-mass allowance; MTOW kept at 7200 kg |
| Rotor model: axisymmetric BEMT -> azimuth-resolved BEMT with cyclic, annular Glauert + tip loss + K-factor | Edgewise and conversion flight; reduces exactly to the M1 solver at mu = 0 |
| Stall model stated as implemented: linear Cl with stall flag and Cl clip | The M1 report described a Viterna-Corrigan post-stall model; the code (M1 and M2) uses the linear model |

## 5.3 Rotor design

| Parameter | Milestone 1 | M2 'M1' variant | M2 'refined' variant |
|---|---|---|---|
| Airfoil | Linear (a0=5.75, Cd=0.0113+1.25*alpha^2) | Linear (a0=5.75, Cd=0.0113+1.25*alpha^2) | Linear (a0=5.75, Cd=0.0113+1.25*alpha^2) |
| Stall angle (flag) [deg] | 14 | same | same |
| Radius R [m] | 3.8 | same | same |
| Number of blades | 3 | same | same |
| Root cut-out [m] (r/R) | 0.5 (0.132) | same | same |
| Chord: root / tip [m], taper | 0.9 / 0.350, 0.3888 | same | same |
| Solidity sigma | 0.1480 | 0.1480 | 0.1480 |
| Twist: root (r/R=0) / rate | 25 deg / -45 deg/R | 25 deg / -45 deg/R | 12 deg / -30 deg/R |
| Built-in pitch at 0.75R [deg] | -8.75 | -8.75 | -10.50 |
| RPM helicopter / conversion | 550 | 550 | 550 |
| RPM airplane mode (schedule) | 250 | 250 (long-range cruise) / 350 (to 100 m/s) / 420 (dash) | 250 (long-range cruise) / 350 (to 100 m/s) / 420 (dash) |
| Hover tip speed / Mach (2000 m) | 219 m/s | 219 m/s / 0.658 | 219 m/s / 0.658 |
| Collective range [deg] | -10 .. 65 | -10 .. 65 | same |
| Cyclic range theta1c / theta1s [deg] | - (axial only) | -10 .. 10 | same |
| Rotation | - | right CCW / left CW (seen from above, helicopter mode) | same |

## 5.4 Wing and empennage

| Parameter | Wing | Horizontal tail | Vertical tail |
|---|---|---|---|
| Area [m^2] | 39.24 | 8.00 | 6.00 |
| Span / height [m] | 18.79 | 6.00 | 3.00 |
| Aspect ratio | 9.0 | 4.5 | 1.5 |
| Mean chord [m] | 2.09 | 1.33 | 2.00 |
| Aerodynamic model | thin airfoil a0=2pi, lifting-line CL_alpha=4.92/rad, e=0.8, CD0=0.015, CM_ac=-0.1, stall 15 deg, post-stall CD90=1.2 | a0=2pi, e=0.8, CD0=0.015, stall 12 deg, downwash 2CL_w/(pi AR_w) | CL_alpha=3.03/rad (end-plate AR_eff=1.55 AR), CD0=0.015 |
| Incidence [deg] | 4.0 | 0.0 | 0 |
| Control surface | flaperons 0.45-0.95 semi-span, tau=0.45, +/-20 deg, Cl_da=0.387/rad | elevator 30 % chord, tau_e=0.4, +/-25 deg | rudder 30 % chord, tau_r=0.5, +/-20 deg |
| a.c. from reference point [m] (x fwd, z down) | (0.00, 0.00, 0.00) | (-6.30, 0.00, -0.50) | (-6.60, 0.00, -1.60) |
| a.c. from CG (helicopter mode) [m] | (0.16, 0.00, -0.35) | (-6.14, 0.00, -0.85) | (-6.44, 0.00, -1.95) |
| Tail volume coefficient | - | V_H = 0.600 | V_V = 0.052 |

Fuselage + nacelle drag: flat-plate area 1.7 m^2 acting at the CG. Rotor-wake/wing and rotor-wake/tail interference and hover download are NOT modelled.
Nacelle pivots at (0.00, 9.40, -0.50) m (and mirror), mast (pivot -> hub) 1.6 m.

## 5.5 Mass properties, CG and limits

| Item | Mass [kg] | x [m] | z [m] | Notes |
|---|---|---|---|---|
| Wing structure | 520 | -0.35 | 0.00 | empty |
| Fuselage structure | 900 | -0.60 | 1.10 | empty |
| Empennage (H + V tail) | 160 | -6.40 | -0.60 | empty |
| Landing gear | 240 | -0.40 | 2.00 | empty |
| Engines (2 x CT7-8A, tilting) | 490 | 0.00 / -0.30 | -0.20 / -0.50 | tilts with nacelle (helicopter / airplane), y = +/-9.4 m |
| Nacelle structure (fixed) | 120 | -0.40 | -0.20 | empty |
| Proprotor gearboxes (tilting) | 240 | 0.00 / 0.60 | -1.10 / -0.50 | tilts with nacelle (helicopter / airplane), y = +/-9.4 m |
| Proprotors, blades + hubs | 520 | 0.00 / 1.60 | -2.10 / -0.50 | tilts with nacelle (helicopter / airplane), y = +/-9.4 m |
| Interconnect drive system | 240 | -0.30 | -0.20 | empty |
| Flight controls | 180 | -1.00 | 1.00 | empty |
| Avionics, electrical, systems | 400 | 2.50 | 1.00 | empty |
| Crew + furnishings | 520 | 2.00 | 1.00 | empty |
| Payload (10 pax) | 1200 | -0.50 | 1.20 | payload |
| Fuel (wing tanks) | 1470 | -0.30 | 0.10 | fuel |
| **Total (MTOW)** | **7200** | | | empty 4530 + payload 1200 + fuel 1470 |

| Loading | i_n [deg] | mass [kg] | CG x [m] | CG z [m] |
|---|---|---|---|---|
| MTOW, full fuel | 90 | 7200 | -0.159 | +0.352 |
| MTOW, full fuel | 45 | 7200 | -0.077 | +0.386 |
| MTOW, full fuel | 0 | 7200 | -0.044 | +0.467 |
| reserve fuel | 90 | 6180 | -0.135 | +0.394 |
| reserve fuel | 45 | 6180 | -0.041 | +0.433 |
| reserve fuel | 0 | 6180 | -0.001 | +0.528 |

| Limit | Value |
|---|---|
| Nacelle angle | 0 .. 95 deg, max rate 8.0 deg/s |
| Rotor RPM | 250 .. 900 (schedule 550 helicopter/conversion, 250 / 350 / 420 airplane) |
| Collective | -10 .. 65 deg |
| Longitudinal / lateral cyclic | +/-10 / +/-10 deg |
| Elevator / flaperon / rudder | +/-25 / +/-20 / +/-20 deg |
| Pitch / roll attitude (trim bounds) | -20 .. 25 / +/-30 deg |
| Advancing-tip Mach | 0.85 |
| Rotor stalled loaded area | 5 % |
| Reverse-flow area | 3 % of disk |
| Power margin | 5 % of available |
| Engines | 2 x GE CT7-8A, 1893 kW take-off each (SL), max continuous 86 %, dry mass 245 kg, SFC 280 g/kWh |
| Installed power | 2 x 1893 kW (SL), lapse (rho/rho0)^1.0, drivetrain eff. 0.95 |

Control mixing (normalized stick -> effector, rotor terms x sin^2(i_n)):

* pitch: theta1c = 10 deg x d_lon, elevator = 25 deg x d_lon
* roll: differential collective = 3 deg x d_lat, flaperons = 20 deg x d_lat
* yaw: differential lateral cyclic = 8 deg x d_ped, rudder = 20 deg x d_ped
