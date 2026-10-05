# Section 5 -- updated tiltrotor design (generated from src/m2/aircraft_input_m2.py)

Active rotor variant: 'M1' (25 deg root, -45 deg/R).

## 5.2 Changes from Milestone 1

| Change | Motivating issue |
|---|---|
| Installed power 2 x 200 kW -> 2 x 1450 kW | M1 power could not hover the 7.2 t aircraft (ideal hover power alone ~1.3 MW); AW609-class engines |
| Airplane-mode RPM 700 -> 420 (84 %) | Helical tip Mach ~0.86 at 85 m/s / 700 RPM exceeded the 0.85 limit |
| Airplane-mode cruise speed 40 -> 85 m/s | 40 m/s is below the airplane-mode wing stall speed (~46 m/s at MTOW) |
| Collective range -5..25 deg -> -5..55 deg | Airplane mode at 85 m/s and 420 RPM needs ~46 deg collective on top of the -45 deg twist |
| Explicit mass breakdown, tilting proprotor mass, CG(i_n) | Trim needs component locations; the CG moves forward as the nacelles tilt down |
| Wing, horizontal and vertical tail defined with locations | Required for transition trim (M1 used a flat-plate drag area only) |
| Rotor variant 'refined': twist 25/-45 -> 12/-30 deg/R, helicopter-mode RPM 500 -> 540 | M1 blade stalls on ~24 % of the hover disk at MTOW/2000 m; refined: stall-free hover, +11 % cruise power |
| Rotor inflow: uniform Glauert -> annular Glauert + tip loss + K-factor | Uniform inflow mis-predicted the M1 hover/axial limit by 23-47 % on the -45 deg twisted blade |

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
| RPM helicopter / conversion | 500 | 500 | 540 |
| RPM airplane mode | 700 | 420 | 420 |
| Hover tip speed / Mach (2000 m) | 199 m/s | 199 m/s / 0.598 | 215 m/s / 0.646 |
| Collective range [deg] | -5 .. 25 | -5 .. 55 | same |
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
| a.c. from reference point [m] (x fwd, z down) | (np.float64(0.0), np.float64(0.0), np.float64(0.0)) | (np.float64(-6.3), np.float64(0.0), np.float64(-0.5)) | (np.float64(-6.6), np.float64(0.0), np.float64(-1.6)) |
| a.c. from CG (helicopter mode) [m] | (np.float64(0.2), np.float64(0.0), np.float64(-0.35)) | (np.float64(-6.1), np.float64(0.0), np.float64(-0.85)) | (np.float64(-6.4), np.float64(0.0), np.float64(-1.95)) |
| Tail volume coefficient | - | V_H = 0.596 | V_V = 0.052 |

Fuselage + nacelle drag: flat-plate area 1.8 m^2 acting at the CG. Rotor-wake/wing and rotor-wake/tail interference and hover download are NOT modelled.
Nacelle pivots at (np.float64(0.0), np.float64(9.4), np.float64(-0.5)) m (and mirror), mast (pivot -> hub) 1.6 m.

## 5.5 Mass properties, CG and limits

| Item | Mass [kg] | x [m] | z [m] | Notes |
|---|---|---|---|---|
| Wing structure | 520 | -0.35 | 0.00 | empty |
| Fuselage structure | 900 | -0.60 | 1.10 | empty |
| Empennage (H + V tail) | 160 | -6.40 | -0.60 | empty |
| Landing gear | 240 | -0.40 | 2.00 | empty |
| Engines (2, fixed at tips) | 460 | -0.60 | -0.20 | empty |
| Nacelle structure (fixed) | 120 | -0.40 | -0.20 | empty |
| Proprotor gearboxes (tilting) | 240 | 0.00 / 0.60 | -1.10 / -0.50 | tilts with nacelle (helicopter / airplane), y = +/-9.4 m |
| Proprotors, blades + hubs | 520 | 0.00 / 1.60 | -2.10 / -0.50 | tilts with nacelle (helicopter / airplane), y = +/-9.4 m |
| Interconnect drive system | 240 | -0.30 | -0.20 | empty |
| Flight controls | 180 | -1.00 | 1.00 | empty |
| Avionics, electrical, systems | 400 | 2.50 | 1.00 | empty |
| Crew + furnishings | 520 | 2.00 | 1.00 | empty |
| Payload (10 pax) | 1200 | -0.50 | 1.20 | payload |
| Fuel (wing tanks) | 1500 | -0.30 | 0.10 | fuel |
| **Total (MTOW)** | **7200** | | | empty 4500 + payload 1200 + fuel 1500 |

| Loading | i_n [deg] | mass [kg] | CG x [m] | CG z [m] |
|---|---|---|---|---|
| MTOW, full fuel | 90 | 7200 | -0.198 | +0.353 |
| MTOW, full fuel | 45 | 7200 | -0.102 | +0.393 |
| MTOW, full fuel | 0 | 7200 | -0.063 | +0.489 |
| reserve fuel | 90 | 6150 | -0.181 | +0.397 |
| reserve fuel | 45 | 6150 | -0.069 | +0.443 |
| reserve fuel | 0 | 6150 | -0.022 | +0.555 |

| Limit | Value |
|---|---|
| Nacelle angle | 0 .. 95 deg, max rate 8.0 deg/s |
| Rotor RPM | 300 .. 900 (schedule 500 helicopter/conversion, 420 airplane) |
| Collective | -5 .. 55 deg |
| Longitudinal / lateral cyclic | +/-10 / +/-10 deg |
| Elevator / flaperon / rudder | +/-25 / +/-20 / +/-20 deg |
| Pitch / roll attitude (trim bounds) | -20 .. 25 / +/-30 deg |
| Advancing-tip Mach | 0.85 |
| Rotor stalled loaded area | 5 % |
| Reverse-flow area | 3 % of disk |
| Power margin | 5 % of available |
| Installed power | 2 x 1450 kW (SL), lapse (rho/rho0)^0.9, drivetrain eff. 0.94 |

Control mixing (normalized stick -> effector, rotor terms x sin^2(i_n)):

* pitch: theta1c = 10 deg x d_lon, elevator = 25 deg x d_lon
* roll: differential collective = 3 deg x d_lat, flaperons = 20 deg x d_lat
* yaw: differential lateral cyclic = 8 deg x d_ped, rudder = 20 deg x d_ped
