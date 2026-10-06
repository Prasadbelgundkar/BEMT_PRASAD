# Section 5 -- engine sizing and selection

Rotor 'M1', MTOW 7200 kg, 6-DOF trimmed power required (both rotors). Power available per engine = 0.95 x P_rated x sigma^1.0; cruise checks use max-continuous = 0.86 x take-off; required margin 5 %.

## Power required at the sizing conditions

| Condition | P required [kW] | Rating used |
|---|---|---|
| R1 hover OGE, 1500 m ISA+15 | 1994 | take-off, 2 engines |
| R2 vertical climb 2.5 m/s, 2000 m | 2084 | take-off, 2 engines |
| R3 airplane cruise 74.3 m/s, 7000 m, 250 RPM | 597 | max continuous, 2 engines |
| R4a OEI, corridor minimum power (50 m/s, i_n = 7.5 deg), 2000 m | 659 | OEI, 1 engine |
| R4b OEI, airplane cruise 85 m/s, 2000 m, 350 RPM (desirable) | 1074 | OEI, 1 engine |

## Candidate engines

| Engine | Take-off power [kW] | Dry mass [kg] | SFC [g/kWh] | R1 | R2 | R3 | R4a | R4b | V_max 2000 m, MCP [m/s] | V_max 7000 m, MCP [m/s] | V_dash 7000 m, take-off [m/s] | Meets R1-R4a | Note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| P&WC PT6C-67A | 1445 | 225 | 290 | +11 % | +8 % | +47 % | +42 % | +5 % | 104 | 101 | 107 | yes | AW609; mass, SFC est. |
| GE T700-GE-701D | 1486 | 207 | 283 | +14 % | +10 % | +49 % | +43 % | +7 % | 105 | 102 | 108 | yes | UH-60M / AH-64E |
| RR/Safran RTM322-01/1 | 1611 | 255 | 255 | +20 % | +17 % | +53 % | +48 % | +15 % | 109 | 105 | 112 | yes | NH90 / EH101 |
| Safran Makila 2A | 1801 | 279 | 270 | +29 % | +26 % | +58 % | +53 % | +24 % | 113 | 110 | 117 | yes | H225; SFC est. |
| GE CT7-8A | 1893 | 245 | 280 | +32 % | +29 % | +60 % | +55 % | +27 % | 115 | 112 | 119 | yes | S-92; mass, SFC est. |

Margins are 1 - P_required / P_available (5 % required). Airplane-mode RPM: 350 up to 100 m/s, 420 above (dash).

No candidate reaches the 125 m/s (450 km/h) target. The airplane-mode trim converges up to 135 m/s, so the top speed is limited by power: 125 m/s at 7000 m needs 1843 kW, i.e. a take-off rating of at least 2121 kW per engine.

**Lightest engine meeting the criteria with this blade: GE T700-GE-701D** (1486 kW take-off, 207 kg, SFC 283 g/kWh): meets R1-R4a, OEI cruise (R4b) -- the lightest engine doing so.

The installed engine (GE CT7-8A) is selected on the design rotor ('refined'), see outputs/m2/rotor_refined/m2_5_engine_selection.md; this table shows how the choice would change with this blade.
