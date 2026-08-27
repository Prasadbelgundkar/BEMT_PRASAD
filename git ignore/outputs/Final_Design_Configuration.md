# Tiltrotor Final Sizing & Configuration Report 

**Design Point Selected:**
*   **Wing Loading (W/S):** $1800 \text{ N/m}^2$
*   **Thrust-to-Weight (T/W):** $0.27$

---

### 1. Mass & Power Overview
*   **Gross Takeoff Mass (MTOM):** 7,200 kg (70,632 N)
*   **Empty Mass:** 4,500 kg
*   **Payload (2 Pilots + 10 Pax):** 1,200 kg
*   **Fuel Capacity:** 1,500 kg
*   **Required Forward Thrust (T):** 19,070.64 N ($0.27 \times W$)

### 2. Wing Geometry (Airplane Mode)
Based on an Aspect Ratio (AR) of 9.0:
*   **Wing Area ($S$):** 39.24 m²
*   **Wingspan ($b$):** 18.79 m
*   **Average Chord ($c$):** 2.09 m
*   **Parasite Drag Area ($f$):** 1.7 m²
*   **Rotor Clearance:** **CLEAR**. *(18.79 m span leaves plenty of room for twin 7.6 m rotors).*

### 3. Proprotor Blade Geometry (Tiltrotor specific)
To handle both hover and 125 m/s forward flight, extreme structural and aerodynamic tapers were applied:
*   **Total Blade Twist:** $-45.0^\circ$ (+25° Root, -20° Tip)
*   **Chord Taper:** Root = 0.90 m, Tip = 0.35 m (Taper Ratio 0.3888)
*   **Blended Airfoils:**
    *   **Root (r/R 0.15):** Thick, high-camber profile (Eq. NACA 23021). $C_{L,max} = 1.6$, Stall = $20^\circ$.
    *   **Mid (r/R 0.50):** Efficient cruise lifter (Eq. NACA 64-212). Low Drag.
    *   **Tip (r/R 1.00):** Razor thin symmetric (Eq. NACA 64-008). Delays drag divergence at high Mach.

### 4. Mission Capabilities (Breguet Performance)
*   **Maximum Dash Speed:** 125.0 m/s (243 knots)
*   **Optimal Economy Cruise Speed ($V_{br}$):** 74.3 m/s (144 knots)
*   **Maximum Aerodynamic Efficiency ($L/D_{max}$):** 11.42
*   **Maximum Theoretical Range:** **2,865 km** (Exceeds 1000 km requirement by 186%)

### 5. Empennage (Tail) Sizing
Designed to counter wing/nacelle pitching moments (assumes moment arm of 8.45 m):
*   **Horizontal Tail Area ($S_{HT}$):** 9.70 m²
*   **Vertical Tail Area ($S_{VT}$):** 6.98 m²
