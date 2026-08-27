# Tiltrotor Sizing Configuration (Updated)

**Design Point Selected:**
*   **Wing Loading (W/S):** $1800 \text{ N/m}^2$
*   **Thrust-to-Weight (T/W):** $0.27$

---

### 1. Mass & Power Overview
*   **Gross Takeoff Weight (W):** 70,632 N  (7,200 kg)
*   **Required Forward Thrust (T):** 19,070.64 N
    *(Calculated as $0.27 \times 70,632$. This is a very balanced medium between the initial 0.235 and the aggressive 0.32, giving you a strong maneuverability margin without over-taxing the engines during steady cruise).*

### 2. Wing Geometry (Airplane Mode)
Based on an Aspect Ratio (AR) of 9.0:
*   **Wing Area ($S$):** 39.24 m²
*   **Wingspan ($b$):** 18.79 m
*   **Average Chord ($c$):** 2.09 m
*   **Airfoil:** NACA 64A223 (Root) / NACA 64A212 (Tip)
*   **Rotor Clearance:** **CLEAR**. *(18.79 m span leaves plenty of room for the 7.6 m total rotor footprint)*.

### 3. Empennage (Tail) Sizing
Because this is a NOTAR (No Tail Rotor) design relying on twin rotors for anti-torque, the tail is a pure empennage designed to counter wing/nacelle pitching and yawing moments.

Assuming standard volume coefficients ($V_{HT}=1.0$, $V_{VT}=0.08$) and a moment arm ($L_{tail}$) of **8.45 m** (approx 45% of wingspan):
*   **Horizontal Tail Area ($S_{HT}$):** 9.70 m²
*   **Vertical Tail Area ($S_{VT}$):** 6.98 m²
