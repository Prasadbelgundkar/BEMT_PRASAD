import numpy as np
import matplotlib.pyplot as plt
import os

from parameters import (
    EMPTY_MASS_KG, PAYLOAD_MASS_KG, FUEL_MASS_KG,
    FLAT_PLATE_AREA_M2, NUM_ENGINES, ENGINE_POWER_W,
    CRUISE_ALTITUDE_AMSL_M, RADIUS_M
)
from environment import isa

def run_sizing():
    # 1. Total Weight & Constants
    M_total = EMPTY_MASS_KG + PAYLOAD_MASS_KG + FUEL_MASS_KG
    g = 9.81
    W = M_total * g
    
    # 2. Aerodynamic assumptions
    atmo_sl = isa(0.0)
    rho_sl = atmo_sl.density_kg_m3
    
    atmo_cruise = isa(CRUISE_ALTITUDE_AMSL_M)
    rho_cruise = atmo_cruise.density_kg_m3
    
    atmo_ceil = isa(10000.0)
    rho_ceil = atmo_ceil.density_kg_m3
    
    e = 0.8  # Oswald efficiency
    AR = 9.0 
    f = FLAT_PLATE_AREA_M2
    
    # X-axis: W/S in N/m^2
    # The user requested increments of 200. We will set the axis limits from 0 to 6000.
    # To avoid divide-by-zero, start WS_range slightly above 0.
    WS_range = np.linspace(50, 6000, 400)
    
    # --- CALCULATE T/W CURVES ---
    # Equation: T/W = (ROC/V) + q*f/W + (W/S * n^2) / (q * pi * e * AR)
    
    # A. Cruise (V = 125 m/s at Cruise Alt, n=1, ROC=0)
    V_cruise = 125.0
    q_cruise = 0.5 * rho_cruise * V_cruise**2
    TW_cruise = (q_cruise * f) / W + WS_range / (q_cruise * np.pi * e * AR)
    
    # B. Climb (ROC = 10 m/s at V = 80 m/s at SL, n=1)
    V_climb = 80.0
    ROC_climb = 10.0
    q_climb = 0.5 * rho_sl * V_climb**2
    TW_climb = (ROC_climb / V_climb) + (q_climb * f) / W + WS_range / (q_climb * np.pi * e * AR)
    
    # C. Turn (Sustained 60-deg bank turn, n=2 at V = 80 m/s at SL)
    V_turn = 80.0
    n_turn = 2.0
    q_turn = 0.5 * rho_sl * V_turn**2
    TW_turn = (q_turn * f) / W + (WS_range * n_turn**2) / (q_turn * np.pi * e * AR)
    
    # D. Ceiling (ROC = 0.5 m/s at V = 90 m/s at 10000m)
    V_ceil = 90.0
    ROC_ceil = 0.5
    q_ceil = 0.5 * rho_ceil * V_ceil**2
    TW_ceil = (ROC_ceil / V_ceil) + (q_ceil * f) / W + WS_range / (q_ceil * np.pi * e * AR)

    # --- CALCULATE VERTICAL LINES ---
    # 1. (W/S) stall
    V_stall = 45.0
    CL_max = 1.6
    WS_stall = 0.5 * rho_sl * V_stall**2 * CL_max
    
    # 2. (W/S) glide (Optimal glide speed, e.g., V_glide = 55 m/s at CL_opt = 1.0)
    V_glide = 55.0
    CL_glide = 1.0
    WS_glide = 0.5 * rho_sl * V_glide**2 * CL_glide

    # --- PLOTTING ---
    # Using a large figure to ensure all the dense tick marks are readable
    plt.figure(figsize=(16, 9))
    
    # Plot Curves
    plt.plot(WS_range, TW_cruise, label='Cruise (125 m/s)', color='blue', linewidth=2.5)
    plt.plot(WS_range, TW_climb, label='Climb (ROC 10 m/s)', color='green', linewidth=2.5)
    plt.plot(WS_range, TW_turn, label='Turn (n=2, 60 deg bank)', color='purple', linewidth=2.5)
    plt.plot(WS_range, TW_ceil, label='Ceiling (10000m Alt)', color='orange', linewidth=2.5)
    
    # Plot Vertical Lines
    plt.axvline(x=WS_stall, color='red', linestyle='--', label=f'(W/S) Stall ({WS_stall:.0f} N/m²)', linewidth=2.5)
    plt.axvline(x=WS_glide, color='brown', linestyle='-.', label=f'(W/S) Glide ({WS_glide:.0f} N/m²)', linewidth=2.5)
    
    # Optional: Fill the valid design space
    # The valid design space is the area ABOVE all T/W curves and to the LEFT of the W/S limits
    max_TW_required = np.maximum.reduce([TW_cruise, TW_climb, TW_turn, TW_ceil])
    plt.fill_between(WS_range, max_TW_required, 1.2, where=(WS_range <= WS_stall), color='gray', alpha=0.15, label='Valid Design Space')
    
    # Plot the Chosen Design Point
    plt.plot(1800.0, 0.27, 'k*', markersize=18, label='Design Point (W/S=1800, T/W=0.27)')
    
    # Formatting
    plt.title('Tiltrotor Constraint Analysis Plot (Airplane Mode)', fontsize=18, fontweight='bold', pad=20)
    plt.xlabel('Wing Loading (W/S) [N/m²]', fontsize=14)
    plt.ylabel('Thrust-to-Weight Ratio (T/W)', fontsize=14)
    
    # --- AXIS TICKS AS REQUESTED ---
    # T/W from 0 to 1.2 with increments of 0.1
    plt.yticks(np.arange(0.0, 1.21, 0.1))
    plt.ylim(0, 1.2)
    
    # W/S from 0 to 2600 with increments of 200
    plt.xticks(np.arange(0, 2601, 200))
    plt.xlim(0, 2600)
    
    # Grid and Legend
    plt.grid(True, which='both', linestyle='-', linewidth=0.5, alpha=0.7)
    plt.legend(loc='upper right', fontsize=12, framealpha=0.9)
    
    # Rotate x-axis labels if they overlap
    plt.xticks(rotation=45)
    
    plt.tight_layout()
    
    os.makedirs('outputs', exist_ok=True)
    plt.savefig('outputs/constraint_analysis.png', dpi=200, facecolor='white')
    print("Graph generated and saved to outputs/constraint_analysis.png")
    
if __name__ == "__main__":
    run_sizing()
