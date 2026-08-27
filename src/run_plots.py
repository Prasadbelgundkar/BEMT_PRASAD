import numpy as np
import matplotlib.pyplot as plt
import os

from environment import isa
from bemt import run_bemt

# Import EVERYTHING from our new parameters control panel!
from parameters import (
    get_configured_rotor, AIRFOIL_PROVIDER, 
    ALTITUDE_M, DISA_K, OMEGA_RPM, V_AXIAL_MPS
)

def main():
    # 1. Setup Environment from parameters
    atmo = isa(ALTITUDE_M, DISA_K)
    omega_rad_s = OMEGA_RPM * (2 * np.pi / 60.0)
    rotor = get_configured_rotor()

    # 2. Sweep over a range of collective angles (2 to 20 degrees)
    collectives_deg = np.linspace(2, 20, 20)
    
    CT_list = []
    CQ_list = []
    CP_list = []

    print(f"Running BEMT sweep for a {rotor.radius_m}m rotor at {OMEGA_RPM} RPM...")

    for c in collectives_deg:
        # The solver takes our rotor and aerodynamic parameters
        perf = run_bemt(
            rotor=rotor,
            airfoil_provider=AIRFOIL_PROVIDER,
            omega_rad_s=omega_rad_s,
            collective_rad=np.radians(c),
            rho=atmo.density_kg_m3,
            a_sound=atmo.speed_of_sound_mps,
            v_axial=V_AXIAL_MPS
        )
        CT_list.append(perf.CT)
        CQ_list.append(perf.CQ)
        CP_list.append(perf.CP)

    # 3. Create the Plot
    os.makedirs("outputs", exist_ok=True)
    plt.figure(figsize=(10, 6))

    # Plot CT, CQ, CP on the same graph (multiplying CQ and CP by 10 so they are visible on the same scale as CT is common practice, but we'll plot them raw first)
    plt.plot(collectives_deg, CT_list, label='$C_T$ (Thrust Coeff)', marker='o', linewidth=2)
    plt.plot(collectives_deg, [cq * 10 for cq in CQ_list], label='$10 \\times C_Q$ (Torque Coeff x 10)', marker='s', linewidth=2)
    plt.plot(collectives_deg, [cp * 10 for cp in CP_list], label='$10 \\times C_P$ (Power Coeff x 10)', marker='^', linewidth=2)

    plt.title("Rotor Performance Coefficients vs Collective Pitch\n(Using parameters.py Inputs)")
    plt.xlabel("Collective Pitch [degrees]")
    plt.ylabel("Coefficient Value")
    plt.grid(True, alpha=0.4, linestyle='--')
    plt.legend()
    plt.tight_layout()

    plot_path = "outputs/coefficient_plots.png"
    plt.savefig(plot_path, dpi=150)
    print(f"Plot successfully saved to: {os.path.abspath(plot_path)}")

if __name__ == "__main__":
    main()
