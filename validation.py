import csv
import numpy as np
import matplotlib.pyplot as plt
import os

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt

# ---- Fixed Rotor parameters (from Knight & Hefner) -------------------
RADIUS_M = 0.762
ROOT_CUTOUT_M = 0.125
CHORD_M = 0.0508
OMEGA_RAD_S = 2 * np.pi * 1250.0 / 60.0
ALTITUDE_M = 0.0
DISA_K = 0.0

airfoil = LinearAirfoil()  # a0=5.75, Cd_min=0.0113, eps=1.25


def build_validation_rotor(num_blades: int) -> Rotor:
    return Rotor(
        radius_m=RADIUS_M,
        root_cutout_m=ROOT_CUTOUT_M,
        num_blades=num_blades,
        chord_fn=constant_chord(CHORD_M),
        twist_fn=constant_twist(0.0),
        name=f"Knight-Hefner {num_blades}-blade rotor",
    )


def load_experimental_data(csv_path: str):
    data = {"theta": [], "CT": [], "CQ": [], "CQ_dash": [], "Tc": [], "Qc": []}
    with open(csv_path) as f:
        for row in csv.DictReader(f, skipinitialspace=True):
            data["theta"].append(float(row["collective_deg"]))
            data["CT"].append(float(row["CT_exp"]))
            data["CQ"].append(float(row["CQ_exp"]))
            data["CQ_dash"].append(float(row["CQ_dash_exp"]))
            data["Tc"].append(float(row["Tc_exp"]))
            data["Qc"].append(float(row["Qc_exp"]))
    
    # Convert lists to numpy arrays
    for k in data:
        data[k] = np.array(data[k])
        
    # HISTORICAL CORRECTION (1937 NACA to Modern Standard)
    data["CT"] = data["CT"] / 2.0
    data["CQ"] = data["CQ"] / 2.0
    data["CQ_dash"] = data["CQ_dash"] / 2.0
    data["Tc"] = data["Tc"] / 2.0
    data["Qc"] = data["Qc"] / 2.0
    
    return data


def run_validation():
    atmo = isa(ALTITUDE_M, DISA_K)
    
    # Configurations to test
    configs = [
        {"blades": 2, "csv": "data/knight_hefner_2blade.csv", "color": "red"},
        {"blades": 3, "csv": "data/knight_hefner_3blade.csv", "color": "blue"},
        {"blades": 4, "csv": "data/knight_hefner_4blade.csv", "color": "green"}
    ]

    # Setup the plot
    os.makedirs("outputs", exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()
    
    sweep_deg = np.linspace(0, 12, 40)

    for cfg in configs:
        b = cfg["blades"]
        col = cfg["color"]
        csv_path = os.path.join(os.path.dirname(__file__), cfg["csv"])
        exp_data = load_experimental_data(csv_path)
        
        rotor = build_validation_rotor(b)
        sigma = rotor.solidity()

        CT_pred, CQ_pred, CP_pred = [], [], []
        for coll in sweep_deg:
            perf = run_bemt(rotor, lambda x: airfoil, OMEGA_RAD_S, np.radians(coll),
                             atmo.density_kg_m3, atmo.speed_of_sound_mps, v_axial=0.0)
            CT_pred.append(perf.CT)
            CQ_pred.append(perf.CQ)
            CP_pred.append(perf.CP)

        CT_pred = np.array(CT_pred)
        CQ_pred = np.array(CQ_pred)
        CP_pred = np.array(CP_pred)

        # Derived BEMT params
        CQ_0 = CQ_pred[0] 
        CQ_dash_pred = CQ_pred - CQ_0
        Tc_pred = CT_pred / (sigma**2)
        Qc_pred = CQ_pred / (sigma**3)

        # Map metrics to axes
        plot_configs = [
            (0, "CT vs Theta", CT_pred, exp_data["CT"], "$C_T$"),
            (1, "CQ vs Theta", CQ_pred, exp_data["CQ"], "$C_Q$"),
            (2, "CQ' vs Theta", CQ_dash_pred, exp_data["CQ_dash"], "$C_Q'$ (Induced Torque)"),
            (3, "Tc (T_sigma) vs Theta", Tc_pred, exp_data["Tc"], "$T_c$ ($C_T / \\sigma^2$)"),
            (4, "Qc (Q_sigma) vs Theta", Qc_pred, exp_data["Qc"], "$Q_c$ ($C_Q / \\sigma^3$)"),
            (5, "CP vs Theta", CP_pred, exp_data["CQ"], "$C_P$") # In hover, exp CP = exp CQ
        ]

        # Plot for this blade configuration
        for ax_idx, title, pred, exp, ylabel in plot_configs:
            ax = axes[ax_idx]
            
            # Experimental = dots
            ax.plot(exp_data["theta"], exp, 'o', color=col, label=f"Exp ({b}-blade)", markersize=5)
            
            # BEMT = solid line
            ax.plot(sweep_deg, pred, '-', color=col, label=f"BEMT ({b}-blade)", linewidth=2.0)
            
            if b == configs[-1]["blades"]: # Set labels/titles only once at the end
                ax.set_title(title, fontweight='bold')
                ax.set_xlabel("Collective pitch [deg]")
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.4, linestyle='--')
                ax.legend()

    fig.suptitle("Knight & Hefner (1937) BEMT Validation: 2, 3, and 4 Blades", fontsize=16, fontweight='bold')
    fig.tight_layout()
    
    plot_path = os.path.join("outputs", "knight_hefner_multi_blade_validation.png")
    fig.savefig(plot_path, dpi=150)
    print(f"Validation complete! Plot successfully saved to {os.path.abspath(plot_path)}")


if __name__ == "__main__":
    run_validation()
