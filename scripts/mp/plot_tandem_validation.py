"""
plot_tandem_validation.py

Validates the tandem BEMT interference model against Dingeldein (1954),
NACA TN-3236 wind-tunnel hover data.

Reference:
    Dingeldein, R. C., "Wind-Tunnel Studies of the Performance of Multirotor
    Configurations," NACA TN-3236, Langley Aeronautical Laboratory, 1954.

Rotor geometry (from NACA TN-3236):
    R = 1.3716 m (4.50 ft), root cutout 0.1981 m, 2 blades,
    constant chord 0.1167 m, NACA 0012 untwisted,
    V_tip = 137.16 m/s (450 ft/s), hover OGE, sea level standard.
    Tandem spacing d = 1.54 R (14.5% disk overlap).

Outputs: outputs/mp/tandem_validation_dingeldein.png
"""

import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

# Ensure src/ is in the module search path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt
from tandem_interference import solve_tandem_bemt, compute_overlap_fraction

# Validation geometry from NACA TN-3236
RADIUS_M = 1.3716       # 4.5 ft
ROOT_CUTOUT_M = 0.1981  # 0.65 ft
CHORD_M = 0.1167        # 0.383 ft
NUM_BLADES = 2
OMEGA_RAD_S = 100.0     # 955 RPM -> V_tip = 137.16 m/s (450 ft/s)
ALTITUDE_M = 0.0
DISA_K = 0.0

# NACA 0012 thin-airfoil representation
airfoil_naca0012 = LinearAirfoil(
    a0=5.73,
    Cd_min=0.0090,
    eps=1.18,
    stall_alpha_rad=np.radians(14.0),
    Cl_max_clip=1.40,
)

def build_benchmark_rotor(name: str = "NACA TN-3236 Rotor") -> Rotor:
    """Build single rotor matching NACA TN-3236 specifications."""
    return Rotor(
        radius_m=RADIUS_M,
        root_cutout_m=ROOT_CUTOUT_M,
        num_blades=NUM_BLADES,
        chord_fn=constant_chord(CHORD_M),
        twist_fn=constant_twist(0.0),
        name=name,
    )

# Tandem separation from Dingeldein (1954):
# d/R = 1.54 -> d = 1.54 * 1.3716 = 2.1123 m
TANDEM_SEP_D_OVER_R = 1.54
LONGITUDINAL_SEP_M = TANDEM_SEP_D_OVER_R * RADIUS_M
LATERAL_OFFSET_M = 0.0
VERTICAL_STAGGER_M = 0.0  # coplanar benchmark test


# Load published data
def load_validation_data(csv_path: str):
    data = {
        "collective_deg": [],
        "CT_iso_exp": [],
        "CP_iso_exp": [],
        "CT_tan_exp": [],
        "CP_tan_exp": [],
    }
    with open(csv_path, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader((row for row in f if not row.startswith("#")))
        for row in reader:
            data["collective_deg"].append(float(row["collective_deg"]))
            data["CT_iso_exp"].append(float(row["CT_isolated_exp"]))
            data["CP_iso_exp"].append(float(row["CP_isolated_exp"]))
            data["CT_tan_exp"].append(float(row["CT_tandem_exp"]))
            data["CP_tan_exp"].append(float(row["CP_tandem_exp"]))

    for k in data:
        data[k] = np.array(data[k])
    return data


def run_tandem_validation():
    atmo = isa(ALTITUDE_M, DISA_K)
    rho = atmo.density_kg_m3
    a_sound = atmo.speed_of_sound_mps

    csv_path = os.path.join(SRC_DIR, "data", "tandem_validation_data.csv")
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Validation CSV not found at: {csv_path}")

    exp_data = load_validation_data(csv_path)

    rotor_fwd = build_benchmark_rotor("NACA TN-3236 Forward Rotor")
    rotor_aft = build_benchmark_rotor("NACA TN-3236 Aft Rotor")

    overlap = compute_overlap_fraction(
        R_fwd=rotor_fwd.radius_m,
        R_aft=rotor_aft.radius_m,
        longitudinal_sep_m=LONGITUDINAL_SEP_M,
        lateral_offset_m=LATERAL_OFFSET_M,
    )

    print("=" * 80)
    print("TANDEM ROTOR BEMT VALIDATION REPORT -- NACA TN-3236 (Dingeldein, 1954)")
    print("=" * 80)
    print(f"Rotor Radius (R)        : {RADIUS_M:.4f} m (Solidity sigma = {rotor_fwd.solidity():.4f})")
    print(f"Separation Spacing      : {LONGITUDINAL_SEP_M:.4f} m (d/R = {TANDEM_SEP_D_OVER_R:.2f})")
    print(f"Calculated Overlap Area : {overlap * 100.0:.2f}% of disk area")
    print(f"Test Condition          : Sea Level Standard (rho = {rho:.4f} kg/m^3, a = {a_sound:.1f} m/s)")
    print("-" * 80)

    # High-resolution collective sweep for smooth theoretical prediction curves
    coll_sweep_deg = np.linspace(1.0, 13.0, 60)

    bemt_iso_CT = []
    bemt_iso_CP = []
    bemt_iso_FM = []

    bemt_tan_CT = []
    bemt_tan_CP = []
    bemt_tan_FM = []

    airfoil_fn = lambda x: airfoil_naca0012

    for theta_deg in coll_sweep_deg:
        theta_rad = np.radians(theta_deg)

        # 1. Isolated single rotor solve
        p_iso = run_bemt(
            rotor=rotor_fwd,
            airfoil_provider=airfoil_fn,
            omega_rad_s=OMEGA_RAD_S,
            collective_rad=theta_rad,
            rho=rho,
            a_sound=a_sound,
            v_axial=0.0,
            n_stations=80,
        )
        bemt_iso_CT.append(p_iso.CT)
        bemt_iso_CP.append(p_iso.CP)
        bemt_iso_FM.append(p_iso.figure_of_merit if p_iso.figure_of_merit is not None else np.nan)

        # 2. Tandem coupled system solve
        p_tan = solve_tandem_bemt(
            rotor_fwd=rotor_fwd,
            rotor_aft=rotor_aft,
            airfoil_provider_fwd=airfoil_fn,
            airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=OMEGA_RAD_S,
            collective_fwd_rad=theta_rad,
            collective_aft_rad=theta_rad,
            rho=rho,
            a_sound=a_sound,
            longitudinal_sep_m=LONGITUDINAL_SEP_M,
            lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            v_axial_freestream=0.0,
            include_interference=True,
            n_stations=80,
        )
        bemt_tan_CT.append(p_tan.total_CT)
        bemt_tan_CP.append(p_tan.total_CP)
        bemt_tan_FM.append(p_tan.figure_of_merit_system if p_tan.figure_of_merit_system is not None else np.nan)

    bemt_iso_CT = np.array(bemt_iso_CT)
    bemt_iso_CP = np.array(bemt_iso_CP)
    bemt_iso_FM = np.array(bemt_iso_FM)

    bemt_tan_CT = np.array(bemt_tan_CT)
    bemt_tan_CP = np.array(bemt_tan_CP)
    bemt_tan_FM = np.array(bemt_tan_FM)

    # Error analysis at each experimental collective angle
    print(f"{'Theta [deg]':>11} | {'CT_exp':>9} {'CT_pred':>9} {'Err%':>7} | {'CP_exp':>9} {'CP_pred':>9} {'Err%':>7}")
    print("-" * 75)

    pred_tan_CT_at_exp = []
    pred_tan_CP_at_exp = []

    for th_exp, ct_exp, cp_exp in zip(exp_data["collective_deg"], exp_data["CT_tan_exp"], exp_data["CP_tan_exp"]):
        th_rad = np.radians(th_exp)
        res = solve_tandem_bemt(
            rotor_fwd=rotor_fwd,
            rotor_aft=rotor_aft,
            airfoil_provider_fwd=airfoil_fn,
            airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=OMEGA_RAD_S,
            collective_fwd_rad=th_rad,
            collective_aft_rad=th_rad,
            rho=rho,
            a_sound=a_sound,
            longitudinal_sep_m=LONGITUDINAL_SEP_M,
            lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            v_axial_freestream=0.0,
            include_interference=True,
            n_stations=80,
        )
        ct_pred = res.total_CT
        cp_pred = res.total_CP
        err_ct = (ct_pred - ct_exp) / ct_exp * 100.0
        err_cp = (cp_pred - cp_exp) / cp_exp * 100.0

        pred_tan_CT_at_exp.append(ct_pred)
        pred_tan_CP_at_exp.append(cp_pred)

        print(f"{th_exp:11.1f} | {ct_exp:9.5f} {ct_pred:9.5f} {err_ct:7.2f}% | {cp_exp:9.6f} {cp_pred:9.6f} {err_cp:7.2f}%")

    pred_tan_CT_at_exp = np.array(pred_tan_CT_at_exp)
    pred_tan_CP_at_exp = np.array(pred_tan_CP_at_exp)

    mape_CT = np.mean(np.abs((pred_tan_CT_at_exp - exp_data["CT_tan_exp"]) / exp_data["CT_tan_exp"])) * 100.0
    mape_CP = np.mean(np.abs((pred_tan_CP_at_exp - exp_data["CP_tan_exp"]) / exp_data["CP_tan_exp"])) * 100.0

    print("-" * 75)
    print(f"Overall Tandem Model Accuracy:")
    print(f"  Thrust Coefficient (CT) Mean Absolute Percentage Error: {mape_CT:.2f}%")
    print(f"  Power Coefficient  (CP) Mean Absolute Percentage Error: {mape_CP:.2f}%")
    print("=" * 80)

    # Plot
    out_dir = os.path.join(PROJECT_ROOT, "outputs", "mp")
    os.makedirs(out_dir, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.patch.set_facecolor("#ffffff")

    # Colors
    c_iso_exp = "#1f77b4"
    c_iso_mod = "#1f77b4"
    c_tan_exp = "#d62728"
    c_tan_mod = "#d62728"

    # Subplot 1: CT vs Collective Pitch
    ax1 = axes[0, 0]
    ax1.plot(exp_data["collective_deg"], exp_data["CT_iso_exp"], "o", color=c_iso_exp, markersize=7, label="Exp: Isolated Rotor (NACA TN-3236)")
    ax1.plot(coll_sweep_deg, bemt_iso_CT, "-", color=c_iso_mod, linewidth=2.0, label="BEMT: Isolated Rotor (Clean Inflow)")
    ax1.plot(exp_data["collective_deg"], exp_data["CT_tan_exp"], "s", color=c_tan_exp, markersize=7, label="Exp: Tandem Rotors (d/R=1.54)")
    ax1.plot(coll_sweep_deg, bemt_tan_CT, "--", color=c_tan_mod, linewidth=2.2, label="BEMT: Tandem (Interference Model)")
    ax1.set_title("(a) Thrust Coefficient ($C_T$) vs. Collective Pitch", fontsize=12, fontweight="bold", pad=8)
    ax1.set_xlabel("Collective Pitch $\\theta$ [deg]", fontsize=11)
    ax1.set_ylabel("Thrust Coefficient $C_T = T / (\\rho A V_{tip}^2)$ [-]", fontsize=11)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Subplot 2: CP vs Collective Pitch
    ax2 = axes[0, 1]
    ax2.plot(exp_data["collective_deg"], exp_data["CP_iso_exp"] * 1e3, "o", color=c_iso_exp, markersize=7, label="Exp: Isolated Rotor (NACA TN-3236)")
    ax2.plot(coll_sweep_deg, bemt_iso_CP * 1e3, "-", color=c_iso_mod, linewidth=2.0, label="BEMT: Isolated Rotor (Clean Inflow)")
    ax2.plot(exp_data["collective_deg"], exp_data["CP_tan_exp"] * 1e3, "s", color=c_tan_exp, markersize=7, label="Exp: Tandem Rotors (d/R=1.54)")
    ax2.plot(coll_sweep_deg, bemt_tan_CP * 1e3, "--", color=c_tan_mod, linewidth=2.2, label="BEMT: Tandem (Interference Model)")
    ax2.set_title("(b) Power Coefficient ($C_P \\times 10^3$) vs. Collective Pitch", fontsize=12, fontweight="bold", pad=8)
    ax2.set_xlabel("Collective Pitch $\\theta$ [deg]", fontsize=11)
    ax2.set_ylabel("Power Coefficient $C_P \\times 10^3 = P / (\\rho A V_{tip}^3) \\times 10^3$ [-]", fontsize=11)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Subplot 3: Hover Polar (CP vs CT)
    ax3 = axes[1, 0]
    ax3.plot(exp_data["CT_iso_exp"], exp_data["CP_iso_exp"] * 1e3, "o", color=c_iso_exp, markersize=7, label="Exp: Isolated Rotor (NACA TN-3236)")
    ax3.plot(bemt_iso_CT, bemt_iso_CP * 1e3, "-", color=c_iso_mod, linewidth=2.0, label="BEMT: Isolated Rotor")
    ax3.plot(exp_data["CT_tan_exp"], exp_data["CP_tan_exp"] * 1e3, "s", color=c_tan_exp, markersize=7, label="Exp: Tandem Rotors (d/R=1.54)")
    ax3.plot(bemt_tan_CT, bemt_tan_CP * 1e3, "--", color=c_tan_mod, linewidth=2.2, label="BEMT: Tandem (Interference Model)")
    # Annotate induced power penalty
    ax3.annotate(
        "Induced Power Penalty\nfrom Downwash Ingestion",
        xy=(0.006, 0.65),
        xytext=(0.003, 0.95),
        arrowprops=dict(facecolor="black", shrink=0.08, width=1.0, headwidth=6),
        fontsize=9.5,
        fontweight="semibold",
        bbox=dict(boxstyle="round,pad=0.3", fc="#fff8dc", ec="#d2b48c", alpha=0.9),
    )
    ax3.set_title("(c) Hover Polar: Power Coefficient ($C_P$) vs. Thrust ($C_T$)", fontsize=12, fontweight="bold", pad=8)
    ax3.set_xlabel("Thrust Coefficient $C_T$ [-]", fontsize=11)
    ax3.set_ylabel("Power Coefficient $C_P \\times 10^3$ [-]", fontsize=11)
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Subplot 4: Figure of Merit vs CT
    ax4 = axes[1, 1]
    # Experimental FM calculation: FM = CT^1.5 / (sqrt(2) * CP)
    exp_iso_FM = exp_data["CT_iso_exp"]**1.5 / (np.sqrt(2.0) * exp_data["CP_iso_exp"])
    exp_tan_FM = exp_data["CT_tan_exp"]**1.5 / (np.sqrt(2.0) * exp_data["CP_tan_exp"])

    ax4.plot(exp_data["CT_iso_exp"], exp_iso_FM, "o", color=c_iso_exp, markersize=7, label="Exp: Isolated Rotor FM")
    ax4.plot(bemt_iso_CT, bemt_iso_FM, "-", color=c_iso_mod, linewidth=2.0, label="BEMT: Isolated Rotor FM")
    ax4.plot(exp_data["CT_tan_exp"], exp_tan_FM, "s", color=c_tan_exp, markersize=7, label="Exp: Tandem System FM")
    ax4.plot(bemt_tan_CT, bemt_tan_FM, "--", color=c_tan_mod, linewidth=2.2, label="BEMT: Tandem System FM")
    ax4.set_title("(d) Hover Figure of Merit ($FM$) vs. Thrust ($C_T$)", fontsize=12, fontweight="bold", pad=8)
    ax4.set_xlabel("Thrust Coefficient $C_T$ [-]", fontsize=11)
    ax4.set_ylabel("Figure of Merit $FM = C_T^{3/2} / (\\sqrt{2} C_P)$ [-]", fontsize=11)
    ax4.set_ylim([0.0, 0.85])
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="lower right", framealpha=0.9, fontsize=9.5)

    # Main title and caption
    fig.suptitle(
        "Tandem Rotor Aerodynamic Interference Validation\n"
        "Quantitative Comparison of BEMT Coleman Model vs. NACA TN-3236 Wind-Tunnel Data (Dingeldein, 1954)",
        fontsize=14,
        fontweight="bold",
        y=0.98,
    )

    caption_text = (
        "Flight Condition: Hover OGE, Sea Level Standard Atmosphere (rho = 1.225 kg/m^3). "
        "Benchmark: NACA TN-3236 dual 2-blade rotors (R = 1.372 m, c = 0.117 m, NACA 0012, untwisted). "
        f"Tandem Spacing: d/R = {TANDEM_SEP_D_OVER_R:.2f} (geometric overlap = {overlap*100:.1f}% disk area). "
        f"Model Accuracy: MAPE(CT) = {mape_CT:.1f}%, MAPE(CP) = {mape_CP:.1f}%."
    )
    fig.text(0.5, 0.015, caption_text, ha="center", fontsize=9.5, style="italic",
             bbox=dict(boxstyle="square,pad=0.4", fc="#f5f5f5", ec="#cccccc", alpha=0.85))

    plt.tight_layout(rect=[0.02, 0.04, 0.98, 0.94])

    plot_path = os.path.join(out_dir, "tandem_validation_dingeldein.png")
    fig.savefig(plot_path, dpi=300)
    plt.close(fig)

    print(f"Validation plot saved: {os.path.abspath(plot_path)}")
    return mape_CT, mape_CP


if __name__ == "__main__":
    run_tandem_validation()
