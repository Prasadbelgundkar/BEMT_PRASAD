"""
plot_tandem_hover_assessment.py

Hover performance assessment for the CH-47 class tandem baseline.
Compares uncoupled vs. interference-augmented BEMT at MTOW trim across
two conditions: Sea Level ISA and Hot-and-High (1500 m, ISA+15 K).

Outputs: outputs/mp/tandem_hover_assessment.png
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Ensure src/ is in the module search path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from environment import isa
from airfoil import LinearAirfoil
from bemt import run_bemt
from tandem_interference import (
    solve_tandem_bemt,
    trim_tandem_hover,
    compute_overlap_fraction,
)
from tandem_rotor_config import (
    build_fwd_rotor,
    build_aft_rotor,
    FWD_ROTOR_RADIUS_M,
    AFT_ROTOR_RADIUS_M,
    LONGITUDINAL_SEP_M,
    LATERAL_OFFSET_M,
    VERTICAL_STAGGER_M,
    HOVER_OMEGA,
    GROSS_MASS_KG,
    G,
    INSTALLED_SHAFT_POWER_W,
    DRIVETRAIN_EFFICIENCY,
)


def run_hover_assessment():
    print("=" * 85)
    print("TANDEM HELICOPTER ROTORCRAFT ASSESSMENT: BASELINE VS. AUGMENTED INTERFERENCE")
    print("=" * 85)

    r_fwd = build_fwd_rotor()
    r_aft = build_aft_rotor()

    # Baseline linear airfoil
    airfoil = LinearAirfoil(
        a0=5.73,
        Cd_min=0.0095,
        eps=1.15,
        stall_alpha_rad=np.radians(14.0),
        Cl_max_clip=1.45,
    )
    airfoil_fn = lambda x: airfoil

    target_thrust_N = GROSS_MASS_KG * G  # 22,680 kg * 9.80665 = 222,414 N
    installed_usable_power_kW = (INSTALLED_SHAFT_POWER_W * DRIVETRAIN_EFFICIENCY) / 1e3

    overlap = compute_overlap_fraction(
        R_fwd=r_fwd.radius_m,
        R_aft=r_aft.radius_m,
        longitudinal_sep_m=LONGITUDINAL_SEP_M,
        lateral_offset_m=LATERAL_OFFSET_M,
    )

    print(f"Aircraft Gross Mass : {GROSS_MASS_KG:,.0f} kg (Weight = {target_thrust_N / 1e3:.2f} kN)")
    print(f"Rotor Radii         : R_fwd = {r_fwd.radius_m:.2f} m, R_aft = {r_aft.radius_m:.2f} m")
    print(f"Longitudinal Spacing: {LONGITUDINAL_SEP_M:.2f} m ({LONGITUDINAL_SEP_M / r_fwd.radius_m:.2f} R)")
    print(f"Disk Overlap Area   : {overlap * 100:.2f}% of aft rotor disk area")
    print(f"Installed Max Power : {installed_usable_power_kW:,.0f} kW (usable shaft power)")
    print("-" * 85)

    # Hover trim at MTOW across conditions
    conditions = [
        {"name": "Sea Level ISA", "alt_m": 0.0, "disa_K": 0.0},
        {"name": "Hot-and-High (1500m, ISA+15K)", "alt_m": 1500.0, "disa_K": 15.0},
    ]

    trim_results = {}

    print(f"{'Condition':<30} | {'Mode':<11} | {'Fwd Coll':<9} | {'Aft Coll':<9} | {'P_req [kW]':<11} | {'FM_sys':<7} | {'Delta_v':<8}")
    print("-" * 96)

    for cond in conditions:
        atmo = isa(cond["alt_m"], cond["disa_K"])
        c_name = cond["name"]

        # A. Baseline uncoupled trim (include_interference=False)
        trim_base = trim_tandem_hover(
            rotor_fwd=r_fwd,
            rotor_aft=r_aft,
            airfoil_provider_fwd=airfoil_fn,
            airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=HOVER_OMEGA,
            target_thrust_total_N=target_thrust_N,
            rho=atmo.density_kg_m3,
            a_sound=atmo.speed_of_sound_mps,
            thrust_split_fwd=0.5,
            longitudinal_sep_m=LONGITUDINAL_SEP_M,
            lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            include_interference=False,
        )

        # B. Augmented interference trim (include_interference=True)
        trim_aug = trim_tandem_hover(
            rotor_fwd=r_fwd,
            rotor_aft=r_aft,
            airfoil_provider_fwd=airfoil_fn,
            airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=HOVER_OMEGA,
            target_thrust_total_N=target_thrust_N,
            rho=atmo.density_kg_m3,
            a_sound=atmo.speed_of_sound_mps,
            thrust_split_fwd=0.5,
            longitudinal_sep_m=LONGITUDINAL_SEP_M,
            lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            include_interference=True,
        )

        trim_results[c_name] = {
            "atmo": atmo,
            "baseline": trim_base,
            "augmented": trim_aug,
        }

        th_f_b, th_a_b, p_b = trim_base
        th_f_a, th_a_a, p_a = trim_aug

        P_base_kW = p_b.total_power_W / 1e3
        P_aug_kW = p_a.total_power_W / 1e3

        print(f"{c_name:<30} | {'Baseline':<11} | {th_f_b:7.2f}°  | {th_a_b:7.2f}°  | {P_base_kW:9.1f} kW | {p_b.figure_of_merit_system:7.3f} | {0.0:6.2f} m/s")
        print(f"{'':<30} | {'Augmented':<11} | {th_f_a:7.2f}°  | {th_a_a:7.2f}°  | {P_aug_kW:9.1f} kW | {p_a.figure_of_merit_system:7.3f} | {p_a.delta_v_axial_mps:6.2f} m/s")
        print("-" * 96)

    # Collective sweep for polar curves (SL ISA)
    atmo_sl = isa(0.0, 0.0)
    rho_sl = atmo_sl.density_kg_m3
    a_sl = atmo_sl.speed_of_sound_mps

    coll_sweep_deg = np.linspace(4.0, 14.0, 45)

    base_CT, base_CP, base_FM = [], [], []
    aug_CT, aug_CP, aug_FM = [], [], []

    for th_deg in coll_sweep_deg:
        th_rad = np.radians(th_deg)

        res_b = solve_tandem_bemt(
            rotor_fwd=r_fwd, rotor_aft=r_aft,
            airfoil_provider_fwd=airfoil_fn, airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=HOVER_OMEGA,
            collective_fwd_rad=th_rad, collective_aft_rad=th_rad,
            rho=rho_sl, a_sound=a_sl,
            longitudinal_sep_m=LONGITUDINAL_SEP_M, lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            include_interference=False,
            n_stations=80,
        )
        base_CT.append(res_b.total_CT)
        base_CP.append(res_b.total_CP)
        base_FM.append(res_b.figure_of_merit_system if res_b.figure_of_merit_system is not None else np.nan)

        res_a = solve_tandem_bemt(
            rotor_fwd=r_fwd, rotor_aft=r_aft,
            airfoil_provider_fwd=airfoil_fn, airfoil_provider_aft=airfoil_fn,
            omega_fwd_rad_s=HOVER_OMEGA,
            collective_fwd_rad=th_rad, collective_aft_rad=th_rad,
            rho=rho_sl, a_sound=a_sl,
            longitudinal_sep_m=LONGITUDINAL_SEP_M, lateral_offset_m=LATERAL_OFFSET_M,
            vertical_stagger_m=VERTICAL_STAGGER_M,
            include_interference=True,
            n_stations=80,
        )
        aug_CT.append(res_a.total_CT)
        aug_CP.append(res_a.total_CP)
        aug_FM.append(res_a.figure_of_merit_system if res_a.figure_of_merit_system is not None else np.nan)

    base_CT = np.array(base_CT)
    base_CP = np.array(base_CP)
    base_FM = np.array(base_FM)

    aug_CT = np.array(aug_CT)
    aug_CP = np.array(aug_CP)
    aug_FM = np.array(aug_FM)

    # Spanwise loading at trim point
    _, _, perf_trim_aug = trim_results["Sea Level ISA"]["augmented"]
    _, _, perf_trim_base = trim_results["Sea Level ISA"]["baseline"]

    r_fwd_elem = np.array([e.r for e in perf_trim_aug.fwd.elements])
    x_span = r_fwd_elem / r_fwd.radius_m

    dT_dr_fwd = np.array([e.dT_dr for e in perf_trim_aug.fwd.elements])
    dT_dr_aft_aug = np.array([e.dT_dr for e in perf_trim_aug.aft.elements])
    dT_dr_aft_base = np.array([e.dT_dr for e in perf_trim_base.aft.elements])

    # Figure
    out_dir = os.path.join(PROJECT_ROOT, "outputs", "mp")
    os.makedirs(out_dir, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(15, 11.5))
    fig.patch.set_facecolor("#ffffff")

    c_base = "#1f77b4"  # deep blue
    c_aug = "#d62728"   # deep red
    c_fwd = "#2ca02c"   # forest green
    c_aft = "#ff7f0e"   # orange

    # Subplot 1: Hover Polar (CP vs CT)
    ax1 = axes[0, 0]
    ax1.plot(base_CT, base_CP * 1e3, "-", color=c_base, linewidth=2.4, label="Baseline (No Interference)")
    ax1.plot(aug_CT, aug_CP * 1e3, "--", color=c_aug, linewidth=2.4, label="Augmented (Coleman Interference)")

    # Mark MTOW operating point
    target_CT_sl = perf_trim_aug.total_CT
    target_CP_base = perf_trim_base.total_CP
    target_CP_aug = perf_trim_aug.total_CP

    ax1.scatter([target_CT_sl], [target_CP_base * 1e3], color=c_base, s=80, zorder=5, label=f"MTOW Baseline (CP={target_CP_base*1e3:.3f})")
    ax1.scatter([target_CT_sl], [target_CP_aug * 1e3], color=c_aug, s=80, marker="s", zorder=5, label=f"MTOW Augmented (CP={target_CP_aug*1e3:.3f})")

    ax1.annotate(
        f"+{(target_CP_aug - target_CP_base)/target_CP_base*100:.1f}% Power Penalty\nat MTOW Hover",
        xy=(target_CT_sl, target_CP_aug * 1e3),
        xytext=(target_CT_sl - 0.0022, target_CP_aug * 1e3 + 0.18),
        arrowprops=dict(facecolor="#333333", shrink=0.08, width=1.0, headwidth=6),
        fontsize=9.5, fontweight="semibold",
        bbox=dict(boxstyle="round,pad=0.3", fc="#fff9e6", ec="#cc9900", alpha=0.9),
    )

    ax1.set_title("(a) Hover Polar: Power Coefficient ($C_P$) vs. Thrust ($C_T$)", fontsize=12, fontweight="bold", pad=8)
    ax1.set_xlabel("Rotorcraft Thrust Coefficient $C_T$ [-]", fontsize=11)
    ax1.set_ylabel("Power Coefficient $C_P \\times 10^3$ [-]", fontsize=11)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Subplot 2: Figure of Merit vs CT
    ax2 = axes[0, 1]
    ax2.plot(base_CT, base_FM, "-", color=c_base, linewidth=2.4, label="Baseline FM")
    ax2.plot(aug_CT, aug_FM, "--", color=c_aug, linewidth=2.4, label="Augmented FM (Interference)")

    fm_base_mtow = perf_trim_base.figure_of_merit_system
    fm_aug_mtow = perf_trim_aug.figure_of_merit_system
    ax2.scatter([target_CT_sl], [fm_base_mtow], color=c_base, s=80, zorder=5)
    ax2.scatter([target_CT_sl], [fm_aug_mtow], color=c_aug, s=80, marker="s", zorder=5)

    ax2.annotate(
        f"$\\Delta FM = {fm_aug_mtow - fm_base_mtow:.3f}$\n({(fm_aug_mtow - fm_base_mtow)/fm_base_mtow*100:.1f}%)",
        xy=(target_CT_sl, fm_aug_mtow),
        xytext=(target_CT_sl + 0.0008, fm_aug_mtow - 0.09),
        arrowprops=dict(facecolor="#333333", shrink=0.08, width=1.0, headwidth=6),
        fontsize=9.5, fontweight="semibold",
        bbox=dict(boxstyle="round,pad=0.3", fc="#fff0f0", ec="#cc0000", alpha=0.9),
    )

    ax2.set_title("(b) System Figure of Merit ($FM$) vs. Thrust Coefficient ($C_T$)", fontsize=12, fontweight="bold", pad=8)
    ax2.set_xlabel("Rotorcraft Thrust Coefficient $C_T$ [-]", fontsize=11)
    ax2.set_ylabel("Figure of Merit $FM = C_T^{3/2} / (\\sqrt{2} C_P)$ [-]", fontsize=11)
    ax2.set_ylim([0.45, 0.82])
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="lower right", framealpha=0.9, fontsize=9.5)

    # Subplot 3: Bar Chart of Power Required across Flight Conditions
    ax3 = axes[1, 0]
    bar_width = 0.35
    x_indices = np.arange(len(conditions))

    p_base_vals = [trim_results[c["name"]]["baseline"][2].total_power_W / 1e3 for c in conditions]
    p_aug_vals = [trim_results[c["name"]]["augmented"][2].total_power_W / 1e3 for c in conditions]

    rects1 = ax3.bar(x_indices - bar_width/2, p_base_vals, bar_width, label="Baseline (No Interference)", color=c_base, alpha=0.9)
    rects2 = ax3.bar(x_indices + bar_width/2, p_aug_vals, bar_width, label="Augmented (With Interference)", color=c_aug, alpha=0.9)

    # Installed power limit horizontal line
    ax3.axhline(installed_usable_power_kW, color="#333333", linestyle="-.", linewidth=1.8, label=f"Installed Usable Power ({installed_usable_power_kW:,.0f} kW)")

    # Label heights and Delta P
    for idx, (pb, pa) in enumerate(zip(p_base_vals, p_aug_vals)):
        dp = pa - pb
        dp_pct = dp / pb * 100.0
        ax3.text(idx - bar_width/2, pb + 60, f"{pb:,.0f} kW", ha="center", fontsize=9, fontweight="bold")
        ax3.text(idx + bar_width/2, pa + 60, f"{pa:,.0f} kW", ha="center", fontsize=9, fontweight="bold", color="#990000")
        ax3.text(idx, pa + 320, f"+{dp:,.0f} kW (+{dp_pct:.1f}%)", ha="center", fontsize=9.5, fontweight="bold",
                 bbox=dict(boxstyle="round,pad=0.2", fc="#ffffcc", ec="#ccaa00", alpha=0.85))

    ax3.set_title("(c) Hover Power Required at MTOW (22,680 kg)", fontsize=12, fontweight="bold", pad=8)
    ax3.set_xticks(x_indices)
    ax3.set_xticklabels([c["name"] for c in conditions], fontsize=10.5, fontweight="semibold")
    ax3.set_ylabel("Hover Shaft Power Required [kW]", fontsize=11)
    ax3.set_ylim([0, installed_usable_power_kW * 1.15])
    ax3.grid(True, linestyle=":", alpha=0.6, axis="y")
    ax3.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Subplot 4: Spanwise Thrust Distribution (dT/dr) at Trim
    ax4 = axes[1, 1]
    ax4.plot(x_span, dT_dr_fwd / 1e3, "-", color=c_fwd, linewidth=2.2, label=f"Forward Rotor (Trim Coll = {trim_results['Sea Level ISA']['augmented'][0]:.2f}°)")
    ax4.plot(x_span, dT_dr_aft_aug / 1e3, "--", color=c_aft, linewidth=2.2, label=f"Aft Rotor (Augmented Coll = {trim_results['Sea Level ISA']['augmented'][1]:.2f}°)")
    ax4.plot(x_span, dT_dr_aft_base / 1e3, ":", color=c_base, linewidth=2.0, label=f"Aft Rotor (Baseline Coll = {trim_results['Sea Level ISA']['baseline'][1]:.2f}°)")

    ax4.set_title("(d) Spanwise Thrust Loading ($dT/dr$) at Hover Trim (SL ISA)", fontsize=12, fontweight="bold", pad=8)
    ax4.set_xlabel("Nondimensional Radial Station $r/R$ [-]", fontsize=11)
    ax4.set_ylabel("Sectional Thrust Loading $dT/dr$ [kN/m]", fontsize=11)
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

    # Main title and caption
    fig.suptitle(
        "Tandem Rotor Helicopter Aerodynamic Interference Assessment (CH-47 Class Baseline)\n"
        "Quantitative Design Impact: Hover Power Margin, Figure of Merit, and Trim Requirements",
        fontsize=14, fontweight="bold", y=0.98,
    )

    p_sl_base = p_base_vals[0]
    p_sl_aug = p_aug_vals[0]
    p_hh_aug = p_aug_vals[1]
    delta_p_sl = p_sl_aug - p_sl_base

    caption_text = (
        f"Aircraft: Tandem Helicopter MTOW = {GROSS_MASS_KG:,.0f} kg ({target_thrust_N/1e3:.1f} kN), 2x3-blade rotors (R = 9.14 m, d = 13.60 m, overlap = {overlap*100:.1f}%). "
        f"At Sea Level ISA, interference increases hover power by +{delta_p_sl:,.0f} kW (+{(delta_p_sl/p_sl_base)*100:.1f}%) and requires +{trim_results['Sea Level ISA']['augmented'][1] - trim_results['Sea Level ISA']['augmented'][0]:.2f}° "
        f"differential collective trim on the aft rotor. At Hot-and-High (1500 m, ISA+15K), hover power increases to {p_hh_aug:,.0f} kW "
        f"({p_hh_aug/installed_usable_power_kW*100:.1f}% of installed power)."
    )
    fig.text(0.5, 0.015, caption_text, ha="center", fontsize=9.2, style="italic",
             bbox=dict(boxstyle="square,pad=0.4", fc="#f8f8f8", ec="#cccccc", alpha=0.85))

    plt.tight_layout(rect=[0.02, 0.04, 0.98, 0.94])

    plot_path = os.path.join(out_dir, "tandem_hover_assessment.png")
    fig.savefig(plot_path, dpi=300)
    plt.close(fig)

    print(f"Rotorcraft assessment plot saved: {os.path.abspath(plot_path)}")
    return trim_results


if __name__ == "__main__":
    run_hover_assessment()
