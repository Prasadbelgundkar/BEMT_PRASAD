"""
Milestone 2 -- Sections 5.2-5.5: design tables generated from the configuration.

  5.2  design changes from Milestone 1 and the issue that motivated each
  5.3  rotor design table (M1 vs M2 variants) + chord/twist distribution plot
  5.4  wing and empennage table
  5.5  mass breakdown, CG (helicopter / airplane mode, full / reserve fuel),
       nacelle, RPM, rotor-control and surface limits, control mixing

Outputs: outputs/m2/rotor_<variant>/m2_5_design_tables.md, m2_5p3_blade_distributions.png
"""
import numpy as np

from _common import CFG, plt, save_figure, write_text

M1 = CFG.M1


def rotor_table():
    rows = ["## 5.3 Rotor design", "",
            "| Parameter | Milestone 1 | M2 'M1' variant | M2 'refined' variant |", "|---|---|---|---|"]
    r1, rr = CFG.ROTOR_M1, CFG.ROTOR_REFINED
    sig = lambda r: r.solidity()
    tw = lambda r, x: np.degrees(r.twist_fn(x))
    entries = [
        ("Airfoil", M1.AIRFOIL_NAME, M1.AIRFOIL_NAME, M1.AIRFOIL_NAME),
        ("Stall angle (flag) [deg]", f"{np.degrees(M1.AIRFOIL.stall_alpha_rad):.0f}", "same", "same"),
        ("Radius R [m]", f"{M1.ROTOR_RADIUS_M}", "same", "same"),
        ("Number of blades", f"{M1.NUM_BLADES}", "same", "same"),
        ("Root cut-out [m] (r/R)", f"{M1.ROOT_CUTOUT_M} ({M1.ROOT_CUTOUT_M / M1.ROTOR_RADIUS_M:.3f})", "same", "same"),
        ("Chord: root / tip [m], taper", f"{M1.ROOT_CHORD_M} / {M1.ROOT_CHORD_M * M1.TAPER_RATIO:.3f}, "
                                          f"{M1.TAPER_RATIO}", "same", "same"),
        ("Solidity sigma", f"{sig(r1):.4f}", f"{sig(r1):.4f}", f"{sig(rr):.4f}"),
        ("Twist: root (r/R=0) / rate", "25 deg / -45 deg/R", "25 deg / -45 deg/R", "12 deg / -30 deg/R"),
        ("Built-in pitch at 0.75R [deg]", f"{tw(r1, 0.75):.2f}", f"{tw(r1, 0.75):.2f}", f"{tw(rr, 0.75):.2f}"),
        ("RPM helicopter / conversion", f"{M1.HOVER_RPM:.0f}", f"{M1.HOVER_RPM:.0f}", "540"),
        ("RPM airplane mode", f"{M1.CRUISE_RPM:.0f}", f"{CFG.AIRPLANE_RPM:.0f}", f"{CFG.AIRPLANE_RPM:.0f}"),
        ("Hover tip speed / Mach (2000 m)", f"{CFG.rpm_to_omega(M1.HOVER_RPM) * M1.ROTOR_RADIUS_M:.0f} m/s",
         f"{CFG.rpm_to_omega(M1.HOVER_RPM) * M1.ROTOR_RADIUS_M:.0f} m/s / "
         f"{CFG.rpm_to_omega(M1.HOVER_RPM) * M1.ROTOR_RADIUS_M / CFG.atmosphere().speed_of_sound_mps:.3f}",
         f"{CFG.rpm_to_omega(540) * M1.ROTOR_RADIUS_M:.0f} m/s / "
         f"{CFG.rpm_to_omega(540) * M1.ROTOR_RADIUS_M / CFG.atmosphere().speed_of_sound_mps:.3f}"),
        ("Collective range [deg]", f"{M1.MIN_COLLECTIVE_DEG:.0f} .. {M1.MAX_COLLECTIVE_DEG:.0f}",
         "{:.0f} .. {:.0f}".format(*CFG.ControlLimits().collective_deg), "same"),
        ("Cyclic range theta1c / theta1s [deg]", "- (axial only)",
         "{:.0f} .. {:.0f}".format(*CFG.ControlLimits().theta_1c_deg), "same"),
        ("Rotation", "-", "right CCW / left CW (seen from above, helicopter mode)", "same"),
    ]
    rows += [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in entries]
    return rows


def blade_plot():
    x = np.linspace(M1.ROOT_CUTOUT_M / M1.ROTOR_RADIUS_M, 1.0, 100)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.2))
    axs[0].plot(x, [CFG.ROTOR_M1.chord_fn(v) for v in x], 'k-', lw=2, label="both variants")
    axs[0].set_xlabel("r/R"); axs[0].set_ylabel("Chord [m]"); axs[0].legend()
    axs[0].set_title("Chord distribution (linear taper, unchanged)")
    for r, lab, st in ((CFG.ROTOR_M1, "M1 / 'M1' variant: 25 deg, -45 deg/R", 'k-'),
                       (CFG.ROTOR_REFINED, "'refined': 12 deg, -30 deg/R", 'r--')):
        axs[1].plot(x, [np.degrees(r.twist_fn(v)) for v in x], st, lw=2, label=lab)
    axs[1].axvline(0.75, color='0.6', ls=':')
    axs[1].set_xlabel("r/R"); axs[1].set_ylabel("Built-in twist [deg] (collective added)")
    axs[1].set_title("Twist distribution")
    axs[1].legend()
    fig.suptitle("Section 5.3 -- Blade chord and twist distributions", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save_figure(fig, "m2_5p3_blade_distributions", "5.3",
                "Blade chord (linear taper 0.90 -> 0.35 m, unchanged from Milestone 1) and built-in twist of the "
                "Milestone 1 blade and of the M2 'refined' blade (reduced root pitch and washout to remove inboard "
                "hover stall at MTOW / 2000 m).")


def wing_table(ac):
    w, t, v = ac.wing, ac.htail, ac.vtail
    cg = ac.cg_ref_m(90.0)
    rows = ["## 5.4 Wing and empennage", "",
            "| Parameter | Wing | Horizontal tail | Vertical tail |", "|---|---|---|---|",
            f"| Area [m^2] | {w.S_m2:.2f} | {t.S_m2:.2f} | {v.S_m2:.2f} |",
            f"| Span / height [m] | {w.span_m:.2f} | {t.span_m:.2f} | {v.height_m:.2f} |",
            f"| Aspect ratio | {w.AR:.1f} | {t.AR:.1f} | {v.AR:.1f} |",
            f"| Mean chord [m] | {w.chord_m:.2f} | {t.chord_m:.2f} | {v.S_m2 / v.height_m:.2f} |",
            f"| Aerodynamic model | thin airfoil a0=2pi, lifting-line CL_alpha={w.CL_alpha:.2f}/rad, e={w.e_oswald}, "
            f"CD0={w.CD0}, CM_ac={w.CM_ac}, stall {w.alpha_stall_deg:.0f} deg, post-stall CD90={w.CD90} | "
            f"a0=2pi, e={t.e_oswald}, CD0={t.CD0}, stall {t.alpha_stall_deg:.0f} deg, downwash 2CL_w/(pi AR_w) | "
            f"CL_alpha={v.CL_alpha:.2f}/rad (end-plate AR_eff=1.55 AR), CD0={v.CD0} |",
            f"| Incidence [deg] | {w.i_w_deg:.1f} | {t.i_t_deg:.1f} | 0 |",
            f"| Control surface | flaperons {w.aileron_eta[0]:.2f}-{w.aileron_eta[1]:.2f} semi-span, tau={w.aileron_tau}, "
            f"+/-{w.aileron_limit_deg:.0f} deg, Cl_da={w.Cl_delta_a:.3f}/rad | elevator {100*t.elevator_chord_frac:.0f} % "
            f"chord, tau_e={t.tau_e}, +/-{t.elevator_limit_deg:.0f} deg | rudder {100*v.rudder_chord_frac:.0f} % chord, "
            f"tau_r={v.tau_r}, +/-{v.rudder_limit_deg:.0f} deg |",
            f"| a.c. from reference point [m] (x fwd, z down) | {tuple(np.round(ac.wing_ac_ref_m, 2))} | "
            f"{tuple(np.round(ac.htail_ac_ref_m, 2))} | {tuple(np.round(ac.vtail_ac_ref_m, 2))} |",
            f"| a.c. from CG (helicopter mode) [m] | {tuple(np.round(ac.wing_ac_from_cg(90), 2))} | "
            f"{tuple(np.round(ac.htail_ac_from_cg(90), 2))} | {tuple(np.round(ac.vtail_ac_from_cg(90), 2))} |",
            f"| Tail volume coefficient | - | V_H = {t.S_m2 * (cg[0] - ac.htail_ac_ref_m[0]) / (w.S_m2 * w.chord_m):.3f} | "
            f"V_V = {v.S_m2 * (cg[0] - ac.vtail_ac_ref_m[0]) / (w.S_m2 * w.span_m):.3f} |",
            "", f"Fuselage + nacelle drag: flat-plate area {ac.flat_plate_area_m2} m^2 acting at the CG. "
                "Rotor-wake/wing and rotor-wake/tail interference and hover download are NOT modelled.",
            f"Nacelle pivots at {tuple(np.round(ac.nacelle_pivot_ref_m, 2))} m (and mirror), mast (pivot -> hub) "
            f"{ac.mast_length_m} m."]
    return rows


def mass_table(ac):
    rows = ["## 5.5 Mass properties, CG and limits", "",
            "| Item | Mass [kg] | x [m] | z [m] | Notes |", "|---|---|---|---|---|"]
    for it in ac.mass_items:
        if it.tilts_with_nacelle:
            p90 = ac.nacelle_pivot_ref_m + it.shaft_offset_m * ac.shaft_axis_body(90)
            p0 = ac.nacelle_pivot_ref_m + it.shaft_offset_m * ac.shaft_axis_body(0)
            rows.append(f"| {it.name} | {it.mass_kg:.0f} | {p90[0]:.2f} / {p0[0]:.2f} | {p90[2]:.2f} / {p0[2]:.2f} | "
                        f"tilts with nacelle (helicopter / airplane), y = +/-{ac.nacelle_pivot_ref_m[1]:.1f} m |")
        else:
            rows.append(f"| {it.name} | {it.mass_kg:.0f} | {it.x_m:.2f} | {it.z_m:.2f} | {it.category} |")
    rows.append(f"| **Total (MTOW)** | **{ac.mass_kg():.0f}** | | | empty {M1.EMPTY_MASS_KG:.0f} + payload "
                f"{M1.PAYLOAD_KG:.0f} + fuel {M1.FUEL_MASS_KG:.0f} |")
    rows += ["", "| Loading | i_n [deg] | mass [kg] | CG x [m] | CG z [m] |", "|---|---|---|---|---|"]
    for fuel, lab in ((M1.FUEL_MASS_KG, "MTOW, full fuel"), (M1.RESERVE_FUEL_KG, "reserve fuel")):
        for n in (90.0, 45.0, 0.0):
            cg = ac.cg_ref_m(n, fuel_kg=fuel)
            rows.append(f"| {lab} | {n:.0f} | {ac.mass_kg(fuel_kg=fuel):.0f} | {cg[0]:+.3f} | {cg[2]:+.3f} |")
    L, mx = ac.limits, ac.mixing
    rows += ["", "| Limit | Value |", "|---|---|",
             f"| Nacelle angle | {L.nacelle_deg[0]:.0f} .. {L.nacelle_deg[1]:.0f} deg, max rate {L.nacelle_rate_deg_s} deg/s |",
             f"| Rotor RPM | {L.rpm[0]:.0f} .. {L.rpm[1]:.0f} (schedule {CFG.HOVER_RPM:.0f} helicopter/conversion, "
             f"{CFG.AIRPLANE_RPM:.0f} airplane) |",
             f"| Collective | {L.collective_deg[0]:.0f} .. {L.collective_deg[1]:.0f} deg |",
             f"| Longitudinal / lateral cyclic | +/-{L.theta_1c_deg[1]:.0f} / +/-{L.theta_1s_deg[1]:.0f} deg |",
             f"| Elevator / flaperon / rudder | +/-{L.elevator_deg[1]:.0f} / +/-{ac.wing.aileron_limit_deg:.0f} / "
             f"+/-{L.rudder_deg[1]:.0f} deg |",
             f"| Pitch / roll attitude (trim bounds) | {L.attitude_deg[0]:.0f} .. {L.attitude_deg[1]:.0f} / "
             f"+/-{L.roll_deg[1]:.0f} deg |",
             f"| Advancing-tip Mach | {L.max_tip_mach} |",
             f"| Rotor stalled loaded area | {100 * L.max_stall_fraction:.0f} % |",
             f"| Reverse-flow area | {100 * L.max_reverse_flow_fraction:.0f} % of disk |",
             f"| Power margin | {100 * L.min_power_margin_frac:.0f} % of available |",
             f"| Installed power | 2 x {CFG.POWER_PER_ENGINE_SL_W / 1e3:.0f} kW (SL), lapse (rho/rho0)^"
             f"{M1.DENSITY_RATIO_EXPONENT}, drivetrain eff. {M1.DRIVETRAIN_EFFICIENCY} |",
             "", "Control mixing (normalized stick -> effector, rotor terms x sin^2(i_n)):", "",
             f"* pitch: theta1c = {mx.K_cyc_deg:.0f} deg x d_lon, elevator = {mx.K_e_deg:.0f} deg x d_lon",
             f"* roll: differential collective = {mx.K_dcol_deg:.0f} deg x d_lat, flaperons = {mx.K_a_deg:.0f} deg x d_lat",
             f"* yaw: differential lateral cyclic = {mx.K_dcyc_deg:.0f} deg x d_ped, rudder = {mx.K_r_deg:.0f} deg x d_ped"]
    return rows


def main():
    ac = CFG.get_default_aircraft()
    md = ["# Section 5 -- updated tiltrotor design (generated from src/m2/aircraft_input_m2.py)", "",
          f"Active rotor variant: '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}).", "",
          "## 5.2 Changes from Milestone 1", "", "| Change | Motivating issue |", "|---|---|"]
    md += [f"| {c} | {why} |" for c, why in CFG.DESIGN_CHANGES]
    md += [""] + rotor_table() + [""] + wing_table(ac) + [""] + mass_table(ac)
    write_text("m2_5_design_tables.md", "\n".join(md) + "\n")
    blade_plot()


if __name__ == "__main__":
    main()
