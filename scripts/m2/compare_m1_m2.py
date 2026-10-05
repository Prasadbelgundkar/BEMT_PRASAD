"""
Milestone 2 -- Sections 9.1 / 9.2: quantitative comparison with Milestone 1
and trade-off data.

9.1  Power required vs airspeed: Milestone 1 axial-flow predictions (hover by
     collective trim; airplane mode as an axial propeller sized by the M1 drag
     model q f + W^2/(q S pi e AR)) against the Milestone 2 trimmed aircraft
     (helicopter mode, airplane mode and the minimum-power envelope over all
     feasible nacelle angles of the corridor). Also stall, control and
     feasible-envelope comparison.
9.2  Rotor-variant trade-off table (uses both corridor caches if available).

Outputs: outputs/m2/rotor_<variant>/m2_9p1_power_comparison.png, m2_9_comparison.md
"""
import os

import numpy as np
from scipy.optimize import brentq

from _common import CFG, plt, save_figure, write_text, OUT_DIR
from bemt import run_bemt, trim_hover_collective
from m2.trim_6dof import trim_6dof, make_condition

M1 = CFG.M1
ALT = CFG.REFERENCE_ALTITUDE_M


def m1_hover(atm, rpm):
    W = CFG.GROSS_MASS_KG * CFG.G / 2
    c = trim_hover_collective(CFG.ROTOR, CFG.airfoil_provider, CFG.rpm_to_omega(rpm), W, atm.density_kg_m3,
                              atm.speed_of_sound_mps, coll_range_deg=(-5, 50))
    p = run_bemt(CFG.ROTOR, CFG.airfoil_provider, CFG.rpm_to_omega(rpm), np.radians(c), atm.density_kg_m3,
                 atm.speed_of_sound_mps)
    return c, 2 * p.power_W, p.stalled_fraction


def m1_axial_cruise(atm, V, rpm):
    """M1 method: thrust = q f + W^2/(q S pi e AR), shared by 2 rotors, axial BEMT."""
    q = 0.5 * atm.density_kg_m3 * V ** 2
    W = CFG.GROSS_MASS_KG * CFG.G
    D = q * M1.FLAT_PLATE_AREA_M2 + W ** 2 / (q * 39.24 * np.pi * 0.8 * 9.0)
    om = CFG.rpm_to_omega(rpm)
    f = lambda c: run_bemt(CFG.ROTOR, CFG.airfoil_provider, om, np.radians(c), atm.density_kg_m3,
                           atm.speed_of_sound_mps, v_axial=V, n_stations=30).thrust_N - D / 2
    try:
        c = brentq(f, 5.0, 70.0, xtol=0.05)
    except ValueError:
        return np.nan, np.nan
    p = run_bemt(CFG.ROTOR, CFG.airfoil_provider, om, np.radians(c), atm.density_kg_m3, atm.speed_of_sound_mps,
                 v_axial=V, n_stations=30)
    return c, 2 * p.power_W


def load_corridor(variant):
    path = os.path.join(os.path.dirname(OUT_DIR), f"rotor_{variant}", "corridor_grid.npz")
    return dict(np.load(path, allow_pickle=True)) if os.path.exists(path) else None


def main():
    atm = CFG.atmosphere(ALT)
    ac = CFG.get_default_aircraft()
    rpm_c = CFG.CONVERSION_RPM
    print(f"M1 vs M2 comparison, rotor '{CFG.ROTOR_VARIANT}'")
    # ---- M1 predictions ----
    c_h1, P_h1, st_h1 = m1_hover(atm, rpm_c)
    Vs = np.arange(50.0, 100.1, 5.0)
    P1_ax = np.array([m1_axial_cruise(atm, V, rpm_c)[1] for V in Vs])
    # ---- M2 corridor (same RPM, altitude, mass) ----
    g = load_corridor(CFG.ROTOR_VARIANT)
    fig, ax = plt.subplots(figsize=(11, 6.2))
    if g is not None:
        V, N, P = g["V"], g["nacelle"], g["P_req_kW"]
        conv = np.isin(g["status_code"], [0, 1])
        feas = g["feasible"] > 0.5
        i90 = int(np.argmin(abs(N - 90.0))); i0 = int(np.argmin(abs(N - 0.0)))
        m = conv[i90]
        ax.plot(V[m], P[i90][m], 'r-o', ms=4, label="M2 trimmed, helicopter mode (i_n = 90)")
        m = conv[i0]
        ax.plot(V[m], P[i0][m], 'b-o', ms=4, label="M2 trimmed, airplane mode (i_n = 0)")
        Pf = np.where(feas, P, np.nan)
        env = np.nanmin(Pf, axis=0)
        nbest = np.array([N[np.nanargmin(Pf[:, j])] if np.isfinite(env[j]) else np.nan for j in range(len(V))])
        ax.plot(V, env, 'g-', lw=3, alpha=0.7, label="M2 minimum-power envelope over feasible nacelle angles")
        for v, p, n in zip(V[::2], env[::2], nbest[::2]):
            if np.isfinite(p):
                ax.annotate(f"{n:.0f}", (v, p), textcoords='offset points', xytext=(0, -12), fontsize=7,
                            color='g', ha='center')
        P_av = g["P_avail_kW"][0, 0]
        ax.axhline(P_av, color='k', ls='--', lw=1, label=f"power available ({P_av:.0f} kW)")
    ax.plot([0], [P_h1 / 1e3], 'k*', ms=15, label="M1 hover (axisymmetric BEMT, collective trim)")
    ax.plot(Vs, P1_ax / 1e3, 'k--s', ms=5, mfc='white', label="M1 airplane mode (axial BEMT, M1 drag model)")
    ax.set_xlabel("True airspeed V [m/s]"); ax.set_ylabel("Total power required, both rotors [kW]")
    ax.set_title(f"Section 9.1 -- Milestone 1 (axial) vs Milestone 2 (trimmed edgewise / conversion) power, "
                 f"{ALT:.0f} m ISA, MTOW, {rpm_c:.0f} RPM\nrotor '{CFG.ROTOR_VARIANT}'; green numbers = nacelle angle "
                 f"of minimum power", fontsize=10)
    ax.legend(fontsize=8, loc='upper center')
    fig.tight_layout()
    save_figure(fig, "m2_9p1_power_comparison", "9.1",
                f"Total power required vs airspeed at {ALT:.0f} m ISA and MTOW, {rpm_c:.0f} RPM: Milestone 1 axial "
                f"predictions (hover and axial airplane mode with the M1 flat-plate + induced drag model) against "
                f"the Milestone 2 6-DOF trimmed aircraft in helicopter mode, airplane mode and the minimum-power "
                f"envelope over the feasible conversion corridor (annotated with the best nacelle angle).")

    # ---- M2 trimmed reference points ----
    t_h = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, make_condition(0.0, 90.0, rpm=rpm_c))
    t_c = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, make_condition(85.0, 0.0, rpm=CFG.AIRPLANE_RPM))
    t_40 = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, make_condition(40.0, 90.0, rpm=rpm_c))
    c_c1, P_c1 = m1_axial_cruise(atm, 85.0, CFG.AIRPLANE_RPM)
    md = ["# Section 9 -- Milestone 2 vs Milestone 1", "",
          f"Rotor '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}), {ALT:.0f} m ISA, {CFG.GROSS_MASS_KG:.0f} kg.", "",
          "## 9.1 Effect of edgewise flight, trim and conversion", "",
          "| Quantity | Milestone 1 (axial model) | Milestone 2 (trimmed) | Change |", "|---|---|---|---|",
          f"| Hover power, both rotors [kW] | {P_h1/1e3:.0f} | {t_h.P_req_W/1e3:.0f} | "
          f"{100*(t_h.P_req_W/P_h1 - 1):+.1f} % (trim cyclic, CG offset, 6-DOF balance) |",
          f"| Hover collective [deg] | {c_h1:.2f} | {t_h.collective_deg:.2f} (+ theta1c {t_h.theta1c_deg:.2f}) | "
          f"{t_h.collective_deg - c_h1:+.2f} deg |",
          f"| Hover stall | {100*st_h1:.0f} % of span | {100*t_h.stalled_fraction:.0f} % of loaded disk | - |",
          f"| Airplane mode 85 m/s, 420 RPM, power [kW] | {P_c1/1e3:.0f} | {t_c.P_req_W/1e3:.0f} | "
          f"{100*(t_c.P_req_W/P_c1 - 1):+.1f} % (wing profile + tail + trim drag, attitude) |",
          f"| Airplane mode collective [deg] | {c_c1:.1f} | {t_c.collective_deg:.1f} | {t_c.collective_deg - c_c1:+.1f} deg |",
          f"| Helicopter mode 40 m/s | not modelled (axial only) | P = {t_40.P_req_W/1e3:.0f} kW, stall "
          f"{100*t_40.stalled_fraction:.0f} %, theta1c = {t_40.theta1c_deg:.1f} deg, attitude {t_40.theta_deg:.1f} deg | "
          f"edgewise power bucket and retreating-blade stall appear |",
          f"| Control inputs | collective only | collective, theta1c, differential collective / theta1s, elevator, "
          f"flaperons, rudder (stick mixing) | 6-DOF trim |"]
    if g is not None:
        feas = g["feasible"] > 0.5
        env_ok = np.isfinite(env)
        md += [f"| Feasible steady operating points | hover/climb and axial airplane mode only | {int(feas.sum())} of "
               f"{feas.size} corridor points; min-power speed {V[env_ok][np.nanargmin(env[env_ok])]:.0f} m/s "
               f"({np.nanmin(env):.0f} kW) | conversion corridor bounded by wing stall, rotor stall, tip Mach, power, "
               f"control saturation |",
               f"| Lowest airplane-mode speed (i_n = 0) | not checked (M1 cruise 40 m/s) | "
               f"{V[feas[i0]].min() if feas[i0].any() else float('nan'):.0f} m/s (wing stall / no trim below) | - |"]
    # ---- 9.2 trade table across rotor variants ----
    md += ["", "## 9.2 Trade-off data: rotor variants", "",
           "| Metric | 'M1' rotor (25/-45, 500 RPM) | 'refined' rotor (12/-30, 540 RPM) |", "|---|---|---|"]
    cols = {}
    for var in ("M1", "refined"):
        gv = load_corridor(var)
        rot = CFG.ROTOR_VARIANTS[var]
        rpm_v = 540.0 if var == 'refined' else M1.HOVER_RPM
        th = trim_6dof(ac, rot, CFG.airfoil_provider, make_condition(0.0, 90.0, rpm=rpm_v))
        tc = trim_6dof(ac, rot, CFG.airfoil_provider, make_condition(85.0, 0.0, rpm=CFG.AIRPLANE_RPM))
        t4 = trim_6dof(ac, rot, CFG.airfoil_provider, make_condition(40.0, 90.0, rpm=rpm_v))
        cols[var] = [f"{th.P_req_W/1e3:.0f}", f"{100*th.stalled_fraction:.0f}", f"{th.stall_margin_deg:+.1f}",
                     f"{100*t4.stalled_fraction:.0f}", f"{tc.P_req_W/1e3:.0f}", f"{tc.collective_deg:.1f}",
                     f"{int((gv['feasible'] > 0.5).sum())}" if gv is not None else "n/a",
                     (f"{gv['V'][(gv['feasible'] > 0.5)[-1]].max():.0f}"
                      if gv is not None and (gv['feasible'] > 0.5)[-1].any() else "n/a")]
    names = ["Hover power (2000 m, MTOW) [kW]", "Hover stalled loaded area [%]", "Hover stall margin [deg]",
             "Helicopter mode 40 m/s stalled area [%]", "Airplane mode 85 m/s power [kW]",
             "Airplane mode 85 m/s collective [deg]", "Feasible corridor points (of 273)",
             "Max feasible speed at i_n = 90 [m/s]"]
    md += [f"| {n} | {a} | {b} |" for n, a, b in zip(names, cols["M1"], cols["refined"])]
    write_text("m2_9_comparison.md", "\n".join(md) + "\n")


if __name__ == "__main__":
    main()
