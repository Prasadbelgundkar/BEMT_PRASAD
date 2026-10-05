"""
Milestone 2 -- Sections 6.1, 6.2, 6.4: trimmed-flight solutions.

Trim matrix: 4 nacelle angles x 3 airspeeds (6-DOF trim, src/m2/trim_6dof.py)
at 2000 m ISA and MTOW. Each nacelle row uses continuation (the previous
speed's solution seeds the next).

Outputs (outputs/m2/rotor_<variant>/):
  m2_6p2_trim_table.csv / .md      full trim-results table (Section 6.2)
  m2_6p4_trim_trends.png           attitude, controls, power vs speed (Section 6.4)
  m2_6p4_lift_sharing.png          rotor vs wing lift and rotor thrust direction (Section 6.4)

Run:  python scripts/m2/demo_trim_matrix.py        (M2_ROTOR=refined for the refined rotor)
"""
import csv
import os
import time

import numpy as np

from _common import CFG, plt, save_figure, write_text, OUT_DIR, RIGID_DISK_NOTE
from m2.trim_6dof import trim_6dof, make_condition

MATRIX = {            # nacelle angle [deg] -> airspeeds [m/s]
    90.0: [0.0, 20.0, 40.0],
    60.0: [30.0, 45.0, 60.0],
    30.0: [50.0, 65.0, 80.0],
    0.0: [70.0, 85.0, 100.0],
}
COLORS = {90.0: 'tab:red', 60.0: 'tab:orange', 30.0: 'tab:green', 0.0: 'tab:blue'}


def rpm_for(nacelle_deg):
    """RPM schedule: helicopter/conversion RPM until the nacelles are down."""
    return CFG.AIRPLANE_RPM if nacelle_deg <= 0.0 else CFG.HOVER_RPM


def run_matrix(ac):
    rows = []
    for nac, speeds in MATRIX.items():
        x0 = None
        for V in speeds:
            t0 = time.time()
            cond = make_condition(V, nac, rpm=rpm_for(nac))
            tr = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond, x0=x0)
            if tr.status.startswith('ok'):
                x0 = tr.x
            print(f"  i_n={nac:4.0f}  V={V:5.1f}  {tr.status:20s} feasible={tr.feasible!s:5}  "
                  f"flags={tr.flags}  |r|={tr.residual_norm:.1e}  ({time.time() - t0:.1f} s)")
            rows.append(tr)
    return rows


def table(rows):
    hdr = ["V [m/s]", "i_n [deg]", "RPM", "theta [deg]", "phi [deg]", "alpha [deg]",
           "theta0 [deg]", "theta1c [deg]", "diff theta1s [deg]", "diff coll [deg]",
           "elevator [deg]", "aileron [deg]", "rudder [deg]",
           "FX [N]", "FY [N]", "FZ [N]", "MX [N m]", "MY [N m]", "MZ [N m]",
           "P_req [kW]", "P_avail [kW]", "power margin [%]", "rotor lift [%W]", "wing lift [%W]",
           "rotor stall [%]", "stall margin [deg]", "reverse flow [%]", "adv tip Mach",
           "status", "flags"]
    data = []
    for t in rows:
        W = CFG.GROSS_MASS_KG * CFG.G
        data.append([t.cond.V_mps, t.cond.nacelle_deg, t.cond.rpm, t.theta_deg, t.phi_deg, t.alpha_deg,
                     t.collective_deg, t.theta1c_deg, t.dcyc_deg, t.dcoll_deg,
                     t.elevator_deg, t.aileron_deg, t.rudder_deg,
                     *t.F_res_N, *t.M_res_Nm,
                     t.P_req_W / 1e3, t.P_avail_W / 1e3, 100 * t.power_margin_frac,
                     100 * t.rotor_lift_N / W, 100 * t.wing_lift_N / W,
                     100 * t.stalled_fraction, t.stall_margin_deg, 100 * t.reverse_flow_fraction,
                     t.adv_tip_mach, t.status, ";".join(t.flags) or "-"])
    with open(os.path.join(OUT_DIR, "m2_6p2_trim_table.csv"), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(data)
    fmt = lambda v: (f"{v:.3g}" if abs(v) < 1e-2 and v != 0 else f"{v:.2f}") if isinstance(v, float) else str(v)
    md = ["# Section 6.2 -- trim results", "",
          f"6-DOF trim at {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, m = {CFG.GROSS_MASS_KG:.0f} kg, gamma = 0, "
          f"beta = 0. Rotor variant '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}). "
          "FX..MZ are the RESIDUAL aircraft loads at the trim solution (body axes, about the CG). "
          "theta1c applies to both rotors; differential controls are right-minus-left halves. "
          "Lift shares are earth-vertical components as % of weight.", "",
          "| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    md += ["| " + " | ".join(fmt(v) for v in row) + " |" for row in data]
    write_text("m2_6p2_trim_table.md", "\n".join(md) + "\n")
    print(f"  saved {os.path.relpath(os.path.join(OUT_DIR, 'm2_6p2_trim_table.csv'))}")


def plots(rows):
    W = CFG.GROSS_MASS_KG * CFG.G
    fig, axs = plt.subplots(2, 3, figsize=(15, 8.2))
    panels = [("theta_deg", "Pitch attitude theta [deg]"), ("collective_deg", "Collective theta0 [deg]"),
              ("theta1c_deg", "Longitudinal cyclic theta1c [deg]"), ("elevator_deg", "Elevator [deg]"),
              ("P_req_W", "Total power required [kW]"), ("rotor_lift_N", "Rotor share of lift [% W]")]
    for ax, (attr, lab) in zip(axs.flat, panels):
        for nac in MATRIX:
            pts = [t for t in rows if t.cond.nacelle_deg == nac]
            ok = [t for t in pts if t.status.startswith('ok')]
            if not ok:
                continue
            V = [t.cond.V_mps for t in ok]
            y = np.array([getattr(t, attr) for t in ok], float)
            if attr == 'P_req_W':
                y = y / 1e3
            if attr == 'rotor_lift_N':
                y = 100 * y / W
            feas = [t.feasible for t in ok]
            ax.plot(V, y, '-', color=COLORS[nac], lw=1.8, label=f"i_n = {nac:.0f} deg")
            ax.scatter([v for v, f in zip(V, feas) if f], [yy for yy, f in zip(y, feas) if f],
                       color=COLORS[nac], s=40, zorder=3)
            ax.scatter([v for v, f in zip(V, feas) if not f], [yy for yy, f in zip(y, feas) if not f],
                       facecolors='white', edgecolors=COLORS[nac], s=40, zorder=3)
        if attr == 'P_req_W':
            ax.axhline(rows[0].P_avail_W / 1e3, color='k', ls='--', lw=1.2, label="P available (2 engines)")
        ax.set_xlabel("Airspeed V [m/s]")
        ax.set_ylabel(lab)
    axs[0, 0].legend()
    axs[1, 1].legend()
    fig.suptitle(f"Section 6.4 -- Trim trends through conversion, {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, "
                 f"{CFG.GROSS_MASS_KG:.0f} kg, level flight, rotor '{CFG.ROTOR_VARIANT}'\n"
                 f"filled marker = feasible, open marker = trimmed but a limit is exceeded "
                 f"(see m2_6p2_trim_table.md); RPM {CFG.HOVER_RPM:.0f} (i_n > 0), {CFG.AIRPLANE_RPM:.0f} (i_n = 0)",
                 fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save_figure(fig, "m2_6p4_trim_trends", "6.4",
                f"6-DOF trim solutions vs airspeed for four nacelle angles: pitch attitude, collective, "
                f"longitudinal cyclic (phased out as sin^2 i_n), elevator, total power vs available and "
                f"rotor share of lift. {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, MTOW, level unaccelerated flight. "
                f"{RIGID_DISK_NOTE}; no rotor-wake/wing interference.")

    ok = [t for t in rows if t.status.startswith('ok')]
    labels = [f"{t.cond.nacelle_deg:.0f}/{t.cond.V_mps:.0f}" for t in ok]
    xs = np.arange(len(ok))
    rl = np.array([t.rotor_lift_N for t in ok]) / W * 100
    wl = np.array([t.wing_lift_N for t in ok]) / W * 100
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.4))
    ax = axs[0]
    ax.bar(xs, rl, color='tab:purple', label="rotors (vertical component)")
    ax.bar(xs, wl, bottom=np.where(wl >= 0, rl, 0), color='tab:cyan', label="wing + tail")
    ax.axhline(100, color='k', ls=':', lw=1)
    ax.set_xticks(xs); ax.set_xticklabels(labels, rotation=45)
    ax.set_xlabel("Nacelle angle / airspeed  [deg / m/s]")
    ax.set_ylabel("Share of weight carried [%]")
    ax.set_title("Lift sharing (earth-vertical components)")
    ax.legend()
    ax = axs[1]
    for t, x in zip(ok, xs):
        Fr = t.right.forces_body_N + t.left.forces_body_N
        ang = np.degrees(np.arctan2(-Fr[2], Fr[0]))          # angle of rotor force above the body x-axis
        ax.bar(x, ang, color=COLORS[t.cond.nacelle_deg])
    ax.set_xticks(xs); ax.set_xticklabels(labels, rotation=45)
    ax.set_xlabel("Nacelle angle / airspeed  [deg / m/s]")
    ax.set_ylabel("Rotor force direction above body x-axis [deg]")
    ax.set_title("Rotor thrust-vector orientation (90 = vertical, 0 = forward)")
    fig.suptitle(f"Section 6.4 -- Rotor/wing lift sharing and thrust-vector tilt through conversion "
                 f"(rotor '{CFG.ROTOR_VARIANT}', {CFG.REFERENCE_ALTITUDE_M:.0f} m, MTOW)", fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save_figure(fig, "m2_6p4_lift_sharing", "6.4",
                "Share of the weight carried by the rotors and by the wing + tail (earth-vertical components) "
                "and direction of the resultant rotor force relative to the body x-axis at each trimmed point "
                "of the matrix; labels are nacelle angle / airspeed. The wing takes over as the nacelles tilt "
                "and speed rises; in airplane mode the rotors only overcome drag.")


def main():
    t0 = time.time()
    ac = CFG.get_default_aircraft()
    print(f"Trim matrix, rotor '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}), hover RPM {CFG.HOVER_RPM:.0f}")
    rows = run_matrix(ac)
    table(rows)
    plots(rows)
    n_ok = sum(t.status.startswith('ok') for t in rows)
    n_f = sum(t.feasible for t in rows)
    print(f"{n_ok}/{len(rows)} trimmed, {n_f}/{len(rows)} feasible; {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
