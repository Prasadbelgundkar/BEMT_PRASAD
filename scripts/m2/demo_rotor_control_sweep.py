"""
Milestone 2 -- Section 4: pilot-input response of ONE rotor.

For each test condition the aircraft is first trimmed (longitudinal trim,
symmetric twin-rotor aircraft), then collective, longitudinal cyclic (theta1c)
and lateral cyclic (theta1s) are swept one at a time with every other input
held at its trim value. For the RIGHT rotor (counter-clockwise seen from above
in helicopter mode) the six loads it applies to the aircraft are plotted in
body axes (x fwd, y right, z down; moments about the CG, solid, and about the
hub, dashed), together with shaft power vs power available and the stall
margin.

Conditions
  A  helicopter-like edgewise flight : V = 30 m/s, i_n = 90 deg   (Sections 4.1-4.3)
  B  intermediate conversion         : V = 45 m/s, i_n = 60 deg   (demonstration case 2)

Outputs: outputs/m2/m2_4_*.png, m2_4_control_derivatives.md

Run:  python scripts/m2/demo_rotor_control_sweep.py
"""
import numpy as np

from _common import CFG, plt, save_figure, write_text, trimmed_state, RIGID_DISK_NOTE
from m2.edgewise_bemt import run_edgewise_bemt

ROTOR, AFP = CFG.ROTOR, CFG.airfoil_provider
N_R, N_PSI = 30, 48
LIM = CFG.ControlLimits()

CONDITIONS = [
    ("A", "helicopter-like edgewise flight", 30.0, 90.0),
    ("B", "intermediate conversion", 45.0, 60.0),
]
SWEEPS = [
    ("collective", "4.1", "Collective pitch theta0 [deg]"),
    ("theta1c", "4.2", "Longitudinal cyclic theta1c [deg]  (pitch ~ cos psi, max at psi = 0 aft)"),
    ("theta1s", "4.3", "Lateral cyclic theta1s [deg]  (pitch ~ sin psi, max at psi = 90 advancing)"),
]
LOAD_NAMES = ["FX", "FY", "FZ", "MX", "MY", "MZ"]


def rotor_at(op, hub, coll, t1c, t1s):
    return run_edgewise_bemt(ROTOR, AFP, op.V_mps, op.omega_rad_s, coll, t1c, t1s,
                             op.alpha_shaft_rad, op.nacelle_deg, op.rho, op.a_sound,
                             r_hub_body=hub, n_r=N_R, n_psi=N_PSI, rotation='ccw')


def sweep(op, hub, which):
    base = dict(collective=op.collective_rad, theta1c=op.theta1c_rad, theta1s=op.theta1s_rad)
    if which == 'collective':
        c0 = np.degrees(op.collective_rad)
        lo, hi = max(LIM.collective_deg[0], c0 - 10.0), min(LIM.collective_deg[1], c0 + 10.0)
    else:
        lo, hi = LIM.theta_1c_deg if which == 'theta1c' else LIM.theta_1s_deg
    xs = np.linspace(lo, hi, 21)
    out = []
    for xd in xs:
        u = dict(base)
        u[which] = np.radians(xd)
        out.append(rotor_at(op, hub, u['collective'], u['theta1c'], u['theta1s']))
    return xs, out


def plot_sweep(tag, cond_name, op, hub, which, sec, xlabel, xs, res, P_avail):
    F = np.array([r.forces_body_N for r in res]) / 1e3
    M_cg = np.array([r.moments_body_Nm for r in res]) / 1e3
    M_hub = np.array([r.moments_hub_body_Nm for r in res]) / 1e3
    P = np.array([r.power_W for r in res]) / 1e3
    sm = np.array([r.stall_margin_deg for r in res])
    sf = np.array([r.stalled_fraction_fwd for r in res]) * 100
    rf = np.array([r.reverse_flow_fraction for r in res]) * 100
    x_trim = np.degrees({'collective': op.collective_rad, 'theta1c': op.theta1c_rad,
                         'theta1s': op.theta1s_rad}[which])

    fig, axs = plt.subplots(2, 4, figsize=(16, 7.6), sharex=True)
    for k in range(3):
        ax = axs[0, k]
        ax.plot(xs, F[:, k], 'b-', lw=2)
        ax.set_ylabel(f"{LOAD_NAMES[k]} [kN]")
        ax = axs[1, k]
        ax.plot(xs, M_cg[:, k], 'r-', lw=2, label="about CG")
        ax.plot(xs, M_hub[:, k], 'r--', lw=1.3, label="about hub")
        ax.set_ylabel(f"{LOAD_NAMES[k + 3]} [kN m]")
        if k == 0:
            ax.legend()
    ax = axs[0, 3]
    ax.plot(xs, P, 'k-', lw=2, label="P required (this rotor)")
    ax.axhline(P_avail / 1e3, color='tab:green', ls='--', label=f"P available/rotor ({P_avail/1e3:.0f} kW)")
    ax.fill_between(xs, P, P_avail / 1e3, where=P <= P_avail / 1e3, color='tab:green', alpha=0.08)
    ax.set_ylabel("Shaft power [kW]")
    ax.legend(loc='best')
    ax = axs[1, 3]
    ax.plot(xs, sm, 'm-', lw=2, label="stall margin (min over loaded disk)")
    ax.axhline(0, color='m', lw=0.8)
    ax.set_ylabel("Stall margin alpha_stall - |alpha| [deg]", color='m')
    ax2 = ax.twinx()
    ax2.plot(xs, sf, 'c-', lw=1.6, label="stalled area [%]")
    ax2.axhline(LIM.max_stall_fraction * 100, color='c', ls=':', lw=1.2, label="stall-fraction limit")
    ax2.set_ylabel("Stalled loaded area [%]", color='c')
    ax2.grid(False)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='best', fontsize=7.5)
    for ax in axs.flat:
        ax.axvline(x_trim, color='0.4', ls=':', lw=1.2)
    for ax in axs[1]:
        ax.set_xlabel(xlabel if ax is axs[1, 0] else xlabel.split('[')[0] + '[deg]')
    axs[0, 0].text(0.02, 0.04, "dotted line: trim value", transform=axs[0, 0].transAxes, fontsize=7.5)
    fig.suptitle(
        f"Section {sec} -- {xlabel.split('[')[0].strip()} sweep, single RIGHT rotor (CCW seen from above), "
        f"condition {tag}: {cond_name}\n{op.label()}, mu = {res[0].mu:.3f};  other inputs at trim: "
        f"{op.controls_label()}\nBody axes: x fwd, y right, z down (FZ < 0 = upward), MX roll right +, "
        f"MY nose-up +, MZ nose-right +; hub at {np.round(hub, 2)} m from CG", fontsize=9.5)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    name = f"m2_{sec.replace('.', 'p')}_{which}_sweep_cond{tag}"
    save_figure(fig, name, sec if tag == 'A' else "4.6",
                f"{xlabel.split('[')[0].strip()} sweep of the right (CCW) rotor at condition {tag} "
                f"({cond_name}): {op.label()}, mu = {res[0].mu:.3f}, other inputs held at trim "
                f"({op.controls_label()}). Rotor loads on the aircraft in body axes; moments about the CG "
                f"(solid) and the hub (dashed). Power vs power available per rotor at "
                f"{op.altitude_m:.0f} m; stall margin over the loaded forward-flow disk. "
                f"Reverse-flow area {rf.min():.1f}-{rf.max():.1f} % of the grid. {RIGID_DISK_NOTE}.")
    return F, M_cg, P, sm


def derivatives(xs, F, M, P, x_trim):
    """Central-difference control derivatives at the trim value, per degree."""
    i = int(np.argmin(np.abs(xs - x_trim)))
    i = min(max(i, 1), len(xs) - 2)
    dx = xs[i + 1] - xs[i - 1]
    d = lambda y: (y[i + 1] - y[i - 1]) / dx
    return [d(F[:, k]) for k in range(3)] + [d(M[:, k]) for k in range(3)] + [d(P)]


def main():
    md = ["# Section 4 -- control derivatives of one rotor (right rotor, CCW from above)", "",
          "Central differences at the trim point. Forces in kN/deg, moments (about the CG) in kN m/deg, "
          "power in kW/deg; body axes x fwd, y right, z down.", ""]
    for tag, cond_name, V, nac in CONDITIONS:
        op = trimmed_state(V, nac)
        ac = CFG.get_default_aircraft()
        hub = ac.hub_from_cg(nac, 'right')
        P_avail = CFG.POWER_MODEL.power_available_W(CFG.atmosphere(op.altitude_m))
        print(f"Condition {tag}: {op.label()} | {op.controls_label()} | trim {op.trim_status}")
        md += [f"## Condition {tag}: {cond_name}", "", f"{op.label()}; trim: {op.controls_label()}", "",
               "| Input | dFX | dFY | dFZ | dMX | dMY | dMZ | dP |", "|---|---|---|---|---|---|---|---|"]
        for which, sec, xlabel in SWEEPS:
            xs, res = sweep(op, hub, which)
            F, M, P, sm = plot_sweep(tag, cond_name, op, hub, which, sec, xlabel, xs, res, P_avail)
            x_trim = np.degrees({'collective': op.collective_rad, 'theta1c': op.theta1c_rad,
                                 'theta1s': op.theta1s_rad}[which])
            dv = derivatives(xs, F, M, P, x_trim)
            md.append(f"| {which} | " + " | ".join(f"{v:+.3f}" for v in dv) + " |")
        r0 = rotor_at(op, hub, op.collective_rad, op.theta1c_rad, op.theta1s_rad)
        md += ["", f"At trim: T = {r0.T_N/1e3:.2f} kN, H = {r0.H_N/1e3:.2f} kN, Y = {r0.Y_N/1e3:.2f} kN, "
                   f"Q = {r0.Q_Nm/1e3:.2f} kN m, P = {r0.power_W/1e3:.0f} kW of {P_avail/1e3:.0f} kW available, "
                   f"mu = {r0.mu:.3f}, adv. tip Mach = {r0.adv_tip_mach:.3f}, reverse-flow area = "
                   f"{100*r0.reverse_flow_fraction:.1f} %, stalled loaded area = {100*r0.stalled_fraction_fwd:.1f} %, "
                   f"stall margin = {r0.stall_margin_deg:+.1f} deg.", ""]
    write_text("m2_4_control_derivatives.md", "\n".join(md) + "\n")


if __name__ == "__main__":
    main()
