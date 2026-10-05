"""
Milestone 2 -- Section 6.3: failed-trim cases.

Each case is solved with the 6-DOF trim (multiple seeds) and classified by
trim_6dof.diagnose() into: numerical failure, insufficient control authority,
rotor stall, wing stall, power limitation, tip-Mach limitation or physical
infeasibility. The numerical-failure case is demonstrated by starting from a
single poor seed with a small iteration budget, then re-solving the SAME
condition with the standard seeds.

Outputs: outputs/m2/rotor_<variant>/m2_6p3_failed_trim.png, m2_6p3_failed_trim.md

Run:  python scripts/m2/demo_failed_trim.py      (M2_ROTOR=refined for the refined rotor)
"""
import numpy as np

from _common import CFG, plt, save_figure, write_text, RIGID_DISK_NOTE
from m2.trim_6dof import trim_6dof, make_condition, diagnose, RES_TOL


def payload_at(x_m):
    ac = CFG.get_default_aircraft()
    for it in ac.mass_items:
        if it.category == 'payload':
            it.x_m = x_m
    return ac


def cases():
    ac = CFG.get_default_aircraft()
    A = CFG.AIRPLANE_RPM
    return [
        ("Airplane mode too slow", "V=40 m/s, i_n=0, 2000 m", ac, make_condition(40.0, 0.0, rpm=A), {}),
        ("Helicopter mode too fast", "V=70 m/s, i_n=90, 2000 m", ac, make_condition(70.0, 90.0), {}),
        ("Helicopter mode far beyond limits", "V=85 m/s, i_n=90, 2000 m", ac, make_condition(85.0, 90.0), {}),
        ("Airplane mode overspeed", "V=120 m/s, i_n=0, 2000 m", ac, make_condition(120.0, 0.0, rpm=A), {}),
        ("Hot-and-high hover", "V=0, i_n=90, 3000 m ISA+20", ac,
         make_condition(0.0, 90.0, altitude_m=3000.0, dISA_K=20.0), {}),
        ("Forward CG (payload +5 m)", "V=60 m/s, i_n=0, 2000 m", payload_at(5.0),
         make_condition(60.0, 0.0, rpm=A), {}),
        ("Numerical: poor seed, 6 evaluations", "V=45 m/s, i_n=60, 2000 m", ac, make_condition(45.0, 60.0),
         dict(x0=np.array([np.radians(-15.0), np.radians(25.0), np.radians(50.0), 0.9, 0.5, -0.5]),
              default_seeds=False, max_nfev=6)),
    ]


def main():
    print(f"Failed-trim cases, rotor '{CFG.ROTOR_VARIANT}'")
    rows, results = [], []
    for name, cond_txt, ac, cond, kw in cases():
        tr = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond, **kw)
        diag = diagnose(tr)
        extra = ""
        if 'Numerical' in name:
            tr2 = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond)
            extra = (f"; standard seeds: {tr2.status}, |r| = {tr2.residual_norm:.1e}, "
                     f"feasible = {tr2.feasible} -> numerical, not physical")
            diag = "numerical failure (poor seed / iteration budget)"
        print(f"  {name:38s} {tr.status:22s} |r|={tr.residual_norm:.2e}  -> {diag}{extra}")
        results.append((name, cond_txt, tr, diag + extra))
        W = ac.W_MTOW_N
        rows.append(f"| {name} | {cond_txt} | {tr.status} | {tr.residual_norm:.2e} | "
                    f"{', '.join(tr.at_bound) or '-'} | {', '.join(tr.flags) or '-'} | "
                    f"{tr.F_res_N[0]:.0f} / {tr.F_res_N[1]:.0f} / {tr.F_res_N[2]:.0f} | "
                    f"{tr.M_res_Nm[0]:.0f} / {tr.M_res_Nm[1]:.0f} / {tr.M_res_Nm[2]:.0f} | "
                    f"{tr.theta_deg:.1f} | {tr.collective_deg:.1f} | {tr.elevator_deg:.1f} | "
                    f"{tr.P_req_W/1e3:.0f} / {tr.P_avail_W/1e3:.0f} | {tr.adv_tip_mach:.3f} | "
                    f"{100*tr.stalled_fraction:.0f} | {diag}{extra} |")

    # ---- figure: residual components per case + diagnosis ----
    fig = plt.figure(figsize=(15, 7.6))
    ax = fig.add_axes([0.06, 0.33, 0.90, 0.60])
    comp = ["FX/W", "FY/W", "FZ/W", "MX/(W 1m)", "MY/(W 1m)", "MZ/(W 1m)"]
    n = len(results)
    width = 0.8 / 6
    colors = plt.cm.tab10(np.arange(6))
    for k in range(6):
        vals = []
        for _, _, tr, _ in results:
            W = CFG.GROSS_MASS_KG * CFG.G
            r = np.concatenate([tr.F_res_N, tr.M_res_Nm]) / W
            vals.append(max(abs(r[k]), 1e-14))
        ax.bar(np.arange(n) + (k - 2.5) * width, vals, width, color=colors[k], label=comp[k])
    ax.axhline(RES_TOL, color='k', ls='--', lw=1.3, label=f"convergence tolerance ||r|| < {RES_TOL:g}")
    ax.set_yscale('log')
    ax.set_ylim(1e-14, 2)
    ax.set_xticks(np.arange(n))
    ax.set_xticklabels([f"{nm}\n{c}\n[{tr.status}]" for nm, c, tr, _ in results], fontsize=8)
    ax.set_ylabel("|normalized residual| at best solution")
    ax.legend(ncol=4, loc='upper left', fontsize=8)
    ax.set_title(f"Section 6.3 -- Failed-trim cases (6-DOF trim, rotor '{CFG.ROTOR_VARIANT}', MTOW): residuals "
                 f"and classification", fontsize=11)
    txt = "\n".join(f"{i + 1}. {nm}: {d}" for i, (nm, _, _, d) in enumerate(results))
    fig.text(0.06, 0.02, txt, fontsize=8.3, va='bottom', family='monospace', wrap=True)
    save_figure(fig, "m2_6p3_failed_trim", "6.3",
                "Normalized force/moment residuals at the best solution found for seven off-design cases, with "
                "the solver status and the diagnosed cause: physical infeasibility (wing stall at low airplane-mode "
                "speed), trimmed-but-infeasible (power, tip Mach, rotor stall in fast helicopter mode, hot-and-high "
                "hover power), insufficient control authority (collective at 120 m/s, elevator with a forward CG) "
                f"and a numerical failure that disappears with proper seeding. {RIGID_DISK_NOTE}.")
    write_text("m2_6p3_failed_trim.md",
               "# Section 6.3 -- failed-trim cases\n\n"
               f"Rotor variant '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}), MTOW unless stated. Convergence: "
               f"||r|| < {RES_TOL:g}. Residuals in N and N m (body axes, about the CG).\n\n"
               "| Case | Condition | Status | norm r | Unknowns at bound | Limit flags | FX/FY/FZ [N] | "
               "MX/MY/MZ [N m] | theta [deg] | theta0 [deg] | elevator [deg] | P req/avail [kW] | "
               "adv tip Mach | rotor stall [%] | Diagnosis |\n"
               "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
