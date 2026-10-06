"""
Milestone 2 -- Section 3: edgewise-flight model verification.

  3.1  Recovery of the Milestone 1 limiting cases (hover, axial climb,
       airplane-mode axial flight): M1 vs M2 (annular-Glauert) vs the original
       uniform-Glauert M2 model.
  3.2  Azimuthal loading and periodicity: polar contours of sectional thrust
       and in-plane force [N/m] at a TRIMMED representative edgewise condition,
       plus per-blade loads vs azimuth over two revolutions.
  3.3  Reverse-flow region, advancing-tip Mach and stall boundaries on the
       in-plane velocity U_T seen by the blades.
  3.4  Sensitivity of thrust and torque to the radial and azimuthal grid (4 plots).

Run:  python scripts/m2/verify_edgewise.py
"""
import time

import numpy as np

from _common import (CFG, plt, save_figure, write_text, trimmed_state, polar_axes_setup,
                     RIGID_DISK_NOTE)
from bemt import run_bemt
from environment import isa
from m2.edgewise_bemt import run_edgewise_bemt

ROTOR, AFP = CFG.ROTOR, CFG.airfoil_provider
N_R, N_PSI = 40, 72                 # plotting grid
REP_V, REP_NAC = 40.0, 90.0         # representative helicopter-mode edgewise condition


# ---------------------------------------------------------------------------
def section_3p1():
    print("3.1  Milestone 1 limiting cases ...")
    atm = isa(0.0)
    rho, a = atm.density_kg_m3, atm.speed_of_sound_mps

    def run_all(V, omega, coll_deg):
        m1 = run_bemt(ROTOR, AFP, omega, np.radians(coll_deg), rho, a, v_axial=V, n_stations=N_R)
        kw = dict(rotor=ROTOR, airfoil_provider=AFP, V_inf=V, omega_rad_s=omega,
                  theta0_rad=np.radians(coll_deg), theta1c_rad=0.0, theta1s_rad=0.0,
                  alpha_shaft_rad=np.pi / 2 if V > 0 else 0.0, nacelle_angle_deg=90.0,
                  rho=rho, a_sound=a, n_r=N_R, n_psi=36)
        m2a = run_edgewise_bemt(**kw, inflow='annular_glauert')
        m2u = run_edgewise_bemt(**kw, inflow='uniform_glauert')
        return m1, m2a, m2u

    cases = [
        (f"Hover (V=0), {CFG.HOVER_RPM:.0f} RPM", "Collective theta0 [deg]", np.arange(6.0, 26.1, 2.0),
         lambda c: (0.0, CFG.HOVER_OMEGA, c)),
        (f"Axial climb, {CFG.HOVER_RPM:.0f} RPM, theta0=20 deg", "Climb speed V_c [m/s]", np.arange(0.0, 15.1, 2.5),
         lambda v: (v, CFG.HOVER_OMEGA, 20.0)),
        (f"Airplane mode (axial), {CFG.AIRPLANE_RPM:.0f} RPM, theta0=48 deg", "Airspeed V [m/s]", np.arange(40.0, 100.1, 10.0),
         lambda v: (v, CFG.AIRPLANE_OMEGA, 48.0)),
    ]
    fig, axs = plt.subplots(2, 3, figsize=(14, 7.5))
    rows = ["| Case | x | T M1 [N] | T M2 [N] | err T [%] | P M1 [kW] | P M2 [kW] | err P [%] | err T uniform [%] |",
            "|---|---|---|---|---|---|---|---|---|"]
    max_err = 0.0
    for k, (title, xlabel, xs, f) in enumerate(cases):
        T1, T2, Tu, P1, P2, Pu = [], [], [], [], [], []
        for xv in xs:
            m1, m2a, m2u = run_all(*f(xv))
            T1.append(m1.thrust_N); T2.append(m2a.T_N); Tu.append(m2u.T_N)
            P1.append(m1.power_W); P2.append(m2a.power_W); Pu.append(m2u.power_W)
            eT = 100 * (m2a.T_N - m1.thrust_N) / max(abs(m1.thrust_N), 1.0)
            eP = 100 * (m2a.power_W - m1.power_W) / max(abs(m1.power_W), 1.0)
            eU = 100 * (m2u.T_N - m1.thrust_N) / max(abs(m1.thrust_N), 1.0)
            max_err = max(max_err, abs(eT), abs(eP))
            rows.append(f"| {title} | {xv:.1f} | {m1.thrust_N:.0f} | {m2a.T_N:.0f} | {eT:+.3f} | "
                        f"{m1.power_W/1e3:.1f} | {m2a.power_W/1e3:.1f} | {eP:+.3f} | {eU:+.1f} |")
        T1, T2, Tu, P1, P2, Pu = map(np.array, (T1, T2, Tu, P1, P2, Pu))
        for row, (y1, y2, yu, unit, sc) in enumerate(((T1, T2, Tu, "Thrust per rotor [kN]", 1e3),
                                                      (P1, P2, Pu, "Shaft power per rotor [kW]", 1e3))):
            ax = axs[row, k]
            ax.plot(xs, y1 / sc, 'k-', lw=2, label="M1 axisymmetric BEMT")
            ax.plot(xs, y2 / sc, 'o', color='tab:blue', ms=6, mfc='none', mew=1.6,
                    label="M2 edgewise, annular Glauert (adopted)")
            ax.plot(xs, yu / sc, '--', color='tab:red', label="M2 edgewise, uniform Glauert (original)")
            ax.set_xlabel(xlabel); ax.set_ylabel(unit)
            if row == 0:
                ax.set_title(title)
            err = np.max(np.abs(y2 - y1) / np.maximum(np.abs(y1), 1.0)) * 100
            ax.text(0.03, 0.95, f"max |M2-M1| = {err:.3f} %", transform=ax.transAxes, va='top',
                    fontsize=8.5, bbox=dict(fc='white', ec='0.7'))
        axs[0, k].legend(loc='lower right' if k < 2 else 'lower left')
    fig.suptitle("Section 3.1 -- Recovery of Milestone 1 limiting cases (mu = 0, theta1c = theta1s = 0), "
                 "sea level ISA, design rotor", fontsize=12)
    fig.tight_layout()
    save_figure(fig, "m2_3p1_m1_recovery", "3.1",
                f"M1 vs M2 thrust and power per rotor for hover (collective sweep), axial climb and "
                f"airplane-mode axial flight; sea-level ISA, design rotor (R=3.8 m, B=3, -45 deg twist), "
                f"no edgewise velocity or cyclic. Adopted annular-Glauert model matches M1 to "
                f"{max_err:.3f} %; the original uniform-Glauert inflow (no tip loss) does not.")
    write_text("m2_3p1_m1_recovery_table.md",
               "# Section 3.1 -- M1 limiting-case recovery\n\n"
               f"Design rotor, sea-level ISA, n_r = {N_R}, n_psi = 36. Tolerance: 1 %. "
               f"Max |error| (annular Glauert) = {max_err:.4f} %.\n\n" + "\n".join(rows) + "\n")
    return max_err


# ---------------------------------------------------------------------------
def representative():
    op = trimmed_state(REP_V, REP_NAC)
    res = run_edgewise_bemt(ROTOR, AFP, op.V_mps, op.omega_rad_s, op.collective_rad, op.theta1c_rad,
                            op.theta1s_rad, op.alpha_shaft_rad, op.nacelle_deg, op.rho, op.a_sound,
                            n_r=N_R, n_psi=N_PSI)
    return op, res


def _polar_mesh(res):
    psi = np.append(res.psi_rad, 2 * np.pi)
    PSI, RR = np.meshgrid(psi, res.r_m / ROTOR.radius_m)
    wrap = lambda m: np.concatenate([m, m[:, :1]], axis=1)
    return PSI, RR, wrap


def _decorate_disk(ax, res):
    ax.set_ylim(0, 1.0)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0.25", "0.5", "0.75", "r/R=1"], fontsize=7)
    ax.fill_between(np.linspace(0, 2 * np.pi, 100), 0, ROTOR.root_cutout_m / ROTOR.radius_m,
                    color='0.85', zorder=3)
    # rotation sense: curved arrow just outside the disk, psi 20 -> 70 deg
    th = np.radians(np.linspace(20, 70, 30))
    ax.plot(th, np.full_like(th, 1.07), 'k-', lw=1.3, clip_on=False)
    ax.annotate("", xy=(th[-1], 1.07), xytext=(th[-3], 1.07),
                arrowprops=dict(arrowstyle='-|>', lw=1.3, color='k'), annotation_clip=False)


def section_3p2(op, res):
    print("3.2  Azimuthal loading ...")
    B = ROTOR.num_blades
    PSI, RR, wrap = _polar_mesh(res)
    dT_blade = res.dT_dr_dpsi / B                                  # per blade [N/m]
    dF_blade = res.dQ_dr_dpsi / res.r_m[:, None] / B               # in-plane force per blade [N/m]

    fig = plt.figure(figsize=(13.5, 7.4))
    for k, (field, title, cmap) in enumerate((
            (dT_blade, "Sectional thrust dT/dr per blade [N/m]", 'viridis'),
            (dF_blade, "Sectional in-plane (drag) force per blade [N/m]", 'magma'))):
        ax = fig.add_subplot(1, 2, k + 1, projection='polar')
        polar_axes_setup(ax)
        lim = np.max(np.abs(field))
        if field.min() < 0:
            cs = ax.contourf(PSI, RR, wrap(field), levels=np.linspace(-lim, lim, 41) if k else 40,
                             cmap='RdBu_r' if k else cmap)
        else:
            cs = ax.contourf(PSI, RR, wrap(field), levels=40, cmap=cmap)
        ax.contour(PSI, RR, wrap(res.U_T), levels=[0.0], colors='w', linewidths=1.6, linestyles='--')
        fig.colorbar(cs, ax=ax, pad=0.12, shrink=0.8)
        ax.set_title(title, pad=28)
        _decorate_disk(ax, res)
    fig.suptitle(f"Section 3.2 -- Azimuthal loading at trimmed helicopter-mode edgewise flight, "
                 f"mu = {res.mu:.3f}\n{op.label()}\n{op.controls_label()}\n"
                 f"Right rotor, CCW seen from above (arrow), nose at top; white dashed = reverse-flow boundary",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save_figure(fig, "m2_3p2_sectional_loads_polar", "3.2",
                f"Polar contours of sectional thrust and in-plane force per blade [N/m] over the rotor disk "
                f"(right rotor, CCW from above, psi = 0 aft, advancing side on the right). Trimmed "
                f"{op.label()}, mu = {res.mu:.3f}; {op.controls_label()}. White dashed line: reverse-flow "
                f"boundary U_T = 0. Grey: root cut-out. {RIGID_DISK_NOTE}.")

    # per-blade loads vs azimuth over two revolutions (periodicity)
    from m2.edgewise_bemt import _trapz
    Tb = _trapz(dT_blade, x=res.r_m, axis=0)
    Fb = _trapz(dF_blade, x=res.r_m, axis=0)
    Mb = _trapz(dT_blade * res.r_m[:, None], x=res.r_m, axis=0)
    psi_deg = np.degrees(res.psi_rad)
    psi2 = np.concatenate([psi_deg, psi_deg + 360])
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, y, lab in zip(axs, (Tb, Fb, Mb / 1e3),
                          ("Blade thrust [N]", "Blade in-plane force [N]", "Blade flapwise root moment [kN m]")):
        yy = np.concatenate([y, y])
        ax.plot(psi2, yy, 'b-', lw=1.8)
        c = np.fft.rfft(y) / len(y)
        ax.axhline(y.mean(), color='k', ls=':', lw=1)
        ax.set_ylabel(lab)
        ax.text(0.01, 0.92, f"mean {y.mean():.0f},  1/rev amplitude {2*abs(c[1]):.0f},  "
                f"2/rev {2*abs(c[2]):.0f},  3/rev {2*abs(c[3]):.0f}",
                transform=ax.transAxes, va='top', fontsize=8.5, bbox=dict(fc='white', ec='0.7'))
    for ax in axs:
        for v in (90, 270, 450, 630):
            ax.axvline(v, color='0.6', lw=0.8, ls='--')
    axs[-1].set_xlabel("Blade azimuth psi [deg]  (0 = aft, 90 = advancing, 270 = retreating)")
    axs[-1].set_xticks(np.arange(0, 721, 90))
    fig.suptitle(f"Section 3.2 -- Per-blade loads over two revolutions (steady periodic solution)\n"
                 f"{op.label()}", fontsize=10.5)
    fig.tight_layout()
    save_figure(fig, "m2_3p2_blade_loads_vs_psi", "3.2",
                f"Radially integrated per-blade thrust, in-plane force and flapwise root moment vs azimuth "
                f"over two revolutions; the solution is exactly 2pi-periodic (no transient), the 1/rev content "
                f"shows the advancing/retreating asymmetry. Trimmed {op.label()}.")
    return Tb


def section_3p3(op, res, extra=None):
    print("3.3  Reverse flow / tip Mach / stall ...")
    cases = [(op, res)] + ([extra] if extra else [])
    fig = plt.figure(figsize=(7.2 * len(cases), 7.4))
    lines = []
    for k, (o, r) in enumerate(cases):
        PSI, RR, wrap = _polar_mesh(r)
        ax = fig.add_subplot(1, len(cases), k + 1, projection='polar')
        polar_axes_setup(ax)
        lim = np.max(np.abs(r.U_T))
        cs = ax.contourf(PSI, RR, wrap(r.U_T), levels=np.linspace(-lim, lim, 41), cmap='coolwarm')
        fig.colorbar(cs, ax=ax, pad=0.12, shrink=0.8, label="In-plane velocity U_T [m/s]")
        ax.contour(PSI, RR, wrap(r.U_T), levels=[0.0], colors='k', linewidths=2.2)
        # stalled cells (loaded forward-flow region) hatched
        stall = (r.stall_mask & ~r.reverse_mask).astype(float)
        if stall.any():
            ax.contourf(PSI, RR, wrap(stall), levels=[0.5, 1.5], colors='none', hatches=['xxx'])
            ax.contour(PSI, RR, wrap(stall), levels=[0.5], colors='lime', linewidths=1.6)
        mach_levels = [m for m in (0.5, 0.6, 0.7, 0.8, CFG.ControlLimits().max_tip_mach)
                       if r.mach.min() < m < r.mach.max()]
        if mach_levels:
            cm = ax.contour(PSI, RR, wrap(r.mach), levels=mach_levels, colors='0.15',
                            linewidths=0.9, linestyles='-.')
            ax.clabel(cm, fmt=lambda v: f"M={v:.2f}", fontsize=7)
        _decorate_disk(ax, r)
        mu_R = r.mu
        ax.set_title(f"{o.label()}\nmu={r.mu:.3f}, adv. tip Mach={r.adv_tip_mach:.3f} "
                     f"(limit {CFG.ControlLimits().max_tip_mach}),\nreverse-flow area={100*r.reverse_flow_fraction:.1f} % "
                     f"of grid, stall margin={r.stall_margin_deg:+.1f} deg", fontsize=8.5, pad=30)
        lines.append(f"| {o.label()} | {r.mu:.3f} | {r.adv_tip_mach:.3f} | {r.max_mach:.3f} | "
                     f"{100*r.reverse_flow_fraction:.2f} | {mu_R:.3f} | {100*r.stalled_fraction_fwd:.1f} | "
                     f"{r.stall_margin_deg:+.1f} |")
    fig.suptitle("Section 3.3 -- Reverse-flow region (black line, U_T = 0), stalled cells (green, hatched) "
                 "and Mach iso-lines on the in-plane velocity seen by the blades\n"
                 "Right rotor, CCW seen from above (arrow), nose at top", fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save_figure(fig, "m2_3p3_UT_reverse_flow_mach_stall", "3.3",
                "Signed in-plane velocity U_T = Omega r + V_x sin(psi) over the disk. Black: reverse-flow "
                "boundary U_T = 0 (circle of diameter mu R centred at psi = 270 deg, r = mu R/2). Green hatched: "
                "sections beyond the 14 deg stall angle in the loaded forward-flow region. Dash-dot: local "
                "helical Mach iso-lines; advancing-tip Mach and the 0.85 limit are given in each title. "
                f"Left: trimmed representative condition; right: high-speed check. {RIGID_DISK_NOTE}.")
    write_text("m2_3p3_boundaries_table.md",
               "# Section 3.3 -- reverse flow, tip Mach, stall\n\n"
               "| Condition | mu | adv. tip Mach | max local Mach | reverse-flow area [% grid] | "
               "reverse circle diameter [R] | stalled loaded fwd cells [%] | stall margin [deg] |\n"
               "|---|---|---|---|---|---|---|---|\n" + "\n".join(lines) + "\n")


def section_3p4(op):
    print("3.4  Grid sensitivity ...")
    run = lambda nr, npsi: run_edgewise_bemt(ROTOR, AFP, op.V_mps, op.omega_rad_s, op.collective_rad,
                                             op.theta1c_rad, op.theta1s_rad, op.alpha_shaft_rad,
                                             op.nacelle_deg, op.rho, op.a_sound, n_r=nr, n_psi=npsi)
    nrs = np.array([8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 80, 100, 120])
    npsis = np.array([8, 12, 16, 20, 24, 36, 48, 72, 96, 144, 180])
    NPSI_FIX, NR_FIX = 72, 60
    t0 = time.time()
    r_nr = [run(n, NPSI_FIX) for n in nrs]
    r_np = [run(NR_FIX, n) for n in npsis]
    print(f"     {len(nrs) + len(npsis)} solves in {time.time() - t0:.1f} s")
    T_nr = np.array([r.T_N for r in r_nr]); Q_nr = np.array([r.Q_Nm for r in r_nr])
    T_np = np.array([r.T_N for r in r_np]); Q_np = np.array([r.Q_Nm for r in r_np])

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    specs = [(axs[0, 0], nrs, T_nr / 1e3, "Thrust T [kN]", f"Number of radial stations n_r  (n_psi = {NPSI_FIX})"),
             (axs[0, 1], nrs, Q_nr / 1e3, "Torque Q [kN m]", f"Number of radial stations n_r  (n_psi = {NPSI_FIX})"),
             (axs[1, 0], npsis, T_np / 1e3, "Thrust T [kN]", f"Number of azimuth stations n_psi  (n_r = {NR_FIX})"),
             (axs[1, 1], npsis, Q_np / 1e3, "Torque Q [kN m]", f"Number of azimuth stations n_psi  (n_r = {NR_FIX})")]
    rows = []
    for ax, n, y, ylab, xlab in specs:
        ref = y[-1]
        ax.fill_between([n[0], n[-1]], ref * 0.995, ref * 1.005, color='tab:green', alpha=0.15,
                        label="+/- 0.5 % of finest grid")
        ax.plot(n, y, 'o-', color='tab:blue')
        for chosen, lab, c in ((25 if 'radial' in xlab else 36, "trim/corridor grid", 'tab:orange'),
                               (N_R if 'radial' in xlab else N_PSI, "plotting grid", 'tab:purple')):
            ax.axvline(chosen, color=c, ls='--', lw=1.2, label=lab)
        ax.set_xscale('log')
        ax.set_xticks(n); ax.set_xticklabels([str(v) for v in n], fontsize=7.5)
        ax.minorticks_off()
        ax.set_xlabel(xlab); ax.set_ylabel(ylab)
        ax.legend(loc='lower right')
        rows.append((ylab.split()[0], xlab.split('(')[0].strip(), n, 100 * (y / ref - 1)))
    fig.suptitle(f"Section 3.4 -- Numerical sensitivity of thrust and torque to the disk discretization\n"
                 f"{op.label()};  {op.controls_label()}", fontsize=10.5)
    fig.tight_layout()
    save_figure(fig, "m2_3p4_grid_sensitivity", "3.4",
                f"Thrust and torque of one rotor vs number of radial stations (n_psi fixed at {NPSI_FIX}) and "
                f"azimuth stations (n_r fixed at {NR_FIX}) at the trimmed representative condition "
                f"{op.label()}. Band: +/-0.5 % of the finest grid. Dashed: grids used for trim/corridor "
                f"(25 x 36) and for plots ({N_R} x {N_PSI}).")
    txt = ["# Section 3.4 -- grid sensitivity (deviation from finest grid, %)", ""]
    for q, axis, n, dev in rows:
        txt.append(f"**{q} vs {axis}**: " + ", ".join(f"{a}: {d:+.2f}" for a, d in zip(n, dev)))
        txt.append("")
    write_text("m2_3p4_grid_sensitivity.md", "\n".join(txt))


def main():
    t0 = time.time()
    err = section_3p1()
    op, res = representative()
    print(f"     representative trimmed state: {op.label()} | {op.controls_label()} | {op.trim_status}")
    section_3p2(op, res)
    extra = None
    for V, nac in ((70.0, 90.0), (60.0, 90.0), (60.0, 75.0)):
        try:
            op2 = trimmed_state(V, nac)
            r2 = run_edgewise_bemt(ROTOR, AFP, op2.V_mps, op2.omega_rad_s, op2.collective_rad,
                                   op2.theta1c_rad, 0.0, op2.alpha_shaft_rad, op2.nacelle_deg,
                                   op2.rho, op2.a_sound, n_r=N_R, n_psi=N_PSI)
            extra = (op2, r2)
            break
        except RuntimeError:
            continue
    section_3p3(op, res, extra)
    section_3p4(op)
    write_text("m2_3_summary.md",
               "# Section 3 summary\n\n"
               f"* Max M1-recovery error (annular Glauert): {err:.4f} %\n"
               f"* Representative condition: {op.label()}\n"
               f"* Trim controls: {op.controls_label()}\n"
               f"* Rotor: T = {res.T_N:.0f} N, H = {res.H_N:.0f} N, Y = {res.Y_N:.0f} N, "
               f"Q = {res.Q_Nm:.0f} N m, P = {res.power_W/1e3:.1f} kW, mu = {res.mu:.3f}, "
               f"lambda_G = {res.lambda_G:.4f}, K = {res.K_inflow:.3f}\n"
               f"* Advancing-tip Mach {res.adv_tip_mach:.3f}, reverse-flow {100*res.reverse_flow_fraction:.2f} % "
               f"of grid, stall margin {res.stall_margin_deg:+.1f} deg\n")
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
