"""
Milestone 2 -- Section 7: conversion corridor.

7.1  Speed - nacelle-angle feasibility map from the 6-DOF trim at every grid
     point (13 nacelle angles x 21 airspeeds), coloured by the primary active
     constraint, with the operational conversion path of Mission Planner v2.
7.2  Active-constraint boundaries: each margin field (power, rotor stall,
     reverse flow, advancing-tip Mach, wing stall, control margin) with its
     zero contour, plus the untrimmable regions (control saturation, excessive
     residual, no physical solution).

Fixed: 2000 m ISA, MTOW, level unaccelerated flight, conversion RPM for all
nacelle angles. Results cached in outputs/m2/rotor_<variant>/corridor_grid.npz;
use --recompute to re-solve.

Run:  python scripts/m2/demo_corridor_map.py [--recompute] [--jobs N]
"""
import argparse
import os
import time

import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

from _common import CFG, plt, save_figure, write_text, OUT_DIR, RIGID_DISK_NOTE
from m2.conversion_corridor import build_corridor_map, CATEGORIES

V_GRID = np.arange(0.0, 100.1, 5.0)
NAC_GRID = np.arange(0.0, 90.1, 7.5)
CACHE = os.path.join(OUT_DIR, "corridor_grid.npz")

CAT_COLORS = {"feasible": "#2ca02c", "power": "#d62728", "wing_stall": "#1f77b4", "rotor_stall": "#ff7f0e",
              "tip_mach": "#9467bd", "reverse_flow": "#e377c2", "control_saturation": "#8c564b",
              "excessive_residual": "#bcbd22", "no_physical_solution": "#7f7f7f"}
CAT_LABELS = {"feasible": "feasible trim", "power": "power limit (P_req > 95 % P_avail)",
              "wing_stall": "wing stall", "rotor_stall": "rotor stall (> 5 % of loaded disk)",
              "tip_mach": "advancing-tip Mach > 0.85", "reverse_flow": "reverse flow > 3 % of disk",
              "control_saturation": "control saturation (no in-bounds trim)",
              "excessive_residual": "excessive trim residual", "no_physical_solution": "no physical trim solution"}


def compute(recompute, jobs):
    meta = dict(rpm=CFG.CONVERSION_RPM, alt=CFG.REFERENCE_ALTITUDE_M, mass=CFG.GROSS_MASS_KG,
                variant=CFG.ROTOR_VARIANT)
    if os.path.exists(CACHE) and not recompute:
        d = dict(np.load(CACHE, allow_pickle=True))
        if (d.get("meta") is not None and d["meta"].item() == meta
                and np.array_equal(d["V"], V_GRID) and np.array_equal(d["nacelle"], NAC_GRID)):
            print(f"  using cached corridor ({os.path.relpath(CACHE)})")
            return d
    print(f"  solving {len(NAC_GRID)} x {len(V_GRID)} trim points ...")
    t0 = time.time()
    g = build_corridor_map(V_GRID, NAC_GRID, CFG.CONVERSION_RPM, CFG.REFERENCE_ALTITUDE_M,
                           CFG.GROSS_MASS_KG, n_jobs=jobs)
    g["meta"] = np.array(meta, dtype=object)
    np.savez(CACHE, **g)
    print(f"  corridor solved in {time.time() - t0:.0f} s")
    return g


def _edges(c):
    d = np.diff(c) / 2.0
    return np.concatenate([[c[0] - d[0]], c[:-1] + d, [c[-1] + d[-1]]])


def plot_map(g):
    V, N = g["V"], g["nacelle"]
    cat = g["category"].astype(int)
    cmap = ListedColormap([CAT_COLORS[c] for c in CATEGORIES])
    norm = BoundaryNorm(np.arange(len(CATEGORIES) + 1) - 0.5, len(CATEGORIES))
    fig, ax = plt.subplots(figsize=(12.5, 7.2))
    ax.pcolormesh(_edges(V), _edges(N), cat, cmap=cmap, norm=norm, edgecolors='white', linewidth=0.4)
    VV, NN = np.meshgrid(V, N)
    conv = np.isin(g["status_code"], [0, 1])
    lines = [("power_margin", CFG.ControlLimits().min_power_margin_frac, CAT_COLORS["power"], "power margin 5 %"),
             ("stall_frac", CFG.ControlLimits().max_stall_fraction, CAT_COLORS["rotor_stall"], "rotor stall 5 %"),
             ("tip_mach", CFG.ControlLimits().max_tip_mach, CAT_COLORS["tip_mach"], "tip Mach 0.85"),
             ("reverse_frac", CFG.ControlLimits().max_reverse_flow_fraction, CAT_COLORS["reverse_flow"],
              "reverse flow 3 %"),
             ("alpha_wing_deg", CFG.get_default_aircraft().wing.alpha_stall_deg, CAT_COLORS["wing_stall"],
              "wing stall")]
    for f, lvl, col, lab in lines:
        z = np.ma.masked_where(~conv | ~np.isfinite(g[f]), g[f])
        if z.count() > 3 and z.min() < lvl < z.max():
            cs = ax.contour(VV, NN, z, levels=[lvl], colors=[col], linewidths=2.4, linestyles='-')
            cs_w = ax.contour(VV, NN, z, levels=[lvl], colors='k', linewidths=0.6, linestyles='--')
            ax.clabel(cs, fmt={lvl: lab}, fontsize=7.5, inline=True)
    path = getattr(CFG, "CONVERSION_PATH", None)
    rpath = getattr(CFG, "RECONVERSION_PATH", None)
    if path:
        pv, pn = zip(*path)
        ax.plot(pv, pn, 'k-o', lw=2.6, ms=6, mfc='yellow', label="Mission Planner v2 conversion path")
    if rpath:
        pv, pn = zip(*rpath)
        ax.plot(pv, pn, 'b--s', lw=2.0, ms=5, mfc='white')
    present = sorted(set(cat.ravel()))
    handles = [Patch(color=CAT_COLORS[CATEGORIES[i]], label=CAT_LABELS[CATEGORIES[i]]) for i in present]
    if path:
        handles.append(plt.Line2D([], [], color='k', marker='o', mfc='yellow', lw=2.6,
                                  label="operational conversion path (MPv2)"))
    if rpath:
        handles.append(plt.Line2D([], [], color='b', ls='--', marker='s', mfc='white', lw=2.0,
                                  label="reconversion (deceleration) path (MPv2)"))
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=8.5)
    ax.set_xlabel("True airspeed V [m/s]")
    ax.set_ylabel("Nacelle angle i_n [deg]  (90 = helicopter, 0 = airplane)")
    ax.set_xlim(_edges(V)[0], _edges(V)[-1]); ax.set_ylim(_edges(N)[0], max(_edges(N)[-1], 94.0))
    ax.set_title(f"Section 7.1 -- Conversion corridor (6-DOF trim at every point), rotor '{CFG.ROTOR_VARIANT}'\n"
                 f"{CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, m = {CFG.GROSS_MASS_KG:.0f} kg (MTOW), level unaccelerated "
                 f"flight, {CFG.CONVERSION_RPM:.0f} RPM, beta = 0; colour = primary active constraint",
                 fontsize=10.5)
    fig.tight_layout()
    nf = int(g["feasible"].sum())
    save_figure(fig, "m2_7p1_conversion_corridor", "7.1",
                f"Speed-nacelle feasibility map: {len(N)} nacelle angles x {len(V)} airspeeds = {cat.size} 6-DOF trim "
                f"solutions ({nf} feasible). Fixed: {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, MTOW "
                f"{CFG.GROSS_MASS_KG:.0f} kg, gamma = 0, no acceleration, {CFG.CONVERSION_RPM:.0f} RPM, CG from the "
                f"mass breakdown at each nacelle angle. Colour: primary active constraint; lines: limit contours of "
                f"each margin (converged region only); black: operational conversion path flown in Mission Planner "
                f"v2. {RIGID_DISK_NOTE}; no rotor-wake/wing interference.")


def plot_constraints(g):
    V, N = g["V"], g["nacelle"]
    VV, NN = np.meshgrid(V, N)
    conv = np.isin(g["status_code"], [0, 1])
    L = CFG.ControlLimits()
    a_st = CFG.get_default_aircraft().wing.alpha_stall_deg
    panels = [("power_margin", 100.0, "Power margin 1 - P_req/P_avail [%]", 100 * L.min_power_margin_frac, 'RdYlGn'),
              ("stall_frac", 100.0, "Rotor stalled loaded area [%]", 100 * L.max_stall_fraction, 'YlOrRd'),
              ("reverse_frac", 100.0, "Reverse-flow area [% of disk]", 100 * L.max_reverse_flow_fraction, 'PuRd'),
              ("tip_mach", 1.0, "Advancing-tip helical Mach [-]", L.max_tip_mach, 'Purples'),
              ("alpha_wing_deg", 1.0, "Wing angle of attack [deg]", a_st, 'coolwarm'),
              ("control_margin", 100.0, "Control margin (1 - max|stick|, collective) [%]", 0.0, 'RdYlGn')]
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    for ax, (f, sc, lab, lim, cm) in zip(axs.flat, panels):
        z = np.ma.masked_where(~conv | ~np.isfinite(g[f]), g[f] * sc)
        pc = ax.pcolormesh(_edges(V), _edges(N), z, cmap=cm, shading='flat')
        fig.colorbar(pc, ax=ax, label=lab)
        if z.count() > 3 and z.min() < lim < z.max():
            cs = ax.contour(VV, NN, z, levels=[lim], colors='k', linewidths=2.0)
            ax.clabel(cs, fmt=f"limit {lim:g}", fontsize=8)
        # untrimmable cells hatched by type
        for code, hatch, name in ((2, '//', 'control saturation'), (3, '..', 'excessive residual'),
                                  (4, 'xx', 'no physical solution')):
            m = (g["status_code"] == code).astype(float)
            if m.any():
                ax.contourf(VV, NN, m, levels=[0.5, 1.5], colors='none', hatches=[hatch])
        ax.set_title(lab, fontsize=9.5)
        ax.set_xlabel("V [m/s]"); ax.set_ylabel("i_n [deg]")
    fig.suptitle(f"Section 7.2 -- Active-constraint fields and boundaries (black line = limit), rotor "
                 f"'{CFG.ROTOR_VARIANT}', {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, MTOW, {CFG.CONVERSION_RPM:.0f} RPM\n"
                 f"blank = no converged trim; hatching: // control saturation, .. excessive residual, "
                 f"xx no physical solution", fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save_figure(fig, "m2_7p2_constraint_boundaries", "7.2",
                "Margin fields behind the corridor map: power margin, rotor stall area, reverse-flow area, "
                "advancing-tip Mach, wing angle of attack and control margin, each with its limit contour, and "
                "the untrimmable regions distinguished by type (control saturation, excessive residual, no "
                f"physical solution). Same fixed conditions as the Section 7.1 map.")


def summary(g):
    V, N = g["V"], g["nacelle"]
    cat = g["category"].astype(int)
    lines = ["# Section 7 -- conversion corridor summary", "",
             f"Rotor '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}), {CFG.REFERENCE_ALTITUDE_M:.0f} m ISA, "
             f"{CFG.GROSS_MASS_KG:.0f} kg, {CFG.CONVERSION_RPM:.0f} RPM, level flight.", "",
             "| Category | points |", "|---|---|"]
    for i, c in enumerate(CATEGORIES):
        lines.append(f"| {CAT_LABELS[c]} | {int((cat == i).sum())} |")
    lines += ["", "Feasible speed range per nacelle angle:", "", "| i_n [deg] | feasible V [m/s] |", "|---|---|"]
    for j, n in enumerate(N):
        vs = V[g["feasible"][j] > 0.5]
        lines.append(f"| {n:.1f} | {', '.join(f'{v:.0f}' for v in vs) if len(vs) else '-'} |")
    write_text("m2_7_corridor_summary.md", "\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recompute", action="store_true")
    ap.add_argument("--jobs", type=int, default=None)
    a = ap.parse_args()
    print(f"Conversion corridor, rotor '{CFG.ROTOR_VARIANT}'")
    g = compute(a.recompute, a.jobs)
    plot_map(g)
    plot_constraints(g)
    summary(g)


if __name__ == "__main__":
    main()
