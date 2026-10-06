"""
Milestone 2 -- Section 8: Mission Planner v2 transition tests.

Outbound: takeoff hover -> vertical climb -> conversion along the corridor
path (nacelle angle scheduled as a function of airspeed, climbing) ->
airplane-mode acceleration with RPM reduction -> airplane cruise.
Inbound : airplane cruise -> deceleration with RPM increase (descending) ->
reconversion along the same path -> hover -> vertical descent -> landing hover.

Online 6-DOF trim at every time step; the time histories are logged and the
limit checks are applied at every step (violations are recorded and marked).

Outputs (outputs/m2/rotor_<variant>/):
  m2_8p1_mission_definition.md                       Section 8.1 table
  m2_8p2_outbound_time_history.png                   Section 8.2
  m2_8p2_inbound_time_history.png                    Section 8.2 (airplane -> hover)
  m2_8p2_paths_on_corridor.png                       flown (V, i_n) on the corridor map
  m2_8_outbound_log.csv, m2_8_inbound_log.csv        full logs

Run:  python scripts/m2/demo_transition_mission.py
"""
import csv
import os
import time

import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

from _common import CFG, plt, save_figure, write_text, OUT_DIR, RIGID_DISK_NOTE
from m2.mission_v2 import MissionPlannerV2, M2Segment, schedule, ramp_schedule

H0 = CFG.REFERENCE_ALTITUDE_M          # take-off / landing pad altitude [m]
HEADWIND = 5.0                          # headwind aloft [m/s]; calm at the pad (hover / vertical)
V_CRUISE = CFG.AIRPLANE_CRUISE_SPEED_MPS
RPM_H, RPM_A = CFG.CONVERSION_RPM, CFG.AIRPLANE_RPM
PATH = CFG.CONVERSION_PATH              # [(V, i_n), ...] from the corridor map
V_CONV_END = PATH[-1][0]                # airspeed at which the nacelles reach 0 deg
T_CONV = CFG.CONVERSION_TIME_S
RPATH = CFG.RECONVERSION_PATH
T_RECONV = CFG.RECONVERSION_TIME_S


def nacelle_of_speed(V):
    v, n = zip(*PATH)
    return float(np.interp(V, v, n))


def nacelle_of_speed_reconv(V):
    v, n = zip(*RPATH)
    return float(np.interp(V, v, n))


def outbound():
    return [
        M2Segment("Take-off hover", "hover", 20.0, 5.0, schedule(0.0), schedule(0.0), schedule(90.0),
                  schedule(RPM_H), headwind_mps=0.0),
        M2Segment("Vertical climb", "vertical", 60.0, 5.0, schedule(0.0),
                  schedule([(0, 0.0), (0.15, 2.5), (0.85, 2.5), (1, 0.0)]), schedule(90.0), schedule(RPM_H),
                  headwind_mps=0.0),
        M2Segment("Conversion (hover -> airplane)", "conversion", T_CONV, 2.0,
                  ramp_schedule(0.0, V_CONV_END), schedule([(0, 0.0), (0.45, 0.0), (0.65, 1.5), (1, 1.5)]),
                  schedule(90.0), schedule(RPM_H), nacelle_of_speed=nacelle_of_speed,
                  headwind_mps=schedule([(0, 0.0), (0.4, HEADWIND), (1, HEADWIND)])),
        M2Segment("Airplane accel. + RPM reduction", "airplane", 60.0, 5.0,
                  ramp_schedule(V_CONV_END, V_CRUISE), schedule([(0, 1.5), (1, 0.0)]), schedule(0.0),
                  schedule([(0, RPM_H), (0.5, RPM_A), (1, RPM_A)]), headwind_mps=HEADWIND),
        M2Segment("Airplane cruise", "airplane", 60.0, 10.0, schedule(V_CRUISE), schedule(0.0), schedule(0.0),
                  schedule(RPM_A), headwind_mps=HEADWIND),
    ]


def inbound():
    return [
        M2Segment("Airplane cruise", "airplane", 30.0, 10.0, schedule(V_CRUISE), schedule(0.0), schedule(0.0),
                  schedule(RPM_A), headwind_mps=HEADWIND),
        M2Segment("Decel. + RPM increase", "airplane", 60.0, 5.0,
                  ramp_schedule(V_CRUISE, V_CONV_END), schedule([(0, 0.0), (1, -1.5)]), schedule(0.0),
                  schedule([(0, RPM_A), (0.5, RPM_H), (1, RPM_H)]), headwind_mps=HEADWIND),
        M2Segment("Reconversion (airplane -> hover)", "conversion", T_RECONV, 2.0,
                  ramp_schedule(V_CONV_END, 0.0),
                  schedule([(0, -1.5), (0.25, 0.0), (0.65, 0.0), (0.8, -1.0), (0.95, -1.0), (1, 0.0)]),
                  schedule(90.0), schedule(RPM_H), nacelle_of_speed=nacelle_of_speed_reconv,
                  headwind_mps=schedule([(0, HEADWIND), (0.6, HEADWIND), (1, 0.0)])),
        M2Segment("Hover", "hover", 20.0, 5.0, schedule(0.0), schedule(0.0), schedule(90.0), schedule(RPM_H),
                  headwind_mps=0.0),
        M2Segment("Vertical descent", "vertical", 60.0, 5.0, schedule(0.0),
                  schedule([(0, 0.0), (0.15, -1.5), (0.85, -1.5), (1, 0.0)]), schedule(90.0), schedule(RPM_H),
                  headwind_mps=0.0),
        M2Segment("Landing hover", "hover", 15.0, 5.0, schedule(0.0), schedule(0.0), schedule(90.0),
                  schedule(RPM_H), headwind_mps=0.0),
    ]


INBOUND_FUEL_KG = 900.0
INBOUND_START_H = H0 + 250.0          # arbitrary starting point of the inbound leg


def fly(name, segments, h0, fuel):
    mp = MissionPlannerV2(CFG.ROTOR, CFG.airfoil_provider, start_altitude_m=h0, fuel_kg=fuel,
                          reserve_fuel_kg=CFG.M1.RESERVE_FUEL_KG, stop_on_violation=False)
    t0 = time.time()
    print(f"{name}: start h = {h0:.0f} m, fuel = {fuel:.0f} kg, m = {mp.mass_kg():.0f} kg")
    log = mp.run(segments)
    print(f"  {len(log)} trimmed steps in {time.time() - t0:.0f} s, {len(mp.violations)} violations")
    with open(os.path.join(OUT_DIR, f"m2_8_{name}_log.csv"), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        w.writeheader()
        w.writerows(log)
    return log, mp.violations


def plot_history(log, viol, name, title):
    L = {k: np.array([e[k] for e in log]) for k in log[0] if k not in ('segment', 'kind', 'flags', 'trim_status')}
    seg = [e['segment'] for e in log]
    t = L['t_s']
    fig, axs = plt.subplots(5, 2, figsize=(15, 15), sharex=True)
    specs = [
        (axs[0, 0], [('altitude_m', 'altitude', 'k')], "Altitude [m]"),
        (axs[0, 1], [('V_mps', 'true airspeed', 'b'), ('ground_speed_mps', 'ground speed (headwind aloft)', 'c')],
         "Speed [m/s]"),
        (axs[1, 0], [('nacelle_deg', 'nacelle angle', 'tab:purple')], "Nacelle angle [deg]"),
        (axs[1, 1], [('rpm', 'rotor speed', 'tab:brown')], "Rotor speed [RPM]"),
        (axs[2, 0], [('collective_deg', 'collective theta0', 'tab:blue')], "Collective [deg]"),
        (axs[2, 1], [('theta1c_deg', 'long. cyclic theta1c', 'tab:green'), ('elevator_deg', 'elevator', 'tab:olive'),
                     ('theta_deg', 'pitch attitude', 'k')], "Cyclic / elevator / attitude [deg]"),
        (axs[3, 0], [('P_req_kW', 'power required', 'r'), ('P_avail_kW', 'power available', 'g')], "Power, both rotors [kW]"),
        (axs[3, 1], [('fuel_kg', 'fuel remaining', 'tab:orange')], "Fuel [kg]"),
        (axs[4, 0], [('rotor_stall_margin_deg', 'rotor stall margin', 'm')], "Rotor stall margin [deg]"),
        (axs[4, 1], [('wing_alpha_margin_deg', 'wing alpha margin (alpha_stall - |alpha_w|)', 'tab:cyan')],
         "Wing stall margin [deg]"),
    ]
    for ax, series, ylab in specs:
        for key, lab, col in series:
            ax.plot(t, L[key], '-', color=col, lw=1.8, label=lab)
        ax.set_ylabel(ylab)
        if len(series) > 1:
            ax.legend(fontsize=7.5, loc='best')
    ax2 = axs[4, 0].twinx()
    ax2.plot(t, L['rotor_stall_pct'], ':', color='m', lw=1.4, label='stalled loaded area [%]')
    ax2.axhline(100 * CFG.ControlLimits().max_stall_fraction, color='m', lw=0.8, ls='--')
    ax2.set_ylabel("Stalled area [%] (dotted)", color='m')
    ax2.grid(False)
    axs[4, 0].axhline(0, color='m', lw=0.8)
    axs[4, 1].axhline(0, color='tab:cyan', lw=0.8)
    # segment shading and labels
    bounds = [0] + [i for i in range(1, len(seg)) if seg[i] != seg[i - 1]] + [len(seg)]
    for j in range(len(bounds) - 1):
        a, b = t[bounds[j]], t[min(bounds[j + 1], len(t) - 1)]
        for ax in axs.flat:
            if j % 2 == 0:
                ax.axvspan(a, b, color='0.92', zorder=0)
        axs[0, 0].text(0.5 * (a + b), 1.02, seg[bounds[j]], transform=axs[0, 0].get_xaxis_transform(),
                       ha='center', va='bottom', fontsize=7, rotation=12)
        axs[0, 1].text(0.5 * (a + b), 1.02, seg[bounds[j]], transform=axs[0, 1].get_xaxis_transform(),
                       ha='center', va='bottom', fontsize=7, rotation=12)
    if viol:
        tv = [v[1] for v in viol]
        for ax in axs.flat:
            for x in tv:
                ax.axvline(x, color='r', lw=0.6, alpha=0.5)
    for ax in axs[-1]:
        ax.set_xlabel("Mission time [s]")
    nv = len(viol)
    fig.suptitle(f"{title}\nrotor '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}), ISA, start m = {log[0]['mass_kg']:.0f} kg, "
                 f"headwind {HEADWIND:.0f} m/s aloft, online 6-DOF trim every step; "
                 f"{'no limit violations' if nv == 0 else f'{nv} limit violations (red lines)'}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save_figure(fig, name, "8.2",
                f"{title}: time histories of altitude, airspeed and ground speed, nacelle angle, rotor RPM, "
                f"collective, longitudinal cyclic / elevator / pitch attitude, power required vs available, fuel "
                f"remaining and rotor and wing stall margins from Mission Planner v2 with the 6-DOF trim solved "
                f"at every step. Start altitude {log[0]['altitude_m']:.0f} m ISA, start mass "
                f"{log[0]['mass_kg']:.0f} kg, {HEADWIND:.0f} m/s headwind aloft (calm at the pad); {nv} limit violations. {RIGID_DISK_NOTE}.")


def plot_paths(logs):
    cache = os.path.join(OUT_DIR, "corridor_grid.npz")
    if not os.path.exists(cache):
        print("  (no corridor cache -- run demo_corridor_map.py first for the path overlay)")
        return
    from m2.conversion_corridor import CATEGORIES
    from demo_corridor_map import CAT_COLORS, CAT_LABELS, _edges
    g = dict(np.load(cache, allow_pickle=True))
    cat = g["category"].astype(int)
    cmap = ListedColormap([CAT_COLORS[c] for c in CATEGORIES])
    norm = BoundaryNorm(np.arange(len(CATEGORIES) + 1) - 0.5, len(CATEGORIES))
    fig, ax = plt.subplots(figsize=(11, 6.5))
    ax.pcolormesh(_edges(g["V"]), _edges(g["nacelle"]), cat, cmap=cmap, norm=norm, alpha=0.55,
                  edgecolors='white', linewidth=0.3)
    for (label, log), st in zip(logs, ('k-', 'b--')):
        V = [e['V_mps'] for e in log]; n = [e['nacelle_deg'] for e in log]
        ax.plot(V, n, st, lw=2.2, label=label)
        bad = [(e['V_mps'], e['nacelle_deg']) for e in log if not e['feasible']]
        if bad:
            ax.plot(*zip(*bad), 'rx', ms=8, mew=2)
    from matplotlib.patches import Patch
    hs, ls = ax.get_legend_handles_labels()
    hs += [Patch(color=CAT_COLORS[CATEGORIES[i]], alpha=0.55, label=CAT_LABELS[CATEGORIES[i]])
           for i in sorted(set(cat.ravel()))]
    ax.legend(handles=hs, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=8)
    ax.set_xlabel("True airspeed V [m/s]"); ax.set_ylabel("Nacelle angle [deg]")
    ax.set_title(f"Section 8.2 -- Flown conversion/reconversion paths on the corridor map "
                 f"(map: {CFG.REFERENCE_ALTITUDE_M:.0f} m, MTOW, {CFG.CONVERSION_RPM:.0f} RPM, unaccelerated)",
                 fontsize=10)
    fig.tight_layout()
    save_figure(fig, "m2_8p2_paths_on_corridor", "8.2",
                "Airspeed-nacelle trajectories actually flown by Mission Planner v2 (outbound conversion and inbound "
                "reconversion, including acceleration/deceleration and climb/descent) over the steady-flight corridor "
                "map; red crosses would mark infeasible steps.")


def definition_table():
    rows = ["# Section 8.1 -- transition-mission definition", "",
            f"Rotor '{CFG.ROTOR_VARIANT}' ({CFG.TWIST_DESC}). ISA (dISA = 0). Wind: calm at the pad (hover/vertical "
            f"segments), along-track headwind building to {HEADWIND:.0f} m/s aloft during conversion. Speed changes "
            f"use trapezoidal acceleration profiles (15 % ramps). "
            f"Outbound start: {H0:.0f} m, MTOW {CFG.GROSS_MASS_KG:.0f} kg (fuel {CFG.FUEL_CAPACITY_KG:.0f} kg). "
            f"Inbound start (arbitrary point): {INBOUND_START_H:.0f} m, fuel {INBOUND_FUEL_KG:.0f} kg. "
            f"Reserve fuel {CFG.M1.RESERVE_FUEL_KG:.0f} kg. Controls from the online 6-DOF trim at every step. "
            f"Conversion path (V [m/s], i_n [deg]): {PATH}; reconversion path: {RPATH}.", "",
            "| Leg | Segment | Type | Duration [s] | dt [s] | Airspeed schedule [m/s] | Climb rate [m/s] | "
            "Nacelle schedule [deg] | RPM schedule |", "|---|---|---|---|---|---|---|---|---|"]
    for leg, segs in (("Outbound", outbound()), ("Inbound", inbound())):
        for s in segs:
            a, b = s.state(0.0), s.state(1.0)
            nac = ("conversion path n(V)" if s.nacelle_of_speed is nacelle_of_speed else
                   "reconversion path n(V)" if s.nacelle_of_speed is nacelle_of_speed_reconv else
                   f"{a['nacelle']:.0f} -> {b['nacelle']:.0f}")
            rows.append(f"| {leg} | {s.name} | {s.kind} | {s.duration_s:.0f} | {s.dt_s:.0f} | "
                        f"{a['V_h']:.0f} -> {b['V_h']:.0f} | {s.climb(0.5):+.1f} (ramped) | {nac} | "
                        f"{a['rpm']:.0f} -> {b['rpm']:.0f} |")
    write_text("m2_8p1_mission_definition.md", "\n".join(rows) + "\n")


def main():
    definition_table()
    out_log, out_v = fly("outbound", outbound(), H0, CFG.FUEL_CAPACITY_KG)
    in_log, in_v = fly("inbound", inbound(), INBOUND_START_H, INBOUND_FUEL_KG)
    plot_history(out_log, out_v, "m2_8p2_outbound_time_history",
                 "Section 8.2 -- Outbound: take-off, hover-to-airplane-mode conversion, cruise")
    plot_history(in_log, in_v, "m2_8p2_inbound_time_history",
                 "Section 8 -- Inbound: airplane-mode-to-hover reconversion and landing")
    plot_paths([("outbound (conversion)", out_log), ("inbound (reconversion)", in_log)])
    txt = ["# Section 8 -- mission summary", ""]
    for name, log, v in (("Outbound", out_log, out_v), ("Inbound", in_log, in_v)):
        fuel = log[0]['fuel_kg'] - log[-1]['fuel_kg']
        txt.append(f"* {name}: {log[-1]['t_s']:.0f} s, {log[-1]['distance_km']:.1f} km ground distance, fuel used "
                   f"{fuel:.1f} kg, max power {max(e['P_req_kW'] for e in log):.0f} kW (min available "
                   f"{min(e['P_avail_kW'] for e in log):.0f} kW), min rotor stall margin "
                   f"{min(e['rotor_stall_margin_deg'] for e in log):+.1f} deg, max nacelle rate "
                   f"{max(abs(e['nacelle_rate_deg_s']) for e in log):.2f} deg/s, violations: {len(v)}")
        for s, t, r in v[:20]:
            txt.append(f"    - t = {t:.0f} s [{s}]: {r}")
    write_text("m2_8_mission_summary.md", "\n".join(txt) + "\n")


if __name__ == "__main__":
    main()
