"""
engine_selection.py  --  Milestone 2 engine sizing and selection (report Sections 5.2 / 5.5)
=========================================================================================
Power required by the trimmed aircraft (6-DOF trim, edgewise BEMT) at the
sizing conditions, compared with the power available from candidate
turboshaft engines.

Sizing conditions (MTOW 7200 kg):
  R1  hover OGE at 1500 m, ISA+15 (Milestone 1 requirement)      take-off power
  R2  vertical climb 2.5 m/s at 2000 m ISA (mission v2)             take-off power
  R3  airplane-mode cruise 74.3 m/s at 7000 m ISA (M1 cruise)       max-continuous power
  R4  one engine inoperative: minimum-power conversion point and
      85 m/s airplane-mode cruise at 2000 m ISA                     OEI (= take-off) power
  Goal: maximum level speed (target 125 m/s = 450 km/h) at 2000 m and 7000 m.
Power available per engine: P = eta_dt * P_rated * sigma^x (sigma = rho/rho_SL),
with a 5 % margin required on every condition.

Outputs (outputs/m2/rotor_<variant>/): m2_5_engine_power_required.json (cache),
m2_5_engine_selection.md, m2_5p2_engine_selection.png
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import CFG, plt, save_figure, write_text, OUT_DIR    # noqa: E402
from environment import isa                                          # noqa: E402
from m2.trim_6dof import trim_6dof, make_condition                   # noqa: E402

CACHE = os.path.join(OUT_DIR, "m2_5_engine_power_required.json")
MTOW = 7200.0
MARGIN = 0.05
ETA_DT = CFG.M1.DRIVETRAIN_EFFICIENCY      # 0.95 (Milestone 1 value)
LAPSE_EXP = CFG.M1.DENSITY_RATIO_EXPONENT  # P ~ sigma^1.0 (Milestone 1 value)
MCP_FRACTION = CFG.MCP_FRACTION            # max continuous / take-off (T700-701D: 1279 / 1486 kW)
RPM_HELI = CFG.HOVER_RPM


def airplane_rpm(V):
    """Airplane-mode RPM schedule of the configuration (Section 5.3)."""
    return CFG.AIRPLANE_RPM if V <= 100.0 + 1e-9 else CFG.DASH_RPM

# Candidate engines: take-off (max) rating, dry mass, SFC at max power.
# Values from manufacturer data sheets / type certificates where published;
# 'est.' marks values not published, estimated from the engine family.
ENGINES = [
    dict(name="P&WC PT6C-67A", P_kW=1445, mass_kg=225, sfc_g_kWh=290, note="AW609; mass, SFC est."),
    dict(name="GE T700-GE-701D", P_kW=1486, mass_kg=207, sfc_g_kWh=283, note="UH-60M / AH-64E"),
    dict(name="RR/Safran RTM322-01/1", P_kW=1611, mass_kg=255, sfc_g_kWh=255, note="NH90 / EH101"),
    dict(name="Safran Makila 2A", P_kW=1801, mass_kg=279, sfc_g_kWh=270, note="H225; SFC est."),
    dict(name="GE CT7-8A", P_kW=1893, mass_kg=245, sfc_g_kWh=280, note="S-92; mass, SFC est."),
]


def sigma(alt_m, dISA=0.0):
    return isa(alt_m, dISA).density_kg_m3 / isa(0.0).density_kg_m3


def trim_power(V, nacelle, rpm, alt, dISA=0.0, gamma_deg=0.0, x0=None):
    ac = CFG.get_default_aircraft(MTOW)
    cond = make_condition(V, nacelle, rpm=rpm, altitude_m=alt, gamma_deg=gamma_deg, dISA_K=dISA)
    tr = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond, x0=x0)
    ok = tr.status in ("ok", "ok_at_limit")
    return dict(P_kW=tr.P_req_W / 1e3 if ok else None, status=tr.status, flags=tr.flags,
                collective_deg=tr.collective_deg if ok else None), (tr.x if ok else None)


def airplane_sweep(alt, speeds):
    out, x0 = [], None
    for V in speeds:
        rpm = CFG.LONG_RANGE_RPM if abs(V - 74.3) < 1e-6 else airplane_rpm(V)
        r, x = trim_power(V, 0.0, rpm, alt, x0=x0 if abs(V - 74.3) > 1e-6 else None)
        if abs(V - 74.3) > 1e-6:
            x0 = x if x is not None else x0
        out.append(dict(V=float(V), rpm=rpm, **r))
        print(f"    airplane {alt:.0f} m, V = {V:5.1f} m/s: {r['status']:22s} "
              f"P = {r['P_kW'] if r['P_kW'] is None else round(r['P_kW'])} kW")
    return out


def compute_power_required():
    res = {}
    print("  R1 hover 1500 m ISA+15 ...")
    res["hover_1500_isa15"], _ = trim_power(0.0, 90.0, RPM_HELI, 1500.0, dISA=15.0)
    print("  hover 2000 m ISA ...")
    res["hover_2000"], _ = trim_power(0.0, 90.0, RPM_HELI, 2000.0)
    print("  R2 vertical climb 2.5 m/s, 2000 m ...")
    res["climb_2000"], _ = trim_power(2.5, 90.0, RPM_HELI, 2000.0, gamma_deg=90.0)
    speeds = np.arange(55.0, 140.0, 5.0)
    res["airplane_2000"] = airplane_sweep(2000.0, speeds)
    res["airplane_7000"] = airplane_sweep(7000.0, np.concatenate([[74.3], speeds]))
    # conversion-corridor minimum power at 2000 m (cached 6-DOF corridor map)
    grid = np.load(os.path.join(OUT_DIR, "corridor_grid.npz"), allow_pickle=True)
    ok = np.isin(grid["status_code"], (0, 1)) & ~np.isnan(grid["P_req_kW"])
    P = np.where(ok, grid["P_req_kW"], np.inf)
    i, j = np.unravel_index(np.argmin(P), P.shape)
    res["corridor_min_2000"] = dict(P_kW=float(P[i, j]), V=float(grid["V"][j]), nacelle=float(grid["nacelle"][i]))
    with open(CACHE, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    return res


def P_at(sweep, V):
    for p in sweep:
        if abs(p["V"] - V) < 1e-6:
            return p["P_kW"]
    return None


def max_level_speed(sweep, P_avail_kW):
    """Highest trimmed speed with P_req <= (1 - margin) P_avail (linear interpolation)."""
    pts = [(p["V"], p["P_kW"]) for p in sweep if p["P_kW"] is not None]
    best = None
    for (v1, p1), (v2, p2) in zip(pts, pts[1:]):
        lim = (1 - MARGIN) * P_avail_kW
        if p1 <= lim:
            best = v1
            if p2 > lim:
                return v1 + (lim - p1) / (p2 - p1) * (v2 - v1)
            best = v2
    return best


def evaluate(res):
    s1500, s2000, s7000 = sigma(1500.0, 15.0), sigma(2000.0), sigma(7000.0)
    req = dict(
        R1=res["hover_1500_isa15"]["P_kW"],
        R2=res["climb_2000"]["P_kW"],
        R3=P_at(res["airplane_7000"], 74.3),
        R4a=res["corridor_min_2000"]["P_kW"],
        R4b=P_at(res["airplane_2000"], 85.0),
    )
    rows = []
    for e in ENGINES:
        P = e["P_kW"]
        avail = dict(
            R1=2 * ETA_DT * P * s1500 ** LAPSE_EXP,
            R2=2 * ETA_DT * P * s2000 ** LAPSE_EXP,
            R3=2 * ETA_DT * MCP_FRACTION * P * s7000 ** LAPSE_EXP,
            R4a=ETA_DT * P * s2000 ** LAPSE_EXP,
            R4b=ETA_DT * P * s2000 ** LAPSE_EXP,
        )
        margin = {k: 1.0 - req[k] / avail[k] for k in req}
        sw2 = [p for p in res["airplane_2000"] if p["V"] != 74.3]
        sw7 = [p for p in res["airplane_7000"] if p["V"] != 74.3]
        vmax2 = max_level_speed(sw2, 2 * ETA_DT * MCP_FRACTION * P * s2000 ** LAPSE_EXP)
        vmax7 = max_level_speed(sw7, 2 * ETA_DT * MCP_FRACTION * P * s7000 ** LAPSE_EXP)
        vdash7 = max_level_speed(sw7, 2 * ETA_DT * P * s7000 ** LAPSE_EXP)
        meets = all(margin[k] >= MARGIN for k in ("R1", "R2", "R3", "R4a"))
        rows.append(dict(e, margin=margin, avail=avail, vmax2000=vmax2, vmax7000=vmax7, vdash7000=vdash7,
                         meets_required=meets, reaches_target=(vdash7 or 0) >= CFG.M1.MAX_SPEED_TARGET_MPS - 1e-6,
                         meets_oei_cruise=margin["R4b"] >= MARGIN, mass_pair_kg=2 * e["mass_kg"],
                         P_W_kg=P / e["mass_kg"]))
    return req, rows


def select(rows):
    """Among engines meeting R1-R4a: prefer OEI cruise capability (R4b), then the 450 km/h dash at
    7000 m on take-off power, then the lightest engine pair."""
    ok = [r for r in rows if r["meets_required"]]
    if not ok:
        return None
    return sorted(ok, key=lambda r: (not r["meets_oei_cruise"], not r["reaches_target"], r["mass_pair_kg"]))[0]


def report(res, req, rows, chosen):
    pct = lambda m: f"{100 * m:+.0f} %"
    lines = ["# Section 5 -- engine sizing and selection", "",
             f"Rotor '{CFG.ROTOR_VARIANT}', MTOW {MTOW:.0f} kg, 6-DOF trimmed power required (both rotors). "
             f"Power available per engine = {ETA_DT} x P_rated x sigma^{LAPSE_EXP}; cruise checks use "
             f"max-continuous = {MCP_FRACTION} x take-off; required margin {100 * MARGIN:.0f} %.", "",
             "## Power required at the sizing conditions", "",
             "| Condition | P required [kW] | Rating used |", "|---|---|---|",
             f"| R1 hover OGE, 1500 m ISA+15 | {req['R1']:.0f} | take-off, 2 engines |",
             f"| R2 vertical climb 2.5 m/s, 2000 m | {req['R2']:.0f} | take-off, 2 engines |",
             f"| R3 airplane cruise 74.3 m/s, 7000 m, {CFG.LONG_RANGE_RPM:.0f} RPM | {req['R3']:.0f} | max continuous, 2 engines |",
             f"| R4a OEI, corridor minimum power ({res['corridor_min_2000']['V']:.0f} m/s, i_n = "
             f"{res['corridor_min_2000']['nacelle']:.1f} deg), 2000 m | {req['R4a']:.0f} | OEI, 1 engine |",
             f"| R4b OEI, airplane cruise 85 m/s, 2000 m, {CFG.AIRPLANE_RPM:.0f} RPM (desirable) | {req['R4b']:.0f} | OEI, 1 engine |", "",
             "## Candidate engines", "",
             "| Engine | Take-off power [kW] | Dry mass [kg] | SFC [g/kWh] | R1 | R2 | R3 | R4a | R4b | "
             "V_max 2000 m, MCP [m/s] | V_max 7000 m, MCP [m/s] | V_dash 7000 m, take-off [m/s] | Meets R1-R4a | Note |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        m = r["margin"]
        fmt_v = lambda v: "-" if v is None else f"{v:.0f}"
        lines.append(f"| {r['name']} | {r['P_kW']} | {r['mass_kg']} | {r['sfc_g_kWh']} | {pct(m['R1'])} | "
                     f"{pct(m['R2'])} | {pct(m['R3'])} | {pct(m['R4a'])} | {pct(m['R4b'])} | "
                     f"{fmt_v(r['vmax2000'])} | {fmt_v(r['vmax7000'])} | {fmt_v(r['vdash7000'])} | "
                     f"{'yes' if r['meets_required'] else 'no'} | "
                     f"{r['note']} |")
    lines += ["", "Margins are 1 - P_required / P_available (5 % required). Airplane-mode RPM: "
              f"{CFG.AIRPLANE_RPM:.0f} up to 100 m/s, {CFG.DASH_RPM:.0f} above (dash).", ""]
    v_t = CFG.M1.MAX_SPEED_TARGET_MPS
    p_t = P_at(res["airplane_7000"], v_t)
    if p_t is not None and not any(r["reaches_target"] for r in rows):
        v_trim = max(p["V"] for p in res["airplane_7000"] if p["P_kW"] is not None)
        p_eng = p_t / ((1 - MARGIN) * 2 * ETA_DT * sigma(7000.0) ** LAPSE_EXP)
        lines += [f"No candidate reaches the {v_t:.0f} m/s ({3.6 * v_t:.0f} km/h) target. The airplane-mode trim "
                  f"converges up to {v_trim:.0f} m/s, so the top speed is limited by power: {v_t:.0f} m/s at 7000 m "
                  f"needs {p_t:.0f} kW, i.e. a take-off rating of at least {p_eng:.0f} kW per engine.", ""]
    if chosen:
        design = CFG.ROTOR_VARIANT == "refined"
        lines += [(f"**Selected: {chosen['name']}**" if design else
                   f"**Lightest engine meeting the criteria with this blade: {chosen['name']}**")
                  + f" ({chosen['P_kW']} kW take-off, {chosen['mass_kg']} kg, "
                  f"SFC {chosen['sfc_g_kWh']} g/kWh): meets R1-R4a"
                  + (", OEI cruise (R4b)" if chosen["meets_oei_cruise"] else "")
                  + (", and the 450 km/h dash at 7000 m on take-off power" if chosen["reaches_target"] else "")
                  + " -- the lightest engine doing so."]
        if not design:
            lines += ["", f"The installed engine ({CFG.ENGINE_NAME}) is selected on the design rotor ('refined'), "
                          "see outputs/m2/rotor_refined/m2_5_engine_selection.md; this table shows how the "
                          "choice would change with this blade."]
    write_text("m2_5_engine_selection.md", "\n".join(lines) + "\n")


def plot(res, rows, chosen):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    for ax, key, alt, s in ((axes[0], "airplane_2000", 2000.0, sigma(2000.0)),
                            (axes[1], "airplane_7000", 7000.0, sigma(7000.0))):
        pts = [(p["V"], p["P_kW"]) for p in res[key] if p["P_kW"] is not None and p["V"] != 74.3]
        if pts:
            V, P = zip(*pts)
            ax.plot(V, P, "ko-", lw=2, label=f"P required, airplane mode (6-DOF trim, {CFG.AIRPLANE_RPM:.0f} / {CFG.DASH_RPM:.0f} RPM)")
        if alt == 2000.0:
            ax.axhline(res["hover_2000"]["P_kW"], color="tab:red", ls="--", label="P required, hover OGE 2000 m")
            ax.axhline(res["climb_2000"]["P_kW"], color="tab:red", ls=":", label="P required, 2.5 m/s vertical climb")
        else:
            ax.plot([74.3], [P_at(res[key], 74.3)], "r*", ms=14, label="M1 cruise point (74.3 m/s)")
        for r in rows:
            lw = 2.5 if (chosen and r is chosen) else 1.0
            ax.axhline(2 * ETA_DT * MCP_FRACTION * r["P_kW"] * s, lw=lw, alpha=0.85,
                       color=plt.cm.viridis(rows.index(r) / max(1, len(rows) - 1)),
                       label=f"{r['name']} (2 engines, max cont.)")
        ax.axvline(125.0, color="grey", ls="-.", lw=1)
        ax.text(125.5, ax.get_ylim()[0], " 450 km/h target", rotation=90, va="bottom", fontsize=8, color="grey")
        ax.set_xlabel("True airspeed V [m/s]")
        ax.set_ylabel("Power, both rotors [kW]")
        ax.set_title(f"{alt:.0f} m ISA, MTOW")
        ax.grid(alpha=0.3)
    axes[1].legend(fontsize=7, loc="upper left")
    fig.suptitle("Section 5 -- engine sizing: 6-DOF trimmed power required vs power available of the candidate engines "
                 f"(rotor '{CFG.ROTOR_VARIANT}')", fontsize=10)
    fig.tight_layout()
    save_figure(fig, "m2_5p2_engine_selection", "5.2",
                f"Power required by the trimmed aircraft in airplane mode ({CFG.AIRPLANE_RPM:.0f} RPM to 100 m/s, "
                f"{CFG.DASH_RPM:.0f} RPM above) at 2000 m and 7000 m ISA and in "
                "hover / vertical climb at 2000 m, against the max-continuous power available (both engines, "
                f"drivetrain {ETA_DT}, sigma^{LAPSE_EXP} lapse) of five candidate turboshafts; the selected engine is "
                "drawn bold. MTOW 7200 kg. " + "Grey line: 450 km/h top-speed target.")


def main():
    res = compute_power_required() if ("--recompute" in sys.argv or not os.path.exists(CACHE)) \
        else json.load(open(CACHE, encoding="utf-8"))
    req, rows = evaluate(res)
    chosen = select(rows)
    report(res, req, rows, chosen)
    plot(res, rows, chosen)
    print("Requirements [kW]:", {k: round(v) for k, v in req.items()})
    for r in rows:
        print(f"  {r['name']:24s} margins " + " ".join(f"{k}={100 * v:+.0f}%" for k, v in r["margin"].items())
              + f"  Vmax2000={r['vmax2000']} Vmax7000={r['vmax7000']} Vdash7000={r['vdash7000']}")
    print("Selected:", chosen["name"] if chosen else None)


if __name__ == "__main__":
    main()
