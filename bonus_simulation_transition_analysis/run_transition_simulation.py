"""
BONUS -- interactive 3D simulation of the tiltrotor transition (Plotly).

THIS IS THE FILE TO RUN. From the repository root:

    python bonus_simulation_transition_analysis/run_transition_simulation.py                 # hover -> airplane
    python bonus_simulation_transition_analysis/run_transition_simulation.py --leg inbound   # airplane -> hover

then open the HTML file it prints (bonus_simulation_transition_analysis/output/) in a web browser.

Replays the trimmed time history logged by scripts/m2/demo_transition_mission.py
(Section 8) and animates the aircraft in 3D: nacelle tilt, pitch / roll attitude
and the CG shift with nacelle angle and fuel. Geometry comes from
src/m2/aircraft_input_m2.py (wing, tails, nacelle pivots, mast, rotor radius,
mass items) and the fuselage stations of scripts/m2/plot_aircraft_schematic.py,
so it matches the Section 5.1 drawing.

Left : aircraft to scale, centred on its CG (rotates about the CG).
Right: flight path, along-track ground distance vs altitude (height exaggerated).
Plot axes: x forward, y left, z up (body axes x fwd / y right / z down rotated
180 deg about x so that "up" is up on screen).

Input : outputs/m2/rotor_<variant>/m2_8_<leg>_log.csv  (committed; regenerate with demo_transition_mission.py)
Output: bonus_simulation_transition_analysis/output/transition_3d_<leg>_rotor_<variant>.html
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts', 'm2'))

from _common import CFG, OUT_DIR as LOG_DIR                                       # noqa: E402
from plot_aircraft_schematic import FUS_NOSE_X, FUS_TAIL_X, FUS_HALF_W, FUS_TOP_Z, FUS_BOT_Z  # noqa: E402

OUTPUT_DIR = os.path.join(HERE, 'output')

T_PLOT = np.diag([1.0, -1.0, -1.0])        # body / NED (x fwd, y right, z down) -> plot (x fwd, y left, z up)
DT_FRAME_S = 1.0                           # replay step; the log is interpolated to it
FRAME_MS = {"Play": 60, "Slow": 200}
NAC_S = (-1.4, -1.25, 1.25, 1.6, 2.1)      # nacelle pod stations along the shaft from the pivot [m]
NAC_R = (0.18, 0.35, 0.35, 0.30, 0.0)
COL = dict(fuselage='#c9cdd3', wing='#a9c9e8', nacelle='#7d838c', disk='#d62728', path='#1f4e9c')
LOG_COLS = ['altitude_m', 'V_mps', 'nacelle_deg', 'theta_deg', 'phi_deg', 'distance_km', 'fuel_kg']


# ---------------- mesh helpers (vertices N x 3, faces 3 x M) ----------------
def _loft(rings):
    nr, nt, _ = rings.shape
    v = rings.reshape(-1, 3)
    f = []
    for r in range(nr - 1):
        for t in range(nt):
            a, b = r * nt + t, r * nt + (t + 1) % nt
            f += [(a, b, b + nt), (a, b + nt, a + nt)]
    for ring in (0, nr - 1):
        c = len(v)
        v = np.vstack([v, rings[ring].mean(0)])
        f += [(c, ring * nt + t, ring * nt + (t + 1) % nt) for t in range(nt)]
    return v, np.array(f).T


def _slab(quad, d):
    """Flat quadrilateral (4 corners in order) thickened by +/- d."""
    v = np.vstack([quad - d, quad + d])
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
    for a in range(4):
        b = (a + 1) % 4
        f += [(a, b, b + 4), (a, b + 4, a + 4)]
    return v, np.array(f).T


def _merge(*meshes):
    vs, fs, n = [], [], 0
    for v, f in meshes:
        vs.append(v)
        fs.append(f + n)
        n += len(v)
    return np.vstack(vs), np.hstack(fs)


# ---------------- aircraft geometry (body axes, from the reference point) ----------------
def airframe_meshes(ac):
    xs = np.linspace(FUS_TAIL_X, FUS_NOSE_X, 24)
    half = FUS_HALF_W * np.clip(np.minimum((FUS_NOSE_X - xs) / 2.5, 1.0), 0, 1) ** 0.5
    half = np.minimum(half, FUS_HALF_W * np.clip((xs - FUS_TAIL_X) / 6.0 + 0.25, 0, 1))
    top = FUS_TOP_Z + np.clip((xs - FUS_NOSE_X + 2.0) / 2.0, 0, 1) * 0.8
    bot = (FUS_BOT_Z - np.clip((FUS_TAIL_X + 5.0 - xs) / 5.0, 0, 1) * 1.6
           - np.clip((xs - FUS_NOSE_X + 2.0) / 2.0, 0, 1) * 0.9)
    ph = np.linspace(0, 2 * np.pi, 14, endpoint=False)
    rings = np.stack([np.column_stack([np.full_like(ph, x), a * np.cos(ph), (t + b) / 2 + (b - t) / 2 * np.sin(ph)])
                      for x, a, t, b in zip(xs, half, top, bot)])
    fuselage = _loft(rings)

    w = ac.wing
    c, b2, s = w.chord_m, w.span_m / 2, np.sin(np.radians(w.i_w_deg))
    le, te = (0.25 * c, -0.25 * c * s), (-0.75 * c, 0.75 * c * s)        # (x, z), nose-up incidence
    wing = _slab(np.array([[le[0], -b2, le[1]], [le[0], b2, le[1]], [te[0], b2, te[1]], [te[0], -b2, te[1]]]),
                 np.array([0, 0, 0.12]))

    t, xt = ac.htail, ac.htail_ac_ref_m
    lt, tt, bt2 = xt[0] + 0.25 * t.chord_m, xt[0] - 0.75 * t.chord_m, t.span_m / 2
    htail = _slab(np.array([[lt, -bt2, xt[2]], [lt, bt2, xt[2]], [tt, bt2, xt[2]], [tt, -bt2, xt[2]]]),
                  np.array([0, 0, 0.07]))

    xv, hv = ac.vtail_ac_ref_m[0], ac.vtail.height_m
    vtail = _slab(np.array([[xv + 0.9, 0, -0.2], [xv - 0.2, 0, -0.2 - hv], [xv - 1.2, 0, -0.2 - hv],
                            [xv - 1.1, 0, -0.2]]), np.array([0, 0.08, 0]))
    return fuselage, wing, htail, vtail


def tilting_meshes(ac, nacelle_deg, R):
    """Nacelle pods, rotor disks and disk rims at nacelle angle i_n (tilt about the wing-tip pivots)."""
    e = ac.shaft_axis_body(nacelle_deg)
    u = np.array([-e[2], 0.0, e[0]])                     # in the disk plane, perpendicular to the shaft in x-z
    yv = np.array([0.0, 1.0, 0.0])
    ring = lambda n: np.outer(np.cos(p := np.linspace(0, 2 * np.pi, n, endpoint=False)), u) + np.outer(np.sin(p), yv)
    pod, disk, rim = [], [], []
    for side in ('right', 'left'):
        pv = np.array(ac.nacelle_pivot_ref_m, float)
        if side == 'left':
            pv[1] = -pv[1]
        pod.append(_loft(np.stack([pv + s * e + r * ring(14) for s, r in zip(NAC_S, NAC_R)])))
        hub = ac.hub_ref_m(nacelle_deg, side)
        edge = hub + R * ring(48)
        n = len(edge)
        disk.append((np.vstack([hub, edge]), np.array([np.zeros(n, int), 1 + np.arange(n), 1 + (np.arange(n) + 1) % n])))
        rim.append(np.vstack([edge, edge[:1], np.full((1, 3), np.nan)]))
    return _merge(*pod), _merge(*disk), np.vstack(rim)


def to_plot(p_body, cg, theta_deg, phi_deg):
    """Body points (from the reference point) -> plot axes, centred on the CG, rotated by the attitude."""
    th, ph = np.radians(theta_deg), np.radians(phi_deg)
    Ry = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(ph), -np.sin(ph)], [0, np.sin(ph), np.cos(ph)]])
    return np.round((p_body - cg) @ (T_PLOT @ Ry @ Rx).T, 2).astype(np.float32)


# ---------------- mission data ----------------
def load_leg(leg):
    path = os.path.join(LOG_DIR, f"m2_8_{leg}_log.csv")
    if not os.path.exists(path):
        raise SystemExit(f"{path} not found -- run scripts/m2/demo_transition_mission.py first")
    log = pd.read_csv(path)
    t = np.arange(log.t_s.iloc[0], log.t_s.iloc[-1] + 1e-9, DT_FRAME_S)
    d = pd.DataFrame({'t_s': t})
    for c in LOG_COLS:
        d[c] = np.interp(t, log.t_s, log[c])
    d['segment'] = log.segment.values[np.searchsorted(log.t_s.values, t, side='right') - 1]
    return d, path


# ---------------- figure ----------------
def _annotations(text):
    base = dict(xref='paper', yref='paper', showarrow=False)
    return [
        dict(base, text=text, x=0.005, y=0.985, xanchor='left', yanchor='top', align='left',
             font=dict(family='Consolas, monospace', size=13, color='#222'),
             bgcolor='rgba(255,255,255,0.85)', bordercolor='#999', borderwidth=1, borderpad=6),
        dict(base, text="Aircraft (to scale, centred on CG)", x=0.31, y=1.0, xanchor='center', yanchor='bottom',
             font=dict(size=12, color='#555')),
        dict(base, text="Flight path (height exaggerated)", x=0.82, y=1.0, xanchor='center', yanchor='bottom',
             font=dict(size=12, color='#555')),
    ]


def _overlay(r):
    pitch = round(r.theta_deg, 1) + 0.0                  # no "-0.0"
    return (f"<b>{r.segment}</b><br>t = {r.t_s:.0f} s<br>airspeed = {r.V_mps:.1f} m/s<br>"
            f"nacelle = {r.nacelle_deg:.1f}°<br>pitch = {pitch:+.1f}°<br>altitude = {r.altitude_m:.0f} m")


def build_figure(ac, d, leg):
    R = CFG.ROTOR.radius_m
    airframe = airframe_meshes(ac)
    xp = ((d.distance_km.values - d.distance_km.values[0]) * 1e3).astype(np.float32)
    zp = d.altitude_m.values.astype(np.float32)

    def dynamic(k):
        r = d.iloc[k]
        cg = ac.cg_ref_m(r.nacelle_deg, fuel_kg=r.fuel_kg)
        pod, disk, rim = tilting_meshes(ac, r.nacelle_deg, R)
        out = []
        for v, _ in (*airframe, pod, disk):
            p = to_plot(v, cg, r.theta_deg, r.phi_deg)
            out.append((p[:, 0], p[:, 1], p[:, 2]))
        p = to_plot(rim, cg, r.theta_deg, r.phi_deg)
        out.append((p[:, 0], p[:, 1], p[:, 2]))
        out.append((xp[:k + 1], np.zeros(k + 1, np.float32), zp[:k + 1]))
        out.append((xp[k:k + 1], [0.0], zp[k:k + 1]))
        return out

    fig = make_subplots(rows=1, cols=2, column_widths=[0.62, 0.38], horizontal_spacing=0.0,
                        specs=[[{'type': 'scene'}, {'type': 'scene'}]])
    s0 = dynamic(0)
    pod0, disk0, _ = tilting_meshes(ac, d.nacelle_deg.iloc[0], R)
    faces = [f for _, f in airframe] + [pod0[1], disk0[1]]
    colors = [COL['fuselage'], COL['wing'], COL['wing'], COL['wing'], COL['nacelle'], COL['disk']]
    light = dict(ambient=0.55, diffuse=0.8, specular=0.15, roughness=0.6)
    for (x, y, z), f, col in zip(s0[:6], faces, colors):
        disk = col == COL['disk']
        fig.add_trace(go.Mesh3d(x=x, y=y, z=z, i=f[0], j=f[1], k=f[2], color=col, flatshading=True,
                                opacity=0.28 if disk else 1.0, lighting=light, hoverinfo='skip'), 1, 1)
    x, y, z = s0[6]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color=COL['disk'], width=3),
                               hoverinfo='skip'), 1, 1)
    fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', hovertext=["CG"], hoverinfo='text',
                               marker=dict(size=5, color='gold', line=dict(color='black', width=1))), 1, 1)
    fig.add_trace(go.Scatter3d(x=xp, y=np.zeros_like(xp), z=zp, mode='lines', hoverinfo='skip',
                               line=dict(color='#c4c4c4', width=3)), 1, 2)
    x, y, z = s0[7]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color=COL['path'], width=6),
                               hoverinfo='skip'), 1, 2)
    x, y, z = s0[8]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='markers', hoverinfo='skip',
                               marker=dict(size=6, color=COL['disk'], line=dict(color='black', width=1))), 1, 2)
    dyn_idx = [0, 1, 2, 3, 4, 5, 6, 9, 10]

    frames = []
    for k in range(len(d)):
        st = dynamic(k)
        data = [go.Mesh3d(x=x, y=y, z=z) for x, y, z in st[:6]] + [go.Scatter3d(x=x, y=y, z=z) for x, y, z in st[6:]]
        frames.append(go.Frame(data=data, traces=dyn_idx, name=str(k),
                               layout=dict(annotations=_annotations(_overlay(d.iloc[k])))))
    fig.frames = frames

    anim = lambda ms: dict(frame=dict(duration=ms, redraw=True), transition=dict(duration=0),
                           fromcurrent=True, mode='immediate')
    axis = lambda title, rng, **kw: dict(title=title, range=rng, backgroundcolor='#f4f6f8', gridcolor='#dde1e6', **kw)
    ext = dict(x=12.0, y=16.5, z=8.0)
    h_lo, h_hi = zp.min() - 20, zp.max() + 20
    fig.update_layout(
        title=dict(text=f"Tiltrotor transition replay -- {leg} leg (rotor '{CFG.ROTOR_VARIANT}')", x=0.5),
        template='plotly_white', height=780, showlegend=False, margin=dict(l=0, r=0, t=50, b=0),
        annotations=_annotations(_overlay(d.iloc[0])),
        scene=dict(xaxis=axis("x fwd [m]", [-ext['x'], ext['x']]), yaxis=axis("y left [m]", [-ext['y'], ext['y']]),
                   zaxis=axis("z up [m]", [-ext['z'], ext['z']]), aspectmode='manual',
                   aspectratio={a: 1.6 * v / ext['y'] for a, v in ext.items()},
                   camera=dict(eye=dict(x=0.9, y=-1.6, z=0.6)), uirevision='keep'),
        scene2=dict(xaxis=axis("distance [m]", [xp.min() - 200, xp.max() + 200]),
                    yaxis=axis("y [m]", [-400, 400]), zaxis=axis("altitude [m]", [h_lo, h_hi]),
                    aspectmode='manual', aspectratio=dict(x=1.5, y=0.25, z=0.6),
                    camera=dict(eye=dict(x=0.15, y=-2.1, z=0.55)), uirevision='keep'),
        updatemenus=[dict(type='buttons', direction='left', x=0.0, y=0.0, xanchor='left', yanchor='top',
                          pad=dict(t=8, r=8), showactive=False,
                          buttons=[dict(label=lab, method='animate', args=[None, anim(ms)])
                                   for lab, ms in FRAME_MS.items()] +
                                  [dict(label="Pause", method='animate',
                                        args=[[None], dict(frame=dict(duration=0, redraw=False),
                                                           transition=dict(duration=0), mode='immediate')])])],
        sliders=[dict(x=0.17, y=0.0, len=0.82, xanchor='left', yanchor='top', pad=dict(t=8),
                      font=dict(color='rgba(0,0,0,0)'), ticklen=0, minorticklen=0, tickcolor='rgba(0,0,0,0)',
                      currentvalue=dict(prefix="t = ", suffix=" s", font=dict(size=12)),
                      steps=[dict(label=f"{t:.0f}", method='animate',
                                  args=[[str(k)], dict(frame=dict(duration=0, redraw=True),
                                                       transition=dict(duration=0), mode='immediate')])
                             for k, t in enumerate(d.t_s)])],
    )
    return fig


def main():
    ap = argparse.ArgumentParser(description="Interactive 3D replay of the M2 transition mission.")
    ap.add_argument('--leg', choices=('outbound', 'inbound'), default='outbound')
    args = ap.parse_args()

    ac = CFG.get_default_aircraft()
    d, src = load_leg(args.leg)
    print(f"{src}: {len(d)} frames, t = {d.t_s.iloc[0]:.0f}..{d.t_s.iloc[-1]:.0f} s, "
          f"V = {d.V_mps.min():.1f}..{d.V_mps.max():.1f} m/s, i_n = {d.nacelle_deg.min():.0f}..{d.nacelle_deg.max():.0f} deg, "
          f"pitch = {d.theta_deg.min():+.1f}..{d.theta_deg.max():+.1f} deg")
    fig = build_figure(ac, d, args.leg)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out = os.path.join(OUTPUT_DIR, f"transition_3d_{args.leg}_rotor_{CFG.ROTOR_VARIANT}.html")
    fig.write_html(out, include_plotlyjs=True, auto_play=False, config=dict(displaylogo=False, responsive=True))
    print(f"saved {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
