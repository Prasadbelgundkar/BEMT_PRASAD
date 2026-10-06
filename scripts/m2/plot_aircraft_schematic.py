"""
Milestone 2 -- Section 5.1: labelled, dimensioned aircraft schematic.

Drawn entirely from src/m2/aircraft_input_m2.py (wing, tails, nacelle pivot,
mast length, rotor radius, mass items / CG), so it always matches the
configuration used by the trim and mission analyses.

Output: outputs/m2/rotor_<variant>/m2_5p1_aircraft_schematic.png
"""
import numpy as np
from matplotlib.patches import Circle, Polygon, FancyBboxPatch

from _common import CFG, plt, save_figure

FUS_NOSE_X, FUS_TAIL_X = 6.0, -7.4      # fuselage stations from the reference point [m]
FUS_HALF_W, FUS_TOP_Z, FUS_BOT_Z = 1.0, -0.15, 2.05


def dim(ax, p, q, text, off=(0, 0), color='0.25', fs=8):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle='<->', color=color, lw=0.9))
    ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha='center', va='center',
            fontsize=fs, color=color, bbox=dict(fc='white', ec='none', pad=0.5))


def top_view(ax, ac):
    w, t, R = ac.wing, ac.htail, CFG.ROTOR.radius_m
    Y = lambda y: -y                       # seen from above, nose to the right: right wing at the bottom
    # fuselage
    xs = np.linspace(FUS_TAIL_X, FUS_NOSE_X, 80)
    half = FUS_HALF_W * np.clip(np.minimum((FUS_NOSE_X - xs) / 2.5, 1.0), 0, 1) ** 0.5
    half = np.minimum(half, FUS_HALF_W * np.clip((xs - FUS_TAIL_X) / 6.0 + 0.25, 0, 1))
    ax.fill(np.r_[xs, xs[::-1]], np.r_[half, -half[::-1]], color='0.8', ec='k', lw=1, zorder=2)
    # wing (straight, quarter chord at x = 0)
    c, b2 = w.chord_m, w.span_m / 2
    le, te = 0.25 * c, -0.75 * c
    ax.add_patch(Polygon([[le, -b2], [le, b2], [te, b2], [te, -b2]], closed=True, fc='#cfe3f5', ec='k', zorder=3))
    for s in (1, -1):                       # flaperons
        y1, y2 = w.aileron_eta[0] * b2, w.aileron_eta[1] * b2
        ax.add_patch(Polygon([[te + 0.25 * c, s * y1], [te + 0.25 * c, s * y2], [te, s * y2], [te, s * y1]],
                             closed=True, fc='#7fb3e0', ec='k', hatch='///', zorder=4))
    ax.text(te - 0.15, Y(0.70 * b2), "flaperon\n(aileron)", fontsize=7, ha='right', va='center')
    # horizontal tail
    xt, ct, bt2 = ac.htail_ac_ref_m[0], t.chord_m, t.span_m / 2
    lt, tt = xt + 0.25 * ct, xt - 0.75 * ct
    ax.add_patch(Polygon([[lt, -bt2], [lt, bt2], [tt, bt2], [tt, -bt2]], closed=True, fc='#cfe3f5', ec='k', zorder=3))
    ax.add_patch(Polygon([[tt + t.elevator_chord_frac * ct, -bt2], [tt + t.elevator_chord_frac * ct, bt2],
                          [tt, bt2], [tt, -bt2]], closed=True, fc='#7fb3e0', ec='k', hatch='///', zorder=4))
    ax.text(tt - 0.15, Y(-bt2 + 0.4), "elevator", fontsize=7, ha='right')
    # vertical tail (edge-on)
    cv = 1.6
    ax.plot([ac.vtail_ac_ref_m[0] + 0.25 * cv, ac.vtail_ac_ref_m[0] - 0.75 * cv], [0, 0], 'k-', lw=3.5, zorder=5)
    # nacelles + rotors
    for side, s in (('right', 1), ('left', -1)):
        pv = ac.nacelle_pivot_ref_m
        ax.add_patch(FancyBboxPatch((pv[0] - 1.6, Y(s * pv[1]) - 0.35), 3.0, 0.7, boxstyle="round,pad=0.05",
                                    fc='0.6', ec='k', zorder=5))
        hub_h = ac.hub_ref_m(90.0, side)
        ax.add_patch(Circle((hub_h[0], Y(hub_h[1])), R, fill=False, ls='--', ec='tab:red', lw=1.3, zorder=6))
        hub_a = ac.hub_ref_m(0.0, side)
        ax.plot([hub_a[0], hub_a[0]], [Y(hub_a[1]) - R, Y(hub_a[1]) + R], '-', color='tab:blue', lw=2.2, zorder=6)
        rot = "CCW" if side == 'right' else "CW"
        yl = Y(hub_h[1]) + np.sign(Y(hub_h[1])) * (R + 0.45)
        ax.text(hub_h[0] - 2.5, yl, f"{side} rotor ({rot} from above)", ha='center', va='center',
                fontsize=7.5, color='tab:red')
    # CG and reference
    for n, mk, col in ((90.0, 'o', 'tab:red'), (0.0, 's', 'tab:blue')):
        cg = ac.cg_ref_m(n)
        ax.plot(cg[0], 0, mk, mfc='yellow', mec=col, ms=9, mew=2, zorder=8)
    ax.plot(0, 0, '+', color='k', ms=12, mew=2, zorder=8)
    # dimensions
    dim(ax, (2.2, -b2), (2.2, b2), f"wing span {w.span_m:.2f} m", off=(0.9, 2.0))
    h = ac.hub_ref_m(90.0, 'right')
    dim(ax, (h[0] - R, Y(h[1]) - R - 1.2), (h[0] + R, Y(h[1]) - R - 1.2), f"rotor dia. {2*R:.1f} m")
    dim(ax, (FUS_TAIL_X, 1.6), (FUS_NOSE_X, 1.6), f"fuselage {FUS_NOSE_X - FUS_TAIL_X:.1f} m", off=(2.5, 0))
    cg0 = ac.cg_ref_m(90.0)[0]
    dim(ax, (cg0, -3.0), (xt, -3.0), f"tail arm {cg0 - xt:.2f} m")
    dim(ax, (-3.2, Y(-pv[1])), (-3.2, 0), f"{pv[1]:.1f} m", off=(-0.6, 0))
    ax.text(FUS_NOSE_X + 0.3, 0, "nose ->", va='center', fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlim(-9.5, 9.5); ax.set_ylim(-14.5, 14.5)
    ax.set_xlabel("x from reference point [m] (+ forward)"); ax.set_ylabel("lateral position [m] (right wing down)")
    ax.set_title("Top view (red dashed: rotor disks in helicopter mode, blue: airplane mode)", fontsize=9.5)


def side(ax, ac):
    w, R = ac.wing, CFG.ROTOR.radius_m
    Z = lambda z: -z                         # plot up = -z_body
    xs = np.linspace(FUS_TAIL_X, FUS_NOSE_X, 80)
    top = np.full_like(xs, FUS_TOP_Z)
    bot = FUS_BOT_Z - np.clip((FUS_TAIL_X + 5.0 - xs) / 5.0, 0, 1) * 1.6 - np.clip((xs - FUS_NOSE_X + 2.0) / 2.0, 0, 1) * 0.9
    top = top + np.clip((xs - FUS_NOSE_X + 2.0) / 2.0, 0, 1) * 0.8
    ax.fill(np.r_[xs, xs[::-1]], np.r_[Z(top), Z(bot[::-1])], color='0.85', ec='k', lw=1, zorder=2)
    c = w.chord_m
    ax.plot([0.25 * c, -0.75 * c], [Z(0.0), Z(0.0 + c * np.sin(np.radians(w.i_w_deg)))], 'k-', lw=4, zorder=4)
    ax.text(0.3, Z(-0.25), f"wing, i_w = {w.i_w_deg:.0f} deg", fontsize=7.5)
    xt = ac.htail_ac_ref_m
    ax.plot([xt[0] + 0.3, xt[0] - 1.0], [Z(xt[2]), Z(xt[2])], 'k-', lw=3.5, zorder=4)
    xv = ac.vtail_ac_ref_m
    hv = ac.vtail.height_m
    ax.add_patch(Polygon([[xv[0] + 0.9, Z(-0.2)], [xv[0] - 0.2, Z(-0.2 - hv)], [xv[0] - 1.2, Z(-0.2 - hv)],
                          [xv[0] - 1.1, Z(-0.2)]], closed=True, fc='#cfe3f5', ec='k', zorder=3))
    ax.text(xv[0] - 0.4, Z(-0.2 - hv - 0.3), "V-tail + rudder", ha='center', fontsize=7.5)
    ax.text(xt[0] - 0.4, Z(xt[2]) - 0.5, "H-tail + elevator", ha='center', fontsize=7.5)
    pv = ac.nacelle_pivot_ref_m
    ax.plot(pv[0], Z(pv[2]), 'ko', ms=6, zorder=7)
    ax.text(pv[0] + 0.25, Z(pv[2]) + 0.15, "conversion pivot", fontsize=7)
    for n, col, ls in ((90.0, 'tab:red', '--'), (45.0, 'tab:orange', ':'), (0.0, 'tab:blue', '-')):
        hub = ac.hub_ref_m(n)
        e = ac.shaft_axis_body(n)
        ax.plot([pv[0], hub[0]], [Z(pv[2]), Z(hub[2])], color=col, lw=4, alpha=0.7, zorder=5)
        d = np.array([e[2], 0, -e[0]])          # disk plane direction (perpendicular to shaft in x-z)
        p1, p2 = hub + R * d, hub - R * d
        ax.plot([p1[0], p2[0]], [Z(p1[2]), Z(p2[2])], color=col, ls=ls, lw=2, zorder=5,
                label=f"rotor disk, i_n = {n:.0f} deg")
    a = np.radians(np.linspace(0, 90, 30))
    ax.plot(pv[0] + 0.9 * np.cos(a), Z(pv[2]) + 0.9 * np.sin(a), 'k-', lw=0.8)
    ax.text(pv[0] + 0.75, Z(pv[2]) + 0.75, "i_n", fontsize=8)
    for n, mk, col in ((90.0, 'o', 'tab:red'), (0.0, 's', 'tab:blue')):
        cg = ac.cg_ref_m(n)
        ax.plot(cg[0], Z(cg[2]), mk, mfc='yellow', mec=col, ms=9, mew=2, zorder=8,
                label=f"CG, i_n = {n:.0f} deg ({cg[0]:+.2f}, {cg[2]:+.2f}) m")
    ax.plot(0, 0, '+', color='k', ms=12, mew=2, zorder=8, label="reference point (wing root c/4)")
    for it in ac.mass_items:
        if not it.tilts_with_nacelle:
            ax.plot(it.x_m, Z(it.z_m), '.', color='0.35', ms=5, zorder=6)
    ax.plot([], [], '.', color='0.35', label="mass-item centroids")
    h = ac.hub_ref_m(90.0)
    dim(ax, (h[0] + 4.3, Z(pv[2])), (h[0] + 4.3, Z(h[2])), f"mast {ac.mast_length_m:.1f} m", off=(0.9, 0))
    dim(ax, (-8.6, Z(FUS_BOT_Z)), (-8.6, Z(h[2])), f"{h[2] - FUS_BOT_Z:+.2f} m".replace('+', '').replace('-', ''),
        off=(-0.6, 0))
    ax.set_aspect('equal')
    ax.set_xlim(-9.5, 9.5); ax.set_ylim(-4.6, 5.0)
    ax.set_xlabel("x from reference point [m] (+ forward)"); ax.set_ylabel("height [m] (+ up)")
    ax.legend(loc='lower right', fontsize=7, ncol=2)
    ax.set_title("Side view (nacelle tilt about the pivot; rotor, gearbox and engine tilt together)", fontsize=9.5)


def main():
    ac = CFG.get_default_aircraft()
    fig = plt.figure(figsize=(15, 13))
    ax1 = fig.add_axes([0.05, 0.36, 0.9, 0.58])
    ax2 = fig.add_axes([0.05, 0.03, 0.9, 0.30])
    top_view(ax1, ac)
    side(ax2, ac)
    for ax in (ax1, ax2):
        ax.grid(True, alpha=0.3)
    fig.suptitle(f"Section 5.1 -- Updated tiltrotor configuration (approximately to scale), MTOW "
                 f"{CFG.GROSS_MASS_KG:.0f} kg, rotor '{CFG.ROTOR_VARIANT}'; '+' = reference point, "
                 f"circle/square = CG in helicopter/airplane mode", fontsize=11)
    save_figure(fig, "m2_5p1_aircraft_schematic", "5.1",
                "Top and side views drawn from the configuration file: proprotors (R = "
                f"{CFG.ROTOR.radius_m} m) at the wing-tip conversion pivots, disks shown in helicopter and airplane "
                "mode, tilting nacelles, fuselage, wing with flaperons, horizontal tail with elevator, vertical tail "
                "with rudder, CG in both modes (CG moves forward as the rotors tilt) and the mass-item centroids. "
                "Dimensions in metres; positions from the reference point (wing root quarter chord).")


if __name__ == "__main__":
    main()
