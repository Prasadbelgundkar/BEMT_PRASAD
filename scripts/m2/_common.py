"""
Shared helpers for the Milestone 2 report scripts (scripts/m2/*).

* puts src/ on the path,
* `trimmed_state()` -- representative operating point from the aircraft trim,
* `save_figure()` -- saves to outputs/m2/ and records a caption (flight
  condition + assumptions) in outputs/m2/captions.json / FIGURES.md, which is
  the figure index for the report (Section 10.1).
"""
import json
import os
import sys
from dataclasses import dataclass

# The rotor grids are small (25 x 36 to 40 x 72), so multi-threaded BLAS only adds
# overhead; one BLAS thread per process also keeps the parallel corridor solve
# from oversubscribing the CPU. Must be set before numpy is imported.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import numpy as np                                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..', 'src'))
if SRC not in sys.path:
    sys.path.insert(0, SRC)

import matplotlib                                   # noqa: E402
matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
import matplotlib.pyplot as plt                     # noqa: E402

import m2.aircraft_input_m2 as CFG                  # noqa: E402
from m2.trim_6dof import trim_6dof, make_condition, shaft_angle_of_attack  # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, '..', '..', 'outputs', 'm2', f'rotor_{CFG.ROTOR_VARIANT}'))
os.makedirs(OUT_DIR, exist_ok=True)
_CAPTIONS = os.path.join(OUT_DIR, 'captions.json')

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 200, "font.size": 10,
    "axes.grid": True, "grid.linestyle": ":", "grid.alpha": 0.6,
    "axes.titlesize": 11, "legend.fontsize": 8.5,
})

RIGID_DISK_NOTE = (f"rotor variant '{CFG.ROTOR_VARIANT}' (twist {CFG.TWIST_DESC}); rigid disk (no flapping), "
                   "quasi-steady linear airfoil (a0=5.75/rad, stall 14 deg), annular-Glauert inflow + K-factor")


@dataclass
class OperatingPoint:
    V_mps: float
    nacelle_deg: float
    altitude_m: float
    omega_rad_s: float
    rho: float
    a_sound: float
    alpha_body_rad: float
    collective_rad: float
    theta1c_rad: float
    theta1s_rad: float
    delta_e_rad: float
    alpha_shaft_rad: float
    trim_status: str
    gross_mass_kg: float

    @property
    def rpm(self) -> float:
        return self.omega_rad_s * 60.0 / (2.0 * np.pi)

    def label(self) -> str:
        return (f"V={self.V_mps:.0f} m/s, i_n={self.nacelle_deg:.0f} deg, h={self.altitude_m:.0f} m ISA, "
                f"{self.rpm:.0f} RPM, m={self.gross_mass_kg:.0f} kg")

    def controls_label(self) -> str:
        return (f"theta0={np.degrees(self.collective_rad):.2f}, theta1c={np.degrees(self.theta1c_rad):.2f}, "
                f"theta1s={np.degrees(self.theta1s_rad):.2f} deg, alpha_body={np.degrees(self.alpha_body_rad):.2f} deg, "
                f"alpha_shaft={np.degrees(self.alpha_shaft_rad):.2f} deg")


def trimmed_state(V_mps: float, nacelle_deg: float, altitude_m: float = CFG.REFERENCE_ALTITUDE_M,
                  rpm: float = None, gross_mass_kg: float = CFG.GROSS_MASS_KG, x0=None) -> OperatingPoint:
    """6-DOF trim of the twin-rotor aircraft (trim_6dof). Returns the right-rotor
    controls and the aircraft attitude at the trimmed state."""
    rpm = CFG.HOVER_RPM if rpm is None else rpm
    cond = make_condition(max(V_mps, 0.0), nacelle_deg, rpm=rpm, altitude_m=altitude_m)
    ac = CFG.get_default_aircraft(gross_mass_kg)
    tr = trim_6dof(ac, CFG.ROTOR, CFG.airfoil_provider, cond, x0=x0)
    if not tr.status.startswith('ok'):
        raise RuntimeError(f"no trim at V={V_mps}, i_n={nacelle_deg}: {tr.status}")
    return OperatingPoint(
        V_mps=V_mps, nacelle_deg=nacelle_deg, altitude_m=altitude_m, omega_rad_s=cond.omega_rad_s,
        rho=cond.rho, a_sound=cond.a_sound,
        alpha_body_rad=np.radians(tr.alpha_deg), collective_rad=np.radians(tr.collective_deg + tr.dcoll_deg),
        theta1c_rad=np.radians(tr.theta1c_deg), theta1s_rad=np.radians(-tr.dcyc_deg),
        delta_e_rad=np.radians(tr.elevator_deg),
        alpha_shaft_rad=shaft_angle_of_attack(np.radians(tr.alpha_deg), nacelle_deg),
        trim_status=tr.status, gross_mass_kg=gross_mass_kg,
    )


def save_figure(fig, name: str, section: str, caption: str):
    """Save outputs/m2/<name>.png and register its caption."""
    path = os.path.join(OUT_DIR, f"{name}.png")
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    caps = {}
    if os.path.exists(_CAPTIONS):
        with open(_CAPTIONS, encoding='utf-8') as f:
            caps = json.load(f)
    caps[name] = {"section": section, "caption": caption}
    with open(_CAPTIONS, 'w', encoding='utf-8') as f:
        json.dump(caps, f, indent=1)
    _write_index(caps)
    print(f"  saved {os.path.relpath(path)}")
    return path


def _sec_key(item):
    sec = item[1]["section"]
    return [int(p) if p.isdigit() else p for p in sec.replace('.', ' ').split()], item[0]


def _write_index(caps):
    lines = ["# Milestone 2 figure index",
             "", "Generated by scripts/m2/*. Each caption states the flight condition and assumptions.", "",
             "| Section | Figure | Caption |", "|---|---|---|"]
    for name, c in sorted(caps.items(), key=_sec_key):
        lines.append(f"| {c['section']} | `{name}.png` | {c['caption']} |")
    with open(os.path.join(OUT_DIR, 'FIGURES.md'), 'w', encoding='utf-8') as f:
        f.write("\n".join(lines) + "\n")


def write_text(name: str, text: str):
    path = os.path.join(OUT_DIR, name)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f"  saved {os.path.relpath(path)}")
    return path


def polar_axes_setup(ax):
    """Rotor-disk polar plot: psi = 0 at the tail (bottom), nose at the top,
    psi increasing counter-clockwise (CCW rotor seen from above), so the
    advancing side (psi = 90) is on the right."""
    ax.set_theta_zero_location('S')
    ax.set_theta_direction(1)
    ax.set_thetagrids(np.arange(0, 360, 45),
                      ["0\n(aft)", "45", "90\n(adv.)", "135", "180\n(fwd)", "225", "270\n(retr.)", "315"])
    ax.grid(True, alpha=0.35)
