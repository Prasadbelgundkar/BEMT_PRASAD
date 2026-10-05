"""
Milestone 1 limiting-case recovery (report Section 3.1, required regression
test of Section 10.1).

With the edgewise velocity and cyclic pitch removed, the azimuth-resolved
solver must reproduce the axisymmetric M1 solver for hover, axial climb and
airplane-mode axial flight. Checked on the DESIGN rotor (-45 deg twist) and on
an untwisted constant-chord rotor, to 1 %.
"""
import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt as run_bemt_m1
from environment import isa
from m2.edgewise_bemt import run_edgewise_bemt
import m2.aircraft_input_m2 as CFG

ATM = isa(0)
RHO, A = ATM.density_kg_m3, ATM.speed_of_sound_mps
TOL = 0.01

UNTWISTED = Rotor(radius_m=3.8, root_cutout_m=0.5, num_blades=3,
                  chord_fn=constant_chord(0.35), twist_fn=constant_twist(0.0))

# (label, rotor, V_axial [m/s], omega [rad/s], collective [deg])
CASES = [
    ("hover, design rotor, 10 deg",     CFG.ROTOR, 0.0,  CFG.HOVER_OMEGA, 10.0),
    ("hover, design rotor, 16 deg",     CFG.ROTOR, 0.0,  CFG.HOVER_OMEGA, 16.0),
    ("hover, design rotor, 22 deg",     CFG.ROTOR, 0.0,  CFG.HOVER_OMEGA, 22.0),
    ("climb 10 m/s, design rotor",      CFG.ROTOR, 10.0, CFG.HOVER_OMEGA, 18.0),
    ("airplane 60 m/s, design rotor",   CFG.ROTOR, 60.0, CFG.AIRPLANE_OMEGA, 42.0),
    ("airplane 85 m/s, design rotor",   CFG.ROTOR, 85.0, CFG.AIRPLANE_OMEGA, 48.0),
    ("hover, untwisted rotor, 12 deg",  UNTWISTED, 0.0,  CFG.HOVER_OMEGA, 12.0),
]


def _pair(rotor, V, omega, coll_deg, n=40):
    m1 = run_bemt_m1(rotor, CFG.airfoil_provider, omega, np.radians(coll_deg), RHO, A,
                     v_axial=V, n_stations=n)
    # Axial freestream: alpha_shaft = +90 deg (flow through the disk, same
    # sense as the induced flow); hover: V = 0.
    m2 = run_edgewise_bemt(rotor, CFG.airfoil_provider, V, omega, np.radians(coll_deg),
                           0.0, 0.0, np.pi / 2 if V > 0 else 0.0, 90.0, RHO, A,
                           n_r=n, n_psi=36)
    return m1, m2


@pytest.mark.parametrize("label,rotor,V,omega,coll", CASES, ids=[c[0] for c in CASES])
def test_m1_recovery(label, rotor, V, omega, coll):
    m1, m2 = _pair(rotor, V, omega, coll)
    assert m2.converged, f"{label}: M2 inflow did not converge"
    for name, a, b in (("thrust", m2.T_N, m1.thrust_N), ("torque", m2.Q_Nm, m1.torque_Nm),
                       ("power", m2.power_W, m1.power_W)):
        assert abs(a - b) <= TOL * max(abs(b), 1.0), f"{label}: {name} M1={b:.1f} M2={a:.1f}"
    # axisymmetric: no in-plane forces and no hub moments
    assert abs(m2.H_N) < 1e-6 * max(abs(m2.T_N), 1.0) + 1e-3
    assert abs(m2.Y_N) < 1e-6 * max(abs(m2.T_N), 1.0) + 1e-3
    assert abs(m2.Mx_hub_Nm) < 1e-3 * max(abs(m2.T_N), 1.0)
    assert abs(m2.My_hub_Nm) < 1e-3 * max(abs(m2.T_N), 1.0)


def test_uniform_glauert_does_not_recover_m1_on_twisted_rotor():
    """Documents WHY the inflow model was changed (Section 3.1 / 5.2)."""
    m1 = run_bemt_m1(CFG.ROTOR, CFG.airfoil_provider, CFG.HOVER_OMEGA, np.radians(16.0),
                     RHO, A, v_axial=0.0, n_stations=40)
    m2u = run_edgewise_bemt(CFG.ROTOR, CFG.airfoil_provider, 0.0, CFG.HOVER_OMEGA,
                            np.radians(16.0), 0.0, 0.0, 0.0, 90.0, RHO, A,
                            n_r=40, n_psi=36, inflow='uniform_glauert')
    assert abs(m2u.T_N / m1.thrust_N - 1.0) > 0.05


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
