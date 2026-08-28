"""
scripts/fm_vs_ct_tiltrotor.py
-----------------------------
Plots Figure of Merit (FM) vs Thrust Coefficient (CT) for the
tiltrotor proprotor defined in src/parameters.py.
  R = 3.8 m, B = 3, root chord = 0.90 m, TR = 0.3888
  Twist: +25 deg root, -45 deg/R washout
  Hover RPM = 500

Sweeps collective pitch, extracts CT and FM at each point,
and plots the characteristic FM vs CT curve with the hover
design point marked.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from bemt import run_bemt
from parameters import (
    get_configured_rotor, AIRFOIL_PROVIDER,
    OMEGA_RPM,
    EMPTY_MASS_KG, PAYLOAD_MASS_KG, FUEL_MASS_KG,
)

ROTOR       = get_configured_rotor()
HOVER_RPM   = OMEGA_RPM
HOVER_OMEGA = 2 * np.pi * HOVER_RPM / 60.0

# --- Atmosphere at sea-level hover ---
atmo = isa(0.0, 0.0)
rho  = atmo.density_kg_m3
a    = atmo.speed_of_sound_mps

R    = ROTOR.radius_m
B    = ROTOR.num_blades
A    = ROTOR.disk_area_m2()
Vtip = ROTOR.tip_speed_mps(HOVER_OMEGA)

print(f"Rotor: R={R} m, B={B}, sigma={ROTOR.solidity():.4f}, Vtip={Vtip:.1f} m/s")

# Collective sweep — wide range to trace the full FM curve
collectives = np.linspace(-5, 25, 80)

CT_list, FM_list = [], []

for col_deg in collectives:
    perf = run_bemt(
        ROTOR, AIRFOIL_PROVIDER, HOVER_OMEGA,
        np.radians(col_deg), rho, a, v_axial=0.0
    )
    T  = perf.thrust_N
    FM = perf.figure_of_merit

    CT = T / (rho * A * Vtip**2)

    if CT > 0 and FM and 0 < FM <= 1.0:
        CT_list.append(CT)
        FM_list.append(FM)

CT_arr = np.array(CT_list)
FM_arr = np.array(FM_list)

# --- Design point: hover thrust required per rotor ---
G             = 9.80665
GROSS_MASS_KG = EMPTY_MASS_KG + PAYLOAD_MASS_KG + FUEL_MASS_KG  # 7200 kg
NUM_ROTORS    = 2
T_hover       = GROSS_MASS_KG * G / NUM_ROTORS
CT_hover      = T_hover / (rho * A * Vtip**2)

idx_design = np.argmin(np.abs(CT_arr - CT_hover))
FM_design  = FM_arr[idx_design]

# --- Plot ---
fig, ax = plt.subplots(figsize=(7, 5))

ax.plot(CT_arr, FM_arr, lw=2.5, color='royalblue', label='Tiltrotor Proprotor (R=3.8 m)')

ax.axvline(CT_hover, color='red', lw=1.5, linestyle='--',
           label=f'Hover Design $C_T$ = {CT_hover:.5f}')
ax.plot(CT_hover, FM_design, 'r*', ms=14, zorder=5,
        label=f'FM at design = {FM_design:.3f}')

ax.set_xlabel(r'Thrust Coefficient  $C_T = T\,/\,(\rho A V_{tip}^2)$', fontsize=11)
ax.set_ylabel('Figure of Merit  FM', fontsize=11)
ax.set_title(
    f'Figure of Merit vs $C_T$\n'
    f'Tiltrotor Proprotor  |  Hover  |  {HOVER_RPM:.0f} RPM  |  ISA SL',
    fontsize=11
)
ax.legend(fontsize=9)
ax.grid(alpha=0.35)
ax.set_ylim(0, 1.0)

specs = (
    f"R = {R:.1f} m,  B = {B},  $\\sigma$ = {ROTOR.solidity():.3f}\n"
    f"Root chord = 0.90 m,  TR = 0.389\n"
    f"Twist: +25° root,  −45°/R washout\n"
    f"GTOW = {GROSS_MASS_KG:.0f} kg,  T/rotor = {T_hover:.0f} N"
)
ax.text(0.03, 0.06, specs, transform=ax.transAxes,
        fontsize=8, verticalalignment='bottom',
        bbox=dict(boxstyle='round,pad=0.4', fc='lightyellow', alpha=0.8))

plt.tight_layout()

outdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
os.makedirs(outdir, exist_ok=True)
path = os.path.join(outdir, "fm_vs_ct_tiltrotor.png")
fig.savefig(path, dpi=150)
plt.close(fig)
print(f"Saved: {path}")
print(f"\nDesign-point summary:")
print(f"  GTOW              = {GROSS_MASS_KG:.0f} kg")
print(f"  T_hover per rotor = {T_hover:.1f} N")
print(f"  CT at hover       = {CT_hover:.6f}")
print(f"  FM at hover CT    = {FM_design:.4f}")
