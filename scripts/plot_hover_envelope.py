"""
Generates a single comprehensive hover operating envelope plot showing:
  - Stall-limited max hover weight vs altitude
  - Power-limited max hover weight vs altitude
  - Safe operating region (green)
  - Stall-limited region (orange)
  - Power-limited region (red)
  - Hover ceiling (where MTOW intersects limit curves)
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from environment import isa
from bemt import run_bemt
import parameters as p

_OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'outputs'))

# ── Constants ────────────────────────────────────────────────────────────────
RHO_SL            = 1.225
LAPSE_EXP         = 0.9
ETA_DT            = 0.97
HOVER_RPM         = 550.0
HOVER_OMEGA       = HOVER_RPM * 2.0 * np.pi / 60.0
NUM_ROTORS        = p.NUM_ENGINES
G                 = 9.80665
MTOW_KG           = p.EMPTY_MASS_KG + p.PAYLOAD_MASS_KG + p.FUEL_MASS_KG
HOVER_STALL_LIMIT = 10.0          # % stall limit for hover
MIN_PWR_MARGIN    = 5.0           # % power margin required
COLLS_DEG         = np.linspace(10.0, 30.0, 60)
COLLS_RAD         = np.radians(COLLS_DEG)

def _P_avail_kW(rho):
    return p.ENGINE_POWER_W * (rho / RHO_SL) ** LAPSE_EXP * ETA_DT / 1e3

def _sweep(h):
    rotor = p.get_configured_rotor()
    atmo  = isa(h, p.DISA_K)
    Pa    = _P_avail_kW(atmo.density_kg_m3)
    T, P, S = [], [], []
    for c in COLLS_RAD:
        perf = run_bemt(rotor, p.AIRFOIL_PROVIDER, HOVER_OMEGA, c,
                        atmo.density_kg_m3, atmo.speed_of_sound_mps, v_axial=0.0)
        T.append(perf.thrust_N / 1e3)
        P.append(perf.power_W  / 1e3)
        S.append(perf.stalled_fraction * 100.0)
    return np.array(T), np.array(P), np.array(S), Pa

# ── Sweep altitudes ──────────────────────────────────────────────────────────
altitudes = np.linspace(0.0, 6000.0, 25)
mass_pwr, mass_stl = [], []

print("Computing hover ceiling envelope (25 altitudes × 60 collectives) ...")
for h in altitudes:
    T, P, S, Pa = _sweep(h)
    Plim   = Pa * (1.0 - MIN_PWR_MARGIN / 100.0)
    ok_p   = P <= Plim
    ok_s   = S <= HOVER_STALL_LIMIT
    Tp     = float(T[ok_p].max()) if ok_p.any() else float('nan')
    Ts     = float(T[ok_s].max()) if ok_s.any() else float('nan')
    mp     = Tp * 1e3 * NUM_ROTORS / G if not np.isnan(Tp) else float('nan')
    ms     = Ts * 1e3 * NUM_ROTORS / G if not np.isnan(Ts) else float('nan')
    mass_pwr.append(mp)
    mass_stl.append(ms)
    print(f"  h={h/1000:.1f} km  | pwr-lim: {mp/1000:.2f} t  | stall-lim: {ms/1000:.2f} t")

alt_km   = altitudes / 1000.0
mp_t     = np.array(mass_pwr) / 1000.0   # tonnes
ms_t     = np.array(mass_stl) / 1000.0
mtow_t   = MTOW_KG / 1000.0
limit_t  = np.fmin(mp_t, ms_t)           # actual ceiling = min of both limits

# Find hover ceiling altitudes (where each curve crosses MTOW)
def _ceiling_km(alt_km, mass_t, mtow_t):
    """Linearly interpolate altitude where mass_t crosses mtow_t from above."""
    for i in range(len(mass_t) - 1):
        if not (np.isnan(mass_t[i]) or np.isnan(mass_t[i+1])):
            if mass_t[i] >= mtow_t >= mass_t[i+1]:
                frac = (mtow_t - mass_t[i]) / (mass_t[i+1] - mass_t[i])
                return alt_km[i] + frac * (alt_km[i+1] - alt_km[i])
    return float('nan')

ceil_pwr_km  = _ceiling_km(alt_km, mp_t, mtow_t)
ceil_stl_km  = _ceiling_km(alt_km, ms_t, mtow_t)
ceil_act_km  = min(x for x in [ceil_pwr_km, ceil_stl_km] if not np.isnan(x))

print(f"\nHover ceiling (power-limited): {ceil_pwr_km:.2f} km")
print(f"Hover ceiling (stall-limited): {ceil_stl_km:.2f} km")
print(f"Actual hover ceiling (most limiting): {ceil_act_km:.2f} km")

# ── Plot ─────────────────────────────────────────────────────────────────────
plt.style.use('ggplot')
fig, ax = plt.subplots(figsize=(10, 7))

rotor = p.get_configured_rotor()
sigma = rotor.solidity()
Vtip  = HOVER_OMEGA * p.RADIUS_M
info  = (f"R={p.RADIUS_M} m, B={p.NUM_BLADES}, sigma={sigma:.4f}, "
         f"{HOVER_RPM:.0f} RPM  (Vtip={Vtip:.1f} m/s)")

# ── Region shading (bottom-to-top layering) ──────────────────────────────────
# 1. Safe operating region: below BOTH limit curves AND above 0
ax.fill_betweenx(alt_km,
                 0, np.fmin(mp_t, ms_t),
                 color='green', alpha=0.12, zorder=0, label='Safe operating region')

# 2. Stall-limited region: between stall curve and power curve (where stall binds more)
stall_more = ms_t < mp_t   # stall is the tighter constraint at these altitudes
if stall_more.any():
    ax.fill_betweenx(alt_km,
                     np.where(stall_more, ms_t, np.nan),
                     np.where(stall_more, mp_t, np.nan),
                     color='orange', alpha=0.22, zorder=1, label='Stall-limited region')

# 3. Power-limited region: between power curve and stall curve (where power binds more)
pwr_more = mp_t < ms_t
if pwr_more.any():
    ax.fill_betweenx(alt_km,
                     np.where(pwr_more, mp_t, np.nan),
                     np.where(pwr_more, ms_t, np.nan),
                     color='red', alpha=0.18, zorder=1, label='Power-limited region')

# 4. Cannot hover at MTOW (above actual ceiling)
y_max = 6.5
ax.fill_betweenx([ceil_act_km, y_max],
                 0, max(mp_t[~np.isnan(mp_t)].max(), ms_t[~np.isnan(ms_t)].max()) * 1.05,
                 color='grey', alpha=0.15, zorder=0, label='Cannot hover at MTOW')

# ── Limit curves ─────────────────────────────────────────────────────────────
ax.plot(mp_t, alt_km, 'r-o', ms=5, lw=2.2, zorder=3,
        label=f'Power-limited max weight  (margin >= {MIN_PWR_MARGIN:.0f}%)')
ax.plot(ms_t, alt_km, 'b--s', ms=5, lw=2.2, zorder=3,
        label=f'Stall-limited max weight  (<{HOVER_STALL_LIMIT:.0f}% stall)')

# ── Design MTOW line ─────────────────────────────────────────────────────────
ax.axvline(mtow_t, color='green', linestyle=':', linewidth=2.0,
           label=f'Design MTOW = {MTOW_KG:.0f} kg ({mtow_t:.1f} t)')

# ── Hover ceiling markers ─────────────────────────────────────────────────────
if not np.isnan(ceil_pwr_km):
    ax.annotate(f'Power ceiling\n{ceil_pwr_km:.2f} km',
                xy=(mtow_t, ceil_pwr_km),
                xytext=(mtow_t + 0.5, ceil_pwr_km + 0.25),
                fontsize=9, color='red', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='red', lw=1.5))

if not np.isnan(ceil_stl_km):
    ax.annotate(f'Stall ceiling\n{ceil_stl_km:.2f} km',
                xy=(mtow_t, ceil_stl_km),
                xytext=(mtow_t + 0.5, ceil_stl_km - 0.5),
                fontsize=9, color='blue', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='blue', lw=1.5))

# ── Actual ceiling horizontal line ────────────────────────────────────────────
ax.axhline(ceil_act_km, color='black', linestyle='--', linewidth=1.5,
           label=f'Actual hover ceiling = {ceil_act_km:.2f} km')

# ── Region text labels ────────────────────────────────────────────────────────
ax.text(3.5, 1.0, 'SAFE\nOPERATING\nREGION', color='darkgreen',
        fontsize=10, fontweight='bold', ha='center', va='center', alpha=0.8)
ax.text(3.5, ceil_act_km + 0.3, 'CANNOT HOVER\nAT MTOW',
        color='dimgray', fontsize=9, fontweight='bold', ha='center', va='bottom')

# ── Axes ─────────────────────────────────────────────────────────────────────
ax.set_xlabel('Max Hover Gross Weight (tonnes)', fontweight='bold', fontsize=12)
ax.set_ylabel('Altitude (km)',                   fontweight='bold', fontsize=12)
ax.set_xlim(left=0)
ax.set_ylim(0, y_max)
ax.set_title(
    f'Hover Operating Envelope  |  {info}\n'
    f'Power margin >= {MIN_PWR_MARGIN:.0f}%  |  Stall limit {HOVER_STALL_LIMIT:.0f}%  |  '
    f'Collective range 10-30 deg',
    fontsize=11, fontweight='bold',
)
ax.legend(fontsize=9, loc='upper right')
ax.grid(True, alpha=0.4)

plt.tight_layout()
path = os.path.join(_OUT_DIR, 'hover_operating_envelope.png')
fig.savefig(path, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f"\nSaved: {path}")
