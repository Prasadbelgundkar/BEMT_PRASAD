"""
scripts/plot_validation.py
--------------------------
Section 3: BEMT Validation against Knight & Hefner (1937).

Plots:
  Fig 3.1 -- CT vs collective (BEMT vs experiment)
  Fig 3.2 -- CQ vs collective (BEMT vs experiment)
  Fig 3.3 -- Figure of Merit vs CT

Prints RMSE and MAPE for CT and CQ.
Saves to outputs/validation_CT_CQ_FM.png
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, constant_twist
from bemt import run_bemt
from validation import load_experimental_data, error_metrics

# ---- Validation rotor (K&H from assignment handout) ----
NUM_BLADES_VAL = 2
RADIUS_M       = 0.762
ROOT_CUTOUT_M  = 0.125
CHORD_M        = 0.0508
OMEGA_RAD_S    = 2 * np.pi * 1250.0 / 60.0  # 1250 RPM
ALTITUDE_M     = 0.0

airfoil_val = LinearAirfoil()
atmo = isa(ALTITUDE_M)
rotor_val = Rotor(radius_m=RADIUS_M, root_cutout_m=ROOT_CUTOUT_M,
                  num_blades=NUM_BLADES_VAL,
                  chord_fn=constant_chord(CHORD_M),
                  twist_fn=constant_twist(0.0),
                  name="K&H Validation Rotor")

# ---- Load experimental data ----
csv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "data", "knight_hefner_data.csv")
coll_exp_deg, CT_exp, CQ_exp = load_experimental_data(csv_path)

# ---- BEMT sweep ----
sweep_deg = np.linspace(2, 14, 25)
CT_pred, CQ_pred, FM_pred = [], [], []
for coll in sweep_deg:
    perf = run_bemt(rotor_val, lambda x: airfoil_val, OMEGA_RAD_S,
                    np.radians(coll), atmo.density_kg_m3, atmo.speed_of_sound_mps)
    CT_pred.append(perf.CT)
    CQ_pred.append(perf.CQ)
    FM_pred.append(perf.figure_of_merit if perf.figure_of_merit else np.nan)

CT_pred = np.array(CT_pred)
CQ_pred = np.array(CQ_pred)
FM_pred = np.array(FM_pred)

# ---- Error metrics (at experimentally measured collectives only) ----
CT_pred_at_exp = np.interp(coll_exp_deg, sweep_deg, CT_pred)
CQ_pred_at_exp = np.interp(coll_exp_deg, sweep_deg, CQ_pred)
CT_rmse, CT_mape = error_metrics(CT_pred_at_exp, CT_exp)
CQ_rmse, CQ_mape = error_metrics(CQ_pred_at_exp, CQ_exp)

print(f"Validation: B={NUM_BLADES_VAL}, R={RADIUS_M} m, c={CHORD_M} m, 1250 RPM, h={ALTITUDE_M} m")
print(f"  CT : RMSE={CT_rmse:.2e}  MAPE={CT_mape:.1f}%")
print(f"  CQ : RMSE={CQ_rmse:.2e}  MAPE={CQ_mape:.1f}%")

fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))

# Fig 3.1 CT
axes[0].plot(sweep_deg, CT_pred, "b-", lw=2, label="BEMT (predicted)")
axes[0].plot(coll_exp_deg, CT_exp, "ro", ms=7, label="K&H Exp. (1937)")
axes[0].set_xlabel("Collective pitch [deg]"); axes[0].set_ylabel("Thrust coefficient $C_T$")
axes[0].set_title(f"Fig 3.1 -- CT vs Collective\nB={NUM_BLADES_VAL}, R={RADIUS_M} m, 1250 RPM, SL ISA")
axes[0].legend(); axes[0].grid(alpha=0.3)
axes[0].annotate(f"RMSE={CT_rmse:.2e}\nMAPE={CT_mape:.1f}%", xy=(0.05, 0.75),
                 xycoords="axes fraction", fontsize=8,
                 bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6))

# Fig 3.2 CQ
axes[1].plot(sweep_deg, CQ_pred, "b-", lw=2, label="BEMT (predicted)")
axes[1].plot(coll_exp_deg, CQ_exp, "rs", ms=7, label="K&H Exp. (1937)")
axes[1].set_xlabel("Collective pitch [deg]"); axes[1].set_ylabel("Torque coefficient $C_Q$")
axes[1].set_title(f"Fig 3.2 -- CQ vs Collective\nB={NUM_BLADES_VAL}, R={RADIUS_M} m, 1250 RPM, SL ISA")
axes[1].legend(); axes[1].grid(alpha=0.3)
axes[1].annotate(f"RMSE={CQ_rmse:.2e}\nMAPE={CQ_mape:.1f}%", xy=(0.05, 0.75),
                 xycoords="axes fraction", fontsize=8,
                 bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6))

# Fig 3.3 FM vs CT
mask = ~np.isnan(FM_pred)
axes[2].plot(CT_pred[mask], FM_pred[mask], "b-", lw=2, label="BEMT")
axes[2].axhline(0.75, color="g", ls="--", lw=1, label="FM=0.75 reference")
axes[2].set_xlabel("Thrust coefficient $C_T$"); axes[2].set_ylabel("Figure of Merit FM")
axes[2].set_title(f"Fig 3.3 -- FM vs CT\nB={NUM_BLADES_VAL}, 1250 RPM, SL ISA")
axes[2].set_ylim(0, 1.0); axes[2].legend(); axes[2].grid(alpha=0.3)

fig.suptitle("Section 3: BEMT Validation -- Knight & Hefner (1937)", fontsize=13, fontweight="bold")
plt.tight_layout()
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "outputs", "validation_CT_CQ_FM.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
