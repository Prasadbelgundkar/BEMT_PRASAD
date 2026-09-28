# BEMT_PRASAD — Bug Fix Implementation Plan

> **Textbook Reference:** *Fundamentals of Helicopter Dynamics* — C. Venkatesan (CRC Press, 2nd Ed.)  
> All equation numbers below refer to this textbook. The codebase itself cites "Venkatesan 3.46–3.48"
> at `edgewise_bemt.py:151`, confirming this is the authoritative reference for the assignment.

---

## Priority & Execution Order

Apply fixes **top to bottom** in this order to avoid merge conflicts.

| Order | Severity | File | Line(s) | Description |
|-------|----------|------|---------|-------------|
| 1 | 🔴 Critical | `src/bemt.py` | 131 | `abs()` in momentum equation |
| 2 | 🔴 Critical | `src/airfoil.py` | 110 | Prandtl-Glauert incorrectly applied to `Cd` |
| 3 | 🔴 Critical | `src/m2/edgewise_bemt.py` | 65–66 | Glauert bracket too narrow at high advance ratio |
| 4 | 🔴 Critical | `src/mission.py` | 193 | Hardcoded `9.81` vs configurable `self.g` |
| 5 | 🟡 Medium | `src/mission.py` | 116–196 | Wing geometry hardcoded in `_required_thrust_N` |
| 6 | 🟡 Medium | `src/airfoil.py` | 75 | `TableAirfoil` stall detection off-by-one |
| 7 | 🟡 Medium | `src/m2/trim_solver.py` | 123–127 | Trim solver silently returns unconverged state |
| 8 | 🟡 Medium | `src/m2/edgewise_bemt.py` | 194–195 | P-G Mach guard missing in M2 (inconsistent with M1) |
| 9 | 🟡 Medium | `src/bemt.py` | 145–151 | Descent scan order — spurious root risk |
| 10 | 🟡 Medium | `src/m2/edgewise_bemt.py` | 211–224 | `dFy` naming ambiguity in torque calculation |
| 11 | 🟡 Medium | `src/bemt.py` | 250–254 | `advance_ratio` imported but never used |
| 12 | 🟡 Medium | `src/mission.py` | 315–350 | `_find_optimal_efficiency` defined but never called |
| 13 | 🟢 Minor | `src/aircraft_sizing.py` | 8 | `RADIUS_M` imported but unused |
| 14 | 🟢 Minor | `src/rotor.py` | 63 | Division by zero in `solidity()` |

---

## FIX 1 — `abs()` in Momentum Equation

**File:** `src/bemt.py` · **Line:** 131  
**Severity:** 🔴 Critical  
**Venkatesan Reference:** §2.4 Equations 2.21 (hover) and 2.28 (climb/descent)

### What Venkatesan Says

Eq. 2.21 (per unit span, including tip-loss):

$$\frac{dT}{dr} = 4\pi r \rho F \,(V_c + v_i)\, v_i$$

where $V_c$ is the **signed** climb velocity — negative for descent (p. 53 explicitly states:
*"For descent, $V_c$ is negative and can cause the vortex ring state when $V_c \approx -v_i$"*).

### Why the Bug Matters

`abs(v_axial + v)` forces the product to always be positive, making the residual
`dT_BET - dT_mom` lose its sign structure during descent. The Brent solver then cannot
find the correct root, and returns `v = 0` (unconverged), producing **zero induced velocity**
in descent — a physically impossible result.

### Code Change

```python
# src/bemt.py  Line 131

# ❌ BEFORE
dT_mom = 4.0 * np.pi * r * rho * F * abs(v_axial + v) * v

# ✅ AFTER  (per Venkatesan §2.4 Eq. 2.28)
dT_mom = 4.0 * np.pi * r * rho * F * (v_axial + v) * v
```

---

## FIX 2 — Prandtl-Glauert Incorrectly Applied to `Cd`

**File:** `src/airfoil.py` · **Lines:** 108–110  
**Severity:** 🔴 Critical  
**Venkatesan Reference:** §2.6 Equation 2.43

### What Venkatesan Says

Eq. 2.43:

$$C_l^{\,comp} = \frac{C_l^{\,inc}}{\sqrt{1 - M^2}}$$

No corresponding formula exists for $C_d$. The Prandtl-Glauert rule is derived from
**linearised potential (inviscid) flow** via a coordinate transformation. In potential
flow, $C_d = 0$ by D'Alembert's paradox — the theory has nothing to say about viscous
or pressure drag. Applying the same $1/\beta$ factor to $C_d$ over-corrects drag by
up to ~15% at $M = 0.65$ and inflates predicted shaft power.

### Code Change

```python
# src/airfoil.py  Lines 108–110

# ❌ BEFORE
m = min(mach, mach_limit)
beta = max((1.0 - m ** 2) ** 0.5, 1e-3)
return Cl / beta, Cd / beta

# ✅ AFTER  (per Venkatesan §2.6 Eq. 2.43 — Cl only)
m = min(mach, mach_limit)
beta = max((1.0 - m ** 2) ** 0.5, 1e-3)
return Cl / beta, Cd   # Cd unchanged: P-G is an inviscid correction (Venkatesan §2.6)
```

---

## FIX 3 — Glauert Bracket Too Narrow at High Advance Ratio

**File:** `src/m2/edgewise_bemt.py` · **Lines:** 63–66  
**Severity:** 🔴 Critical  
**Venkatesan Reference:** §3.4 Equation 3.22

### What Venkatesan Says

Eq. 3.22 (non-dimensional Glauert inflow, forward flight):

$$\lambda_i = \frac{C_T}{2\sqrt{\mu^2 + (\lambda_c + \lambda_i)^2}}$$

At high advance ratio $\mu$, the denominator is dominated by $\mu$, so:

$$\lambda_i \approx \frac{C_T}{2\mu} \ll \sqrt{\frac{C_T}{2}} = \lambda_{hover}$$

The current bracket upper bound `lam_max = √(CT/2) + 0.001` can be a poor bound
when `CT_current` hasn't converged yet in early Glauert iterations.

### Code Change

```python
# src/m2/edgewise_bemt.py  Lines 63–66

# ❌ BEFORE — bracket can be too tight in early Glauert iterations
lam_max = np.sqrt(CT / 2.0) + 1e-3
return float(brentq(residual, 1e-6, lam_max))

# ✅ AFTER — generous upper bound valid for all flight regimes (hover → fast cruise)
lam_max = np.sqrt(max(CT, 1e-6) / 2.0) + 0.5
return float(brentq(residual, 1e-6, lam_max))
```

---

## FIX 4 — Gravity Constant Inconsistency in CRUISE Drag

**File:** `src/mission.py` · **Line:** 193  
**Severity:** 🔴 Critical

### Problem

`MissionPlanner.__init__` receives `g = 9.80665` m/s² (ICAO standard gravity) as a
configurable argument, but the CRUISE `_required_thrust_N` branch hardcodes `9.81`.
The 0.034% error accumulates silently over 25,500 seconds of cruise and makes the
code impossible to unit-test with custom gravity values.

### Code Change

```python
# src/mission.py  Line 193

# ❌ BEFORE
L = self.state.gross_mass_kg * 9.81

# ✅ AFTER
L = self.state.gross_mass_kg * self.g
```

---

## FIX 5 — Wing Geometry Hardcoded in `_required_thrust_N`

**File:** `src/mission.py` · **Lines:** 116–196  
**Severity:** 🟡 Medium

### Problem

`MissionPlanner`'s own docstring states: *"This module intentionally does NOT
hard-code an aircraft."* But `_required_thrust_N` for CRUISE uses hardcoded
`wing_area = 39.24`, `AR = 9.0`, `e = 0.8`. If the wing changes, cruise drag
will silently be wrong.

### Code Change

**Step 1 — Update `__init__` signature:**

```python
# src/mission.py  MissionPlanner.__init__

# ✅ ADD three keyword arguments with the current values as defaults
def __init__(self, rotor, airfoil_provider, num_rotors, empty_mass_kg, fuel_mass_kg,
             power_model, fuel_model, limits, g=9.80665, flat_plate_area_m2=1.7,
             wing_area_m2: float = 39.24,      # <-- ADD
             wing_AR: float = 9.0,             # <-- ADD
             wing_e_oswald: float = 0.8,       # <-- ADD
             step_callback=None):
    ...
    self.wing_area_m2   = wing_area_m2
    self.wing_AR        = wing_AR
    self.wing_e_oswald  = wing_e_oswald
```

**Step 2 — Replace hardcoded values in `_required_thrust_N`:**

```python
# src/mission.py  inside _required_thrust_N, CRUISE branch

# ❌ BEFORE
wing_area = 39.24
AR = 9.0
e = 0.8

# ✅ AFTER
wing_area = self.wing_area_m2
AR        = self.wing_AR
e         = self.wing_e_oswald
```

---

## FIX 6 — `TableAirfoil` Stall Detection Off-by-One

**File:** `src/airfoil.py` · **Line:** 75  
**Severity:** 🟡 Medium  
**Venkatesan Reference:** §2.3 "Airfoil Characteristics"

### Problem

Venkatesan's treatment defines the stall angle as the **last valid linear operating
point** before separation. A table entry at exactly `alpha == lo` or `alpha == hi`
is a valid data point, not a stalled condition. Using `<=` / `>=` incorrectly flags
boundary AoA values as stalled, over-counting stalled fraction.

### Code Change

```python
# src/airfoil.py  Line 75

# ❌ BEFORE — boundary AoA wrongly flagged as stalled
stalled = alpha_rad <= lo or alpha_rad >= hi

# ✅ AFTER
stalled = alpha_rad < lo or alpha_rad > hi
```

---

## FIX 7 — Trim Solver Silently Returns Unconverged State

**File:** `src/m2/trim_solver.py` · **Lines:** 123–127  
**Severity:** 🟡 Medium

### Problem

`scipy.optimize.root` with `method='hybr'` sets `res.success = False` and stores
the last (non-converged) iterate in `res.x` when it fails. The caller has no way
to distinguish a converged from a failed trim — silent bad data propagates into
the conversion corridor plot.

### Code Change

```python
# src/m2/trim_solver.py  Lines 123–127

# ❌ BEFORE — returns bad state silently
res = root(cost, x0, method='hybr')
if not res.success:
    print(f"Warning: Trim did not converge at V={V_inf}. Msg: {res.message}")
return res.x

# ✅ AFTER — return None on failure so callers can skip the point
res = root(cost, x0, method='hybr')
if not res.success:
    print(f"[trim_solver] WARNING: Trim did not converge at V={V_inf:.1f} m/s  "
          f"residual_norm={np.linalg.norm(res.fun):.4f}  msg: {res.message}")
    return None
return res.x
```

> **Action Required:** Update all callers of `trim_aircraft()` (e.g., `conversion_corridor.py`)
> to handle `None`:
> ```python
> result = trim_aircraft(...)
> if result is None:
>     continue   # skip this speed point in the corridor sweep
> ```

Also update the return-type annotation:
```python
def trim_aircraft(...) -> Optional[np.ndarray]:
```

---

## FIX 8 — P-G Mach Guard Missing in M2 `edgewise_bemt.py`

**File:** `src/m2/edgewise_bemt.py` · **Lines:** 194–195  
**Severity:** 🟡 Medium  
**Venkatesan Reference:** §2.6 — "valid for M < 0.7 (subcritical subsonic regime)"

### Problem

M1 (`bemt.py`) explicitly guards with `if mach < 0.7`. M2 (`edgewise_bemt.py`)
has no such guard — the correction *looks* unconditional to a reader, even though
`prandtl_glauert_correct` internally caps at `mach_limit=0.7`. This also combines
naturally with Fix 2 (Cd correction) at the same location.

### Code Change

```python
# src/m2/edgewise_bemt.py  Lines 194–195

# ❌ BEFORE — appears unconditional; also incorrectly corrects Cd
mach = U_mag / a_sound
Cl, Cd = prandtl_glauert_correct(Cl, Cd, mach)

# ✅ AFTER — explicit guard (consistent with bemt.py); Cd fix applied here too
mach = U_mag / a_sound
if mach < 0.7:                                   # subcritical only (Venkatesan §2.6)
    Cl, _ = prandtl_glauert_correct(Cl, Cd, mach)   # only Cl is corrected (Fix 2)
```

---

## FIX 9 — Descent Root Scan Order

**File:** `src/bemt.py` · **Lines:** 145–151  
**Severity:** 🟡 Medium  
**Venkatesan Reference:** §2.5 pp. 58–62, Vortex Ring State

### Problem

For steep descent (`v_axial` strongly negative), the physical induced velocity
can itself be negative. The current code scans positive-v first and only falls
back to negative-v if no positive root is found. Venkatesan §2.5 notes the
residual can have a **spurious positive root** near the vortex ring state
($V_c \approx -v_i$), causing the solver to latch onto the wrong solution.

### Code Change

```python
# src/bemt.py  Lines 145–151

# ❌ BEFORE — always scans positive v first
v_lo_scan, v_hi_scan = v_scan_range
bracket = find_bracket(0.0, v_hi_scan, n_scan // 2)
if bracket is None:
    bracket = find_bracket(0.0, v_lo_scan, n_scan // 2)

# ✅ AFTER — prefer negative scan for descent cases
v_lo_scan, v_hi_scan = v_scan_range
if v_axial < -1.0:
    # Descending: physical induced velocity may be negative; scan that direction first
    bracket = find_bracket(0.0, v_lo_scan, n_scan // 2)
    if bracket is None:
        bracket = find_bracket(0.0, v_hi_scan, n_scan // 2)
else:
    # Hover / climb / propeller: positive induced velocity expected
    bracket = find_bracket(0.0, v_hi_scan, n_scan // 2)
    if bracket is None:
        bracket = find_bracket(0.0, v_lo_scan, n_scan // 2)
```

---

## FIX 10 — `dFy` Naming Ambiguity in Torque Calculation

**File:** `src/m2/edgewise_bemt.py` · **Lines:** 211–224  
**Severity:** 🟡 Medium (latent bug risk)

### Problem

`dFy` is used for the in-plane drag (circumferential) force on the blade element,
but the name suggests a body-frame Y-axis force. This creates confusion when
reading the hub-force decomposition (`dFx_hub`, `dFy_hub`) which also use `dFy`
as input — making it unclear whether these refer to the same quantity or different frames.

### Code Change

```python
# src/m2/edgewise_bemt.py  Lines 211–220

# ❌ BEFORE — ambiguous naming
dFz = B * (dL * np.cos(phi) - dD * np.sin(phi))
dFy = B * (dL * np.sin(phi) + dD * np.cos(phi))
...
dFx_hub = dFy * np.sin(psi)
dFy_hub = -dFy * np.cos(psi)
...
dQ_mat[i, j] = dFy * r

# ✅ AFTER — rename to clearly indicate in-plane (circumferential) direction
dFz       = B * (dL * np.cos(phi) - dD * np.sin(phi))  # Axial (thrust), hub frame
dF_inplane = B * (dL * np.sin(phi) + dD * np.cos(phi)) # In-plane (drag), hub frame
...
dFx_hub = dF_inplane * np.sin(psi)
dFy_hub = -dF_inplane * np.cos(psi)
...
dQ_mat[i, j] = dF_inplane * r   # Torque = in-plane drag force × moment arm
```

---

## FIX 11 — Dead Code: `advance_ratio` Imported but Never Called

**File:** `src/bemt.py` · **Line:** 250 | `src/mission.py` · **Line:** 28  
**Severity:** 🟡 Medium

### Option A — Wire it into the mission log (Recommended)

```python
# src/mission.py  inside run_segment(), inside the log.append() dict:
advance_J=advance_ratio(v_axial, omega, self.rotor.radius_m),
```

### Option B — Remove the dead code

```python
# src/mission.py  Line 28
# ❌ BEFORE
from bemt import run_bemt, advance_ratio

# ✅ AFTER
from bemt import run_bemt
```

---

## FIX 12 — Dead Code: `_find_optimal_efficiency` Never Called

**File:** `src/mission.py` · **Lines:** 315–350  
**Severity:** 🟡 Medium

### Recommended Action

Add a docstring explaining why it is currently disabled, so the marker knows it
is intentional and not forgotten:

```python
def _find_optimal_efficiency(self, target_thrust_per_rotor, atmo, v_axial, fallback_rpm):
    """
    RPM sweep optimizer: scans allowed RPM range to find the lowest-power
    (most fuel-efficient) trim state for the given flight condition.

    STATUS: Implemented but currently deferred.
    REASON: Each RPM sweep evaluates ~(max_rpm - min_rpm)/25 full BEMT solves.
            With a 60-second cruise time-step this is acceptable offline, but
            adds excessive wall-clock time for interactive mission runs.
    TO ENABLE: Replace the `_optimize_trim` call in `run_segment` with this
               method for CRUISE segments only.
    """
    ...
```

---

## FIX 13 — Unused Import `RADIUS_M` in `aircraft_sizing.py`

**File:** `src/aircraft_sizing.py` · **Line:** 8  
**Severity:** 🟢 Minor

### Code Change

```python
# src/aircraft_sizing.py  Lines 5–9

# ❌ BEFORE
from parameters import (
    EMPTY_MASS_KG, PAYLOAD_MASS_KG, FUEL_MASS_KG,
    FLAT_PLATE_AREA_M2, NUM_ENGINES, ENGINE_POWER_W,
    CRUISE_ALTITUDE_AMSL_M, RADIUS_M
)

# ✅ AFTER
from parameters import (
    EMPTY_MASS_KG, PAYLOAD_MASS_KG, FUEL_MASS_KG,
    FLAT_PLATE_AREA_M2, NUM_ENGINES, ENGINE_POWER_W,
    CRUISE_ALTITUDE_AMSL_M
)
```

---

## FIX 14 — Division by Zero in `rotor.solidity()`

**File:** `src/rotor.py` · **Line:** 63  
**Severity:** 🟢 Minor

### Code Change

```python
# src/rotor.py  Lines 59–64

# ❌ BEFORE — crashes with ZeroDivisionError if root_cutout_m == radius_m
def solidity(self, n_stations: int = 200) -> float:
    x = np.linspace(self.root_cutout_m / self.radius_m, 1.0, n_stations)
    chords = np.array([self.chord_fn(xi) for xi in x])
    mean_chord = _trapz(chords, x) / (1.0 - self.root_cutout_m / self.radius_m)
    return self.num_blades * mean_chord / (np.pi * self.radius_m)

# ✅ AFTER
def solidity(self, n_stations: int = 200) -> float:
    x_root   = self.root_cutout_m / self.radius_m
    span_frac = 1.0 - x_root
    if span_frac <= 0.0:
        raise ValueError(
            f"Degenerate rotor: root_cutout_m ({self.root_cutout_m:.3f} m) "
            f">= radius_m ({self.radius_m:.3f} m)."
        )
    x = np.linspace(x_root, 1.0, n_stations)
    chords = np.array([self.chord_fn(xi) for xi in x])
    mean_chord = _trapz(chords, x) / span_frac
    return self.num_blades * mean_chord / (np.pi * self.radius_m)
```

---

## Venkatesan Equation Cross-Reference Table

| Fix | §  | Equation | Statement Verified |
|-----|----|----------|--------------------|
| 1 — momentum `abs()` | §2.4 | Eq. 2.21, 2.28 | `dT/dr = 4πrρF(Vc+vi)vi` — $V_c$ is signed |
| 2 — P-G on Cd | §2.6 | Eq. 2.43 | `Cl_comp = Cl_inc / √(1−M²)` — Cl only |
| 3 — Glauert bracket | §3.4 | Eq. 3.22 | `λi = CT/[2√(μ²+(λc+λi)²)]` — λi ≪ λhover at high μ |
| 6 — stall boundary | §2.3 | — | Stall angle = last valid data point |
| 8 — P-G Mach guard | §2.6 | Eq. 2.43 | Correction valid for M < 0.7 (subcritical) only |
| 9 — descent scan | §2.5 | pp. 58–62 | Vortex ring state: multiple residual roots near Vc ≈ −vi |

---

## Post-Fix Test Commands

```powershell
# Run from the project root (BEMT_PRASAD-main/BEMT_PRASAD-main)
cd "f:\Rotory Wing\BEMT_PRASAD-main\BEMT_PRASAD-main"

# M1 tests
pytest tests/test_bemt.py -v

# M2 tests
pytest tests/m2/ -v

# Quick smoke test — run the main mission planner
python src/run_mission.py
```

---

*Generated: 2026-09-28 | Reviewer: Antigravity AI Code Review*
