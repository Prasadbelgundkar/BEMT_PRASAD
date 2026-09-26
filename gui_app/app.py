"""
Propelli/o – Rotor BEMT Analysis
=====================================
Phase 2: Interactive Plotly plots, Multi-airfoil blending, Rotor Schematic, and Branding.
"""

import sys
import os
import io
import pathlib

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import streamlit as st

# ── path setup ──────────────────────────────────────────────────────────────
_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE / "src"))

from rotor import Rotor, linear_taper_chord, linear_twist, constant_chord, constant_twist
from airfoil import LinearAirfoil, TableAirfoil
from bemt import run_bemt
from environment import isa

# ── page config ──────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="Propelli/o",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded",
)

# colour palette
C_THRUST   = "#4fc3f7"
C_TORQUE   = "#ef5350"
C_CHORD    = "#b0bec5"
C_TWIST    = "#66bb6a"
C_AOA      = "#ce93d8"
C_INFLOW   = "#4dd0e1"
C_TIPL     = "#ffca28"
C_MACH     = "#ff7043"
C_STALL    = "#ef5350"
C_CL       = "#4fc3f7"
C_CD       = "#ef5350"
C_CLCD     = "#66bb6a"
C_POLAR    = "#ffca28"
COLORS_MULTI = ["#4fc3f7", "#ef5350", "#66bb6a", "#ffca28", "#ce93d8"]

AIRFOIL_DIR = _HERE / "airfoils"


# ═══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

class MultiSectionAirfoilProvider:
    """Piecewise airfoil provider: different airfoil per radial section."""
    def __init__(self, boundaries, airfoils, labels):
        # boundaries = [0.0, 0.4, 1.0]  (N+1 values for N sections)
        # airfoils   = [airfoil_root, airfoil_tip]  (N airfoil objects)
        self.boundaries = boundaries
        self.airfoils = airfoils
        self.labels = labels

    def __call__(self, x):
        """Return the airfoil for radial station x (r/R)."""
        for i in range(len(self.airfoils)):
            if x <= self.boundaries[i + 1]:
                return self.airfoils[i]
        return self.airfoils[-1]


def parse_airfoiltools_csv(content: str) -> pd.DataFrame:
    """
    Parse a raw Airfoil Tools / XFOIL polar CSV/txt.
    """
    lines = content.splitlines()

    # ── Step 1: find the header row.
    header_idx = None
    for i, line in enumerate(lines):
        stripped = line.strip()
        if not stripped:
            continue
        if "\t" in stripped:
            parts = [p.strip().lower() for p in stripped.split("\t")]
        else:
            parts = [p.strip().lower() for p in stripped.split(",")]
        if len(parts) >= 3 and parts[0] in ("alpha", "alfa"):
            header_idx = i
            break

    if header_idx is not None:
        candidate_lines = lines[header_idx:]
        header_line = candidate_lines[0]
        delimiter = "\t" if "\t" in header_line else ("," if "," in header_line else None)

        clean_lines = [candidate_lines[0]]
        for ln in candidate_lines[1:]:
            stripped = ln.strip()
            if not stripped:
                continue
            first_char = stripped[0]
            if first_char.isdigit() or first_char == "-" or first_char == ".":
                clean_lines.append(ln)

        clean_text = "\n".join(clean_lines)

        if delimiter:
            df = pd.read_csv(io.StringIO(clean_text), sep=delimiter)
        else:
            df = pd.read_csv(io.StringIO(clean_text), sep=r"\s+", engine="python")
    else:
        try:
            df = pd.read_csv(io.StringIO(content))
            if df.shape[1] < 3: raise ValueError
        except Exception:
            df = pd.read_csv(io.StringIO(content), sep=r"\s+", engine="python", comment="#")

    df.columns = [str(c).strip().lower().replace(" ", "_") for c in df.columns]

    rename = {}
    for col in df.columns:
        clean = col.replace("_", "")
        if "alpha" in clean or "alfa" in clean: rename[col] = "alpha_deg"
        elif clean == "cl": rename[col] = "Cl"
        elif clean == "cd" and "p" not in clean: rename[col] = "Cd"
    df = df.rename(columns=rename)

    needed = [c for c in ["alpha_deg", "Cl", "Cd"] if c in df.columns]
    if len(needed) < 3:
        raise ValueError(f"Could not find Alpha, Cl and Cd columns. Found: {list(df.columns)}")
    
    df = df[["alpha_deg", "Cl", "Cd"]].copy()
    df = df.apply(pd.to_numeric, errors="coerce").dropna()
    df = df.sort_values("alpha_deg").reset_index(drop=True)

    if len(df) < 3:
        raise ValueError("Fewer than 3 valid numeric data rows found. Check the file.")

    return df


def load_builtin_airfoil(csv_path: pathlib.Path) -> TableAirfoil:
    df = pd.read_csv(csv_path)
    return TableAirfoil(
        alpha_deg=df["alpha_deg"].tolist(),
        Cl=df["Cl"].tolist(),
        Cd=df["Cd"].tolist(),
        name=csv_path.stem,
    )

def builtin_airfoil_names() -> dict:
    if not AIRFOIL_DIR.exists(): return {}
    return {p.stem.replace("_", " "): p for p in sorted(AIRFOIL_DIR.glob("*.csv"))}


def draw_rotor_schematic(R, root_cutout, blades, chord_fn):
    """Draws a static top-down schematic of the rotor geometry."""
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    ax.set_aspect('equal')
    ax.axis('off')

    # Hub cutout
    hub = mpatches.Circle((0, 0), root_cutout, color='#555577', alpha=0.5, zorder=2)
    ax.add_patch(hub)

    # Blades
    for i in range(blades):
        angle = i * (2 * np.pi / blades)
        rs = np.linspace(root_cutout, R, 50)
        chords = np.array([chord_fn(r/R) for r in rs])
        
        y_upper = chords / 2
        y_lower = -chords / 2
        
        xs_upper = rs * np.cos(angle) - y_upper * np.sin(angle)
        ys_upper = rs * np.sin(angle) + y_upper * np.cos(angle)
        xs_lower = rs * np.cos(angle) - y_lower * np.sin(angle)
        ys_lower = rs * np.sin(angle) + y_lower * np.cos(angle)
        
        poly_pts = np.column_stack([np.concatenate([xs_upper, xs_lower[::-1]]),
                                    np.concatenate([ys_upper, ys_lower[::-1]])])
        blade_poly = mpatches.Polygon(poly_pts, color='#b0bec5', alpha=0.8, zorder=3)
        ax.add_patch(blade_poly)

    # Rotation arrow
    ax.annotate("", xy=(R*0.85, R*0.15), xytext=(R*0.85, -R*0.15),
                arrowprops=dict(arrowstyle="->", color="#e0e0f0", lw=2, connectionstyle="arc3,rad=0.3"))
    ax.text(R*0.95, 0, "Ω", color="#e0e0f0", fontsize=14, va='center', ha='left')

    # Radius annotation
    ax.annotate("", xy=(0, 0), xytext=(0, R), arrowprops=dict(arrowstyle="<->", color="#4fc3f7", lw=1.5))
    ax.text(-R*0.05, R/2, "Radius R", color="#4fc3f7", va='center', ha='right', fontsize=10)

    # Cutout annotation
    if root_cutout > 0.05*R:
        ax.annotate("", xy=(0, 0), xytext=(root_cutout, 0), arrowprops=dict(arrowstyle="<->", color="#ef5350", lw=1.5))
        ax.text(root_cutout/2, -R*0.08, "Cutout", color="#ef5350", ha='center', fontsize=9)

    ax.set_xlim(-R*1.15, R*1.15)
    ax.set_ylim(-R*1.15, R*1.15)
    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
# SIDEBAR
# ═══════════════════════════════════════════════════════════════════════════════

with st.sidebar:
    st.image(str(_HERE / "logo_dark.png"), width=80)
    st.markdown("### Propelli/o Settings")

    with st.expander("ℹ️ How to Use"):
        st.markdown(
            """
1. Set **Rotor Geometry** (radius, blades, chord, twist).
2. Choose an **Airfoil** (Linear, Built-in, Upload, or Multi-Section).
3. Set **Operating Conditions** (altitude auto-computes ISA).
4. Click **🚀 Run Analysis** to solve and plot results.
5. Hover over plots to see exact values.
            """
        )
    st.divider()

    st.header("1 · Rotor Geometry")
    radius_m = st.number_input("Blade Radius (m)", min_value=0.10, max_value=30.0, value=5.0, step=0.1)
    root_cutout_m = st.number_input("Root Cutout (m)", min_value=0.0, max_value=float(radius_m) - 0.05, value=min(0.5, float(radius_m) * 0.1), step=0.05)
    num_blades = st.number_input("Number of Blades", min_value=2, max_value=8, value=3, step=1)

    st.subheader("Chord Distribution")
    chord_type = st.selectbox("Chord Profile", ["Constant", "Linear Taper"])
    root_chord = st.number_input("Root Chord (m)", min_value=0.01, max_value=3.0, value=0.30, step=0.01)
    if chord_type == "Linear Taper":
        taper_ratio = st.slider("Taper Ratio (c_tip / c_root)", 0.10, 1.0, 0.70, 0.05)
        chord_fn = linear_taper_chord(root_chord, taper_ratio)
    else:
        taper_ratio = 1.0
        chord_fn = constant_chord(root_chord)

    st.subheader("Twist Distribution")
    twist_type = st.selectbox("Twist Profile", ["Linear Twist", "Constant"])
    theta_root_deg = st.number_input("Root Pitch (deg)", min_value=-30.0, max_value=60.0, value=10.0, step=0.5)
    if twist_type == "Linear Twist":
        twist_rate_deg = st.number_input("Washout / Twist Rate (deg root→tip)", min_value=-60.0, max_value=20.0, value=-8.0, step=0.5)
        twist_fn = linear_twist(np.radians(theta_root_deg), np.radians(twist_rate_deg))
    else:
        twist_fn = constant_twist(np.radians(theta_root_deg))

    st.divider()

    st.header("2 · Airfoil")
    airfoil_mode = st.radio(
        "Airfoil Source",
        ["Linear Model (default)", "Built-in Library", "Upload AirfoilTools / XFOIL CSV", "Multi-Section (Radial Blend)"]
    )

    airfoil_provider_callable = None
    airfoil_label = "Linear Model"
    airfoil_obj = LinearAirfoil()

    if airfoil_mode == "Linear Model (default)":
        with st.expander("Edit Linear Model Parameters"):
            a0_val    = st.number_input("a₀ – Lift slope", value=5.75, step=0.05)
            cdmin_val = st.number_input("Cd_min", value=0.0113, step=0.001, format="%.4f")
            eps_val   = st.number_input("ε – Quad drag", value=1.25, step=0.05)
            stall_deg = st.number_input("Stall α (deg)", value=12.0, step=0.5)
        airfoil_obj = LinearAirfoil(a0=a0_val, Cd_min=cdmin_val, eps=eps_val, stall_alpha_rad=np.radians(stall_deg))
        airfoil_provider_callable = lambda x: airfoil_obj

    elif airfoil_mode == "Built-in Library":
        lib = builtin_airfoil_names()
        if not lib:
            st.warning("No built-in airfoil files found.")
            airfoil_provider_callable = lambda x: airfoil_obj
        else:
            chosen = st.selectbox("Select Airfoil", list(lib.keys()))
            try:
                airfoil_obj = load_builtin_airfoil(lib[chosen])
                airfoil_label = chosen
                st.success(f"✅ Loaded {chosen}")
            except Exception as e:
                st.error(f"Failed to load: {e}")
            airfoil_provider_callable = lambda x: airfoil_obj

    elif airfoil_mode == "Upload AirfoilTools / XFOIL CSV":
        uploaded = st.file_uploader("Upload Polar File", type=["csv", "txt", "dat"])
        if uploaded is not None:
            try:
                raw = uploaded.getvalue().decode("utf-8", errors="replace")
                df_polar = parse_airfoiltools_csv(raw)
                airfoil_obj = TableAirfoil(
                    alpha_deg=df_polar["alpha_deg"].tolist(),
                    Cl=df_polar["Cl"].tolist(),
                    Cd=df_polar["Cd"].tolist(),
                    name=uploaded.name,
                )
                airfoil_label = uploaded.name
                st.success(f"✅ {uploaded.name}")
            except Exception as e:
                st.error(f"❌ Parse error: {e}")
        airfoil_provider_callable = lambda x: airfoil_obj

    elif airfoil_mode == "Multi-Section (Radial Blend)":
        n_sections = st.slider("Number of Sections", 2, 5, 2)
        boundaries = [0.0]
        airfoils_list = []
        labels_list = []
        lib = builtin_airfoil_names()
        
        for i in range(n_sections):
            st.markdown(f"**Section {i+1}**")
            if i < n_sections - 1:
                b = st.slider(f"Outer Boundary (r/R) for Sec {i+1}", 
                              min_value=float(boundaries[-1] + 0.05), 
                              max_value=0.95, 
                              value=float(boundaries[-1] + (1.0 - boundaries[-1]) / (n_sections - i)),
                              step=0.01, key=f"bound_{i}")
                boundaries.append(b)
            else:
                boundaries.append(1.0)
                st.caption("Outer Boundary: 1.0 (Tip)")
                
            src = st.selectbox(f"Source (Sec {i+1})", ["Built-in Library", "Upload CSV", "Linear Model"], key=f"src_{i}")
            if src == "Built-in Library" and lib:
                chosen = st.selectbox(f"Select Airfoil (Sec {i+1})", list(lib.keys()), key=f"sel_{i}")
                try:
                    af = load_builtin_airfoil(lib[chosen])
                    airfoils_list.append(af)
                    labels_list.append(chosen)
                except Exception as e:
                    st.error(f"Failed to load: {e}")
                    airfoils_list.append(LinearAirfoil())
                    labels_list.append("Linear (Fallback)")
            elif src == "Upload CSV":
                uploaded = st.file_uploader(f"Upload CSV (Sec {i+1})", type=["csv", "txt", "dat"], key=f"up_{i}")
                if uploaded:
                    try:
                        raw = uploaded.getvalue().decode("utf-8", errors="replace")
                        df_polar = parse_airfoiltools_csv(raw)
                        af = TableAirfoil(
                            alpha_deg=df_polar["alpha_deg"].tolist(),
                            Cl=df_polar["Cl"].tolist(),
                            Cd=df_polar["Cd"].tolist(),
                            name=uploaded.name,
                        )
                        airfoils_list.append(af)
                        labels_list.append(uploaded.name)
                    except Exception as e:
                        st.error(f"Parse error: {e}")
                        airfoils_list.append(LinearAirfoil())
                        labels_list.append("Error (Linear)")
                else:
                    st.info("Waiting for file...")
                    airfoils_list.append(LinearAirfoil())
                    labels_list.append("Linear")
            else:
                airfoils_list.append(LinearAirfoil())
                labels_list.append("Linear")
            st.write("") # spacer
            
        airfoil_provider_callable = MultiSectionAirfoilProvider(boundaries, airfoils_list, labels_list)
        airfoil_label = "Multi-Section Blended"
        airfoil_obj = airfoil_provider_callable # Store provider as object for multi handling

    st.divider()

    st.header("3 · Operating Conditions")
    omega_rpm = st.number_input("Rotor Speed (RPM)", 10.0, 5000.0, 300.0, 10.0)
    collective_deg = st.number_input("Collective Pitch (deg)", -30.0, 40.0, 0.0, 0.5)
    v_axial = st.number_input("Axial Velocity (m/s)", 0.0, 300.0, 0.0, 1.0)

    st.subheader("ISA Atmosphere")
    altitude_m = st.slider("Altitude (m AMSL)", 0, 10000, 0, 100)
    disa_k = st.slider("Temperature Offset ΔT_ISA (K)", -30, 30, 0, 1)

    try:
        atmo = isa(float(altitude_m), float(disa_k))
        rho, a_sound = atmo.density_kg_m3, atmo.speed_of_sound_mps
        st.info(f"**T** = {atmo.temperature_K:.1f} K  ·  **ρ** = {rho:.4f} kg/m³  ·  **a** = {a_sound:.1f} m/s")
    except Exception as e:
        st.error(f"ISA error: {e}")
        rho, a_sound = 1.225, 340.0

    st.divider()
    run_btn   = st.button("🚀 Run Analysis", use_container_width=True, type="primary")
    reset_btn = st.button("🔄 Reset Results", use_container_width=True)

    if reset_btn:
        for k in ["perf", "rotor_snapshot", "airfoil_snapshot"]:
            st.session_state.pop(k, None)
        st.rerun()


# ═══════════════════════════════════════════════════════════════════════════════
# BUILD & RUN
# ═══════════════════════════════════════════════════════════════════════════════

rotor = Rotor(radius_m=float(radius_m), root_cutout_m=float(root_cutout_m), num_blades=int(num_blades), chord_fn=chord_fn, twist_fn=twist_fn)
omega_rad_s = float(omega_rpm) * np.pi / 30.0
collective_rad = np.radians(float(collective_deg))

if run_btn:
    with st.spinner("⚙️ Running BEMT solver…"):
        try:
            perf = run_bemt(
                rotor=rotor, airfoil_provider=airfoil_provider_callable,
                omega_rad_s=omega_rad_s, collective_rad=collective_rad,
                rho=rho, a_sound=a_sound, v_axial=float(v_axial), n_stations=80
            )
            st.session_state["perf"]  = perf
            st.session_state["rotor_snapshot"]   = rotor
            st.session_state["airfoil_snapshot"] = airfoil_obj
            st.session_state["airfoil_label"]    = airfoil_label
        except Exception as e:
            st.error(f"❌ Solver error: {e}")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN PANEL
# ═══════════════════════════════════════════════════════════════════════════════

col_logo, col_title = st.columns([1, 10])
with col_logo:
    st.image(str(_HERE / "logo_dark.png"), width=80)
with col_title:
    st.markdown("# Propelli/o")
    st.caption("Blade Element Momentum Theory — Rotor Analysis Tool")

if "perf" not in st.session_state:
    st.markdown("Configure the rotor in the **sidebar**, then click **🚀 Run Analysis**.")
    st.stop()

perf    = st.session_state["perf"]
r_snap  = st.session_state["rotor_snapshot"]
af_snap = st.session_state["airfoil_snapshot"]
af_lbl  = st.session_state.get("airfoil_label", "Unknown")

elems       = perf.elements
r_R         = np.array([e.x for e in elems])
dT_arr      = np.array([e.dT_dr for e in elems])
dQ_arr      = np.array([e.dQ_dr for e in elems])
alpha_arr   = np.degrees(np.array([e.alpha_rad for e in elems]))
mach_arr    = np.array([e.mach for e in elems])
inflow_arr  = np.array([e.v_induced for e in elems])
F_arr       = np.array([e.tip_loss_F for e in elems])
stall_arr   = np.array([e.stalled for e in elems], dtype=bool)
Cl_arr      = np.array([e.Cl for e in elems])
Cd_arr      = np.array([e.Cd for e in elems])
chord_arr   = np.array([r_snap.chord_fn(x) for x in r_R])
twist_arr   = np.degrees(np.array([r_snap.twist_fn(x) for x in r_R]))

sigma = r_snap.solidity()
vtip  = r_snap.tip_speed_mps(omega_rad_s)

st.divider()
st.markdown(f"### Analysis Results — {af_lbl}")
st.caption(f"R = {r_snap.radius_m:.2f} m  ·  B = {r_snap.num_blades}  ·  σ = {sigma:.3f}  ·  Ω = {omega_rpm:.0f} RPM  ·  θ_coll = {collective_deg:.1f}°  ·  V_ax = {v_axial:.1f} m/s")

if not perf.converged:
    st.warning("⚠️  BEMT did not converge at all stations — results may be inaccurate.")
if perf.stalled_fraction > 0.15:
    st.error(f"🔴  {perf.stalled_fraction*100:.1f} % of the blade is stalled — reduce collective or check airfoil range.")
elif perf.stalled_fraction > 0:
    st.warning(f"⚠️  {perf.stalled_fraction*100:.1f} % of the blade is stalled.")

# ── Metrics ─────────────────────────────────────────────────────────────
m1, m2, m3, m4, m5, m6 = st.columns(6)
m1.metric("Thrust", f"{perf.thrust_N:.1f} N")
m2.metric("Torque", f"{perf.torque_Nm:.1f} N·m")
m3.metric("Power", f"{perf.power_W/1000:.2f} kW")
m4.metric("CT", f"{perf.CT:.4f}")
m5.metric("FM / η", f"{perf.figure_of_merit:.3f}" if perf.figure_of_merit else (f"{perf.propulsive_efficiency:.3f}" if perf.propulsive_efficiency else "N/A"))
m6.metric("Max Tip Mach", f"{perf.max_tip_mach:.3f}")

with st.expander("📐 Nondimensional Coefficients & Design Info"):
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("CT", f"{perf.CT:.5f}"); c2.metric("CQ", f"{perf.CQ:.6f}"); c3.metric("CP", f"{perf.CP:.6f}"); c4.metric("Solidity σ", f"{sigma:.4f}")
    d1, d2, d3, d4 = st.columns(4)
    d1.metric("Disk Loading", f"{perf.thrust_N / r_snap.disk_area_m2():.1f} N/m²"); d2.metric("Disk Area", f"{r_snap.disk_area_m2():.2f} m²")
    d3.metric("Tip Speed", f"{vtip:.1f} m/s"); d4.metric("Stalled Span", f"{perf.stalled_fraction*100:.1f} %")

st.divider()
tab1, tab2, tab3 = st.tabs(["📈 Aerodynamic Loading", "🔧 Blade Geometry & Inflow", "✈️ Airfoil Polar"])

def get_base_layout(title=""):
    return dict(template="plotly_dark", plot_bgcolor="#1a1a2e", paper_bgcolor="#1a1a2e",
                margin=dict(l=40, r=40, t=60, b=40), showlegend=True,
                legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01))

# ─────────────────────────────────────────────────────────────────────────────
# TAB 1 – Aerodynamic Loading
# ─────────────────────────────────────────────────────────────────────────────
with tab1:
    fig1 = make_subplots(rows=2, cols=2, subplot_titles=("Spanwise Thrust (dT/dr)", "Spanwise Torque (dQ/dr)", "Sectional Angle of Attack", "Mach Number"))

    fig1.add_trace(go.Scatter(x=r_R, y=dT_arr, mode='lines', line=dict(color=C_THRUST, width=2.5), fill='tozeroy', fillcolor='rgba(79, 195, 247, 0.15)', name="dT/dr", hovertemplate="r/R: %{x:.3f}<br>dT/dr: %{y:.1f} N/m<extra></extra>"), row=1, col=1)
    fig1.add_trace(go.Scatter(x=r_R, y=dQ_arr, mode='lines', line=dict(color=C_TORQUE, width=2.5), fill='tozeroy', fillcolor='rgba(239, 83, 80, 0.15)', name="dQ/dr", hovertemplate="r/R: %{x:.3f}<br>dQ/dr: %{y:.2f} N·m/m<extra></extra>"), row=1, col=2)
    
    fig1.add_trace(go.Scatter(x=r_R, y=alpha_arr, mode='lines', line=dict(color=C_AOA, width=2.5), name="AoA", hovertemplate="r/R: %{x:.3f}<br>AoA: %{y:.2f}°<extra></extra>"), row=2, col=1)
    if stall_arr.any():
        fig1.add_trace(go.Scatter(x=r_R[stall_arr], y=alpha_arr[stall_arr], mode='markers', marker=dict(color=C_STALL, size=6), name="Stalled", hovertemplate="r/R: %{x:.3f}<br>AoA: %{y:.2f}° (Stall)<extra></extra>"), row=2, col=1)

    fig1.add_trace(go.Scatter(x=r_R, y=mach_arr, mode='lines', line=dict(color=C_MACH, width=2.5), name="Mach", hovertemplate="r/R: %{x:.3f}<br>Mach: %{y:.3f}<extra></extra>"), row=2, col=2)
    fig1.add_hline(y=0.9, line_dash="dash", line_color=C_STALL, annotation_text="M=0.9", row=2, col=2)

    fig1.update_layout(**get_base_layout(), height=700)
    fig1.update_xaxes(title_text="r/R")
    st.plotly_chart(fig1, use_container_width=True)

# ─────────────────────────────────────────────────────────────────────────────
# TAB 2 – Blade Geometry & Inflow
# ─────────────────────────────────────────────────────────────────────────────
with tab2:
    with st.expander("📐 Rotor Schematic", expanded=True):
        st.pyplot(draw_rotor_schematic(r_snap.radius_m, r_snap.root_cutout_m, r_snap.num_blades, chord_fn))

    fig2 = make_subplots(rows=2, cols=2, subplot_titles=("Blade Planform", "Twist Distribution", "Inflow Velocity", "Tip-Loss Factor (F)"))

    half = chord_arr / 2.0
    fig2.add_trace(go.Scatter(x=r_R, y=half, mode='lines', line=dict(color=C_CHORD, width=2), name="+Chord/2", showlegend=False, hovertemplate="r/R: %{x:.3f}<br>y: %{y:.3f} m<extra></extra>"), row=1, col=1)
    fig2.add_trace(go.Scatter(x=r_R, y=-half, mode='lines', line=dict(color=C_CHORD, width=2), fill='tonexty', fillcolor='rgba(176, 190, 197, 0.35)', name="-Chord/2", showlegend=False, hovertemplate="r/R: %{x:.3f}<br>y: %{y:.3f} m<extra></extra>"), row=1, col=1)
    
    if isinstance(af_snap, MultiSectionAirfoilProvider):
        for idx, b in enumerate(af_snap.boundaries[1:-1]):
            fig2.add_vline(x=b, line_dash="dash", line_color="#fff", annotation_text=f"Sec {idx+1}|{idx+2}", row=1, col=1)

    fig2.add_trace(go.Scatter(x=r_R, y=twist_arr, mode='lines', line=dict(color=C_TWIST, width=2.5), name="Twist", hovertemplate="r/R: %{x:.3f}<br>Twist: %{y:.2f}°<extra></extra>"), row=1, col=2)
    fig2.add_trace(go.Scatter(x=r_R, y=inflow_arr, mode='lines', line=dict(color=C_INFLOW, width=2.5), fill='tozeroy', fillcolor='rgba(77, 208, 225, 0.15)', name="Inflow", hovertemplate="r/R: %{x:.3f}<br>Inflow: %{y:.2f} m/s<extra></extra>"), row=2, col=1)
    fig2.add_trace(go.Scatter(x=r_R, y=F_arr, mode='lines', line=dict(color=C_TIPL, width=2.5), fill='tonexty', fillcolor='rgba(255, 202, 40, 0.12)', name="Tip Loss F", hovertemplate="r/R: %{x:.3f}<br>F: %{y:.3f}<extra></extra>"), row=2, col=2)
    
    fig2.update_layout(**get_base_layout(), height=700)
    fig2.update_xaxes(title_text="r/R")
    st.plotly_chart(fig2, use_container_width=True)

# ─────────────────────────────────────────────────────────────────────────────
# TAB 3 – Airfoil Polar
# ─────────────────────────────────────────────────────────────────────────────
with tab3:
    if isinstance(af_snap, MultiSectionAirfoilProvider):
        airfoils_to_plot = af_snap.airfoils
        labels = af_snap.labels
    else:
        airfoils_to_plot = [af_snap]
        labels = [af_lbl]

    fig3 = make_subplots(rows=2, cols=2, subplot_titles=("Lift Coefficient (Cl vs α)", "Drag Coefficient (Cd vs α)", "Lift-to-Drag (Cl/Cd vs α)", "Drag Polar (Cl vs Cd)"))

    for idx, (af, lbl) in enumerate(zip(airfoils_to_plot, labels)):
        color = COLORS_MULTI[idx % len(COLORS_MULTI)]
        
        if isinstance(af, TableAirfoil):
            alpha_plot = np.degrees(af.alpha_rad_arr)
            a_lo, a_hi = float(alpha_plot[0]), float(alpha_plot[-1])
        else:
            a_lo, a_hi = -12.0, 20.0

        alphas_plot = np.linspace(a_lo, a_hi, 200)
        Cl_pl, Cd_pl, st_pl = [], [], []
        for a in alphas_plot:
            cl, cd, stalled = af.get_coeffs(np.radians(a))
            Cl_pl.append(cl)
            Cd_pl.append(cd)
            st_pl.append(stalled)
            
        Cl_pl = np.array(Cl_pl)
        Cd_pl = np.array(Cd_pl)
        ClCd_pl = np.where(Cd_pl > 1e-6, Cl_pl / Cd_pl, 0.0)

        fig3.add_trace(go.Scatter(x=alphas_plot, y=Cl_pl, mode='lines', line=dict(color=color, width=2), name=f"{lbl} Cl", hovertemplate="α: %{x:.1f}°<br>Cl: %{y:.3f}<extra></extra>"), row=1, col=1)
        fig3.add_trace(go.Scatter(x=alphas_plot, y=Cd_pl, mode='lines', line=dict(color=color, width=2), name=f"{lbl} Cd", hovertemplate="α: %{x:.1f}°<br>Cd: %{y:.4f}<extra></extra>"), row=1, col=2)
        fig3.add_trace(go.Scatter(x=alphas_plot, y=ClCd_pl, mode='lines', line=dict(color=color, width=2), name=f"{lbl} L/D", hovertemplate="α: %{x:.1f}°<br>L/D: %{y:.1f}<extra></extra>"), row=2, col=1)
        fig3.add_trace(go.Scatter(x=Cd_pl, y=Cl_pl, mode='lines', line=dict(color=color, width=2), name=f"{lbl} Polar", hovertemplate="Cd: %{x:.4f}<br>Cl: %{y:.3f}<extra></extra>"), row=2, col=2)

    fig3.update_layout(**get_base_layout(), height=750)
    fig3.update_xaxes(title_text="α (deg)", row=1, col=1)
    fig3.update_xaxes(title_text="α (deg)", row=1, col=2)
    fig3.update_xaxes(title_text="α (deg)", row=2, col=1)
    fig3.update_xaxes(title_text="Cd", row=2, col=2)
    st.plotly_chart(fig3, use_container_width=True)


# ═══════════════════════════════════════════════════════════════════════════════
# DOWNLOAD SECTION
# ═══════════════════════════════════════════════════════════════════════════════
st.divider()
st.subheader("📥 Export Results")
df_export = pd.DataFrame({
    "r_over_R": r_R, "r_m": r_R * r_snap.radius_m, "chord_m": chord_arr, "twist_deg": twist_arr,
    "AoA_deg": alpha_arr, "Cl": Cl_arr, "Cd": Cd_arr, "dT_dr_N_m": dT_arr, "dQ_dr_Nm_m": dQ_arr,
    "v_induced_m_s": inflow_arr, "mach": mach_arr, "tip_loss_F": F_arr, "stalled": stall_arr.astype(int),
})
st.download_button(label="⬇️  Download Element Data (CSV)", data=df_export.to_csv(index=False).encode(), file_name="bemt_element_results.csv", mime="text/csv", use_container_width=True)
with st.expander("Preview Element Data Table"):
    st.dataframe(df_export.style.format(precision=5), use_container_width=True)
