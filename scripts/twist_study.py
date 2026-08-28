import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt

from environment import isa
from airfoil import LinearAirfoil
from rotor import Rotor, constant_chord, linear_twist
from bemt import run_bemt

airfoil = LinearAirfoil()
atmo = isa(0.0)
OMEGA = 2 * np.pi * 1000.0 / 60.0
R = 0.762
rc = 0.125
B = 2
c = 0.0508 # Constant chord

def make_twisted_rotor(twist_rate_deg):
    # To keep the pitch at 75% radius constant across different twists for a given
    # collective setting, we offset the root twist.
    # theta(r) = theta_root + twist_rate * (r/R).
    # We want theta(0.75R) = 0 before collective is added.
    # 0 = theta_root + twist_rate_rad * 0.75  => theta_root = -0.75 * twist_rate_rad
    twist_rate_rad = np.radians(twist_rate_deg)
    theta_root = -0.75 * twist_rate_rad
    
    return Rotor(R, rc, B,
                 constant_chord(c),
                 linear_twist(theta_root, twist_rate_rad))

def main():
    # Use 5 extreme twist values to make the aerodynamic differences highly visible:
    # +15 (Washin), 0 (Untwisted), -15 (Moderate Washout), -30 (High Washout), -45 (Extreme Washout)
    twist_rates = [15.0, 0.0, -15.0, -30.0, -45.0]
    collectives = np.linspace(2, 16, 15)
    
    outdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(outdir, exist_ok=True)
    
    fig1, ax1 = plt.subplots(figsize=(6, 4))
    fig2, ax2 = plt.subplots(figsize=(6, 4))
    fig3, ax3 = plt.subplots(figsize=(6, 4))
    
    for tw in twist_rates:
        rotor = make_twisted_rotor(tw)
        
        T, P, FM, Stall = [], [], [], []
        for col in collectives:
            perf = run_bemt(rotor, lambda x: airfoil, OMEGA, np.radians(col),
                            atmo.density_kg_m3, atmo.speed_of_sound_mps)
            T.append(perf.thrust_N)
            P.append(perf.power_W / 1000.0) # kW
            FM.append(perf.figure_of_merit if perf.figure_of_merit else 0)
            Stall.append(perf.stalled_fraction)
            
        label = f"Twist = {tw:.0f}$^\circ$/R"
        ax1.plot(collectives, T, lw=2, label=label)
        ax2.plot(collectives, P, lw=2, label=label)
        ax3.plot(collectives, FM, lw=2, label=label)
        
    ax1.set_xlabel("Collective (at 75% R) [deg]")
    ax1.set_ylabel("Thrust [N]")
    ax1.set_title("Thrust vs Collective for Varying Twist")
    ax1.grid(True)
    ax1.legend()
    path1 = os.path.join(outdir, "twist_study_thrust.png")
    fig1.tight_layout()
    fig1.savefig(path1, dpi=150)
    plt.close(fig1)
    
    ax2.set_xlabel("Collective (at 75% R) [deg]")
    ax2.set_ylabel("Power [kW]")
    ax2.set_title("Power vs Collective for Varying Twist")
    ax2.grid(True)
    ax2.legend()
    path2 = os.path.join(outdir, "twist_study_power.png")
    fig2.tight_layout()
    fig2.savefig(path2, dpi=150)
    plt.close(fig2)
    
    ax3.set_xlabel("Collective (at 75% R) [deg]")
    ax3.set_ylabel("Figure of Merit")
    ax3.set_title("Efficiency (FM) vs Collective for Varying Twist")
    ax3.grid(True)
    ax3.legend()
    path3 = os.path.join(outdir, "twist_study_efficiency.png")
    fig3.tight_layout()
    fig3.savefig(path3, dpi=150)
    plt.close(fig3)

    print(f"Saved: {path1}")
    print(f"Saved: {path2}")
    print(f"Saved: {path3}")

if __name__ == '__main__':
    main()
