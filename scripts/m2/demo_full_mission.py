import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from rotor import Rotor, linear_taper_chord, linear_twist
from airfoil import BlendedLinearAirfoilProvider, LinearAirfoil
from environment import isa
from m2.aircraft_input_m2 import get_default_aircraft
from m2.mission_v2 import run_tiltrotor_mission

def main():
    aircraft = get_default_aircraft()
    
    # Standard metrics from previous reports
    R = 3.8
    rotor = Rotor(
        radius_m=R,
        root_cutout_m=0.5,
        num_blades=3,
        chord_fn=linear_taper_chord(0.90, 0.3888),
        twist_fn=linear_twist(np.radians(25), np.radians(-45))
    )
    
    airfoil_provider = BlendedLinearAirfoilProvider(
        stations=[0.0, 1.0],
        airfoils=[LinearAirfoil(), LinearAirfoil()]
    )
    
    atmo = isa(2000) # Fly at 2000m altitude
    
    # Mission parameters
    cruise_distance_m = 400 * 1000 # 400 km
    cruise_velocity_m_s = 90.0     # ~175 knots
    
    hover_omega = 500 * (2 * np.pi / 60)
    cruise_omega = 400 * (2 * np.pi / 60) # drop RPM in airplane mode to increase prop efficiency
    
    print(f"--- M2 Full Mission Simulation ---")
    print(f"Aircraft Weight: {aircraft.W_MTOW_N/9.81:.1f} kg")
    print(f"Altitude: 2000 m")
    print(f"Distance: {cruise_distance_m/1000:.1f} km")
    print(f"Cruise Speed: {cruise_velocity_m_s:.1f} m/s")
    print(f"Hover RPM: 500  |  Cruise RPM: 400")
    print("-" * 34)
    
    segments = run_tiltrotor_mission(
        aircraft=aircraft,
        rotor=rotor,
        airfoil_provider=airfoil_provider,
        rho=atmo.density_kg_m3,
        a_sound=atmo.speed_of_sound_mps,
        hover_omega_rad_s=hover_omega,
        cruise_omega_rad_s=cruise_omega,
        cruise_distance_m=cruise_distance_m,
        cruise_velocity_m_s=cruise_velocity_m_s
    )
    
    total_time_s = sum(s.duration_s for s in segments)
    total_dist_m = sum(s.distance_m for s in segments)
    total_fuel_kg = sum(s.fuel_burn_kg for s in segments)
    
    print("\n--- Mission Results ---")
    print(f"{'Segment Name':<15} | {'Duration (s)':>12} | {'Avg Power (kW)':>14} | {'Fuel Burn (kg)':>14}")
    print("-" * 65)
    
    names = []
    fuels = []
    
    for s in segments:
        print(f"{s.name:<15} | {s.duration_s:12.1f} | {s.avg_power_kW:14.1f} | {s.fuel_burn_kg:14.1f}")
        names.append(s.name)
        fuels.append(s.fuel_burn_kg)
        
    print("-" * 65)
    print(f"{'TOTAL':<15} | {total_time_s:12.1f} | {'':>14} | {total_fuel_kg:14.1f} kg")
    
    # Optional: Plot the fuel breakdown
    os.makedirs(os.path.join(os.path.dirname(__file__), '../../outputs/m2'), exist_ok=True)
    
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(names, fuels, color='tab:orange')
    ax.set_ylabel('Fuel Burned (kg)')
    ax.set_title('Mission Fuel Breakdown by Segment')
    ax.grid(axis='y', linestyle='--')
    
    for i, v in enumerate(fuels):
        ax.text(i, v + (max(fuels)*0.02), f"{v:.1f}", ha='center')
        
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), '../../outputs/m2/mission_fuel.png')
    plt.savefig(out_path, dpi=300)
    print(f"\nSaved mission plot to {out_path}")

if __name__ == "__main__":
    main()
