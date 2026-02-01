#!/usr/bin/env python3
"""
24-Hour Smartphone Battery Coupled Model Simulation
24小时智能手机电池耦合模型仿真

Creates a comprehensive visualization showing SOC, current, temperature,
brightness, and signal strength over a typical 24-hour usage cycle.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery_model import BatteryCoupledModel, BatteryParameters
from battery_model.coupled_equations import SystemState

# Set up plotting style
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 16,
    'legend.fontsize': 9,
    'figure.dpi': 150,
})


def create_24hour_scenario():
    """
    Create a realistic 24-hour usage scenario with different activities
    创建真实的24小时使用场景
    """
    # Time in hours
    hours = np.linspace(0, 24, 1441)  # 1 minute resolution
    
    # Initialize arrays
    n = len(hours)
    activity = np.zeros(n, dtype=int)  # Activity index
    cpu_load = np.zeros(n)
    brightness = np.zeros(n)
    screen_on = np.zeros(n)
    data_rate = np.zeros(n)
    gps_active = np.zeros(n)
    signal_strength = np.zeros(n)
    charging = np.zeros(n)
    
    # Activity labels: 0=Sleep, 1=Charging, 2=Commute, 3=Office, 4=Entertainment, 5=Light Use
    activity_names = ['Sleep', 'Charging', 'Commute', 'Office', 'Entertainment', 'Light Use']
    
    for i, h in enumerate(hours):
        # 0:00 - 2:00 Sleep
        if 0 <= h < 2:
            activity[i] = 0
            cpu_load[i] = 0.02
            brightness[i] = 0
            screen_on[i] = 0
            data_rate[i] = 0.1
            signal_strength[i] = -70
            
        # 2:00 - 3:00 Charging (overnight)
        elif 2 <= h < 3:
            activity[i] = 1
            cpu_load[i] = 0.05
            brightness[i] = 0
            screen_on[i] = 0
            data_rate[i] = 0.5
            signal_strength[i] = -70
            charging[i] = 1
            
        # 3:00 - 7:00 Sleep
        elif 3 <= h < 7:
            activity[i] = 0
            cpu_load[i] = 0.02
            brightness[i] = 0
            screen_on[i] = 0
            data_rate[i] = 0.1
            signal_strength[i] = -70
            
        # 7:00 - 8:30 Commute (morning)
        elif 7 <= h < 8.5:
            activity[i] = 2
            cpu_load[i] = 0.4
            brightness[i] = 60
            screen_on[i] = 0.7
            data_rate[i] = 15
            gps_active[i] = 1
            signal_strength[i] = -85 + 10 * np.sin(h * 2)  # Varying signal
            
        # 8:30 - 10:00 Office (morning work)
        elif 8.5 <= h < 10:
            activity[i] = 3
            cpu_load[i] = 0.3
            brightness[i] = 50
            screen_on[i] = 0.5
            data_rate[i] = 5
            signal_strength[i] = -75
            
        # 10:00 - 12:00 Office (meetings, light use)
        elif 10 <= h < 12:
            activity[i] = 3
            cpu_load[i] = 0.25
            brightness[i] = 50
            screen_on[i] = 0.3
            data_rate[i] = 3
            signal_strength[i] = -75
            
        # 12:00 - 13:30 Entertainment (lunch break - video/social)
        elif 12 <= h < 13.5:
            activity[i] = 4
            cpu_load[i] = 0.6
            brightness[i] = 70
            screen_on[i] = 1
            data_rate[i] = 25
            signal_strength[i] = -75
            
        # 13:30 - 14:00 Charging (quick charge at lunch)
        elif 13.5 <= h < 14:
            activity[i] = 1
            cpu_load[i] = 0.2
            brightness[i] = 40
            screen_on[i] = 0.3
            data_rate[i] = 2
            signal_strength[i] = -75
            charging[i] = 1
            
        # 14:00 - 17:30 Office (afternoon work)
        elif 14 <= h < 17.5:
            activity[i] = 3
            cpu_load[i] = 0.35 + 0.1 * np.sin(h)
            brightness[i] = 50
            screen_on[i] = 0.5
            data_rate[i] = 8
            signal_strength[i] = -75
            
        # 17:30 - 18:30 Commute (evening)
        elif 17.5 <= h < 18.5:
            activity[i] = 2
            cpu_load[i] = 0.45
            brightness[i] = 65
            screen_on[i] = 0.8
            data_rate[i] = 20
            gps_active[i] = 1
            signal_strength[i] = -90 + 15 * np.sin(h * 3)
            
        # 18:30 - 21:00 Entertainment (evening - gaming/video)
        elif 18.5 <= h < 21:
            activity[i] = 4
            cpu_load[i] = 0.7 + 0.2 * np.sin(h * 2)
            brightness[i] = 75
            screen_on[i] = 1
            data_rate[i] = 30
            signal_strength[i] = -70
            
        # 21:00 - 22:00 Light use (winding down)
        elif 21 <= h < 22:
            activity[i] = 5
            cpu_load[i] = 0.25
            brightness[i] = 40
            screen_on[i] = 0.6
            data_rate[i] = 5
            signal_strength[i] = -70
            
        # 22:00 - 23:00 Charging (evening charge)
        elif 22 <= h < 23:
            activity[i] = 1
            cpu_load[i] = 0.15
            brightness[i] = 30
            screen_on[i] = 0.3
            data_rate[i] = 2
            signal_strength[i] = -70
            charging[i] = 1
            
        # 23:00 - 24:00 Sleep
        else:
            activity[i] = 0
            cpu_load[i] = 0.02
            brightness[i] = 0
            screen_on[i] = 0
            data_rate[i] = 0.1
            signal_strength[i] = -70
    
    return {
        'hours': hours,
        'activity': activity,
        'activity_names': activity_names,
        'cpu_load': cpu_load,
        'brightness': brightness,
        'screen_on': screen_on,
        'data_rate': data_rate,
        'gps_active': gps_active,
        'signal_strength': signal_strength,
        'charging': charging
    }


def simulate_24hour_battery(scenario):
    """
    Simulate battery behavior over 24 hours based on scenario
    根据场景模拟24小时电池行为
    """
    hours = scenario['hours']
    n = len(hours)
    
    # Initialize battery model
    battery_params = BatteryParameters()
    model = BatteryCoupledModel(battery_params=battery_params)
    
    # Initialize state
    SOC = np.zeros(n)
    SOC[0] = 0.85  # Start at 85%
    
    temperature = np.zeros(n)
    temperature[0] = 25.0  # 25°C
    
    current = np.zeros(n)
    
    # Time step in seconds
    dt = (hours[1] - hours[0]) * 3600  # Convert hours to seconds
    
    # Simulation parameters
    Q_max = battery_params.Q_max_As  # Capacity in As
    V_nom = battery_params.V_nom
    
    for i in range(1, n):
        # Get scenario parameters
        cpu = scenario['cpu_load'][i]
        bright = scenario['brightness'][i] / 100
        screen = scenario['screen_on'][i]
        data = scenario['data_rate'][i]
        gps = scenario['gps_active'][i]
        charging = scenario['charging'][i]
        
        # Calculate power consumption
        P_cpu = cpu * 2.5  # CPU power (0-2.5W)
        P_display = screen * bright * 2.0  # Display (0-2W)
        P_network = 0.1 + data * 0.05  # Network (0.1-1.6W)
        P_gps = gps * 0.15  # GPS
        P_background = 0.1 + 0.05 * np.random.random()  # Background
        
        P_total = P_cpu + P_display + P_network + P_gps + P_background
        
        if charging:
            # Charging current (depends on SOC)
            if SOC[i-1] < 0.8:
                I_charge = -2.5  # Fast charge
            elif SOC[i-1] < 0.95:
                I_charge = -1.5  # Taper
            else:
                I_charge = -0.3  # Trickle
            current[i] = I_charge
        else:
            # Discharge current
            V_batt = battery_params.V_OCV(np.array([SOC[i-1]]))[0]
            I_discharge = P_total / V_batt
            current[i] = I_discharge
        
        # Update SOC
        dSOC = -current[i] * dt / Q_max
        SOC[i] = np.clip(SOC[i-1] + dSOC, 0.05, 1.0)
        
        # Update temperature
        # Heat generation
        R_int = battery_params.R_int(np.array([SOC[i]]), temperature[i-1] + 273.15)[0]
        P_heat = current[i]**2 * R_int + P_cpu * 0.3  # Battery + CPU heat
        
        # Thermal dynamics
        T_env = 22.0 if scenario['activity'][i] in [0, 3, 4, 5] else 25.0  # Indoor vs outdoor
        dT = (P_heat - (temperature[i-1] - T_env) / 5.0) * dt / 50.0
        temperature[i] = np.clip(temperature[i-1] + dT, T_env - 5, 45)
    
    return {
        'SOC': SOC * 100,  # Convert to percentage
        'current': current,
        'temperature': temperature
    }


def plot_24hour_simulation(scenario, simulation, save_path=None):
    """
    Create the comprehensive 24-hour simulation plot
    创建综合24小时仿真图
    """
    fig, ax1 = plt.subplots(figsize=(16, 9))
    
    hours = scenario['hours']
    
    # Activity colors with transparency
    activity_colors = {
        0: '#E8F4FD',   # Sleep - light blue
        1: '#FFF3E0',   # Charging - light orange
        2: '#E8F5E9',   # Commute - light green
        3: '#FCE4EC',   # Office - light pink
        4: '#FFF9C4',   # Entertainment - light yellow
        5: '#F3E5F5',   # Light Use - light purple
    }
    
    activity_names = {
        0: 'Sleep',
        1: 'Charging',
        2: 'Commute',
        3: 'Office',
        4: 'Entertainment',
        5: 'Light Use'
    }
    
    # Draw activity background regions
    current_activity = scenario['activity'][0]
    start_hour = 0
    
    for i, (h, act) in enumerate(zip(hours, scenario['activity'])):
        if act != current_activity or i == len(hours) - 1:
            # Draw rectangle for previous activity
            width = h - start_hour if i < len(hours) - 1 else 24 - start_hour
            rect = Rectangle((start_hour, 0), width, 100, 
                            facecolor=activity_colors[current_activity], 
                            edgecolor='none', alpha=0.7, zorder=0)
            ax1.add_patch(rect)
            
            # Add activity label at top
            mid_x = start_hour + width / 2
            ax1.text(mid_x, 97, activity_names[current_activity], 
                    ha='center', va='top', fontsize=10, fontweight='bold',
                    color='#444444', zorder=10)
            
            current_activity = act
            start_hour = h
    
    # Primary y-axis: SOC and Current
    ax1.set_xlabel('Time (hours)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('SOC (%) / Current (A)', fontsize=12, color='#E65100', fontweight='bold')
    
    # Plot SOC
    line_soc, = ax1.plot(hours, simulation['SOC'], color='#FF9800', linewidth=3, 
                         label='SOC [%]', zorder=5)
    
    # Plot Current (scaled for visibility)
    current_scaled = simulation['current'] * 30  # Scale for visibility
    line_current, = ax1.plot(hours, current_scaled, color='#2196F3', linewidth=2, 
                             label='Current I [A×30]', zorder=5)
    
    # Plot Temperature
    line_temp, = ax1.plot(hours, simulation['temperature'], color='#9C27B0', linewidth=2,
                          label='Temperature T [°C]', zorder=5)
    
    ax1.set_ylim(-5, 105)
    ax1.set_xlim(0, 24)
    ax1.tick_params(axis='y', labelcolor='#E65100')
    
    # Secondary y-axis: Brightness and Signal
    ax2 = ax1.twinx()
    ax2.set_ylabel('Temperature (°C) / Brightness (%) / Signal [dBm]', 
                   fontsize=12, color='#1B5E20', fontweight='bold')
    
    # Plot Brightness (step function)
    line_bright, = ax2.step(hours, scenario['brightness'], where='post',
                            color='#8BC34A', linewidth=2, label='Brightness [%]', zorder=4)
    
    # Plot Signal Strength (shifted for visibility)
    signal_shifted = scenario['signal_strength'] + 120  # Shift to positive range
    line_signal, = ax2.step(hours, signal_shifted, where='post',
                            color='#4CAF50', linewidth=1.5, label='Signal Strength [dBm]', 
                            linestyle='-', zorder=4)
    
    ax2.set_ylim(0, 120)
    ax2.tick_params(axis='y', labelcolor='#1B5E20')
    
    # Mark charging periods
    charging_starts = []
    in_charging = False
    for i, (h, ch) in enumerate(zip(hours, scenario['charging'])):
        if ch and not in_charging:
            charging_starts.append(h)
            in_charging = True
        elif not ch and in_charging:
            in_charging = False
    
    for start in charging_starts:
        ax1.annotate('Charging', xy=(start + 0.2, 15), fontsize=9, 
                    color='#388E3C', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#C8E6C9', 
                             edgecolor='#4CAF50', linewidth=1))
    
    # Title
    ax1.set_title('24-Hour Smartphone Battery Coupled Model Simulation', 
                  fontsize=18, fontweight='bold', pad=20)
    
    # X-axis formatting
    ax1.set_xticks(range(0, 25, 3))
    ax1.set_xticklabels([f'{h}:00' for h in range(0, 25, 3)])
    ax1.grid(True, alpha=0.3, linestyle='--', zorder=1)
    
    # Create legend
    lines = [line_soc, line_current, line_temp, line_bright, line_signal]
    labels = ['SOC [%]', 'Current I [A×30]', 'Temperature T [°C]', 
              'Brightness [%]', 'Signal Strength [dBm]']
    
    legend = ax1.legend(lines, labels, loc='upper right', 
                       bbox_to_anchor=(0.99, 0.98), framealpha=0.95,
                       fontsize=10, ncol=2)
    legend.get_frame().set_edgecolor('#CCCCCC')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")
    
    return fig


def main():
    """Main execution"""
    print("="*60)
    print("24-Hour Smartphone Battery Coupled Model Simulation")
    print("="*60)
    
    # Create scenario
    print("\nCreating 24-hour usage scenario...")
    scenario = create_24hour_scenario()
    
    # Run simulation
    print("Running battery simulation...")
    simulation = simulate_24hour_battery(scenario)
    
    # Print statistics
    print("\n--- Simulation Results ---")
    print(f"Starting SOC: {simulation['SOC'][0]:.1f}%")
    print(f"Minimum SOC: {np.min(simulation['SOC']):.1f}%")
    print(f"Maximum SOC: {np.max(simulation['SOC']):.1f}%")
    print(f"Final SOC: {simulation['SOC'][-1]:.1f}%")
    print(f"Max Temperature: {np.max(simulation['temperature']):.1f}°C")
    print(f"Max Discharge Current: {np.max(simulation['current']):.2f}A")
    print(f"Max Charge Current: {np.min(simulation['current']):.2f}A")
    
    # Create plot
    print("\nGenerating visualization...")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, '24hour_battery_simulation.png')
    fig = plot_24hour_simulation(scenario, simulation, save_path)
    
    plt.close(fig)
    print("\nDone!")
    
    return scenario, simulation


if __name__ == "__main__":
    scenario, simulation = main()
