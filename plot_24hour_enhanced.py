#!/usr/bin/env python3
"""
Enhanced 24-Hour Battery Simulation Plot
增强版24小时电池仿真图

Creates a visualization closely matching the reference style with:
- Colored activity regions
- Multiple overlaid variables
- Dual y-axes
- Charging annotations
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery_model import BatteryParameters

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 16,
    'legend.fontsize': 9,
    'figure.dpi': 150,
})


def create_detailed_scenario():
    """Create detailed 24-hour scenario matching the reference image"""
    # Time points in hours (minute resolution)
    hours = np.linspace(0, 24, 1441)
    n = len(hours)
    
    # Initialize all arrays
    activity = np.zeros(n, dtype=int)  # 0=Sleep, 1=Charging, 2=Commute, 3=Office, 4=Entertainment
    SOC = np.zeros(n)
    current = np.zeros(n)
    temperature = np.zeros(n)
    brightness = np.zeros(n)
    signal_strength = np.zeros(n)
    charging = np.zeros(n, dtype=bool)
    
    # Initial conditions
    SOC[0] = 85
    temperature[0] = 20
    
    # Battery parameters
    Q_max = 14400  # As (4000 mAh)
    
    for i, h in enumerate(hours[1:], 1):
        # Determine activity and parameters based on time
        
        # 0:00 - 2:00 Sleep
        if 0 <= h < 2:
            activity[i] = 0
            P_total = 0.15
            brightness[i] = 0
            signal_strength[i] = -70
            
        # 2:00 - 3:00 Charging (overnight)
        elif 2 <= h < 3:
            activity[i] = 1
            P_total = 0.2
            brightness[i] = 0
            signal_strength[i] = -70
            charging[i] = True
            
        # 3:00 - 7:00 Sleep
        elif 3 <= h < 7:
            activity[i] = 0
            P_total = 0.12
            brightness[i] = 0
            signal_strength[i] = -70
            
        # 7:00 - 8:30 Commute
        elif 7 <= h < 8.5:
            activity[i] = 2
            P_total = 2.5 + 0.5 * np.sin((h - 7) * 4)
            brightness[i] = 60
            signal_strength[i] = -85 + 15 * np.sin((h - 7) * 5)
            
        # 8:30 - 10:00 Office
        elif 8.5 <= h < 10:
            activity[i] = 3
            P_total = 1.5
            brightness[i] = 50
            signal_strength[i] = -75
            
        # 10:00 - 12:00 Office (meetings)
        elif 10 <= h < 12:
            activity[i] = 3
            P_total = 1.2
            brightness[i] = 50
            signal_strength[i] = -75
            
        # 12:00 - 13:30 Entertainment (lunch)
        elif 12 <= h < 13.5:
            activity[i] = 4
            P_total = 3.5 + 0.5 * np.sin((h - 12) * 6)
            brightness[i] = 75
            signal_strength[i] = -75
            
        # 13:30 - 14:00 Charging
        elif 13.5 <= h < 14:
            activity[i] = 1
            P_total = 0.5
            brightness[i] = 40
            signal_strength[i] = -75
            charging[i] = True
            
        # 14:00 - 17:30 Office
        elif 14 <= h < 17.5:
            activity[i] = 3
            P_total = 1.8 + 0.3 * np.sin((h - 14) * 2)
            brightness[i] = 50
            signal_strength[i] = -75
            
        # 17:30 - 18:30 Commute (evening)
        elif 17.5 <= h < 18.5:
            activity[i] = 2
            P_total = 2.8 + 0.4 * np.sin((h - 17.5) * 5)
            brightness[i] = 65
            signal_strength[i] = -90 + 20 * np.sin((h - 17.5) * 6)
            
        # 18:30 - 21:00 Entertainment (evening)
        elif 18.5 <= h < 21:
            activity[i] = 4
            P_total = 4.0 + 0.5 * np.sin((h - 18.5) * 2)
            brightness[i] = 75
            signal_strength[i] = -70
            
        # 21:00 - 22:00 Light use
        elif 21 <= h < 22:
            activity[i] = 3
            P_total = 1.5
            brightness[i] = 40
            signal_strength[i] = -70
            
        # 22:00 - 23:00 Charging
        elif 22 <= h < 23:
            activity[i] = 1
            P_total = 0.3
            brightness[i] = 30
            signal_strength[i] = -70
            charging[i] = True
            
        # 23:00 - 24:00 Sleep
        else:
            activity[i] = 0
            P_total = 0.1
            brightness[i] = 0
            signal_strength[i] = -70
        
        # Calculate current
        V_batt = 3.7 + 0.5 * (SOC[i-1] / 100)
        
        if charging[i]:
            # Charging
            if SOC[i-1] < 80:
                I_charge = -2.5  # Fast charge
            elif SOC[i-1] < 95:
                I_charge = -1.2  # Taper
            else:
                I_charge = -0.2  # Trickle
            current[i] = I_charge
        else:
            # Discharge
            current[i] = P_total / V_batt
        
        # Update SOC
        dt = (hours[1] - hours[0]) * 3600  # seconds
        dSOC = -current[i] * dt / Q_max * 100
        SOC[i] = np.clip(SOC[i-1] + dSOC, 5, 100)
        
        # Update temperature
        P_heat = abs(current[i])**2 * 0.05 + P_total * 0.2
        T_env = 20 if activity[i] in [0, 1, 3, 4] else 25
        dT = (P_heat * 3 - (temperature[i-1] - T_env) * 0.15) * dt / 50
        temperature[i] = np.clip(temperature[i-1] + dT, T_env - 2, 40)
    
    return {
        'hours': hours,
        'activity': activity,
        'SOC': SOC,
        'current': current,
        'temperature': temperature,
        'brightness': brightness,
        'signal_strength': signal_strength,
        'charging': charging
    }


def plot_enhanced_simulation(data, save_path=None):
    """Create the enhanced 24-hour plot"""
    
    fig, ax1 = plt.subplots(figsize=(18, 10))
    
    hours = data['hours']
    
    # Activity colors (matching reference)
    activity_colors = {
        0: '#D6EAF8',   # Sleep - light blue
        1: '#FDEBD0',   # Charging - light orange/yellow  
        2: '#D5F5E3',   # Commute - light green
        3: '#FADBD8',   # Office - light pink/red
        4: '#FEF9E7',   # Entertainment - light yellow
    }
    
    activity_names = {
        0: 'Sleep',
        1: 'Charging',
        2: 'Commute',
        3: 'Office',
        4: 'Entertainment',
    }
    
    # Draw activity background regions
    current_activity = data['activity'][0]
    start_idx = 0
    
    for i in range(len(hours)):
        if i == len(hours) - 1 or data['activity'][i] != current_activity:
            end_hour = hours[i] if i < len(hours) - 1 else 24
            start_hour = hours[start_idx]
            width = end_hour - start_hour
            
            if width > 0:
                rect = Rectangle((start_hour, 0), width, 100,
                                facecolor=activity_colors[current_activity],
                                edgecolor='none', alpha=0.8, zorder=0)
                ax1.add_patch(rect)
                
                # Activity label at top
                mid_x = start_hour + width / 2
                ax1.text(mid_x, 98, activity_names[current_activity],
                        ha='center', va='top', fontsize=11, fontweight='bold',
                        color='#2C3E50', zorder=15)
            
            if i < len(hours) - 1:
                current_activity = data['activity'][i]
                start_idx = i
    
    # Primary Y-axis labels
    ax1.set_xlabel('Time (hours)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('SOC (%) / Current (A)', fontsize=13, color='#D35400', fontweight='bold')
    
    # Plot SOC (thick orange line)
    line_soc, = ax1.plot(hours, data['SOC'], color='#E67E22', linewidth=3.5,
                         label='SOC [%]', zorder=10)
    
    # Plot Current (scaled, blue line)
    current_scaled = data['current'] * 30
    line_current, = ax1.plot(hours, current_scaled, color='#3498DB', linewidth=2,
                             label='Current I [A×30]', zorder=9)
    
    # Plot Temperature (purple line)
    line_temp, = ax1.plot(hours, data['temperature'], color='#8E44AD', linewidth=2.5,
                          label='Temperature T [°C]', zorder=9)
    
    ax1.set_ylim(-10, 105)
    ax1.set_xlim(0, 24)
    ax1.tick_params(axis='y', labelcolor='#D35400', labelsize=11)
    
    # Secondary Y-axis
    ax2 = ax1.twinx()
    ax2.set_ylabel('Temperature (°C) / Brightness (%) / Signal [dBm]',
                   fontsize=13, color='#27AE60', fontweight='bold')
    
    # Plot Brightness (olive/yellow-green step)
    line_bright, = ax2.step(hours, data['brightness'], where='post',
                            color='#9ACD32', linewidth=2.5, 
                            label='Brightness [%]', zorder=8)
    
    # Plot Signal Strength (green step, shifted)
    signal_shifted = data['signal_strength'] + 120
    line_signal, = ax2.step(hours, signal_shifted, where='post',
                            color='#2ECC71', linewidth=2,
                            label='Signal Strength [dBm]', zorder=8)
    
    ax2.set_ylim(0, 120)
    ax2.tick_params(axis='y', labelcolor='#27AE60', labelsize=11)
    
    # Mark charging periods with annotations
    charging_regions = []
    in_charge = False
    start_charge = 0
    
    for i, (h, ch) in enumerate(zip(hours, data['charging'])):
        if ch and not in_charge:
            start_charge = h
            in_charge = True
        elif not ch and in_charge:
            charging_regions.append((start_charge, h))
            in_charge = False
    
    if in_charge:
        charging_regions.append((start_charge, 24))
    
    for start, end in charging_regions:
        mid = (start + end) / 2
        ax1.annotate('Charging', xy=(mid, 12),
                    ha='center', va='center', fontsize=10,
                    fontweight='bold', color='#27AE60',
                    bbox=dict(boxstyle='round,pad=0.4', 
                             facecolor='#ABEBC6', edgecolor='#27AE60',
                             linewidth=1.5, alpha=0.9),
                    zorder=20)
    
    # Title
    ax1.set_title('24-Hour Smartphone Battery Coupled Model Simulation',
                  fontsize=20, fontweight='bold', pad=25, color='#2C3E50')
    
    # X-axis formatting
    ax1.set_xticks(range(0, 25, 3))
    ax1.set_xticklabels([f'{h}:00' for h in range(0, 25, 3)], fontsize=11)
    
    # Grid
    ax1.grid(True, alpha=0.4, linestyle='-', linewidth=0.5, zorder=1)
    ax1.set_axisbelow(True)
    
    # Legend
    lines = [line_soc, line_current, line_temp, line_bright, line_signal]
    labels = ['SOC [%]', 'Current I [A×30]', 'Temperature T [°C]',
              'Brightness [%]', 'Signal Strength [dBm]']
    
    # Create custom legend with colored markers
    legend_elements = [
        Line2D([0], [0], color='#E67E22', linewidth=3.5, label='SOC [%]'),
        Line2D([0], [0], color='#3498DB', linewidth=2, label='Current I [A×30]'),
        Line2D([0], [0], color='#8E44AD', linewidth=2.5, label='Temperature T [°C]'),
        Line2D([0], [0], color='#9ACD32', linewidth=2.5, label='Brightness [%]'),
        Line2D([0], [0], color='#2ECC71', linewidth=2, label='Signal Strength [dBm]'),
    ]
    
    legend = ax1.legend(handles=legend_elements, loc='upper right',
                       bbox_to_anchor=(0.995, 0.97), framealpha=0.95,
                       fontsize=10, ncol=2, columnspacing=1.5)
    legend.get_frame().set_edgecolor('#BDC3C7')
    legend.get_frame().set_linewidth(1.5)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        print(f"Enhanced figure saved to: {save_path}")
    
    return fig


def main():
    """Main execution"""
    print("="*65)
    print("Enhanced 24-Hour Smartphone Battery Coupled Model Simulation")
    print("="*65)
    
    # Create scenario
    print("\nCreating detailed 24-hour usage scenario...")
    data = create_detailed_scenario()
    
    # Print statistics
    print("\n--- Simulation Statistics ---")
    print(f"Initial SOC: {data['SOC'][0]:.1f}%")
    print(f"Minimum SOC: {np.min(data['SOC']):.1f}%")
    print(f"Maximum SOC: {np.max(data['SOC']):.1f}%")
    print(f"Final SOC: {data['SOC'][-1]:.1f}%")
    print(f"Max Temperature: {np.max(data['temperature']):.1f}°C")
    print(f"Min Temperature: {np.min(data['temperature']):.1f}°C")
    print(f"Max Discharge Current: {np.max(data['current']):.2f}A")
    print(f"Max Charge Current: {np.min(data['current']):.2f}A")
    
    # Total charging periods
    charge_time = np.sum(data['charging']) * (24 / len(data['hours'])) * 60
    print(f"Total Charging Time: {charge_time:.0f} minutes")
    
    # Create plot
    print("\nGenerating enhanced visualization...")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, '24hour_battery_simulation_enhanced.png')
    fig = plot_enhanced_simulation(data, save_path)
    
    plt.close(fig)
    print("\nSimulation complete!")
    
    return data


if __name__ == "__main__":
    data = main()
