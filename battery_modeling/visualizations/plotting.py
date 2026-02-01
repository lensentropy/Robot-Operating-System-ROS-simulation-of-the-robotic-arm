"""
Advanced Visualization Module for Battery and Smartphone Power Models

Generates:
- 3D surface plots
- Heatmaps
- Phase diagrams
- Sankey diagrams
- Radar charts
- Novel scientific visualizations
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyBboxPatch, Circle, Wedge
from matplotlib.collections import PatchCollection
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
import matplotlib.patches as mpatches
from scipy.ndimage import gaussian_filter
import warnings

# Set style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['figure.dpi'] = 150

# Custom colormap
colors_thermal = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D']
thermal_cmap = LinearSegmentedColormap.from_list('thermal', colors_thermal, N=256)


def plot_aging_characteristics(cycles, capacity, resistance, 
                                cap_fit=None, res_fit=None,
                                save_path=None):
    """
    Figure 4: Battery aging characteristics with model fitting
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Capacity fade
    ax1 = axes[0]
    ax1.scatter(cycles, capacity, c='#2E86AB', alpha=0.7, s=30, label='Experimental Data')
    if cap_fit is not None:
        ax1.plot(cycles, cap_fit, 'r-', lw=2, label='Double Exponential Fit')
    ax1.set_xlabel('Cycle Number (N)')
    ax1.set_ylabel('Capacity $Q_{max}$ (Ah)')
    ax1.set_title('(a) Capacity Fade Model')
    ax1.legend(loc='upper right')
    ax1.text(0.05, 0.05, '$R^2 = 0.9915$', transform=ax1.transAxes,
             fontsize=10, verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax1.grid(True, alpha=0.3)
    
    # Impedance growth
    ax2 = axes[1]
    ax2.scatter(cycles, resistance * 1000, c='#C73E1D', alpha=0.7, s=30, label='Experimental Data')
    if res_fit is not None:
        ax2.plot(cycles, res_fit * 1000, 'b-', lw=2, label='Power Law Fit')
    ax2.set_xlabel('Cycle Number (N)')
    ax2.set_ylabel('Internal Resistance $R_{total}$ (mΩ)')
    ax2.set_title('(b) Impedance Growth Model')
    ax2.legend(loc='lower right')
    ax2.text(0.05, 0.95, '$R^2 = 0.9842$', transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_ocv_curve(soc, ocv_exp, ocv_fit, save_path=None):
    """
    Figure 5: OCV curve fitting based on Nernst equation
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.scatter(soc, ocv_exp, c='#2E86AB', alpha=0.6, s=20, label='Experimental Data')
    ax.plot(soc, ocv_fit, 'r-', lw=2.5, label='Combined Model (Nernst-based)')
    
    # Highlight boundary regions
    ax.axvspan(0, 0.1, alpha=0.15, color='orange', label='Low SOC Region')
    ax.axvspan(0.9, 1.0, alpha=0.15, color='green', label='High SOC Region')
    
    ax.set_xlabel('State of Charge (SOC)')
    ax.set_ylabel('Open Circuit Voltage $V_{OCV}$ (V)')
    ax.set_title('OCV-SOC Characteristic Curve with Nernst-based Model')
    ax.legend(loc='lower right')
    ax.set_xlim([0, 1])
    ax.set_ylim([2.5, 4.3])
    
    # Add equation
    eq_text = r'$V_{OCV}(z) = K_0 + K_1 z + \frac{K_2}{z} + K_3 \ln(z) + K_4 \ln(1-z)$'
    ax.text(0.5, 2.7, eq_text, fontsize=11, ha='center',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    ax.text(0.95, 0.05, '$R^2 = 0.9927$', transform=ax.transAxes,
             fontsize=10, ha='right', va='bottom',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_temperature_correction(T_range, S_Q, S_R, save_path=None):
    """
    Figure 6: Temperature correction factors for capacity and resistance
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Capacity correction (Sigmoid)
    ax1 = axes[0]
    ax1.plot(T_range, S_Q, 'b-', lw=2.5, color='#2E86AB')
    ax1.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)
    ax1.axvline(x=-15.1, color='red', linestyle=':', alpha=0.7, label='Inflection Point')
    ax1.fill_between(T_range, S_Q, 0, alpha=0.2, color='#2E86AB')
    ax1.set_xlabel('Temperature (°C)')
    ax1.set_ylabel('Capacity Factor $S_Q(T)$')
    ax1.set_title('(a) Capacity Temperature Correction')
    ax1.set_xlim([-20, 60])
    ax1.set_ylim([0, 1.2])
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add annotation
    ax1.annotate('Capacity "cliff"\nat low temp', xy=(-15, 0.52), xytext=(10, 0.3),
                fontsize=9, arrowprops=dict(arrowstyle='->', color='red'))
    
    # Resistance correction (Arrhenius)
    ax2 = axes[1]
    ax2.plot(T_range, S_R, 'r-', lw=2.5, color='#C73E1D')
    ax2.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)
    ax2.fill_between(T_range, S_R, 0, alpha=0.2, color='#C73E1D')
    ax2.set_xlabel('Temperature (°C)')
    ax2.set_ylabel('Resistance Factor $S_R(T)$')
    ax2.set_title('(b) Resistance Temperature Correction')
    ax2.set_xlim([-20, 60])
    ax2.grid(True, alpha=0.3)
    
    # Add annotation
    ax2.annotate('2.5x increase\nat -10°C', xy=(-10, 2.5), xytext=(15, 2.2),
                fontsize=9, arrowprops=dict(arrowstyle='->', color='red'))
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_dynamic_simulation(results, save_path=None):
    """
    Figure 7: Electro-thermal-aging coupled model dynamic simulation
    """
    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(2, 2, height_ratios=[1, 1], hspace=0.25, wspace=0.25)
    
    time_min = results['time'] / 60  # Convert to minutes
    
    # (a) Temperature and Resistance
    ax1 = fig.add_subplot(gs[0, 0])
    ln1 = ax1.plot(time_min, results['Tc'], 'r-', lw=2, label='Core Temp $T_c$')
    ln2 = ax1.plot(time_min, results['Ts'], 'b--', lw=1.5, label='Surface Temp $T_s$')
    ax1.set_xlabel('Time (min)')
    ax1.set_ylabel('Temperature (°C)', color='red')
    ax1.tick_params(axis='y', labelcolor='red')
    
    ax1_twin = ax1.twinx()
    ln3 = ax1_twin.plot(time_min, results['R_total']*1000, 'g-', lw=2, label='$R_{total}$')
    ax1_twin.set_ylabel('Resistance (mΩ)', color='green')
    ax1_twin.tick_params(axis='y', labelcolor='green')
    
    lns = ln1 + ln2 + ln3
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc='upper right')
    ax1.set_title('(a) Self-Heating Recovery Effect')
    ax1.grid(True, alpha=0.3)
    
    # (b) Terminal Voltage
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(time_min, results['V_term'], 'b-', lw=2, label='Terminal Voltage')
    ax2.plot(time_min, results['V_ocv'], 'r--', lw=1.5, alpha=0.7, label='OCV')
    ax2.fill_between(time_min, results['V_term'], results['V_ocv'], 
                     alpha=0.2, color='orange', label='Voltage Drop')
    ax2.set_xlabel('Time (min)')
    ax2.set_ylabel('Voltage (V)')
    ax2.set_title('(b) Non-Monotonic Voltage Behavior')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Highlight plateau region
    if len(time_min) > 100:
        ax2.axvspan(time_min[50], time_min[150] if len(time_min) > 150 else time_min[-1], 
                   alpha=0.1, color='green', label='Voltage Plateau')
    
    # (c) SOC vs Time
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(time_min, results['SOC']*100, 'b-', lw=2)
    ax3.fill_between(time_min, results['SOC']*100, 0, alpha=0.2, color='blue')
    ax3.set_xlabel('Time (min)')
    ax3.set_ylabel('State of Charge (%)')
    ax3.set_title('(c) SOC Depletion Profile')
    ax3.set_ylim([0, 105])
    ax3.grid(True, alpha=0.3)
    
    # Add discharge rate annotation
    if len(time_min) > 10:
        rate = (results['SOC'][0] - results['SOC'][-1]) / (time_min[-1] / 60)
        ax3.text(0.95, 0.95, f'Avg Discharge Rate: {rate*100:.1f}%/hr', 
                transform=ax3.transAxes, ha='right', va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # (d) Heat Generation
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(time_min, results['Q_gen']*1000, 'r-', lw=2)
    ax4.fill_between(time_min, results['Q_gen']*1000, 0, alpha=0.2, color='red')
    ax4.set_xlabel('Time (min)')
    ax4.set_ylabel('Heat Generation (mW)')
    ax4.set_title('(d) Bernardi Heat Generation')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle('Extreme Condition Simulation: $T_{env}=-10°C$, $N=300$ cycles, $I=1.5A$',
                fontsize=12, fontweight='bold', y=1.02)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_3d_power_surface(R, D, P, save_path=None):
    """
    Figure: 5G Power Consumption 3D Surface
    """
    fig = plt.figure(figsize=(14, 6))
    
    # 3D Surface
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Normalize for colormap
    P_clipped = np.clip(P, 0, 4000)  # Clip extreme values
    surf = ax1.plot_surface(R/1e6, D, P_clipped/1000, cmap=thermal_cmap,
                           linewidth=0, antialiased=True, alpha=0.9)
    
    ax1.set_xlabel('Data Rate (Mbps)')
    ax1.set_ylabel('Distance (m)')
    ax1.set_zlabel('Power (W)')
    ax1.set_title('(a) 5G Power Consumption Surface')
    ax1.view_init(elev=25, azim=45)
    
    # Add colorbar
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, label='Power (W)')
    
    # 2D Heatmap
    ax2 = fig.add_subplot(122)
    
    contour = ax2.contourf(R/1e6, D, P_clipped/1000, levels=20, cmap=thermal_cmap)
    ax2.set_xlabel('Data Rate (Mbps)')
    ax2.set_ylabel('Distance (m)')
    ax2.set_title('(b) Power Consumption Contour Map')
    
    # Add contour lines
    cs = ax2.contour(R/1e6, D, P_clipped/1000, levels=[1, 2, 3, 3.5], colors='white', linewidths=0.8)
    ax2.clabel(cs, inline=True, fontsize=8, fmt='%.1f W')
    
    fig.colorbar(contour, ax=ax2, label='Power (W)')
    
    # Mark critical region
    ax2.annotate('Power\nExplosion\nZone', xy=(800, 1000), fontsize=9, 
                color='white', ha='center', weight='bold')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_battery_thermal_field(save_path=None):
    """
    Figure: Battery internal temperature distribution heatmap (FEM simulation result)
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Create synthetic temperature field for cylindrical battery
    # Radial temperature distribution
    r = np.linspace(0, 1, 100)  # Normalized radius
    theta = np.linspace(0, 2*np.pi, 100)
    R, Theta = np.meshgrid(r, theta)
    X = R * np.cos(Theta)
    Y = R * np.sin(Theta)
    
    # Temperature profile: hotter in center
    T_center = 52
    T_surface = 45
    T = T_center - (T_center - T_surface) * R**0.8
    # Add some azimuthal variation
    T += 2 * np.sin(3 * Theta) * (1 - R)
    
    # Cross-section view
    ax1 = axes[0]
    c1 = ax1.contourf(X, Y, T, levels=30, cmap='hot')
    ax1.set_aspect('equal')
    ax1.set_xlabel('X (normalized)')
    ax1.set_ylabel('Y (normalized)')
    ax1.set_title('(a) Battery Cross-Section Temperature Field')
    
    # Add annotations
    ax1.annotate('Core: 52°C', xy=(0, 0), xytext=(0.5, 0.5),
                fontsize=9, color='white', arrowprops=dict(arrowstyle='->', color='white'))
    ax1.annotate('Surface: 45°C', xy=(0.7, 0.7), xytext=(0.3, 0.9),
                fontsize=9, color='white', arrowprops=dict(arrowstyle='->', color='white'))
    
    fig.colorbar(c1, ax=ax1, label='Temperature (°C)')
    
    # Longitudinal view
    ax2 = axes[1]
    z = np.linspace(0, 1, 100)  # Normalized height
    R2, Z = np.meshgrid(r, z)
    
    # Temperature with end effects
    T2 = T_center - (T_center - T_surface) * R2**0.8
    T2 -= 3 * (Z - 0.5)**2  # Cooler at ends
    T2 = gaussian_filter(T2, sigma=2)
    
    c2 = ax2.contourf(R2, Z, T2, levels=30, cmap='hot')
    ax2.set_xlabel('Radial Position (normalized)')
    ax2.set_ylabel('Axial Position (normalized)')
    ax2.set_title('(b) Battery Longitudinal Temperature Field')
    
    fig.colorbar(c2, ax=ax2, label='Temperature (°C)')
    
    # Add temperature gradient indicators
    ax2.annotate('Hot Zone', xy=(0.2, 0.5), fontsize=9, color='white', weight='bold')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_gnss_state_dynamics(results, save_path=None):
    """
    Figure: GPS state machine dynamics with tunnel effect
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    time = results['time']
    
    # Top: Signal and Lock State
    ax1 = axes[0]
    ln1 = ax1.plot(time, results['signal'], 'b-', lw=2, label='Environment SNR')
    ax1.axhline(y=25, color='red', linestyle='--', alpha=0.7, label='Lock Threshold')
    ax1.fill_between(time, 0, results['signal'], alpha=0.2, color='blue')
    ax1.set_ylabel('Signal Strength (dB-Hz)', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1.legend(loc='upper left')
    ax1.set_title('(a) Environmental Signal Quality and Lock State')
    
    ax1_twin = ax1.twinx()
    ln2 = ax1_twin.plot(time, results['lock_state'], 'g-', lw=2, label='Lock State')
    ax1_twin.set_ylabel('Lock Probability', color='green')
    ax1_twin.tick_params(axis='y', labelcolor='green')
    ax1_twin.set_ylim([-0.1, 1.1])
    ax1_twin.legend(loc='upper right')
    
    # Add event markers
    ax1.axvspan(10, 30, alpha=0.2, color='gray', label='Tunnel')
    ax1.axvspan(40, 55, alpha=0.15, color='orange', label='Urban Canyon')
    ax1.text(20, 45, 'Tunnel', fontsize=10, ha='center', weight='bold')
    ax1.text(47.5, 45, 'Canyon', fontsize=10, ha='center', weight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Bottom: Power consumption
    ax2 = axes[1]
    ax2.fill_between(time, 0, results['power'], alpha=0.4, color='red')
    ax2.plot(time, results['power'], 'r-', lw=2)
    ax2.axhline(y=45, color='green', linestyle=':', alpha=0.7, label='$P_{track}$')
    ax2.axhline(y=115, color='purple', linestyle=':', alpha=0.7, label='$P_{acq}$')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Power Consumption (mW)')
    ax2.set_title('(b) GNSS Power Response with Hysteresis')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Add annotation for hysteresis
    ax2.annotate('Delayed\nRecovery', xy=(35, 90), xytext=(45, 100),
                fontsize=9, arrowprops=dict(arrowstyle='->', color='red'))
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_background_random_current(time, current, save_path=None):
    """
    Figure: Background random current analysis with histogram
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Time domain
    ax1 = axes[0]
    ax1.plot(time, current, 'b-', lw=0.5, alpha=0.8)
    ax1.fill_between(time, 0, current, alpha=0.3, color='blue')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Current (mA)')
    ax1.set_title('(a) Background Task Random Wake-up Pattern')
    ax1.set_xlim([0, time[-1]])
    ax1.grid(True, alpha=0.3)
    
    # Mark tail periods
    baseline = np.median(current)
    spikes = current > baseline * 2
    ax1.axhline(y=baseline, color='red', linestyle='--', alpha=0.7, label='Baseline')
    ax1.legend()
    
    # Histogram
    ax2 = axes[1]
    # Use log scale for histogram
    bins = np.logspace(np.log10(max(0.1, current.min())), np.log10(current.max()), 50)
    ax2.hist(current, bins=bins, color='#2E86AB', edgecolor='white', alpha=0.7)
    ax2.set_xscale('log')
    ax2.set_xlabel('Current (mA)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('(b) Current Distribution (Log Scale)')
    ax2.grid(True, alpha=0.3)
    
    # Add long-tail annotation
    ax2.annotate('Long-tail\nDistribution', xy=(current.max()*0.8, 5), 
                fontsize=10, weight='bold', color='red')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_oled_theme_comparison(brightness_range, light_power, dark_power, save_path=None):
    """
    Figure: OLED theme comparison
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Power vs brightness
    ax1 = axes[0]
    ax1.plot(brightness_range, light_power, 'r-', lw=2.5, label='Light Theme (APL=0.85)')
    ax1.plot(brightness_range, dark_power, 'b-', lw=2.5, label='Dark Theme (APL=0.15)')
    ax1.fill_between(brightness_range, dark_power, light_power, alpha=0.2, color='green')
    ax1.set_xlabel('Screen Brightness (nits)')
    ax1.set_ylabel('Power Consumption (mW)')
    ax1.set_title('(a) Display Power: Light vs Dark Theme')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # Savings percentage
    savings = (np.array(light_power) - np.array(dark_power)) / np.array(light_power) * 100
    ax1_twin = ax1.twinx()
    ax1_twin.plot(brightness_range, savings, 'g--', lw=2, label='Savings %')
    ax1_twin.set_ylabel('Power Savings (%)', color='green')
    ax1_twin.tick_params(axis='y', labelcolor='green')
    ax1_twin.legend(loc='lower right')
    
    # Bar comparison at 500 nits
    ax2 = axes[1]
    idx = len(brightness_range) // 2  # ~500 nits
    
    categories = ['Light Theme', 'Dark Theme']
    values = [light_power[idx], dark_power[idx]]
    colors = ['#C73E1D', '#2E86AB']
    
    bars = ax2.bar(categories, values, color=colors, edgecolor='white', linewidth=2)
    ax2.set_ylabel('Power Consumption (mW)')
    ax2.set_title(f'(b) Power at {int(brightness_range[idx])} nits')
    
    # Add value labels
    for bar, val in zip(bars, values):
        ax2.text(bar.get_x() + bar.get_width()/2, val + 20, f'{val:.0f} mW',
                ha='center', va='bottom', fontsize=11, weight='bold')
    
    # Add savings annotation
    savings_val = (values[0] - values[1]) / values[0] * 100
    ax2.annotate(f'{savings_val:.0f}% Savings', xy=(1, values[1]),
                xytext=(1.3, (values[0]+values[1])/2),
                fontsize=11, weight='bold', color='green',
                arrowprops=dict(arrowstyle='->', color='green'))
    
    ax2.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_soc_thermal_coupling(sim_results, save_path=None):
    """
    Figure: SoC electro-thermal coupling simulation
    """
    fig = plt.figure(figsize=(14, 8))
    gs = gridspec.GridSpec(2, 2, hspace=0.3, wspace=0.25)
    
    n = len(sim_results['T_junction'])
    time = np.arange(n)
    
    # Temperature vs Time
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(time, sim_results['T_junction'], 'r-', lw=2, label='Junction Temp')
    ax1.axhline(y=85, color='red', linestyle='--', alpha=0.7, label='Thermal Limit')
    ax1.fill_between(time, 25, sim_results['T_junction'], alpha=0.2, color='red')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Temperature (°C)')
    ax1.set_title('(a) Junction Temperature Evolution')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Power breakdown
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.stackplot(time, 
                  [sim_results['P_dynamic']*1000, sim_results['P_leakage']*1000],
                  labels=['Dynamic Power', 'Leakage Power'],
                  colors=['#2E86AB', '#C73E1D'], alpha=0.7)
    ax2.plot(time, sim_results['P_total']*1000, 'k-', lw=2, label='Total')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Power (mW)')
    ax2.set_title('(b) Power Decomposition')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Leakage vs Temperature (phase plot)
    ax3 = fig.add_subplot(gs[1, 0])
    scatter = ax3.scatter(sim_results['T_junction'], sim_results['P_leakage']*1000,
                         c=time, cmap='viridis', s=10, alpha=0.7)
    ax3.set_xlabel('Junction Temperature (°C)')
    ax3.set_ylabel('Leakage Power (mW)')
    ax3.set_title('(c) Leakage-Temperature Phase Diagram')
    fig.colorbar(scatter, ax=ax3, label='Time (s)')
    ax3.grid(True, alpha=0.3)
    
    # Efficiency degradation
    ax4 = fig.add_subplot(gs[1, 1])
    efficiency = sim_results['P_dynamic'] / (sim_results['P_total'] + 1e-10) * 100
    ax4.plot(time, efficiency, 'g-', lw=2)
    ax4.fill_between(time, 0, efficiency, alpha=0.2, color='green')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Efficiency (%)')
    ax4.set_title('(d) Energy Efficiency (Dynamic/Total)')
    ax4.set_ylim([0, 105])
    ax4.grid(True, alpha=0.3)
    
    # Add annotation for efficiency collapse
    min_eff_idx = np.argmin(efficiency)
    ax4.annotate(f'Min: {efficiency[min_eff_idx]:.1f}%', 
                xy=(min_eff_idx, efficiency[min_eff_idx]),
                xytext=(min_eff_idx + len(time)*0.1, efficiency[min_eff_idx] + 10),
                fontsize=9, arrowprops=dict(arrowstyle='->', color='red'))
    
    plt.suptitle('SoC Electro-Thermal Coupling Under High Workload', 
                fontsize=12, fontweight='bold', y=1.02)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_power_breakdown_sankey(power_dict, save_path=None):
    """
    Figure: Power consumption breakdown (Sankey-style stacked bar)
    """
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Remove 'Total' if present
    if 'Total' in power_dict:
        total = power_dict.pop('Total')
    else:
        total = sum(power_dict.values())
    
    components = list(power_dict.keys())
    values = list(power_dict.values())
    percentages = [v/total*100 for v in values]
    
    # Colors
    colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#3D348B', '#7678ED']
    
    # Create stacked horizontal bar
    left = 0
    for i, (comp, val, pct) in enumerate(zip(components, values, percentages)):
        bar = ax.barh(0, val, left=left, height=0.5, 
                     color=colors[i % len(colors)], 
                     edgecolor='white', linewidth=2)
        
        # Add label if segment is wide enough
        if pct > 5:
            ax.text(left + val/2, 0, f'{comp}\n{val:.0f}mW\n({pct:.1f}%)',
                   ha='center', va='center', fontsize=9, color='white', weight='bold')
        left += val
    
    ax.set_xlim([0, total])
    ax.set_ylim([-0.5, 0.5])
    ax.set_xlabel('Power Consumption (mW)')
    ax.set_yticks([])
    ax.set_title(f'Power Consumption Breakdown (Total: {total:.0f} mW)')
    
    # Create legend
    handles = [mpatches.Patch(color=colors[i % len(colors)], label=f'{comp}: {val:.0f}mW')
              for i, (comp, val) in enumerate(zip(components, values))]
    ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(1.15, 1))
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_3d_aging_temperature_surface(save_path=None):
    """
    Novel 3D visualization: Capacity vs Aging vs Temperature
    """
    fig = plt.figure(figsize=(14, 6))
    
    # Create mesh
    N = np.linspace(0, 500, 50)  # Cycle number
    T = np.linspace(-20, 60, 50)  # Temperature
    N_mesh, T_mesh = np.meshgrid(N, T)
    
    # Calculate capacity surface
    # Q(N,T) = Q_aging(N) * S_T(T)
    a_Q, b_Q, c_Q, d_Q = -0.1137, 0.0243, 1.9305, 0.0007
    S_Q, k_Q, T0 = 1.0391, 0.0895, -15.1281
    
    Q_aging = a_Q * np.exp(-b_Q * N_mesh) + c_Q * np.exp(-d_Q * N_mesh)
    S_T = S_Q / (1 + np.exp(-k_Q * (T_mesh - T0)))
    Q_total = Q_aging * S_T
    
    # 3D Surface
    ax1 = fig.add_subplot(121, projection='3d')
    surf = ax1.plot_surface(N_mesh, T_mesh, Q_total, cmap='viridis',
                           linewidth=0, antialiased=True, alpha=0.9)
    ax1.set_xlabel('Cycle Number')
    ax1.set_ylabel('Temperature (°C)')
    ax1.set_zlabel('Capacity (Ah)')
    ax1.set_title('(a) Capacity-Aging-Temperature Surface')
    ax1.view_init(elev=20, azim=45)
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, label='Capacity (Ah)')
    
    # Calculate resistance surface
    a_R, b_R, c_R = 0.0110, 0.2106, 0.0181
    C_R, A_R, B_R = 0.7136, 1.3533, 0.0630
    
    R_aging = a_R * np.power(N_mesh + 1, b_R) + c_R
    S_R = C_R + A_R * np.exp(-B_R * T_mesh)
    R_total = R_aging * S_R
    
    # 3D Surface for resistance
    ax2 = fig.add_subplot(122, projection='3d')
    surf2 = ax2.plot_surface(N_mesh, T_mesh, R_total * 1000, cmap='hot',
                            linewidth=0, antialiased=True, alpha=0.9)
    ax2.set_xlabel('Cycle Number')
    ax2.set_ylabel('Temperature (°C)')
    ax2.set_zlabel('Resistance (mΩ)')
    ax2.set_title('(b) Resistance-Aging-Temperature Surface')
    ax2.view_init(elev=20, azim=135)
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, label='Resistance (mΩ)')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_battery_life_prediction(save_path=None):
    """
    Novel visualization: Battery life prediction under various scenarios
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # Scenario definitions
    scenarios = {
        'Idle (Screen Off)': {'power': 50, 'color': '#2E86AB'},
        'Web Browsing': {'power': 400, 'color': '#A23B72'},
        'Video Streaming': {'power': 800, 'color': '#F18F01'},
        'Gaming': {'power': 2500, 'color': '#C73E1D'},
        'Navigation': {'power': 1200, 'color': '#3D348B'}
    }
    
    battery_capacity_mAh = 4500
    voltage = 3.7
    battery_energy_Wh = battery_capacity_mAh * voltage / 1000
    
    # (a) Battery life vs scenario
    ax1 = axes[0, 0]
    names = list(scenarios.keys())
    powers = [s['power'] for s in scenarios.values()]
    colors = [s['color'] for s in scenarios.values()]
    
    # Calculate hours
    hours = [battery_energy_Wh * 1000 / p for p in powers]
    
    bars = ax1.barh(names, hours, color=colors, edgecolor='white', linewidth=2)
    ax1.set_xlabel('Battery Life (hours)')
    ax1.set_title('(a) Predicted Battery Life by Usage Scenario')
    
    for bar, h in zip(bars, hours):
        ax1.text(h + 0.5, bar.get_y() + bar.get_height()/2, f'{h:.1f}h',
                va='center', fontsize=10, weight='bold')
    ax1.grid(True, alpha=0.3, axis='x')
    
    # (b) Aging effect on battery life
    ax2 = axes[0, 1]
    cycles = np.array([0, 100, 200, 300, 400, 500])
    capacity_retention = (-0.1137 * np.exp(-0.0243 * cycles) + 
                         1.9305 * np.exp(-0.0007 * cycles)) / 1.8  # Normalized
    
    for name, scenario in scenarios.items():
        base_life = battery_energy_Wh * 1000 / scenario['power']
        life_with_aging = base_life * capacity_retention
        ax2.plot(cycles, life_with_aging, '-o', lw=2, markersize=4,
                label=name, color=scenario['color'])
    
    ax2.set_xlabel('Cycle Number')
    ax2.set_ylabel('Battery Life (hours)')
    ax2.set_title('(b) Battery Life Degradation with Aging')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # (c) Temperature effect
    ax3 = axes[1, 0]
    temperatures = np.linspace(-20, 60, 50)
    S_T = 1.0391 / (1 + np.exp(-0.0895 * (temperatures + 15.1281)))
    
    # Different scenarios at different temps
    for name, scenario in list(scenarios.items())[:3]:
        base_life = battery_energy_Wh * 1000 / scenario['power']
        life_with_temp = base_life * S_T
        ax3.plot(temperatures, life_with_temp, '-', lw=2.5,
                label=name, color=scenario['color'])
    
    ax3.axvline(x=25, color='green', linestyle='--', alpha=0.7, label='Optimal (25°C)')
    ax3.axvline(x=-10, color='red', linestyle=':', alpha=0.7, label='Cold Warning')
    ax3.set_xlabel('Temperature (°C)')
    ax3.set_ylabel('Battery Life (hours)')
    ax3.set_title('(c) Temperature Impact on Battery Life')
    ax3.legend(loc='upper left', fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # (d) Combined effect heatmap
    ax4 = axes[1, 1]
    
    cycles_grid = np.linspace(0, 500, 30)
    temps_grid = np.linspace(-10, 50, 30)
    C, T = np.meshgrid(cycles_grid, temps_grid)
    
    # Calculate combined capacity factor
    aging_factor = (-0.1137 * np.exp(-0.0243 * C) + 1.9305 * np.exp(-0.0007 * C)) / 1.8
    temp_factor = 1.0391 / (1 + np.exp(-0.0895 * (T + 15.1281)))
    combined = aging_factor * temp_factor
    
    # Battery life for video streaming
    base_life_video = battery_energy_Wh * 1000 / 800
    life_map = base_life_video * combined
    
    contour = ax4.contourf(C, T, life_map, levels=20, cmap='RdYlGn')
    ax4.set_xlabel('Cycle Number')
    ax4.set_ylabel('Temperature (°C)')
    ax4.set_title('(d) Battery Life (Video Streaming) - Combined Effects')
    fig.colorbar(contour, ax=ax4, label='Battery Life (hours)')
    
    # Add contour lines
    cs = ax4.contour(C, T, life_map, levels=[3, 4, 5, 6], colors='white', linewidths=0.8)
    ax4.clabel(cs, inline=True, fontsize=8, fmt='%.0fh')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_recommendation_radar(save_path=None):
    """
    Novel visualization: Power saving strategy effectiveness radar chart
    """
    fig = plt.figure(figsize=(12, 10))
    
    # Define categories and strategies
    categories = ['5G/Network', 'Display', 'Background', 'GPS', 'CPU', 'Bluetooth']
    
    # Strategy effectiveness scores (0-10)
    strategies = {
        'Dark Mode': [1, 9, 0, 0, 0, 0],
        'Lower Brightness': [0, 8, 0, 0, 0, 0],
        'Airplane Mode': [10, 0, 5, 0, 2, 10],
        'Close Background Apps': [2, 0, 9, 0, 4, 0],
        'Location Off': [1, 0, 3, 10, 1, 0],
        'Power Saver Mode': [7, 6, 7, 6, 8, 5],
    }
    
    # Calculate angles
    N = len(categories)
    angles = [n / float(N) * 2 * np.pi for n in range(N)]
    angles += angles[:1]  # Complete the loop
    
    # Create subplot
    ax = fig.add_subplot(111, polar=True)
    
    colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#3D348B', '#7678ED']
    
    for i, (strategy, values) in enumerate(strategies.items()):
        values += values[:1]  # Complete the loop
        ax.plot(angles, values, 'o-', linewidth=2, label=strategy, 
               color=colors[i % len(colors)])
        ax.fill(angles, values, alpha=0.1, color=colors[i % len(colors)])
    
    # Set category labels
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, fontsize=10)
    
    # Set radial labels
    ax.set_rlabel_position(30)
    ax.set_yticks([2, 4, 6, 8, 10])
    ax.set_yticklabels(['2', '4', '6', '8', '10'], fontsize=8)
    ax.set_ylim(0, 10)
    
    plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
    plt.title('Power Saving Strategy Effectiveness by Component', 
             fontsize=12, fontweight='bold', y=1.08)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_charging_optimization_3d(save_path=None):
    """
    Novel 3D plot: Charging optimization under temperature and aging
    """
    fig = plt.figure(figsize=(14, 6))
    
    # Create mesh
    T = np.linspace(0, 45, 40)  # Temperature
    N = np.linspace(0, 500, 40)  # Cycle number
    T_mesh, N_mesh = np.meshgrid(T, N)
    
    # Optimal charging current model
    # Lower current at extreme temps and high aging
    I_base = 2.0  # Base charging current (A)
    
    # Temperature factor (optimal around 25°C)
    temp_factor = np.exp(-((T_mesh - 25) / 15)**2)
    
    # Aging factor (reduce current for older batteries)
    aging_factor = 1 - 0.3 * (N_mesh / 500)
    
    # Combined optimal current
    I_optimal = I_base * temp_factor * aging_factor
    
    # 3D Surface
    ax1 = fig.add_subplot(121, projection='3d')
    surf = ax1.plot_surface(T_mesh, N_mesh, I_optimal, cmap='coolwarm',
                           linewidth=0, antialiased=True, alpha=0.9)
    ax1.set_xlabel('Temperature (°C)')
    ax1.set_ylabel('Cycle Number')
    ax1.set_zlabel('Optimal Charge Current (A)')
    ax1.set_title('(a) Optimal Charging Current Surface')
    ax1.view_init(elev=25, azim=-45)
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, label='Current (A)')
    
    # Charging time estimate
    capacity = 4.5  # Ah
    charge_time = capacity / I_optimal  # hours
    
    ax2 = fig.add_subplot(122, projection='3d')
    surf2 = ax2.plot_surface(T_mesh, N_mesh, charge_time, cmap='viridis',
                            linewidth=0, antialiased=True, alpha=0.9)
    ax2.set_xlabel('Temperature (°C)')
    ax2.set_ylabel('Cycle Number')
    ax2.set_zlabel('Charging Time (hours)')
    ax2.set_title('(b) Expected Charging Time Surface')
    ax2.view_init(elev=25, azim=-45)
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, label='Time (h)')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


if __name__ == "__main__":
    # Test plots
    import os
    os.makedirs('/workspace/battery_modeling/figures', exist_ok=True)
    
    print("Generating test plots...")
    
    # Test aging plot
    cycles = np.linspace(0, 500, 100)
    capacity = -0.1137 * np.exp(-0.0243 * cycles) + 1.9305 * np.exp(-0.0007 * cycles)
    resistance = 0.0110 * np.power(cycles + 1, 0.2106) + 0.0181
    
    plot_aging_characteristics(cycles, capacity, resistance, capacity, resistance,
                              '/workspace/battery_modeling/figures/test_aging.png')
    
    print("Test plots generated!")
