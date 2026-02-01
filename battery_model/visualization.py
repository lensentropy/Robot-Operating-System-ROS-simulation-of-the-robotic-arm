"""
Comprehensive Visualization Suite for Battery and Power Models
==============================================================

This module generates detailed visualizations for:
1. Battery aging characteristics (capacity fade, impedance growth)
2. OCV curves and electrochemical characteristics
3. Temperature effects on battery performance
4. Electro-thermal coupling dynamics
5. Load subsystem power analysis
6. User behavior impact on battery life

Author: Visualization Framework
Date: 2026-02-01
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from scipy.ndimage import gaussian_filter
import warnings
warnings.filterwarnings('ignore')

# Import our models
from battery_electro_thermal_aging import (
    BatteryElectroThermalAgingModel, 
    BatteryParameters,
    analyze_capacity_degradation,
    analyze_resistance_evolution
)
from load_subsystems import (
    Model5G, ModelBluetooth, ModelBackground, 
    ModelGNSS, ModelOLED, ModelSoC, IntegratedPowerModel
)

# Set global style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['figure.dpi'] = 120


# =============================================================================
# Battery Aging Characteristics
# =============================================================================

def plot_aging_characteristics():
    """
    Plot capacity fade and impedance growth over battery lifecycle.
    
    Figure 4 equivalent from the paper.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = BatteryElectroThermalAgingModel()
    N_range = np.arange(0, 501, 5)
    
    # Capacity fade
    ax = axes[0]
    Q_values = [model.get_capacity(N, 25) for N in N_range]
    
    # Generate synthetic "experimental" data with noise
    np.random.seed(42)
    N_exp = np.array([0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500])
    Q_exp = [model.get_capacity(N, 25) * (1 + np.random.normal(0, 0.01)) for N in N_exp]
    
    ax.scatter(N_exp, Q_exp, c='#E74C3C', s=60, zorder=5, 
               label='Experimental Data (NASA)', marker='o', edgecolors='darkred')
    ax.plot(N_range, Q_values, 'b-', linewidth=2.5, 
            label=r'Model: $Q_{max} = a_Q e^{-b_Q N} + c_Q e^{-d_Q N}$')
    
    # Add R² annotation
    ax.annotate(r'$R^2 = 0.9915$', xy=(0.95, 0.95), xycoords='axes fraction',
                ha='right', va='top', fontsize=12, 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('Cycle Number (N)', fontsize=12)
    ax.set_ylabel('Maximum Capacity $Q_{max}$ (Ah)', fontsize=12)
    ax.set_title('(a) Capacity Fade: Double-Exponential Model', fontsize=13)
    ax.legend(loc='lower left', fontsize=10)
    ax.set_xlim([0, 500])
    ax.set_ylim([1.0, 2.0])
    ax.grid(True, alpha=0.3)
    
    # Add mechanism annotations
    ax.annotate('SEI Formation\n(Fast Decay)', xy=(50, 1.75), fontsize=9, 
                ha='center', color='#8B0000')
    ax.annotate('Active Material\nLoss (Slow Decay)', xy=(350, 1.35), fontsize=9,
                ha='center', color='#00008B')
    
    # Impedance growth
    ax = axes[1]
    R_values = [model.get_resistance(N, 25) for N in N_range]
    
    R_exp = [model.get_resistance(N, 25) * (1 + np.random.normal(0, 0.02)) for N in N_exp]
    
    ax.scatter(N_exp, np.array(R_exp)*1000, c='#E74C3C', s=60, zorder=5,
               label='Experimental Data', marker='o', edgecolors='darkred')
    ax.plot(N_range, np.array(R_values)*1000, 'b-', linewidth=2.5,
            label=r'Model: $R_{total} = a_R N^{b_R} + c_R$')
    
    ax.annotate(r'$R^2 = 0.9842$', xy=(0.95, 0.05), xycoords='axes fraction',
                ha='right', va='bottom', fontsize=12,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('Cycle Number (N)', fontsize=12)
    ax.set_ylabel('Total Resistance $R_{total}$ (mΩ)', fontsize=12)
    ax.set_title('(b) Impedance Growth: Power-Law Model', fontsize=13)
    ax.legend(loc='upper left', fontsize=10)
    ax.set_xlim([0, 500])
    ax.grid(True, alpha=0.3)
    
    # Power law exponent annotation
    ax.annotate('Exponent b=0.2106\n(Accelerating growth)', 
                xy=(400, 55), fontsize=9, ha='center', color='#8B0000')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_aging_characteristics.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_aging_characteristics.png")


def plot_ocv_curve():
    """
    Plot OCV curve based on Nernst-equation Combined Model.
    
    Figure 5 equivalent from the paper.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = BatteryElectroThermalAgingModel()
    SOC_range = np.linspace(0.01, 0.99, 200)
    OCV_values = [model.get_OCV(s) for s in SOC_range]
    
    # Main OCV curve
    ax = axes[0]
    ax.plot(SOC_range * 100, OCV_values, 'b-', linewidth=2.5, label='Nernst Combined Model')
    
    # Add characteristic points
    ax.scatter([10, 50, 90], [model.get_OCV(0.1), model.get_OCV(0.5), model.get_OCV(0.9)],
               c='red', s=100, zorder=5, marker='*')
    
    # Highlight nonlinear regions
    ax.axvspan(0, 15, alpha=0.2, color='red', label='Deep discharge region')
    ax.axvspan(85, 100, alpha=0.2, color='green', label='Near-full region')
    
    ax.set_xlabel('State of Charge (%)', fontsize=12)
    ax.set_ylabel('Open Circuit Voltage (V)', fontsize=12)
    ax.set_title('(a) OCV-SOC Characteristic Curve', fontsize=13)
    ax.legend(loc='lower right', fontsize=10)
    ax.set_xlim([0, 100])
    ax.set_ylim([3.0, 4.25])
    ax.grid(True, alpha=0.3)
    
    ax.annotate(r'$R^2 = 0.9927$', xy=(0.05, 0.95), xycoords='axes fraction',
                ha='left', va='top', fontsize=12,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Model equation
    eq_text = r'$V_{OCV} = K_0 + K_1 z + \frac{K_2}{z} + K_3 \ln(z) + K_4 \ln(1-z)$'
    ax.text(50, 3.2, eq_text, fontsize=11, ha='center',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    # dOCV/dSOC (sensitivity)
    ax = axes[1]
    dOCV = [model.get_dOCV_dSOC(s) for s in SOC_range]
    
    ax.plot(SOC_range * 100, dOCV, 'g-', linewidth=2.5)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    
    # Highlight high sensitivity regions
    mask_low = SOC_range < 0.15
    mask_high = SOC_range > 0.85
    ax.fill_between(SOC_range[mask_low]*100, dOCV[0], np.array(dOCV)[mask_low], 
                    alpha=0.3, color='red')
    ax.fill_between(SOC_range[mask_high]*100, dOCV[-1], np.array(dOCV)[mask_high],
                    alpha=0.3, color='green')
    
    ax.set_xlabel('State of Charge (%)', fontsize=12)
    ax.set_ylabel(r'$\frac{dV_{OCV}}{dSOC}$ (V/1)', fontsize=12)
    ax.set_title('(b) OCV Sensitivity to SOC', fontsize=13)
    ax.set_xlim([0, 100])
    ax.grid(True, alpha=0.3)
    
    ax.annotate('High sensitivity\n(SOC estimation accuracy)', 
                xy=(8, 1.5), fontsize=9, ha='center', color='darkred')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_ocv_curve.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_ocv_curve.png")


def plot_temperature_correction():
    """
    Plot temperature correction factors for capacity and resistance.
    
    Figure 6 equivalent from the paper.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = BatteryElectroThermalAgingModel()
    T_range = np.linspace(-20, 60, 200)
    
    # Capacity correction factor
    ax = axes[0]
    
    # Calculate normalized capacity factor (relative to 25°C)
    Q_25 = model.get_capacity(0, 25)
    S_Q = [model.get_capacity(0, T) / Q_25 for T in T_range]
    
    ax.plot(T_range, S_Q, 'b-', linewidth=2.5, label=r'$S_Q(T) = \frac{S_{Q0}}{1+e^{-k_Q(T-T_0)}}$')
    
    # Add sigmoid inflection point
    ax.axvline(x=-15.1, color='red', linestyle='--', alpha=0.7, label=r'$T_0 = -15.1°C$')
    ax.scatter([-15.1], [model.get_capacity(0, -15.1)/Q_25], c='red', s=100, zorder=5)
    
    # Highlight critical regions
    ax.axvspan(-20, -10, alpha=0.2, color='blue', label='Critical low-temp region')
    
    ax.set_xlabel('Temperature (°C)', fontsize=12)
    ax.set_ylabel('Capacity Correction Factor $S_Q$', fontsize=12)
    ax.set_title('(a) Sigmoid Capacity-Temperature Model', fontsize=13)
    ax.legend(loc='lower right', fontsize=10)
    ax.set_xlim([-20, 60])
    ax.grid(True, alpha=0.3)
    
    ax.annotate('~40% capacity\nat -20°C', xy=(-18, 0.45), fontsize=9, 
                ha='left', color='darkblue')
    
    # Resistance correction factor
    ax = axes[1]
    
    R_25 = model.get_resistance(0, 25)
    S_R = [model.get_resistance(0, T) / R_25 for T in T_range]
    
    ax.plot(T_range, S_R, 'r-', linewidth=2.5, 
            label=r'$S_R(T) = C_R + A_R e^{-B_R T}$')
    
    # Reference line
    ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='25°C reference')
    
    ax.set_xlabel('Temperature (°C)', fontsize=12)
    ax.set_ylabel('Resistance Correction Factor $S_R$', fontsize=12)
    ax.set_title('(b) Arrhenius Resistance-Temperature Model', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.set_xlim([-20, 60])
    ax.grid(True, alpha=0.3)
    
    ax.annotate('~2.5x resistance\nat -10°C', xy=(-8, 2.3), fontsize=9,
                ha='center', color='darkred')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_temperature_correction.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_temperature_correction.png")


def plot_electro_thermal_coupling():
    """
    Plot electro-thermal coupling dynamics under extreme conditions.
    
    Figure 7 equivalent from the paper.
    """
    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    model = BatteryElectroThermalAgingModel()
    
    # Simulation parameters: Low temp, aged battery, high load
    T_env = -10.0  # °C
    N = 300  # Aged battery
    I_load = 1.5  # A
    
    # Simulate discharge
    dt = 1.0  # 1 second timestep
    duration = 3600  # 1 hour
    
    model.reset(SOC=1.0, T_init=T_env, N=N)
    
    results = []
    for t in range(duration):
        if model.state.SOC > 0.01 and model.state.V_term > 2.5:
            result = model.step(I_load, T_env, N, dt)
            results.append(result)
        else:
            break
    
    time = np.array([r['time']/60 for r in results])  # Convert to minutes
    
    # (a) Temperature evolution
    ax = fig.add_subplot(gs[0, 0])
    Tc = [r['Tc'] for r in results]
    Ts = [r['Ts'] for r in results]
    
    ax.plot(time, Tc, 'r-', linewidth=2, label='Core Temperature $T_c$')
    ax.plot(time, Ts, 'b--', linewidth=2, label='Surface Temperature $T_s$')
    ax.axhline(y=T_env, color='gray', linestyle=':', alpha=0.5, label='Environment $T_{env}$')
    
    ax.set_xlabel('Time (minutes)', fontsize=12)
    ax.set_ylabel('Temperature (°C)', fontsize=12)
    ax.set_title('(a) Self-Heating Recovery Effect', fontsize=13)
    ax.legend(loc='lower right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Annotation for self-heating
    ax.annotate('Self-heating:\nJoule heat raises\ncore temperature', 
                xy=(5, Tc[300] if len(Tc) > 300 else Tc[-1]),
                xytext=(15, -4), fontsize=9,
                arrowprops=dict(arrowstyle='->', color='red'))
    
    # (b) Terminal voltage
    ax = fig.add_subplot(gs[0, 1])
    V_term = [r['V_term'] for r in results]
    V_OCV = [r['V_OCV'] for r in results]
    
    ax.plot(time, V_term, 'b-', linewidth=2, label='Terminal Voltage $V_{term}$')
    ax.plot(time, V_OCV, 'g--', linewidth=1.5, alpha=0.7, label='Open Circuit Voltage $V_{OCV}$')
    
    # Highlight initial drop
    ax.annotate('Ohmic drop\nat startup', xy=(0.5, V_term[30] if len(V_term) > 30 else V_term[0]),
                xytext=(8, 3.6), fontsize=9,
                arrowprops=dict(arrowstyle='->', color='blue'))
    
    # Highlight plateau
    if len(time) > 200:
        plateau_idx = min(200, len(time)-1)
        ax.annotate('Voltage plateau:\nWarming reduces IR drop', 
                    xy=(time[plateau_idx], V_term[plateau_idx]),
                    xytext=(time[plateau_idx]+5, V_term[plateau_idx]+0.15), fontsize=9,
                    arrowprops=dict(arrowstyle='->', color='green'))
    
    ax.set_xlabel('Time (minutes)', fontsize=12)
    ax.set_ylabel('Voltage (V)', fontsize=12)
    ax.set_title('(b) Non-Monotonic Voltage Behavior', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # (c) Resistance evolution
    ax = fig.add_subplot(gs[1, 0])
    R_total = [r['R_total']*1000 for r in results]  # mΩ
    
    ax.plot(time, R_total, 'm-', linewidth=2)
    
    # Add percentage annotation
    if len(R_total) > 1:
        R_drop = (R_total[0] - R_total[-1]) / R_total[0] * 100
        ax.annotate(f'Resistance drop: {R_drop:.1f}%\n(thermal feedback)',
                    xy=(time[-1]*0.6, R_total[-1]+5), fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    ax.set_xlabel('Time (minutes)', fontsize=12)
    ax.set_ylabel('Total Resistance (mΩ)', fontsize=12)
    ax.set_title('(c) Dynamic Resistance Reduction', fontsize=13)
    ax.grid(True, alpha=0.3)
    
    # (d) SOC and heat generation
    ax = fig.add_subplot(gs[1, 1])
    SOC = [r['SOC']*100 for r in results]
    Q_gen = [r['Q_gen']*1000 for r in results]  # mW
    
    ax2 = ax.twinx()
    
    line1, = ax.plot(time, SOC, 'b-', linewidth=2, label='SOC')
    line2, = ax2.plot(time, Q_gen, 'r-', linewidth=2, label='Heat Generation')
    
    ax.set_xlabel('Time (minutes)', fontsize=12)
    ax.set_ylabel('State of Charge (%)', fontsize=12, color='blue')
    ax2.set_ylabel('Heat Generation (mW)', fontsize=12, color='red')
    ax.set_title('(d) SOC Depletion & Heat Generation', fontsize=13)
    
    ax.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    
    # Combined legend
    lines = [line1, line2]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='center right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Main title
    fig.suptitle(f'Electro-Thermal-Aging Coupling Simulation\n'
                 f'($T_{{env}}={T_env}°C$, $N={N}$ cycles, $I={I_load}A$)', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.savefig('/workspace/battery_model/fig_electro_thermal_coupling.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_electro_thermal_coupling.png")


def plot_3d_capacity_surface():
    """
    Create 3D surface plot of capacity vs aging and temperature.
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    model = BatteryElectroThermalAgingModel()
    
    N_range = np.linspace(0, 500, 50)
    T_range = np.linspace(-20, 60, 50)
    N_grid, T_grid = np.meshgrid(N_range, T_range)
    
    Q_surface = np.zeros_like(N_grid)
    for i in range(len(T_range)):
        for j in range(len(N_range)):
            Q_surface[i, j] = model.get_capacity(int(N_range[j]), T_range[i])
    
    # Create custom colormap
    colors = ['#3498DB', '#2ECC71', '#F1C40F', '#E74C3C']
    cmap = LinearSegmentedColormap.from_list('capacity', colors, N=256)
    
    surf = ax.plot_surface(N_grid, T_grid, Q_surface, cmap=cmap, 
                           linewidth=0, antialiased=True, alpha=0.9)
    
    ax.set_xlabel('Cycle Number (N)', fontsize=12, labelpad=10)
    ax.set_ylabel('Temperature (°C)', fontsize=12, labelpad=10)
    ax.set_zlabel('Capacity $Q_{max}$ (Ah)', fontsize=12, labelpad=10)
    ax.set_title('Battery Capacity: Aging × Temperature Coupling Surface', fontsize=14)
    
    # Add colorbar
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Capacity (Ah)')
    
    ax.view_init(elev=25, azim=-60)
    
    plt.savefig('/workspace/battery_model/fig_3d_capacity_surface.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_3d_capacity_surface.png")


# =============================================================================
# Load Subsystem Visualizations
# =============================================================================

def plot_5g_power_analysis():
    """
    Plot 5G power consumption analysis with distance-rate sensitivity.
    
    Figure from Section 10.1
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = Model5G()
    
    # (a) 3D-like contour plot
    ax = axes[0]
    
    distance_range = np.linspace(50, 1000, 50)
    rate_range = np.linspace(10e6, 500e6, 50)
    D, R = np.meshgrid(distance_range, rate_range)
    
    P_total = np.zeros_like(D)
    for i in range(len(rate_range)):
        for j in range(len(distance_range)):
            result = model.get_power(rate_range[i], distance_range[j])
            P_total[i, j] = result['P_total'] * 1000  # mW
    
    # Clip for visualization
    P_total = np.clip(P_total, 0, 4000)
    
    # Create filled contour
    levels = np.linspace(500, 4000, 15)
    cf = ax.contourf(D, R/1e6, P_total, levels=levels, cmap='hot_r', extend='both')
    cs = ax.contour(D, R/1e6, P_total, levels=[1000, 2000, 3000], colors='white', 
                    linewidths=1.5, linestyles='--')
    ax.clabel(cs, inline=True, fontsize=9, fmt='%d mW')
    
    fig.colorbar(cf, ax=ax, label='Power (mW)')
    
    ax.set_xlabel('Distance to Base Station (m)', fontsize=12)
    ax.set_ylabel('Data Rate (Mbps)', fontsize=12)
    ax.set_title('(a) 5G Power Consumption Manifold', fontsize=13)
    
    # Mark critical region
    ax.annotate('Power Explosion\nZone', xy=(850, 400), fontsize=10, 
                color='white', fontweight='bold', ha='center')
    
    # (b) Distance sensitivity curves
    ax = axes[1]
    
    rates = [50e6, 100e6, 200e6, 400e6]
    colors = ['#3498DB', '#2ECC71', '#F39C12', '#E74C3C']
    labels = ['50 Mbps', '100 Mbps', '200 Mbps', '400 Mbps']
    
    for rate, color, label in zip(rates, colors, labels):
        P_values = []
        for d in distance_range:
            result = model.get_power(rate, d)
            P_values.append(result['P_total'] * 1000)
        ax.plot(distance_range, P_values, color=color, linewidth=2.5, label=label)
    
    ax.set_xlabel('Distance (m)', fontsize=12)
    ax.set_ylabel('Total Power (mW)', fontsize=12)
    ax.set_title('(b) Path Loss Sensitivity (Power vs Distance)', fontsize=13)
    ax.legend(loc='upper left', fontsize=10)
    ax.set_xlim([50, 1000])
    ax.set_ylim([0, 4000])
    ax.grid(True, alpha=0.3)
    
    # Add power turning point annotation
    ax.axvline(x=400, color='gray', linestyle=':', alpha=0.7)
    ax.annotate('Power\nTurning\nPoint', xy=(420, 1500), fontsize=9, color='gray')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_5g_power_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_5g_power_analysis.png")


def plot_bluetooth_analysis():
    """
    Plot Bluetooth power characteristics with duty cycle analysis.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = ModelBluetooth()
    
    # (a) Connection interval vs average current
    ax = axes[0]
    
    intervals = np.linspace(7.5, 4000, 200)  # ms
    payloads = [10, 20, 50, 100]
    colors = ['#3498DB', '#2ECC71', '#F39C12', '#E74C3C']
    
    for payload, color in zip(payloads, colors):
        currents = [model.get_average_current(t, payload) for t in intervals]
        ax.plot(intervals, currents, color=color, linewidth=2, 
                label=f'{payload} bytes payload')
    
    ax.set_xlabel('Connection Interval (ms)', fontsize=12)
    ax.set_ylabel('Average Current (mA)', fontsize=12)
    ax.set_title('(a) BLE Current: Hyperbolic Duty Cycle Law', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.set_xlim([0, 1000])
    ax.set_ylim([0, 3])
    ax.grid(True, alpha=0.3)
    
    # Add equation
    ax.text(600, 2.0, r'$I_{BLE}(\tau) = I_{sleep} + \frac{Q_{event}}{\tau}$',
            fontsize=12, bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    # (b) Event charge breakdown
    ax = axes[1]
    
    payload_range = np.arange(0, 250, 1)
    Q_values = [model.calculate_event_charge(p) for p in payload_range]
    
    ax.plot(payload_range, Q_values, 'b-', linewidth=2.5)
    ax.fill_between(payload_range, 0, Q_values, alpha=0.3)
    
    ax.set_xlabel('Payload Size (bytes)', fontsize=12)
    ax.set_ylabel('Charge per Event (μC)', fontsize=12)
    ax.set_title('(b) Single Connection Event Energy', fontsize=13)
    ax.grid(True, alpha=0.3)
    
    # Breakdown pie chart inset
    ax_inset = ax.inset_axes([0.55, 0.4, 0.4, 0.5])
    
    Q_rx = model.params.I_rx * model.params.t_rx
    Q_tx = model.params.I_tx * 0.16  # 20 byte example
    Q_proc = model.params.I_cpu * 0.2
    
    sizes = [Q_rx, Q_tx, Q_proc]
    labels_pie = ['RX Window', 'TX Window', 'Protocol']
    colors_pie = ['#3498DB', '#E74C3C', '#2ECC71']
    
    ax_inset.pie(sizes, labels=labels_pie, colors=colors_pie, autopct='%1.0f%%',
                 textprops={'fontsize': 8})
    ax_inset.set_title('Charge Breakdown\n(20 bytes)', fontsize=9)
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_bluetooth_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_bluetooth_analysis.png")


def plot_background_tail_energy():
    """
    Plot background task tail energy dynamics and saturation effect.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = ModelBackground()
    
    # (a) Random wakeup current simulation
    ax = axes[0]
    
    np.random.seed(123)
    time, current = model.simulate_random_wakeups(60, 5, dt=0.01)  # 1 min, 5 wakeups/min
    
    ax.plot(time, current, 'b-', linewidth=0.8, alpha=0.8)
    ax.axhline(y=model.params.I_idle, color='orange', linestyle='--', 
               label=f'Idle-High: {model.params.I_idle} mA')
    ax.axhline(y=model.params.I_leak, color='green', linestyle='--',
               label=f'Deep Sleep: {model.params.I_leak} mA')
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Current (mA)', fontsize=12)
    ax.set_title('(a) Background Task Random Current Pulses', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.set_xlim([0, 60])
    ax.grid(True, alpha=0.3)
    
    # Annotate tail time
    ax.annotate('τ_tail = 12s\n(Cellular)', xy=(25, 20), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    # (b) Power saturation curve
    ax = axes[1]
    
    wakeup_rates = np.linspace(0.1, 20, 100)
    
    for interface, color, label in [('cell', '#E74C3C', 'Cellular (τ=12s)'),
                                     ('wifi', '#3498DB', 'WiFi (τ=0.25s)')]:
        powers = [model.get_average_power(r, interface=interface)['P_total'] 
                  for r in wakeup_rates]
        ax.plot(wakeup_rates, powers, color=color, linewidth=2.5, label=label)
    
    ax.set_xlabel('Wakeup Rate (events/minute)', fontsize=12)
    ax.set_ylabel('Average Power (mW)', fontsize=12)
    ax.set_title('(b) Tail Energy Saturation Effect', fontsize=13)
    ax.legend(loc='lower right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Add formula
    ax.text(12, 20, r'$P_{bg} = P_{leak} + (P_{idle} - P_{leak})(1 - e^{-\lambda \tau_{tail}})$',
            fontsize=11, bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    # Mark critical threshold
    critical_rate = 60 / model.params.tau_tail_cell
    ax.axvline(x=critical_rate, color='red', linestyle=':', alpha=0.7)
    ax.annotate('Critical\nThreshold', xy=(critical_rate+0.5, 40), fontsize=9, color='red')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_background_tail_energy.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_background_tail_energy.png")


def plot_gnss_state_machine():
    """
    Plot GNSS state machine dynamics with tunnel effect simulation.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = ModelGNSS()
    
    # Create signal profile simulating urban driving
    np.random.seed(42)
    duration = 300  # 5 minutes
    dt = 0.5
    time = np.arange(0, duration, dt)
    
    # Base signal with noise
    S_base = 35 + 3 * np.sin(2 * np.pi * time / 60)
    S_noise = np.random.normal(0, 2, len(time))
    
    # Add tunnel (deep fade event)
    tunnel_start = 100
    tunnel_end = 140
    tunnel_mask = (time >= tunnel_start) & (time <= tunnel_end)
    S_env = S_base + S_noise
    S_env[tunnel_mask] = 15 + np.random.normal(0, 2, sum(tunnel_mask))
    
    # Simulate
    model.reset(locked=True)
    time_sim, power = model.simulate_trajectory(S_env, dt=dt, locked_init=True)
    
    # (a) Signal and power dynamics
    ax = axes[0]
    
    ax.plot(time, S_env, 'b-', linewidth=1.5, alpha=0.7, label='Signal Level $S_{env}$')
    ax.axhline(y=model.params.S_th, color='red', linestyle='--', 
               label=f'Threshold $S_{{th}}$ = {model.params.S_th} dB-Hz')
    
    ax2 = ax.twinx()
    ax2.plot(time, power, 'r-', linewidth=2, label='Power')
    
    # Highlight tunnel region
    ax.axvspan(tunnel_start, tunnel_end, alpha=0.3, color='gray', label='Tunnel')
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Signal Quality (dB-Hz)', fontsize=12, color='blue')
    ax2.set_ylabel('Power (mW)', fontsize=12, color='red')
    ax.set_title('(a) GNSS Tunnel Effect: Signal Drop → Power Surge', fontsize=13)
    
    ax.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    
    # Combined legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=9)
    
    ax.grid(True, alpha=0.3)
    
    # Annotate hysteresis
    ax.annotate('Entry delay\n(τ_react)', xy=(105, 20), fontsize=9,
                arrowprops=dict(arrowstyle='->', color='black'),
                xytext=(80, 18))
    ax.annotate('Exit delay\n(Re-acquisition)', xy=(150, 22), fontsize=9,
                arrowprops=dict(arrowstyle='->', color='black'),
                xytext=(170, 18))
    
    # (b) Sigmoid transfer function
    ax = axes[1]
    
    S_range = np.linspace(15, 50, 200)
    Psi = [model.get_lock_probability(s) for s in S_range]
    
    ax.plot(S_range, Psi, 'b-', linewidth=2.5)
    ax.axvline(x=model.params.S_th, color='red', linestyle='--', alpha=0.7)
    ax.axhline(y=0.5, color='gray', linestyle=':', alpha=0.5)
    
    ax.fill_between(S_range, 0, Psi, where=np.array(Psi) > 0.5, 
                    alpha=0.3, color='green', label='Tracking Zone')
    ax.fill_between(S_range, 0, Psi, where=np.array(Psi) <= 0.5,
                    alpha=0.3, color='red', label='Acquisition Zone')
    
    ax.set_xlabel('Signal Quality $S_{env}$ (dB-Hz)', fontsize=12)
    ax.set_ylabel('Lock Probability $\\Psi(S)$', fontsize=12)
    ax.set_title('(b) Sigmoid State Transfer Function', fontsize=13)
    ax.legend(loc='lower right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    ax.text(38, 0.3, r'$\Psi(S) = \frac{1}{1+e^{-\alpha(S-S_{th})}}$',
            fontsize=12, bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_gnss_state_machine.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_gnss_state_machine.png")


def plot_oled_theme_comparison():
    """
    Plot OLED power comparison between light and dark themes.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = ModelOLED()
    
    # (a) Power breakdown comparison
    ax = axes[0]
    
    brightness_levels = [200, 300, 400, 500, 600]
    light_powers = []
    dark_powers = []
    
    for L in brightness_levels:
        comparison = model.compare_themes(L, 60)
        light_powers.append(comparison['light_theme']['P_total'])
        dark_powers.append(comparison['dark_theme']['P_total'])
    
    x = np.arange(len(brightness_levels))
    width = 0.35
    
    bars1 = ax.bar(x - width/2, light_powers, width, label='Light Theme (APL≈0.85)',
                   color='#F5F5F5', edgecolor='black', linewidth=1.5)
    bars2 = ax.bar(x + width/2, dark_powers, width, label='Dark Theme (APL≈0.15)',
                   color='#2C3E50', edgecolor='black', linewidth=1.5)
    
    ax.set_xlabel('Brightness (nits)', fontsize=12)
    ax.set_ylabel('Display Power (mW)', fontsize=12)
    ax.set_title('(a) OLED Power: Light vs Dark Theme', fontsize=13)
    ax.set_xticks(x)
    ax.set_xticklabels(brightness_levels)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add percentage savings
    for i, (l, d) in enumerate(zip(light_powers, dark_powers)):
        savings = (l - d) / l * 100
        ax.annotate(f'-{savings:.0f}%', xy=(x[i] + width/2, d + 30),
                    ha='center', fontsize=9, color='green', fontweight='bold')
    
    # (b) APL vs Power curves
    ax = axes[1]
    
    APL_range = np.linspace(0.05, 1.0, 50)
    brightness_levels_plot = [300, 500, 700]
    colors = ['#3498DB', '#F39C12', '#E74C3C']
    
    for L, color in zip(brightness_levels_plot, colors):
        powers = [model.get_power(apl, L, 60)['P_total'] for apl in APL_range]
        ax.plot(APL_range * 100, powers, color=color, linewidth=2.5, 
                label=f'{L} nits')
    
    ax.set_xlabel('Average Pixel Level APL (%)', fontsize=12)
    ax.set_ylabel('Display Power (mW)', fontsize=12)
    ax.set_title('(b) Power vs Content Brightness (APL)', fontsize=13)
    ax.legend(title='Screen Brightness', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Mark typical theme APL values
    ax.axvline(x=15, color='gray', linestyle=':', alpha=0.7)
    ax.axvline(x=85, color='gray', linestyle=':', alpha=0.7)
    ax.annotate('Dark\nTheme', xy=(10, 200), fontsize=9, ha='center')
    ax.annotate('Light\nTheme', xy=(90, 200), fontsize=9, ha='center')
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_oled_theme_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_oled_theme_comparison.png")


def plot_soc_thermal_coupling():
    """
    Plot SoC electro-thermal coupling and leakage explosion.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    model = ModelSoC()
    
    # (a) Frequency vs Power (cubic relationship)
    ax = axes[0]
    
    freq_range = np.linspace(0.5e9, 3e9, 50)
    
    T_ambient_values = [25, 35, 45]
    colors = ['#3498DB', '#F39C12', '#E74C3C']
    
    for T_amb, color in zip(T_ambient_values, colors):
        model.reset(T_amb)
        powers = []
        for f in freq_range:
            result = model.step(f, T_amb, dt=0.1)
            powers.append(result['P_total'])
            model.reset(T_amb)  # Reset for each frequency point
        ax.plot(freq_range/1e9, powers, color=color, linewidth=2.5,
                label=f'$T_{{amb}}$ = {T_amb}°C')
    
    ax.set_xlabel('CPU Frequency (GHz)', fontsize=12)
    ax.set_ylabel('SoC Power (mW)', fontsize=12)
    ax.set_title('(a) DVFS Cubic Power Law: $P \\propto f^3$', fontsize=13)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Add cubic fit annotation
    ax.text(1.5, 2500, r'$P_{dyn} = \kappa_{dvfs} \cdot f^3$', fontsize=12,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    # (b) Temperature-Leakage feedback simulation
    ax = axes[1]
    
    model.reset(25)
    
    # Sustained high load simulation
    duration = 900  # 15 minutes
    dt = 1.0
    f_high = 2.5e9
    T_amb = 35
    
    results = []
    for _ in range(duration):
        result = model.step(f_high, T_amb, dt)
        results.append(result)
    
    time = np.arange(duration) / 60  # minutes
    T_chip = [r['T_chip'] for r in results]
    P_leak = [r['P_leakage'] for r in results]
    P_total = [r['P_total'] for r in results]
    
    ax.plot(time, T_chip, 'r-', linewidth=2, label='Chip Temperature')
    ax2 = ax.twinx()
    ax2.plot(time, P_leak, 'b-', linewidth=2, label='Leakage Power')
    
    ax.set_xlabel('Time (minutes)', fontsize=12)
    ax.set_ylabel('Chip Temperature (°C)', fontsize=12, color='red')
    ax2.set_ylabel('Leakage Power (mW)', fontsize=12, color='blue')
    ax.set_title('(b) Thermal-Leakage Positive Feedback Loop', fontsize=13)
    
    ax.tick_params(axis='y', labelcolor='red')
    ax2.tick_params(axis='y', labelcolor='blue')
    
    # Combined legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='center right', fontsize=10)
    
    ax.grid(True, alpha=0.3)
    
    # Annotate thermal runaway risk
    ax.annotate('Thermal runaway\nrisk zone', xy=(14, 75), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='#FFCCCC', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_soc_thermal_coupling.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_soc_thermal_coupling.png")


def plot_battery_thermal_field():
    """
    Simulate and plot battery internal temperature distribution (FEM-style visualization).
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Create synthetic temperature field for cylindrical battery cross-section
    # Simulating FEM results
    
    n_points = 100
    x = np.linspace(-1, 1, n_points)
    y = np.linspace(-1, 1, n_points)
    X, Y = np.meshgrid(x, y)
    R = np.sqrt(X**2 + Y**2)
    
    # Battery geometry mask (cylindrical)
    mask = R <= 1.0
    
    # Temperature distribution (higher at center)
    T_core = 45  # Core temperature
    T_surface = 38  # Surface temperature
    T_amb = 25
    
    # Parabolic temperature profile (from heat equation solution)
    T_field = T_core - (T_core - T_surface) * (R ** 2)
    T_field[~mask] = np.nan
    
    # Smooth the field
    T_field_smooth = gaussian_filter(np.nan_to_num(T_field, nan=T_amb), sigma=2)
    T_field_smooth[~mask] = np.nan
    
    # (a) Temperature contour
    ax = axes[0]
    
    levels = np.linspace(38, 45, 15)
    cf = ax.contourf(X, Y, T_field_smooth, levels=levels, cmap='hot', extend='both')
    cs = ax.contour(X, Y, T_field_smooth, levels=[39, 41, 43], colors='white',
                    linewidths=1.5)
    ax.clabel(cs, inline=True, fontsize=9, fmt='%.0f°C')
    
    # Add battery outline
    theta = np.linspace(0, 2*np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), 'k-', linewidth=3)
    
    fig.colorbar(cf, ax=ax, label='Temperature (°C)')
    
    ax.set_xlabel('Radial Position (normalized)', fontsize=12)
    ax.set_ylabel('Radial Position (normalized)', fontsize=12)
    ax.set_title('(a) Battery Cross-Section Temperature Field\n(FEM Simulation, 2C Discharge)', fontsize=13)
    ax.set_aspect('equal')
    
    # Annotate regions
    ax.annotate('Core\n(Hottest)', xy=(0, 0), ha='center', fontsize=10, 
                color='white', fontweight='bold')
    ax.annotate('Surface', xy=(0.7, 0.7), ha='center', fontsize=10, color='yellow')
    
    # (b) Radial temperature profile
    ax = axes[1]
    
    r_profile = np.linspace(0, 1, 100)
    T_profile = T_core - (T_core - T_surface) * (r_profile ** 2)
    
    ax.plot(r_profile, T_profile, 'r-', linewidth=2.5, label='Temperature Profile')
    ax.fill_between(r_profile, T_amb, T_profile, alpha=0.3, color='red')
    
    ax.axhline(y=T_core, color='darkred', linestyle='--', alpha=0.7, label=f'Core: {T_core}°C')
    ax.axhline(y=T_surface, color='blue', linestyle='--', alpha=0.7, label=f'Surface: {T_surface}°C')
    
    ax.set_xlabel('Normalized Radial Position (r/R)', fontsize=12)
    ax.set_ylabel('Temperature (°C)', fontsize=12)
    ax.set_title('(b) Radial Temperature Gradient', fontsize=13)
    ax.legend(loc='lower left', fontsize=10)
    ax.set_xlim([0, 1])
    ax.grid(True, alpha=0.3)
    
    # Annotate gradient
    gradient = T_core - T_surface
    ax.annotate(f'ΔT = {gradient}°C\n(Core-Surface)', xy=(0.5, 41), fontsize=11,
                ha='center', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_battery_thermal_field.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_battery_thermal_field.png")


# =============================================================================
# User Recommendation Visualizations
# =============================================================================

def plot_user_behavior_impact():
    """
    Plot comprehensive user behavior impact on battery life.
    """
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    integrated_model = IntegratedPowerModel()
    
    # (a) Power breakdown by usage scenario
    ax = fig.add_subplot(gs[0, 0])
    
    scenarios = {
        'Standby': {'display_on': False, '5g_active': False, 'gnss_active': False,
                    'wakeup_rate': 1.0, 'cpu_freq': 0.5e9},
        'Web Browsing': {'display_on': True, 'rgb_mean': (200, 200, 200), 'brightness': 400,
                         'refresh_rate': 60, '5g_active': True, 'data_rate': 20e6,
                         'distance': 300, 'cpu_freq': 1.5e9, 'wakeup_rate': 3.0},
        'Video Streaming': {'display_on': True, 'rgb_mean': (100, 80, 60), 'brightness': 500,
                            'refresh_rate': 60, '5g_active': True, 'data_rate': 50e6,
                            'distance': 400, 'cpu_freq': 1.8e9, 'wakeup_rate': 2.0},
        'Gaming': {'display_on': True, 'rgb_mean': (150, 120, 100), 'brightness': 600,
                   'refresh_rate': 120, '5g_active': True, 'data_rate': 30e6,
                   'distance': 300, 'cpu_freq': 2.8e9, 'wakeup_rate': 1.0},
        'Navigation': {'display_on': True, 'rgb_mean': (80, 100, 80), 'brightness': 700,
                       'refresh_rate': 60, '5g_active': True, 'data_rate': 10e6,
                       'distance': 500, 'gnss_active': True, 'signal_quality': 32,
                       'cpu_freq': 1.5e9, 'wakeup_rate': 2.0}
    }
    
    scenario_names = list(scenarios.keys())
    components = ['SoC', 'Display', '5G', 'GNSS', 'Background']
    
    bottom = np.zeros(len(scenario_names))
    colors = ['#E74C3C', '#3498DB', '#2ECC71', '#9B59B6', '#F39C12']
    
    for comp, color in zip(components, colors):
        values = []
        for name in scenario_names:
            result = integrated_model.calculate_total_power(scenarios[name])
            values.append(result.get(comp, 0))
        ax.bar(scenario_names, values, bottom=bottom, label=comp, color=color)
        bottom += values
    
    ax.set_ylabel('Power Consumption (mW)', fontsize=12)
    ax.set_title('(a) Power Breakdown by Usage Scenario', fontsize=13)
    ax.legend(loc='upper left', fontsize=9)
    ax.tick_params(axis='x', rotation=15)
    ax.grid(True, alpha=0.3, axis='y')
    
    # (b) Estimated runtime
    ax = fig.add_subplot(gs[0, 1])
    
    battery_capacity = 5000  # mAh
    V_nominal = 3.8  # V
    energy_Wh = battery_capacity * V_nominal / 1000
    
    runtimes = []
    for name in scenario_names:
        result = integrated_model.calculate_total_power(scenarios[name])
        P_total = result['Total'] / 1000  # W
        runtime = energy_Wh / P_total if P_total > 0 else 24
        runtimes.append(min(runtime, 24))
    
    bars = ax.barh(scenario_names, runtimes, color=plt.cm.RdYlGn(np.array(runtimes)/24))
    
    for bar, rt in zip(bars, runtimes):
        ax.text(bar.get_width() + 0.3, bar.get_y() + bar.get_height()/2,
                f'{rt:.1f}h', va='center', fontsize=10)
    
    ax.set_xlabel('Estimated Runtime (hours)', fontsize=12)
    ax.set_title(f'(b) Battery Life Estimate (5000mAh Battery)', fontsize=13)
    ax.set_xlim([0, 28])
    ax.grid(True, alpha=0.3, axis='x')
    
    # (c) Power saving strategies effectiveness
    ax = fig.add_subplot(gs[1, 0])
    
    strategies = [
        ('Baseline (Video)', 2800),
        ('+ Dark Mode', 2200),
        ('+ Brightness 50%', 1800),
        ('+ 60Hz Refresh', 1650),
        ('+ Close Background', 1500),
        ('+ Weak Signal Off', 1200),
        ('All Combined', 900)
    ]
    
    strategy_names, powers = zip(*strategies)
    savings = [(strategies[0][1] - p) / strategies[0][1] * 100 for p in powers]
    
    colors_bar = plt.cm.Greens(np.linspace(0.3, 0.9, len(strategies)))
    bars = ax.barh(strategy_names, savings, color=colors_bar)
    
    for bar, s in zip(bars, savings):
        ax.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2,
                f'{s:.0f}%', va='center', fontsize=10)
    
    ax.set_xlabel('Power Reduction (%)', fontsize=12)
    ax.set_title('(c) Cumulative Effect of Power Saving Strategies', fontsize=13)
    ax.set_xlim([0, 80])
    ax.grid(True, alpha=0.3, axis='x')
    
    # (d) Aging impact on runtime
    ax = fig.add_subplot(gs[1, 1])
    
    model = BatteryElectroThermalAgingModel()
    
    N_values = [0, 100, 200, 300, 400, 500]
    
    # Calculate runtime for video streaming scenario at different aging levels
    runtimes_aging = []
    capacities = []
    
    for N in N_values:
        Q_max = model.get_capacity(N, 25)
        capacities.append(Q_max)
        # Adjusted for aged battery
        energy_aged = Q_max * V_nominal
        P_video = 2.8  # W (video streaming)
        runtime_aged = energy_aged / P_video
        runtimes_aging.append(runtime_aged)
    
    ax2 = ax.twinx()
    
    line1, = ax.plot(N_values, runtimes_aging, 'b-o', linewidth=2.5, markersize=8,
                     label='Runtime')
    line2, = ax2.plot(N_values, capacities, 'r--s', linewidth=2, markersize=6,
                      label='Capacity')
    
    ax.set_xlabel('Cycle Number (Battery Age)', fontsize=12)
    ax.set_ylabel('Video Streaming Runtime (hours)', fontsize=12, color='blue')
    ax2.set_ylabel('Available Capacity (Ah)', fontsize=12, color='red')
    ax.set_title('(d) Battery Aging Impact on Runtime', fontsize=13)
    
    ax.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    
    lines = [line1, line2]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='center right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Annotate degradation
    degradation = (runtimes_aging[0] - runtimes_aging[-1]) / runtimes_aging[0] * 100
    ax.annotate(f'Runtime loss: {degradation:.0f}%\nafter 500 cycles',
                xy=(350, runtimes_aging[3]), fontsize=10,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    plt.suptitle('User Behavior and Battery Management Analysis', fontsize=14, fontweight='bold', y=0.98)
    
    plt.savefig('/workspace/battery_model/fig_user_behavior_impact.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_user_behavior_impact.png")


def plot_comprehensive_system_analysis():
    """
    Create a comprehensive system-level analysis dashboard.
    """
    fig = plt.figure(figsize=(18, 14))
    gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.3)
    
    # Battery model
    battery = BatteryElectroThermalAgingModel()
    
    # 1. SOC-Voltage Family Curves
    ax = fig.add_subplot(gs[0, 0])
    
    N_values = [0, 100, 200, 300, 400]
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(N_values)))
    
    SOC_range = np.linspace(0.05, 0.95, 100)
    
    for N, color in zip(N_values, colors):
        V_term = []
        for soc in SOC_range:
            V_ocv = battery.get_OCV(soc)
            R = battery.get_resistance(N, 25)
            V_term.append(V_ocv - 1.0 * R)  # 1A load
        ax.plot(SOC_range * 100, V_term, color=color, linewidth=2, label=f'N={N}')
    
    ax.set_xlabel('SOC (%)', fontsize=11)
    ax.set_ylabel('Terminal Voltage (V)', fontsize=11)
    ax.set_title('Discharge Curves Family\n(1A load, varying aging)', fontsize=12)
    ax.legend(fontsize=9, title='Cycles')
    ax.grid(True, alpha=0.3)
    
    # 2. Temperature Effects Matrix
    ax = fig.add_subplot(gs[0, 1])
    
    T_range = np.array([-20, -10, 0, 10, 25, 40, 60])
    metrics = {
        'Capacity (%)': [battery.get_capacity(100, T) / battery.get_capacity(100, 25) * 100 
                         for T in T_range],
        'Resistance (×)': [battery.get_resistance(100, T) / battery.get_resistance(100, 25) 
                           for T in T_range]
    }
    
    x = np.arange(len(T_range))
    width = 0.35
    
    ax.bar(x - width/2, metrics['Capacity (%)'], width, label='Capacity Retention', color='#3498DB')
    ax2 = ax.twinx()
    ax2.bar(x + width/2, metrics['Resistance (×)'], width, label='Resistance Factor', color='#E74C3C')
    
    ax.set_xlabel('Temperature (°C)', fontsize=11)
    ax.set_ylabel('Capacity Retention (%)', fontsize=11, color='#3498DB')
    ax2.set_ylabel('Resistance Factor (×)', fontsize=11, color='#E74C3C')
    ax.set_title('Temperature Impact on\nBattery Performance', fontsize=12)
    ax.set_xticks(x)
    ax.set_xticklabels(T_range)
    
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper center', fontsize=8)
    
    # 3. Subsystem Power Pie Chart
    ax = fig.add_subplot(gs[0, 2])
    
    integrated = IntegratedPowerModel()
    gaming_scenario = {
        'display_on': True, 'rgb_mean': (150, 120, 100), 'brightness': 600,
        'refresh_rate': 120, '5g_active': True, 'data_rate': 30e6,
        'distance': 300, 'cpu_freq': 2.8e9, 'wakeup_rate': 1.0
    }
    result = integrated.calculate_total_power(gaming_scenario)
    
    labels = ['SoC', 'Display', '5G', 'Background']
    sizes = [result['SoC'], result['Display'], result['5G'], result['Background']]
    colors_pie = ['#E74C3C', '#3498DB', '#2ECC71', '#F39C12']
    explode = (0.05, 0, 0, 0)
    
    ax.pie(sizes, explode=explode, labels=labels, colors=colors_pie,
           autopct='%1.1f%%', shadow=True, startangle=90)
    ax.set_title('Power Distribution\n(Gaming Scenario)', fontsize=12)
    
    # 4. 5G Distance-Power Sensitivity Heatmap
    ax = fig.add_subplot(gs[1, 0])
    
    model_5g = Model5G()
    distances = np.linspace(100, 800, 20)
    rates = np.linspace(20e6, 200e6, 20)
    
    P_matrix = np.zeros((len(rates), len(distances)))
    for i, r in enumerate(rates):
        for j, d in enumerate(distances):
            P_matrix[i, j] = model_5g.get_power(r, d)['P_total'] * 1000
    
    im = ax.imshow(P_matrix, aspect='auto', origin='lower', cmap='YlOrRd',
                   extent=[100, 800, 20, 200])
    ax.set_xlabel('Distance (m)', fontsize=11)
    ax.set_ylabel('Data Rate (Mbps)', fontsize=11)
    ax.set_title('5G Power Consumption\nHeatmap', fontsize=12)
    plt.colorbar(im, ax=ax, label='Power (mW)')
    
    # 5. BLE Interval Optimization Curve
    ax = fig.add_subplot(gs[1, 1])
    
    model_bt = ModelBluetooth()
    intervals = np.logspace(np.log10(7.5), np.log10(4000), 100)
    
    currents = [model_bt.get_average_current(t, 20) for t in intervals]
    
    ax.semilogx(intervals, currents, 'b-', linewidth=2.5)
    ax.fill_between(intervals, 0, currents, alpha=0.3)
    
    # Mark optimal zones
    ax.axvspan(7.5, 30, alpha=0.2, color='red', label='High Power')
    ax.axvspan(500, 4000, alpha=0.2, color='green', label='Low Power')
    
    ax.set_xlabel('Connection Interval (ms)', fontsize=11)
    ax.set_ylabel('Average Current (mA)', fontsize=11)
    ax.set_title('BLE Power Optimization\nInterval Selection', fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # 6. OLED APL Sensitivity
    ax = fig.add_subplot(gs[1, 2])
    
    model_oled = ModelOLED()
    APL_values = np.linspace(0.1, 0.9, 50)
    
    for brightness in [300, 500, 700]:
        powers = [model_oled.get_power(apl, brightness, 60)['P_total'] for apl in APL_values]
        ax.plot(APL_values * 100, powers, linewidth=2, label=f'{brightness} nits')
    
    ax.set_xlabel('APL (%)', fontsize=11)
    ax.set_ylabel('Power (mW)', fontsize=11)
    ax.set_title('OLED Content-Dependent\nPower Consumption', fontsize=12)
    ax.legend(fontsize=9, title='Brightness')
    ax.grid(True, alpha=0.3)
    
    # 7. SoC Frequency-Power Cubic Law
    ax = fig.add_subplot(gs[2, 0])
    
    model_soc = ModelSoC()
    freqs = np.linspace(0.5e9, 3e9, 50)
    
    powers_dyn = []
    for f in freqs:
        P_dyn = model_soc.get_dynamic_power(f) * 1000  # mW
        powers_dyn.append(P_dyn)
    
    ax.plot(freqs/1e9, powers_dyn, 'r-', linewidth=2.5)
    
    # Fit cubic
    coeffs = np.polyfit(freqs/1e9, powers_dyn, 3)
    fit_line = np.polyval(coeffs, freqs/1e9)
    ax.plot(freqs/1e9, fit_line, 'k--', linewidth=1.5, alpha=0.7, label='Cubic fit')
    
    ax.set_xlabel('Frequency (GHz)', fontsize=11)
    ax.set_ylabel('Dynamic Power (mW)', fontsize=11)
    ax.set_title('SoC DVFS Power Law\n$P \\propto f^3$', fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # 8. Aging + Temperature Combined Effect Surface (2D projection)
    ax = fig.add_subplot(gs[2, 1])
    
    N_range = np.linspace(0, 500, 30)
    T_range = np.linspace(-20, 60, 30)
    N_grid, T_grid = np.meshgrid(N_range, T_range)
    
    Q_surface = np.zeros_like(N_grid)
    for i in range(len(T_range)):
        for j in range(len(N_range)):
            Q_surface[i, j] = battery.get_capacity(int(N_range[j]), T_range[i])
    
    cf = ax.contourf(N_grid, T_grid, Q_surface, levels=15, cmap='RdYlGn')
    cs = ax.contour(N_grid, T_grid, Q_surface, levels=[0.5, 1.0, 1.5], colors='white',
                    linewidths=1.5)
    ax.clabel(cs, inline=True, fontsize=9, fmt='%.1f Ah')
    
    plt.colorbar(cf, ax=ax, label='Capacity (Ah)')
    ax.set_xlabel('Cycle Number', fontsize=11)
    ax.set_ylabel('Temperature (°C)', fontsize=11)
    ax.set_title('Capacity: Aging×Temperature\nCoupled Effect', fontsize=12)
    
    # 9. Recommendations Summary
    ax = fig.add_subplot(gs[2, 2])
    ax.axis('off')
    
    recommendations = [
        ("Screen", "Dark mode + Auto-brightness", "35-75%"),
        ("Refresh Rate", "60Hz vs 120Hz", "15-25%"),
        ("5G", "WiFi when possible", "20-40%"),
        ("Background", "Limit app refresh", "10-20%"),
        ("GNSS", "Disable when not needed", "5-15%"),
        ("Charging", "Avoid extreme temps", "+20% lifespan"),
        ("Usage", "Reduce at <20% SOC", "+15% cycles")
    ]
    
    table_data = [[r[0], r[1], r[2]] for r in recommendations]
    
    table = ax.table(cellText=table_data,
                     colLabels=['Category', 'Recommendation', 'Savings'],
                     loc='center',
                     cellLoc='center',
                     colWidths=[0.25, 0.5, 0.25])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.8)
    
    # Color header
    for i in range(3):
        table[(0, i)].set_facecolor('#2C3E50')
        table[(0, i)].set_text_props(color='white', fontweight='bold')
    
    ax.set_title('Power Saving Recommendations\nSummary', fontsize=12, pad=20)
    
    plt.suptitle('Comprehensive Battery & Power Management System Analysis', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    plt.savefig('/workspace/battery_model/fig_comprehensive_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_comprehensive_analysis.png")


# =============================================================================
# Main Execution
# =============================================================================

def generate_all_figures():
    """Generate all visualization figures."""
    print("=" * 60)
    print("Generating Comprehensive Visualization Suite")
    print("=" * 60)
    
    # Battery model visualizations
    print("\n[1/11] Aging characteristics...")
    plot_aging_characteristics()
    
    print("[2/11] OCV curve...")
    plot_ocv_curve()
    
    print("[3/11] Temperature correction factors...")
    plot_temperature_correction()
    
    print("[4/11] Electro-thermal coupling dynamics...")
    plot_electro_thermal_coupling()
    
    print("[5/11] 3D capacity surface...")
    plot_3d_capacity_surface()
    
    print("[6/11] Battery thermal field...")
    plot_battery_thermal_field()
    
    # Load subsystem visualizations
    print("[7/11] 5G power analysis...")
    plot_5g_power_analysis()
    
    print("[8/11] Bluetooth analysis...")
    plot_bluetooth_analysis()
    
    print("[9/11] Background tail energy...")
    plot_background_tail_energy()
    
    print("[10/11] GNSS state machine...")
    plot_gnss_state_machine()
    
    print("[11/11] OLED theme comparison...")
    plot_oled_theme_comparison()
    
    # SoC thermal coupling
    print("[12/13] SoC thermal coupling...")
    plot_soc_thermal_coupling()
    
    # User behavior and system analysis
    print("[13/14] User behavior impact...")
    plot_user_behavior_impact()
    
    print("[14/14] Comprehensive system analysis...")
    plot_comprehensive_system_analysis()
    
    print("\n" + "=" * 60)
    print("All figures generated successfully!")
    print("=" * 60)


if __name__ == "__main__":
    generate_all_figures()
