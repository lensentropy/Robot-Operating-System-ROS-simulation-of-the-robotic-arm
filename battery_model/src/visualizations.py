"""
SCI-Level Visualization Module for Battery Discharge Model
=========================================================

This module generates publication-quality figures for the smartphone
battery discharge model, suitable for academic papers.

Figure specifications follow IEEE/Nature/Science guidelines:
- High resolution (300+ DPI)
- Colorblind-friendly palettes
- Clear axis labels with units
- Appropriate font sizes

Key visualizations:
1. 5G power consumption surface (rate vs distance)
2. GPS state machine dynamics
3. Background task stochastic analysis
4. Bluetooth comparison charts
5. Coupled system simulation results
6. SOC evolution curves
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import gaussian_kde
import warnings

# Configure matplotlib for publication quality
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.linewidth': 0.8,
    'grid.linewidth': 0.5,
    'lines.linewidth': 1.5,
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Colorblind-friendly palette
COLORS = {
    'blue': '#0077BB',
    'orange': '#EE7733',
    'green': '#009988',
    'red': '#CC3311',
    'purple': '#AA3377',
    'cyan': '#33BBEE',
    'yellow': '#CCBB44',
    'grey': '#BBBBBB'
}

# Scientific color maps
CMAP_POWER = LinearSegmentedColormap.from_list(
    'power', ['#f7fbff', '#6baed6', '#2171b5', '#08306b']
)
CMAP_DIVERGING = plt.cm.RdYlBu_r


def figure_5g_power_analysis(save_path: str = None):
    """
    Figure 1: 5G Module Power Consumption Analysis
    
    Panel A: 3D surface of power vs (distance, data rate)
    Panel B: Distance sensitivity at fixed rates
    """
    # Import module
    import sys
    sys.path.insert(0, '/workspace/battery_model/src')
    from network_5g_module import Network5GModule, analyze_5g_power_sensitivity
    
    module = Network5GModule()
    data = analyze_5g_power_sensitivity()
    
    fig = plt.figure(figsize=(12, 5))
    
    # Panel A: 3D Surface
    ax1 = fig.add_subplot(121, projection='3d')
    
    surf = ax1.plot_surface(
        data['D_mesh'],
        data['R_mesh'] / 1e6,  # Convert to Mbps
        data['power'] * 1000,  # Convert to mW
        cmap=CMAP_POWER,
        alpha=0.9,
        edgecolor='none'
    )
    
    ax1.set_xlabel('Distance to BS (m)', fontsize=10, labelpad=8)
    ax1.set_ylabel('Data Rate (Mbps)', fontsize=10, labelpad=8)
    ax1.set_zlabel('Power (mW)', fontsize=10, labelpad=8)
    ax1.set_title('(a) 5G Module Power Consumption', fontsize=11, fontweight='bold')
    ax1.view_init(elev=25, azim=45)
    
    # Add colorbar
    cbar = fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, pad=0.1)
    cbar.set_label('Power (mW)', fontsize=9)
    
    # Panel B: 2D curves for different data rates
    ax2 = fig.add_subplot(122)
    
    distances = data['distances']
    rates_to_plot = [5e6, 20e6, 50e6, 100e6]
    rate_labels = ['5 Mbps', '20 Mbps', '50 Mbps', '100 Mbps']
    colors_line = [COLORS['blue'], COLORS['green'], COLORS['orange'], COLORS['red']]
    
    for rate, label, color in zip(rates_to_plot, rate_labels, colors_line):
        power = [module.total_power(rate, d) * 1000 for d in distances]
        ax2.plot(distances, power, label=label, color=color, linewidth=2)
    
    # Mark critical region
    ax2.axvspan(600, 1000, alpha=0.15, color=COLORS['red'], label='Cell Edge')
    
    ax2.set_xlabel('Distance to Base Station (m)', fontsize=10)
    ax2.set_ylabel('Power Consumption (mW)', fontsize=10)
    ax2.set_title('(b) Power vs Distance at Fixed Data Rates', fontsize=11, fontweight='bold')
    ax2.legend(loc='upper left', framealpha=0.9)
    ax2.set_xlim([50, 1000])
    ax2.set_ylim([0, 800])
    
    # Add annotation for exponential region
    ax2.annotate('Exponential\nIncrease',
                xy=(750, 500), xytext=(500, 600),
                fontsize=9, ha='center',
                arrowprops=dict(arrowstyle='->', color='black', lw=1))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def figure_gnss_state_dynamics(save_path: str = None):
    """
    Figure 2: GNSS State Machine Dynamics
    
    Panel A: SNR vs Power with state transitions
    Panel B: Time-domain example with environment changes
    """
    import sys
    sys.path.insert(0, '/workspace/battery_model/src')
    from gnss_module import GNSSModule, GNSSEnvironmentModel, analyze_gnss_state_dynamics
    
    module = GNSSModule()
    data = analyze_gnss_state_dynamics()
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    
    # Panel A: State weights and power vs SNR
    ax1 = axes[0]
    
    snr = data['snr_range']
    weights = data['weights']
    power = data['power'] * 1000  # mW
    
    # Stack plot for state weights
    colors_states = [COLORS['grey'], COLORS['red'], COLORS['orange'], COLORS['green']]
    ax1.stackplot(snr, weights.T, labels=data['state_names'], 
                  colors=colors_states, alpha=0.7)
    
    # Overlay power curve on twin axis
    ax1_twin = ax1.twinx()
    ax1_twin.plot(snr, power, 'k-', linewidth=2.5, label='Power')
    ax1_twin.set_ylabel('Power (mW)', fontsize=10, color='black')
    ax1_twin.tick_params(axis='y', colors='black')
    ax1_twin.set_ylim([0, 200])
    
    ax1.set_xlabel('Signal-to-Noise Ratio (dB-Hz)', fontsize=10)
    ax1.set_ylabel('State Occupation Probability', fontsize=10)
    ax1.set_title('(a) GNSS State Transition Model', fontsize=11, fontweight='bold')
    ax1.set_xlim([10, 50])
    ax1.set_ylim([0, 1])
    ax1.legend(loc='center left', fontsize=8)
    
    # Add threshold annotations
    ax1.axvline(x=25, color='black', linestyle='--', alpha=0.5, linewidth=1)
    ax1.axvline(x=30, color='black', linestyle='--', alpha=0.5, linewidth=1)
    ax1.text(25.5, 0.95, 'Track\nThreshold', fontsize=7, va='top')
    ax1.text(30.5, 0.95, 'Acq\nThreshold', fontsize=7, va='top')
    
    # Panel B: Time-domain simulation
    ax2 = axes[1]
    
    # Simulate subway scenario
    t = np.linspace(0, 0.3, 1000)  # 18 minutes
    snr_func = GNSSEnvironmentModel.subway_commute()
    
    snr_t = np.array([snr_func(ti) for ti in t])
    power_t = np.array([module.power_consumption(snr_func(ti), requested=True) * 1000 
                        for ti in t])
    
    # Plot SNR
    ax2.plot(t * 60, snr_t, color=COLORS['blue'], linewidth=1.5, label='SNR')
    ax2.set_ylabel('SNR (dB-Hz)', fontsize=10, color=COLORS['blue'])
    ax2.tick_params(axis='y', labelcolor=COLORS['blue'])
    ax2.set_ylim([0, 50])
    
    # Plot power on twin axis
    ax2_twin = ax2.twinx()
    ax2_twin.fill_between(t * 60, 0, power_t, alpha=0.3, color=COLORS['red'])
    ax2_twin.plot(t * 60, power_t, color=COLORS['red'], linewidth=1.5, label='Power')
    ax2_twin.set_ylabel('Power (mW)', fontsize=10, color=COLORS['red'])
    ax2_twin.tick_params(axis='y', labelcolor=COLORS['red'])
    ax2_twin.set_ylim([0, 200])
    
    ax2.set_xlabel('Time (minutes)', fontsize=10)
    ax2.set_title('(b) Subway Commute Scenario', fontsize=11, fontweight='bold')
    
    # Add station markers
    for i in range(6):
        station_time = i * 3
        ax2.axvline(x=station_time, color='black', linestyle=':', alpha=0.3)
    ax2.text(1.5, 45, 'Station', fontsize=7, ha='center', alpha=0.7)
    ax2.text(4.5, 5, 'Tunnel', fontsize=7, ha='center', alpha=0.7)
    
    # Combined legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def figure_background_stochastic(save_path: str = None):
    """
    Figure 3: Background Task Stochastic Analysis
    
    Panel A: Sample path with burst events highlighted
    Panel B: Heavy-tail distribution histogram
    """
    import sys
    sys.path.insert(0, '/workspace/battery_model/src')
    from background_tasks_module import BackgroundTasksModule, analyze_background_statistics
    
    data = analyze_background_statistics()
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    
    # Panel A: Time series
    ax1 = axes[0]
    
    t = data['time']
    I = data['current'] * 1000  # Convert to mA
    burst_times = data['burst_times']
    
    # Plot current trace
    ax1.plot(t * 60, I, color=COLORS['blue'], linewidth=0.5, alpha=0.8)
    
    # Highlight burst events
    burst_mask = np.zeros_like(t, dtype=bool)
    for bt in burst_times:
        burst_mask |= np.abs(t - bt) < 0.002
    
    ax1.fill_between(t * 60, 0, I, where=I > np.percentile(I, 95),
                     alpha=0.4, color=COLORS['red'], label='Burst events')
    
    # Add mean line
    ax1.axhline(y=np.mean(I), color=COLORS['orange'], linestyle='--', 
                linewidth=2, label=f'Mean = {np.mean(I):.1f} mA')
    
    # Add 95th percentile
    p95 = np.percentile(I, 95)
    ax1.axhline(y=p95, color=COLORS['red'], linestyle=':', 
                linewidth=1.5, label=f'95th %ile = {p95:.1f} mA')
    
    ax1.set_xlabel('Time (minutes)', fontsize=10)
    ax1.set_ylabel('Background Current (mA)', fontsize=10)
    ax1.set_title('(a) Background Task Current with Burst Events', fontsize=11, fontweight='bold')
    ax1.set_xlim([0, 20])  # Show first 20 minutes
    ax1.set_ylim([0, max(I) * 1.1])
    ax1.legend(loc='upper right', fontsize=8)
    
    # Panel B: Distribution
    ax2 = axes[1]
    
    # Histogram
    bins = np.linspace(0, 250, 80)
    ax2.hist(I, bins=bins, density=True, alpha=0.7, color=COLORS['blue'],
             edgecolor='white', linewidth=0.5, label='Empirical')
    
    # Fit and plot KDE
    kde = gaussian_kde(I)
    x_kde = np.linspace(0, 250, 200)
    ax2.plot(x_kde, kde(x_kde), color=COLORS['red'], linewidth=2, label='KDE fit')
    
    # Add statistics annotation
    stats = data['statistics']
    textstr = '\n'.join([
        f"Mean: {stats['mean']*1000:.1f} mA",
        f"Std: {stats['std']*1000:.1f} mA",
        f"Skewness: {stats['skew']:.2f}",
        f"Kurtosis: {stats['kurtosis']:.2f}"
    ])
    props = dict(boxstyle='round', facecolor='white', alpha=0.8)
    ax2.text(0.95, 0.95, textstr, transform=ax2.transAxes, fontsize=9,
             verticalalignment='top', horizontalalignment='right', bbox=props)
    
    # Mark tail region
    ax2.axvspan(p95, 250, alpha=0.2, color=COLORS['red'])
    ax2.annotate('Heavy Tail\n(Burst Events)', xy=(180, 0.005), fontsize=9, ha='center')
    
    ax2.set_xlabel('Current (mA)', fontsize=10)
    ax2.set_ylabel('Probability Density', fontsize=10)
    ax2.set_title('(b) Current Distribution - Heavy Tail Characteristic', fontsize=11, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.set_xlim([0, 250])
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def figure_bluetooth_analysis(save_path: str = None):
    """
    Figure 4: Bluetooth Module Power Analysis
    
    Panel A: Scenario comparison bar chart
    Panel B: BLE connection interval impact
    Panel C: Audio codec comparison
    """
    import sys
    sys.path.insert(0, '/workspace/battery_model/src')
    from bluetooth_module import BluetoothModule, analyze_bluetooth_power_breakdown
    
    data = analyze_bluetooth_power_breakdown()
    
    fig = plt.figure(figsize=(14, 4.5))
    gs = GridSpec(1, 3, figure=fig, width_ratios=[1.2, 1, 1])
    
    # Panel A: Scenario comparison
    ax1 = fig.add_subplot(gs[0])
    
    scenarios = ['TWS Music', 'Smartwatch', 'Fitness Tracker', 'Car Audio', 'Idle Discoverable']
    powers = [data[s]['power_mW'] for s in scenarios]
    
    bars = ax1.barh(scenarios, powers, color=[COLORS['blue'], COLORS['green'], 
                    COLORS['cyan'], COLORS['orange'], COLORS['grey']])
    
    # Add value labels
    for bar, power in zip(bars, powers):
        ax1.text(power + 2, bar.get_y() + bar.get_height()/2, f'{power:.1f}',
                va='center', fontsize=9)
    
    ax1.set_xlabel('Power Consumption (mW)', fontsize=10)
    ax1.set_title('(a) Bluetooth Usage Scenarios', fontsize=11, fontweight='bold')
    ax1.set_xlim([0, max(powers) * 1.15])
    
    # Panel B: Connection interval
    ax2 = fig.add_subplot(gs[1])
    
    intervals = data['interval_sweep']['intervals']
    ble_power = data['interval_sweep']['power_mW']
    
    ax2.semilogy(intervals, ble_power, 'o-', color=COLORS['blue'], 
                 linewidth=2, markersize=6)
    
    # Highlight typical ranges
    ax2.axvspan(7.5, 50, alpha=0.15, color=COLORS['green'], label='Low latency')
    ax2.axvspan(100, 500, alpha=0.15, color=COLORS['orange'], label='Balanced')
    ax2.axvspan(1000, 4000, alpha=0.15, color=COLORS['cyan'], label='Power saving')
    
    ax2.set_xlabel('Connection Interval (ms)', fontsize=10)
    ax2.set_ylabel('Power (mW)', fontsize=10)
    ax2.set_title('(b) BLE Connection Interval Impact', fontsize=11, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=7)
    ax2.set_xscale('log')
    
    # Panel C: Codec comparison
    ax3 = fig.add_subplot(gs[2])
    
    codecs = data['codec_comparison']['codecs']
    codec_power = data['codec_comparison']['power_mW']
    
    colors_codec = [COLORS['grey'], COLORS['blue'], COLORS['green'], 
                    COLORS['orange'], COLORS['red'], COLORS['purple']]
    
    bars = ax3.bar(codecs, codec_power, color=colors_codec, edgecolor='white', linewidth=1)
    
    # Add quality/efficiency annotations
    qualities = ['Basic', 'Good', 'High', 'Very High', 'Excellent', 'Efficient']
    for bar, quality in zip(bars, qualities):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                quality, ha='center', va='bottom', fontsize=7, rotation=45)
    
    ax3.set_xlabel('Audio Codec', fontsize=10)
    ax3.set_ylabel('Total Power (mW)', fontsize=10)
    ax3.set_title('(c) Audio Codec Power Comparison', fontsize=11, fontweight='bold')
    ax3.set_xticklabels(codecs, rotation=30, ha='right')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def figure_coupled_system_simulation(results: dict = None, save_path: str = None):
    """
    Figure 5: Coupled System Simulation Results
    
    Panel A: SOC evolution for different scenarios
    Panel B: Power breakdown stacked area
    Panel C: Temperature evolution
    """
    if results is None:
        # Generate results if not provided
        import sys
        sys.path.insert(0, '/workspace/battery_model/src')
        from coupled_system import CoupledBatterySystem, MixedUsageScenario
        
        system = CoupledBatterySystem()
        scenario = MixedUsageScenario()
        results = {'Mixed Daily Use': system.simulate(scenario, t_span=(0, 12), dt=0.005)}
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(2, 2, figure=fig, height_ratios=[1, 1])
    
    # Panel A: SOC evolution
    ax1 = fig.add_subplot(gs[0, 0])
    
    colors_scenarios = [COLORS['green'], COLORS['blue'], COLORS['orange'], 
                       COLORS['red'], COLORS['purple']]
    
    for (name, result), color in zip(results.items(), colors_scenarios):
        t = result['time']
        soc = result['SOC'] * 100
        ax1.plot(t, soc, label=name, color=color, linewidth=2)
    
    ax1.set_xlabel('Time (hours)', fontsize=10)
    ax1.set_ylabel('State of Charge (%)', fontsize=10)
    ax1.set_title('(a) Battery SOC Evolution', fontsize=11, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_xlim([0, max([r['time'][-1] for r in results.values()]) * 1.05])
    ax1.set_ylim([0, 105])
    
    # Add discharge milestones
    ax1.axhline(y=20, color='red', linestyle=':', alpha=0.5, linewidth=1)
    ax1.text(0.2, 22, 'Low Battery', fontsize=8, color='red', alpha=0.7)
    
    # Panel B: Power breakdown (for mixed usage)
    ax2 = fig.add_subplot(gs[0, 1])
    
    if 'Mixed Daily Use' in results:
        result = results['Mixed Daily Use']
        t = result['time']
        bd = result['breakdown']
        
        # Stack the components
        components = ['screen', 'cpu', '5g', 'gnss', 'bluetooth', 'background']
        component_labels = ['Screen', 'CPU', '5G Network', 'GNSS', 'Bluetooth', 'Background']
        component_colors = [COLORS['yellow'], COLORS['red'], COLORS['blue'],
                           COLORS['green'], COLORS['purple'], COLORS['grey']]
        
        # Convert to power (mW)
        V = result['voltage']
        data_stack = [(bd[c] * V * 1000) for c in components]
        
        ax2.stackplot(t, data_stack, labels=component_labels, 
                      colors=component_colors, alpha=0.8)
        
        ax2.set_xlabel('Time (hours)', fontsize=10)
        ax2.set_ylabel('Power (mW)', fontsize=10)
        ax2.set_title('(b) Power Consumption Breakdown', fontsize=11, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=8, ncol=2)
        ax2.set_xlim([0, t[-1]])
    
    # Panel C: Temperature evolution
    ax3 = fig.add_subplot(gs[1, 0])
    
    for (name, result), color in zip(results.items(), colors_scenarios):
        t = result['time']
        temp = result['temperature'] - 273.15  # Convert to Celsius
        ax3.plot(t, temp, label=name, color=color, linewidth=2)
    
    ax3.set_xlabel('Time (hours)', fontsize=10)
    ax3.set_ylabel('Temperature (°C)', fontsize=10)
    ax3.set_title('(c) Battery Temperature Evolution', fontsize=11, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=8)
    
    # Add thermal warning zone
    ax3.axhspan(40, 50, alpha=0.2, color=COLORS['red'])
    ax3.text(0.2, 42, 'Thermal Throttling Zone', fontsize=8, color='red')
    
    # Panel D: Current vs Time (sample 1 hour zoomed)
    ax4 = fig.add_subplot(gs[1, 1])
    
    if 'Mixed Daily Use' in results:
        result = results['Mixed Daily Use']
        t = result['time']
        I = result['current'] * 1000  # mA
        
        # Zoom to interesting region (navigation period)
        mask = (t >= 0.5) & (t <= 1.5)
        t_zoom = t[mask]
        I_zoom = I[mask]
        
        ax4.plot(t_zoom, I_zoom, color=COLORS['blue'], linewidth=1)
        ax4.fill_between(t_zoom, 0, I_zoom, alpha=0.3, color=COLORS['blue'])
        
        ax4.set_xlabel('Time (hours)', fontsize=10)
        ax4.set_ylabel('Current (mA)', fontsize=10)
        ax4.set_title('(d) Current Draw Detail (Navigation Period)', fontsize=11, fontweight='bold')
        
        # Add activity annotations
        ax4.annotate('GPS\nActive', xy=(0.7, max(I_zoom)*0.8), fontsize=8, ha='center')
        ax4.annotate('Music\n+Nav', xy=(1.0, max(I_zoom)*0.6), fontsize=8, ha='center')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def figure_model_validation(save_path: str = None):
    """
    Figure 6: Model Validation and Parameter Sensitivity
    
    Panel A: Peukert effect on capacity
    Panel B: Temperature effect on capacity
    Panel C: OCV-SOC curve
    Panel D: Internal resistance vs SOC
    """
    import sys
    sys.path.insert(0, '/workspace/battery_model/src')
    from battery_core import BatteryCore, BatteryParameters
    
    battery = BatteryCore()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Panel A: Peukert effect
    ax1 = axes[0, 0]
    
    currents = np.linspace(0.1, 3.0, 50)
    capacities = [battery.effective_capacity(I, 298.15) for I in currents]
    
    ax1.plot(currents, capacities, color=COLORS['blue'], linewidth=2)
    ax1.fill_between(currents, 0, capacities, alpha=0.2, color=COLORS['blue'])
    
    ax1.set_xlabel('Discharge Current (A)', fontsize=10)
    ax1.set_ylabel('Effective Capacity (Ah)', fontsize=10)
    ax1.set_title('(a) Peukert Effect on Capacity', fontsize=11, fontweight='bold')
    
    # Add C-rate annotations
    C_rate_points = [(0.9, '0.2C'), (2.25, '0.5C'), (4.5, '1C')]
    for I, label in C_rate_points:
        if I <= 3.0:
            C = battery.effective_capacity(I, 298.15)
            ax1.scatter([I], [C], color=COLORS['red'], s=50, zorder=5)
            ax1.annotate(label, (I, C), textcoords="offset points", 
                        xytext=(10, 5), fontsize=8)
    
    # Panel B: Temperature effect
    ax2 = axes[0, 1]
    
    temps_C = np.linspace(-20, 50, 50)
    temps_K = temps_C + 273.15
    capacities_T = [battery.effective_capacity(1.0, T) for T in temps_K]
    
    ax2.plot(temps_C, capacities_T, color=COLORS['orange'], linewidth=2)
    ax2.fill_between(temps_C, 0, capacities_T, alpha=0.2, color=COLORS['orange'])
    
    # Mark operating regions
    ax2.axvspan(-20, 0, alpha=0.1, color=COLORS['blue'], label='Cold')
    ax2.axvspan(0, 35, alpha=0.1, color=COLORS['green'], label='Optimal')
    ax2.axvspan(35, 50, alpha=0.1, color=COLORS['red'], label='Hot')
    
    ax2.set_xlabel('Temperature (°C)', fontsize=10)
    ax2.set_ylabel('Effective Capacity (Ah)', fontsize=10)
    ax2.set_title('(b) Temperature Effect on Capacity', fontsize=11, fontweight='bold')
    ax2.legend(loc='lower right', fontsize=8)
    
    # Panel C: OCV-SOC curve
    ax3 = axes[1, 0]
    
    soc = np.linspace(0.05, 0.95, 100)
    ocv = [battery.open_circuit_voltage(s) for s in soc]
    
    ax3.plot(soc * 100, ocv, color=COLORS['green'], linewidth=2)
    
    # Add key voltage points
    key_points = [(100, 4.2, 'Full'), (50, 3.7, 'Half'), (20, 3.4, 'Low'), (5, 3.0, 'Empty')]
    for soc_p, v_approx, label in key_points:
        s = soc_p / 100
        v = battery.open_circuit_voltage(s)
        ax3.scatter([soc_p], [v], color=COLORS['red'], s=50, zorder=5)
        ax3.annotate(f'{label}\n({v:.2f}V)', (soc_p, v), 
                    textcoords="offset points", xytext=(10, 10), fontsize=8)
    
    ax3.set_xlabel('State of Charge (%)', fontsize=10)
    ax3.set_ylabel('Open Circuit Voltage (V)', fontsize=10)
    ax3.set_title('(c) OCV-SOC Characteristic Curve', fontsize=11, fontweight='bold')
    ax3.set_xlim([0, 105])
    ax3.set_ylim([2.8, 4.4])
    
    # Panel D: Internal resistance
    ax4 = axes[1, 1]
    
    R_25 = [battery.internal_resistance(s, 298.15) * 1000 for s in soc]  # mOhm
    R_0 = [battery.internal_resistance(s, 273.15) * 1000 for s in soc]   # 0°C
    R_40 = [battery.internal_resistance(s, 313.15) * 1000 for s in soc]  # 40°C
    
    ax4.plot(soc * 100, R_25, color=COLORS['green'], linewidth=2, label='25°C')
    ax4.plot(soc * 100, R_0, color=COLORS['blue'], linewidth=2, label='0°C')
    ax4.plot(soc * 100, R_40, color=COLORS['orange'], linewidth=2, label='40°C')
    
    ax4.set_xlabel('State of Charge (%)', fontsize=10)
    ax4.set_ylabel('Internal Resistance (mΩ)', fontsize=10)
    ax4.set_title('(d) Internal Resistance vs SOC and Temperature', fontsize=11, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=8)
    ax4.set_xlim([0, 105])
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def generate_all_figures(output_dir: str = '/workspace/battery_model/visualizations'):
    """
    Generate all publication-quality figures.
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    figures = {}
    
    print("Generating Figure 1: 5G Power Analysis...")
    figures['5g'] = figure_5g_power_analysis(f'{output_dir}/fig1_5g_power.png')
    
    print("Generating Figure 2: GNSS State Dynamics...")
    figures['gnss'] = figure_gnss_state_dynamics(f'{output_dir}/fig2_gnss_dynamics.png')
    
    print("Generating Figure 3: Background Stochastic...")
    figures['background'] = figure_background_stochastic(f'{output_dir}/fig3_background_stochastic.png')
    
    print("Generating Figure 4: Bluetooth Analysis...")
    figures['bluetooth'] = figure_bluetooth_analysis(f'{output_dir}/fig4_bluetooth_analysis.png')
    
    print("Generating Figure 6: Model Validation...")
    figures['validation'] = figure_model_validation(f'{output_dir}/fig6_model_validation.png')
    
    print("\nAll figures generated successfully!")
    
    return figures


if __name__ == "__main__":
    generate_all_figures()
