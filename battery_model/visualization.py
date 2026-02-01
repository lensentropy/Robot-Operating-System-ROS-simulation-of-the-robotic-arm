"""
Smartphone Battery Model Visualization
=======================================
Comprehensive visualization for the coupled electro-thermal-aging model.

This module provides:
1. Aging model parameter plots
2. OCV model fitting visualization
3. Temperature correction factor plots
4. Dynamic simulation results
5. Power breakdown analysis
6. Multi-scenario comparison
7. 3D thermal distribution (conceptual FEM visualization)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D

# Set style
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.dpi'] = 150

from battery_core import BatteryAgingModel, TemperatureCorrectionModel, OCVModel
from battery_params import (
    AGING_CAPACITY_PARAMS, AGING_RESISTANCE_PARAMS,
    TEMP_CAPACITY_PARAMS, TEMP_RESISTANCE_PARAMS, OCV_PARAMS
)


def plot_aging_models(save_path=None):
    """
    Plot battery aging models: capacity fade and impedance growth.
    
    Parameters:
        save_path: Path to save figure (optional)
    """
    aging_model = BatteryAgingModel()
    
    # Generate data
    N = np.linspace(1, 500, 500)
    Q_max = np.array([aging_model.capacity_fade(n) for n in N])
    R_total = np.array([aging_model.impedance_growth(n) for n in N])
    
    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    
    # Capacity fade plot
    ax1 = axes[0]
    ax1.plot(N, Q_max, 'b-', linewidth=2, label='Model: Double-exponential')
    ax1.set_xlabel('Cycle Number N')
    ax1.set_ylabel('Maximum Capacity $Q_{max}$ [Ah]')
    ax1.set_title('(a) Capacity Fade Model')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 500])
    ax1.legend(loc='upper right')
    
    # Add equation annotation
    eq_text = (r'$Q_{max}(N) = a_Q e^{-b_Q N} + c_Q e^{-d_Q N}$'
               '\n' + r'$R^2 = 0.9915$')
    ax1.text(0.95, 0.55, eq_text, transform=ax1.transAxes, 
             fontsize=9, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Impedance growth plot
    ax2 = axes[1]
    ax2.plot(N, R_total * 1000, 'r-', linewidth=2, label='Model: Power-law')
    ax2.set_xlabel('Cycle Number N')
    ax2.set_ylabel('Total Resistance $R_{total}$ [mΩ]')
    ax2.set_title('(b) Impedance Growth Model')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 500])
    ax2.legend(loc='lower right')
    
    # Add equation annotation
    eq_text = (r'$R_{total}(N) = a_R N^{b_R} + c_R$'
               '\n' + r'$R^2 = 0.9842$')
    ax2.text(0.95, 0.35, eq_text, transform=ax2.transAxes, 
             fontsize=9, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_ocv_model(save_path=None):
    """
    Plot OCV model based on Nernst equation.
    
    Parameters:
        save_path: Path to save figure (optional)
    """
    ocv_model = OCVModel()
    
    # Generate data
    z = np.linspace(0.01, 0.99, 500)
    V_ocv = np.array([ocv_model.voltage(zi) for zi in z])
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 5))
    
    ax.plot(z * 100, V_ocv, 'b-', linewidth=2.5, label='Nernst-based Combined Model')
    ax.set_xlabel('State of Charge SOC [%]')
    ax.set_ylabel('Open Circuit Voltage $V_{OCV}$ [V]')
    ax.set_title('Open Circuit Voltage Model')
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 100])
    ax.set_ylim([3.0, 4.3])
    ax.legend(loc='lower right')
    
    # Add equation annotation
    eq_text = (r'$V_{OCV}(z) = K_0 + K_1 z + \frac{K_2}{z} + K_3 \ln(z) + K_4 \ln(1-z)$'
               '\n' + r'$R^2 = 0.9927$')
    ax.text(0.5, 0.15, eq_text, transform=ax.transAxes, 
            fontsize=10, verticalalignment='top', horizontalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Highlight nonlinear regions
    ax.axvspan(0, 10, alpha=0.15, color='red', label='High nonlinearity region')
    ax.axvspan(90, 100, alpha=0.15, color='red')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_temperature_corrections(save_path=None):
    """
    Plot temperature correction factors for capacity and resistance.
    
    Parameters:
        save_path: Path to save figure (optional)
    """
    temp_model = TemperatureCorrectionModel()
    
    # Generate data
    T = np.linspace(-20, 60, 200)
    S_Q = np.array([temp_model.capacity_factor(t) for t in T])
    S_R = np.array([temp_model.resistance_factor(t) for t in T])
    
    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    
    # Capacity correction
    ax1 = axes[0]
    ax1.plot(T, S_Q, 'b-', linewidth=2.5, label='Sigmoid Model')
    ax1.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)
    ax1.axvline(x=-15, color='red', linestyle=':', alpha=0.7, label='Inflection point')
    ax1.set_xlabel('Temperature [°C]')
    ax1.set_ylabel('Capacity Factor $S_Q$ [-]')
    ax1.set_title('(a) Capacity Temperature Correction')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([-20, 60])
    ax1.legend(loc='lower right')
    
    # Add equation annotation
    eq_text = r'$S_Q(T) = \frac{S_{Q,max}}{1 + e^{-k_Q(T - T_0)}}$'
    ax1.text(0.5, 0.2, eq_text, transform=ax1.transAxes, 
            fontsize=10, verticalalignment='top', horizontalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Resistance correction
    ax2 = axes[1]
    ax2.plot(T, S_R, 'r-', linewidth=2.5, label='Arrhenius Model')
    ax2.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Temperature [°C]')
    ax2.set_ylabel('Resistance Factor $S_R$ [-]')
    ax2.set_title('(b) Resistance Temperature Correction')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([-20, 60])
    ax2.legend(loc='upper right')
    
    # Add equation annotation
    eq_text = r'$S_R(T) = C_R + A_R \cdot e^{-B_R \cdot T}$'
    ax2.text(0.5, 0.85, eq_text, transform=ax2.transAxes, 
            fontsize=10, verticalalignment='top', horizontalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_simulation_results(results, save_path=None):
    """
    Plot comprehensive simulation results.
    
    Parameters:
        results: SimulationResults object
        save_path: Path to save figure (optional)
    """
    t_hours = results.t / 3600
    
    # Create figure with grid
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    # (a) SOC and Voltage
    ax1 = fig.add_subplot(gs[0, 0])
    ax1_twin = ax1.twinx()
    
    line1, = ax1.plot(t_hours, results.soc * 100, 'b-', linewidth=2, label='SOC')
    line2, = ax1_twin.plot(t_hours, results.V_term, 'r-', linewidth=2, label='Voltage')
    
    ax1.set_xlabel('Time [hours]')
    ax1.set_ylabel('State of Charge [%]', color='blue')
    ax1_twin.set_ylabel('Terminal Voltage [V]', color='red')
    ax1.set_title('(a) Battery State Evolution')
    ax1.grid(True, alpha=0.3)
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1_twin.tick_params(axis='y', labelcolor='red')
    ax1.legend(handles=[line1, line2], loc='upper right')
    
    # (b) Temperature dynamics
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t_hours, results.T_core, 'r-', linewidth=2, label='Core $T_c$')
    ax2.plot(t_hours, results.T_surface, 'b--', linewidth=2, label='Surface $T_s$')
    ax2.plot(t_hours, results.T_soc, 'g:', linewidth=2, label='SoC chip')
    ax2.set_xlabel('Time [hours]')
    ax2.set_ylabel('Temperature [°C]')
    ax2.set_title('(b) Thermal Dynamics')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='lower right')
    
    # (c) Current profile
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t_hours, results.I_load * 1000, 'k-', linewidth=1.5)
    ax3.fill_between(t_hours, 0, results.I_load * 1000, alpha=0.3)
    ax3.set_xlabel('Time [hours]')
    ax3.set_ylabel('Load Current [mA]')
    ax3.set_title('(c) Load Current Profile')
    ax3.grid(True, alpha=0.3)
    
    # (d) Power consumption
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t_hours, results.P_total * 1000, 'g-', linewidth=2)
    ax4.fill_between(t_hours, 0, results.P_total * 1000, alpha=0.3, color='green')
    ax4.axhline(y=results.average_power() * 1000, color='red', linestyle='--', 
                label=f'Average: {results.average_power()*1000:.1f} mW')
    ax4.set_xlabel('Time [hours]')
    ax4.set_ylabel('Power [mW]')
    ax4.set_title('(d) Power Consumption')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='upper right')
    
    # (e) Polarization voltages
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(t_hours, results.V_1 * 1000, 'b-', linewidth=2, label='$V_1$ (fast)')
    ax5.plot(t_hours, results.V_2 * 1000, 'r-', linewidth=2, label='$V_2$ (slow)')
    ax5.set_xlabel('Time [hours]')
    ax5.set_ylabel('Polarization Voltage [mV]')
    ax5.set_title('(e) RC Circuit Polarization')
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='upper right')
    
    # (f) Power breakdown (pie chart for average)
    ax6 = fig.add_subplot(gs[2, 1])
    
    # Calculate average breakdown
    avg_breakdown = {}
    for key in results.power_breakdown[0].keys():
        if key != 'Total':
            avg_breakdown[key] = np.mean([b[key] for b in results.power_breakdown])
    
    # Sort and prepare for pie chart
    sorted_items = sorted(avg_breakdown.items(), key=lambda x: -x[1])
    labels = [item[0] for item in sorted_items]
    sizes = [item[1] * 1000 for item in sorted_items]  # mW
    colors = plt.cm.Set3(np.linspace(0, 1, len(labels)))
    
    wedges, texts, autotexts = ax6.pie(sizes, labels=labels, autopct='%1.1f%%',
                                        colors=colors, startangle=90)
    ax6.set_title('(f) Average Power Breakdown')
    
    plt.suptitle('Coupled Electro-Thermal-Aging Model Simulation Results', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_extreme_condition_validation(results, save_path=None):
    """
    Plot validation results for extreme conditions (low temp, aged battery, high load).
    
    Parameters:
        results: SimulationResults object from extreme condition test
        save_path: Path to save figure (optional)
    """
    t_sec = results.t
    t_min = t_sec / 60
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    
    # (a) Temperature and Resistance
    ax1 = axes[0, 0]
    ax1_twin = ax1.twinx()
    
    # Calculate effective resistance at each time point
    from battery_core import BatteryCoreModel
    model = results.model.battery
    R_eff = np.array([model.get_effective_resistance(T) * 1000 
                      for T in results.T_core])
    
    line1, = ax1.plot(t_min, results.T_core, 'r-', linewidth=2, label='$T_c$ (Core)')
    line2, = ax1_twin.plot(t_min, R_eff, 'b-', linewidth=2, label='$R_{total}$')
    
    ax1.set_xlabel('Time [min]')
    ax1.set_ylabel('Core Temperature [°C]', color='red')
    ax1_twin.set_ylabel('Effective Resistance [mΩ]', color='blue')
    ax1.set_title('(a) Self-Heating Recovery Effect')
    ax1.tick_params(axis='y', labelcolor='red')
    ax1_twin.tick_params(axis='y', labelcolor='blue')
    ax1.grid(True, alpha=0.3)
    ax1.legend(handles=[line1, line2], loc='center right')
    
    # (b) Voltage behavior
    ax2 = axes[0, 1]
    ax2.plot(t_min, results.V_term, 'b-', linewidth=2)
    ax2.set_xlabel('Time [min]')
    ax2.set_ylabel('Terminal Voltage [V]')
    ax2.set_title('(b) Non-monotonic Voltage Behavior')
    ax2.grid(True, alpha=0.3)
    
    # Annotate plateau region
    if len(t_min) > 10:
        ax2.annotate('Voltage plateau\n(self-heating effect)', 
                    xy=(t_min[len(t_min)//4], results.V_term[len(t_min)//4]),
                    xytext=(t_min[len(t_min)//2], results.V_term[0] - 0.1),
                    arrowprops=dict(arrowstyle='->', color='red'),
                    fontsize=9, color='red')
    
    # (c) SOC depletion
    ax3 = axes[1, 0]
    ax3.plot(t_min, results.soc * 100, 'g-', linewidth=2)
    ax3.set_xlabel('Time [min]')
    ax3.set_ylabel('State of Charge [%]')
    ax3.set_title('(c) SOC Depletion Under Extreme Load')
    ax3.grid(True, alpha=0.3)
    
    # (d) Power and current
    ax4 = axes[1, 1]
    ax4_twin = ax4.twinx()
    
    line3, = ax4.plot(t_min, results.P_total * 1000, 'k-', linewidth=2, label='Power')
    line4, = ax4_twin.plot(t_min, results.I_load * 1000, 'orange', linewidth=2, label='Current')
    
    ax4.set_xlabel('Time [min]')
    ax4.set_ylabel('Power [mW]', color='black')
    ax4_twin.set_ylabel('Current [mA]', color='orange')
    ax4.set_title('(d) Power and Current Profile')
    ax4.tick_params(axis='y', labelcolor='black')
    ax4_twin.tick_params(axis='y', labelcolor='orange')
    ax4.grid(True, alpha=0.3)
    ax4.legend(handles=[line3, line4], loc='upper right')
    
    plt.suptitle('Extreme Condition Validation: Low Temp (-10°C), N=300, High Load', 
                 fontsize=13, fontweight='bold', y=0.98)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_thermal_gradient_fem(save_path=None):
    """
    Create conceptual FEM-style visualization of battery internal temperature distribution.
    
    Parameters:
        save_path: Path to save figure (optional)
    """
    # Create cylindrical battery cross-section
    n_r = 50
    n_theta = 100
    
    r = np.linspace(0, 1, n_r)
    theta = np.linspace(0, 2 * np.pi, n_theta)
    R, Theta = np.meshgrid(r, theta)
    
    # Convert to Cartesian
    X = R * np.cos(Theta)
    Y = R * np.sin(Theta)
    
    # Temperature distribution (parabolic profile - hotter at center)
    T_center = 45  # Core temperature
    T_surface = 42  # Surface temperature
    T = T_surface + (T_center - T_surface) * (1 - R**2)
    
    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # (a) 2D cross-section
    ax1 = axes[0]
    
    # Custom colormap
    cmap = plt.cm.hot_r
    
    c = ax1.pcolormesh(X, Y, T, cmap=cmap, shading='auto', vmin=40, vmax=46)
    ax1.set_aspect('equal')
    ax1.set_xlabel('x [normalized]')
    ax1.set_ylabel('y [normalized]')
    ax1.set_title('(a) Battery Cross-Section Temperature Distribution')
    
    # Add colorbar
    cbar = plt.colorbar(c, ax=ax1, label='Temperature [°C]')
    
    # Add labels
    ax1.annotate('Core\n(High T)', xy=(0, 0), fontsize=10, ha='center', va='center',
                 color='white', fontweight='bold')
    ax1.annotate('Surface\n(Low T)', xy=(0.7, 0.7), fontsize=9, ha='center', va='center',
                 color='black')
    
    # (b) Radial profile
    ax2 = axes[1]
    r_profile = np.linspace(0, 1, 100)
    T_profile = T_surface + (T_center - T_surface) * (1 - r_profile**2)
    
    ax2.plot(r_profile * 9, T_profile, 'r-', linewidth=2.5)  # 9mm radius typical
    ax2.fill_between(r_profile * 9, T_surface - 1, T_profile, alpha=0.3, color='red')
    ax2.axhline(y=T_center, color='red', linestyle='--', alpha=0.5, label=f'$T_c$ = {T_center}°C')
    ax2.axhline(y=T_surface, color='blue', linestyle='--', alpha=0.5, label=f'$T_s$ = {T_surface}°C')
    
    ax2.set_xlabel('Radial Position [mm]')
    ax2.set_ylabel('Temperature [°C]')
    ax2.set_title('(b) Radial Temperature Profile')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='lower left')
    ax2.set_xlim([0, 9])
    ax2.set_ylim([40, 46])
    
    # Add gradient annotation
    ax2.annotate(f'ΔT = {T_center - T_surface:.1f}°C', 
                xy=(4.5, (T_center + T_surface)/2),
                fontsize=11, ha='center', va='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    plt.suptitle('Battery Internal Temperature Gradient (FEM Conceptual Validation)', 
                 fontsize=13, fontweight='bold', y=0.98)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_scenario_comparison(scenario_results, save_path=None):
    """
    Compare discharge curves across different usage scenarios.
    
    Parameters:
        scenario_results: Dict of {scenario_name: SimulationResults}
        save_path: Path to save figure (optional)
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(scenario_results)))
    
    # (a) SOC comparison
    ax1 = axes[0, 0]
    for i, (name, results) in enumerate(scenario_results.items()):
        t_hours = results.t / 3600
        ax1.plot(t_hours, results.soc * 100, color=colors[i], linewidth=2, label=name)
    ax1.set_xlabel('Time [hours]')
    ax1.set_ylabel('State of Charge [%]')
    ax1.set_title('(a) SOC Discharge Curves')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_xlim([0, None])
    ax1.set_ylim([0, 100])
    
    # (b) Voltage comparison
    ax2 = axes[0, 1]
    for i, (name, results) in enumerate(scenario_results.items()):
        t_hours = results.t / 3600
        ax2.plot(t_hours, results.V_term, color=colors[i], linewidth=2, label=name)
    ax2.set_xlabel('Time [hours]')
    ax2.set_ylabel('Terminal Voltage [V]')
    ax2.set_title('(b) Voltage Profiles')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.set_xlim([0, None])
    
    # (c) Power comparison
    ax3 = axes[1, 0]
    for i, (name, results) in enumerate(scenario_results.items()):
        t_hours = results.t / 3600
        ax3.plot(t_hours, results.P_total * 1000, color=colors[i], linewidth=2, label=name)
    ax3.set_xlabel('Time [hours]')
    ax3.set_ylabel('Power [mW]')
    ax3.set_title('(c) Power Consumption')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=8)
    ax3.set_xlim([0, None])
    
    # (d) Battery life bar chart
    ax4 = axes[1, 1]
    names = list(scenario_results.keys())
    lifetimes = [r.discharge_time_hours for r in scenario_results.values()]
    avg_powers = [r.average_power() * 1000 for r in scenario_results.values()]
    
    x = np.arange(len(names))
    width = 0.35
    
    bars1 = ax4.bar(x - width/2, lifetimes, width, label='Battery Life [hours]', color='steelblue')
    ax4_twin = ax4.twinx()
    bars2 = ax4_twin.bar(x + width/2, avg_powers, width, label='Avg Power [mW]', color='coral')
    
    ax4.set_ylabel('Battery Life [hours]', color='steelblue')
    ax4_twin.set_ylabel('Average Power [mW]', color='coral')
    ax4.set_title('(d) Battery Life & Power Summary')
    ax4.set_xticks(x)
    ax4.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    ax4.tick_params(axis='y', labelcolor='steelblue')
    ax4_twin.tick_params(axis='y', labelcolor='coral')
    
    # Combine legends
    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_twin.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    
    plt.suptitle('Multi-Scenario Battery Discharge Comparison', 
                 fontsize=13, fontweight='bold', y=0.98)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_rc_circuit_diagram(save_path=None):
    """
    Draw 2nd-order Thevenin equivalent circuit diagram.
    
    Parameters:
        save_path: Path to save figure (optional)
    """
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 4)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Draw components
    # OCV source
    circle = plt.Circle((1, 2), 0.4, fill=False, linewidth=2)
    ax.add_patch(circle)
    ax.text(1, 2, '+\n-', ha='center', va='center', fontsize=10)
    ax.text(1, 1.2, '$V_{OCV}$', ha='center', va='top', fontsize=11)
    
    # Lines from OCV
    ax.plot([1.4, 2], [2, 2], 'k-', linewidth=2)
    
    # R0 (Ohmic resistance)
    rect = mpatches.Rectangle((2, 1.8), 1, 0.4, fill=False, linewidth=2)
    ax.add_patch(rect)
    ax.text(2.5, 1.5, '$R_0$', ha='center', va='top', fontsize=11)
    
    # Lines
    ax.plot([3, 3.5], [2, 2], 'k-', linewidth=2)
    
    # R1-C1 parallel
    ax.plot([3.5, 3.5], [1.5, 2.5], 'k-', linewidth=2)  # Vertical line
    rect1 = mpatches.Rectangle((3.7, 2.2), 0.8, 0.3, fill=False, linewidth=2)
    ax.add_patch(rect1)
    ax.text(4.1, 2.8, '$R_1$', ha='center', va='bottom', fontsize=10)
    
    # C1 (capacitor symbol)
    ax.plot([3.85, 3.85], [1.5, 1.75], 'k-', linewidth=2)
    ax.plot([3.65, 4.05], [1.75, 1.75], 'k-', linewidth=3)
    ax.plot([3.65, 4.05], [1.85, 1.85], 'k-', linewidth=3)
    ax.plot([3.85, 3.85], [1.85, 2.1], 'k-', linewidth=2)
    ax.text(4.3, 1.8, '$C_1$', ha='left', va='center', fontsize=10)
    
    # Vertical lines for parallel
    ax.plot([4.5, 4.5], [1.5, 2.5], 'k-', linewidth=2)
    ax.plot([3.5, 3.7], [2.35, 2.35], 'k-', linewidth=2)
    ax.plot([4.5, 4.5], [2.35, 2.35], 'k-', linewidth=2)
    ax.plot([3.5, 3.5, 3.85], [1.5, 1.5, 1.5], 'k-', linewidth=2)
    ax.plot([4.5, 4.5, 3.85], [1.5, 1.5, 1.5], 'k-', linewidth=2)
    
    # Connect to second RC
    ax.plot([4.5, 5.5], [2, 2], 'k-', linewidth=2)
    
    # R2-C2 parallel
    ax.plot([5.5, 5.5], [1.5, 2.5], 'k-', linewidth=2)
    rect2 = mpatches.Rectangle((5.7, 2.2), 0.8, 0.3, fill=False, linewidth=2)
    ax.add_patch(rect2)
    ax.text(6.1, 2.8, '$R_2$', ha='center', va='bottom', fontsize=10)
    
    # C2 (capacitor symbol)
    ax.plot([5.85, 5.85], [1.5, 1.75], 'k-', linewidth=2)
    ax.plot([5.65, 6.05], [1.75, 1.75], 'k-', linewidth=3)
    ax.plot([5.65, 6.05], [1.85, 1.85], 'k-', linewidth=3)
    ax.plot([5.85, 5.85], [1.85, 2.1], 'k-', linewidth=2)
    ax.text(6.3, 1.8, '$C_2$', ha='left', va='center', fontsize=10)
    
    # Vertical lines for parallel
    ax.plot([6.5, 6.5], [1.5, 2.5], 'k-', linewidth=2)
    ax.plot([5.5, 5.7], [2.35, 2.35], 'k-', linewidth=2)
    ax.plot([6.5, 6.5], [2.35, 2.35], 'k-', linewidth=2)
    ax.plot([5.5, 5.5, 5.85], [1.5, 1.5, 1.5], 'k-', linewidth=2)
    ax.plot([6.5, 6.5, 5.85], [1.5, 1.5, 1.5], 'k-', linewidth=2)
    
    # Output terminal
    ax.plot([6.5, 8], [2, 2], 'k-', linewidth=2)
    ax.plot([8, 8], [1.8, 2.2], 'k-', linewidth=3)
    ax.text(8.3, 2, '$V_{term}$', ha='left', va='center', fontsize=11)
    
    # Ground
    ax.plot([1, 8], [0.5, 0.5], 'k-', linewidth=2)
    ax.plot([8, 8], [0.5, 0.7], 'k-', linewidth=3)
    for i, w in enumerate([0.4, 0.3, 0.2]):
        ax.plot([4.5 - w, 4.5 + w], [0.3 - i*0.1, 0.3 - i*0.1], 'k-', linewidth=2)
    
    # Connect ground to circuit
    ax.plot([1, 1], [0.5, 1.6], 'k-', linewidth=2)
    
    # Current arrow
    ax.annotate('', xy=(7.5, 2.5), xytext=(6.8, 2.5),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))
    ax.text(7.15, 2.8, '$I(t)$', ha='center', va='bottom', fontsize=11, color='red')
    
    # Title
    ax.text(4.5, 3.8, '2nd-Order Thevenin Equivalent Circuit Model', 
            ha='center', va='top', fontsize=13, fontweight='bold')
    
    # Annotations
    ax.text(2.5, 0.8, 'Ohmic\nResistance', ha='center', va='top', fontsize=8, style='italic')
    ax.text(4, 0.8, 'Electrochemical\nPolarization', ha='center', va='top', fontsize=8, style='italic')
    ax.text(6, 0.8, 'Concentration\nPolarization', ha='center', va='top', fontsize=8, style='italic')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def generate_all_figures(output_dir='figures'):
    """
    Generate all figures for the paper/report.
    
    Parameters:
        output_dir: Directory to save figures
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating all figures...")
    
    # 1. Aging models
    print("  [1/6] Aging models...")
    plot_aging_models(f'{output_dir}/fig_aging_models.png')
    
    # 2. OCV model
    print("  [2/6] OCV model...")
    plot_ocv_model(f'{output_dir}/fig_ocv_model.png')
    
    # 3. Temperature corrections
    print("  [3/6] Temperature corrections...")
    plot_temperature_corrections(f'{output_dir}/fig_temp_corrections.png')
    
    # 4. RC circuit diagram
    print("  [4/6] RC circuit diagram...")
    plot_rc_circuit_diagram(f'{output_dir}/fig_rc_circuit.png')
    
    # 5. Thermal gradient
    print("  [5/6] Thermal gradient FEM...")
    plot_thermal_gradient_fem(f'{output_dir}/fig_thermal_fem.png')
    
    print("Static figures generated successfully!")
    print(f"\nTo generate simulation figures, run the main simulation script.")


if __name__ == "__main__":
    # Generate static figures
    generate_all_figures('figures')
    
    plt.show()
