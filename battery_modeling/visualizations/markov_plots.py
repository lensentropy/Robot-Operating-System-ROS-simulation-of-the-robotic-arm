"""
Visualization module for Markov Chain User Behavior Model

Generates:
- Time-partitioned state evolution plots
- Probability distribution dynamics
- ODE solution comparisons
- Power consumption analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Patch, Rectangle
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.markov_user_behavior import (
    MarkovChainSimulator, MarkovDifferentialEquations,
    TransitionMatrices, TimePartition, MarkovStates,
    analyze_stationary_distributions, analyze_mode_transitions
)

# Set style
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 150


def plot_time_partitions(save_path=None):
    """
    Figure: 24-hour time partition visualization
    
    展示一天中的时间分区:
    - 睡眠模式 (Sleep): 23:00-07:00
    - 工作模式 (Work): 09:00-12:00, 14:00-18:00
    - 休闲模式 (Leisure): 07:00-09:00, 12:00-14:00, 18:00-23:00
    """
    fig, ax = plt.subplots(figsize=(14, 3))
    
    partition = TimePartition()
    
    # Define colors for each mode
    colors = {
        'sleep': '#4A5568',    # Dark gray
        'work': '#E53E3E',     # Red
        'leisure': '#48BB78'   # Green
    }
    
    # Plot time blocks
    # Sleep: 23:00-24:00 and 00:00-07:00
    ax.axvspan(23, 24, alpha=0.5, color=colors['sleep'], label='Sleep Mode')
    ax.axvspan(0, 7, alpha=0.5, color=colors['sleep'])
    
    # Work periods
    ax.axvspan(9, 12, alpha=0.5, color=colors['work'], label='Work Mode')
    ax.axvspan(14, 18, alpha=0.5, color=colors['work'])
    
    # Leisure periods
    ax.axvspan(7, 9, alpha=0.5, color=colors['leisure'], label='Leisure Mode')
    ax.axvspan(12, 14, alpha=0.5, color=colors['leisure'])
    ax.axvspan(18, 23, alpha=0.5, color=colors['leisure'])
    
    # Add transition matrix labels
    transitions = [
        (3.5, 'P_sleep'),
        (8, 'P_leisure'),
        (10.5, 'P_work'),
        (13, 'P_leisure'),
        (16, 'P_work'),
        (20.5, 'P_leisure'),
        (23.5, 'P_sleep')
    ]
    
    for x, label in transitions:
        ax.annotate(label, xy=(x, 0.5), ha='center', va='center',
                   fontsize=11, fontweight='bold', color='white',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.6))
    
    # Formatting
    ax.set_xlim(0, 24)
    ax.set_ylim(0, 1)
    ax.set_xlabel('Time of Day (Hour)', fontsize=12)
    ax.set_xticks(range(0, 25, 2))
    ax.set_yticks([])
    ax.set_title('Time-Inhomogeneous Markov Chain: Mode Schedule', fontsize=14, fontweight='bold')
    
    # Legend
    handles = [Patch(facecolor=c, alpha=0.5, label=l) 
               for l, c in [('Sleep (23:00-07:00)', colors['sleep']),
                           ('Work (09:00-12:00, 14:00-18:00)', colors['work']),
                           ('Leisure (07:00-09:00, 12:00-14:00, 18:00-23:00)', colors['leisure'])]]
    ax.legend(handles=handles, loc='upper right', fontsize=10)
    
    # Add hour markers
    for h in range(0, 25, 6):
        ax.axvline(x=h, color='gray', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_transition_matrices(save_path=None):
    """
    Figure: Transition probability matrices for each mode
    
    可视化各模式下的状态转移概率矩阵
    """
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    tm = TransitionMatrices()
    states = MarkovStates.names
    
    matrices = {
        'Sleep Mode': tm.P_sleep,
        'Work Mode': tm.P_work,
        'Leisure Mode': tm.P_leisure
    }
    
    for ax, (title, P) in zip(axes, matrices.items()):
        im = ax.imshow(P, cmap='YlOrRd', vmin=0, vmax=1)
        
        # Add text annotations
        for i in range(4):
            for j in range(4):
                text = ax.text(j, i, f'{P[i,j]:.3f}',
                              ha='center', va='center', fontsize=9,
                              color='white' if P[i,j] > 0.5 else 'black')
        
        ax.set_xticks(range(4))
        ax.set_yticks(range(4))
        ax.set_xticklabels(['S1', 'S2', 'S3', 'S4'], fontsize=9)
        ax.set_yticklabels(['S1', 'S2', 'S3', 'S4'], fontsize=9)
        ax.set_xlabel('To State', fontsize=10)
        ax.set_ylabel('From State', fontsize=10)
        ax.set_title(title, fontsize=12, fontweight='bold')
    
    # Add colorbar
    cbar = fig.colorbar(im, ax=axes, orientation='vertical', shrink=0.8, pad=0.02)
    cbar.set_label('Transition Probability', fontsize=10)
    
    # Add state legend
    state_labels = '\n'.join([f'S{i+1}: {name}' for i, name in enumerate(states)])
    fig.text(0.02, 0.5, state_labels, fontsize=9, va='center',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_stationary_distributions(save_path=None):
    """
    Figure: Stationary distributions for each mode
    
    各模式下的稳态概率分布
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    stat_dist = analyze_stationary_distributions()
    states = MarkovStates.names
    
    x = np.arange(4)
    width = 0.25
    
    colors = ['#4A5568', '#E53E3E', '#48BB78']
    labels = ['Sleep Mode', 'Work Mode', 'Leisure Mode']
    
    for i, (mode, color, label) in enumerate(zip(['sleep', 'work', 'leisure'], colors, labels)):
        dist = stat_dist[mode]['distribution']
        bars = ax.bar(x + i*width, dist, width, label=label, color=color, alpha=0.8)
        
        # Add value labels
        for bar, val in zip(bars, dist):
            if val > 0.05:
                ax.text(bar.get_x() + bar.get_width()/2, val + 0.02,
                       f'{val:.1%}', ha='center', va='bottom', fontsize=8)
    
    ax.set_xticks(x + width)
    ax.set_xticklabels(states, fontsize=10)
    ax.set_ylabel('Stationary Probability π*', fontsize=11)
    ax.set_ylim(0, 1.1)
    ax.set_title('Stationary Distributions by Mode (Long-term Behavior)', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add equation annotation
    eq_text = r'$\pi^* P = \pi^* \Rightarrow \pi^*$ is left eigenvector of $P$ with eigenvalue 1'
    ax.text(0.5, -0.12, eq_text, transform=ax.transAxes, fontsize=10,
            ha='center', style='italic')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_probability_evolution(save_path=None):
    """
    Figure: Probability distribution evolution over 24 hours
    
    概率分布随时间的演化（求解Kolmogorov前向方程）
    """
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    ode_system = MarkovDifferentialEquations()
    partition = TimePartition()
    states = MarkovStates.names
    
    # Initial distribution: start in Deep Sleep at midnight
    pi0 = np.array([1.0, 0.0, 0.0, 0.0])
    
    result = ode_system.solve_probability_evolution(
        pi0=pi0,
        t_span=(0, 24),
        start_hour=0,
        num_points=1440
    )
    
    time = result['time']
    probs = result['probabilities']
    
    # Top plot: Probability evolution
    ax1 = axes[0]
    
    colors = ['#4A5568', '#3182CE', '#DD6B20', '#E53E3E']
    
    # Add mode background
    mode_colors = {'sleep': '#E2E8F0', 'work': '#FED7D7', 'leisure': '#C6F6D5'}
    for i, mode in enumerate(result['modes']):
        if i == 0 or result['modes'][i-1] != mode:
            start_idx = i
        if i == len(result['modes'])-1 or result['modes'][i+1] != mode:
            ax1.axvspan(time[start_idx], time[i], alpha=0.3, 
                       color=mode_colors[mode], zorder=0)
    
    # Plot probability trajectories
    for i, (name, color) in enumerate(zip(states, colors)):
        ax1.plot(time, probs[:, i], '-', lw=2, color=color, label=name)
        ax1.fill_between(time, 0, probs[:, i], alpha=0.1, color=color)
    
    ax1.set_ylabel('State Probability π(t)', fontsize=11)
    ax1.set_ylim(0, 1.05)
    ax1.legend(loc='upper right', ncol=4)
    ax1.set_title('Kolmogorov Forward Equation: dπ/dt = π·Q(t)', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Bottom plot: Expected power
    ax2 = axes[1]
    
    ax2.fill_between(time, 0, result['expected_power'], alpha=0.4, color='red')
    ax2.plot(time, result['expected_power'], 'r-', lw=2)
    
    # Add mode labels
    for i, mode in enumerate(result['modes']):
        if i == 0 or result['modes'][i-1] != mode:
            start_idx = i
        if i == len(result['modes'])-1 or result['modes'][i+1] != mode:
            mid_time = (time[start_idx] + time[i]) / 2
            ax2.axvspan(time[start_idx], time[i], alpha=0.2,
                       color=mode_colors[mode], zorder=0)
    
    ax2.set_xlabel('Time of Day (Hour)', fontsize=11)
    ax2.set_ylabel('Expected Power E[P(t)] (mW)', fontsize=11)
    ax2.set_title('Expected Power Consumption: E[P(t)] = Σᵢ πᵢ(t)·Pᵢ', fontsize=12, fontweight='bold')
    ax2.set_xlim(0, 24)
    ax2.set_xticks(range(0, 25, 2))
    ax2.grid(True, alpha=0.3)
    
    # Add average power annotation
    avg_power = np.mean(result['expected_power'])
    ax2.axhline(y=avg_power, color='blue', linestyle='--', alpha=0.7)
    ax2.text(23.5, avg_power + 50, f'Avg: {avg_power:.0f} mW', 
             ha='right', fontsize=10, color='blue')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_coupled_system_dynamics(save_path=None):
    """
    Figure: Coupled ODE system solution (Markov + Battery + Thermal)
    
    耦合微分方程组的求解结果:
    - 概率分布演化
    - SOC衰减
    - 温度变化
    """
    fig = plt.figure(figsize=(14, 12))
    gs = gridspec.GridSpec(3, 2, height_ratios=[1, 1, 1], hspace=0.3, wspace=0.25)
    
    ode_system = MarkovDifferentialEquations()
    
    # Initial conditions
    pi0 = np.array([1.0, 0.0, 0.0, 0.0])
    
    result = ode_system.solve_coupled_system(
        pi0=pi0,
        SOC0=1.0,
        T0=25.0,
        t_span=(0, 24),
        start_hour=0
    )
    
    time = result['time']
    probs = result['probabilities']
    states = MarkovStates.names
    colors = ['#4A5568', '#3182CE', '#DD6B20', '#E53E3E']
    
    # Plot 1: State probabilities (stacked area)
    ax1 = fig.add_subplot(gs[0, :])
    ax1.stackplot(time, probs.T, labels=states, colors=colors, alpha=0.7)
    ax1.set_ylabel('Probability', fontsize=11)
    ax1.set_ylim(0, 1)
    ax1.legend(loc='upper right', ncol=4)
    ax1.set_title('(a) State Probability Distribution Evolution', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, 24)
    
    # Plot 2: SOC evolution
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.fill_between(time, 0, result['SOC'] * 100, alpha=0.3, color='blue')
    ax2.plot(time, result['SOC'] * 100, 'b-', lw=2)
    ax2.set_xlabel('Time (Hours)', fontsize=11)
    ax2.set_ylabel('State of Charge (%)', fontsize=11)
    ax2.set_title('(b) Battery SOC: dSOC/dt = -E[P]/(V·Q)', fontsize=12, fontweight='bold')
    ax2.set_ylim(0, 105)
    ax2.set_xlim(0, 24)
    ax2.grid(True, alpha=0.3)
    
    # Add discharge rate annotation
    discharge_rate = (1 - result['SOC'][-1]) / 24 * 100
    ax2.text(0.95, 0.95, f'Discharge Rate: {discharge_rate:.1f}%/hr',
             transform=ax2.transAxes, ha='right', va='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Plot 3: Temperature evolution
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.fill_between(time, 25, result['temperature'], alpha=0.3, color='red')
    ax3.plot(time, result['temperature'], 'r-', lw=2)
    ax3.axhline(y=25, color='green', linestyle='--', alpha=0.7, label='Ambient')
    ax3.set_xlabel('Time (Hours)', fontsize=11)
    ax3.set_ylabel('Temperature (°C)', fontsize=11)
    ax3.set_title('(c) Thermal: dT/dt = (Q_gen - Q_diss)/C', fontsize=12, fontweight='bold')
    ax3.set_xlim(0, 24)
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Expected power by state contribution
    ax4 = fig.add_subplot(gs[2, 0])
    state_power = np.array([50, 800, 1500, 4000])
    power_contributions = probs * state_power
    
    ax4.stackplot(time, power_contributions.T, labels=states, colors=colors, alpha=0.7)
    ax4.set_xlabel('Time (Hours)', fontsize=11)
    ax4.set_ylabel('Power (mW)', fontsize=11)
    ax4.set_title('(d) Power Decomposition by State', fontsize=12, fontweight='bold')
    ax4.set_xlim(0, 24)
    ax4.legend(loc='upper right', ncol=2)
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Phase diagram (SOC vs Temperature)
    ax5 = fig.add_subplot(gs[2, 1])
    scatter = ax5.scatter(result['SOC'] * 100, result['temperature'], 
                         c=time, cmap='viridis', s=2, alpha=0.7)
    ax5.plot(result['SOC'][0] * 100, result['temperature'][0], 'go', 
             markersize=10, label='Start')
    ax5.plot(result['SOC'][-1] * 100, result['temperature'][-1], 'rs',
             markersize=10, label='End')
    ax5.set_xlabel('SOC (%)', fontsize=11)
    ax5.set_ylabel('Temperature (°C)', fontsize=11)
    ax5.set_title('(e) Phase Diagram: SOC vs Temperature', fontsize=12, fontweight='bold')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)
    
    cbar = fig.colorbar(scatter, ax=ax5)
    cbar.set_label('Time (Hours)')
    
    plt.suptitle('Coupled Electro-Thermal-Markov System Dynamics', 
                fontsize=14, fontweight='bold', y=1.01)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_monte_carlo_simulation(save_path=None):
    """
    Figure: Monte Carlo simulation results (single trajectory)
    
    蒙特卡洛仿真结果展示
    """
    fig = plt.figure(figsize=(14, 12))
    gs = gridspec.GridSpec(4, 1, height_ratios=[1, 1, 1, 1], hspace=0.25)
    
    simulator = MarkovChainSimulator(seed=42)
    result = simulator.simulate(duration_hours=24, start_hour=0, initial_state=0)
    
    time = result['time']
    states_names = MarkovStates.names
    
    # Define mode colors for background
    mode_colors = {'sleep': '#E2E8F0', 'work': '#FED7D7', 'leisure': '#C6F6D5'}
    
    # Plot 1: State trajectory
    ax1 = fig.add_subplot(gs[0])
    
    # Add mode backgrounds
    current_mode = result['mode'][0]
    start_idx = 0
    for i, mode in enumerate(result['mode']):
        if mode != current_mode or i == len(result['mode']) - 1:
            ax1.axvspan(time[start_idx], time[i], alpha=0.4, 
                       color=mode_colors[current_mode], zorder=0)
            current_mode = mode
            start_idx = i
    
    ax1.step(time, result['state'] + 1, where='post', color='black', linewidth=1.5)
    ax1.set_yticks([1, 2, 3, 4])
    ax1.set_yticklabels(states_names)
    ax1.set_ylim(0.5, 4.5)
    ax1.set_xlim(0, 24)
    ax1.set_ylabel('User State')
    ax1.set_title('(a) User Behavior State Trajectory (Markov Chain)', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Add mode legend
    handles = [Patch(facecolor=c, alpha=0.4, label=m.capitalize()) 
               for m, c in mode_colors.items()]
    ax1.legend(handles=handles, loc='upper right', ncol=3)
    
    # Plot 2: Display parameters
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    
    ax2_left = ax2
    ax2_right = ax2.twinx()
    
    ax2_left.plot(time, result['apl'], 'b-', lw=0.8, alpha=0.7, label='APL')
    ax2_left.set_ylabel('APL (%)', color='blue')
    ax2_left.tick_params(axis='y', labelcolor='blue')
    ax2_left.set_ylim(0, 100)
    
    ax2_right.fill_between(time, 0, result['brightness'], alpha=0.3, color='orange')
    ax2_right.set_ylabel('Brightness (nits)', color='orange')
    ax2_right.tick_params(axis='y', labelcolor='orange')
    ax2_right.set_ylim(0, 1500)
    
    ax2.set_title('(b) Display Parameters: APL & Brightness', fontsize=12, fontweight='bold')
    ax2.set_xlim(0, 24)
    
    # Plot 3: CPU parameters
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    
    ax3_left = ax3
    ax3_right = ax3.twinx()
    
    ax3_left.plot(time, result['cpu_util'], 'g-', lw=0.8, alpha=0.7, label='Utilization')
    ax3_left.set_ylabel('CPU Util (%)', color='green')
    ax3_left.tick_params(axis='y', labelcolor='green')
    ax3_left.set_ylim(0, 100)
    
    ax3_right.plot(time, result['cpu_freq'], 'r-', lw=0.8, alpha=0.7)
    ax3_right.set_ylabel('Frequency (GHz)', color='red')
    ax3_right.tick_params(axis='y', labelcolor='red')
    ax3_right.set_ylim(0, 3.5)
    
    ax3.set_title('(c) Processor Parameters: Utilization & Frequency', fontsize=12, fontweight='bold')
    ax3.set_xlim(0, 24)
    
    # Plot 4: Power consumption
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    
    ax4.fill_between(time, 0, result['power'], alpha=0.4, color='purple')
    ax4.plot(time, result['power'], 'purple', lw=1)
    
    avg_power = np.mean(result['power'])
    ax4.axhline(y=avg_power, color='red', linestyle='--', alpha=0.7)
    ax4.text(23.5, avg_power + 100, f'Avg: {avg_power:.0f} mW',
             ha='right', fontsize=10, color='red')
    
    ax4.set_xlabel('Time of Day (Hour)', fontsize=11)
    ax4.set_ylabel('Power (mW)', fontsize=11)
    ax4.set_title('(d) Estimated Power Consumption', fontsize=12, fontweight='bold')
    ax4.set_xlim(0, 24)
    ax4.set_xticks(range(0, 25, 2))
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle('Monte Carlo Simulation: 24-Hour User Behavior Trajectory',
                fontsize=14, fontweight='bold', y=1.01)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_ode_vs_monte_carlo(save_path=None):
    """
    Figure: Comparison of ODE solution vs Monte Carlo ensemble
    
    比较微分方程解与蒙特卡洛集成平均
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # ODE solution
    ode_system = MarkovDifferentialEquations()
    pi0 = np.array([1.0, 0.0, 0.0, 0.0])
    ode_result = ode_system.solve_probability_evolution(
        pi0=pi0, t_span=(0, 24), start_hour=0, num_points=1440
    )
    
    # Monte Carlo ensemble
    n_runs = 100
    simulator = MarkovChainSimulator()
    
    mc_states = np.zeros((n_runs, 1440, 4))
    mc_power = np.zeros((n_runs, 1440))
    
    for i in range(n_runs):
        result = simulator.simulate(duration_hours=24, start_hour=0, initial_state=0)
        # Convert states to one-hot
        for t, s in enumerate(result['state']):
            if t < 1440:
                mc_states[i, t, s] = 1
                mc_power[i, t] = result['power'][t]
    
    mc_probs_mean = np.mean(mc_states, axis=0)
    mc_probs_std = np.std(mc_states, axis=0)
    mc_power_mean = np.mean(mc_power, axis=0)
    mc_power_std = np.std(mc_power, axis=0)
    
    time = ode_result['time']
    states = MarkovStates.names
    colors = ['#4A5568', '#3182CE', '#DD6B20', '#E53E3E']
    
    # Plot probability comparisons for each state
    for i, (ax, name, color) in enumerate(zip(axes.flat[:4], states, colors)):
        # ODE solution
        ax.plot(time, ode_result['probabilities'][:, i], '-', 
               color=color, lw=2, label='ODE Solution')
        
        # Monte Carlo mean + std
        ax.fill_between(time, 
                       mc_probs_mean[:, i] - mc_probs_std[:, i],
                       mc_probs_mean[:, i] + mc_probs_std[:, i],
                       alpha=0.3, color=color)
        ax.plot(time, mc_probs_mean[:, i], '--', 
               color='black', lw=1.5, label=f'MC Mean (n={n_runs})')
        
        ax.set_xlabel('Time (Hours)')
        ax.set_ylabel('Probability')
        ax.set_title(f'{name}')
        ax.set_xlim(0, 24)
        ax.set_ylim(0, 1.05)
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
    
    plt.suptitle('ODE Solution vs Monte Carlo Ensemble (n=100 runs)',
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def plot_differential_equations_summary(save_path=None):
    """
    Figure: Summary of differential equations in the model
    
    微分方程组的数学表述汇总
    """
    fig = plt.figure(figsize=(12, 10))
    
    ax = fig.add_subplot(111)
    ax.axis('off')
    
    # Create text content
    equations = r"""
    \textbf{Coupled Differential Equation System for Battery-Markov Dynamics}
    
    \hrulefill
    
    \textbf{1. Kolmogorov Forward Equation (Probability Evolution)}
    
    $$\frac{d\boldsymbol{\pi}(t)}{dt} = \boldsymbol{\pi}(t) \cdot \mathbf{Q}(t)$$
    
    where $\boldsymbol{\pi}(t) = [\pi_1(t), \pi_2(t), \pi_3(t), \pi_4(t)]$ and $\mathbf{Q}(t)$ is the time-dependent generator matrix.
    
    \hrulefill
    
    \textbf{2. Time-Inhomogeneous Generator Matrix}
    
    $$\mathbf{Q}(t) = \begin{cases} 
    \mathbf{Q}_{sleep} & \text{if } t \in [23:00, 07:00] \\
    \mathbf{Q}_{work} & \text{if } t \in [09:00, 12:00] \cup [14:00, 18:00] \\
    \mathbf{Q}_{leisure} & \text{otherwise}
    \end{cases}$$
    
    Generator matrix derived from transition matrix: $\mathbf{Q} = (\mathbf{P} - \mathbf{I}) / \Delta t$
    
    \hrulefill
    
    \textbf{3. Expected Power Consumption}
    
    $$E[P(t)] = \sum_{i=1}^{4} \pi_i(t) \cdot P_i = \boldsymbol{\pi}(t) \cdot \mathbf{P}_{states}$$
    
    where $\mathbf{P}_{states} = [P_{sleep}, P_{light}, P_{stream}, P_{game}]^T$
    
    \hrulefill
    
    \textbf{4. Battery SOC Evolution}
    
    $$\frac{d(\text{SOC})}{dt} = -\frac{E[P(t)]}{V_{bat} \cdot Q_{max}}$$
    
    \hrulefill
    
    \textbf{5. Thermal Dynamics}
    
    $$C_{th} \frac{dT}{dt} = \eta \cdot E[P(t)] - \frac{T - T_{env}}{R_{th}}$$
    
    \hrulefill
    
    \textbf{6. Stationary Distribution (Long-term Behavior)}
    
    $$\boldsymbol{\pi}^* \cdot \mathbf{P} = \boldsymbol{\pi}^* \quad \Leftrightarrow \quad \boldsymbol{\pi}^* \cdot \mathbf{Q} = \mathbf{0}$$
    """
    
    # Use simpler text rendering
    text_content = """
COUPLED DIFFERENTIAL EQUATION SYSTEM FOR BATTERY-MARKOV DYNAMICS
═══════════════════════════════════════════════════════════════

1. KOLMOGOROV FORWARD EQUATION (Probability Evolution)
   
   dπ(t)/dt = π(t) · Q(t)
   
   where π(t) = [π₁(t), π₂(t), π₃(t), π₄(t)]
   Q(t) is the time-dependent generator matrix

───────────────────────────────────────────────────────────────

2. TIME-INHOMOGENEOUS GENERATOR MATRIX
   
   Q(t) = Q_sleep     if t ∈ [23:00, 07:00]  (Sleep Mode)
          Q_work      if t ∈ [09:00-12:00, 14:00-18:00]  (Work Mode)  
          Q_leisure   otherwise  (Leisure Mode)
   
   Generator: Q = (P - I) / Δt

───────────────────────────────────────────────────────────────

3. EXPECTED POWER CONSUMPTION
   
   E[P(t)] = Σᵢ πᵢ(t) · Pᵢ = π(t) · P_states
   
   P_states = [P_sleep, P_light, P_stream, P_game]ᵀ
            = [50, 800, 1500, 4000]ᵀ mW

───────────────────────────────────────────────────────────────

4. BATTERY SOC EVOLUTION
   
   dSOC/dt = -E[P(t)] / (V_bat · Q_max)
   
   where V_bat = 3.7V, Q_max = 4.5Ah

───────────────────────────────────────────────────────────────

5. THERMAL DYNAMICS (Coupled Heat Equation)
   
   C_th · dT/dt = η · E[P(t)] - (T - T_env) / R_th
   
   Heat generation: Q_gen = η · E[P]
   Heat dissipation: Q_diss = (T - T_env) / R_th

───────────────────────────────────────────────────────────────

6. STATIONARY DISTRIBUTION (Long-term Equilibrium)
   
   π* · P = π*  ⟺  π* · Q = 0
   
   π* is the left eigenvector of P with eigenvalue 1
"""
    
    ax.text(0.5, 0.5, text_content, transform=ax.transAxes,
            fontsize=11, fontfamily='monospace',
            verticalalignment='center', horizontalalignment='center',
            bbox=dict(boxstyle='round', facecolor='#F7FAFC', edgecolor='#4A5568'))
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    return fig


def generate_all_markov_plots(output_dir: str):
    """Generate all Markov chain related plots"""
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating Markov chain visualizations...")
    
    plots = [
        ('fig_time_partitions.png', plot_time_partitions),
        ('fig_transition_matrices.png', plot_transition_matrices),
        ('fig_stationary_distributions.png', plot_stationary_distributions),
        ('fig_probability_evolution.png', plot_probability_evolution),
        ('fig_coupled_system.png', plot_coupled_system_dynamics),
        ('fig_monte_carlo.png', plot_monte_carlo_simulation),
        ('fig_ode_vs_mc.png', plot_ode_vs_monte_carlo),
        ('fig_equations_summary.png', plot_differential_equations_summary),
    ]
    
    for filename, plot_func in plots:
        print(f"  Generating {filename}...")
        fig = plot_func(os.path.join(output_dir, filename))
        plt.close(fig)
    
    print(f"All plots saved to {output_dir}")


if __name__ == "__main__":
    generate_all_markov_plots('/workspace/battery_modeling/figures')
