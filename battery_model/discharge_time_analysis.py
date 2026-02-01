#!/usr/bin/env python3
"""
Remaining Discharge Time Prediction Analysis
=============================================
MCM 2026 Problem A - Task 2

This module performs:
1. Discharge time prediction for various initial SOC and scenarios
2. Comparison with observed/expected battery performance
3. Uncertainty quantification using Monte Carlo simulation
4. Sensitivity analysis to identify key factors
5. Identification of factors with large/small impact

Output:
- Discharge time predictions across scenarios
- Uncertainty bounds (confidence intervals)
- Sensitivity rankings
- Factor impact analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import warnings
warnings.filterwarnings('ignore')

# Import model components
from daily_simulation_english import (
    CoupledBatteryModel, BatteryParameters, ActivityScenarios
)

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 150


# =============================================================================
# Discharge Time Prediction Functions
# =============================================================================

def predict_discharge_time(model, initial_soc, scenario_func, 
                           V_cutoff=3.0, max_time=86400, dt=60.0):
    """
    Predict remaining discharge time from initial SOC to cutoff voltage.
    
    Parameters:
        model: CoupledBatteryModel instance
        initial_soc: Initial state of charge [0-1]
        scenario_func: Scenario function
        V_cutoff: Cutoff voltage [V]
        max_time: Maximum simulation time [s]
        dt: Time step [s]
    
    Returns:
        t_discharge: Time to reach cutoff [hours]
        final_soc: Final SOC [%]
        trajectory: Dict with time series data
    """
    scenario = scenario_func()
    state = [initial_soc, 0.0, 0.0, 25.0, 25.0]
    
    t_data, soc_data, v_data, temp_data, power_data = [], [], [], [], []
    
    t = 0
    while t < max_time:
        # Update state
        dstate = model.state_derivatives(t, state, scenario)
        state = [state[i] + dstate[i] * dt for i in range(5)]
        state[0] = np.clip(state[0], 0.001, 0.999)
        
        # Get voltage
        V_term, I = model.terminal_voltage(state, scenario)
        P_load, _ = model.load_power(scenario, V_term, state[3])
        
        # Store data
        t_data.append(t / 3600)
        soc_data.append(state[0] * 100)
        v_data.append(V_term)
        temp_data.append(state[3])
        power_data.append(P_load * 1000)
        
        # Check termination conditions
        if V_term < V_cutoff or state[0] < 0.01:
            break
        
        t += dt
    
    trajectory = {
        't': np.array(t_data),
        'soc': np.array(soc_data),
        'voltage': np.array(v_data),
        'temperature': np.array(temp_data),
        'power': np.array(power_data)
    }
    
    return t_data[-1], soc_data[-1], trajectory


def monte_carlo_discharge_time(model, initial_soc, scenario_func, 
                                n_samples=100, param_uncertainty=0.1):
    """
    Monte Carlo simulation for uncertainty quantification.
    
    Parameters:
        model: Base CoupledBatteryModel
        initial_soc: Initial SOC
        scenario_func: Scenario function
        n_samples: Number of Monte Carlo samples
        param_uncertainty: Parameter uncertainty (fraction)
    
    Returns:
        mean_time: Mean discharge time [hours]
        std_time: Standard deviation [hours]
        ci_95: 95% confidence interval
        samples: All sample values
    """
    samples = []
    
    for i in range(n_samples):
        # Create model with perturbed parameters
        perturbed_model = CoupledBatteryModel(
            cycle_number=model.cycle_number + int(np.random.normal(0, 20))
        )
        
        # Perturb capacity and resistance
        perturbed_model.Q_max_base *= (1 + np.random.normal(0, param_uncertainty))
        perturbed_model.R_total_base *= (1 + np.random.normal(0, param_uncertainty))
        
        # Perturb initial SOC slightly
        perturbed_soc = initial_soc * (1 + np.random.normal(0, 0.02))
        perturbed_soc = np.clip(perturbed_soc, 0.1, 1.0)
        
        try:
            t_discharge, _, _ = predict_discharge_time(
                perturbed_model, perturbed_soc, scenario_func, dt=120.0
            )
            samples.append(t_discharge)
        except:
            pass
    
    samples = np.array(samples)
    mean_time = np.mean(samples)
    std_time = np.std(samples)
    ci_95 = (np.percentile(samples, 2.5), np.percentile(samples, 97.5))
    
    return mean_time, std_time, ci_95, samples


# =============================================================================
# Scenario Definitions
# =============================================================================

SCENARIOS = {
    'Sleep (Screen Off)': ActivityScenarios.sleep,
    'Office Work': ActivityScenarios.office,
    'Video Streaming': ActivityScenarios.entertainment,
    'Navigation + Audio': ActivityScenarios.commute,
    'Heavy Gaming': lambda: {**ActivityScenarios.entertainment(), 
                              'cpu_load': 0.95, 'display_brightness': 1.0,
                              'display_refresh': 120, '5g_data_rate': 10e6,
                              'name': 'Heavy Gaming'},
    'Weak Signal': lambda: {**ActivityScenarios.office(),
                            '5g_distance': 2000, 'signal_strength': -95,
                            'name': 'Weak Signal'},
}

# Expected/observed battery life (hours) for reference
EXPECTED_BATTERY_LIFE = {
    'Sleep (Screen Off)': (20, 30),      # Typical standby: 20-30 hours
    'Office Work': (6, 10),               # Light use: 6-10 hours
    'Video Streaming': (4, 7),            # Video: 4-7 hours
    'Navigation + Audio': (3, 6),         # Navigation: 3-6 hours
    'Heavy Gaming': (1.5, 3),             # Gaming: 1.5-3 hours
    'Weak Signal': (4, 7),                # Weak signal: reduced by 20-40%
}


# =============================================================================
# Main Analysis Functions
# =============================================================================

def run_discharge_time_analysis(cycle_number=150):
    """
    Run comprehensive discharge time analysis.
    """
    print("="*70)
    print("REMAINING DISCHARGE TIME PREDICTION ANALYSIS")
    print("="*70)
    
    model = CoupledBatteryModel(cycle_number=cycle_number)
    
    # Initial SOC levels to test
    initial_socs = [1.0, 0.8, 0.6, 0.4, 0.2]
    
    results = {}
    
    # 1. Deterministic predictions for all combinations
    print("\n1. Computing discharge times for all scenarios...")
    print("-"*70)
    
    for scenario_name, scenario_func in SCENARIOS.items():
        print(f"\n  Scenario: {scenario_name}")
        results[scenario_name] = {}
        
        for soc in initial_socs:
            t_discharge, final_soc, trajectory = predict_discharge_time(
                model, soc, scenario_func
            )
            results[scenario_name][soc] = {
                't_discharge': t_discharge,
                'final_soc': final_soc,
                'avg_power': np.mean(trajectory['power']),
                'peak_temp': np.max(trajectory['temperature']),
                'trajectory': trajectory
            }
            print(f"    SOC={soc*100:.0f}%: {t_discharge:.2f} hours "
                  f"(Avg Power: {np.mean(trajectory['power']):.0f} mW)")
    
    return results, model


def run_uncertainty_analysis(model, n_samples=50):
    """
    Run Monte Carlo uncertainty analysis.
    """
    print("\n\n2. Uncertainty Quantification (Monte Carlo)...")
    print("-"*70)
    
    uncertainty_results = {}
    
    for scenario_name, scenario_func in SCENARIOS.items():
        print(f"\n  Analyzing: {scenario_name}...", end=" ")
        
        mean_time, std_time, ci_95, samples = monte_carlo_discharge_time(
            model, 1.0, scenario_func, n_samples=n_samples
        )
        
        uncertainty_results[scenario_name] = {
            'mean': mean_time,
            'std': std_time,
            'ci_95': ci_95,
            'samples': samples,
            'cv': std_time / mean_time * 100  # Coefficient of variation
        }
        
        print(f"{mean_time:.2f} ± {std_time:.2f} hours (CV: {std_time/mean_time*100:.1f}%)")
    
    return uncertainty_results


def run_sensitivity_analysis(model):
    """
    Run sensitivity analysis to identify key factors.
    """
    print("\n\n3. Sensitivity Analysis...")
    print("-"*70)
    
    # Base case
    base_scenario = ActivityScenarios.office()
    base_time, _, _ = predict_discharge_time(model, 1.0, lambda: base_scenario)
    
    # Parameters to vary
    factors = {
        'Display Brightness': ('display_brightness', [0.2, 0.5, 0.8, 1.0]),
        'CPU Load': ('cpu_load', [0.1, 0.3, 0.5, 0.8]),
        'Refresh Rate': ('display_refresh', [30, 60, 90, 120]),
        'Data Rate': ('5g_data_rate', [1e6, 5e6, 15e6, 30e6]),
        'Distance to BS': ('5g_distance', [200, 500, 1000, 2000]),
        'Ambient Temp': ('T_env', [0, 15, 25, 40]),
        'BT Streaming': ('bt_streaming', [False, True]),
        'GNSS Active': ('gnss_active', [False, True]),
        'Battery Age (N)': ('cycle_number', [0, 100, 200, 400]),
    }
    
    sensitivity_results = {}
    
    for factor_name, (param_key, values) in factors.items():
        times = []
        
        for val in values:
            if param_key == 'cycle_number':
                test_model = CoupledBatteryModel(cycle_number=val)
                scenario = base_scenario.copy()
            else:
                test_model = model
                scenario = base_scenario.copy()
                scenario[param_key] = val
            
            try:
                t, _, _ = predict_discharge_time(test_model, 1.0, lambda s=scenario: s)
                times.append(t)
            except:
                times.append(np.nan)
        
        # Calculate sensitivity (range / mean)
        times = np.array(times)
        valid_times = times[~np.isnan(times)]
        if len(valid_times) > 1:
            sensitivity = (np.max(valid_times) - np.min(valid_times)) / np.mean(valid_times)
        else:
            sensitivity = 0
        
        sensitivity_results[factor_name] = {
            'values': values,
            'times': times,
            'sensitivity': sensitivity,
            'range': (np.nanmin(times), np.nanmax(times))
        }
    
    # Sort by sensitivity
    sorted_factors = sorted(sensitivity_results.items(), 
                           key=lambda x: x[1]['sensitivity'], reverse=True)
    
    print("\n  Factor Sensitivity Ranking:")
    print("  " + "-"*50)
    for i, (name, data) in enumerate(sorted_factors, 1):
        print(f"  {i:2d}. {name:20s}: Sensitivity={data['sensitivity']:.3f} "
              f"(Range: {data['range'][0]:.1f}-{data['range'][1]:.1f} hrs)")
    
    return sensitivity_results


def analyze_drain_causes():
    """
    Analyze specific causes of battery drain for each scenario.
    """
    print("\n\n4. Battery Drain Cause Analysis...")
    print("-"*70)
    
    model = CoupledBatteryModel(cycle_number=150)
    
    drain_analysis = {}
    
    for scenario_name, scenario_func in SCENARIOS.items():
        scenario = scenario_func()
        
        # Calculate power breakdown
        V_bat = 3.8
        T_c = 30.0
        P_total, breakdown = model.load_power(scenario, V_bat, T_c)
        
        # Sort by contribution
        sorted_breakdown = sorted(breakdown.items(), key=lambda x: x[1], reverse=True)
        
        drain_analysis[scenario_name] = {
            'total_power': P_total * 1000,
            'breakdown': {k: v*1000 for k, v in sorted_breakdown},
            'percentages': {k: v/P_total*100 for k, v in sorted_breakdown},
            'top_consumer': sorted_breakdown[0][0]
        }
    
    print("\n  Power Breakdown by Scenario:")
    print("  " + "-"*60)
    
    for scenario_name, data in drain_analysis.items():
        print(f"\n  {scenario_name}:")
        print(f"    Total Power: {data['total_power']:.0f} mW")
        print(f"    Top Consumer: {data['top_consumer']}")
        print("    Breakdown:")
        for component, pct in data['percentages'].items():
            if pct > 1:
                print(f"      - {component}: {data['breakdown'][component]:.0f} mW ({pct:.1f}%)")
    
    return drain_analysis


def compare_with_expected():
    """
    Compare model predictions with expected/observed values.
    """
    print("\n\n5. Model Validation: Predicted vs Expected...")
    print("-"*70)
    
    model = CoupledBatteryModel(cycle_number=150)
    
    comparison = {}
    
    print("\n  {:25s} {:>12s} {:>15s} {:>10s}".format(
        "Scenario", "Predicted", "Expected", "Status"))
    print("  " + "-"*62)
    
    for scenario_name, scenario_func in SCENARIOS.items():
        t_pred, _, _ = predict_discharge_time(model, 1.0, scenario_func)
        expected = EXPECTED_BATTERY_LIFE.get(scenario_name, (0, 100))
        
        in_range = expected[0] <= t_pred <= expected[1]
        status = "✓ Good" if in_range else ("↑ High" if t_pred > expected[1] else "↓ Low")
        
        comparison[scenario_name] = {
            'predicted': t_pred,
            'expected': expected,
            'in_range': in_range,
            'error': (t_pred - (expected[0]+expected[1])/2) / ((expected[0]+expected[1])/2) * 100
        }
        
        print(f"  {scenario_name:25s} {t_pred:8.2f} hrs   {expected[0]:.1f}-{expected[1]:.1f} hrs   {status}")
    
    return comparison


def identify_high_low_impact_factors(sensitivity_results):
    """
    Identify factors with high and surprisingly low impact.
    """
    print("\n\n6. Factor Impact Analysis...")
    print("-"*70)
    
    sorted_factors = sorted(sensitivity_results.items(), 
                           key=lambda x: x[1]['sensitivity'], reverse=True)
    
    # High impact factors (top 3)
    print("\n  HIGH IMPACT FACTORS (Most significant):")
    print("  " + "-"*50)
    for name, data in sorted_factors[:3]:
        print(f"  • {name}")
        print(f"    - Sensitivity: {data['sensitivity']:.3f}")
        print(f"    - Battery life range: {data['range'][0]:.1f} - {data['range'][1]:.1f} hours")
        print(f"    - Can reduce battery life by up to {(1-data['range'][0]/data['range'][1])*100:.0f}%")
    
    # Low impact factors (bottom 3)
    print("\n  SURPRISINGLY LOW IMPACT FACTORS:")
    print("  " + "-"*50)
    for name, data in sorted_factors[-3:]:
        print(f"  • {name}")
        print(f"    - Sensitivity: {data['sensitivity']:.3f}")
        print(f"    - Battery life range: {data['range'][0]:.1f} - {data['range'][1]:.1f} hours")
        print(f"    - Only affects battery life by ~{abs(data['range'][1]-data['range'][0])/data['range'][1]*100:.0f}%")
    
    return sorted_factors


# =============================================================================
# Visualization Functions
# =============================================================================

def plot_discharge_time_comparison(results, save_path=None):
    """
    Plot discharge time predictions across scenarios and SOC levels.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Panel (a): Bar chart of discharge times at 100% SOC
    ax1 = axes[0]
    scenarios = list(results.keys())
    times = [results[s][1.0]['t_discharge'] for s in scenarios]
    expected_low = [EXPECTED_BATTERY_LIFE[s][0] for s in scenarios]
    expected_high = [EXPECTED_BATTERY_LIFE[s][1] for s in scenarios]
    
    x = np.arange(len(scenarios))
    width = 0.35
    
    bars = ax1.bar(x, times, width, label='Model Prediction', color='steelblue', alpha=0.8)
    ax1.errorbar(x, [(l+h)/2 for l, h in zip(expected_low, expected_high)],
                 yerr=[(h-l)/2 for l, h in zip(expected_low, expected_high)],
                 fmt='ro', capsize=5, label='Expected Range', markersize=8)
    
    ax1.set_xlabel('Usage Scenario')
    ax1.set_ylabel('Discharge Time [hours]')
    ax1.set_title('(a) Model Prediction vs Expected Battery Life')
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.replace(' ', '\n') for s in scenarios], fontsize=8)
    ax1.legend()
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Panel (b): Discharge curves for different scenarios
    ax2 = axes[1]
    colors = plt.cm.tab10(np.linspace(0, 1, len(scenarios)))
    
    for i, (scenario, color) in enumerate(zip(scenarios, colors)):
        traj = results[scenario][1.0]['trajectory']
        ax2.plot(traj['t'], traj['soc'], color=color, linewidth=2, 
                 label=scenario.split()[0])
    
    ax2.set_xlabel('Time [hours]')
    ax2.set_ylabel('State of Charge [%]')
    ax2.set_title('(b) SOC Discharge Curves (Starting at 100%)')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, None])
    ax2.set_ylim([0, 100])
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved: {save_path}")
    
    return fig


def plot_uncertainty_analysis(uncertainty_results, save_path=None):
    """
    Plot uncertainty quantification results.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    scenarios = list(uncertainty_results.keys())
    means = [uncertainty_results[s]['mean'] for s in scenarios]
    stds = [uncertainty_results[s]['std'] for s in scenarios]
    cvs = [uncertainty_results[s]['cv'] for s in scenarios]
    
    # Panel (a): Mean and uncertainty bars
    ax1 = axes[0]
    x = np.arange(len(scenarios))
    
    ax1.bar(x, means, yerr=stds, capsize=5, color='steelblue', alpha=0.8,
            error_kw={'elinewidth': 2, 'capthick': 2})
    
    # Add 95% CI as error region
    for i, s in enumerate(scenarios):
        ci = uncertainty_results[s]['ci_95']
        ax1.plot([i, i], [ci[0], ci[1]], 'r-', linewidth=3, alpha=0.7)
    
    ax1.set_xlabel('Usage Scenario')
    ax1.set_ylabel('Discharge Time [hours]')
    ax1.set_title('(a) Discharge Time with Uncertainty (Mean ± 1σ)')
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.replace(' ', '\n') for s in scenarios], fontsize=8)
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Panel (b): Coefficient of variation
    ax2 = axes[1]
    colors = ['green' if cv < 10 else 'orange' if cv < 20 else 'red' for cv in cvs]
    bars = ax2.bar(x, cvs, color=colors, alpha=0.8)
    
    ax2.axhline(y=10, color='green', linestyle='--', alpha=0.5, label='Low uncertainty')
    ax2.axhline(y=20, color='red', linestyle='--', alpha=0.5, label='High uncertainty')
    
    ax2.set_xlabel('Usage Scenario')
    ax2.set_ylabel('Coefficient of Variation [%]')
    ax2.set_title('(b) Prediction Uncertainty (CV%)')
    ax2.set_xticks(x)
    ax2.set_xticklabels([s.replace(' ', '\n') for s in scenarios], fontsize=8)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved: {save_path}")
    
    return fig


def plot_sensitivity_analysis(sensitivity_results, save_path=None):
    """
    Plot sensitivity analysis results.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Sort by sensitivity
    sorted_factors = sorted(sensitivity_results.items(), 
                           key=lambda x: x[1]['sensitivity'], reverse=True)
    
    # Panel (a): Sensitivity ranking
    ax1 = axes[0]
    names = [f[0] for f in sorted_factors]
    sensitivities = [f[1]['sensitivity'] for f in sorted_factors]
    
    colors = ['darkred' if s > 0.5 else 'orange' if s > 0.2 else 'green' 
              for s in sensitivities]
    
    y = np.arange(len(names))
    ax1.barh(y, sensitivities, color=colors, alpha=0.8)
    ax1.set_yticks(y)
    ax1.set_yticklabels(names)
    ax1.set_xlabel('Sensitivity Index')
    ax1.set_title('(a) Factor Sensitivity Ranking')
    ax1.axvline(x=0.5, color='red', linestyle='--', alpha=0.5)
    ax1.axvline(x=0.2, color='orange', linestyle='--', alpha=0.5)
    ax1.grid(True, alpha=0.3, axis='x')
    ax1.invert_yaxis()
    
    # Panel (b): Battery life range for top factors
    ax2 = axes[1]
    top_factors = sorted_factors[:5]
    
    for i, (name, data) in enumerate(top_factors):
        values = data['values']
        times = data['times']
        
        # Normalize values for plotting
        if isinstance(values[0], bool):
            x_vals = [0, 1]
            x_labels = ['Off', 'On']
        else:
            x_vals = np.array(values)
            if max(values) > 1000:
                x_vals = x_vals / 1e6
                x_label_suffix = ' (M)'
            else:
                x_label_suffix = ''
        
        valid_idx = ~np.isnan(times)
        ax2.plot(np.array(values)[valid_idx], np.array(times)[valid_idx], 
                'o-', linewidth=2, markersize=6, label=name)
    
    ax2.set_xlabel('Parameter Value')
    ax2.set_ylabel('Battery Life [hours]')
    ax2.set_title('(b) Battery Life vs Key Parameters')
    ax2.legend(loc='best', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved: {save_path}")
    
    return fig


def plot_power_breakdown(drain_analysis, save_path=None):
    """
    Plot power breakdown analysis.
    """
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    axes = axes.flatten()
    
    colors = plt.cm.Set3(np.linspace(0, 1, 6))
    
    for i, (scenario_name, data) in enumerate(drain_analysis.items()):
        if i >= 6:
            break
            
        ax = axes[i]
        
        components = list(data['breakdown'].keys())
        powers = list(data['breakdown'].values())
        
        wedges, texts, autotexts = ax.pie(
            powers, labels=components, autopct='%1.0f%%',
            colors=colors, startangle=90,
            textprops={'fontsize': 8}
        )
        
        ax.set_title(f'{scenario_name}\n(Total: {data["total_power"]:.0f} mW)', 
                     fontsize=10)
    
    plt.suptitle('Power Consumption Breakdown by Scenario', 
                 fontsize=13, fontweight='bold', y=1.02)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved: {save_path}")
    
    return fig


def plot_comprehensive_results(results, uncertainty_results, sensitivity_results, 
                               drain_analysis, comparison, save_path=None):
    """
    Create comprehensive summary figure.
    """
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.3)
    
    scenarios = list(results.keys())
    
    # (a) Discharge time prediction vs expected
    ax1 = fig.add_subplot(gs[0, 0])
    x = np.arange(len(scenarios))
    times_pred = [results[s][1.0]['t_discharge'] for s in scenarios]
    times_exp_mid = [(EXPECTED_BATTERY_LIFE[s][0] + EXPECTED_BATTERY_LIFE[s][1])/2 
                     for s in scenarios]
    times_exp_err = [(EXPECTED_BATTERY_LIFE[s][1] - EXPECTED_BATTERY_LIFE[s][0])/2 
                     for s in scenarios]
    
    ax1.bar(x - 0.2, times_pred, 0.4, label='Predicted', color='steelblue', alpha=0.8)
    ax1.errorbar(x + 0.2, times_exp_mid, yerr=times_exp_err, fmt='o', 
                 color='darkred', capsize=5, label='Expected', markersize=8)
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax1.set_ylabel('Hours')
    ax1.set_title('(a) Predicted vs Expected Battery Life')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # (b) Uncertainty analysis
    ax2 = fig.add_subplot(gs[0, 1])
    means = [uncertainty_results[s]['mean'] for s in scenarios]
    stds = [uncertainty_results[s]['std'] for s in scenarios]
    ax2.bar(x, means, yerr=stds, capsize=4, color='teal', alpha=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax2.set_ylabel('Hours')
    ax2.set_title('(b) Discharge Time (Mean ± σ)')
    ax2.grid(True, alpha=0.3, axis='y')
    
    # (c) Sensitivity ranking
    ax3 = fig.add_subplot(gs[0, 2])
    sorted_sens = sorted(sensitivity_results.items(), key=lambda x: x[1]['sensitivity'], reverse=True)
    names = [f[0][:15] for f in sorted_sens[:7]]
    sens_vals = [f[1]['sensitivity'] for f in sorted_sens[:7]]
    colors = ['darkred' if s > 0.5 else 'orange' if s > 0.2 else 'green' for s in sens_vals]
    ax3.barh(range(len(names)), sens_vals, color=colors, alpha=0.8)
    ax3.set_yticks(range(len(names)))
    ax3.set_yticklabels(names, fontsize=8)
    ax3.set_xlabel('Sensitivity')
    ax3.set_title('(c) Factor Sensitivity Ranking')
    ax3.invert_yaxis()
    ax3.grid(True, alpha=0.3, axis='x')
    
    # (d-f) Discharge curves for 3 scenarios
    scenario_subset = ['Sleep (Screen Off)', 'Office Work', 'Heavy Gaming']
    for i, scenario in enumerate(scenario_subset):
        ax = fig.add_subplot(gs[1, i])
        
        for soc in [1.0, 0.8, 0.6, 0.4]:
            traj = results[scenario][soc]['trajectory']
            ax.plot(traj['t'], traj['soc'], linewidth=2, 
                    label=f'SOC₀={soc*100:.0f}%')
        
        ax.set_xlabel('Time [hours]')
        ax.set_ylabel('SOC [%]')
        ax.set_title(f'(d-{i+1}) {scenario.split()[0]}')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_ylim([0, 100])
    
    # (g) Average power by scenario
    ax7 = fig.add_subplot(gs[2, 0])
    avg_powers = [results[s][1.0]['avg_power'] for s in scenarios]
    colors = plt.cm.Reds(np.linspace(0.3, 1, len(scenarios)))
    ax7.bar(x, avg_powers, color=colors, alpha=0.8)
    ax7.set_xticks(x)
    ax7.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax7.set_ylabel('Power [mW]')
    ax7.set_title('(g) Average Power Consumption')
    ax7.grid(True, alpha=0.3, axis='y')
    
    # (h) Temperature rise
    ax8 = fig.add_subplot(gs[2, 1])
    peak_temps = [results[s][1.0]['peak_temp'] for s in scenarios]
    ax8.bar(x, peak_temps, color='coral', alpha=0.8)
    ax8.axhline(y=25, color='blue', linestyle='--', alpha=0.5, label='Ambient')
    ax8.set_xticks(x)
    ax8.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax8.set_ylabel('Temperature [°C]')
    ax8.set_title('(h) Peak Battery Temperature')
    ax8.legend()
    ax8.grid(True, alpha=0.3, axis='y')
    
    # (i) Model accuracy
    ax9 = fig.add_subplot(gs[2, 2])
    errors = [comparison[s]['error'] for s in scenarios]
    in_range = [comparison[s]['in_range'] for s in scenarios]
    colors = ['green' if ir else 'red' for ir in in_range]
    ax9.bar(x, errors, color=colors, alpha=0.8)
    ax9.axhline(y=0, color='black', linewidth=1)
    ax9.axhline(y=20, color='orange', linestyle='--', alpha=0.5)
    ax9.axhline(y=-20, color='orange', linestyle='--', alpha=0.5)
    ax9.set_xticks(x)
    ax9.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax9.set_ylabel('Error [%]')
    ax9.set_title('(i) Prediction Error vs Expected')
    ax9.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Comprehensive Discharge Time Prediction Analysis', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved: {save_path}")
    
    return fig


# =============================================================================
# Main Execution
# =============================================================================

def main():
    """Run complete discharge time analysis."""
    import os
    os.makedirs('output/results', exist_ok=True)
    
    # 1. Run discharge time predictions
    results, model = run_discharge_time_analysis(cycle_number=150)
    
    # 2. Uncertainty quantification
    uncertainty_results = run_uncertainty_analysis(model, n_samples=30)
    
    # 3. Sensitivity analysis
    sensitivity_results = run_sensitivity_analysis(model)
    
    # 4. Drain cause analysis
    drain_analysis = analyze_drain_causes()
    
    # 5. Compare with expected
    comparison = compare_with_expected()
    
    # 6. Identify high/low impact factors
    identify_high_low_impact_factors(sensitivity_results)
    
    # 7. Generate figures
    print("\n\n7. Generating Visualization Figures...")
    print("-"*70)
    
    fig1 = plot_discharge_time_comparison(results, 
        'output/results/discharge_time_comparison.png')
    plt.close(fig1)
    
    fig2 = plot_uncertainty_analysis(uncertainty_results,
        'output/results/uncertainty_analysis.png')
    plt.close(fig2)
    
    fig3 = plot_sensitivity_analysis(sensitivity_results,
        'output/results/sensitivity_analysis.png')
    plt.close(fig3)
    
    fig4 = plot_power_breakdown(drain_analysis,
        'output/results/power_breakdown.png')
    plt.close(fig4)
    
    fig5 = plot_comprehensive_results(results, uncertainty_results, 
        sensitivity_results, drain_analysis, comparison,
        'output/results/comprehensive_discharge_analysis.png')
    plt.close(fig5)
    
    # 8. Summary
    print("\n\n" + "="*70)
    print("ANALYSIS SUMMARY")
    print("="*70)
    
    print("\n■ MODEL PERFORMANCE:")
    good_count = sum(1 for c in comparison.values() if c['in_range'])
    print(f"  - Predictions within expected range: {good_count}/{len(comparison)}")
    print(f"  - Average absolute error: {np.mean([abs(c['error']) for c in comparison.values()]):.1f}%")
    
    print("\n■ HIGHEST IMPACT FACTORS:")
    sorted_sens = sorted(sensitivity_results.items(), key=lambda x: x[1]['sensitivity'], reverse=True)
    for name, data in sorted_sens[:3]:
        print(f"  - {name}: can change battery life by {(data['range'][1]-data['range'][0]):.1f} hours")
    
    print("\n■ LOWEST IMPACT FACTORS:")
    for name, data in sorted_sens[-3:]:
        print(f"  - {name}: only {data['sensitivity']*100:.1f}% sensitivity")
    
    print("\n■ PRIMARY DRAIN CAUSES BY SCENARIO:")
    for scenario, data in drain_analysis.items():
        print(f"  - {scenario}: {data['top_consumer']} ({data['percentages'][data['top_consumer']]:.0f}%)")
    
    return results, uncertainty_results, sensitivity_results, drain_analysis, comparison


if __name__ == "__main__":
    results = main()
