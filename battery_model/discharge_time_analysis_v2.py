#!/usr/bin/env python3
"""
Remaining Discharge Time Prediction Analysis (Calibrated Version)
=================================================================
MCM 2026 Problem A - Task 2

Properly calibrated model for realistic battery life predictions.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 150


# =============================================================================
# Calibrated Battery Model
# =============================================================================

class CalibratedBatteryModel:
    """
    Calibrated coupled electro-thermal-aging model.
    
    Key equations:
    - SOC: dz/dt = -I(t)·η / (Q_max·3600)
    - Voltage: V_term = V_OCV(z) - I·R_total
    - Thermal: dT/dt = (I²R - (T-T_env)/R_th) / C_th
    """
    
    def __init__(self, cycle_number=150, capacity_ah=4.0):
        # Battery specifications
        self.Q_nom = capacity_ah          # Nominal capacity [Ah]
        self.V_nom = 3.85                 # Nominal voltage [V]
        self.V_max = 4.35                 # Max voltage [V]
        self.V_min = 3.0                  # Cutoff voltage [V]
        
        # Aging parameters
        self.cycle_number = cycle_number
        self._compute_aging()
        
        # OCV parameters (Nernst model)
        self.K = [3.45, 0.18, -0.005, 0.06, -0.09]
        
        # Thermal parameters
        self.C_th = 40.0                  # Thermal capacity [J/K]
        self.R_th = 15.0                  # Thermal resistance [K/W]
        self.dV_dT = 0.0003               # Entropy coefficient [V/K]
        
    def _compute_aging(self):
        """Compute aging-dependent parameters."""
        N = self.cycle_number
        
        # Capacity fade (double-exponential)
        self.Q_max = -0.12 * np.exp(-0.02 * N) + (self.Q_nom + 0.12) * np.exp(-0.0006 * N)
        self.Q_max = max(self.Q_max, 0.5)
        
        # Impedance growth (power-law) - calibrated for realistic values
        # Typical internal resistance: 50-150 mΩ
        self.R_internal = 0.050 + 0.008 * (N ** 0.25)  # [Ω]
    
    def capacity_factor(self, T):
        """Temperature capacity factor."""
        return 1.02 / (1 + np.exp(-0.1 * (T + 12)))
    
    def resistance_factor(self, T):
        """Temperature resistance factor."""
        return 0.75 + 1.2 * np.exp(-0.055 * T)
    
    def get_capacity(self, T):
        """Effective capacity at temperature T."""
        return self.Q_max * self.capacity_factor(T)
    
    def get_resistance(self, T):
        """Effective resistance at temperature T."""
        return self.R_internal * self.resistance_factor(T)
    
    def ocv(self, z):
        """Open circuit voltage from SOC."""
        z = np.clip(z, 0.001, 0.999)
        K = self.K
        return K[0] + K[1]*z + K[2]/z + K[3]*np.log(z) + K[4]*np.log(1-z)
    
    def terminal_voltage(self, z, I, T):
        """Terminal voltage under load."""
        V_ocv = self.ocv(z)
        R = self.get_resistance(T)
        return V_ocv - I * R


# =============================================================================
# Scenario Power Models
# =============================================================================

def get_scenario_power(scenario_name, T=25.0):
    """
    Get power consumption for each scenario.
    Returns power in Watts.
    
    Calibrated against typical smartphone measurements.
    """
    power_profiles = {
        'Sleep (Screen Off)': {
            'total': 0.050,  # 50 mW standby
            'breakdown': {'5G': 0.020, 'Background': 0.015, 'SoC': 0.010, 'Display': 0.005}
        },
        'Idle (Screen On)': {
            'total': 0.350,  # 350 mW light use
            'breakdown': {'Display': 0.200, '5G': 0.050, 'SoC': 0.060, 'Background': 0.040}
        },
        'Office Work': {
            'total': 0.550,  # 550 mW office use
            'breakdown': {'Display': 0.280, '5G': 0.080, 'SoC': 0.120, 'Background': 0.070}
        },
        'Video Streaming': {
            'total': 1.200,  # 1.2W video
            'breakdown': {'Display': 0.500, 'SoC': 0.350, '5G': 0.300, 'Background': 0.050}
        },
        'Navigation + Audio': {
            'total': 1.500,  # 1.5W navigation
            'breakdown': {'Display': 0.450, '5G': 0.400, 'SoC': 0.350, 'GNSS': 0.250, 'BT': 0.050}
        },
        'Heavy Gaming': {
            'total': 4.500,  # 4.5W gaming
            'breakdown': {'SoC': 2.800, 'Display': 1.400, '5G': 0.200, 'Background': 0.100}
        },
        'Weak Signal': {
            'total': 1.800,  # Higher due to TX power
            'breakdown': {'5G': 1.200, 'Display': 0.350, 'SoC': 0.150, 'Background': 0.100}
        },
    }
    
    return power_profiles.get(scenario_name, power_profiles['Office Work'])


SCENARIOS = [
    'Sleep (Screen Off)',
    'Idle (Screen On)',
    'Office Work', 
    'Video Streaming',
    'Navigation + Audio',
    'Heavy Gaming',
    'Weak Signal'
]

# Expected battery life (hours) from typical usage
EXPECTED_BATTERY_LIFE = {
    'Sleep (Screen Off)': (40, 60),      # 2-3 days standby
    'Idle (Screen On)': (8, 14),         # Light use
    'Office Work': (6, 10),              # Moderate use
    'Video Streaming': (5, 8),           # Video playback
    'Navigation + Audio': (4, 7),        # Navigation
    'Heavy Gaming': (1.5, 3.0),          # Intensive gaming
    'Weak Signal': (4, 7),               # Reduced due to TX power
}


# =============================================================================
# Discharge Prediction Functions  
# =============================================================================

def predict_discharge_time(model, initial_soc, scenario_name, 
                           T_env=25.0, dt=60.0, max_hours=72):
    """
    Predict time to discharge from initial SOC to cutoff.
    
    Uses Coulomb counting with thermal feedback.
    """
    z = initial_soc
    T = T_env
    
    power_data = get_scenario_power(scenario_name, T)
    P_load = power_data['total']
    
    t_data, soc_data, v_data, temp_data, power_list = [], [], [], [], []
    
    t = 0
    max_time = max_hours * 3600
    
    while t < max_time and z > 0.01:
        # Get effective capacity and resistance
        Q_eff = model.get_capacity(T)
        R_eff = model.get_resistance(T)
        
        # Calculate current
        V_ocv = model.ocv(z)
        
        # Solve for current: P = V*I, V = Vocv - I*R
        # P = I*(Vocv - I*R) => I*R*I - Vocv*I + P = 0
        # I = (Vocv - sqrt(Vocv^2 - 4*R*P)) / (2*R)
        discriminant = V_ocv**2 - 4 * R_eff * P_load
        if discriminant < 0:
            I = P_load / V_ocv  # Approximate
        else:
            I = (V_ocv - np.sqrt(discriminant)) / (2 * R_eff)
        
        I = max(I, P_load / model.V_max)  # Ensure positive current
        
        V_term = V_ocv - I * R_eff
        
        # Check voltage cutoff
        if V_term < model.V_min:
            break
        
        # SOC dynamics
        eta = 0.995  # Coulombic efficiency
        dz = -I * eta * dt / (Q_eff * 3600)
        z = max(z + dz, 0.001)
        
        # Thermal dynamics
        Q_gen = I**2 * R_eff
        dT = (Q_gen - (T - T_env) / model.R_th) / model.C_th * dt
        T = T + dT
        
        # Store
        t_data.append(t / 3600)
        soc_data.append(z * 100)
        v_data.append(V_term)
        temp_data.append(T)
        power_list.append(P_load * 1000)
        
        t += dt
    
    trajectory = {
        't': np.array(t_data),
        'soc': np.array(soc_data),
        'voltage': np.array(v_data),
        'temperature': np.array(temp_data),
        'power': np.array(power_list)
    }
    
    return t_data[-1] if t_data else 0, soc_data[-1] if soc_data else 0, trajectory


def monte_carlo_analysis(model, scenario_name, initial_soc=1.0, n_samples=100):
    """Monte Carlo uncertainty quantification."""
    samples = []
    
    for _ in range(n_samples):
        # Perturb model parameters
        test_model = CalibratedBatteryModel(
            cycle_number=model.cycle_number + int(np.random.normal(0, 30)),
            capacity_ah=model.Q_nom * (1 + np.random.normal(0, 0.05))
        )
        
        # Perturb initial SOC
        soc = initial_soc * (1 + np.random.normal(0, 0.02))
        soc = np.clip(soc, 0.1, 1.0)
        
        # Perturb temperature
        T_env = 25 + np.random.normal(0, 3)
        
        try:
            t_discharge, _, _ = predict_discharge_time(
                test_model, soc, scenario_name, T_env, dt=120.0
            )
            if t_discharge > 0:
                samples.append(t_discharge)
        except:
            pass
    
    samples = np.array(samples)
    return {
        'mean': np.mean(samples),
        'std': np.std(samples),
        'ci_95': (np.percentile(samples, 2.5), np.percentile(samples, 97.5)),
        'cv': np.std(samples) / np.mean(samples) * 100 if np.mean(samples) > 0 else 0,
        'samples': samples
    }


def sensitivity_analysis(model):
    """Analyze sensitivity to key parameters."""
    base_time, _, _ = predict_discharge_time(model, 1.0, 'Office Work')
    
    factors = {
        'Display Brightness': {
            'param': 'display_brightness',
            'values': [0.2, 0.5, 0.8, 1.0],
            'power_mult': [0.7, 1.0, 1.3, 1.6]
        },
        'CPU Load': {
            'values': [0.1, 0.3, 0.6, 0.9],
            'power_mult': [0.6, 1.0, 1.8, 3.0]
        },
        'Ambient Temperature': {
            'values': [-10, 10, 25, 40],
            'temp_effect': True
        },
        'Battery Age (cycles)': {
            'values': [0, 100, 300, 500],
            'age_effect': True
        },
        'Signal Strength': {
            'values': ['Strong', 'Normal', 'Weak', 'Very Weak'],
            'power_mult': [0.8, 1.0, 1.5, 2.5]
        },
    }
    
    results = {}
    
    for factor_name, config in factors.items():
        times = []
        
        if 'age_effect' in config:
            for cycle in config['values']:
                test_model = CalibratedBatteryModel(cycle_number=cycle)
                t, _, _ = predict_discharge_time(test_model, 1.0, 'Office Work')
                times.append(t)
        elif 'temp_effect' in config:
            for temp in config['values']:
                t, _, _ = predict_discharge_time(model, 1.0, 'Office Work', T_env=temp)
                times.append(t)
        else:
            base_power = get_scenario_power('Office Work')['total']
            for mult in config['power_mult']:
                # Simulate by adjusting effective power
                adjusted_power = base_power * mult
                Q = model.get_capacity(25)
                V = model.V_nom
                t = Q * V / adjusted_power
                times.append(t)
        
        times = np.array(times)
        sensitivity = (np.max(times) - np.min(times)) / np.mean(times) if np.mean(times) > 0 else 0
        
        results[factor_name] = {
            'values': config['values'],
            'times': times,
            'sensitivity': sensitivity,
            'range': (np.min(times), np.max(times))
        }
    
    return results


# =============================================================================
# Main Analysis
# =============================================================================

def run_comprehensive_analysis():
    """Run complete discharge time analysis."""
    print("="*70)
    print("REMAINING DISCHARGE TIME PREDICTION ANALYSIS")
    print("Coupled Electro-Thermal-Aging Model")
    print("="*70)
    
    model = CalibratedBatteryModel(cycle_number=150, capacity_ah=4.0)
    
    print(f"\nModel Parameters:")
    print(f"  - Nominal Capacity: {model.Q_nom:.2f} Ah")
    print(f"  - Effective Capacity (after aging): {model.Q_max:.2f} Ah")
    print(f"  - Internal Resistance: {model.R_internal*1000:.1f} mΩ")
    print(f"  - Cycle Count: {model.cycle_number}")
    
    # 1. Discharge time predictions
    print("\n" + "="*70)
    print("1. DISCHARGE TIME PREDICTIONS")
    print("="*70)
    
    results = {}
    initial_socs = [1.0, 0.8, 0.6, 0.4, 0.2]
    
    print(f"\n{'Scenario':<25} {'100%':>8} {'80%':>8} {'60%':>8} {'40%':>8} {'20%':>8} {'Power':>10}")
    print("-"*80)
    
    for scenario in SCENARIOS:
        results[scenario] = {}
        times_str = []
        
        power_info = get_scenario_power(scenario)
        
        for soc in initial_socs:
            t, final_soc, traj = predict_discharge_time(model, soc, scenario)
            results[scenario][soc] = {
                't_discharge': t,
                'final_soc': final_soc,
                'trajectory': traj,
                'avg_power': power_info['total'] * 1000
            }
            times_str.append(f"{t:.1f}h")
        
        print(f"{scenario:<25} {times_str[0]:>8} {times_str[1]:>8} {times_str[2]:>8} "
              f"{times_str[3]:>8} {times_str[4]:>8} {power_info['total']*1000:>8.0f}mW")
    
    # 2. Comparison with expected values
    print("\n" + "="*70)
    print("2. MODEL VALIDATION (Predicted vs Expected)")
    print("="*70)
    
    comparison = {}
    print(f"\n{'Scenario':<25} {'Predicted':>12} {'Expected':>15} {'Error':>10} {'Status':>10}")
    print("-"*75)
    
    for scenario in SCENARIOS:
        predicted = results[scenario][1.0]['t_discharge']
        expected = EXPECTED_BATTERY_LIFE[scenario]
        expected_mid = (expected[0] + expected[1]) / 2
        error = (predicted - expected_mid) / expected_mid * 100
        in_range = expected[0] <= predicted <= expected[1]
        
        status = "✓ Good" if in_range else ("↑ High" if predicted > expected[1] else "↓ Low")
        
        comparison[scenario] = {
            'predicted': predicted,
            'expected': expected,
            'error': error,
            'in_range': in_range
        }
        
        print(f"{scenario:<25} {predicted:>10.1f}h   {expected[0]:.1f}-{expected[1]:.1f}h   "
              f"{error:>+8.1f}%   {status:>10}")
    
    # 3. Uncertainty quantification
    print("\n" + "="*70)
    print("3. UNCERTAINTY QUANTIFICATION (Monte Carlo, n=100)")
    print("="*70)
    
    uncertainty = {}
    print(f"\n{'Scenario':<25} {'Mean':>10} {'Std':>10} {'CV%':>8} {'95% CI':>20}")
    print("-"*75)
    
    for scenario in SCENARIOS:
        uc = monte_carlo_analysis(model, scenario, n_samples=100)
        uncertainty[scenario] = uc
        print(f"{scenario:<25} {uc['mean']:>8.1f}h  {uc['std']:>8.2f}h  {uc['cv']:>6.1f}%  "
              f"[{uc['ci_95'][0]:.1f}, {uc['ci_95'][1]:.1f}]h")
    
    # 4. Sensitivity analysis
    print("\n" + "="*70)
    print("4. SENSITIVITY ANALYSIS")
    print("="*70)
    
    sensitivity = sensitivity_analysis(model)
    
    sorted_factors = sorted(sensitivity.items(), key=lambda x: x[1]['sensitivity'], reverse=True)
    
    print(f"\n{'Factor':<25} {'Sensitivity':>12} {'Min Life':>10} {'Max Life':>10} {'Impact':>12}")
    print("-"*70)
    
    for name, data in sorted_factors:
        impact = (data['range'][1] - data['range'][0]) / data['range'][1] * 100
        print(f"{name:<25} {data['sensitivity']:>10.3f}   {data['range'][0]:>8.1f}h  "
              f"{data['range'][1]:>8.1f}h  {impact:>10.0f}%")
    
    # 5. Power breakdown analysis
    print("\n" + "="*70)
    print("5. POWER BREAKDOWN BY SCENARIO")
    print("="*70)
    
    for scenario in SCENARIOS:
        power_info = get_scenario_power(scenario)
        print(f"\n{scenario}:")
        print(f"  Total Power: {power_info['total']*1000:.0f} mW")
        
        # Sort by contribution
        sorted_breakdown = sorted(power_info['breakdown'].items(), key=lambda x: x[1], reverse=True)
        top_consumer = sorted_breakdown[0][0]
        
        print(f"  Primary Drain: {top_consumer} ({sorted_breakdown[0][1]/power_info['total']*100:.0f}%)")
        print("  Breakdown:")
        for comp, power in sorted_breakdown:
            pct = power / power_info['total'] * 100
            if pct >= 1:
                print(f"    - {comp}: {power*1000:.0f} mW ({pct:.1f}%)")
    
    # 6. Key findings
    print("\n" + "="*70)
    print("6. KEY FINDINGS")
    print("="*70)
    
    print("\n■ HIGH IMPACT FACTORS (Most significant drain causes):")
    for name, data in sorted_factors[:3]:
        print(f"  • {name}")
        print(f"    - Can reduce battery life by up to {(1-data['range'][0]/data['range'][1])*100:.0f}%")
        print(f"    - Battery life range: {data['range'][0]:.1f}h - {data['range'][1]:.1f}h")
    
    print("\n■ LOW IMPACT FACTORS (Surprisingly small effect):")
    for name, data in sorted_factors[-2:]:
        print(f"  • {name}")
        print(f"    - Only {data['sensitivity']*100:.0f}% sensitivity")
        print(f"    - Battery life variation: {abs(data['range'][1]-data['range'][0]):.1f}h")
    
    print("\n■ ACTIVITIES WITH HIGHEST BATTERY DRAIN:")
    drain_ranking = sorted([(s, get_scenario_power(s)['total']) for s in SCENARIOS], 
                          key=lambda x: x[1], reverse=True)
    for i, (scenario, power) in enumerate(drain_ranking[:3], 1):
        life = results[scenario][1.0]['t_discharge']
        print(f"  {i}. {scenario}: {power*1000:.0f} mW → {life:.1f}h battery life")
    
    print("\n■ MODEL ACCURACY:")
    good_predictions = sum(1 for c in comparison.values() if c['in_range'])
    avg_error = np.mean([abs(c['error']) for c in comparison.values()])
    print(f"  - Predictions within expected range: {good_predictions}/{len(comparison)}")
    print(f"  - Average absolute error: {avg_error:.1f}%")
    
    print("\n■ SPECIFIC DRAIN CAUSES:")
    for scenario in SCENARIOS:
        power_info = get_scenario_power(scenario)
        sorted_breakdown = sorted(power_info['breakdown'].items(), key=lambda x: x[1], reverse=True)
        top = sorted_breakdown[0]
        pct = top[1] / power_info['total'] * 100
        print(f"  - {scenario}: {top[0]} ({pct:.0f}% of total)")
    
    return results, comparison, uncertainty, sensitivity


def plot_results(results, comparison, uncertainty, sensitivity):
    """Generate comprehensive visualization."""
    import os
    os.makedirs('output/results', exist_ok=True)
    
    fig = plt.figure(figsize=(16, 14))
    gs = GridSpec(4, 3, figure=fig, hspace=0.35, wspace=0.3)
    
    scenarios = SCENARIOS
    
    # (a) Predicted vs Expected battery life
    ax1 = fig.add_subplot(gs[0, 0])
    x = np.arange(len(scenarios))
    predicted = [results[s][1.0]['t_discharge'] for s in scenarios]
    expected_mid = [(EXPECTED_BATTERY_LIFE[s][0]+EXPECTED_BATTERY_LIFE[s][1])/2 for s in scenarios]
    expected_err = [(EXPECTED_BATTERY_LIFE[s][1]-EXPECTED_BATTERY_LIFE[s][0])/2 for s in scenarios]
    
    ax1.bar(x - 0.2, predicted, 0.4, label='Predicted', color='steelblue', alpha=0.8)
    ax1.errorbar(x + 0.2, expected_mid, yerr=expected_err, fmt='o', color='darkred', 
                 capsize=5, label='Expected', markersize=8)
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax1.set_ylabel('Battery Life [hours]')
    ax1.set_title('(a) Predicted vs Expected Battery Life')
    ax1.legend()
    ax1.grid(True, alpha=0.3, axis='y')
    
    # (b) Discharge curves comparison
    ax2 = fig.add_subplot(gs[0, 1])
    colors = plt.cm.tab10(np.linspace(0, 1, len(scenarios)))
    for i, scenario in enumerate(scenarios):
        traj = results[scenario][1.0]['trajectory']
        if len(traj['t']) > 0:
            ax2.plot(traj['t'], traj['soc'], color=colors[i], linewidth=2, 
                     label=scenario.split()[0])
    ax2.set_xlabel('Time [hours]')
    ax2.set_ylabel('SOC [%]')
    ax2.set_title('(b) Discharge Curves (SOC=100%)')
    ax2.legend(loc='upper right', fontsize=7)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 100])
    
    # (c) Uncertainty analysis
    ax3 = fig.add_subplot(gs[0, 2])
    means = [uncertainty[s]['mean'] for s in scenarios]
    stds = [uncertainty[s]['std'] for s in scenarios]
    ax3.bar(x, means, yerr=stds, capsize=4, color='teal', alpha=0.8)
    ax3.set_xticks(x)
    ax3.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax3.set_ylabel('Battery Life [hours]')
    ax3.set_title('(c) Uncertainty (Mean ± σ)')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # (d) Power consumption by scenario
    ax4 = fig.add_subplot(gs[1, 0])
    powers = [get_scenario_power(s)['total'] * 1000 for s in scenarios]
    colors_power = plt.cm.Reds(np.linspace(0.3, 1, len(scenarios)))
    ax4.bar(x, powers, color=colors_power, alpha=0.8)
    ax4.set_xticks(x)
    ax4.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax4.set_ylabel('Power [mW]')
    ax4.set_title('(d) Average Power Consumption')
    ax4.grid(True, alpha=0.3, axis='y')
    
    # (e) Sensitivity ranking
    ax5 = fig.add_subplot(gs[1, 1])
    sorted_sens = sorted(sensitivity.items(), key=lambda x: x[1]['sensitivity'], reverse=True)
    names = [f[0][:18] for f in sorted_sens]
    sens_vals = [f[1]['sensitivity'] for f in sorted_sens]
    colors_sens = ['darkred' if s > 0.4 else 'orange' if s > 0.2 else 'green' for s in sens_vals]
    ax5.barh(range(len(names)), sens_vals, color=colors_sens, alpha=0.8)
    ax5.set_yticks(range(len(names)))
    ax5.set_yticklabels(names, fontsize=8)
    ax5.set_xlabel('Sensitivity Index')
    ax5.set_title('(e) Factor Sensitivity Ranking')
    ax5.invert_yaxis()
    ax5.grid(True, alpha=0.3, axis='x')
    
    # (f) Prediction error
    ax6 = fig.add_subplot(gs[1, 2])
    errors = [comparison[s]['error'] for s in scenarios]
    in_range = [comparison[s]['in_range'] for s in scenarios]
    colors_err = ['green' if ir else 'red' for ir in in_range]
    ax6.bar(x, errors, color=colors_err, alpha=0.8)
    ax6.axhline(y=0, color='black', linewidth=1)
    ax6.axhline(y=20, color='orange', linestyle='--', alpha=0.5)
    ax6.axhline(y=-20, color='orange', linestyle='--', alpha=0.5)
    ax6.set_xticks(x)
    ax6.set_xticklabels([s.split()[0] for s in scenarios], rotation=45, ha='right', fontsize=8)
    ax6.set_ylabel('Error [%]')
    ax6.set_title('(f) Prediction Error')
    ax6.grid(True, alpha=0.3, axis='y')
    
    # (g-i) Power breakdown pie charts
    pie_scenarios = ['Sleep (Screen Off)', 'Office Work', 'Heavy Gaming']
    for i, scenario in enumerate(pie_scenarios):
        ax = fig.add_subplot(gs[2, i])
        power_info = get_scenario_power(scenario)
        components = list(power_info['breakdown'].keys())
        powers_pie = [power_info['breakdown'][c] * 1000 for c in components]
        
        colors_pie = plt.cm.Set3(np.linspace(0, 1, len(components)))
        wedges, texts, autotexts = ax.pie(powers_pie, labels=components, autopct='%1.0f%%',
                                           colors=colors_pie, textprops={'fontsize': 7})
        ax.set_title(f'({chr(103+i)}) {scenario.split()[0]}\n({sum(powers_pie):.0f} mW)', fontsize=9)
    
    # (j) Initial SOC effect
    ax10 = fig.add_subplot(gs[3, 0])
    for scenario in ['Sleep (Screen Off)', 'Office Work', 'Heavy Gaming']:
        socs = [1.0, 0.8, 0.6, 0.4, 0.2]
        times = [results[scenario][soc]['t_discharge'] for soc in socs]
        ax10.plot([s*100 for s in socs], times, 'o-', linewidth=2, markersize=6, 
                  label=scenario.split()[0])
    ax10.set_xlabel('Initial SOC [%]')
    ax10.set_ylabel('Battery Life [hours]')
    ax10.set_title('(j) Effect of Initial SOC')
    ax10.legend(fontsize=8)
    ax10.grid(True, alpha=0.3)
    
    # (k) Temperature profile during discharge
    ax11 = fig.add_subplot(gs[3, 1])
    for scenario in ['Office Work', 'Heavy Gaming']:
        traj = results[scenario][1.0]['trajectory']
        if len(traj['t']) > 0:
            ax11.plot(traj['t'], traj['temperature'], linewidth=2, label=scenario.split()[0])
    ax11.axhline(y=25, color='gray', linestyle='--', alpha=0.5, label='Ambient')
    ax11.set_xlabel('Time [hours]')
    ax11.set_ylabel('Temperature [°C]')
    ax11.set_title('(k) Battery Temperature During Discharge')
    ax11.legend(fontsize=8)
    ax11.grid(True, alpha=0.3)
    
    # (l) Voltage profiles
    ax12 = fig.add_subplot(gs[3, 2])
    for scenario in ['Sleep (Screen Off)', 'Office Work', 'Heavy Gaming']:
        traj = results[scenario][1.0]['trajectory']
        if len(traj['t']) > 0:
            ax12.plot(traj['t'], traj['voltage'], linewidth=2, label=scenario.split()[0])
    ax12.axhline(y=3.0, color='red', linestyle='--', alpha=0.5, label='Cutoff')
    ax12.set_xlabel('Time [hours]')
    ax12.set_ylabel('Voltage [V]')
    ax12.set_title('(l) Terminal Voltage During Discharge')
    ax12.legend(fontsize=8)
    ax12.grid(True, alpha=0.3)
    
    plt.suptitle('Comprehensive Discharge Time Prediction Analysis', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.savefig('output/results/discharge_analysis_comprehensive.png', dpi=300, bbox_inches='tight')
    print("\nFigure saved: output/results/discharge_analysis_comprehensive.png")
    
    return fig


if __name__ == "__main__":
    results, comparison, uncertainty, sensitivity = run_comprehensive_analysis()
    fig = plot_results(results, comparison, uncertainty, sensitivity)
    plt.close(fig)
