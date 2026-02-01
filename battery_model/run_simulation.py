#!/usr/bin/env python3
"""
Smartphone Battery Discharge Simulation
========================================
Main simulation script for the coupled electro-thermal-aging model.

This script performs:
1. Static model parameter visualization
2. Dynamic simulation under various scenarios
3. Extreme condition validation
4. Multi-scenario comparison
5. Remaining battery life prediction

Usage:
    python run_simulation.py [--quick] [--scenario <name>] [--output <dir>]

Author: MCM 2026 Team
Date: February 2026
"""

import argparse
import os
import sys
import time
import numpy as np
import warnings

# Suppress integration warnings for cleaner output
warnings.filterwarnings('ignore', category=RuntimeWarning)

# Import model components
from battery_core import create_battery_model, BatteryCoreModel
from load_models import SmartphoneLoadManager, UsageScenarios
from coupled_system import (
    CoupledSmartphoneModel, 
    create_smartphone_model, 
    quick_discharge_simulation,
    SimulationResults
)
from visualization import (
    plot_aging_models,
    plot_ocv_model,
    plot_temperature_corrections,
    plot_simulation_results,
    plot_extreme_condition_validation,
    plot_thermal_gradient_fem,
    plot_scenario_comparison,
    plot_rc_circuit_diagram,
    generate_all_figures
)

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt


def run_single_scenario_simulation(scenario_name, cycle_number=100, 
                                   initial_soc=1.0, T_env=25.0,
                                   duration_hours=24.0, output_dir='results'):
    """
    Run simulation for a single usage scenario.
    
    Parameters:
        scenario_name: Name of usage scenario
        cycle_number: Battery cycle count
        initial_soc: Initial state of charge
        T_env: Environment temperature [°C]
        duration_hours: Maximum simulation duration [hours]
        output_dir: Output directory
    """
    print(f"\n{'='*60}")
    print(f"Simulating: {scenario_name}")
    print(f"{'='*60}")
    print(f"  Cycle count: {cycle_number}")
    print(f"  Initial SOC: {initial_soc*100:.0f}%")
    print(f"  Environment: {T_env}°C")
    
    start_time = time.time()
    
    # Create and run simulation
    model = create_smartphone_model(
        cycle_number=cycle_number,
        initial_soc=initial_soc,
        environment_temp=T_env,
        scenario=scenario_name
    )
    
    results = model.simulate(
        duration=duration_hours * 3600,
        time_step=30.0  # 30 second max step
    )
    
    elapsed = time.time() - start_time
    print(f"  Computation time: {elapsed:.1f} s")
    
    # Print summary
    results.summary()
    
    # Save results
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot and save
    fig = plot_simulation_results(results, 
        f'{output_dir}/sim_{scenario_name.replace(" ", "_")}.png')
    plt.close(fig)
    
    return results


def run_extreme_condition_test(output_dir='results'):
    """
    Run extreme condition validation test.
    
    Test conditions:
    - Low temperature: -10°C
    - Aged battery: N=300 cycles
    - High load: Gaming scenario
    """
    print(f"\n{'='*60}")
    print("EXTREME CONDITION VALIDATION")
    print(f"{'='*60}")
    print("  Temperature: -10°C")
    print("  Battery age: 300 cycles")
    print("  Load: High (Gaming)")
    
    # Create model with extreme conditions
    model = create_smartphone_model(
        cycle_number=300,
        initial_soc=0.8,
        environment_temp=-10.0,
        scenario='gaming'
    )
    
    # Reduce CPU load slightly for cold conditions
    model.profile['cpu_load'] = 0.7
    
    results = model.simulate(
        duration=1800,  # 30 minutes
        time_step=5.0
    )
    
    results.summary()
    
    # Save validation plot
    os.makedirs(output_dir, exist_ok=True)
    fig = plot_extreme_condition_validation(results, 
        f'{output_dir}/extreme_validation.png')
    plt.close(fig)
    
    return results


def run_multi_scenario_comparison(output_dir='results'):
    """
    Run and compare multiple usage scenarios.
    """
    print(f"\n{'='*60}")
    print("MULTI-SCENARIO COMPARISON")
    print(f"{'='*60}")
    
    scenarios = {
        'Idle (Off)': 'idle_screen_off',
        'Idle (On)': 'idle_screen_on',
        'Video': 'video_streaming',
        'Gaming': 'gaming',
        'Navigation': 'navigation',
        'Voice Call': 'voice_call',
    }
    
    results_dict = {}
    
    for name, scenario_key in scenarios.items():
        print(f"\nRunning: {name}")
        
        model = create_smartphone_model(
            cycle_number=100,
            initial_soc=1.0,
            environment_temp=25.0,
            scenario=scenario_key
        )
        
        # Limit simulation time based on expected battery life
        max_hours = 48.0 if 'idle' in scenario_key.lower() else 12.0
        
        results = model.simulate(
            duration=max_hours * 3600,
            time_step=60.0  # Coarser step for faster comparison
        )
        
        results_dict[name] = results
        print(f"  Battery life: {results.discharge_time_hours:.2f} hours")
    
    # Save comparison plot
    os.makedirs(output_dir, exist_ok=True)
    fig = plot_scenario_comparison(results_dict, 
        f'{output_dir}/scenario_comparison.png')
    plt.close(fig)
    
    # Print summary table
    print(f"\n{'='*60}")
    print("BATTERY LIFE COMPARISON SUMMARY")
    print(f"{'='*60}")
    print(f"{'Scenario':<20} {'Battery Life':>15} {'Avg Power':>15}")
    print("-" * 60)
    for name, result in results_dict.items():
        print(f"{name:<20} {result.discharge_time_hours:>12.2f} hr {result.average_power()*1000:>12.1f} mW")
    
    return results_dict


def run_temperature_sweep(output_dir='results'):
    """
    Run simulations across different temperatures.
    """
    print(f"\n{'='*60}")
    print("TEMPERATURE SENSITIVITY ANALYSIS")
    print(f"{'='*60}")
    
    temperatures = [-10, 0, 10, 25, 40]
    results_by_temp = {}
    
    for T in temperatures:
        print(f"\nRunning at T = {T}°C")
        
        model = create_smartphone_model(
            cycle_number=100,
            initial_soc=1.0,
            environment_temp=T,
            scenario='idle_screen_on'
        )
        
        results = model.simulate(
            duration=12 * 3600,
            time_step=60.0
        )
        
        results_by_temp[f"{T}°C"] = results
        print(f"  Battery life: {results.discharge_time_hours:.2f} hours")
    
    # Create comparison figure
    os.makedirs(output_dir, exist_ok=True)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    colors = plt.cm.coolwarm(np.linspace(0, 1, len(temperatures)))
    
    for i, (label, result) in enumerate(results_by_temp.items()):
        t_hours = result.t / 3600
        axes[0].plot(t_hours, result.soc * 100, color=colors[i], 
                     linewidth=2, label=label)
        axes[1].plot(t_hours, result.V_term, color=colors[i], 
                     linewidth=2, label=label)
    
    axes[0].set_xlabel('Time [hours]')
    axes[0].set_ylabel('State of Charge [%]')
    axes[0].set_title('(a) SOC vs Temperature')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].set_xlabel('Time [hours]')
    axes[1].set_ylabel('Terminal Voltage [V]')
    axes[1].set_title('(b) Voltage vs Temperature')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.suptitle('Temperature Sensitivity Analysis', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/temperature_sweep.png', dpi=300, bbox_inches='tight')
    plt.close(fig)
    
    return results_by_temp


def run_aging_sweep(output_dir='results'):
    """
    Run simulations across different battery ages.
    """
    print(f"\n{'='*60}")
    print("BATTERY AGING SENSITIVITY ANALYSIS")
    print(f"{'='*60}")
    
    cycles = [0, 100, 200, 300, 400]
    results_by_age = {}
    
    for N in cycles:
        print(f"\nRunning at N = {N} cycles")
        
        model = create_smartphone_model(
            cycle_number=N,
            initial_soc=1.0,
            environment_temp=25.0,
            scenario='video_streaming'
        )
        
        results = model.simulate(
            duration=12 * 3600,
            time_step=60.0
        )
        
        results_by_age[f"N={N}"] = results
        print(f"  Battery life: {results.discharge_time_hours:.2f} hours")
    
    # Create comparison figure
    os.makedirs(output_dir, exist_ok=True)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    colors = plt.cm.Reds(np.linspace(0.3, 1, len(cycles)))
    
    for i, (label, result) in enumerate(results_by_age.items()):
        t_hours = result.t / 3600
        axes[0].plot(t_hours, result.soc * 100, color=colors[i], 
                     linewidth=2, label=label)
        axes[1].plot(t_hours, result.V_term, color=colors[i], 
                     linewidth=2, label=label)
    
    axes[0].set_xlabel('Time [hours]')
    axes[0].set_ylabel('State of Charge [%]')
    axes[0].set_title('(a) SOC vs Battery Age')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].set_xlabel('Time [hours]')
    axes[1].set_ylabel('Terminal Voltage [V]')
    axes[1].set_title('(b) Voltage vs Battery Age')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.suptitle('Battery Aging Impact Analysis', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/aging_sweep.png', dpi=300, bbox_inches='tight')
    plt.close(fig)
    
    return results_by_age


def generate_model_figures(output_dir='figures'):
    """Generate all model figures."""
    print(f"\n{'='*60}")
    print("GENERATING MODEL FIGURES")
    print(f"{'='*60}")
    
    os.makedirs(output_dir, exist_ok=True)
    
    print("  [1/5] Aging models...")
    fig = plot_aging_models(f'{output_dir}/fig_aging_models.png')
    plt.close(fig)
    
    print("  [2/5] OCV model...")
    fig = plot_ocv_model(f'{output_dir}/fig_ocv_model.png')
    plt.close(fig)
    
    print("  [3/5] Temperature corrections...")
    fig = plot_temperature_corrections(f'{output_dir}/fig_temp_corrections.png')
    plt.close(fig)
    
    print("  [4/5] RC circuit diagram...")
    fig = plot_rc_circuit_diagram(f'{output_dir}/fig_rc_circuit.png')
    plt.close(fig)
    
    print("  [5/5] Thermal FEM concept...")
    fig = plot_thermal_gradient_fem(f'{output_dir}/fig_thermal_fem.png')
    plt.close(fig)
    
    print("\nModel figures generated successfully!")


def print_model_equations():
    """Print the key model equations in the system."""
    print("""
================================================================================
COUPLED ELECTRO-THERMAL-AGING MODEL - KEY EQUATIONS
================================================================================

1. STATE OF CHARGE DYNAMICS (Coulomb Counting)
   ─────────────────────────────────────────────
   dz/dt = -I(t) · η(T_c) / (Q_max(N, T_c) · 3600)

2. CAPACITY FADE MODEL (Double-Exponential)
   ─────────────────────────────────────────────
   Q_max(N) = a_Q · exp(-b_Q · N) + c_Q · exp(-d_Q · N)
   
   With temperature correction:
   Q_max(N, T) = Q_max(N) · S_Q(T)
   S_Q(T) = S_Q_max / (1 + exp(-k_Q · (T - T_0)))

3. IMPEDANCE GROWTH MODEL (Power-Law)
   ─────────────────────────────────────────────
   R_total(N) = a_R · N^b_R + c_R
   
   With temperature correction:
   R_total(N, T) = R_total(N) · S_R(T)
   S_R(T) = C_R + A_R · exp(-B_R · T)

4. OPEN CIRCUIT VOLTAGE (Nernst-based Combined Model)
   ─────────────────────────────────────────────
   V_OCV(z) = K_0 + K_1·z + K_2/z + K_3·ln(z) + K_4·ln(1-z)

5. 2ND-ORDER RC CIRCUIT DYNAMICS
   ─────────────────────────────────────────────
   dV_1/dt = -V_1/(R_1·C_1) + I(t)/C_1
   dV_2/dt = -V_2/(R_2·C_2) + I(t)/C_2
   V_term(t) = V_OCV(z) - V_1(t) - V_2(t) - I(t)·R_0

6. TWO-STATE THERMAL MODEL
   ─────────────────────────────────────────────
   Heat generation (Bernardi equation):
   Q_gen = I²·R_total + I·T_c·(∂V_OCV/∂T)
   
   Core temperature dynamics:
   C_c · dT_c/dt = Q_gen - (T_c - T_s)/R_cs
   
   Surface temperature dynamics:
   C_s · dT_s/dt = (T_c - T_s)/R_cs - (T_s - T_env)/R_se

7. LOAD POWER MODELS
   ─────────────────────────────────────────────
   5G Communication:
   P_5G(t) = P_static + α_bb·R(t) + Λ_env·d(t)^n·(2^(R(t)/B) - 1)/η_PA
   
   Bluetooth/BLE:
   I_BLE(τ) ≈ I_sleep + Q_event(L)/τ
   
   GNSS (Environment-aware):
   P_GNSS(t) = x_lock·P_track + (1 - x_lock)·P_acq + P_LNA
   dx_lock/dt = (Ψ(S_env) - x_lock)/τ_react
   
   Display (OLED with LTPO):
   P_disp(t) = P_base + k_drv·f_refresh(t) + β_panel·Θ(L_set)·APL(t)
   
   SoC (DVFS with thermal feedback):
   P_SoC(t) = κ_dvfs·f(t)³ + V_dd·I_leak(T_soc)

8. COUPLED SYSTEM OUTPUT
   ─────────────────────────────────────────────
   Total load current:
   I(t) = P_total(t) / V_term(t)
   
   where:
   P_total = P_5G + P_BT + P_bg + P_GNSS + P_disp + P_SoC

================================================================================
""")


def main():
    """Main entry point for simulations."""
    parser = argparse.ArgumentParser(
        description='Smartphone Battery Discharge Simulation'
    )
    parser.add_argument('--quick', action='store_true',
                        help='Run quick test simulation only')
    parser.add_argument('--scenario', type=str, default=None,
                        help='Specific scenario to simulate')
    parser.add_argument('--output', type=str, default='output',
                        help='Output directory')
    parser.add_argument('--equations', action='store_true',
                        help='Print model equations')
    parser.add_argument('--figures-only', action='store_true',
                        help='Generate static figures only')
    
    args = parser.parse_args()
    
    print("""
================================================================================
    SMARTPHONE BATTERY COUPLED ELECTRO-THERMAL-AGING MODEL
                         MCM 2026 Problem A
================================================================================
    """)
    
    if args.equations:
        print_model_equations()
        return
    
    # Create output directories
    os.makedirs(args.output, exist_ok=True)
    figures_dir = os.path.join(args.output, 'figures')
    results_dir = os.path.join(args.output, 'results')
    
    # Generate static model figures
    generate_model_figures(figures_dir)
    
    if args.figures_only:
        print("\nStatic figures generated. Exiting.")
        return
    
    if args.quick:
        # Quick test simulation
        print("\nRunning quick test simulation...")
        results = run_single_scenario_simulation(
            'idle_screen_on',
            cycle_number=100,
            initial_soc=0.8,
            duration_hours=2.0,
            output_dir=results_dir
        )
        return
    
    if args.scenario:
        # Single scenario simulation
        results = run_single_scenario_simulation(
            args.scenario,
            output_dir=results_dir
        )
        return
    
    # Full simulation suite
    print("\n" + "="*60)
    print("RUNNING FULL SIMULATION SUITE")
    print("="*60)
    
    # 1. Extreme condition validation
    extreme_results = run_extreme_condition_test(results_dir)
    
    # 2. Multi-scenario comparison
    scenario_results = run_multi_scenario_comparison(results_dir)
    
    # 3. Temperature sensitivity
    temp_results = run_temperature_sweep(results_dir)
    
    # 4. Aging sensitivity
    aging_results = run_aging_sweep(results_dir)
    
    # Summary
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print(f"\nAll results saved to: {args.output}/")
    print(f"  - Figures: {figures_dir}/")
    print(f"  - Results: {results_dir}/")
    
    # Print key findings
    print("\n" + "="*60)
    print("KEY FINDINGS")
    print("="*60)
    
    best_scenario = max(scenario_results.items(), 
                        key=lambda x: x[1].discharge_time_hours)
    worst_scenario = min(scenario_results.items(), 
                         key=lambda x: x[1].discharge_time_hours)
    
    print(f"\n1. Battery Life Range:")
    print(f"   Best:  {best_scenario[0]} - {best_scenario[1].discharge_time_hours:.1f} hours")
    print(f"   Worst: {worst_scenario[0]} - {worst_scenario[1].discharge_time_hours:.1f} hours")
    
    print(f"\n2. Temperature Impact (at T=-10°C vs T=25°C):")
    if '-10°C' in temp_results and '25°C' in temp_results:
        cold_life = temp_results['-10°C'].discharge_time_hours
        warm_life = temp_results['25°C'].discharge_time_hours
        reduction = (1 - cold_life/warm_life) * 100
        print(f"   Cold weather reduces battery life by {reduction:.1f}%")
    
    print(f"\n3. Aging Impact (N=0 vs N=400 cycles):")
    if 'N=0' in aging_results and 'N=400' in aging_results:
        new_life = aging_results['N=0'].discharge_time_hours
        old_life = aging_results['N=400'].discharge_time_hours
        reduction = (1 - old_life/new_life) * 100
        print(f"   Aging reduces battery life by {reduction:.1f}%")


if __name__ == "__main__":
    main()
