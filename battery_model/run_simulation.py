#!/usr/bin/env python3
"""
Main Simulation Runner for Smartphone Battery Model
===================================================

This script runs the complete battery discharge simulation with all
subsystem models and generates publication-quality visualizations.

Usage:
    python run_simulation.py [--quick] [--figures-only] [--no-figures]
    
Options:
    --quick: Run shorter simulations for testing
    --figures-only: Only generate figures without full simulation
    --no-figures: Run simulation without generating figures
"""

import sys
import os
import argparse
import time
import numpy as np

# Add source directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from battery_core import BatteryCore, BatteryParameters
from network_5g_module import Network5GModule, analyze_5g_power_sensitivity
from gnss_module import GNSSModule, analyze_gnss_state_dynamics
from background_tasks_module import BackgroundTasksModule, analyze_background_statistics
from bluetooth_module import BluetoothModule, analyze_bluetooth_power_breakdown
from coupled_system import (
    CoupledBatterySystem, 
    IdleScenario, VideoStreamingScenario, NavigationScenario,
    GamingScenario, MixedUsageScenario
)


def print_header(title: str):
    """Print formatted section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60 + "\n")


def test_individual_modules():
    """Test each module independently."""
    print_header("Individual Module Testing")
    
    # Battery Core
    print("1. Battery Core Model")
    battery = BatteryCore()
    print(f"   Nominal capacity: {battery.params.C_nom} Ah")
    print(f"   Effective capacity at 1A, 25°C: {battery.effective_capacity(1.0, 298.15):.3f} Ah")
    print(f"   OCV at 50% SOC: {battery.open_circuit_voltage(0.5):.3f} V")
    
    # 5G Network
    print("\n2. 5G Network Module")
    network = Network5GModule()
    scenarios = [
        ("Indoor (100m, 20 Mbps)", 20e6, 100),
        ("Outdoor (300m, 50 Mbps)", 50e6, 300),
        ("Cell edge (800m, 10 Mbps)", 10e6, 800)
    ]
    for name, rate, dist in scenarios:
        power = network.total_power(rate, dist) * 1000
        print(f"   {name}: {power:.1f} mW")
    
    # GNSS
    print("\n3. GNSS Module")
    gnss = GNSSModule()
    snr_levels = [45, 35, 25, 15]
    for snr in snr_levels:
        power = gnss.power_consumption(snr, requested=True) * 1000
        print(f"   SNR {snr} dB-Hz: {power:.1f} mW")
    
    # Background Tasks
    print("\n4. Background Tasks Module")
    bg = BackgroundTasksModule()
    t_test = np.linspace(0, 0.5, 100)
    data = bg.generate_sample_path(t_test, seed=42)
    I_mean = np.mean(data['I_total']) * 1000
    I_max = np.max(data['I_total']) * 1000
    print(f"   Mean current (30 min): {I_mean:.1f} mA")
    print(f"   Peak current (30 min): {I_max:.1f} mA")
    print(f"   Burst events: {len(data['burst_times'])}")
    
    # Bluetooth
    print("\n5. Bluetooth Module")
    bt = BluetoothModule()
    bt_scenarios = [
        ("TWS Audio (AAC)", {'audio_streaming': True, 'audio_codec': 'aac', 'n_audio_devices': 2}),
        ("Smartwatch BLE", {'ble_connections': [{'interval_ms': 500}]}),
        ("Idle", {'bt_classic_active': False})
    ]
    for name, config in bt_scenarios:
        power = bt.total_power(**config) * 1000
        print(f"   {name}: {power:.1f} mW")


def run_scenario_simulations(quick: bool = False):
    """Run full scenario simulations."""
    print_header("Full Scenario Simulations")
    
    system = CoupledBatterySystem()
    
    scenarios = {
        'Idle (Screen Off)': IdleScenario(),
        'Video Streaming (4K)': VideoStreamingScenario(),
        'GPS Navigation': NavigationScenario(),
        'Mobile Gaming': GamingScenario(),
        'Mixed Daily Use': MixedUsageScenario()
    }
    
    t_max = 6 if quick else 24
    dt = 0.01 if quick else 0.005
    
    results = {}
    
    for name, scenario in scenarios.items():
        print(f"Simulating: {name}...", end=" ", flush=True)
        start_time = time.time()
        
        result = system.simulate(
            scenario,
            S0=1.0,
            t_span=(0, t_max),
            dt=dt
        )
        
        elapsed = time.time() - start_time
        results[name] = result
        
        life = result['time'][-1]
        final_soc = result['SOC'][-1] * 100
        avg_power = np.mean(result['power']) * 1000
        
        print(f"Done ({elapsed:.1f}s)")
        print(f"   Battery life: {life:.2f} hours")
        print(f"   Final SOC: {final_soc:.1f}%")
        print(f"   Average power: {avg_power:.0f} mW")
    
    return results


def generate_figures(results: dict = None, output_dir: str = None):
    """Generate all visualization figures."""
    print_header("Generating Visualizations")
    
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(__file__), 'visualizations')
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Import visualization module
    from visualizations import (
        figure_5g_power_analysis,
        figure_gnss_state_dynamics,
        figure_background_stochastic,
        figure_bluetooth_analysis,
        figure_coupled_system_simulation,
        figure_model_validation
    )
    
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    
    figures = []
    
    print("Generating Figure 1: 5G Power Analysis...")
    fig1 = figure_5g_power_analysis(f'{output_dir}/fig1_5g_power_analysis.png')
    figures.append(('5G Power', fig1))
    plt.close(fig1)
    
    print("Generating Figure 2: GNSS State Dynamics...")
    fig2 = figure_gnss_state_dynamics(f'{output_dir}/fig2_gnss_state_dynamics.png')
    figures.append(('GNSS', fig2))
    plt.close(fig2)
    
    print("Generating Figure 3: Background Stochastic Process...")
    fig3 = figure_background_stochastic(f'{output_dir}/fig3_background_stochastic.png')
    figures.append(('Background', fig3))
    plt.close(fig3)
    
    print("Generating Figure 4: Bluetooth Analysis...")
    fig4 = figure_bluetooth_analysis(f'{output_dir}/fig4_bluetooth_analysis.png')
    figures.append(('Bluetooth', fig4))
    plt.close(fig4)
    
    print("Generating Figure 5: Model Validation...")
    fig5 = figure_model_validation(f'{output_dir}/fig5_model_validation.png')
    figures.append(('Validation', fig5))
    plt.close(fig5)
    
    if results:
        print("Generating Figure 6: Coupled System Results...")
        fig6 = figure_coupled_system_simulation(results, f'{output_dir}/fig6_coupled_system.png')
        figures.append(('Coupled System', fig6))
        plt.close(fig6)
    
    print(f"\nAll figures saved to: {output_dir}")
    
    return figures


def print_model_summary():
    """Print summary of the mathematical model."""
    print_header("Model Mathematical Framework Summary")
    
    print("""
    CORE SOC DYNAMICS:
    ==================
    dS(t)/dt = -I_total(t) / (C_eff(I,T) * η_coulomb) - k_sd * S(t)
    
    Where:
      S(t)     : State of Charge (0-1)
      I_total  : Total load current (sum of all modules)
      C_eff    : Effective capacity (Peukert + temperature corrected)
      η_coulomb: Coulombic efficiency (~0.995)
      k_sd     : Self-discharge rate
    
    SUBSYSTEM MODELS:
    =================
    
    1. 5G Network (Link Budget Based):
       P_5G = P_base + k_b*R(t) + [λ*d(t)^n * (2^(R/B) - 1)] / η_PA
       
       Key insight: Power grows exponentially with distance at cell edge
    
    2. GNSS (Continuous State Machine):
       P_GNSS = Σ w_i(SNR) * P_i
       
       w_i = σ(k*(SNR - SNR_threshold))  (Sigmoid transitions)
       
       States: Sleep → Acquisition → Tracking
    
    3. Background Tasks (Ornstein-Uhlenbeck Process):
       dI(t) = -θ*(I(t) - μ)*dt + σ*dW_t + Burst(t)
       
       Captures mean-reverting behavior with periodic bursts
    
    4. Bluetooth:
       P_BT = P_classic + P_BLE(connection_interval, PHY)
       
       BLE power inversely proportional to connection interval
    
    COUPLED EFFECTS:
    ================
    - CPU-Network coupling: Processing overhead for data
    - CPU-GNSS coupling: Position calculation overhead
    - Thermal coupling: All modules contribute to heating
    
    dT/dt = (Q_gen - h*A*(T - T_ambient)) / (m*c_p)
    """)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Smartphone Battery Discharge Model Simulation'
    )
    parser.add_argument('--quick', action='store_true',
                       help='Run shorter simulations for testing')
    parser.add_argument('--figures-only', action='store_true',
                       help='Only generate figures')
    parser.add_argument('--no-figures', action='store_true',
                       help='Skip figure generation')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for figures')
    
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("  SMARTPHONE BATTERY DISCHARGE MODEL")
    print("  2026 MCM Problem A Solution")
    print("=" * 60)
    
    # Print model summary
    print_model_summary()
    
    if args.figures_only:
        generate_figures(output_dir=args.output_dir)
        return
    
    # Test individual modules
    test_individual_modules()
    
    # Run full simulations
    results = run_scenario_simulations(quick=args.quick)
    
    # Generate figures
    if not args.no_figures:
        generate_figures(results, output_dir=args.output_dir)
    
    # Final summary
    print_header("Simulation Complete")
    print("Results Summary:")
    print("-" * 50)
    print(f"{'Scenario':<25} {'Battery Life':>12} {'Avg Power':>12}")
    print("-" * 50)
    for name, result in results.items():
        life = result['time'][-1]
        avg_power = np.mean(result['power']) * 1000
        print(f"{name:<25} {life:>10.2f} h {avg_power:>10.0f} mW")
    print("-" * 50)
    
    print("\nModel implementation complete!")


if __name__ == "__main__":
    main()
