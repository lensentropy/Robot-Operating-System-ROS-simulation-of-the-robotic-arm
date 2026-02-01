"""
Practical User Recommendations and Extended Theoretical Analysis
================================================================

This module provides:
1. Actionable user recommendations based on model insights
2. Extended theoretical analysis with additional formulas
3. OS-level power management strategies
4. Battery aging mitigation techniques
5. Generalization to other portable devices

Author: Recommendation Framework
Date: 2026-02-01
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple
import json

from battery_electro_thermal_aging import BatteryElectroThermalAgingModel, BatteryParameters
from load_subsystems import (
    Model5G, ModelBluetooth, ModelBackground,
    ModelGNSS, ModelOLED, ModelSoC, IntegratedPowerModel
)


# =============================================================================
# Extended Theoretical Analysis
# =============================================================================

class ExtendedTheoreticalAnalysis:
    """
    Extended theoretical analysis providing deeper physical insights
    beyond the basic model equations.
    """
    
    def __init__(self):
        self.battery = BatteryElectroThermalAgingModel()
        self.integrated = IntegratedPowerModel()
        
    # -------------------------------------------------------------------------
    # 1. Battery State Estimation Theory
    # -------------------------------------------------------------------------
    
    def analyze_soc_observability(self) -> Dict:
        """
        Analyze State of Charge observability through OCV-SOC relationship.
        
        The Cramér-Rao Lower Bound (CRLB) for SOC estimation accuracy:
        
        σ²_SOC ≥ [I(SOC)]^{-1}
        
        where Fisher Information I(SOC) = (dV_OCV/dSOC)² / σ²_noise
        
        High |dV_OCV/dSOC| regions provide better observability.
        """
        SOC_range = np.linspace(0.05, 0.95, 100)
        
        # Calculate OCV sensitivity
        dV_dSOC = [self.battery.get_dOCV_dSOC(s) for s in SOC_range]
        
        # Assume measurement noise σ_noise = 5 mV
        sigma_noise = 0.005  # V
        
        # Fisher Information
        I_SOC = np.array(dV_dSOC) ** 2 / (sigma_noise ** 2)
        
        # CRLB (minimum estimation variance)
        CRLB = 1.0 / I_SOC
        
        # Estimation accuracy (1-sigma bound)
        sigma_SOC = np.sqrt(CRLB)
        
        # Find high/low observability regions
        low_obs_threshold = np.percentile(np.abs(dV_dSOC), 25)
        
        analysis = {
            'SOC_range': SOC_range.tolist(),
            'dV_dSOC': dV_dSOC,
            'Fisher_Information': I_SOC.tolist(),
            'CRLB': CRLB.tolist(),
            'estimation_accuracy_1sigma': sigma_SOC.tolist(),
            'best_observability_SOC': SOC_range[np.argmax(np.abs(dV_dSOC))],
            'worst_observability_SOC_range': (0.3, 0.7),
            'insight': (
                "SOC estimation is most accurate at extreme SOC levels (0-20%, 80-100%) "
                "where OCV changes rapidly. In the 30-70% plateau region, small voltage "
                "changes make accurate SOC estimation challenging without Coulomb counting."
            )
        }
        
        return analysis
    
    def derive_runtime_equation(self) -> Dict:
        """
        Derive the complete runtime prediction equation.
        
        Total Runtime:
        T_end = ∫_{SOC_0}^{SOC_min} Q_max(N,T) / I_total(t) dSOC
        
        For constant load:
        T_end = Q_max(N,T) × (SOC_0 - SOC_min) × 3600 / I_total
        
        With efficiency correction:
        T_end = Q_max(N,T) × (SOC_0 - SOC_min) × 3600 × η(T) / I_total
        
        Where I_total = P_total / (V_batt × η_PMIC)
        """
        # Reference conditions
        N = 0  # New battery
        T = 25  # Room temperature
        SOC_0 = 1.0
        SOC_min = 0.05  # Cutoff at 5%
        
        Q_max = self.battery.get_capacity(N, T)
        eta = self.battery.get_coulombic_efficiency(T)
        
        # Example load scenarios
        scenarios = {
            'light_use': {'P_total': 500, 'description': 'Reading/Messaging'},
            'medium_use': {'P_total': 1500, 'description': 'Web browsing'},
            'heavy_use': {'P_total': 3500, 'description': 'Gaming'},
            'extreme_use': {'P_total': 5000, 'description': 'Gaming + 5G + Navigation'}
        }
        
        V_batt_avg = 3.7  # V (average)
        eta_PMIC = 0.92
        
        runtimes = {}
        for name, scenario in scenarios.items():
            P_total = scenario['P_total'] / 1000  # W
            I_total = P_total / (V_batt_avg * eta_PMIC)
            
            # Runtime in hours
            T_end = Q_max * (SOC_0 - SOC_min) * eta / I_total
            
            runtimes[name] = {
                'P_total_mW': scenario['P_total'],
                'I_total_A': round(I_total, 3),
                'runtime_hours': round(T_end, 2),
                'description': scenario['description']
            }
        
        return {
            'formula': 'T_end = Q_max × (SOC_0 - SOC_min) × η / I_total',
            'parameters': {
                'Q_max_Ah': Q_max,
                'SOC_range': f'{SOC_0} to {SOC_min}',
                'coulombic_efficiency': eta,
                'V_batt_avg': V_batt_avg,
                'PMIC_efficiency': eta_PMIC
            },
            'runtime_estimates': runtimes,
            'insight': (
                "Runtime scales inversely with total power consumption. "
                "At 500mW (light use), a 2Ah battery can last ~14 hours, "
                "but at 5W (extreme use), this drops to ~1.4 hours - a 10× reduction."
            )
        }
    
    # -------------------------------------------------------------------------
    # 2. Thermal Management Theory
    # -------------------------------------------------------------------------
    
    def analyze_thermal_runaway_risk(self) -> Dict:
        """
        Analyze thermal runaway conditions based on energy balance.
        
        Stability criterion (no thermal runaway):
        dP_gen/dT < 1/R_th
        
        Where:
        - P_gen(T) = I² × R(T) is the heat generation
        - R_th is thermal resistance
        
        Since R(T) decreases with T, P_gen decreases with T for constant I.
        However, leakage current I_leak increases exponentially with T.
        
        Runaway occurs when:
        P_leak(T) = V × I_leak(T) > (T - T_amb) / R_th
        """
        # SoC leakage model
        soc_model = self.integrated.model_soc
        
        T_range = np.linspace(25, 100, 100)
        T_amb = 25
        R_th = soc_model.params.R_th
        V_dd = 0.9  # Nominal voltage
        
        # Leakage power vs temperature
        P_leak = []
        for T in T_range:
            I_leak = soc_model.get_leakage_current(V_dd, T)
            P_leak.append(V_dd * I_leak)
        
        P_leak = np.array(P_leak)
        
        # Heat dissipation capacity
        P_dissipate = (T_range - T_amb) / R_th
        
        # Find crossover point (thermal equilibrium)
        # If P_leak > P_dissipate, runaway occurs
        crossover_idx = np.where(P_leak > P_dissipate)[0]
        
        if len(crossover_idx) > 0:
            T_critical = T_range[crossover_idx[0]]
        else:
            T_critical = ">100°C (safe)"
        
        return {
            'stability_criterion': 'dP_gen/dT < 1/R_th',
            'R_th_KperW': R_th,
            'T_ambient': T_amb,
            'V_dd': V_dd,
            'T_critical': T_critical,
            'P_leak_at_25C_W': P_leak[0],
            'P_leak_at_75C_W': P_leak[np.argmin(np.abs(T_range - 75))],
            'insight': (
                "Thermal runaway is a critical safety concern. The exponential "
                "increase in leakage current with temperature creates positive feedback. "
                "Modern SoCs implement thermal throttling at ~80°C to prevent runaway. "
                "Users should avoid blocking vents and using devices in hot environments."
            )
        }
    
    def derive_optimal_charging_temperature(self) -> Dict:
        """
        Derive optimal charging temperature window.
        
        Capacity preservation: f_Q(T) × f_aging(N) should be maximized
        
        - Low T (<10°C): Lithium plating risk, reduced capacity
        - High T (>45°C): Accelerated SEI growth, faster degradation
        - Optimal: 15-35°C
        
        Arrhenius acceleration factor for aging:
        AF(T) = exp[E_a/R × (1/T_ref - 1/T)]
        
        Where E_a ≈ 50-60 kJ/mol for lithium-ion batteries
        """
        T_range = np.linspace(-20, 60, 100)
        
        # Capacity factor (from model)
        f_Q = [self.battery.get_capacity(0, T) / self.battery.get_capacity(0, 25) 
               for T in T_range]
        
        # Aging acceleration factor (Arrhenius)
        E_a = 55000  # J/mol (typical for Li-ion)
        R_gas = 8.314  # J/(mol·K)
        T_ref = 298.15  # 25°C in Kelvin
        
        def aging_factor(T_celsius):
            T_kelvin = T_celsius + 273.15
            return np.exp(E_a / R_gas * (1/T_ref - 1/T_kelvin))
        
        AF = [aging_factor(T) for T in T_range]
        
        # Combined health factor (higher is better)
        # H(T) = f_Q(T) / AF(T) - prioritize capacity while minimizing aging
        H = np.array(f_Q) / np.array(AF)
        H_normalized = H / np.max(H)
        
        # Find optimal temperature
        optimal_T = T_range[np.argmax(H)]
        
        # Acceptable range (H > 0.9 of max)
        acceptable_mask = H_normalized > 0.8
        acceptable_range = (T_range[acceptable_mask][0], T_range[acceptable_mask][-1])
        
        return {
            'optimal_temperature_C': round(optimal_T, 1),
            'acceptable_range_C': acceptable_range,
            'aging_acceleration_at_45C': round(aging_factor(45), 2),
            'aging_acceleration_at_25C': 1.0,
            'capacity_at_0C_percent': round(f_Q[np.argmin(np.abs(T_range))]*100, 1),
            'formula': 'AF(T) = exp[E_a/R × (1/T_ref - 1/T)]',
            'insight': (
                f"Optimal charging temperature is around {optimal_T:.0f}°C. "
                f"Charging at 45°C accelerates aging by ~{aging_factor(45):.1f}× compared to 25°C. "
                f"Avoid fast charging in hot environments. In cold weather (<10°C), "
                "warm the battery before fast charging to prevent lithium plating."
            )
        }
    
    # -------------------------------------------------------------------------
    # 3. Power Optimization Theory
    # -------------------------------------------------------------------------
    
    def analyze_dvfs_efficiency_frontier(self) -> Dict:
        """
        Analyze the DVFS efficiency frontier for optimal performance/power tradeoff.
        
        Energy per operation:
        E_op = P / f = α × C × V² × f / f = α × C × V²
        
        Since V ∝ f^β (β ≈ 0.5 for modern CMOS):
        E_op ∝ f^(2β) = f
        
        Energy efficiency (MIPS/Watt):
        η_eff = f / P ∝ f / f³ = 1/f²
        
        Thus, lower frequency is always more energy efficient per operation,
        but may not meet latency requirements.
        """
        soc = self.integrated.model_soc
        
        f_range = np.linspace(0.3e9, 3e9, 100)
        
        # Power at each frequency
        powers = [soc.get_dynamic_power(f) for f in f_range]
        
        # Energy per operation (normalized)
        E_op = np.array(powers) / f_range
        
        # Energy efficiency (ops per Joule)
        efficiency = f_range / np.array(powers)
        
        # Performance per Watt (normalized)
        perf_per_watt = f_range / np.array(powers) / 1e9
        
        # Find Pareto-optimal points
        # At low frequency: high efficiency, low performance
        # At high frequency: low efficiency, high performance
        
        results = {
            'formula_energy_per_op': 'E_op = α × C × V²',
            'formula_efficiency': 'η_eff = f/P ∝ 1/f²',
            'data_points': {
                'frequency_GHz': (f_range / 1e9).tolist()[::10],
                'power_W': [round(p, 3) for p in powers[::10]],
                'energy_per_op_nJ': (np.array(E_op) * 1e9).tolist()[::10],
                'efficiency_GOPS_per_W': [round(e/1e9, 2) for e in efficiency[::10]]
            },
            'insight': (
                "Lower frequencies are ALWAYS more energy efficient for a given "
                "amount of computation. A task that takes 1s at 3GHz uses ~9× more "
                "energy than running it for 3s at 1GHz. OS schedulers should use "
                "the minimum frequency that meets latency requirements (race-to-idle)."
            ),
            'recommendation': (
                "For background tasks: Use minimum frequency (0.5-1GHz). "
                "For interactive tasks: Use medium frequency (1.5-2GHz). "
                "For gaming/rendering: Use high frequency only when needed (2.5-3GHz)."
            )
        }
        
        return results
    
    def analyze_display_power_optimization(self) -> Dict:
        """
        Analyze display power optimization strategies.
        
        OLED power model:
        P_disp = P_base + k_f × f_refresh + β × L × APL
        
        Optimization axes:
        1. APL reduction (dark mode): Linear savings
        2. Brightness reduction: Linear savings
        3. Refresh rate reduction: Linear savings on driver power
        
        Combined effect:
        P_optimized / P_baseline = (P_base + k_f×f_new + β×L_new×APL_new) / 
                                   (P_base + k_f×f_old + β×L_old×APL_old)
        """
        oled = self.integrated.model_oled
        
        # Baseline: light theme, full brightness, 120Hz
        baseline = oled.get_power(0.85, 600, 120)
        
        # Optimization scenarios
        optimizations = [
            {'name': 'Dark Mode Only', 'APL': 0.15, 'L': 600, 'f': 120},
            {'name': 'Brightness 50%', 'APL': 0.85, 'L': 300, 'f': 120},
            {'name': '60Hz Refresh', 'APL': 0.85, 'L': 600, 'f': 60},
            {'name': 'Dark + 50% Bright', 'APL': 0.15, 'L': 300, 'f': 120},
            {'name': 'All Optimizations', 'APL': 0.15, 'L': 300, 'f': 60},
        ]
        
        results = {'baseline_mW': baseline['P_total']}
        
        for opt in optimizations:
            power = oled.get_power(opt['APL'], opt['L'], opt['f'])
            savings = (baseline['P_total'] - power['P_total']) / baseline['P_total'] * 100
            results[opt['name']] = {
                'power_mW': round(power['P_total'], 1),
                'savings_percent': round(savings, 1)
            }
        
        results['insight'] = (
            "Display is often the #1 power consumer. Dark mode alone can save 50-70% "
            "of emissive power. Combined with reduced brightness and 60Hz refresh, "
            "display power can be reduced by >80%. Auto-brightness sensors help maintain "
            "readability while minimizing power waste."
        )
        
        return results
    
    def analyze_communication_power_tradeoffs(self) -> Dict:
        """
        Analyze WiFi vs 5G vs LTE power tradeoffs.
        
        Key insight: Power scales with distance and data rate very differently:
        
        WiFi: Low distance (~10m), high bandwidth, low power
        P_wifi ≈ 100-400 mW
        
        5G: Variable distance (50-1000m), very high bandwidth, variable power
        P_5g ≈ 500-3500 mW (depends strongly on signal)
        
        LTE: Variable distance, medium bandwidth, medium power
        P_lte ≈ 300-1500 mW
        
        Energy per bit:
        E_bit = P / R
        WiFi typically wins for energy efficiency when available.
        """
        model_5g = self.integrated.model_5g
        
        # Compare scenarios
        scenarios = {
            'WiFi_typical': {
                'power_mW': 200,
                'rate_Mbps': 100,
                'distance_m': 10,
                'description': 'Home/Office WiFi'
            },
            '5G_good_signal': {
                'power_mW': model_5g.get_power(100e6, 200)['P_total'] * 1000,
                'rate_Mbps': 100,
                'distance_m': 200,
                'description': 'Near cell tower'
            },
            '5G_weak_signal': {
                'power_mW': model_5g.get_power(50e6, 600)['P_total'] * 1000,
                'rate_Mbps': 50,
                'distance_m': 600,
                'description': 'Cell edge'
            },
            '5G_indoor': {
                'power_mW': model_5g.get_power(30e6, 400)['P_total'] * 1000,
                'rate_Mbps': 30,
                'distance_m': 400,
                'description': 'Indoor penetration loss'
            }
        }
        
        for name, scenario in scenarios.items():
            scenario['energy_per_MB_mJ'] = round(
                scenario['power_mW'] / scenario['rate_Mbps'], 2
            )
        
        return {
            'scenarios': scenarios,
            'formula': 'E_bit = P / R',
            'insight': (
                "WiFi uses ~5× less energy per bit compared to 5G in weak signal conditions. "
                "When WiFi is available, switching from 5G can save 500-2000mW. "
                "OS should aggressively prefer WiFi for background downloads."
            ),
            'recommendation': (
                "1. Enable WiFi whenever available\n"
                "2. Download large files on WiFi, not cellular\n"
                "3. Avoid streaming in weak signal areas\n"
                "4. Consider airplane mode + WiFi for max battery"
            )
        }


# =============================================================================
# User Recommendations Generator
# =============================================================================

class UserRecommendationsGenerator:
    """
    Generate actionable recommendations for users based on model insights.
    """
    
    def __init__(self):
        self.analysis = ExtendedTheoreticalAnalysis()
        self.integrated = IntegratedPowerModel()
        
    def generate_all_recommendations(self) -> Dict:
        """Generate comprehensive recommendations."""
        recommendations = {
            'display_settings': self._display_recommendations(),
            'connectivity': self._connectivity_recommendations(),
            'performance_settings': self._performance_recommendations(),
            'background_activity': self._background_recommendations(),
            'battery_health': self._battery_health_recommendations(),
            'usage_patterns': self._usage_pattern_recommendations(),
            'environmental_factors': self._environmental_recommendations(),
            'quantified_impact': self._quantified_savings()
        }
        
        return recommendations
    
    def _display_recommendations(self) -> Dict:
        """Recommendations for display settings."""
        return {
            'priority': 'HIGH',
            'potential_savings': '35-75%',
            'recommendations': [
                {
                    'action': 'Enable Dark Mode',
                    'impact': '50-70% reduction in OLED emissive power',
                    'technical_basis': 'OLED pixels emit zero light when displaying black',
                    'implementation': 'System settings → Display → Dark theme'
                },
                {
                    'action': 'Use Auto-Brightness',
                    'impact': '20-40% average brightness reduction',
                    'technical_basis': 'Ambient light sensors adjust to actual needs',
                    'implementation': 'Enable adaptive brightness in display settings'
                },
                {
                    'action': 'Reduce Refresh Rate',
                    'impact': '15-25% reduction in driver power',
                    'technical_basis': 'P_driver ∝ refresh_rate (LTPO helps, but 60Hz saves more)',
                    'implementation': 'Settings → Display → Refresh rate → Standard (60Hz)'
                },
                {
                    'action': 'Reduce Screen Timeout',
                    'impact': 'Variable (depends on usage)',
                    'technical_basis': 'Display off = ~0W emissive power',
                    'implementation': 'Set 30s or 1min timeout'
                }
            ]
        }
    
    def _connectivity_recommendations(self) -> Dict:
        """Recommendations for connectivity management."""
        return {
            'priority': 'HIGH',
            'potential_savings': '20-40%',
            'recommendations': [
                {
                    'action': 'Prefer WiFi over 5G/LTE',
                    'impact': '60-80% reduction in communication power',
                    'technical_basis': 'WiFi: ~200mW vs 5G: 800-3000mW at weak signal',
                    'implementation': 'Auto-connect to known WiFi networks'
                },
                {
                    'action': 'Disable 5G in Weak Signal Areas',
                    'impact': 'Prevents exponential power increase',
                    'technical_basis': 'P_tx ∝ d^n × 2^(R/B), exponential with distance',
                    'implementation': 'Switch to LTE or WiFi in basements, tunnels'
                },
                {
                    'action': 'Manage Bluetooth Connections',
                    'impact': '10-50mW savings per idle connection',
                    'technical_basis': 'BLE current scales as 1/connection_interval',
                    'implementation': 'Disconnect unused devices, use 500ms+ intervals'
                },
                {
                    'action': 'Disable GPS When Not Navigating',
                    'impact': '45-115mW savings',
                    'technical_basis': 'GNSS acquisition draws 115mW, tracking 45mW',
                    'implementation': 'Turn off location services for non-essential apps'
                }
            ]
        }
    
    def _performance_recommendations(self) -> Dict:
        """Recommendations for performance settings."""
        return {
            'priority': 'MEDIUM',
            'potential_savings': '15-30%',
            'recommendations': [
                {
                    'action': 'Use Battery Saver Mode',
                    'impact': '15-25% overall power reduction',
                    'technical_basis': 'Reduces CPU frequency, limits background',
                    'implementation': 'Enable automatically at 20-30% SOC'
                },
                {
                    'action': 'Close Unused Apps',
                    'impact': '5-15% reduction in background power',
                    'technical_basis': 'Prevents random wakeups and tail energy waste',
                    'implementation': 'Regular app cleanup, disable keep-alive for non-essential'
                },
                {
                    'action': 'Limit Gaming Sessions in Hot Weather',
                    'impact': 'Reduces leakage power explosion',
                    'technical_basis': 'I_leak ∝ exp(T), 10°C rise = 2× leakage',
                    'implementation': 'Take breaks, use cooling accessories'
                }
            ]
        }
    
    def _background_recommendations(self) -> Dict:
        """Recommendations for background activity management."""
        return {
            'priority': 'MEDIUM',
            'potential_savings': '10-20%',
            'recommendations': [
                {
                    'action': 'Limit Background App Refresh',
                    'impact': 'Reduces tail energy waste',
                    'technical_basis': 'Each wakeup triggers 12s tail time on cellular',
                    'implementation': 'Settings → Apps → Background refresh → Essential only'
                },
                {
                    'action': 'Schedule Sync Intervals',
                    'impact': 'Allows longer deep sleep periods',
                    'technical_basis': 'P_bg saturates when wakeup_rate × τ_tail > 1',
                    'implementation': 'Use manual sync or hourly updates'
                },
                {
                    'action': 'Disable Push Notifications for Non-Essential Apps',
                    'impact': '5-10mW average reduction',
                    'technical_basis': 'Each push wakes CPU + radio',
                    'implementation': 'Audit notification permissions'
                }
            ]
        }
    
    def _battery_health_recommendations(self) -> Dict:
        """Recommendations for battery longevity."""
        return {
            'priority': 'MEDIUM-HIGH',
            'potential_savings': '+20-50% battery lifespan',
            'recommendations': [
                {
                    'action': 'Avoid Charging Above 80% Regularly',
                    'impact': '20-40% longer battery lifespan',
                    'technical_basis': 'High SOC accelerates SEI growth and lithium plating',
                    'implementation': 'Use optimized charging features'
                },
                {
                    'action': 'Avoid Deep Discharge Below 20%',
                    'impact': 'Reduces stress cycles',
                    'technical_basis': 'Deep cycles cause greater structural strain',
                    'implementation': 'Charge before reaching critical low levels'
                },
                {
                    'action': 'Charge at Moderate Temperatures (15-35°C)',
                    'impact': 'Prevents accelerated aging',
                    'technical_basis': 'Aging rate follows Arrhenius: AF ∝ exp(E_a/RT)',
                    'implementation': 'Remove case during charging in warm environments'
                },
                {
                    'action': 'Avoid Fast Charging When Hot',
                    'impact': 'Reduces thermal stress',
                    'technical_basis': 'Fast charging generates significant heat (I²R)',
                    'implementation': 'Use standard charging when phone is warm'
                },
                {
                    'action': 'Store at 50% SOC for Long Periods',
                    'impact': 'Minimizes self-discharge degradation',
                    'technical_basis': 'Calendar aging minimized at mid-SOC',
                    'implementation': 'Charge to 50% before long-term storage'
                }
            ]
        }
    
    def _usage_pattern_recommendations(self) -> Dict:
        """Recommendations for usage patterns."""
        return {
            'priority': 'VARIES',
            'recommendations': [
                {
                    'action': 'Batch Tasks Together',
                    'impact': 'Maximizes deep sleep opportunities',
                    'technical_basis': 'Continuous activity is more efficient than sporadic',
                    'implementation': 'Check email/social in scheduled sessions'
                },
                {
                    'action': 'Download Content on WiFi for Offline Use',
                    'impact': 'Avoids streaming power cost',
                    'technical_basis': 'Playback: ~300mW, Streaming: ~1500mW',
                    'implementation': 'Use download features in streaming apps'
                },
                {
                    'action': 'Use Airplane Mode in No-Signal Areas',
                    'impact': 'Prevents futile signal searching',
                    'technical_basis': 'Cell search at max TX power (~3W) drains rapidly',
                    'implementation': 'Enable before entering tunnels, basements'
                }
            ]
        }
    
    def _environmental_recommendations(self) -> Dict:
        """Recommendations for environmental factors."""
        return {
            'priority': 'MEDIUM',
            'recommendations': [
                {
                    'action': 'Keep Phone Cool During Heavy Use',
                    'impact': 'Maintains efficiency, prevents throttling',
                    'technical_basis': 'Leakage doubles every ~10°C',
                    'implementation': 'Remove case, avoid direct sunlight'
                },
                {
                    'action': 'Warm Up Battery Before Use in Cold',
                    'impact': 'Restores capacity and reduces internal resistance',
                    'technical_basis': 'At -10°C, capacity drops to ~60%',
                    'implementation': 'Keep in pocket before extended outdoor use'
                },
                {
                    'action': 'Avoid Leaving in Hot Cars',
                    'impact': 'Prevents permanent capacity loss',
                    'technical_basis': '>60°C accelerates electrolyte decomposition',
                    'implementation': 'Take phone with you or keep in shade'
                }
            ]
        }
    
    def _quantified_savings(self) -> Dict:
        """Provide quantified power savings for key actions."""
        return {
            'action_impact_table': [
                {'action': 'Dark Mode (OLED)', 'typical_savings_mW': '400-800', 'percentage': '35-50% of display'},
                {'action': 'Brightness 50%→30%', 'typical_savings_mW': '200-400', 'percentage': '15-25% of display'},
                {'action': '120Hz→60Hz', 'typical_savings_mW': '75-150', 'percentage': '5-10% of display'},
                {'action': 'WiFi vs 5G (weak signal)', 'typical_savings_mW': '1000-2500', 'percentage': '50-70% of comms'},
                {'action': 'Disable GPS', 'typical_savings_mW': '45-115', 'percentage': '100% of GNSS'},
                {'action': 'Background limit', 'typical_savings_mW': '20-80', 'percentage': '30-60% of background'},
                {'action': 'CPU 3GHz→1.5GHz', 'typical_savings_mW': '1500-2500', 'percentage': '60-75% of CPU dynamic'},
            ],
            'combined_maximum_savings': '60-70% of total system power',
            'realistic_savings': '30-40% with comfortable use experience'
        }


# =============================================================================
# OS-Level Power Management Strategies
# =============================================================================

class OSPowerManagementStrategies:
    """
    Strategies for OS developers to implement effective power management.
    """
    
    @staticmethod
    def get_dvfs_policy_recommendations() -> Dict:
        """DVFS scheduling policy recommendations."""
        return {
            'race_to_idle': {
                'description': 'Complete tasks quickly at high frequency, then sleep',
                'when_to_use': 'Short burst tasks (UI interactions)',
                'power_model': 'E = P_active × t_active + P_idle × t_idle',
                'insight': 'Only beneficial when P_idle << P_active'
            },
            'pace_to_idle': {
                'description': 'Run at minimum frequency that meets deadline',
                'when_to_use': 'Deadline-insensitive background tasks',
                'power_model': 'E ∝ V² × ops, V ∝ f^0.5, so E ∝ f × ops',
                'insight': 'Always more energy efficient for fixed workload'
            },
            'adaptive_policy': {
                'description': 'Predict workload and select optimal V/F point',
                'implementation': [
                    'Track task completion times',
                    'Use ML predictor for workload estimation',
                    'Set frequency to just meet predicted demand',
                    'Include headroom for prediction errors'
                ]
            }
        }
    
    @staticmethod
    def get_connectivity_management_policy() -> Dict:
        """Network interface power management policies."""
        return {
            'wifi_offload_policy': {
                'description': 'Aggressively shift traffic to WiFi when available',
                'triggers': ['WiFi RSSI > -70dBm', 'Estimated data > 1MB'],
                'savings': '60-80% compared to cellular for same data'
            },
            'cell_discontinuous_reception': {
                'description': 'Maximize DRX cycles in connected mode',
                'implementation': 'Request longer DRX cycles from network',
                'savings': '30-50% of cellular idle power'
            },
            'adaptive_scan_interval': {
                'description': 'Reduce WiFi/BT scan frequency based on context',
                'implementation': [
                    'If stationary (no motion): scan every 60s',
                    'If walking: scan every 30s',
                    'If driving: scan every 15s'
                ],
                'savings': '10-20mW average'
            }
        }
    
    @staticmethod
    def get_display_management_policy() -> Dict:
        """Display power management policies."""
        return {
            'content_aware_brightness': {
                'description': 'Reduce backlight and boost gamma for dark content',
                'implementation': 'APL < 0.3 → reduce brightness 20%, boost contrast',
                'savings': '15-25% additional on dark content'
            },
            'ltpo_optimization': {
                'description': 'Dynamically adjust refresh rate based on content',
                'policy': {
                    'static_content': '1-10Hz',
                    'scrolling_text': '30Hz',
                    'video_24fps': '24Hz',
                    'gaming': '60-120Hz as needed'
                },
                'savings': '20-40% of driver power'
            },
            'proximity_dimming': {
                'description': 'Dim display when user not actively looking',
                'implementation': 'Use front camera + ML for attention detection',
                'savings': '10-30% of display power'
            }
        }
    
    @staticmethod
    def get_thermal_management_policy() -> Dict:
        """Thermal management policies."""
        return {
            'proactive_throttling': {
                'description': 'Reduce power before reaching thermal limit',
                'implementation': [
                    'Monitor temperature trend (dT/dt)',
                    'If dT/dt > threshold, start gentle frequency reduction',
                    'Avoid sudden performance drops'
                ],
                'benefit': 'Smoother UX, prevents thermal shutdown'
            },
            'skin_temperature_targeting': {
                'description': 'Target comfortable skin temperature, not just safety',
                'target': '40-43°C skin temperature',
                'implementation': 'Use skin temp sensor or thermal model'
            },
            'workload_spreading': {
                'description': 'Distribute workload across time to manage heat',
                'implementation': 'Defer non-urgent tasks if temperature rising',
                'benefit': 'Maintains performance without throttling'
            }
        }


# =============================================================================
# Generalization to Other Devices
# =============================================================================

class DeviceGeneralization:
    """
    Generalization of modeling framework to other portable devices.
    """
    
    @staticmethod
    def get_device_profiles() -> Dict:
        """Power profiles for different device categories."""
        return {
            'smartphone': {
                'battery_capacity_Wh': 15-20,
                'dominant_loads': ['Display', 'SoC', '5G'],
                'typical_runtime_h': 8-15,
                'unique_challenges': ['Thermal in thin form factor', 'Variable connectivity']
            },
            'tablet': {
                'battery_capacity_Wh': 30-40,
                'dominant_loads': ['Display (larger)', 'SoC'],
                'typical_runtime_h': 10-15,
                'unique_challenges': ['Larger display dominates', 'Often WiFi-only']
            },
            'smartwatch': {
                'battery_capacity_Wh': 1-2,
                'dominant_loads': ['Display', 'Sensors', 'BLE'],
                'typical_runtime_h': 24-72,
                'unique_challenges': ['Extreme miniaturization', 'Always-on display']
            },
            'laptop': {
                'battery_capacity_Wh': 50-100,
                'dominant_loads': ['CPU', 'Display', 'GPU'],
                'typical_runtime_h': 8-15,
                'unique_challenges': ['Higher TDP', 'Active cooling']
            },
            'wireless_earbuds': {
                'battery_capacity_Wh': 0.1-0.3,
                'dominant_loads': ['Bluetooth', 'Audio DSP'],
                'typical_runtime_h': 4-8,
                'unique_challenges': ['Tiny battery', 'Constant BT streaming']
            },
            'e_reader': {
                'battery_capacity_Wh': 5-10,
                'dominant_loads': ['E-ink refresh', 'WiFi (when on)'],
                'typical_runtime_h': 200-500,  # Weeks!
                'unique_challenges': ['E-ink is bistable (no power to hold image)']
            }
        }
    
    @staticmethod
    def get_model_adaptations() -> Dict:
        """How to adapt the framework for different devices."""
        return {
            'battery_model': {
                'adaptation': 'Scale capacity parameters, adjust thermal constants',
                'key_differences': [
                    'Smaller batteries have higher C-rates for same current',
                    'Thermal resistance varies with form factor',
                    'Aging rates depend on typical use patterns'
                ]
            },
            'load_models': {
                'adaptation': 'Replace/remove subsystems, adjust power levels',
                'examples': [
                    'Smartwatch: Remove 5G, add heart rate sensor',
                    'Laptop: Add GPU model, scale CPU power',
                    'Earbuds: Simplify to BT + audio codec only'
                ]
            },
            'thermal_model': {
                'adaptation': 'Adjust thermal resistances and capacitances',
                'considerations': [
                    'Larger devices have better thermal spreading',
                    'Wearables transfer heat to body',
                    'Laptops have active cooling option'
                ]
            }
        }


# =============================================================================
# Main Execution
# =============================================================================

def generate_comprehensive_report():
    """Generate comprehensive analysis report."""
    
    print("=" * 70)
    print("COMPREHENSIVE BATTERY MODELING AND POWER OPTIMIZATION REPORT")
    print("=" * 70)
    
    analysis = ExtendedTheoreticalAnalysis()
    recommendations = UserRecommendationsGenerator()
    
    # Extended theoretical analysis
    print("\n" + "=" * 70)
    print("SECTION 1: EXTENDED THEORETICAL ANALYSIS")
    print("=" * 70)
    
    print("\n1.1 SOC Observability Analysis")
    print("-" * 40)
    soc_analysis = analysis.analyze_soc_observability()
    print(f"Best observability SOC: {soc_analysis['best_observability_SOC']:.2f}")
    print(f"Worst observability range: {soc_analysis['worst_observability_SOC_range']}")
    print(f"Insight: {soc_analysis['insight']}")
    
    print("\n1.2 Runtime Prediction")
    print("-" * 40)
    runtime = analysis.derive_runtime_equation()
    print(f"Formula: {runtime['formula']}")
    for name, data in runtime['runtime_estimates'].items():
        print(f"  {name}: {data['runtime_hours']}h ({data['description']})")
    
    print("\n1.3 Thermal Runaway Analysis")
    print("-" * 40)
    thermal = analysis.analyze_thermal_runaway_risk()
    print(f"Critical temperature: {thermal['T_critical']}")
    print(f"Insight: {thermal['insight'][:200]}...")
    
    print("\n1.4 Optimal Charging Temperature")
    print("-" * 40)
    charging = analysis.derive_optimal_charging_temperature()
    print(f"Optimal temperature: {charging['optimal_temperature_C']}°C")
    print(f"Acceptable range: {charging['acceptable_range_C']}")
    print(f"Aging acceleration at 45°C: {charging['aging_acceleration_at_45C']}×")
    
    print("\n1.5 DVFS Efficiency Analysis")
    print("-" * 40)
    dvfs = analysis.analyze_dvfs_efficiency_frontier()
    print(f"Key insight: {dvfs['insight'][:200]}...")
    
    print("\n1.6 Display Optimization")
    print("-" * 40)
    display = analysis.analyze_display_power_optimization()
    print(f"Baseline power: {display['baseline_mW']:.0f} mW")
    print(f"All optimizations: {display['All Optimizations']['power_mW']:.0f} mW "
          f"({display['All Optimizations']['savings_percent']:.0f}% savings)")
    
    print("\n1.7 Communication Power Tradeoffs")
    print("-" * 40)
    comm = analysis.analyze_communication_power_tradeoffs()
    for name, data in comm['scenarios'].items():
        print(f"  {name}: {data['power_mW']:.0f} mW, {data['energy_per_MB_mJ']} mJ/MB")
    
    # User recommendations
    print("\n" + "=" * 70)
    print("SECTION 2: USER RECOMMENDATIONS")
    print("=" * 70)
    
    all_recs = recommendations.generate_all_recommendations()
    
    for category, data in all_recs.items():
        if category == 'quantified_impact':
            continue
        print(f"\n{category.upper().replace('_', ' ')}")
        print("-" * 40)
        if 'recommendations' in data:
            for rec in data['recommendations'][:2]:  # First 2 per category
                print(f"  • {rec['action']}: {rec['impact']}")
    
    print("\n" + "=" * 70)
    print("QUANTIFIED SAVINGS SUMMARY")
    print("=" * 70)
    
    impact = all_recs['quantified_impact']['action_impact_table']
    print(f"\n{'Action':<30} {'Savings (mW)':<15} {'Percentage':<20}")
    print("-" * 65)
    for item in impact:
        print(f"{item['action']:<30} {item['typical_savings_mW']:<15} {item['percentage']:<20}")
    
    print(f"\nCombined maximum savings: {all_recs['quantified_impact']['combined_maximum_savings']}")
    print(f"Realistic savings: {all_recs['quantified_impact']['realistic_savings']}")
    
    print("\n" + "=" * 70)
    print("Report generation complete!")
    print("=" * 70)


if __name__ == "__main__":
    generate_comprehensive_report()
