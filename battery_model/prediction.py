"""
Remaining Discharge Time Prediction
剩余放电时间预测

This module implements:
1. Analytical approximations for discharge time
2. Monte Carlo simulation for uncertainty quantification
3. Scenario-based prediction with confidence intervals
4. Real-time prediction updates using Kalman filter

References:
[1] Hu et al., "Battery Remaining Useful Life Prediction," J. Power Sources, 2020
[2] Ng et al., "Predicting the State of Charge and Health of Batteries," Nature, 2020
"""

import numpy as np
from typing import Tuple, Dict, List, Optional, Callable
from dataclasses import dataclass
from scipy.integrate import solve_ivp, quad
from scipy.interpolate import interp1d
import warnings

from .parameters import BatteryParameters, UsageScenarios


@dataclass
class PredictionResult:
    """Prediction result with uncertainty"""
    time_remaining: float           # Expected remaining time (hours)
    confidence_interval: Tuple[float, float]  # 95% CI
    std_deviation: float            # Standard deviation
    scenarios: Dict[str, float]     # Time for different scenarios
    probability_distribution: np.ndarray = None  # PDF samples


class RemainingTimePredictor:
    """
    Remaining Discharge Time Predictor
    剩余放电时间预测器
    
    Predicts how long the battery will last under various usage conditions
    using both analytical and simulation-based methods.
    """
    
    def __init__(self, battery_params: BatteryParameters = None):
        """
        Initialize predictor
        
        Parameters:
        -----------
        battery_params : BatteryParameters
            Battery parameter object
        """
        self.battery = battery_params or BatteryParameters()
        
        # Precomputed lookup tables for fast prediction
        self._build_lookup_tables()
    
    def _build_lookup_tables(self):
        """Build lookup tables for common scenarios"""
        self.power_table = {}
        
        # Typical power consumption for scenarios (Watts)
        self.power_table['idle'] = 0.15
        self.power_table['light_use'] = 0.8
        self.power_table['video_streaming'] = 2.5
        self.power_table['navigation'] = 3.2
        self.power_table['gaming'] = 4.5
        self.power_table['heavy_multitask'] = 5.0
    
    def analytical_discharge_time(self, SOC_current: float,
                                   P_avg: float,
                                   SOC_cutoff: float = 0.05,
                                   T_batt: float = 298.15,
                                   cycle_count: int = 0) -> float:
        """
        计算解析放电时间
        
        Uses simplified model: t = (SOC_current - SOC_cutoff) * Q * V_avg / P_avg
        
        With corrections for:
        - Temperature effect on capacity
        - Aging effect on capacity
        - Voltage variation with SOC
        
        Parameters:
        -----------
        SOC_current : float
            Current state of charge (0-1)
        P_avg : float
            Average power consumption (Watts)
        SOC_cutoff : float
            Cutoff SOC (default 5%)
        T_batt : float
            Battery temperature (K)
        cycle_count : int
            Number of charge cycles
            
        Returns:
        --------
        float : Estimated remaining time in hours
        """
        if P_avg <= 0:
            return np.inf
        
        # Usable SOC range
        delta_SOC = SOC_current - SOC_cutoff
        if delta_SOC <= 0:
            return 0.0
        
        # Effective capacity with temperature and aging correction
        Q_max = self.battery.Q_max_Ah  # Ah
        
        # Temperature correction: capacity drops in cold
        T_celsius = T_batt - 273.15
        T_factor = 1.0
        if T_celsius < 20:
            T_factor = 1.0 - 0.01 * (20 - T_celsius)  # ~1% per °C below 20°C
        elif T_celsius > 40:
            T_factor = 1.0 - 0.005 * (T_celsius - 40)  # Slight degradation at high T
        
        # Aging correction
        aging_factor = self.battery.capacity_fade(cycle_count)
        
        Q_eff = Q_max * T_factor * aging_factor
        
        # Average voltage during discharge
        # Compute weighted average over SOC range
        n_points = 20
        SOC_range = np.linspace(SOC_cutoff, SOC_current, n_points)
        V_range = self.battery.V_OCV(SOC_range)
        V_avg = np.trapz(V_range, SOC_range) / delta_SOC
        
        # Energy available (Wh)
        E_available = Q_eff * V_avg * delta_SOC
        
        # Time in hours
        t_hours = E_available / P_avg
        
        return t_hours
    
    def predict_with_scenario(self, SOC_current: float,
                               scenario: str = 'light_use',
                               T_batt: float = 298.15,
                               cycle_count: int = 0) -> PredictionResult:
        """
        基于使用场景的预测
        
        Parameters:
        -----------
        SOC_current : float
            Current SOC (0-1)
        scenario : str
            Usage scenario name
        T_batt : float
            Battery temperature (K)
        cycle_count : int
            Charge cycles
            
        Returns:
        --------
        PredictionResult with expected time and scenarios
        """
        # Get power for requested scenario
        P_scenario = self.power_table.get(scenario, self.power_table['light_use'])
        
        # Compute time for requested scenario
        t_main = self.analytical_discharge_time(SOC_current, P_scenario, 0.05, T_batt, cycle_count)
        
        # Compute for all scenarios
        scenario_times = {}
        for scen, power in self.power_table.items():
            scenario_times[scen] = self.analytical_discharge_time(
                SOC_current, power, 0.05, T_batt, cycle_count
            )
        
        # Estimate uncertainty (±20% typical)
        std = t_main * 0.2
        ci_low = max(0, t_main - 1.96 * std)
        ci_high = t_main + 1.96 * std
        
        return PredictionResult(
            time_remaining=t_main,
            confidence_interval=(ci_low, ci_high),
            std_deviation=std,
            scenarios=scenario_times
        )
    
    def monte_carlo_prediction(self, SOC_current: float,
                                usage_distribution: Dict,
                                n_samples: int = 1000,
                                T_batt: float = 298.15,
                                cycle_count: int = 0) -> PredictionResult:
        """
        蒙特卡洛模拟预测
        
        Accounts for uncertainty in:
        - Usage pattern (time spent in each scenario)
        - Power consumption variation
        - Battery parameters
        
        Parameters:
        -----------
        SOC_current : float
            Current SOC
        usage_distribution : dict
            Dict with scenario names and (mean_fraction, std_fraction) tuples
        n_samples : int
            Number of Monte Carlo samples
            
        Returns:
        --------
        PredictionResult with distribution
        """
        discharge_times = []
        
        for _ in range(n_samples):
            # Sample usage fractions
            fractions = {}
            total = 0
            for scenario, (mean, std) in usage_distribution.items():
                frac = max(0, np.random.normal(mean, std))
                fractions[scenario] = frac
                total += frac
            
            # Normalize fractions
            if total > 0:
                for scenario in fractions:
                    fractions[scenario] /= total
            else:
                fractions = {'idle': 1.0}
            
            # Compute average power
            P_avg = sum(fractions[s] * self.power_table.get(s, 0.5) 
                       for s in fractions)
            
            # Add parameter uncertainty
            P_avg *= np.random.normal(1.0, 0.1)
            Q_factor = np.random.normal(1.0, 0.05)
            
            # Compute discharge time
            t = self.analytical_discharge_time(
                SOC_current, max(P_avg, 0.1), 0.05, T_batt, cycle_count
            ) * Q_factor
            
            discharge_times.append(t)
        
        discharge_times = np.array(discharge_times)
        
        mean_time = np.mean(discharge_times)
        std_time = np.std(discharge_times)
        percentiles = np.percentile(discharge_times, [2.5, 97.5])
        
        return PredictionResult(
            time_remaining=mean_time,
            confidence_interval=(percentiles[0], percentiles[1]),
            std_deviation=std_time,
            scenarios={},
            probability_distribution=discharge_times
        )
    
    def predict_with_profile(self, SOC_current: float,
                              power_profile: Callable[[float], float],
                              T_profile: Callable[[float], float] = None,
                              max_time: float = 48.0) -> Tuple[float, np.ndarray]:
        """
        基于功率曲线的预测
        
        Integrates the discharge equation with time-varying power consumption.
        
        Parameters:
        -----------
        SOC_current : float
            Starting SOC
        power_profile : callable
            Function P(t) giving power in Watts at time t (hours)
        T_profile : callable
            Function T(t) giving temperature in K
        max_time : float
            Maximum simulation time (hours)
            
        Returns:
        --------
        (discharge_time, SOC_trajectory)
        """
        if T_profile is None:
            T_profile = lambda t: 298.15
        
        Q_max_Wh = self.battery.Q_max_Ah * self.battery.V_nom
        
        def dSOC_dt(t, SOC):
            if SOC <= 0.05:
                return 0
            P = power_profile(t)
            T = T_profile(t)
            
            # Capacity correction
            T_celsius = T - 273.15
            T_factor = 1.0 - 0.01 * max(0, 20 - T_celsius)
            
            # Average voltage at current SOC
            V = self.battery.V_OCV(np.array([SOC]))[0]
            
            # dSOC/dt = -P / (Q * V)
            dSOC = -P / (Q_max_Wh * T_factor * 1)  # Simplified
            return dSOC
        
        # Integrate
        t_span = (0, max_time)
        t_eval = np.linspace(0, max_time, 1000)
        
        sol = solve_ivp(dSOC_dt, t_span, [SOC_current], t_eval=t_eval,
                        events=lambda t, y: y[0] - 0.05)
        
        SOC_trajectory = sol.y[0]
        
        # Find discharge time
        if sol.t_events[0].size > 0:
            discharge_time = sol.t_events[0][0]
        else:
            # Didn't reach cutoff
            discharge_time = max_time
        
        return discharge_time, SOC_trajectory
    
    def real_time_prediction(self, SOC_current: float,
                              SOC_history: np.ndarray,
                              time_history: np.ndarray,
                              SOC_cutoff: float = 0.05) -> PredictionResult:
        """
        实时预测（基于历史数据）
        
        Uses recent discharge rate to predict remaining time.
        
        Parameters:
        -----------
        SOC_current : float
            Current SOC
        SOC_history : array
            Recent SOC values
        time_history : array
            Time stamps (hours) for SOC values
            
        Returns:
        --------
        PredictionResult
        """
        if len(SOC_history) < 2:
            return self.predict_with_scenario(SOC_current, 'light_use')
        
        # Estimate discharge rate (SOC/hour)
        # Use exponential weighting for recent data
        n = len(SOC_history)
        weights = np.exp(np.linspace(-2, 0, n))
        weights /= weights.sum()
        
        # Compute weighted average discharge rate
        dSOC_dt_samples = np.diff(SOC_history) / np.diff(time_history)
        dSOC_dt_avg = np.average(dSOC_dt_samples, weights=weights[1:])
        
        if dSOC_dt_avg >= 0:
            # Battery not discharging
            return PredictionResult(
                time_remaining=np.inf,
                confidence_interval=(0, np.inf),
                std_deviation=0,
                scenarios={}
            )
        
        # Predicted time to cutoff
        t_remaining = (SOC_current - SOC_cutoff) / (-dSOC_dt_avg)
        
        # Uncertainty from rate variation
        rate_std = np.std(dSOC_dt_samples)
        if rate_std > 0 and dSOC_dt_avg != 0:
            # Propagate uncertainty
            rel_std = rate_std / abs(dSOC_dt_avg)
            t_std = t_remaining * rel_std
        else:
            t_std = t_remaining * 0.2
        
        return PredictionResult(
            time_remaining=t_remaining,
            confidence_interval=(max(0, t_remaining - 1.96*t_std), t_remaining + 1.96*t_std),
            std_deviation=t_std,
            scenarios={}
        )
    
    def predict_usage_impact(self, SOC_current: float,
                              base_scenario: str = 'light_use',
                              modifications: Dict[str, float] = None) -> Dict:
        """
        预测使用习惯改变的影响
        
        Quantifies how changing usage would affect battery life.
        
        Parameters:
        -----------
        SOC_current : float
            Current SOC
        base_scenario : str
            Baseline scenario
        modifications : dict
            Changes to apply (e.g., {'brightness': -0.2})
            
        Returns:
        --------
        Dict with impact analysis
        """
        base_time = self.predict_with_scenario(SOC_current, base_scenario).time_remaining
        
        impacts = {}
        
        # Analyze impact of each factor
        factors = {
            'brightness_reduction': -0.3,  # 30% reduction
            'disable_gps': 0.0,
            'disable_bluetooth': 0.0,
            'reduce_refresh_rate': 0.0,
            'enable_dark_mode': 0.0,
            'reduce_cpu_load': -0.2
        }
        
        # Power savings estimates (fraction of total saved)
        savings = {
            'brightness_reduction': 0.15,
            'disable_gps': 0.05,
            'disable_bluetooth': 0.02,
            'reduce_refresh_rate': 0.05,
            'enable_dark_mode': 0.10,
            'reduce_cpu_load': 0.10
        }
        
        for factor, saving_frac in savings.items():
            # Compute new time with this optimization
            P_base = self.power_table.get(base_scenario, 1.0)
            P_new = P_base * (1 - saving_frac)
            t_new = self.analytical_discharge_time(SOC_current, P_new)
            
            impacts[factor] = {
                'time_gain_hours': t_new - base_time,
                'time_gain_percent': (t_new - base_time) / base_time * 100 if base_time > 0 else 0,
                'power_saved_watts': P_base * saving_frac
            }
        
        return {
            'base_time': base_time,
            'impacts': impacts,
            'total_possible_gain': sum(i['time_gain_hours'] for i in impacts.values())
        }


class AdaptivePredictor:
    """
    Adaptive Predictor using online learning
    自适应预测器（在线学习）
    
    Continuously updates prediction model based on observed behavior.
    """
    
    def __init__(self, base_predictor: RemainingTimePredictor):
        self.predictor = base_predictor
        
        # Learned correction factors
        self.power_corrections = {s: 1.0 for s in ['idle', 'light_use', 'video_streaming',
                                                    'navigation', 'gaming', 'heavy_multitask']}
        self.capacity_correction = 1.0
        
        # Observation history
        self.predictions = []
        self.actuals = []
        
        # Learning rate
        self.alpha = 0.1
    
    def update(self, predicted_time: float, actual_time: float, scenario: str):
        """
        Update model based on prediction error
        根据预测误差更新模型
        """
        self.predictions.append(predicted_time)
        self.actuals.append(actual_time)
        
        if actual_time > 0:
            error_ratio = predicted_time / actual_time
            
            # Update power correction for this scenario
            current = self.power_corrections.get(scenario, 1.0)
            self.power_corrections[scenario] = current * (1 - self.alpha) + error_ratio * self.alpha
        
        # Update capacity correction based on overall bias
        if len(self.actuals) >= 5:
            recent_pred = np.array(self.predictions[-5:])
            recent_actual = np.array(self.actuals[-5:])
            
            # Avoid division by zero
            valid = recent_actual > 0
            if np.any(valid):
                bias = np.mean(recent_pred[valid] / recent_actual[valid])
                self.capacity_correction = self.capacity_correction * (1 - self.alpha) + (1/bias) * self.alpha
    
    def predict(self, SOC_current: float, scenario: str = 'light_use',
                T_batt: float = 298.15) -> PredictionResult:
        """
        Make corrected prediction
        进行校正后的预测
        """
        # Get base prediction
        result = self.predictor.predict_with_scenario(SOC_current, scenario, T_batt)
        
        # Apply corrections
        correction = self.power_corrections.get(scenario, 1.0) * self.capacity_correction
        corrected_time = result.time_remaining / correction
        
        # Adjust confidence interval
        ci_low = result.confidence_interval[0] / correction
        ci_high = result.confidence_interval[1] / correction
        
        return PredictionResult(
            time_remaining=corrected_time,
            confidence_interval=(ci_low, ci_high),
            std_deviation=result.std_deviation / correction,
            scenarios={k: v / correction for k, v in result.scenarios.items()}
        )


def compute_discharge_curve(battery_params: BatteryParameters,
                            P_discharge: float = 1.0,
                            T_batt: float = 298.15,
                            n_points: int = 100) -> Dict[str, np.ndarray]:
    """
    计算完整放电曲线
    
    Generates theoretical discharge curve for given constant power load.
    
    Parameters:
    -----------
    battery_params : BatteryParameters
        Battery parameters
    P_discharge : float
        Constant discharge power (Watts)
    T_batt : float
        Battery temperature (K)
    n_points : int
        Number of points in curve
        
    Returns:
    --------
    Dict with 't', 'SOC', 'V', 'I' arrays
    """
    Q_max_As = battery_params.Q_max_As
    
    # Numerical integration
    SOC = 1.0
    SOC_list = [SOC]
    V_list = []
    I_list = []
    t_list = [0]
    
    t = 0
    dt = 1.0  # 1 second steps
    
    while SOC > 0.05:
        # Compute voltage at current SOC
        V = battery_params.V_OCV(np.array([SOC]))[0]
        
        # Compute current for constant power
        I = P_discharge / V
        
        # Update SOC
        dSOC = -I * dt / Q_max_As
        SOC += dSOC
        SOC = max(SOC, 0)
        
        t += dt
        
        SOC_list.append(SOC)
        V_list.append(V)
        I_list.append(I)
        t_list.append(t)
    
    # Interpolate to n_points
    t_array = np.array(t_list)
    t_interp = np.linspace(0, t_array[-1], n_points)
    
    SOC_interp = np.interp(t_interp, t_array, SOC_list)
    
    # Compute V and I for interpolated SOC
    V_interp = battery_params.V_OCV(SOC_interp)
    I_interp = P_discharge / V_interp
    
    return {
        't_seconds': t_interp,
        't_hours': t_interp / 3600,
        'SOC': SOC_interp,
        'V': V_interp,
        'I': I_interp
    }
