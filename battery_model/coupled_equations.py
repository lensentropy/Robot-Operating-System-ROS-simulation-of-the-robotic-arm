"""
Coupled Differential Equations for Battery Model
电池耦合微分方程组

This module implements the core mathematical model based on:
- Battery electrochemical-thermal coupling
- Multi-component power consumption
- User behavior dynamics

The main coupled system:
    dSOC/dt = -I_total(t) / (Q_max * f(T_batt, N))
    dT_batt/dt = (P_joule + P_entropy - (T_batt - T_env)/R_th) / C_th
    dT_cpu/dt = (P_cpu - (T_cpu - T_batt)/R_th_cpu_batt - (T_cpu - T_env)/R_th_cpu_env) / C_th_cpu
    dx_lock/dt = (1/τ_react) * (S(t) - x_lock)
    dI_bg/dt = θ * (μ - I_bg) + σ * dW/dt  (Ornstein-Uhlenbeck)
"""

import numpy as np
from scipy.integrate import solve_ivp, odeint
from scipy.interpolate import interp1d
from typing import Tuple, Dict, List, Optional, Callable
from dataclasses import dataclass
import warnings

from .parameters import (
    BatteryParameters, CPUParameters, DisplayParameters,
    NetworkParameters, BluetoothParameters, GNSSParameters,
    BackgroundParameters, UsageScenarios, UserBehaviorModel, UserState
)


@dataclass
class SystemState:
    """System state vector for coupled equations"""
    SOC: float           # State of Charge [0, 1]
    T_batt: float        # Battery temperature [K]
    T_cpu: float         # CPU temperature [K]  
    x_lock: float        # GNSS lock state [0, 1]
    I_bg: float          # Background current [A]
    user_state: int      # User state index
    
    def to_array(self) -> np.ndarray:
        return np.array([self.SOC, self.T_batt, self.T_cpu, self.x_lock, self.I_bg, self.user_state])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'SystemState':
        return cls(
            SOC=arr[0], T_batt=arr[1], T_cpu=arr[2],
            x_lock=arr[3], I_bg=arr[4], user_state=int(arr[5])
        )


class BatteryCoupledModel:
    """
    Main coupled battery model class
    主要耦合电池模型类
    
    Implements the full system of coupled differential equations for
    smartphone battery discharge modeling.
    """
    
    def __init__(self, 
                 battery_params: Optional[BatteryParameters] = None,
                 cpu_params: Optional[CPUParameters] = None,
                 display_params: Optional[DisplayParameters] = None,
                 network_params: Optional[NetworkParameters] = None,
                 bt_params: Optional[BluetoothParameters] = None,
                 gnss_params: Optional[GNSSParameters] = None,
                 bg_params: Optional[BackgroundParameters] = None):
        """Initialize model with component parameters"""
        
        self.battery = battery_params or BatteryParameters()
        self.cpu = cpu_params or CPUParameters()
        self.display = display_params or DisplayParameters()
        self.network = network_params or NetworkParameters()
        self.bluetooth = bt_params or BluetoothParameters()
        self.gnss = gnss_params or GNSSParameters()
        self.background = bg_params or BackgroundParameters()
        self.user_model = UserBehaviorModel()
        
        # Simulation settings
        self.dt = 1.0  # Default time step for stochastic components
        self.cycle_count = 0  # Battery cycle count
        
        # State history for analysis
        self.state_history = []
        self.power_history = []
        self.time_history = []
    
    def compute_V_batt(self, SOC: float, I_total: float, T_batt: float) -> float:
        """
        计算电池端电压
        V_batt = V_OCV(SOC) - I_total * R_int(SOC, T_batt, N)
        """
        V_OCV = self.battery.V_OCV(np.array([SOC]))[0]
        R_int = self.battery.R_int(np.array([SOC]), T_batt, self.cycle_count)[0]
        
        V_batt = V_OCV - I_total * R_int
        return max(V_batt, self.battery.V_min)
    
    def compute_power_components(self, state: SystemState, 
                                  usage: Dict,
                                  t: float = 0) -> Dict[str, float]:
        """
        计算各模块功耗
        
        Returns dictionary with power consumption of each component in Watts
        """
        powers = {}
        
        # 1. CPU/SoC power (电热强耦合)
        cpu_load = usage.get('cpu_load', 0.2)
        P_cpu, freq_idx = self.cpu.power_consumption(cpu_load, state.T_cpu)
        powers['P_cpu'] = P_cpu
        
        # 2. Display power (环境光内容耦合)
        if usage.get('screen_on', True):
            brightness = usage.get('brightness', 0.5)
            content_brightness = usage.get('content_brightness', 0.5)
            refresh_idx = usage.get('refresh_idx', 1)
            P_disp = self.display.power_consumption(brightness, refresh_idx, content_brightness)
        else:
            P_disp = 0.0
        powers['P_display'] = P_disp
        
        # 3. Network power (信道距离耦合)
        if usage.get('network_active', True):
            data_rate = usage.get('data_rate_Mbps', 1.0)
            distance = usage.get('cell_distance_m', 200)
            signal = usage.get('signal_quality', 0.8)
            P_net = self.network.power_5G(data_rate, distance, signal)
        else:
            P_net = self.network.P_5G_idle
        powers['P_network'] = P_net
        
        # 4. Bluetooth power (事件驱动耦合)
        bt_audio = usage.get('bluetooth_audio', False)
        bt_data = usage.get('bluetooth_data_kbps', 0)
        P_bt = self.bluetooth.power_consumption(bt_audio, bt_data)
        powers['P_bluetooth'] = P_bt
        
        # 5. GNSS power (环境信号耦合)
        if usage.get('gps_active', False):
            signal_quality = usage.get('gps_signal', 0.8)
            P_gnss = self.gnss.power_consumption(state.x_lock, signal_quality)
        else:
            P_gnss = self.gnss.P_off
        powers['P_gnss'] = P_gnss
        
        # 6. Background power (随机过程耦合)
        # Using current I_bg state value
        V_nom = self.battery.V_nom
        P_bg = V_nom * state.I_bg
        powers['P_background'] = P_bg
        
        # Total power
        powers['P_total'] = sum(powers.values())
        
        return powers
    
    def compute_total_current(self, P_total: float, V_batt: float) -> float:
        """
        计算总放电电流 (能量守恒)
        I_total = P_total / (η_PMIC * V_load * V_batt)
        
        Assumes V_load ≈ V_batt for simplicity
        """
        eta = self.battery.eta_PMIC
        I_total = P_total / (eta * V_batt)
        return I_total
    
    def compute_heat_generation(self, I_total: float, SOC: float, 
                                 T_batt: float) -> Tuple[float, float]:
        """
        计算电池发热
        P_joule = I²R (焦耳热)
        P_entropy = I * T * dV/dT (可逆熵热)
        """
        R_int = self.battery.R_int(np.array([SOC]), T_batt, self.cycle_count)[0]
        
        # Joule heating
        P_joule = I_total**2 * R_int
        
        # Reversible entropy heating
        P_entropy = abs(I_total) * T_batt * abs(self.battery.dVdT)
        
        return P_joule, P_entropy
    
    def dynamics(self, t: float, y: np.ndarray, 
                 usage_func: Callable[[float], Dict],
                 noise_bg: float = 0) -> np.ndarray:
        """
        主耦合微分方程组右端函数
        
        State vector y = [SOC, T_batt, T_cpu, x_lock, I_bg, user_state]
        
        Returns dy/dt
        """
        state = SystemState.from_array(y)
        usage = usage_func(t)
        
        # Clip states to valid ranges
        state.SOC = np.clip(state.SOC, 0.001, 1.0)
        state.T_batt = np.clip(state.T_batt, 273.15, 373.15)  # 0-100°C
        state.T_cpu = np.clip(state.T_cpu, 273.15, 383.15)    # 0-110°C
        state.x_lock = np.clip(state.x_lock, 0, 1)
        state.I_bg = np.clip(state.I_bg, 0, 0.5)
        
        # Compute powers
        powers = self.compute_power_components(state, usage, t)
        P_total = powers['P_total']
        P_cpu = powers['P_cpu']
        
        # Compute battery voltage and current
        # First estimate with nominal voltage
        V_batt_est = self.compute_V_batt(state.SOC, 0.5, state.T_batt)
        I_total = self.compute_total_current(P_total, V_batt_est)
        
        # Refine voltage with actual current
        V_batt = self.compute_V_batt(state.SOC, I_total, state.T_batt)
        I_total = self.compute_total_current(P_total, V_batt)
        
        # Compute heat generation
        P_joule, P_entropy = self.compute_heat_generation(I_total, state.SOC, state.T_batt)
        
        # ========== DIFFERENTIAL EQUATIONS ==========
        
        # 1. SOC dynamics: dSOC/dt = -I_total / (Q_max * f(T, N))
        Q_eff = self.battery.Q_max_As * self.battery.capacity_fade(self.cycle_count)
        # Temperature effect on capacity
        T_factor = 1.0 - 0.002 * max(0, 298.15 - state.T_batt)  # Cold reduces capacity
        Q_eff *= T_factor
        
        dSOC_dt = -I_total / Q_eff
        
        # 2. Battery temperature: dT_batt/dt = (P_joule + P_entropy - (T_batt - T_env)/R_th) / C_th
        T_env = usage.get('T_env', self.battery.T_env)
        heat_dissipation = (state.T_batt - T_env) / self.battery.R_th_batt
        dT_batt_dt = (P_joule + P_entropy - heat_dissipation) / self.battery.C_th_batt
        
        # 3. CPU temperature: dT_cpu/dt
        heat_cpu_batt = (state.T_cpu - state.T_batt) / self.cpu.R_th_cpu_batt
        heat_cpu_env = (state.T_cpu - T_env) / self.cpu.R_th_cpu_env
        dT_cpu_dt = (P_cpu - heat_cpu_batt - heat_cpu_env) / self.cpu.C_th_cpu
        
        # 4. GNSS lock state: dx_lock/dt = (1/τ) * (S(t) - x_lock)
        if usage.get('gps_active', False):
            S_signal = usage.get('gps_signal', 0.8)
            tau = self.gnss.tau_lock if S_signal > 0.5 else self.gnss.tau_unlock
            dx_lock_dt = (S_signal - state.x_lock) / tau
        else:
            dx_lock_dt = -state.x_lock / self.gnss.tau_unlock  # Decay to 0
        
        # 5. Background current (Ornstein-Uhlenbeck process approximation)
        # dI_bg/dt = θ(μ - I_bg) + σ*noise
        user_activity = 0.5 if usage.get('screen_on', True) else 0.1
        mu_bg = self.background.mean_power(user_activity) / self.battery.V_nom
        theta = self.background.theta_ou
        sigma = self.background.sigma_ou / self.battery.V_nom
        
        dI_bg_dt = theta * (mu_bg - state.I_bg) + sigma * noise_bg
        
        # 6. User state (handled separately in simulation for Markov process)
        d_user_state_dt = 0  # Placeholder, actual transitions handled discretely
        
        return np.array([dSOC_dt, dT_batt_dt, dT_cpu_dt, dx_lock_dt, dI_bg_dt, d_user_state_dt])
    
    def simulate(self, 
                 t_span: Tuple[float, float],
                 initial_state: Optional[SystemState] = None,
                 usage_func: Optional[Callable[[float], Dict]] = None,
                 scenario: str = 'light_use',
                 dt_output: float = 60.0,
                 include_stochastic: bool = True) -> Dict:
        """
        运行完整模拟
        
        Parameters:
        -----------
        t_span : tuple
            (t_start, t_end) in seconds
        initial_state : SystemState, optional
            Initial system state
        usage_func : callable, optional
            Function(t) -> usage_dict defining usage over time
        scenario : str
            Predefined scenario name if usage_func not provided
        dt_output : float
            Output time step in seconds
        include_stochastic : bool
            Whether to include stochastic background process
            
        Returns:
        --------
        dict with 't', 'states', 'powers', 'voltages', 'currents'
        """
        
        # Default initial state
        if initial_state is None:
            initial_state = SystemState(
                SOC=1.0,
                T_batt=298.15,  # 25°C
                T_cpu=303.15,   # 30°C
                x_lock=0.0,
                I_bg=0.04,      # 40mA background
                user_state=1   # WORK state
            )
        
        # Default usage function
        if usage_func is None:
            base_usage = UsageScenarios.get_scenario(scenario)
            usage_func = lambda t: base_usage
        
        # Time points
        t_start, t_end = t_span
        t_eval = np.arange(t_start, t_end, dt_output)
        
        # Storage for results
        results = {
            't': [],
            'SOC': [],
            'T_batt': [],
            'T_cpu': [],
            'x_lock': [],
            'I_bg': [],
            'V_batt': [],
            'I_total': [],
            'P_total': [],
            'P_cpu': [],
            'P_display': [],
            'P_network': [],
            'P_bluetooth': [],
            'P_gnss': [],
            'P_background': []
        }
        
        # Simulation with adaptive stepping and stochastic noise
        y = initial_state.to_array()
        t_current = t_start
        dt_sim = min(1.0, dt_output / 10)  # Internal simulation step
        
        np.random.seed(42)  # Reproducibility
        
        output_idx = 0
        while t_current < t_end and y[0] > 0.05:  # Stop at 5% SOC
            # Generate noise for stochastic component
            noise_bg = np.random.normal(0, 1) * np.sqrt(dt_sim) if include_stochastic else 0
            
            # RK4 step
            k1 = self.dynamics(t_current, y, usage_func, noise_bg)
            k2 = self.dynamics(t_current + dt_sim/2, y + dt_sim/2 * k1, usage_func, noise_bg)
            k3 = self.dynamics(t_current + dt_sim/2, y + dt_sim/2 * k2, usage_func, noise_bg)
            k4 = self.dynamics(t_current + dt_sim, y + dt_sim * k3, usage_func, noise_bg)
            
            y = y + (dt_sim/6) * (k1 + 2*k2 + 2*k3 + k4)
            
            # Clip states
            y[0] = np.clip(y[0], 0, 1)  # SOC
            y[1] = np.clip(y[1], 273.15, 373.15)  # T_batt
            y[2] = np.clip(y[2], 273.15, 383.15)  # T_cpu
            y[3] = np.clip(y[3], 0, 1)  # x_lock
            y[4] = np.clip(y[4], 0, 0.5)  # I_bg
            
            t_current += dt_sim
            
            # Record at output times
            if output_idx < len(t_eval) and t_current >= t_eval[output_idx]:
                state = SystemState.from_array(y)
                usage = usage_func(t_current)
                powers = self.compute_power_components(state, usage, t_current)
                
                V_batt = self.compute_V_batt(state.SOC, 0.5, state.T_batt)
                I_total = self.compute_total_current(powers['P_total'], V_batt)
                
                results['t'].append(t_current)
                results['SOC'].append(state.SOC)
                results['T_batt'].append(state.T_batt)
                results['T_cpu'].append(state.T_cpu)
                results['x_lock'].append(state.x_lock)
                results['I_bg'].append(state.I_bg)
                results['V_batt'].append(V_batt)
                results['I_total'].append(I_total)
                results['P_total'].append(powers['P_total'])
                results['P_cpu'].append(powers['P_cpu'])
                results['P_display'].append(powers['P_display'])
                results['P_network'].append(powers['P_network'])
                results['P_bluetooth'].append(powers['P_bluetooth'])
                results['P_gnss'].append(powers['P_gnss'])
                results['P_background'].append(powers['P_background'])
                
                output_idx += 1
        
        # Convert to numpy arrays
        for key in results:
            results[key] = np.array(results[key])
        
        # Compute derived quantities
        if len(results['t']) > 0:
            results['t_hours'] = results['t'] / 3600
            results['T_batt_C'] = results['T_batt'] - 273.15
            results['T_cpu_C'] = results['T_cpu'] - 273.15
            results['SOC_percent'] = results['SOC'] * 100
        
        return results
    
    def simulate_scenario(self, scenario_name: str, 
                          SOC_initial: float = 1.0,
                          max_hours: float = 24) -> Dict:
        """
        使用预定义场景进行模拟
        """
        initial_state = SystemState(
            SOC=SOC_initial,
            T_batt=298.15,
            T_cpu=303.15,
            x_lock=0.0,
            I_bg=0.04,
            user_state=1
        )
        
        return self.simulate(
            t_span=(0, max_hours * 3600),
            initial_state=initial_state,
            scenario=scenario_name,
            dt_output=60.0
        )
    
    def compute_sensitivity(self, base_scenario: str = 'light_use',
                           param_ranges: Optional[Dict] = None) -> Dict:
        """
        计算参数敏感性分析
        
        Varies each parameter and measures effect on discharge time
        """
        if param_ranges is None:
            param_ranges = {
                'cpu_load': [0.1, 0.3, 0.5, 0.7, 0.9],
                'brightness': [0.2, 0.4, 0.6, 0.8, 1.0],
                'data_rate_Mbps': [0, 5, 15, 30, 50],
                'T_env': [273.15, 288.15, 298.15, 308.15, 318.15]  # -0 to 45°C
            }
        
        results = {}
        base_usage = UsageScenarios.get_scenario(base_scenario)
        
        for param, values in param_ranges.items():
            discharge_times = []
            
            for val in values:
                # Create modified usage
                modified_usage = base_usage.copy()
                modified_usage[param] = val
                usage_func = lambda t, u=modified_usage: u
                
                # Run simulation
                sim_result = self.simulate(
                    t_span=(0, 24*3600),
                    usage_func=usage_func,
                    dt_output=300
                )
                
                # Find discharge time (to 5% SOC)
                if len(sim_result['t']) > 0:
                    t_discharge = sim_result['t'][-1] / 3600  # hours
                else:
                    t_discharge = 0
                
                discharge_times.append(t_discharge)
            
            results[param] = {
                'values': values,
                'discharge_times': discharge_times
            }
        
        return results


class ThermalCoupledModel:
    """
    Extended model with detailed thermal coupling
    扩展热耦合模型
    """
    
    def __init__(self, base_model: BatteryCoupledModel):
        self.base = base_model
        
        # Additional thermal nodes
        self.T_nodes = {
            'battery': 298.15,
            'cpu': 303.15,
            'display': 298.15,
            'modem': 298.15,
            'case': 298.15
        }
        
        # Thermal conductance matrix (W/K)
        self.G_thermal = np.array([
            [0.5, 0.2, 0.1, 0.1, 0.3],   # Battery
            [0.2, 0.3, 0.1, 0.05, 0.2],  # CPU
            [0.1, 0.1, 0.2, 0.05, 0.15], # Display
            [0.1, 0.05, 0.05, 0.15, 0.1],# Modem
            [0.3, 0.2, 0.15, 0.1, 1.0]   # Case (to environment)
        ])
    
    def thermal_dynamics(self, T_vec: np.ndarray, P_vec: np.ndarray,
                         T_env: float) -> np.ndarray:
        """
        Compute temperature derivatives for all nodes
        
        dT/dt = (P - G*(T - T_neighbors)) / C
        """
        # Thermal capacitances
        C_th = np.array([50, 2, 5, 3, 20])  # J/K
        
        # Heat generation in each node
        # P_vec = [P_batt, P_cpu, P_disp, P_modem, 0]
        
        # Heat flow to environment
        Q_env = self.G_thermal[:, 4] * (T_vec - T_env)
        
        # Internal heat flow (conduction between nodes)
        Q_internal = np.zeros_like(T_vec)
        for i in range(len(T_vec)):
            for j in range(len(T_vec)):
                if i != j:
                    Q_internal[i] += self.G_thermal[i, j] * (T_vec[i] - T_vec[j])
        
        dT_dt = (P_vec - Q_internal - Q_env) / C_th
        return dT_dt
