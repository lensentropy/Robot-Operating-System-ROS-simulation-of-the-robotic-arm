#!/usr/bin/env python3
"""
24-Hour Smartphone Battery Simulation Based on Coupled Differential Equations
基于耦合微分方程组的24小时智能手机电池仿真

This script implements the full coupled ODE system:

1. SOC Dynamics (电化学方程):
   dSOC/dt = -I_total(t) / (Q_max · f(T_batt, N))

2. Battery Thermal Dynamics (电池热动力学):
   C_th · dT_batt/dt = P_joule + P_entropy - (T_batt - T_env)/R_th

3. CPU Thermal Dynamics (CPU热动力学):
   C_th,cpu · dT_cpu/dt = P_cpu - (T_cpu - T_batt)/R_th,cpu-batt - (T_cpu - T_env)/R_th,cpu-env

4. GNSS Lock State (GNSS锁定状态):
   dx_lock/dt = (1/τ_react) · (S(t) - x_lock)

5. Background Current (后台电流 - Ornstein-Uhlenbeck过程):
   dI_bg = θ·(μ - I_bg)dt + σ·dW

Total Power from Modules:
   P_total = P_cpu + P_display + P_network + P_bluetooth + P_gnss + P_background

Current Coupling (能量守恒):
   I_total = P_total / (η_PMIC · V_batt)

Voltage Model:
   V_batt = V_OCV(SOC) - I_total · R_int(SOC, T, N)
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery_model import BatteryParameters

# Plotting style
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 11,
    'axes.labelsize': 13,
    'axes.titlesize': 18,
    'legend.fontsize': 10,
    'figure.dpi': 150,
})


class CoupledBatteryODE:
    """
    Coupled ODE System for Battery Simulation
    电池耦合微分方程组
    
    State vector: y = [SOC, T_batt, T_cpu, x_lock, I_bg]
    """
    
    def __init__(self):
        # Battery electrochemical parameters
        self.Q_max = 14400.0      # As (4000 mAh)
        self.V_nom = 3.7          # V
        self.V_max = 4.2          # V
        self.V_min = 3.0          # V
        self.R_0 = 0.05           # Ω base internal resistance
        self.eta_PMIC = 0.92      # PMIC efficiency
        
        # Thermal parameters
        self.C_th_batt = 50.0     # J/K battery thermal capacitance
        self.R_th_batt = 10.0     # K/W battery thermal resistance
        self.C_th_cpu = 2.0       # J/K CPU thermal capacitance
        self.R_th_cpu_batt = 5.0  # K/W CPU-battery thermal resistance
        self.R_th_cpu_env = 15.0  # K/W CPU-environment thermal resistance
        
        # Entropy coefficient
        self.dVdT = -0.0002       # V/K
        
        # GNSS parameters
        self.tau_lock = 30.0      # s time to lock
        self.tau_unlock = 10.0    # s time to lose lock
        
        # Background O-U process parameters
        self.theta_bg = 0.1       # mean reversion rate
        self.mu_bg = 0.04         # A, mean background current
        self.sigma_bg = 0.01      # volatility
        
        # Aging
        self.N_cycles = 100       # cycle count
        self.alpha_aging = 0.001  # capacity fade per cycle
        
    def V_OCV(self, SOC):
        """Open Circuit Voltage model: V_OCV(SOC)"""
        SOC = np.clip(SOC, 0.01, 0.99)
        # Characteristic Li-ion OCV curve
        V = (self.V_min + 
             (self.V_max - self.V_min) * (
                 0.1 * SOC + 
                 0.8 * (1 - np.exp(-5 * SOC)) +
                 0.1 * SOC**2
             ) * (1 - 0.1 * (1 - SOC)**8))
        return np.clip(V, self.V_min, self.V_max)
    
    def R_int(self, SOC, T_batt):
        """
        Internal resistance model: R_int(SOC, T, N)
        R_int = R_0 · f(SOC) · g(T) · h(N)
        """
        SOC = np.clip(SOC, 0.01, 0.99)
        
        # f(SOC): U-shaped dependence
        f_SOC = 1.0 + 0.3 * (SOC - 0.5)**2 + 0.5 * np.exp(-10 * SOC)
        
        # g(T): Arrhenius temperature dependence
        E_a = 20000.0  # J/mol
        R_gas = 8.314  # J/(mol·K)
        T_ref = 298.15 # K
        g_T = np.exp(E_a / R_gas * (1/T_batt - 1/T_ref))
        
        # h(N): Aging factor
        h_N = 1.0 + self.alpha_aging * self.N_cycles
        
        return self.R_0 * f_SOC * g_T * h_N
    
    def capacity_factor(self, T_batt):
        """Temperature-dependent capacity factor"""
        # Cold temperature reduces effective capacity
        T_celsius = T_batt - 273.15
        if T_celsius < 20:
            factor = 1.0 - 0.01 * (20 - T_celsius)
        elif T_celsius > 40:
            factor = 1.0 - 0.005 * (T_celsius - 40)
        else:
            factor = 1.0
        # Aging factor
        factor *= (1.0 - self.alpha_aging * self.N_cycles)
        return max(0.7, factor)
    
    def compute_module_powers(self, t, usage):
        """
        Compute power consumption for each module
        计算各模块功耗
        
        P_total = P_cpu + P_display + P_network + P_bluetooth + P_gnss + P_background
        """
        powers = {}
        
        # 1. CPU/SoC Power (DVFS model)
        # P_cpu = C_eff · V_dd² · f + P_leak(T)
        cpu_load = usage['cpu_load']
        T_cpu = usage.get('T_cpu', 308.15)
        
        # DVFS: frequency and voltage scale with load
        f_cpu = 0.3 + 2.5 * cpu_load  # GHz
        V_dd = 0.65 + 0.5 * cpu_load  # V
        C_eff = 0.5e-9  # F
        
        P_dyn = C_eff * V_dd**2 * f_cpu * 1e9 * cpu_load
        P_leak = 0.2 * (1 + 0.03 * (T_cpu - 298.15))
        powers['P_cpu'] = P_dyn + P_leak
        
        # 2. Display Power (brightness and content coupling)
        # P_disp = P_static + k_drv · f_refresh + L · A · ξ(content)
        if usage['screen_on']:
            brightness = usage['brightness']
            content_brightness = usage.get('content_brightness', 0.5)
            refresh_rate = usage.get('refresh_rate', 60)
            
            P_static = 0.1
            P_driver = 1e-5 * refresh_rate
            P_panel = 0.003 * brightness * 1000 * (0.3 + 0.7 * content_brightness)
            powers['P_display'] = P_static + P_driver + P_panel
        else:
            powers['P_display'] = 0.0
        
        # 3. Network Power (path loss coupling)
        # P_5G = P_idle + P_data(R) + P_tx · (d/d_ref)^α · (1/S)
        if usage['network_active']:
            data_rate = usage['data_rate']
            signal_quality = usage['signal_quality']
            
            P_idle = 0.05
            P_data = 0.01 * data_rate
            P_tx = 0.5 * (1.0 / max(signal_quality, 0.1)) * 0.1
            powers['P_network'] = P_idle + P_data + P_tx
        else:
            powers['P_network'] = 0.05  # Idle
        
        # 4. Bluetooth Power (event-driven)
        if usage.get('bluetooth_audio', False):
            powers['P_bluetooth'] = 0.05
        elif usage.get('bluetooth_active', False):
            powers['P_bluetooth'] = 0.02
        else:
            powers['P_bluetooth'] = 0.001
        
        # 5. GNSS Power (signal coupling)
        # P_GNSS = P_LNA + x_lock · P_track + (1-x_lock) · P_acq
        if usage.get('gps_active', False):
            x_lock = usage.get('x_lock', 0.5)
            signal = usage.get('gps_signal', 0.8)
            P_LNA = 0.02
            P_track = 0.08 * x_lock
            P_acq = 0.15 * (1 - x_lock) * signal
            powers['P_gnss'] = P_LNA + P_track + P_acq
        else:
            powers['P_gnss'] = 0.0
        
        # 6. Background Power
        I_bg = usage.get('I_bg', 0.04)
        powers['P_background'] = self.V_nom * I_bg
        
        powers['P_total'] = sum(powers.values())
        return powers
    
    def coupled_ode(self, t, y, usage_func, charging_func, noise_func):
        """
        Main coupled ODE system
        主耦合微分方程组
        
        dy/dt = f(t, y)
        
        y = [SOC, T_batt, T_cpu, x_lock, I_bg]
        """
        SOC, T_batt, T_cpu, x_lock, I_bg = y
        
        # Clip states to physical bounds
        SOC = np.clip(SOC, 0.01, 0.99)
        T_batt = np.clip(T_batt, 273.15, 373.15)
        T_cpu = np.clip(T_cpu, 273.15, 383.15)
        x_lock = np.clip(x_lock, 0, 1)
        I_bg = np.clip(I_bg, 0.001, 0.5)
        
        # Get usage parameters
        usage = usage_func(t)
        usage['T_cpu'] = T_cpu
        usage['x_lock'] = x_lock
        usage['I_bg'] = I_bg
        
        # Check if charging
        is_charging = charging_func(t)
        T_env = usage.get('T_env', 298.15)
        
        # Compute module powers
        powers = self.compute_module_powers(t, usage)
        P_total = powers['P_total']
        P_cpu = powers['P_cpu']
        
        # Compute battery voltage and current
        V_OCV = self.V_OCV(SOC)
        R_int = self.R_int(SOC, T_batt)
        
        if is_charging:
            # Charging current (CC-CV profile) - negative means charging
            if SOC < 0.8:
                I_total = -2.0  # CC phase (fast charge)
            elif SOC < 0.95:
                # CV taper: reduce current as SOC increases
                I_total = -2.0 * (0.95 - SOC) / 0.15
            else:
                # Trickle charge at high SOC
                I_total = -0.1
            # Limit charging at near full
            if SOC > 0.98:
                I_total = 0  # Stop charging
            V_batt = V_OCV - I_total * R_int
        else:
            # Discharge: I = P / (η · V)
            V_batt = V_OCV - 0.5 * R_int  # Initial estimate
            I_total = P_total / (self.eta_PMIC * max(V_batt, self.V_min))
            # Refine
            V_batt = V_OCV - I_total * R_int
            I_total = P_total / (self.eta_PMIC * max(V_batt, self.V_min))
            # Limit discharge at low SOC
            if SOC < 0.05:
                I_total = 0  # Cutoff
        
        # ========== DIFFERENTIAL EQUATIONS ==========
        
        # 1. dSOC/dt = -I_total / (Q_max · f(T,N))
        Q_eff = self.Q_max * self.capacity_factor(T_batt)
        dSOC_dt = -I_total / Q_eff
        
        # Apply SOC limits to prevent out-of-bounds
        if SOC >= 0.99 and dSOC_dt > 0:
            dSOC_dt = 0  # Stop charging at max
        if SOC <= 0.05 and dSOC_dt < 0:
            dSOC_dt = 0  # Stop discharging at cutoff
        
        # 2. dT_batt/dt = (P_joule + P_entropy - Q_diss) / C_th
        P_joule = I_total**2 * R_int
        P_entropy = abs(I_total) * T_batt * abs(self.dVdT)
        Q_diss = (T_batt - T_env) / self.R_th_batt
        dT_batt_dt = (P_joule + P_entropy - Q_diss) / self.C_th_batt
        
        # 3. dT_cpu/dt = (P_cpu - Q_cpu_batt - Q_cpu_env) / C_th_cpu
        Q_cpu_batt = (T_cpu - T_batt) / self.R_th_cpu_batt
        Q_cpu_env = (T_cpu - T_env) / self.R_th_cpu_env
        dT_cpu_dt = (P_cpu - Q_cpu_batt - Q_cpu_env) / self.C_th_cpu
        
        # 4. dx_lock/dt = (1/τ) · (S - x_lock)
        if usage.get('gps_active', False):
            S_signal = usage.get('gps_signal', 0.8)
            tau = self.tau_lock if S_signal > 0.5 else self.tau_unlock
            dx_lock_dt = (S_signal - x_lock) / tau
        else:
            dx_lock_dt = -x_lock / self.tau_unlock
        
        # 5. dI_bg/dt = θ(μ - I_bg) + σ·noise (Ornstein-Uhlenbeck)
        user_activity = 1.0 if usage['screen_on'] else 0.3
        mu = self.mu_bg * user_activity
        noise = noise_func(t)
        dI_bg_dt = self.theta_bg * (mu - I_bg) + self.sigma_bg * noise
        
        return [dSOC_dt, dT_batt_dt, dT_cpu_dt, dx_lock_dt, dI_bg_dt]


def create_24hour_usage_scenario():
    """
    Create 24-hour usage scenario function
    创建24小时使用场景函数
    """
    def get_usage(t):
        """Return usage parameters at time t (in seconds)"""
        h = (t / 3600) % 24  # Convert to hours
        
        usage = {
            'cpu_load': 0.02,
            'screen_on': False,
            'brightness': 0,
            'content_brightness': 0.5,
            'refresh_rate': 60,
            'network_active': True,
            'data_rate': 0.1,
            'signal_quality': 0.8,
            'bluetooth_active': False,
            'bluetooth_audio': False,
            'gps_active': False,
            'gps_signal': 0.8,
            'T_env': 298.15,
            'activity': 'Sleep',
        }
        
        # 0:00 - 2:00 Sleep
        if 0 <= h < 2:
            usage['activity'] = 'Sleep'
            usage['cpu_load'] = 0.02
            usage['T_env'] = 295.15  # Indoor
            
        # 2:00 - 3:00 Charging
        elif 2 <= h < 3:
            usage['activity'] = 'Charging'
            usage['cpu_load'] = 0.05
            usage['T_env'] = 295.15
            
        # 3:00 - 7:00 Sleep
        elif 3 <= h < 7:
            usage['activity'] = 'Sleep'
            usage['cpu_load'] = 0.02
            usage['T_env'] = 295.15
            
        # 7:00 - 8:30 Commute
        elif 7 <= h < 8.5:
            usage['activity'] = 'Commute'
            usage['cpu_load'] = 0.4 + 0.1 * np.sin(h * 3)
            usage['screen_on'] = True
            usage['brightness'] = 0.6
            usage['data_rate'] = 15 + 5 * np.sin(h * 2)
            usage['signal_quality'] = 0.5 + 0.3 * np.sin(h * 5)
            usage['gps_active'] = True
            usage['gps_signal'] = 0.6 + 0.2 * np.sin(h * 4)
            usage['T_env'] = 298.15 + 5 * np.sin(h)
            
        # 8:30 - 10:00 Office
        elif 8.5 <= h < 10:
            usage['activity'] = 'Office'
            usage['cpu_load'] = 0.3
            usage['screen_on'] = True
            usage['brightness'] = 0.5
            usage['data_rate'] = 5
            usage['signal_quality'] = 0.9
            usage['T_env'] = 296.15
            
        # 10:00 - 12:00 Office (meetings)
        elif 10 <= h < 12:
            usage['activity'] = 'Office'
            usage['cpu_load'] = 0.25
            usage['screen_on'] = h % 1 < 0.5  # Intermittent screen
            usage['brightness'] = 0.5
            usage['data_rate'] = 3
            usage['signal_quality'] = 0.9
            usage['T_env'] = 296.15
            
        # 12:00 - 13:30 Entertainment
        elif 12 <= h < 13.5:
            usage['activity'] = 'Entertainment'
            usage['cpu_load'] = 0.6 + 0.15 * np.sin(h * 4)
            usage['screen_on'] = True
            usage['brightness'] = 0.75
            usage['content_brightness'] = 0.6
            usage['refresh_rate'] = 90
            usage['data_rate'] = 25 + 10 * np.sin(h * 3)
            usage['signal_quality'] = 0.85
            usage['bluetooth_audio'] = True
            usage['T_env'] = 296.15
            
        # 13:30 - 14:00 Charging
        elif 13.5 <= h < 14:
            usage['activity'] = 'Charging'
            usage['cpu_load'] = 0.2
            usage['screen_on'] = True
            usage['brightness'] = 0.4
            usage['data_rate'] = 2
            usage['T_env'] = 296.15
            
        # 14:00 - 17:30 Office
        elif 14 <= h < 17.5:
            usage['activity'] = 'Office'
            usage['cpu_load'] = 0.35 + 0.1 * np.sin(h)
            usage['screen_on'] = True
            usage['brightness'] = 0.5
            usage['data_rate'] = 8 + 3 * np.sin(h * 2)
            usage['signal_quality'] = 0.9
            usage['T_env'] = 296.15
            
        # 17:30 - 18:30 Commute
        elif 17.5 <= h < 18.5:
            usage['activity'] = 'Commute'
            usage['cpu_load'] = 0.45
            usage['screen_on'] = True
            usage['brightness'] = 0.65
            usage['data_rate'] = 20
            usage['signal_quality'] = 0.4 + 0.3 * np.sin(h * 6)
            usage['gps_active'] = True
            usage['gps_signal'] = 0.5 + 0.3 * np.sin(h * 5)
            usage['T_env'] = 300.15
            
        # 18:30 - 21:00 Entertainment
        elif 18.5 <= h < 21:
            usage['activity'] = 'Entertainment'
            usage['cpu_load'] = 0.7 + 0.2 * np.sin(h * 2)
            usage['screen_on'] = True
            usage['brightness'] = 0.75
            usage['content_brightness'] = 0.7
            usage['refresh_rate'] = 120
            usage['data_rate'] = 30
            usage['signal_quality'] = 0.85
            usage['bluetooth_audio'] = True
            usage['T_env'] = 295.15
            
        # 21:00 - 22:00 Light use
        elif 21 <= h < 22:
            usage['activity'] = 'Office'
            usage['cpu_load'] = 0.25
            usage['screen_on'] = True
            usage['brightness'] = 0.4
            usage['data_rate'] = 5
            usage['signal_quality'] = 0.9
            usage['T_env'] = 295.15
            
        # 22:00 - 23:00 Charging
        elif 22 <= h < 23:
            usage['activity'] = 'Charging'
            usage['cpu_load'] = 0.15
            usage['screen_on'] = True
            usage['brightness'] = 0.3
            usage['data_rate'] = 2
            usage['T_env'] = 295.15
            
        # 23:00 - 24:00 Sleep
        else:
            usage['activity'] = 'Sleep'
            usage['cpu_load'] = 0.02
            usage['T_env'] = 295.15
        
        return usage
    
    return get_usage


def create_charging_function():
    """Create function that returns True during charging periods"""
    def is_charging(t):
        h = (t / 3600) % 24
        return (2 <= h < 3) or (13.5 <= h < 14) or (22 <= h < 23)
    return is_charging


def run_coupled_simulation():
    """
    Run the coupled ODE simulation
    运行耦合微分方程组仿真
    """
    print("="*70)
    print("24-Hour Battery Simulation using Coupled Differential Equations")
    print("基于耦合微分方程组的24小时电池仿真")
    print("="*70)
    
    # Initialize ODE system
    ode_system = CoupledBatteryODE()
    
    # Create scenario functions
    usage_func = create_24hour_usage_scenario()
    charging_func = create_charging_function()
    
    # Noise function for O-U process
    np.random.seed(42)
    noise_cache = {}
    def noise_func(t):
        t_idx = int(t / 60)  # Cache per minute
        if t_idx not in noise_cache:
            noise_cache[t_idx] = np.random.normal(0, 1)
        return noise_cache[t_idx]
    
    # Initial state: [SOC, T_batt, T_cpu, x_lock, I_bg]
    y0 = [0.85, 295.15, 300.15, 0.0, 0.04]
    
    # Time span (24 hours in seconds)
    t_span = (0, 24 * 3600)
    t_eval = np.linspace(0, 24 * 3600, 1441)  # 1 minute resolution
    
    print("\nSolving coupled ODE system...")
    print("  State vector: y = [SOC, T_batt, T_cpu, x_lock, I_bg]")
    print("  Equations:")
    print("    dSOC/dt = -I_total / (Q_max · f(T,N))")
    print("    dT_batt/dt = (P_joule + P_entropy - Q_diss) / C_th")
    print("    dT_cpu/dt = (P_cpu - Q_cpu_batt - Q_cpu_env) / C_th_cpu")
    print("    dx_lock/dt = (1/τ) · (S - x_lock)")
    print("    dI_bg/dt = θ(μ - I_bg) + σ·dW")
    
    # Define ODE with state clipping wrapper
    def ode_with_bounds(t, y):
        # Clip states before passing to ODE
        y_clipped = np.array([
            np.clip(y[0], 0.01, 0.99),   # SOC
            np.clip(y[1], 273.15, 373.15),  # T_batt
            np.clip(y[2], 273.15, 383.15),  # T_cpu
            np.clip(y[3], 0, 1),          # x_lock
            np.clip(y[4], 0.001, 0.5)     # I_bg
        ])
        return ode_system.coupled_ode(t, y_clipped, usage_func, charging_func, noise_func)
    
    # Solve ODE using RK45 with adaptive stepping
    solution = solve_ivp(
        ode_with_bounds,
        t_span,
        y0,
        method='RK45',
        t_eval=t_eval,
        max_step=30,  # Max 30 second steps for accuracy
        rtol=1e-6,
        atol=1e-8,
        dense_output=True
    )
    
    # Post-process: clip solution to physical bounds
    solution.y[0] = np.clip(solution.y[0], 0.05, 1.0)  # SOC
    solution.y[1] = np.clip(solution.y[1], 273.15, 373.15)  # T_batt
    solution.y[2] = np.clip(solution.y[2], 273.15, 383.15)  # T_cpu
    solution.y[3] = np.clip(solution.y[3], 0, 1)  # x_lock
    solution.y[4] = np.clip(solution.y[4], 0, 0.5)  # I_bg
    
    print(f"\nODE solver status: {'Success' if solution.success else 'Failed'}")
    
    # Extract results
    t_hours = solution.t / 3600
    SOC = solution.y[0] * 100
    T_batt = solution.y[1] - 273.15  # Convert to Celsius
    T_cpu = solution.y[2] - 273.15
    x_lock = solution.y[3]
    I_bg = solution.y[4]
    
    # Compute additional variables
    current = np.zeros_like(t_hours)
    power_total = np.zeros_like(t_hours)
    brightness = np.zeros_like(t_hours)
    signal_quality = np.zeros_like(t_hours)
    activity = []
    
    for i, t in enumerate(solution.t):
        usage = usage_func(t)
        activity.append(usage['activity'])
        brightness[i] = usage['brightness'] * 100 if usage['screen_on'] else 0
        signal_quality[i] = usage['signal_quality']
        
        # Recompute current
        is_charging = charging_func(t)
        powers = ode_system.compute_module_powers(t, {
            **usage,
            'T_cpu': solution.y[2, i],
            'x_lock': solution.y[3, i],
            'I_bg': solution.y[4, i]
        })
        power_total[i] = powers['P_total']
        
        if is_charging:
            if SOC[i] < 80:
                current[i] = -2.5
            elif SOC[i] < 95:
                current[i] = -2.5 * (95 - SOC[i]) / 15
            else:
                current[i] = -0.1
        else:
            V_batt = ode_system.V_OCV(solution.y[0, i])
            current[i] = powers['P_total'] / (ode_system.eta_PMIC * V_batt)
    
    # Print statistics
    print("\n--- Simulation Results ---")
    print(f"Initial SOC: {SOC[0]:.1f}%")
    print(f"Minimum SOC: {np.min(SOC):.1f}%")
    print(f"Maximum SOC: {np.max(SOC):.1f}%")
    print(f"Final SOC: {SOC[-1]:.1f}%")
    print(f"Max Battery Temperature: {np.max(T_batt):.1f}°C")
    print(f"Max CPU Temperature: {np.max(T_cpu):.1f}°C")
    print(f"Max Discharge Current: {np.max(current):.2f}A")
    print(f"Max Charge Current: {np.min(current):.2f}A")
    print(f"Max Power Consumption: {np.max(power_total):.2f}W")
    
    return {
        't_hours': t_hours,
        'SOC': SOC,
        'T_batt': T_batt,
        'T_cpu': T_cpu,
        'x_lock': x_lock,
        'I_bg': I_bg,
        'current': current,
        'power_total': power_total,
        'brightness': brightness,
        'signal_quality': signal_quality,
        'activity': activity
    }


def plot_coupled_simulation(data, save_path=None):
    """
    Create comprehensive visualization of coupled simulation
    创建耦合仿真的综合可视化
    """
    fig, ax1 = plt.subplots(figsize=(18, 10))
    
    t = data['t_hours']
    
    # Activity colors
    activity_colors = {
        'Sleep': '#D6EAF8',
        'Charging': '#FDEBD0',
        'Commute': '#D5F5E3',
        'Office': '#FADBD8',
        'Entertainment': '#FEF9E7',
    }
    
    # Draw activity regions
    current_activity = data['activity'][0]
    start_idx = 0
    
    for i in range(len(t)):
        if i == len(t) - 1 or data['activity'][i] != current_activity:
            end_hour = t[i] if i < len(t) - 1 else 24
            start_hour = t[start_idx]
            width = end_hour - start_hour
            
            if width > 0.05:
                rect = Rectangle((start_hour, -10), width, 120,
                                facecolor=activity_colors.get(current_activity, '#F5F5F5'),
                                edgecolor='none', alpha=0.75, zorder=0)
                ax1.add_patch(rect)
                
                mid_x = start_hour + width / 2
                ax1.text(mid_x, 102, current_activity,
                        ha='center', va='bottom', fontsize=11, fontweight='bold',
                        color='#2C3E50', zorder=15)
            
            if i < len(t) - 1:
                current_activity = data['activity'][i]
                start_idx = i
    
    # Primary Y-axis
    ax1.set_xlabel('Time (hours)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('SOC (%) / Current (A)', fontsize=13, color='#D35400', fontweight='bold')
    
    # Plot SOC
    line_soc, = ax1.plot(t, data['SOC'], color='#E67E22', linewidth=3.5,
                         label='SOC [%]', zorder=10)
    
    # Plot Current (scaled)
    current_scaled = data['current'] * 30
    line_current, = ax1.plot(t, current_scaled, color='#3498DB', linewidth=2,
                             label='Current I [A×30]', zorder=9)
    
    # Plot Battery Temperature
    line_temp, = ax1.plot(t, data['T_batt'], color='#8E44AD', linewidth=2.5,
                          label='Temperature T [°C]', zorder=9)
    
    ax1.set_ylim(-15, 110)
    ax1.set_xlim(0, 24)
    ax1.tick_params(axis='y', labelcolor='#D35400', labelsize=11)
    
    # Secondary Y-axis
    ax2 = ax1.twinx()
    ax2.set_ylabel('Temperature (°C) / Brightness (%) / Signal [dBm]',
                   fontsize=13, color='#27AE60', fontweight='bold')
    
    # Plot Brightness
    line_bright, = ax2.step(t, data['brightness'], where='post',
                            color='#9ACD32', linewidth=2.5,
                            label='Brightness [%]', zorder=8)
    
    # Plot Signal (shifted)
    signal_shifted = data['signal_quality'] * 50 + 30  # Scale to 30-80 range
    line_signal, = ax2.step(t, signal_shifted, where='post',
                            color='#2ECC71', linewidth=2,
                            label='Signal Strength [dBm]', zorder=8)
    
    ax2.set_ylim(0, 120)
    ax2.tick_params(axis='y', labelcolor='#27AE60', labelsize=11)
    
    # Charging annotations
    charging_periods = [(2, 3), (13.5, 14), (22, 23)]
    for start, end in charging_periods:
        mid = (start + end) / 2
        ax1.annotate('Charging', xy=(mid, 15),
                    ha='center', va='center', fontsize=10,
                    fontweight='bold', color='#27AE60',
                    bbox=dict(boxstyle='round,pad=0.4',
                             facecolor='#ABEBC6', edgecolor='#27AE60',
                             linewidth=1.5, alpha=0.9),
                    zorder=20)
    
    # Title with equation reference
    title_text = '24-Hour Smartphone Battery Coupled Model Simulation\n'
    title_text += r'$\frac{dSOC}{dt} = -\frac{I_{total}}{Q_{max} \cdot f(T,N)}$, '
    title_text += r'$C_{th}\frac{dT}{dt} = P_{joule} + P_{entropy} - \frac{T-T_{env}}{R_{th}}$'
    ax1.set_title(title_text, fontsize=16, fontweight='bold', pad=20, color='#2C3E50')
    
    # X-axis
    ax1.set_xticks(range(0, 25, 3))
    ax1.set_xticklabels([f'{h}:00' for h in range(0, 25, 3)], fontsize=11)
    ax1.grid(True, alpha=0.4, linestyle='-', linewidth=0.5, zorder=1)
    
    # Legend
    legend_elements = [
        Line2D([0], [0], color='#E67E22', linewidth=3.5, label='SOC [%]'),
        Line2D([0], [0], color='#3498DB', linewidth=2, label='Current I [A×30]'),
        Line2D([0], [0], color='#8E44AD', linewidth=2.5, label='Temperature T [°C]'),
        Line2D([0], [0], color='#9ACD32', linewidth=2.5, label='Brightness [%]'),
        Line2D([0], [0], color='#2ECC71', linewidth=2, label='Signal Strength [dBm]'),
    ]
    
    legend = ax1.legend(handles=legend_elements, loc='upper right',
                       bbox_to_anchor=(0.995, 0.95), framealpha=0.95,
                       fontsize=10, ncol=2, columnspacing=1.5)
    legend.get_frame().set_edgecolor('#BDC3C7')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"\nFigure saved to: {save_path}")
    
    return fig


def main():
    """Main execution"""
    # Run simulation
    data = run_coupled_simulation()
    
    # Create plot
    print("\nGenerating visualization...")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, '24hour_coupled_ode_simulation.png')
    fig = plot_coupled_simulation(data, save_path)
    
    plt.close(fig)
    print("\nSimulation complete!")
    
    return data


if __name__ == "__main__":
    data = main()
