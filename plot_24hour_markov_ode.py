#!/usr/bin/env python3
"""
24-Hour Battery Simulation with Time-Inhomogeneous Markov Chain
基于时变马尔科夫链的24小时电池仿真

Based on the Markov chain model from MCM 2026:
- 4 States: Deep Sleep, Light Use, Streaming, Gaming
- 3 Time Modes: Sleep (23:00-7:00), Work (9-12, 14-18), Leisure (7-9, 12-14, 18-23)
- Coupled ODE system for battery dynamics

All curves derived rigorously from the coupled differential equations.
所有曲线由耦合微分方程组严格推导求解
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import matplotlib.gridspec as gridspec
import sys
import os

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 13,
    'legend.fontsize': 9,
    'figure.dpi': 150,
    'mathtext.fontset': 'dejavusans',
})


class MarkovBatteryModel:
    """
    Battery Model with Time-Inhomogeneous Markov Chain User Behavior
    基于时变马尔科夫链用户行为的电池模型
    """
    
    def __init__(self):
        # ==================== STATE DEFINITIONS ====================
        # S1: Deep Sleep, S2: Light Use, S3: Streaming, S4: Gaming
        self.state_names = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        self.num_states = 4
        
        # ==================== HARDWARE PARAMETERS ====================
        # APL (Average Picture Level) [Min, Max] for each state
        self.P_APL = np.array([
            [0, 0],       # Deep Sleep
            [60, 95],     # Light Use
            [20, 50],     # Streaming
            [40, 75]      # Gaming
        ])
        
        # CPU Utilization [Min, Max] %
        self.P_CPU_Util = np.array([
            [0, 2],       # Deep Sleep
            [5, 25],      # Light Use
            [15, 35],     # Streaming
            [70, 95]      # Gaming
        ])
        
        # CPU Frequency [Min, Max] GHz
        self.P_CPU_Freq = np.array([
            [0.3, 0.3],   # Deep Sleep (with micro-jitter)
            [0.8, 1.8],   # Light Use
            [1.0, 2.0],   # Streaming
            [2.2, 3.0]    # Gaming
        ])
        
        # Brightness [Min, Max] nits (0 for sleep)
        self.P_Brightness = np.array([
            [0, 0],       # Deep Sleep
            [150, 400],   # Light Use
            [300, 600],   # Streaming
            [400, 800]    # Gaming
        ])
        
        # ==================== TRANSITION MATRICES ====================
        # Sleep Mode (23:00 - 7:00)
        self.P_sleep = np.array([
            [0.995, 0.005, 0.000, 0.000],
            [0.600, 0.400, 0.000, 0.000],
            [0.100, 0.000, 0.900, 0.000],
            [0.100, 0.000, 0.000, 0.900]
        ])
        
        # Work Mode (9:00-12:00, 14:00-18:00)
        self.P_work = np.array([
            [0.850, 0.145, 0.003, 0.002],
            [0.250, 0.700, 0.040, 0.010],
            [0.100, 0.100, 0.800, 0.000],
            [0.200, 0.100, 0.000, 0.700]
        ])
        
        # Leisure Mode (7:00-9:00, 12:00-14:00, 18:00-23:00)
        self.P_leisure = np.array([
            [0.800, 0.150, 0.030, 0.020],
            [0.050, 0.650, 0.200, 0.100],
            [0.010, 0.040, 0.940, 0.010],
            [0.010, 0.010, 0.010, 0.970]
        ])
        
        # ==================== BATTERY PARAMETERS ====================
        self.Q_max = 4.0 * 3600       # As (4000 mAh)
        self.V_nom = 3.7              # V
        self.V_max = 4.2              # V
        self.V_min = 3.0              # V
        self.R_0 = 0.045              # Ω
        self.eta_pmic = 0.91          # PMIC efficiency
        self.dV_dT = -0.0003          # V/K (entropy coefficient)
        
        # Thermal parameters
        self.C_th_batt = 48.0         # J/K
        self.R_th_batt = 11.0         # K/W
        self.C_th_cpu = 2.0           # J/K
        self.R_th_cpu_batt = 4.5      # K/W
        self.R_th_cpu_env = 16.0      # K/W
        
        # Background O-U process
        self.theta_bg = 0.1           # mean reversion rate
        self.mu_bg = 0.03             # A (mean)
        self.sigma_bg = 0.008         # volatility
        
    def get_mode(self, hour):
        """
        Get current mode based on time
        根据时间获取当前模式
        
        Returns: 'sleep', 'work', 'leisure'
        """
        if hour >= 23 or hour < 7:
            return 'sleep'
        elif (9 <= hour < 12) or (14 <= hour < 18):
            return 'work'
        else:  # 7-9, 12-14, 18-23
            return 'leisure'
    
    def get_transition_matrix(self, hour):
        """Get appropriate transition matrix for current time"""
        mode = self.get_mode(hour)
        if mode == 'sleep':
            return self.P_sleep
        elif mode == 'work':
            return self.P_work
        else:
            return self.P_leisure
    
    def get_Q_matrix(self, hour):
        """
        Convert discrete transition matrix P to continuous rate matrix Q
        将离散转移矩阵P转换为连续速率矩阵Q
        
        Q = (P - I) / Δt, where Δt = 1 minute = 60 seconds
        """
        P = self.get_transition_matrix(hour)
        I = np.eye(self.num_states)
        dt = 60.0  # 1 minute in seconds
        Q = (P - I) / dt
        return Q
    
    def V_OCV(self, SOC):
        """Open Circuit Voltage model"""
        SOC = np.clip(SOC, 0.001, 0.999)
        V = 3.0 + 0.9 * SOC + 0.15 * SOC**2 + 0.15 * SOC**3
        V += 0.15 * (1 - np.exp(-8 * (SOC - 0.85))) * (SOC > 0.85)
        V -= 0.25 * np.exp(-12 * SOC)
        return np.clip(V, self.V_min, self.V_max)
    
    def R_int(self, SOC, T):
        """Internal resistance model"""
        SOC = np.clip(SOC, 0.01, 0.99)
        f_SOC = 1.0 + 0.4 * (SOC - 0.5)**2 + 0.5 * np.exp(-8 * SOC)
        E_a = 20000
        R_gas = 8.314
        T_ref = 298.15
        g_T = np.exp(E_a / R_gas * (1/T - 1/T_ref))
        return self.R_0 * f_SOC * g_T
    
    def capacity_factor(self, T):
        """Temperature-dependent capacity factor"""
        T_celsius = T - 273.15
        if T_celsius < 15:
            f = 1.0 - 0.012 * (15 - T_celsius)
        elif T_celsius > 40:
            f = 1.0 - 0.003 * (T_celsius - 40)
        else:
            f = 1.0
        return max(0.7, f)
    
    def compute_hardware_params(self, p_states, hour):
        """
        Compute hardware parameters from Markov state distribution
        从马尔科夫状态分布计算硬件参数
        """
        # Weighted average based on state probabilities
        cpu_util = 0
        cpu_freq = 0
        brightness = 0
        apl = 0
        
        for i in range(self.num_states):
            p = p_states[i]
            # CPU Utilization
            cpu_util += p * (self.P_CPU_Util[i, 0] + self.P_CPU_Util[i, 1]) / 2
            # CPU Frequency
            cpu_freq += p * (self.P_CPU_Freq[i, 0] + self.P_CPU_Freq[i, 1]) / 2
            # Brightness
            brightness += p * (self.P_Brightness[i, 0] + self.P_Brightness[i, 1]) / 2
            # APL
            apl += p * (self.P_APL[i, 0] + self.P_APL[i, 1]) / 2
        
        # Sunlight factor for brightness
        sunlight_factor = np.exp(-((hour - 13)**2) / (2 * 3**2))
        brightness_scale = 1.0 + 0.5 * sunlight_factor
        
        # Micro-jitter for CPU frequency (even in sleep)
        micro_jitter = 0.03  # GHz
        
        # Add leisure boost
        mode = self.get_mode(hour)
        if mode == 'leisure':
            cpu_util += 5
            cpu_freq += 0.1
        
        return {
            'cpu_load': np.clip(cpu_util / 100, 0.01, 1.0),
            'cpu_freq': np.clip(cpu_freq + micro_jitter, 0.2, 3.2),
            'brightness': np.clip(brightness * brightness_scale, 0, 1200),
            'apl': np.clip(apl, 0, 100),
            'screen_on': p_states[0] < 0.7  # Screen on if not mostly sleeping
        }
    
    def P_cpu(self, cpu_load, cpu_freq, T_cpu):
        """
        CPU power consumption
        P = C_eff · V² · f + P_leak · exp(k·ΔT)
        """
        # DVFS voltage estimation
        V_dd = 0.6 + 0.4 * (cpu_freq / 3.0)
        f = cpu_freq * 1e9  # Hz
        C_eff = 2e-9  # F
        
        P_dyn = C_eff * V_dd**2 * f * cpu_load
        P_leak = 0.15 * np.exp(0.04 * (T_cpu - 298.15))
        
        return P_dyn + P_leak
    
    def P_display(self, brightness, apl, screen_on):
        """
        Display power (AMOLED model)
        P = P_static + k_APL · APL + k_L · L
        """
        if not screen_on or brightness < 1:
            return 0.0
        
        P_static = 0.08
        P_apl = 0.015 * apl / 100  # APL contribution
        P_brightness = 0.002 * brightness  # nits contribution
        
        return P_static + P_apl + P_brightness
    
    def P_other(self, p_states, hour):
        """
        Other power consumption (network, GPS, bluetooth, background)
        """
        # Network power depends on activity
        P_network = 0.05 + 0.3 * (1 - p_states[0])  # More active = more network
        
        # Streaming uses more network
        P_network += 0.4 * p_states[2]  # Streaming
        
        # Gaming uses less network but still some
        P_network += 0.2 * p_states[3]  # Gaming
        
        # Background power
        P_bg = 0.1 * (1 - p_states[0])
        
        return P_network + P_bg
    
    def coupled_ode(self, t, y, charging_func, env_func, noise_seq):
        """
        Complete coupled ODE system
        完整耦合微分方程组
        
        State: y = [SOC, T_batt, T_cpu, I_bg, p0, p1, p2, p3]
        """
        # Unpack state
        SOC = np.clip(y[0], 0.01, 0.99)
        T_batt = np.clip(y[1], 268.15, 333.15)
        T_cpu = np.clip(y[2], 268.15, 363.15)
        I_bg = np.clip(y[3], 0.001, 0.2)
        p_states = np.clip(y[4:8], 0.001, 0.999)
        p_states = p_states / np.sum(p_states)  # Normalize
        
        hour = (t / 3600) % 24
        T_env = env_func(hour)
        is_charging = charging_func(hour)
        
        # Get hardware parameters from Markov state
        hw = self.compute_hardware_params(p_states, hour)
        
        # ========== COMPUTE POWER ==========
        P_cpu = self.P_cpu(hw['cpu_load'], hw['cpu_freq'], T_cpu)
        P_disp = self.P_display(hw['brightness'], hw['apl'], hw['screen_on'])
        P_other = self.P_other(p_states, hour)
        P_bg = self.V_nom * I_bg
        P_total = P_cpu + P_disp + P_other + P_bg
        
        # ========== VOLTAGE AND CURRENT ==========
        V_OCV = self.V_OCV(SOC)
        R_int = self.R_int(SOC, T_batt)
        
        if is_charging:
            if SOC < 0.80:
                I_total = -2.0  # Fast charge
            elif SOC < 0.95:
                I_total = -2.0 * (0.98 - SOC) / 0.18
            else:
                I_total = -0.1
            if SOC > 0.99:
                I_total = 0
        else:
            V_est = V_OCV - 0.1
            I_total = P_total / (self.eta_pmic * max(V_est, self.V_min))
        
        # ========== DIFFERENTIAL EQUATIONS ==========
        
        # 1. dSOC/dt = -I_total / (Q_max · f(T))
        Q_eff = self.Q_max * self.capacity_factor(T_batt)
        dSOC = -I_total / Q_eff
        if SOC >= 0.99 and dSOC > 0:
            dSOC = 0
        if SOC <= 0.05 and dSOC < 0:
            dSOC = 0
        
        # 2. dT_batt/dt = (P_joule + P_entropy - Q_diss) / C_th
        P_joule = I_total**2 * R_int
        P_entropy = abs(I_total) * T_batt * abs(self.dV_dT)
        Q_diss = (T_batt - T_env) / self.R_th_batt
        dT_batt = (P_joule + P_entropy - Q_diss) / self.C_th_batt
        
        # 3. dT_cpu/dt = (P_cpu - Q_cpu→batt - Q_cpu→env) / C_th_cpu
        Q_cpu_batt = (T_cpu - T_batt) / self.R_th_cpu_batt
        Q_cpu_env = (T_cpu - T_env) / self.R_th_cpu_env
        dT_cpu = (P_cpu - Q_cpu_batt - Q_cpu_env) / self.C_th_cpu
        
        # 4. dI_bg/dt = θ(μ - I_bg) + σ·ξ (Ornstein-Uhlenbeck)
        mu = self.mu_bg * (1 + p_states[2] + 2 * p_states[3])  # More active = more bg
        noise_idx = int(t / 60) % len(noise_seq)
        xi = noise_seq[noise_idx]
        dI_bg = self.theta_bg * (mu - I_bg) + self.sigma_bg * xi
        
        # 5-8. dp/dt = p · Q(t) (Markov chain dynamics)
        Q_mat = self.get_Q_matrix(hour)
        dp = p_states @ Q_mat
        
        return [dSOC, dT_batt, dT_cpu, dI_bg, dp[0], dp[1], dp[2], dp[3]]


def create_environment():
    """Create environment functions"""
    
    def charging_schedule(hour):
        """Charging periods: 2:00-3:30, 13:00-13:30, 22:00-23:30"""
        return (2 <= hour < 3.5) or (13 <= hour < 13.5) or (22 <= hour < 23.5)
    
    def env_temperature(hour):
        """Environment temperature"""
        # Indoor mostly, outdoor during leisure commute times
        if 7 <= hour < 9 or 18 <= hour < 19:
            return 298.15 + 3 * np.sin(2 * np.pi * (hour - 6) / 24)
        else:
            return 295.15
    
    return charging_schedule, env_temperature


def run_simulation():
    """Run the complete simulation"""
    print("="*75)
    print("24-HOUR BATTERY SIMULATION WITH TIME-INHOMOGENEOUS MARKOV CHAIN")
    print("基于时变马尔科夫链的24小时电池仿真")
    print("="*75)
    
    print("\n[User States]")
    print("  S1: Deep Sleep   S2: Light Use   S3: Streaming   S4: Gaming")
    
    print("\n[Time Periods]")
    print("  Sleep Mode:   23:00 - 7:00")
    print("  Leisure Mode: 7:00-9:00, 12:00-14:00, 18:00-23:00")
    print("  Work Mode:    9:00-12:00, 14:00-18:00")
    
    print("\n[Coupled Differential Equations]")
    print("  dSOC/dt = -I_total / (Q_max · f(T))")
    print("  C_th·dT_batt/dt = I²R + |I|T|dV/dT| - (T-T_env)/R_th")
    print("  C_cpu·dT_cpu/dt = P_cpu - (T_cpu-T_batt)/R₁ - (T_cpu-T_env)/R₂")
    print("  dI_bg = θ(μ-I_bg)dt + σdW  [O-U process]")
    print("  dp/dt = p·Q(t)  [Time-inhomogeneous Markov]")
    
    # Initialize
    model = MarkovBatteryModel()
    charging_func, env_func = create_environment()
    
    np.random.seed(42)
    noise_seq = np.random.normal(0, 1, 1500)
    
    # Initial state: mostly sleeping at midnight
    # [SOC, T_batt, T_cpu, I_bg, p_sleep, p_light, p_stream, p_game]
    y0 = [0.85, 294.15, 296.15, 0.03, 0.90, 0.08, 0.01, 0.01]
    
    t_span = (0, 24 * 3600)
    t_eval = np.linspace(0, 24 * 3600, 1441)
    
    print("\n[Solving ODE System...]")
    
    solution = solve_ivp(
        lambda t, y: model.coupled_ode(t, y, charging_func, env_func, noise_seq),
        t_span,
        y0,
        method='RK45',
        t_eval=t_eval,
        max_step=60,
        rtol=1e-4,
        atol=1e-6
    )
    
    print(f"  Status: {'Success' if solution.success else 'Failed'}")
    print(f"  Function evaluations: {solution.nfev}")
    
    # Extract results
    t_hours = solution.t / 3600
    n = len(t_hours)
    
    SOC = np.clip(solution.y[0], 0.05, 1.0) * 100
    T_batt = solution.y[1] - 273.15
    T_cpu = solution.y[2] - 273.15
    I_bg = solution.y[3]
    p_states = solution.y[4:8]
    
    # Normalize probabilities
    for i in range(n):
        total = np.sum(p_states[:, i])
        if total > 0:
            p_states[:, i] /= total
    
    # Compute derived quantities
    current = np.zeros(n)
    P_total = np.zeros(n)
    P_cpu = np.zeros(n)
    P_disp = np.zeros(n)
    brightness = np.zeros(n)
    cpu_util = np.zeros(n)
    cpu_freq = np.zeros(n)
    apl = np.zeros(n)
    mode_arr = np.zeros(n, dtype=int)  # 0=sleep, 1=work, 2=leisure
    
    for i in range(n):
        hour = t_hours[i] % 24
        p = p_states[:, i]
        hw = model.compute_hardware_params(p, hour)
        
        brightness[i] = hw['brightness']
        cpu_util[i] = hw['cpu_load'] * 100
        cpu_freq[i] = hw['cpu_freq']
        apl[i] = hw['apl']
        
        P_cpu[i] = model.P_cpu(hw['cpu_load'], hw['cpu_freq'], solution.y[2, i])
        P_disp[i] = model.P_display(hw['brightness'], hw['apl'], hw['screen_on'])
        P_other = model.P_other(p, hour)
        P_bg = model.V_nom * solution.y[3, i]
        P_total[i] = P_cpu[i] + P_disp[i] + P_other + P_bg
        
        is_charging = charging_func(hour)
        if is_charging:
            soc = solution.y[0, i]
            if soc < 0.8:
                current[i] = -2.0
            elif soc < 0.95:
                current[i] = -2.0 * (0.98 - soc) / 0.18
            else:
                current[i] = -0.1
        else:
            V_est = model.V_OCV(solution.y[0, i]) - 0.1
            current[i] = P_total[i] / (model.eta_pmic * max(V_est, model.V_min))
        
        mode = model.get_mode(hour)
        if mode == 'sleep':
            mode_arr[i] = 0
        elif mode == 'work':
            mode_arr[i] = 1
        else:
            mode_arr[i] = 2
    
    # Statistics
    print("\n[Results]")
    print(f"  Initial SOC: {SOC[0]:.1f}%")
    print(f"  Min SOC: {np.min(SOC):.1f}%")
    print(f"  Max SOC: {np.max(SOC):.1f}%")
    print(f"  Final SOC: {SOC[-1]:.1f}%")
    print(f"  Max Battery Temp: {np.max(T_batt):.1f}°C")
    print(f"  Max CPU Temp: {np.max(T_cpu):.1f}°C")
    print(f"  Max Power: {np.max(P_total):.2f}W")
    
    return {
        't_hours': t_hours,
        'SOC': SOC,
        'T_batt': T_batt,
        'T_cpu': T_cpu,
        'I_bg': I_bg,
        'p_states': p_states,
        'current': current,
        'P_total': P_total,
        'P_cpu': P_cpu,
        'P_disp': P_disp,
        'brightness': brightness,
        'cpu_util': cpu_util,
        'cpu_freq': cpu_freq,
        'apl': apl,
        'mode': mode_arr,
        'charging': np.array([charging_func(h) for h in t_hours])
    }


def plot_simulation(data, save_path=None):
    """Create comprehensive visualization"""
    
    fig = plt.figure(figsize=(16, 18))
    gs = gridspec.GridSpec(4, 2, height_ratios=[1, 1, 1, 1], hspace=0.28, wspace=0.25)
    
    t = data['t_hours']
    
    # Mode colors and labels
    mode_colors = ['#E8E8E8', '#FFE4E4', '#E4FFE4']  # sleep, work, leisure
    mode_names = ['Sleep', 'Work', 'Leisure']
    
    # Time period definitions
    periods = [
        (0, 7, 0, 'Sleep'),
        (7, 9, 2, 'Leisure'),
        (9, 12, 1, 'Work'),
        (12, 14, 2, 'Leisure'),
        (14, 18, 1, 'Work'),
        (18, 23, 2, 'Leisure'),
        (23, 24, 0, 'Sleep')
    ]
    
    def draw_background(ax, y_min, y_max):
        for start, end, mode, name in periods:
            rect = Rectangle((start, y_min), end - start, y_max - y_min,
                            facecolor=mode_colors[mode], edgecolor='none',
                            alpha=0.6, zorder=0)
            ax.add_patch(rect)
            mid = (start + end) / 2
            ax.text(mid, y_max - (y_max - y_min) * 0.05, name,
                   ha='center', va='top', fontsize=8, fontweight='bold',
                   color=['#666', '#A66', '#6A6'][mode])
    
    # ========== SUBPLOT 1: User State (Markov Chain) ==========
    ax1 = fig.add_subplot(gs[0, :])
    draw_background(ax1, 0.3, 4.5)
    
    # Plot state probabilities as stacked
    p = data['p_states']
    ax1.fill_between(t, 0.5, 0.5 + p[0] * 4, alpha=0.8, color='#5DADE2', label='Deep Sleep')
    ax1.fill_between(t, 0.5 + p[0] * 4, 0.5 + (p[0] + p[1]) * 4, alpha=0.8, color='#58D68D', label='Light Use')
    ax1.fill_between(t, 0.5 + (p[0] + p[1]) * 4, 0.5 + (p[0] + p[1] + p[2]) * 4, alpha=0.8, color='#F7DC6F', label='Streaming')
    ax1.fill_between(t, 0.5 + (p[0] + p[1] + p[2]) * 4, 4.5, alpha=0.8, color='#EC7063', label='Gaming')
    
    # Most likely state line
    dominant_state = np.argmax(p, axis=0) + 1
    ax1.plot(t, dominant_state, 'k-', linewidth=1.5, alpha=0.7, label='Dominant State')
    
    ax1.set_xlim(0, 24)
    ax1.set_ylim(0.3, 4.7)
    ax1.set_yticks([1, 2, 3, 4])
    ax1.set_yticklabels(['Deep Sleep', 'Light Use', 'Streaming', 'Gaming'])
    ax1.set_xlabel('Time (hour)')
    ax1.set_title('User Behavior State (Time-Inhomogeneous Markov Chain)\n' +
                 r'$\frac{dp}{dt} = p \cdot Q(t)$, where $Q(t)$ switches between Sleep/Work/Leisure modes',
                 fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', ncol=5, fontsize=8)
    ax1.set_xticks(range(0, 25, 2))
    ax1.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 2: SOC & Current ==========
    ax2 = fig.add_subplot(gs[1, 0])
    draw_background(ax2, -20, 110)
    
    ax2.plot(t, data['SOC'], color='#E67E22', linewidth=2.5, label='SOC [%]')
    ax2.plot(t, data['current'] * 30, color='#3498DB', linewidth=1.5, label='Current [A×30]')
    
    # Mark charging
    for i, ch in enumerate(data['charging']):
        if ch and (i == 0 or not data['charging'][i-1]):
            ax2.axvline(x=t[i], color='#27AE60', linestyle='--', alpha=0.5)
            ax2.annotate('Charging', xy=(t[i]+0.2, 15), fontsize=8, color='#27AE60')
    
    ax2.set_xlim(0, 24)
    ax2.set_ylim(-20, 110)
    ax2.set_xlabel('Time (hour)')
    ax2.set_ylabel('SOC (%) / Current (A×30)')
    ax2.set_title('Battery SOC & Current from ODE\n' +
                 r'$\frac{dSOC}{dt} = -\frac{I_{total}}{Q_{max} \cdot f(T)}$',
                 fontsize=11, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.set_xticks(range(0, 25, 2))
    ax2.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 3: Temperature ==========
    ax3 = fig.add_subplot(gs[1, 1])
    draw_background(ax3, 18, 38)
    
    ax3.plot(t, data['T_batt'], color='#E74C3C', linewidth=2, label='Battery')
    ax3.plot(t, data['T_cpu'], color='#3498DB', linewidth=2, label='CPU')
    ax3.axhline(y=22, color='gray', linestyle='--', alpha=0.5, label='Ambient')
    
    ax3.set_xlim(0, 24)
    ax3.set_ylim(18, 38)
    ax3.set_xlabel('Time (hour)')
    ax3.set_ylabel('Temperature (°C)')
    ax3.set_title('Thermal Dynamics from ODE\n' +
                 r'$C_{th}\frac{dT}{dt} = P_{gen} - \sum\frac{\Delta T}{R_{th}}$',
                 fontsize=11, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.set_xticks(range(0, 25, 2))
    ax3.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 4: Display Parameters ==========
    ax4 = fig.add_subplot(gs[2, 0])
    draw_background(ax4, 0, 110)
    
    ax4a = ax4
    ax4a.plot(t, data['apl'], color='#3498DB', linewidth=1, label='APL [%]')
    ax4a.set_ylabel('APL (%)', color='#3498DB')
    ax4a.tick_params(axis='y', labelcolor='#3498DB')
    ax4a.set_ylim(0, 110)
    
    ax4b = ax4a.twinx()
    ax4b.fill_between(t, 0, data['brightness'], color='#F39C12', alpha=0.3)
    ax4b.plot(t, data['brightness'], color='#F39C12', linewidth=1, label='Brightness')
    ax4b.set_ylabel('Brightness (nits)', color='#F39C12')
    ax4b.tick_params(axis='y', labelcolor='#F39C12')
    ax4b.set_ylim(0, 1200)
    
    ax4.set_xlim(0, 24)
    ax4.set_xlabel('Time (hour)')
    ax4.set_title('Display Parameters (Derived from Markov State)\n' +
                 r'$P_{disp} = P_0 + k_{APL} \cdot APL + k_L \cdot L$',
                 fontsize=11, fontweight='bold')
    ax4.set_xticks(range(0, 25, 2))
    ax4.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 5: CPU Parameters ==========
    ax5 = fig.add_subplot(gs[2, 1])
    draw_background(ax5, 0, 110)
    
    ax5a = ax5
    ax5a.plot(t, data['cpu_util'], color='#27AE60', linewidth=1, label='CPU Util [%]')
    ax5a.set_ylabel('CPU Util (%)', color='#27AE60')
    ax5a.tick_params(axis='y', labelcolor='#27AE60')
    ax5a.set_ylim(0, 110)
    
    ax5b = ax5a.twinx()
    ax5b.plot(t, data['cpu_freq'], color='#E74C3C', linewidth=1.2, label='Freq')
    ax5b.set_ylabel('Frequency (GHz)', color='#E74C3C')
    ax5b.tick_params(axis='y', labelcolor='#E74C3C')
    ax5b.set_ylim(0, 3.5)
    
    ax5.set_xlim(0, 24)
    ax5.set_xlabel('Time (hour)')
    ax5.set_title('CPU Parameters (Derived from Markov State)\n' +
                 r'$P_{cpu} = C_{eff}V^2f + P_{leak}e^{k\Delta T}$',
                 fontsize=11, fontweight='bold')
    ax5.set_xticks(range(0, 25, 2))
    ax5.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 6: Power Breakdown ==========
    ax6 = fig.add_subplot(gs[3, 0])
    draw_background(ax6, 0, 4)
    
    ax6.fill_between(t, 0, data['P_cpu'], alpha=0.8, color='#E74C3C', label='CPU')
    ax6.fill_between(t, data['P_cpu'], data['P_cpu'] + data['P_disp'],
                    alpha=0.8, color='#3498DB', label='Display')
    ax6.fill_between(t, data['P_cpu'] + data['P_disp'], data['P_total'],
                    alpha=0.8, color='#95A5A6', label='Other')
    ax6.plot(t, data['P_total'], 'k-', linewidth=1.5, label='Total')
    
    ax6.set_xlim(0, 24)
    ax6.set_ylim(0, 4)
    ax6.set_xlabel('Time (hour)')
    ax6.set_ylabel('Power (W)')
    ax6.set_title('Power Consumption Breakdown\n' +
                 r'$P_{total} = P_{cpu} + P_{disp} + P_{other}$',
                 fontsize=11, fontweight='bold')
    ax6.legend(loc='upper right', fontsize=9)
    ax6.set_xticks(range(0, 25, 2))
    ax6.grid(True, alpha=0.3)
    
    # ========== SUBPLOT 7: Markov State Probabilities ==========
    ax7 = fig.add_subplot(gs[3, 1])
    draw_background(ax7, 0, 1.05)
    
    ax7.stackplot(t, data['p_states'][0], data['p_states'][1], 
                 data['p_states'][2], data['p_states'][3],
                 colors=['#5DADE2', '#58D68D', '#F7DC6F', '#EC7063'],
                 labels=['Deep Sleep', 'Light Use', 'Streaming', 'Gaming'],
                 alpha=0.8)
    
    ax7.set_xlim(0, 24)
    ax7.set_ylim(0, 1.05)
    ax7.set_xlabel('Time (hour)')
    ax7.set_ylabel('Probability')
    ax7.set_title('Markov State Probabilities\n' +
                 r'$\frac{dp}{dt} = p \cdot Q(t)$',
                 fontsize=11, fontweight='bold')
    ax7.legend(loc='upper right', fontsize=8, ncol=2)
    ax7.set_xticks(range(0, 25, 2))
    ax7.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"\nFigure saved: {save_path}")
    
    return fig


def main():
    """Main execution"""
    data = run_simulation()
    
    print("\n[Generating Visualization...]")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, '24hour_markov_battery_simulation.png')
    fig = plot_simulation(data, save_path)
    
    plt.close(fig)
    print("\n" + "="*75)
    print("Simulation Complete!")
    print("="*75)
    
    return data


if __name__ == "__main__":
    data = main()
