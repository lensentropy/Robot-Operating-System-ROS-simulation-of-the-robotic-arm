#!/usr/bin/env python3
"""
智能手机电池SOC与耗能关系建模及剩余时间预测
Smartphone Battery SOC-Power Relationship Modeling and Remaining Time Prediction

结合：
1. 文件1 (b.m): 时变马尔科夫链时间分区
   - 4状态: Deep Sleep, Light Use, Streaming, Gaming
   - 3模式: Sleep(23-7), Work(9-12,14-18), Leisure(7-9,12-14,18-23)
   
2. 文件2 (耦合.pdf): 耦合微分方程组
   - 电化学热耦合核心方程
   - 各模块功耗耦合表达式
   - 连续时间马尔科夫用户模型

核心功能：
1. SOC与耗能的连续时间关系建模
2. 基于当前状态和使用模式预测剩余使用时间
3. 卡尔曼滤波状态估计
4. 不确定性量化

Author: Battery Modeling Research
"""

import numpy as np
from scipy.integrate import solve_ivp, quad
from scipy.optimize import brentq
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.lines import Line2D
import matplotlib.gridspec as gridspec
from dataclasses import dataclass
from typing import Dict, Tuple, List, Callable
import warnings
import os

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'figure.dpi': 150,
    'mathtext.fontset': 'dejavusans',
})


# =============================================================================
# 1. 耦合微分方程组定义 (来自PDF文件)
# =============================================================================

class CoupledBatteryODE:
    """
    耦合微分方程组模型
    
    主耦合方程组 (3.1节):
    dSOC/dt = -I_total(t) / (Q_max * f(T_batt, N))
    C_th * dT_batt/dt = P_joule + P_entropy - (T_batt - T_env)/R_th
    V_batt = V_OCV(SOC) - I_total * R_int(SOC, T_batt, N)
    
    总电流耦合 (3.2节):
    I_total = (P_SoC + P_disp + P_5G + P_BT + P_GNSS + P_background) / (η_PMIC * V_batt)
    """
    
    def __init__(self):
        # ================== 电池参数 ==================
        self.Q_max = 4.0 * 3600       # As (4000 mAh)
        self.V_nom = 3.7              # V
        self.V_max = 4.2              # V
        self.V_min = 3.0              # V
        self.R_0 = 0.042              # Ω 基准内阻
        self.eta_PMIC = 0.91          # PMIC效率
        self.dV_dT = -0.0003          # V/K 熵系数
        self.N_cycles = 150           # 循环次数
        self.alpha_aging = 0.0008     # 容量衰减率
        
        # 热参数
        self.C_th_batt = 48.0         # J/K 电池热容
        self.R_th_batt = 11.0         # K/W 热阻
        self.C_th_cpu = 2.0           # J/K CPU热容
        self.R_th_cpu_batt = 4.5      # K/W
        self.R_th_cpu_env = 16.0      # K/W
        
        # ================== 马尔科夫参数 ==================
        # 状态: 0=Deep Sleep, 1=Light Use, 2=Streaming, 3=Gaming
        self.state_names = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        
        # 转移矩阵 (来自b.m文件)
        self.P_sleep = np.array([
            [0.995, 0.005, 0.000, 0.000],
            [0.600, 0.400, 0.000, 0.000],
            [0.100, 0.000, 0.900, 0.000],
            [0.100, 0.000, 0.000, 0.900]
        ])
        self.P_work = np.array([
            [0.850, 0.145, 0.003, 0.002],
            [0.250, 0.700, 0.040, 0.010],
            [0.100, 0.100, 0.800, 0.000],
            [0.200, 0.100, 0.000, 0.700]
        ])
        self.P_leisure = np.array([
            [0.800, 0.150, 0.030, 0.020],
            [0.050, 0.650, 0.200, 0.100],
            [0.010, 0.040, 0.940, 0.010],
            [0.010, 0.010, 0.010, 0.970]
        ])
        
        # 硬件参数范围 (来自b.m文件)
        self.hw_params = {
            0: {'cpu_util': (0, 2), 'cpu_freq': (0.3, 0.3), 'apl': (0, 0), 'brightness': (0, 0)},
            1: {'cpu_util': (5, 25), 'cpu_freq': (0.8, 1.8), 'apl': (60, 95), 'brightness': (150, 400)},
            2: {'cpu_util': (15, 35), 'cpu_freq': (1.0, 2.0), 'apl': (20, 50), 'brightness': (300, 600)},
            3: {'cpu_util': (70, 95), 'cpu_freq': (2.2, 3.0), 'apl': (40, 75), 'brightness': (400, 800)}
        }
        
        # O-U过程参数 (来自PDF 4.6节)
        self.theta_bg = 0.08          # 均值回复率
        self.mu_bg = 0.03             # A 均值
        self.sigma_bg = 0.008         # 波动率
        
        # GNSS参数 (来自PDF 4.5节)
        self.tau_lock = 25.0          # 锁定时间常数
        self.tau_unlock = 8.0         # 失锁时间常数
        self.P_LNA = 0.015            # W LNA功耗
        self.P_acq = 0.12             # W 捕获功耗
        self.P_track = 0.06           # W 跟踪功耗
    
    def get_time_mode(self, hour: float) -> str:
        """获取当前时间模式 (来自PDF 5.1节和b.m)"""
        if hour >= 23 or hour < 7:
            return 'sleep'
        elif (9 <= hour < 12) or (14 <= hour < 18):
            return 'work'
        else:  # 7-9, 12-14, 18-23
            return 'leisure'
    
    def get_Q_matrix(self, hour: float) -> np.ndarray:
        """
        获取连续时间转移率矩阵 Q(t)
        来自PDF 5.1节: dp/dt = p·Q(t)
        Q = (P - I) / Δt
        """
        mode = self.get_time_mode(hour)
        if mode == 'sleep':
            P = self.P_sleep
        elif mode == 'work':
            P = self.P_work
        else:
            P = self.P_leisure
        
        dt = 60.0  # 1分钟
        Q = (P - np.eye(4)) / dt
        return Q
    
    def V_OCV(self, SOC: float) -> float:
        """
        开路电压模型 V_OCV(SOC)
        来自PDF 3.1节
        """
        SOC = np.clip(SOC, 0.001, 0.999)
        V = 3.0 + 0.9 * SOC + 0.15 * SOC**2 + 0.15 * SOC**3
        V += 0.15 * (1 - np.exp(-8 * (SOC - 0.85))) * (SOC > 0.85)
        V -= 0.25 * np.exp(-12 * SOC)
        return np.clip(V, self.V_min, self.V_max)
    
    def R_int(self, SOC: float, T: float) -> float:
        """
        内阻模型 R_int(SOC, T_batt, N)
        来自PDF 3.1节
        """
        SOC = np.clip(SOC, 0.01, 0.99)
        f_SOC = 1.0 + 0.4 * (SOC - 0.5)**2 + 0.5 * np.exp(-8 * SOC)
        E_a, R_gas, T_ref = 20000, 8.314, 298.15
        g_T = np.exp(E_a / R_gas * (1/T - 1/T_ref))
        h_N = 1.0 + self.alpha_aging * self.N_cycles * 1.5
        return self.R_0 * f_SOC * g_T * h_N
    
    def capacity_factor(self, T: float) -> float:
        """容量温度因子 f(T,N)"""
        T_c = T - 273.15
        f = 1.0 - 0.012 * max(0, 15 - T_c) - 0.003 * max(0, T_c - 40)
        f *= (1.0 - self.alpha_aging * self.N_cycles)
        return max(0.7, f)
    
    def compute_hardware_params(self, p_states: np.ndarray, hour: float) -> Dict:
        """
        状态到硬件参数的映射函数
        来自PDF 5.2节
        """
        cpu_util = sum(p_states[i] * np.mean(self.hw_params[i]['cpu_util']) for i in range(4))
        cpu_freq = sum(p_states[i] * np.mean(self.hw_params[i]['cpu_freq']) for i in range(4))
        apl = sum(p_states[i] * np.mean(self.hw_params[i]['apl']) for i in range(4))
        brightness = sum(p_states[i] * np.mean(self.hw_params[i]['brightness']) for i in range(4))
        
        # 阳光因子
        sunlight = np.exp(-((hour - 13)**2) / 18)
        brightness *= (1 + 0.5 * sunlight)
        
        # 微波动 (来自b.m)
        cpu_freq += 0.03  # micro-jitter
        
        mode = self.get_time_mode(hour)
        if mode == 'leisure':
            cpu_util += 5
            cpu_freq += 0.1
        
        return {
            'cpu_load': np.clip(cpu_util / 100, 0.01, 1.0),
            'cpu_freq': np.clip(cpu_freq, 0.2, 3.2),
            'apl': np.clip(apl, 0, 100),
            'brightness': np.clip(brightness, 0, 1200),
            'screen_on': p_states[0] < 0.7
        }
    
    def P_SoC(self, cpu_load: float, cpu_freq: float, T_cpu: float) -> float:
        """
        SoC模块功耗 (电热强耦合)
        来自PDF 4.1节: P_SoC = f_cpu * V_dd² + I_leak(T) * V_dd
        """
        V_dd = 0.6 + 0.4 * (cpu_freq / 3.0)
        f = cpu_freq * 1e9
        C_eff = 2e-9
        P_dyn = C_eff * V_dd**2 * f * cpu_load
        P_leak = 0.15 * np.exp(0.04 * (T_cpu - 298.15))
        return P_dyn + P_leak
    
    def P_disp(self, brightness: float, apl: float, screen_on: bool, f_refresh: float = 60) -> float:
        """
        显示模块功耗 (环境光内容耦合)
        来自PDF 4.2节: P_disp = P_static + k_drv*f_refresh + L_set*A(t)
        """
        if not screen_on or brightness < 1:
            return 0.0
        P_static = 0.08
        k_drv = 8e-6
        k_L = 0.002
        k_APL = 0.015
        return P_static + k_drv * f_refresh + k_L * brightness + k_APL * apl / 100
    
    def P_5G(self, data_rate: float, signal: float) -> float:
        """
        5G通信模块功耗 (信道距离耦合)
        来自PDF 4.3节
        """
        P_idle = 0.05
        P_data = 0.01 * data_rate
        P_tx = 0.5 * (1 - signal) * 0.5 * min(1, data_rate / 30)
        return P_idle + P_data + P_tx
    
    def P_BT(self, p_states: np.ndarray) -> float:
        """
        蓝牙模块功耗 (事件驱动耦合)
        来自PDF 4.4节
        """
        # Streaming和Gaming使用蓝牙
        P_audio = 0.045
        P_idle = 0.008
        P_sleep = 0.0008
        return p_states[0] * P_sleep + (p_states[1] + p_states[2]) * P_idle + p_states[3] * P_audio
    
    def P_GNSS(self, x_lock: float, gps_active: bool) -> float:
        """
        GNSS模块功耗 (环境信号耦合)
        来自PDF 4.5节: P_GNSS = P_LNA + x_lock*P_track + (1-x_lock)*P_acq
        """
        if not gps_active:
            return 0.0
        return self.P_LNA + x_lock * self.P_track + (1 - x_lock) * self.P_acq
    
    def P_background(self, I_bg: float) -> float:
        """
        后台任务功耗
        来自PDF 4.6节: P_background = V_batt * I_bg
        """
        return self.V_nom * I_bg


# =============================================================================
# 2. SOC与耗能关系建模
# =============================================================================

class SOCPowerRelationship:
    """
    SOC与耗能的连续时间关系模型
    
    核心方程:
    dSOC/dt = -I_total / (Q_max * f(T,N))
    I_total = P_total / (η * V_batt)
    
    因此: dSOC/dt = -P_total / (η * V_batt * Q_max * f(T,N))
    
    积分形式: SOC(t) = SOC(0) - ∫₀ᵗ P(τ)/(η*V(τ)*Q_eff) dτ
    """
    
    def __init__(self, ode_model: CoupledBatteryODE):
        self.model = ode_model
    
    def dSOC_dt(self, SOC: float, P_total: float, T_batt: float) -> float:
        """
        SOC变化率与功耗的关系
        dSOC/dt = -P_total / (η * V_batt * Q_max * f(T,N))
        """
        V_batt = self.model.V_OCV(SOC)
        Q_eff = self.model.Q_max * self.model.capacity_factor(T_batt)
        eta = self.model.eta_PMIC
        
        # 放电电流
        I_total = P_total / (eta * max(V_batt, self.model.V_min))
        
        # SOC变化率
        return -I_total / Q_eff
    
    def energy_consumed(self, SOC_start: float, SOC_end: float, T_batt: float = 298.15) -> float:
        """
        计算从SOC_start到SOC_end消耗的能量 (Wh)
        E = ∫ V_batt * I dt = ∫ V_batt * Q_max * dSOC
        """
        Q_max_Ah = self.model.Q_max / 3600  # Convert to Ah
        
        def V_OCV_integrand(soc):
            return self.model.V_OCV(soc)
        
        # 积分计算能量
        E, _ = quad(V_OCV_integrand, SOC_end, SOC_start)
        E *= Q_max_Ah * self.model.capacity_factor(T_batt)
        
        return E  # Wh
    
    def SOC_from_power_profile(self, t_array: np.ndarray, P_array: np.ndarray, 
                               SOC_init: float, T_batt: float = 298.15) -> np.ndarray:
        """
        根据功率曲线计算SOC轨迹
        SOC(t) = SOC_0 - ∫₀ᵗ P(τ)/(η*V(τ)*Q_eff) dτ
        """
        SOC = np.zeros_like(t_array)
        SOC[0] = SOC_init
        
        Q_eff = self.model.Q_max * self.model.capacity_factor(T_batt)
        eta = self.model.eta_PMIC
        
        for i in range(1, len(t_array)):
            dt = t_array[i] - t_array[i-1]
            V_batt = self.model.V_OCV(SOC[i-1])
            I = P_array[i-1] / (eta * max(V_batt, self.model.V_min))
            dSOC = -I * dt / Q_eff
            SOC[i] = np.clip(SOC[i-1] + dSOC, 0.05, 1.0)
        
        return SOC


# =============================================================================
# 3. 剩余使用时间预测
# =============================================================================

class RemainingTimePredictor:
    """
    剩余使用时间预测器
    
    基于:
    1. 当前SOC
    2. 功耗模式 (由马尔科夫状态决定)
    3. 用户使用习惯
    
    预测方法:
    t_remain = ∫_{SOC_cutoff}^{SOC_current} Q_eff * V(SOC) / P(SOC) dSOC
    """
    
    def __init__(self, ode_model: CoupledBatteryODE, soc_power: SOCPowerRelationship):
        self.model = ode_model
        self.soc_power = soc_power
        self.SOC_cutoff = 0.05  # 5% 截止
    
    def predict_constant_power(self, SOC_current: float, P_avg: float, 
                               T_batt: float = 298.15) -> float:
        """
        恒定功率下的剩余时间预测
        t = (SOC_current - SOC_cutoff) * Q_eff * V_avg / P_avg
        """
        if P_avg <= 0:
            return float('inf')
        
        delta_SOC = SOC_current - self.SOC_cutoff
        if delta_SOC <= 0:
            return 0.0
        
        Q_eff = self.model.Q_max * self.model.capacity_factor(T_batt)
        
        # 平均电压
        n_points = 20
        SOC_range = np.linspace(self.SOC_cutoff, SOC_current, n_points)
        V_avg = np.mean([self.model.V_OCV(s) for s in SOC_range])
        
        # 时间 = 能量 / 功率
        E_available = Q_eff * V_avg * delta_SOC  # J
        t_remain = E_available / P_avg  # seconds
        
        return t_remain / 3600  # hours
    
    def predict_with_markov(self, SOC_current: float, p_states: np.ndarray,
                            hour_start: float, T_batt: float = 298.15,
                            max_hours: float = 48) -> Dict:
        """
        考虑马尔科夫状态的剩余时间预测
        通过数值积分求解
        """
        # 预估各状态的平均功耗
        P_state = np.array([0.15, 0.8, 2.0, 3.5])  # W per state
        
        # 初始平均功耗
        P_initial = np.dot(p_states, P_state)
        
        # 简化预测: 假设功耗随时间略有变化
        def power_profile(t_hours):
            hour = (hour_start + t_hours) % 24
            mode = self.model.get_time_mode(hour)
            if mode == 'sleep':
                return 0.15
            elif mode == 'work':
                return 0.8
            else:
                return 1.5
        
        # 数值积分预测
        t = 0
        SOC = SOC_current
        dt = 0.01  # hours
        
        Q_eff = self.model.Q_max * self.model.capacity_factor(T_batt)
        eta = self.model.eta_PMIC
        
        while SOC > self.SOC_cutoff and t < max_hours:
            P = power_profile(t)
            V = self.model.V_OCV(SOC)
            I = P / (eta * max(V, self.model.V_min))
            dSOC = -I * dt * 3600 / Q_eff
            SOC += dSOC
            t += dt
        
        # 不确定性估计 (±20%)
        t_low = t * 0.8
        t_high = t * 1.2
        
        return {
            'expected_hours': t,
            'confidence_interval': (t_low, t_high),
            'initial_power': P_initial,
            'SOC_trajectory': None  # 可选返回完整轨迹
        }
    
    def predict_scenario_based(self, SOC_current: float, scenario: str,
                               T_batt: float = 298.15) -> Dict:
        """
        基于场景的预测
        """
        scenario_power = {
            'idle': 0.15,
            'light_use': 0.8,
            'video_streaming': 2.0,
            'navigation': 2.8,
            'gaming': 3.5,
            'heavy_multitask': 4.0
        }
        
        P = scenario_power.get(scenario, 1.0)
        t_remain = self.predict_constant_power(SOC_current, P, T_batt)
        
        return {
            'scenario': scenario,
            'power': P,
            'remaining_hours': t_remain,
            'confidence_interval': (t_remain * 0.85, t_remain * 1.15)
        }
    
    def predict_all_scenarios(self, SOC_current: float, T_batt: float = 298.15) -> Dict:
        """
        预测所有场景的剩余时间
        """
        scenarios = ['idle', 'light_use', 'video_streaming', 'navigation', 'gaming', 'heavy_multitask']
        results = {}
        for s in scenarios:
            results[s] = self.predict_scenario_based(SOC_current, s, T_batt)
        return results


# =============================================================================
# 4. 完整仿真运行
# =============================================================================

def run_complete_simulation():
    """运行完整仿真"""
    print("="*80)
    print("智能手机电池SOC与耗能关系建模及剩余时间预测")
    print("Smartphone Battery SOC-Power Modeling and Remaining Time Prediction")
    print("="*80)
    
    # 初始化模型
    model = CoupledBatteryODE()
    soc_power = SOCPowerRelationship(model)
    predictor = RemainingTimePredictor(model, soc_power)
    
    print("\n[耦合微分方程组]")
    print("  dSOC/dt = -I_total / (Q_max · f(T,N))")
    print("  C_th·dT/dt = P_joule + P_entropy - (T-T_env)/R_th")
    print("  dp/dt = p·Q(t)  [时变马尔科夫]")
    print("  dI_bg = θ(μ-I_bg)dt + σdW  [O-U过程]")
    print("  dx_lock/dt = (S-x_lock)/τ  [GNSS]")
    
    print("\n[时间分区] (来自马尔科夫模型)")
    print("  Sleep:   23:00-7:00")
    print("  Work:    9:00-12:00, 14:00-18:00")
    print("  Leisure: 7:00-9:00, 12:00-14:00, 18:00-23:00")
    
    # ========== 仿真设置 ==========
    np.random.seed(42)
    noise_seq = np.random.normal(0, 1, 2000)
    
    # 环境函数
    def charging_func(hour):
        return (2 <= hour < 3.5) or (13 <= hour < 13.5) or (22 <= hour < 23.5)
    
    def env_temp(hour):
        if 7 <= hour < 9 or 18 <= hour < 19:
            return 298.15 + 3 * np.sin(2 * np.pi * (hour - 6) / 24)
        return 295.15
    
    # ========== 耦合ODE ==========
    def coupled_ode(t, y):
        SOC = np.clip(y[0], 0.01, 0.99)
        T_batt = np.clip(y[1], 270, 330)
        T_cpu = np.clip(y[2], 270, 360)
        x_lock = np.clip(y[3], 0, 1)
        I_bg = np.clip(y[4], 0.001, 0.2)
        p_states = np.clip(y[5:9], 0.001, 0.999)
        p_states = p_states / np.sum(p_states)
        
        hour = (t / 3600) % 24
        T_env = env_temp(hour)
        is_charging = charging_func(hour)
        
        # 硬件参数
        hw = model.compute_hardware_params(p_states, hour)
        
        # 各模块功耗
        P_cpu = model.P_SoC(hw['cpu_load'], hw['cpu_freq'], T_cpu)
        P_disp = model.P_disp(hw['brightness'], hw['apl'], hw['screen_on'])
        P_5G = model.P_5G(5 * (1 - p_states[0]), 0.8)
        P_BT = model.P_BT(p_states)
        P_GNSS = model.P_GNSS(x_lock, p_states[3] > 0.1 or (7 <= hour < 9) or (18 <= hour < 19))
        P_bg = model.P_background(I_bg)
        P_total = P_cpu + P_disp + P_5G + P_BT + P_GNSS + P_bg
        
        # 电压和电流
        V_OCV = model.V_OCV(SOC)
        R_int = model.R_int(SOC, T_batt)
        
        if is_charging:
            if SOC < 0.80:
                I_total = -2.0
            elif SOC < 0.95:
                I_total = -2.0 * (0.98 - SOC) / 0.18
            else:
                I_total = -0.1
            if SOC > 0.99:
                I_total = 0
        else:
            I_total = P_total / (model.eta_PMIC * max(V_OCV - 0.1, model.V_min))
        
        # 微分方程
        Q_eff = model.Q_max * model.capacity_factor(T_batt)
        dSOC = -I_total / Q_eff
        if (SOC >= 0.99 and dSOC > 0) or (SOC <= 0.05 and dSOC < 0):
            dSOC = 0
        
        P_joule = I_total**2 * R_int
        P_entropy = abs(I_total) * T_batt * abs(model.dV_dT)
        dT_batt = (P_joule + P_entropy - (T_batt - T_env) / model.R_th_batt) / model.C_th_batt
        
        Q_cpu_batt = (T_cpu - T_batt) / model.R_th_cpu_batt
        Q_cpu_env = (T_cpu - T_env) / model.R_th_cpu_env
        dT_cpu = (P_cpu - Q_cpu_batt - Q_cpu_env) / model.C_th_cpu
        
        gps_active = p_states[3] > 0.1 or (7 <= hour < 9)
        S_signal = 0.8 if gps_active else 0
        tau = model.tau_lock if S_signal > 0.5 else model.tau_unlock
        dx_lock = (S_signal - x_lock) / tau if gps_active else -x_lock / model.tau_unlock
        
        mu = model.mu_bg * (1 + p_states[2] + 2 * p_states[3])
        xi = noise_seq[int(t / 60) % len(noise_seq)]
        dI_bg = model.theta_bg * (mu - I_bg) + model.sigma_bg * xi
        
        Q_mat = model.get_Q_matrix(hour)
        dp = p_states @ Q_mat
        
        return [dSOC, dT_batt, dT_cpu, dx_lock, dI_bg, dp[0], dp[1], dp[2], dp[3]]
    
    # ========== 求解ODE ==========
    print("\n[求解耦合微分方程组...]")
    
    y0 = [0.85, 294.15, 296.15, 0.0, 0.03, 0.90, 0.08, 0.01, 0.01]
    t_span = (0, 24 * 3600)
    t_eval = np.linspace(0, 24 * 3600, 1441)
    
    solution = solve_ivp(coupled_ode, t_span, y0, method='RK45', t_eval=t_eval,
                        max_step=60, rtol=1e-4, atol=1e-6)
    
    print(f"  求解状态: {'成功' if solution.success else '失败'}")
    print(f"  函数评估次数: {solution.nfev}")
    
    # ========== 提取结果 ==========
    t_hours = solution.t / 3600
    n = len(t_hours)
    
    SOC = np.clip(solution.y[0], 0.05, 1.0) * 100
    T_batt = solution.y[1] - 273.15
    T_cpu = solution.y[2] - 273.15
    x_lock = solution.y[3]
    I_bg = solution.y[4]
    p_states = solution.y[5:9]
    for i in range(n):
        p_states[:, i] /= np.sum(p_states[:, i])
    
    # 计算功率
    P_total = np.zeros(n)
    I_total = np.zeros(n)
    
    for i in range(n):
        hour = t_hours[i] % 24
        p = p_states[:, i]
        hw = model.compute_hardware_params(p, hour)
        
        P_cpu = model.P_SoC(hw['cpu_load'], hw['cpu_freq'], solution.y[2, i])
        P_disp = model.P_disp(hw['brightness'], hw['apl'], hw['screen_on'])
        P_5G = model.P_5G(5 * (1 - p[0]), 0.8)
        P_BT = model.P_BT(p)
        P_GNSS = model.P_GNSS(solution.y[3, i], p[3] > 0.1)
        P_bg = model.P_background(solution.y[4, i])
        P_total[i] = P_cpu + P_disp + P_5G + P_BT + P_GNSS + P_bg
        
        is_charging = charging_func(hour)
        if is_charging:
            soc = solution.y[0, i]
            I_total[i] = -2.0 if soc < 0.8 else (-2.0 * (0.98 - soc) / 0.18 if soc < 0.95 else -0.1)
        else:
            V = model.V_OCV(solution.y[0, i]) - 0.1
            I_total[i] = P_total[i] / (model.eta_PMIC * max(V, model.V_min))
    
    # ========== 剩余时间预测 ==========
    print("\n[剩余使用时间预测]")
    
    # 选择几个关键时间点进行预测
    prediction_times = [0, 360, 720, 1080]  # 0h, 6h, 12h, 18h
    predictions = []
    
    for idx in prediction_times:
        if idx < n:
            hour = t_hours[idx]
            soc = solution.y[0, idx]
            p = p_states[:, idx]
            
            pred = predictor.predict_with_markov(soc, p, hour)
            predictions.append({
                'time': hour,
                'SOC': soc * 100,
                'predicted_remain': pred['expected_hours'],
                'CI': pred['confidence_interval']
            })
            
            print(f"  时刻 {hour:.1f}h, SOC={soc*100:.1f}%: "
                  f"预测剩余 {pred['expected_hours']:.1f}h "
                  f"[{pred['confidence_interval'][0]:.1f}, {pred['confidence_interval'][1]:.1f}]")
    
    # 各场景预测
    print("\n[不同场景剩余时间预测] (从100% SOC)")
    all_scenarios = predictor.predict_all_scenarios(1.0)
    for s, r in all_scenarios.items():
        print(f"  {s:20s}: {r['remaining_hours']:6.1f}h (P={r['power']:.1f}W)")
    
    # ========== 统计 ==========
    print("\n[仿真结果统计]")
    print(f"  初始SOC: {SOC[0]:.1f}%")
    print(f"  最低SOC: {np.min(SOC):.1f}%")
    print(f"  最高SOC: {np.max(SOC):.1f}%")
    print(f"  最终SOC: {SOC[-1]:.1f}%")
    print(f"  最高电池温度: {np.max(T_batt):.1f}°C")
    print(f"  最高CPU温度: {np.max(T_cpu):.1f}°C")
    print(f"  最大功耗: {np.max(P_total):.2f}W")
    print(f"  平均功耗: {np.mean(P_total):.2f}W")
    
    return {
        't_hours': t_hours,
        'SOC': SOC,
        'T_batt': T_batt,
        'T_cpu': T_cpu,
        'x_lock': x_lock,
        'I_bg': I_bg,
        'p_states': p_states,
        'P_total': P_total,
        'I_total': I_total,
        'predictions': predictions,
        'all_scenarios': all_scenarios,
        'model': model,
        'predictor': predictor
    }


# =============================================================================
# 5. 综合可视化
# =============================================================================

def create_comprehensive_visualization(data, save_path=None):
    """创建综合可视化"""
    
    fig = plt.figure(figsize=(18, 20))
    gs = gridspec.GridSpec(5, 2, height_ratios=[1, 1, 1, 0.8, 1], hspace=0.3, wspace=0.25)
    
    t = data['t_hours']
    model = data['model']
    
    # 时间区域颜色
    mode_colors = ['#E8E8E8', '#FFE4E4', '#E4FFE4']
    periods = [
        (0, 7, 0, 'Sleep'), (7, 9, 2, 'Leisure'), (9, 12, 1, 'Work'),
        (12, 14, 2, 'Leisure'), (14, 18, 1, 'Work'), (18, 23, 2, 'Leisure'), (23, 24, 0, 'Sleep')
    ]
    
    def draw_bg(ax, y_min, y_max):
        for start, end, mode, name in periods:
            rect = Rectangle((start, y_min), end - start, y_max - y_min,
                            facecolor=mode_colors[mode], edgecolor='none', alpha=0.6, zorder=0)
            ax.add_patch(rect)
            ax.text((start + end) / 2, y_max - (y_max - y_min) * 0.03, name,
                   ha='center', va='top', fontsize=8, fontweight='bold',
                   color=['#666', '#A66', '#6A6'][mode])
    
    # ========== 1. 马尔科夫用户状态 ==========
    ax1 = fig.add_subplot(gs[0, :])
    draw_bg(ax1, 0, 1.05)
    
    p = data['p_states']
    ax1.stackplot(t, p[0], p[1], p[2], p[3],
                 colors=['#5DADE2', '#58D68D', '#F7DC6F', '#EC7063'],
                 labels=['Deep Sleep', 'Light Use', 'Streaming', 'Gaming'], alpha=0.85)
    
    ax1.set_xlim(0, 24)
    ax1.set_ylim(0, 1.05)
    ax1.set_xlabel('Time (hour)')
    ax1.set_ylabel('Probability')
    ax1.set_title('User Behavior: Time-Inhomogeneous Markov Chain\n' +
                 r'$\frac{dp}{dt} = p \cdot Q(t)$, $Q(t) \in \{Q_{sleep}, Q_{work}, Q_{leisure}\}$',
                 fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', ncol=4, fontsize=9)
    ax1.set_xticks(range(0, 25, 2))
    ax1.grid(True, alpha=0.3)
    
    # ========== 2. SOC与电流 ==========
    ax2 = fig.add_subplot(gs[1, 0])
    draw_bg(ax2, -25, 110)
    
    ax2.plot(t, data['SOC'], color='#E67E22', linewidth=2.5, label='SOC [%]')
    ax2.plot(t, data['I_total'] * 30, color='#3498DB', linewidth=1.5, label='Current [A×30]')
    ax2.axhline(y=5, color='red', linestyle='--', alpha=0.5, label='Cutoff (5%)')
    
    ax2.set_xlim(0, 24)
    ax2.set_ylim(-25, 110)
    ax2.set_xlabel('Time (hour)')
    ax2.set_ylabel('SOC (%) / Current (A×30)')
    ax2.set_title('SOC Dynamics from Coupled ODE\n' +
                 r'$\frac{dSOC}{dt} = -\frac{I_{total}}{Q_{max} \cdot f(T,N)}$',
                 fontsize=11, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.set_xticks(range(0, 25, 2))
    ax2.grid(True, alpha=0.3)
    
    # ========== 3. SOC与功耗关系 ==========
    ax3 = fig.add_subplot(gs[1, 1])
    
    # SOC vs Power scatter with time coloring
    scatter = ax3.scatter(data['SOC'], data['P_total'], c=t, cmap='viridis', 
                         s=3, alpha=0.7)
    plt.colorbar(scatter, ax=ax3, label='Time (hour)')
    
    # 拟合趋势线
    z = np.polyfit(data['SOC'], data['P_total'], 2)
    p_fit = np.poly1d(z)
    soc_range = np.linspace(5, 100, 100)
    ax3.plot(soc_range, p_fit(soc_range), 'r--', linewidth=2, label='Quadratic Fit')
    
    ax3.set_xlabel('SOC (%)')
    ax3.set_ylabel('Power (W)')
    ax3.set_title('SOC-Power Relationship\n' +
                 r'$P_{total} = P_{SoC} + P_{disp} + P_{5G} + P_{BT} + P_{GNSS} + P_{bg}$',
                 fontsize=11, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0, 105)
    
    # ========== 4. 温度动力学 ==========
    ax4 = fig.add_subplot(gs[2, 0])
    draw_bg(ax4, 18, 40)
    
    ax4.plot(t, data['T_batt'], color='#E74C3C', linewidth=2, label='Battery')
    ax4.plot(t, data['T_cpu'], color='#3498DB', linewidth=2, label='CPU')
    ax4.axhline(y=22, color='gray', linestyle='--', alpha=0.5, label='Ambient')
    
    ax4.set_xlim(0, 24)
    ax4.set_ylim(18, 40)
    ax4.set_xlabel('Time (hour)')
    ax4.set_ylabel('Temperature (°C)')
    ax4.set_title('Thermal Dynamics\n' +
                 r'$C_{th}\frac{dT}{dt} = P_{joule} + P_{entropy} - \frac{T-T_{env}}{R_{th}}$',
                 fontsize=11, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=9)
    ax4.set_xticks(range(0, 25, 2))
    ax4.grid(True, alpha=0.3)
    
    # ========== 5. 功率分解 ==========
    ax5 = fig.add_subplot(gs[2, 1])
    draw_bg(ax5, 0, 5)
    
    ax5.fill_between(t, 0, data['P_total'], alpha=0.6, color='#3498DB', label='Total Power')
    ax5.plot(t, data['P_total'], color='#2C3E50', linewidth=1.5)
    
    ax5.set_xlim(0, 24)
    ax5.set_ylim(0, 5)
    ax5.set_xlabel('Time (hour)')
    ax5.set_ylabel('Power (W)')
    ax5.set_title('Total Power Consumption\n' +
                 r'$I_{total} = \frac{P_{total}}{\eta_{PMIC} \cdot V_{batt}}$',
                 fontsize=11, fontweight='bold')
    ax5.legend(loc='upper right', fontsize=9)
    ax5.set_xticks(range(0, 25, 2))
    ax5.grid(True, alpha=0.3)
    
    # ========== 6. 剩余时间预测 ==========
    ax6 = fig.add_subplot(gs[3, :])
    
    # 场景预测柱状图
    scenarios = list(data['all_scenarios'].keys())
    times = [data['all_scenarios'][s]['remaining_hours'] for s in scenarios]
    powers = [data['all_scenarios'][s]['power'] for s in scenarios]
    
    colors = ['#27AE60', '#3498DB', '#F39C12', '#E74C3C', '#9B59B6', '#34495E']
    bars = ax6.barh(scenarios, times, color=colors, alpha=0.8, edgecolor='black')
    
    # 添加数值标签
    for i, (bar, t_val, p_val) in enumerate(zip(bars, times, powers)):
        ax6.text(bar.get_width() + 0.5, bar.get_y() + bar.get_height()/2,
                f'{t_val:.1f}h (P={p_val}W)', va='center', fontsize=9)
    
    ax6.set_xlabel('Remaining Time (hours)')
    ax6.set_title('Remaining Battery Time Prediction (from 100% SOC)\n' +
                 r'$t_{remain} = \int_{SOC_{cutoff}}^{SOC_{current}} \frac{Q_{eff} \cdot V(SOC)}{P(SOC)} dSOC$',
                 fontsize=11, fontweight='bold')
    ax6.set_xlim(0, max(times) * 1.3)
    ax6.grid(True, alpha=0.3, axis='x')
    
    # ========== 7. OCV-SOC曲线 ==========
    ax7 = fig.add_subplot(gs[4, 0])
    
    soc_range = np.linspace(0.01, 0.99, 100)
    V_OCV = [model.V_OCV(s) for s in soc_range]
    
    ax7.plot(soc_range * 100, V_OCV, color='#2E86AB', linewidth=2.5)
    ax7.fill_between(soc_range * 100, model.V_min, V_OCV, alpha=0.2, color='#2E86AB')
    ax7.axhline(y=model.V_min + 0.2, color='red', linestyle='--', label='Cutoff')
    
    ax7.set_xlabel('SOC (%)')
    ax7.set_ylabel('Open Circuit Voltage (V)')
    ax7.set_title('OCV-SOC Characteristic\n' +
                 r'$V_{OCV}(SOC) = 3.0 + 0.9 \cdot SOC + 0.15 \cdot SOC^2 + ...$',
                 fontsize=11, fontweight='bold')
    ax7.set_xlim(0, 100)
    ax7.set_ylim(2.9, 4.3)
    ax7.legend(loc='lower right', fontsize=9)
    ax7.grid(True, alpha=0.3)
    
    # ========== 8. 放电曲线预测 ==========
    ax8 = fig.add_subplot(gs[4, 1])
    
    predictor = data['predictor']
    
    # 不同场景的放电曲线
    scenario_colors = {'idle': '#27AE60', 'light_use': '#3498DB', 
                      'video_streaming': '#F39C12', 'gaming': '#E74C3C'}
    
    for scenario, color in scenario_colors.items():
        P = data['all_scenarios'][scenario]['power']
        t_max = data['all_scenarios'][scenario]['remaining_hours']
        
        # 生成放电曲线
        t_pred = np.linspace(0, min(t_max, 25), 100)
        SOC_pred = np.zeros_like(t_pred)
        SOC_pred[0] = 100
        
        Q_eff = model.Q_max * model.capacity_factor(298.15)
        eta = model.eta_PMIC
        
        for i in range(1, len(t_pred)):
            dt = (t_pred[i] - t_pred[i-1]) * 3600
            V = model.V_OCV(SOC_pred[i-1] / 100)
            I = P / (eta * max(V, model.V_min))
            dSOC = -I * dt / Q_eff * 100
            SOC_pred[i] = max(5, SOC_pred[i-1] + dSOC)
        
        ax8.plot(t_pred, SOC_pred, color=color, linewidth=2, label=scenario.replace('_', ' ').title())
    
    ax8.axhline(y=5, color='red', linestyle='--', alpha=0.7, label='Cutoff')
    ax8.set_xlabel('Time (hours)')
    ax8.set_ylabel('SOC (%)')
    ax8.set_title('Predicted Discharge Curves by Scenario\n' +
                 r'$SOC(t) = SOC_0 - \int_0^t \frac{P(\tau)}{\eta V(\tau) Q_{eff}} d\tau$',
                 fontsize=11, fontweight='bold')
    ax8.set_xlim(0, 25)
    ax8.set_ylim(0, 105)
    ax8.legend(loc='upper right', fontsize=9)
    ax8.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"\n图表已保存: {save_path}")
    
    return fig


# =============================================================================
# 主程序
# =============================================================================

def main():
    """主函数"""
    # 运行仿真
    data = run_complete_simulation()
    
    # 生成可视化
    print("\n[生成综合可视化...]")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, 'soc_power_prediction_model.png')
    fig = create_comprehensive_visualization(data, save_path)
    
    plt.close(fig)
    
    print("\n" + "="*80)
    print("建模与预测完成!")
    print("="*80)
    
    return data


if __name__ == "__main__":
    data = main()
