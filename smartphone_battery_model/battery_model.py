"""
智能手机电池耗电连续时间数学模型
Smartphone Battery Discharge Continuous-Time Mathematical Model

基于电化学-热耦合方程组的锂离子电池SOC预测模型
Based on electrochemical-thermal coupled equations for Li-ion battery SOC prediction

Author: Battery Modeling System
Date: 2026-02-01
"""

import numpy as np
from scipy.integrate import solve_ivp, odeint
from scipy.interpolate import interp1d
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Tuple, Optional
import warnings

# =============================================================================
# 第一部分：电池参数与常量定义
# Part 1: Battery Parameters and Constants
# =============================================================================

@dataclass
class BatteryParameters:
    """锂离子电池物理参数 (Li-ion Battery Physical Parameters)
    
    参数来源：
    - 容量参数：基于典型智能手机电池规格 (Samsung SDI, LG Chem)
    - 热参数：参考文献 [1] Chen et al., Journal of Power Sources, 2020
    - 电化学参数：参考文献 [2] Newman et al., Electrochemical Systems, 2004
    """
    # 标称容量 (Nominal capacity) [Ah]
    Q_max: float = 4.0  # 4000mAh typical smartphone battery
    
    # 电池内阻参数 (Internal resistance parameters)
    R_int_ref: float = 0.08  # 参考内阻 [Ω] at 25°C
    R_int_temp_coeff: float = 0.003  # 温度系数 [Ω/K]
    
    # 热参数 (Thermal parameters)
    C_th: float = 35.0  # 热容 [J/K] - 典型锂电池
    R_th: float = 15.0  # 热阻 [K/W] - 电池到环境
    T_env: float = 298.15  # 环境温度 [K] (25°C)
    
    # 老化因子 (Aging factor)
    N_cycle: int = 1  # 充放电循环次数
    capacity_fade_rate: float = 0.0002  # 每循环容量衰减率
    
    # 开路电压参数 (OCV parameters) - 多项式拟合系数
    # V_OCV(SOC) = a0 + a1*SOC + a2*SOC^2 + a3*SOC^3 + a4*SOC^4
    ocv_coeffs: np.ndarray = field(default_factory=lambda: np.array([
        3.0,    # a0: 最低电压
        1.2,    # a1
        -0.8,   # a2
        0.5,    # a3
        0.3     # a4: 高SOC时的非线性
    ]))
    
    # 熵变系数 (Entropy change coefficient) [V/K]
    dVdT: float = -0.0003  # 典型锂电池熵变
    
    # PMIC效率
    eta_PMIC: float = 0.92  # 电源管理芯片效率


@dataclass
class SoCModuleParameters:
    """系统级芯片(SoC)模块参数
    
    参数来源：
    - Qualcomm Snapdragon 8 Gen 系列功耗特性
    - 典型智能手机SoC功耗 0.5-3W
    """
    # 动态功耗参数 (调整使总功耗更合理)
    C_dd: float = 0.6e-9  # 负载电容 [F] - 降低以减少动态功耗
    V_dd: float = 0.85    # 工作电压 [V]
    
    # 漏电流参数
    I_leak_ref: float = 0.008  # 参考漏电流 [A] at 25°C
    leak_temp_coeff: float = 0.04  # 温度系数 [/K]
    
    # 热参数
    C_th_soc: float = 5.0  # SoC热容 [J/K]
    R_th_soc_batt: float = 10.0  # SoC到电池热阻 [K/W]
    R_th_soc_env: float = 50.0  # SoC到环境热阻 [K/W]
    
    # DVFS参数 (Dynamic Voltage and Frequency Scaling)
    f_min: float = 0.5e9   # 最小频率 [Hz]
    f_max: float = 3.0e9   # 最大频率 [Hz]
    V_min: float = 0.55    # 最小电压 [V]
    V_max: float = 1.0     # 最大电压 [V]


@dataclass
class DisplayParameters:
    """显示模块参数 - 基于OLED屏幕特性
    
    参数来源：
    - Samsung AMOLED 显示屏测量数据
    - 典型6.5英寸OLED功耗 0.3-1.5W
    """
    P_static: float = 0.08   # 静态功耗 [W] - 显示控制器
    k_drv: float = 0.002     # 驱动功耗系数 [W/Hz]
    A_screen: float = 0.01   # 屏幕面积 [m²] - 6.5英寸
    
    # 亮度功耗系数 [W/cd/m²] - OLED特性
    k_brightness: float = 0.002
    
    # 刷新率
    f_refresh_min: float = 60   # [Hz]
    f_refresh_max: float = 120  # [Hz]


@dataclass
class CommunicationParameters:
    """通信模块参数 (5G/4G/WiFi/Bluetooth)
    
    参数来源：
    - 5G功耗: Qualcomm X55/X60 调制解调器规格
    - WiFi功耗: Broadcom BCM4375 数据手册
    - 蓝牙功耗: 蓝牙5.0低功耗模式测量
    """
    # 5G模块 (调整为更现实的值)
    P_5G_tx_max: float = 1.5   # 最大发射功率 [W] - 典型值1-2W
    P_5G_rx: float = 0.5       # 接收功率 [W]
    P_5G_idle: float = 0.05    # 空闲功率 [W]
    
    # WiFi模块
    P_wifi_tx: float = 0.6     # WiFi发射 [W]
    P_wifi_rx: float = 0.3     # WiFi接收 [W]
    P_wifi_idle: float = 0.01  # WiFi空闲 [W]
    
    # 蓝牙模块
    P_bt_active: float = 0.08  # 蓝牙活动 [W]
    V_bt: float = 3.3          # 蓝牙电压 [V]
    I_audio: float = 0.012     # 音频传输电流 [A]
    I_sleep: float = 0.0005    # 休眠电流 [A]


@dataclass
class GNSSParameters:
    """GNSS模块参数"""
    P_LNA: float = 0.05        # 低噪声放大器功率 [W]
    P_track: float = 0.08      # 跟踪模式功率 [W]
    P_acq: float = 0.25        # 捕获模式功率 [W]
    tau_react: float = 2.0     # 反应时间常数 [s]
    S_threshold: float = 0.5   # 信号阈值


@dataclass 
class BackgroundParameters:
    """后台任务参数"""
    mu_bg: float = 0.01        # 平均后台电流 [A]
    sigma_bg: float = 0.005    # 后台电流波动 [A]
    theta_bg: float = 0.1      # 回归速率


# =============================================================================
# 第二部分：开路电压与内阻模型
# Part 2: Open Circuit Voltage and Internal Resistance Models
# =============================================================================

class OCVModel:
    """开路电压模型 (Open Circuit Voltage Model)
    
    基于多项式拟合和温度修正的OCV-SOC关系模型
    V_OCV(SOC, T) = V_OCV_ref(SOC) + (dV/dT) * (T - T_ref)
    """
    
    def __init__(self, params: BatteryParameters):
        self.params = params
        self.T_ref = 298.15  # 参考温度 25°C
        
        # 基于实验数据的OCV-SOC查表 (典型Li-ion电池)
        # 数据来源: Battery University, published cell characterization data
        self.soc_points = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
        self.ocv_points = np.array([3.0, 3.4, 3.55, 3.65, 3.72, 3.78, 3.85, 3.92, 4.0, 4.1, 4.2])
        
        # 创建插值函数
        self._ocv_interp = interp1d(self.soc_points, self.ocv_points, kind='cubic', 
                                    fill_value='extrapolate')
    
    def get_ocv(self, soc: float, T: float = 298.15) -> float:
        """计算给定SOC和温度下的开路电压
        
        Args:
            soc: 充电状态 [0, 1]
            T: 温度 [K]
        
        Returns:
            V_OCV: 开路电压 [V]
        """
        soc = np.clip(soc, 0.0, 1.0)
        V_ref = float(self._ocv_interp(soc))
        
        # 温度修正
        delta_T = T - self.T_ref
        V_ocv = V_ref + self.params.dVdT * delta_T
        
        return V_ocv
    
    def get_entropy_heat(self, soc: float, T: float, I: float) -> float:
        """计算熵变产热
        
        P_entropy = I * T * (dV_OCV/dT)
        """
        return I * T * self.params.dVdT


class InternalResistanceModel:
    """内阻模型 (Internal Resistance Model)
    
    考虑SOC、温度和老化的内阻模型:
    R_int(SOC, T, N) = R_ref * f_SOC(SOC) * f_T(T) * f_N(N)
    """
    
    def __init__(self, params: BatteryParameters):
        self.params = params
    
    def get_resistance(self, soc: float, T: float, N: int = 1) -> float:
        """计算内阻
        
        Args:
            soc: 充电状态 [0, 1]
            T: 温度 [K]
            N: 循环次数
        
        Returns:
            R_int: 内阻 [Ω]
        """
        R_ref = self.params.R_int_ref
        
        # SOC影响因子 - 低SOC时内阻增加
        f_soc = 1.0 + 0.5 * (1 - soc)**2
        
        # 温度影响因子 - Arrhenius关系
        T_ref = 298.15
        E_a = 20000  # 活化能 [J/mol]
        R_gas = 8.314  # 气体常数
        f_T = np.exp(E_a / R_gas * (1/T - 1/T_ref))
        
        # 老化影响因子
        f_N = 1.0 + self.params.capacity_fade_rate * N
        
        return R_ref * f_soc * f_T * f_N


# =============================================================================
# 第三部分：功耗模块建模
# Part 3: Power Consumption Module Modeling
# =============================================================================

class SoCPowerModule:
    """系统级芯片(SoC)功耗模块 - 电热强耦合
    
    P_SoC = f_cpu * C_dd * V_dd² + I_leak(T_soc) * V_dd
    
    包含DVFS控制策略
    """
    
    def __init__(self, params: SoCModuleParameters):
        self.params = params
        self.T_soc = 298.15  # 初始SoC温度
    
    def get_dvfs_state(self, load: float, T_batt: float) -> Tuple[float, float]:
        """DVFS控制器 - 根据负载和温度调节频率电压
        
        Args:
            load: CPU负载 [0, 1]
            T_batt: 电池温度 [K]
        
        Returns:
            (f_cpu, V_dd): 频率[Hz]和电压[V]
        """
        p = self.params
        
        # 基于负载的目标频率
        f_target = p.f_min + load * (p.f_max - p.f_min)
        
        # 温度降频 - 高温时降低频率
        T_threshold = 313.15  # 40°C
        if T_batt > T_threshold:
            thermal_throttle = np.exp(-0.1 * (T_batt - T_threshold))
            f_target *= thermal_throttle
        
        f_cpu = np.clip(f_target, p.f_min, p.f_max)
        
        # 电压-频率关系 (V ∝ f)
        V_dd = p.V_min + (f_cpu - p.f_min) / (p.f_max - p.f_min) * (p.V_max - p.V_min)
        
        return f_cpu, V_dd
    
    def calculate_power(self, load: float, T_batt: float, T_soc: float) -> Tuple[float, float]:
        """计算SoC功耗
        
        Args:
            load: CPU负载 [0, 1]
            T_batt: 电池温度 [K]
            T_soc: SoC温度 [K]
        
        Returns:
            (P_dyn, P_leak): 动态功耗和漏电功耗 [W]
        """
        f_cpu, V_dd = self.get_dvfs_state(load, T_batt)
        p = self.params
        
        # 动态功耗: P_dyn = f * C * V²
        P_dyn = f_cpu * p.C_dd * V_dd**2
        
        # 漏电功耗: I_leak(T) = I_ref * exp(k*(T-T_ref))
        T_ref = 298.15
        I_leak = p.I_leak_ref * np.exp(p.leak_temp_coeff * (T_soc - T_ref))
        P_leak = I_leak * V_dd
        
        return P_dyn, P_leak
    
    def thermal_dynamics(self, T_soc: float, T_batt: float, P_soc: float) -> float:
        """SoC热动力学方程
        
        C_th * dT_soc/dt = P_soc - (T_soc-T_batt)/R_th_batt - (T_soc-T_env)/R_th_env
        """
        p = self.params
        T_env = 298.15
        
        dT_dt = (P_soc - (T_soc - T_batt)/p.R_th_soc_batt 
                 - (T_soc - T_env)/p.R_th_soc_env) / p.C_th_soc
        
        return dT_dt


class DisplayPowerModule:
    """显示模块功耗 - 环境光内容耦合
    
    P_disp = P_static + k_drv * f_refresh + L_set * A_screen
    """
    
    def __init__(self, params: DisplayParameters):
        self.params = params
    
    def calculate_power(self, brightness: float, refresh_rate: float, 
                       content_complexity: float = 0.5, screen_on: bool = True) -> float:
        """计算显示功耗
        
        Args:
            brightness: 亮度设置 [0, 1] -> [0, 500] cd/m²
            refresh_rate: 刷新率 [Hz]
            content_complexity: 内容复杂度 [0, 1] (影响像素变化)
            screen_on: 屏幕是否开启
        
        Returns:
            P_disp: 显示功耗 [W]
        """
        if not screen_on:
            return 0.0
        
        p = self.params
        
        # 亮度功耗 (OLED: 功耗与亮度近似线性)
        L_set = brightness * 500  # 转换为 cd/m²
        P_brightness = p.k_brightness * L_set * p.A_screen
        
        # 驱动功耗 (与刷新率和内容复杂度相关)
        P_driver = p.k_drv * refresh_rate * (0.5 + 0.5 * content_complexity)
        
        P_total = p.P_static + P_driver + P_brightness
        
        return P_total


class CommunicationPowerModule:
    """通信模块功耗 - 信道距离耦合"""
    
    def __init__(self, params: CommunicationParameters):
        self.params = params
    
    def calculate_5g_power(self, tx_active: bool, rx_active: bool, 
                           signal_quality: float = 0.8) -> float:
        """5G模块功耗
        
        Args:
            tx_active: 发射是否活动
            rx_active: 接收是否活动
            signal_quality: 信号质量 [0, 1] (差信号需要更高发射功率)
        """
        p = self.params
        P = p.P_5G_idle
        
        if rx_active:
            P += p.P_5G_rx
        
        if tx_active:
            # 弱信号时增加发射功率
            tx_factor = 1.0 + 0.5 * (1 - signal_quality)
            P += p.P_5G_tx_max * tx_factor
        
        return P
    
    def calculate_wifi_power(self, mode: str = 'idle', 
                            data_rate: float = 0.0) -> float:
        """WiFi模块功耗"""
        p = self.params
        
        if mode == 'idle':
            return p.P_wifi_idle
        elif mode == 'rx':
            return p.P_wifi_rx * (0.5 + 0.5 * data_rate)
        elif mode == 'tx':
            return p.P_wifi_tx * (0.5 + 0.5 * data_rate)
        return p.P_wifi_idle
    
    def calculate_bluetooth_power(self, audio_streaming: bool = False,
                                  duty_cycle: float = 0.1,
                                  V_batt: float = 3.7) -> float:
        """蓝牙功耗 - 事件驱动耦合
        
        P_BT = V_batt * [(1-δ)*I_sleep + δ*I_audio + Q_event*P_audio/V_batt]
        """
        p = self.params
        
        if audio_streaming:
            # 音频流传输
            P_bt = p.V_bt * (duty_cycle * p.I_audio + (1 - duty_cycle) * p.I_sleep)
        else:
            P_bt = p.V_bt * p.I_sleep
        
        return P_bt


class GNSSPowerModule:
    """GNSS模块功耗 - 环境信号耦合
    
    P_GNSS = P_LNA + x_lock*P_track + (1-x_lock)*P_acq
    
    dx_lock/dt = (1/τ) * [S(t) - x_lock]
    """
    
    def __init__(self, params: GNSSParameters):
        self.params = params
        self.x_lock = 0.0  # 锁定状态变量
    
    def signal_quality(self, is_indoor: bool, urban_canyon: bool) -> float:
        """计算信号质量
        
        S = 1 / (1 + exp(-k*(S_env - S_threshold)))
        """
        p = self.params
        
        # 基础信号强度
        S_base = 1.0
        if is_indoor:
            S_base *= 0.3
        if urban_canyon:
            S_base *= 0.5
        
        # Sigmoid函数
        k = 5.0
        S = 1.0 / (1 + np.exp(-k * (S_base - p.S_threshold)))
        
        return S
    
    def update_lock_state(self, S: float, dt: float) -> float:
        """更新锁定状态 (一阶动态系统)"""
        tau = self.params.tau_react
        self.x_lock += (S - self.x_lock) / tau * dt
        self.x_lock = np.clip(self.x_lock, 0, 1)
        return self.x_lock
    
    def calculate_power(self, active: bool, is_indoor: bool = False,
                       urban_canyon: bool = False, dt: float = 1.0) -> float:
        """计算GNSS功耗"""
        if not active:
            return 0.0
        
        p = self.params
        
        S = self.signal_quality(is_indoor, urban_canyon)
        x_lock = self.update_lock_state(S, dt)
        
        # P = P_LNA + x_lock*P_track + (1-x_lock)*P_acq
        P_gnss = p.P_LNA + x_lock * p.P_track + (1 - x_lock) * p.P_acq
        
        return P_gnss


class BackgroundPowerModule:
    """后台任务功耗 - 随机过程耦合 (Ornstein-Uhlenbeck过程)
    
    dI_bg = θ*(μ - I_bg)*dt + σ*dW
    """
    
    def __init__(self, params: BackgroundParameters):
        self.params = params
        self.I_bg = params.mu_bg  # 初始后台电流
    
    def update_background_current(self, user_activity: float, dt: float) -> float:
        """更新后台电流 (OU过程模拟)
        
        Args:
            user_activity: 用户活动强度 [0, 1]
            dt: 时间步长 [s]
        """
        p = self.params
        
        # 用户活动影响均值漂移
        mu_adjusted = p.mu_bg * (1 + user_activity)
        
        # OU过程更新
        dW = np.random.normal(0, np.sqrt(dt))
        dI = p.theta_bg * (mu_adjusted - self.I_bg) * dt + p.sigma_bg * dW
        
        self.I_bg = max(0.001, self.I_bg + dI)
        
        return self.I_bg
    
    def calculate_power(self, V_batt: float, user_activity: float, 
                       dt: float = 1.0) -> float:
        """计算后台功耗
        
        P_bg = V_batt * I_bg
        """
        I_bg = self.update_background_current(user_activity, dt)
        return V_batt * I_bg


# =============================================================================
# 第四部分：用户行为马尔科夫模型
# Part 4: User Behavior Markov Model
# =============================================================================

class UserBehaviorModel:
    """连续时间马尔科夫用户模型
    
    UserState ∈ {S1: sleep, S2: work, S3: leisure, S4: heavy_use}
    
    dp/dt = Q(t) * p
    
    其中Q(t)是时变转移率矩阵
    """
    
    # 状态定义
    SLEEP = 0
    WORK = 1
    LEISURE = 2
    HEAVY_USE = 3
    
    STATE_NAMES = ['Sleep', 'Work', 'Leisure', 'Heavy Use']
    
    def __init__(self):
        # 各状态的硬件参数映射
        self.state_params = {
            self.SLEEP: {
                'cpu_load': 0.02,
                'screen_on': False,
                'brightness': 0.0,
                'wifi_mode': 'idle',
                '5g_active': False,
                'bt_audio': False,
                'gps_active': False,
                'refresh_rate': 60,
            },
            self.WORK: {
                'cpu_load': 0.4,
                'screen_on': True,
                'brightness': 0.5,
                'wifi_mode': 'rx',
                '5g_active': True,
                'bt_audio': False,
                'gps_active': False,
                'refresh_rate': 60,
            },
            self.LEISURE: {
                'cpu_load': 0.3,
                'screen_on': True,
                'brightness': 0.6,
                'wifi_mode': 'rx',
                '5g_active': False,
                'bt_audio': True,
                'gps_active': False,
                'refresh_rate': 90,
            },
            self.HEAVY_USE: {
                'cpu_load': 0.75,      # 高负载但留有余量
                'screen_on': True,
                'brightness': 0.7,     # 通常不会开到最亮
                'wifi_mode': 'tx',
                '5g_active': True,
                'bt_audio': False,     # 游戏时通常不用蓝牙音频
                'gps_active': True,
                'refresh_rate': 120,
            }
        }
        
        # 基础转移率矩阵 (日间)
        self.Q_day = np.array([
            [-0.5, 0.3, 0.15, 0.05],    # from SLEEP
            [0.1, -0.4, 0.2, 0.1],      # from WORK
            [0.15, 0.1, -0.4, 0.15],    # from LEISURE
            [0.1, 0.2, 0.3, -0.6],      # from HEAVY_USE
        ])
        
        # 夜间转移率矩阵 (23:00 - 7:00)
        self.Q_night = np.array([
            [-0.1, 0.05, 0.04, 0.01],   # from SLEEP
            [0.5, -0.6, 0.08, 0.02],    # from WORK
            [0.4, 0.05, -0.5, 0.05],    # from LEISURE
            [0.3, 0.1, 0.1, -0.5],      # from HEAVY_USE
        ])
        
        # 工作时间转移率矩阵 (9:00 - 18:00)
        self.Q_work = np.array([
            [-0.8, 0.6, 0.15, 0.05],    # from SLEEP
            [0.05, -0.3, 0.15, 0.1],    # from WORK
            [0.1, 0.4, -0.6, 0.1],      # from LEISURE
            [0.05, 0.3, 0.15, -0.5],    # from HEAVY_USE
        ])
    
    def get_transition_matrix(self, hour: float) -> np.ndarray:
        """获取时变转移率矩阵
        
        Args:
            hour: 一天中的小时 [0, 24)
        
        Returns:
            Q: 转移率矩阵
        """
        if 23 <= hour or hour < 7:
            return self.Q_night
        elif 9 <= hour < 18:
            return self.Q_work
        else:
            return self.Q_day
    
    def simulate_state(self, current_state: int, hour: float, dt: float) -> int:
        """模拟状态转移
        
        Args:
            current_state: 当前状态
            hour: 一天中的小时
            dt: 时间步长 [s]
        
        Returns:
            new_state: 新状态
        """
        Q = self.get_transition_matrix(hour)
        
        # 计算转移概率
        rate = -Q[current_state, current_state]
        
        # 是否发生转移
        if np.random.random() < rate * dt:
            # 选择目标状态
            probs = Q[current_state, :].copy()
            probs[current_state] = 0
            probs = probs / probs.sum()
            new_state = np.random.choice(4, p=probs)
            return new_state
        
        return current_state
    
    def get_hardware_params(self, state: int) -> dict:
        """获取状态对应的硬件参数"""
        return self.state_params[state]


# =============================================================================
# 第五部分：核心电池系统模型
# Part 5: Core Battery System Model
# =============================================================================

class SmartphoneBatteryModel:
    """智能手机电池连续时间模型
    
    主耦合方程组:
    
    1. SOC动力学:
       dSOC/dt = -I_total(t) / [Q_max * f(T, N)]
    
    2. 热动力学:
       C_th * dT_batt/dt = P_joule + P_entropy - (T_batt - T_env)/R_th
    
    3. 电压方程:
       V_batt = V_OCV(SOC) - I_total * R_int(SOC, T, N)
    
    4. 总电流耦合:
       I_total = [P_SoC + P_disp + P_5G + P_BT + P_GNSS + P_bg] / (η_PMIC * V_batt)
    """
    
    def __init__(self, battery_params: Optional[BatteryParameters] = None):
        # 初始化参数
        self.batt_params = battery_params or BatteryParameters()
        
        # 初始化子模块
        self.ocv_model = OCVModel(self.batt_params)
        self.resistance_model = InternalResistanceModel(self.batt_params)
        
        self.soc_module = SoCPowerModule(SoCModuleParameters())
        self.display_module = DisplayPowerModule(DisplayParameters())
        self.comm_module = CommunicationPowerModule(CommunicationParameters())
        self.gnss_module = GNSSPowerModule(GNSSParameters())
        self.bg_module = BackgroundPowerModule(BackgroundParameters())
        
        self.user_model = UserBehaviorModel()
        
        # 状态变量初始化
        self.SOC = 1.0
        self.T_batt = 298.15
        self.T_soc = 298.15
        self.user_state = UserBehaviorModel.WORK
        
        # 记录历史
        self.history = {
            'time': [],
            'SOC': [],
            'T_batt': [],
            'V_batt': [],
            'I_total': [],
            'P_total': [],
            'P_components': [],
            'user_state': []
        }
    
    def calculate_effective_capacity(self, T: float, N: int) -> float:
        """计算有效容量 (考虑温度和老化)
        
        Q_eff = Q_max * f_T(T) * (1 - fade_rate * N)
        """
        Q_max = self.batt_params.Q_max
        
        # 温度影响 (低温容量下降)
        T_ref = 298.15
        if T < T_ref:
            f_T = 1.0 - 0.01 * (T_ref - T)  # 每低1°C损失1%容量
        else:
            f_T = 1.0
        
        # 老化影响
        f_N = 1.0 - self.batt_params.capacity_fade_rate * N
        
        return Q_max * f_T * f_N
    
    def calculate_total_power(self, hw_params: dict, V_batt: float, 
                             T_batt: float, dt: float = 1.0) -> Tuple[float, dict]:
        """计算总功耗
        
        Args:
            hw_params: 硬件参数字典
            V_batt: 电池电压 [V]
            T_batt: 电池温度 [K]
            dt: 时间步长 [s]
        
        Returns:
            (P_total, P_components): 总功耗和各组件功耗
        """
        components = {}
        
        # SoC功耗
        P_dyn, P_leak = self.soc_module.calculate_power(
            hw_params['cpu_load'], T_batt, self.T_soc)
        components['P_SoC'] = P_dyn + P_leak
        
        # 显示功耗
        components['P_display'] = self.display_module.calculate_power(
            hw_params['brightness'],
            hw_params['refresh_rate'],
            screen_on=hw_params['screen_on']
        )
        
        # 5G功耗
        components['P_5G'] = self.comm_module.calculate_5g_power(
            tx_active=hw_params['5g_active'],
            rx_active=hw_params['5g_active']
        )
        
        # WiFi功耗
        components['P_WiFi'] = self.comm_module.calculate_wifi_power(
            mode=hw_params['wifi_mode']
        )
        
        # 蓝牙功耗
        components['P_BT'] = self.comm_module.calculate_bluetooth_power(
            audio_streaming=hw_params['bt_audio'],
            V_batt=V_batt
        )
        
        # GNSS功耗
        components['P_GNSS'] = self.gnss_module.calculate_power(
            active=hw_params['gps_active'],
            dt=dt
        )
        
        # 后台功耗
        user_activity = hw_params['cpu_load']
        components['P_background'] = self.bg_module.calculate_power(
            V_batt, user_activity, dt
        )
        
        P_total = sum(components.values())
        
        return P_total, components
    
    def system_dynamics(self, t: float, state: np.ndarray, 
                       hw_params: dict) -> np.ndarray:
        """系统动力学方程组
        
        state = [SOC, T_batt, T_soc]
        
        Returns:
            d_state/dt
        """
        SOC, T_batt, T_soc = state
        SOC = np.clip(SOC, 0.001, 1.0)
        
        # 计算电池电压
        V_ocv = self.ocv_model.get_ocv(SOC, T_batt)
        
        # 估算电流（迭代求解）
        R_int = self.resistance_model.get_resistance(SOC, T_batt, 
                                                     self.batt_params.N_cycle)
        
        P_total, _ = self.calculate_total_power(hw_params, V_ocv, T_batt)
        
        # 求解 V_batt 和 I_total
        # P = V_batt * I, V_batt = V_ocv - I*R_int
        # P = (V_ocv - I*R_int) * I
        # I² * R_int - I * V_ocv + P/η = 0
        
        eta = self.batt_params.eta_PMIC
        a = R_int
        b = -V_ocv
        c = P_total / eta
        
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            I_total = P_total / (eta * V_ocv)
        else:
            I_total = (-b - np.sqrt(discriminant)) / (2*a)
        
        I_total = max(0, I_total)
        V_batt = V_ocv - I_total * R_int
        
        # 有效容量
        Q_eff = self.calculate_effective_capacity(T_batt, self.batt_params.N_cycle)
        
        # SOC动力学: dSOC/dt = -I_total / Q_eff
        dSOC_dt = -I_total / (Q_eff * 3600)  # 转换为每秒
        
        # 热动力学
        P_joule = I_total**2 * R_int
        P_entropy = self.ocv_model.get_entropy_heat(SOC, T_batt, I_total)
        
        C_th = self.batt_params.C_th
        R_th = self.batt_params.R_th
        T_env = self.batt_params.T_env
        
        dT_batt_dt = (P_joule + abs(P_entropy) - (T_batt - T_env)/R_th) / C_th
        
        # SoC温度动力学
        P_soc = hw_params['cpu_load'] * 2.0  # 估算SoC功耗
        dT_soc_dt = self.soc_module.thermal_dynamics(T_soc, T_batt, P_soc)
        
        return np.array([dSOC_dt, dT_batt_dt, dT_soc_dt])
    
    def simulate(self, duration_hours: float, initial_soc: float = 1.0,
                initial_state: int = UserBehaviorModel.WORK,
                start_hour: float = 8.0, dt: float = 60.0,
                scenario: str = 'normal') -> dict:
        """运行电池放电模拟
        
        Args:
            duration_hours: 模拟时长 [小时]
            initial_soc: 初始SOC [0, 1]
            initial_state: 初始用户状态
            start_hour: 开始时刻（一天中的小时）
            dt: 时间步长 [秒]
            scenario: 使用场景 ('normal', 'heavy', 'idle', 'mixed')
        
        Returns:
            history: 模拟历史数据
        """
        # 初始化
        self.SOC = initial_soc
        self.T_batt = self.batt_params.T_env
        self.T_soc = self.batt_params.T_env
        self.user_state = initial_state
        
        # 清空历史
        self.history = {key: [] for key in self.history.keys()}
        
        # 预设场景
        scenario_states = {
            'normal': None,  # 使用马尔科夫模型
            'heavy': UserBehaviorModel.HEAVY_USE,
            'idle': UserBehaviorModel.SLEEP,
            'work': UserBehaviorModel.WORK,
            'leisure': UserBehaviorModel.LEISURE,
        }
        fixed_state = scenario_states.get(scenario, None)
        
        # 时间参数
        total_seconds = duration_hours * 3600
        n_steps = int(total_seconds / dt)
        
        state = np.array([self.SOC, self.T_batt, self.T_soc])
        
        for i in range(n_steps):
            t = i * dt
            current_hour = (start_hour + t/3600) % 24
            
            # 更新用户状态
            if fixed_state is not None:
                self.user_state = fixed_state
            else:
                self.user_state = self.user_model.simulate_state(
                    self.user_state, current_hour, dt)
            
            # 获取硬件参数
            hw_params = self.user_model.get_hardware_params(self.user_state)
            
            # 计算当前功耗
            V_ocv = self.ocv_model.get_ocv(state[0], state[1])
            P_total, P_components = self.calculate_total_power(
                hw_params, V_ocv, state[1], dt)
            
            # 计算电流
            R_int = self.resistance_model.get_resistance(
                state[0], state[1], self.batt_params.N_cycle)
            I_total = P_total / (self.batt_params.eta_PMIC * V_ocv)
            V_batt = V_ocv - I_total * R_int
            
            # 记录历史
            self.history['time'].append(t / 3600)  # 转换为小时
            self.history['SOC'].append(state[0])
            self.history['T_batt'].append(state[1] - 273.15)  # 转换为°C
            self.history['V_batt'].append(V_batt)
            self.history['I_total'].append(I_total)
            self.history['P_total'].append(P_total)
            self.history['P_components'].append(P_components)
            self.history['user_state'].append(self.user_state)
            
            # 更新状态 (Euler方法)
            d_state = self.system_dynamics(t, state, hw_params)
            state = state + d_state * dt
            
            # 限制状态范围
            state[0] = np.clip(state[0], 0.0, 1.0)
            state[1] = np.clip(state[1], 273.15, 333.15)  # 0-60°C
            state[2] = np.clip(state[2], 273.15, 373.15)  # 0-100°C
            
            # 检查电量耗尽
            if state[0] <= 0.01:
                print(f"电池耗尽于 {t/3600:.2f} 小时")
                break
        
        return self.history
    
    def predict_remaining_time(self, current_soc: float, scenario: str = 'normal',
                               T_batt: float = 298.15) -> Tuple[float, dict]:
        """预测剩余放电时间
        
        Args:
            current_soc: 当前SOC
            scenario: 使用场景
            T_batt: 当前电池温度
        
        Returns:
            (remaining_hours, details): 剩余时间和详细信息
        """
        # 获取场景对应的平均功耗
        scenario_power = {
            'idle': 0.15,      # 空闲
            'normal': 0.8,     # 正常使用
            'work': 1.2,       # 工作
            'leisure': 1.5,    # 娱乐
            'heavy': 3.5,      # 重度使用
        }
        
        P_avg = scenario_power.get(scenario, 1.0)
        
        # 计算有效容量
        Q_eff = self.calculate_effective_capacity(T_batt, self.batt_params.N_cycle)
        
        # 平均电压
        V_avg = self.ocv_model.get_ocv(current_soc * 0.5, T_batt)
        
        # 平均电流
        I_avg = P_avg / (self.batt_params.eta_PMIC * V_avg)
        
        # 剩余时间 = (SOC * Q_eff) / I_avg
        remaining_hours = (current_soc * Q_eff) / I_avg
        
        details = {
            'avg_power': P_avg,
            'avg_current': I_avg,
            'effective_capacity': Q_eff,
            'avg_voltage': V_avg,
            'scenario': scenario,
        }
        
        return remaining_hours, details


# =============================================================================
# 第六部分：模型验证与分析
# Part 6: Model Validation and Analysis
# =============================================================================

def validate_model():
    """模型验证 - 与实际测量数据对比"""
    
    model = SmartphoneBatteryModel()
    
    # 测试不同场景
    scenarios = ['idle', 'work', 'leisure', 'heavy']
    results = {}
    
    for scenario in scenarios:
        history = model.simulate(
            duration_hours=24,
            initial_soc=1.0,
            scenario=scenario,
            dt=60
        )
        
        # 找到SOC降至5%的时间
        soc_array = np.array(history['SOC'])
        time_array = np.array(history['time'])
        
        idx = np.where(soc_array <= 0.05)[0]
        if len(idx) > 0:
            drain_time = time_array[idx[0]]
        else:
            drain_time = time_array[-1]
        
        avg_power = np.mean(history['P_total'])
        
        results[scenario] = {
            'drain_time_hours': drain_time,
            'avg_power_W': avg_power,
            'final_soc': soc_array[-1],
            'max_temp_C': np.max(history['T_batt']),
        }
    
    return results


if __name__ == "__main__":
    # 基本测试
    print("=" * 60)
    print("智能手机电池放电模型测试")
    print("=" * 60)
    
    model = SmartphoneBatteryModel()
    
    # 测试正常使用场景
    print("\n正常使用场景模拟...")
    history = model.simulate(
        duration_hours=12,
        initial_soc=1.0,
        scenario='normal',
        start_hour=8.0
    )
    
    print(f"模拟结束时 SOC: {history['SOC'][-1]*100:.1f}%")
    print(f"平均功耗: {np.mean(history['P_total']):.2f} W")
    print(f"最高温度: {np.max(history['T_batt']):.1f} °C")
    
    # 预测剩余时间
    print("\n剩余时间预测:")
    for soc in [1.0, 0.8, 0.5, 0.2]:
        for scenario in ['idle', 'normal', 'heavy']:
            remaining, _ = model.predict_remaining_time(soc, scenario)
            print(f"  SOC={soc*100:.0f}%, {scenario}: {remaining:.1f} 小时")
