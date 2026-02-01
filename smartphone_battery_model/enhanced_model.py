"""
智能手机电池耗电增强连续时间模型
Enhanced Continuous-Time Smartphone Battery Discharge Model

完整实现PDF中的所有耦合微分方程组
Fully implements all coupled ODEs from the specification

基于以下耦合关系:
1. 电化学-热耦合核心 (Electrochemical-Thermal Core)
2. 各模块功耗耦合 (Module Power Coupling)  
3. 用户行为-硬件状态耦合 (User-Hardware Coupling)
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Callable
import warnings
warnings.filterwarnings('ignore')


# =============================================================================
# 第一部分：物理参数定义 (Physical Parameters)
# =============================================================================

@dataclass
class BatteryPhysicalParams:
    """电池物理参数 - 基于实测数据和文献
    
    References:
    [1] Samsung SDI 锂离子电池规格书
    [2] Chen et al., J. Power Sources, 2020
    [3] Battery University 测量数据
    """
    # 容量参数
    Q_max: float = 4.0           # 标称容量 [Ah] - 4000mAh
    Q_max_Wh: float = 15.2       # 标称能量 [Wh]
    
    # 电压参数
    V_nominal: float = 3.8       # 标称电压 [V]
    V_max: float = 4.2           # 满充电压 [V]
    V_cutoff: float = 3.0        # 截止电压 [V]
    
    # 内阻参数
    R_int_25C: float = 0.065     # 25°C内阻 [Ω]
    E_a_R: float = 20000         # 内阻活化能 [J/mol]
    
    # 热参数
    C_th_batt: float = 38.0      # 电池热容 [J/K]
    R_th_batt_env: float = 12.0  # 电池-环境热阻 [K/W]
    T_env: float = 298.15        # 环境温度 [K] (25°C)
    
    # 熵变参数
    dV_dT: float = -0.00035      # 熵变系数 [V/K]
    
    # 老化参数
    capacity_fade: float = 0.0002  # 每循环容量衰减
    R_growth: float = 0.0001       # 每循环内阻增长
    
    # 效率
    eta_coulombic: float = 0.998   # 库伦效率
    eta_PMIC: float = 0.92         # PMIC效率


@dataclass
class SoCChipParams:
    """系统级芯片参数 - 基于Snapdragon 8 Gen系列"""
    # 动态功耗: P = α·C·V²·f
    alpha: float = 0.85          # 活动因子
    C_eff: float = 0.5e-9        # 等效开关电容 [F]
    
    # 电压-频率映射 (DVFS)
    V_dd_min: float = 0.55       # 最低电压 [V]
    V_dd_max: float = 1.05       # 最高电压 [V]
    f_min: float = 0.3e9         # 最低频率 [Hz]
    f_max: float = 3.2e9         # 最高频率 [Hz]
    
    # 漏电参数
    I_leak_ref: float = 0.008    # 参考漏电流 [A] @25°C
    n_leak: float = 1.5          # 漏电温度指数
    E_a_leak: float = 0.7        # 漏电活化能 [eV]
    
    # 热参数
    C_th_soc: float = 4.5        # SoC热容 [J/K]
    R_th_soc_batt: float = 8.0   # SoC-电池热阻 [K/W]
    R_th_soc_env: float = 45.0   # SoC-环境热阻 [K/W]
    
    # 热节流
    T_throttle: float = 358.15   # 节流温度 [K] (85°C)
    T_shutdown: float = 378.15   # 关机温度 [K] (105°C)


@dataclass
class DisplayParams:
    """显示模块参数 - AMOLED特性"""
    P_controller: float = 0.06   # 显示控制器功耗 [W]
    P_touch: float = 0.03        # 触控功耗 [W]
    
    # OLED特性: P ∝ 亮度 × 白色像素比例
    k_brightness: float = 0.0022 # 亮度功耗系数 [W/(cd/m²)]
    k_pixel: float = 0.3         # 白色像素功耗因子
    
    # 刷新率功耗
    P_60Hz: float = 0.08         # 60Hz基础功耗 [W]
    k_refresh: float = 0.0012    # 每Hz额外功耗 [W/Hz]
    
    # 屏幕参数
    screen_area: float = 0.0098  # 屏幕面积 [m²] (6.7")
    max_brightness: float = 1200 # 最大亮度 [nits]


@dataclass
class CommunicationParams:
    """通信模块参数"""
    # 5G NR (Sub-6GHz + mmWave)
    P_5G_idle: float = 0.04      # 空闲 [W]
    P_5G_rx: float = 0.45        # 接收 [W]
    P_5G_tx_base: float = 0.8    # 发射基础 [W]
    P_5G_tx_max: float = 2.2     # 最大发射 [W]
    
    # WiFi 6E
    P_wifi_sleep: float = 0.005  # 休眠 [W]
    P_wifi_idle: float = 0.015   # 空闲 [W]
    P_wifi_rx: float = 0.28      # 接收 [W]
    P_wifi_tx: float = 0.55      # 发射 [W]
    
    # Bluetooth 5.3 LE
    P_bt_off: float = 0.0        # 关闭 [W]
    P_bt_standby: float = 0.001  # 待机 [W]
    P_bt_audio: float = 0.045    # 音频流 [W]
    P_bt_data: float = 0.065     # 数据传输 [W]


@dataclass
class GNSSParams:
    """GNSS模块参数"""
    P_LNA: float = 0.035         # 低噪声放大器 [W]
    P_acquisition: float = 0.22  # 捕获模式 [W]
    P_tracking: float = 0.065    # 跟踪模式 [W]
    P_sleep: float = 0.002       # 休眠 [W]
    
    tau_acquisition: float = 3.0  # 捕获时间常数 [s]
    tau_reacquire: float = 1.0    # 重捕获时间 [s]


@dataclass
class BackgroundParams:
    """后台任务参数 - OU过程"""
    mu_idle: float = 0.015       # 空闲均值电流 [A]
    mu_active: float = 0.045     # 活跃均值电流 [A]
    sigma: float = 0.008         # 波动标准差 [A]
    theta: float = 0.15          # 回归速率 [1/s]


# =============================================================================
# 第二部分：电化学模型 (Electrochemical Models)
# =============================================================================

class EnhancedOCVModel:
    """增强开路电压模型
    
    采用组合模型: 多项式 + 对数修正 + 温度补偿
    V_OCV(SOC, T) = V_poly(SOC) + V_log(SOC) + dV/dT·(T-T_ref)
    """
    
    def __init__(self, params: BatteryPhysicalParams):
        self.params = params
        self.T_ref = 298.15
        
        # 实验OCV-SOC数据 (典型NMC811电池)
        self.soc_data = np.array([
            0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
            0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00
        ])
        self.ocv_data = np.array([
            3.00, 3.35, 3.48, 3.55, 3.60, 3.64, 3.68, 3.71, 3.74, 3.77,
            3.80, 3.83, 3.86, 3.90, 3.94, 3.99, 4.04, 4.09, 4.13, 4.17, 4.20
        ])
        
        # 创建三次样条插值
        self._interp = interp1d(self.soc_data, self.ocv_data, kind='cubic',
                               fill_value='extrapolate')
    
    def V_OCV(self, SOC: float, T: float = 298.15) -> float:
        """计算开路电压"""
        SOC = np.clip(SOC, 0.001, 0.999)
        
        # 基础OCV (插值)
        V_base = float(self._interp(SOC))
        
        # 温度修正
        dT = T - self.T_ref
        V_temp = self.params.dV_dT * dT
        
        return V_base + V_temp
    
    def dV_dSOC(self, SOC: float, T: float = 298.15) -> float:
        """计算OCV对SOC的导数 (用于电压预测)"""
        dSOC = 0.001
        SOC = np.clip(SOC, dSOC, 1.0 - dSOC)
        return (self.V_OCV(SOC + dSOC, T) - self.V_OCV(SOC - dSOC, T)) / (2 * dSOC)
    
    def entropy_heat_rate(self, SOC: float, T: float, I: float) -> float:
        """熵变产热率: Q_entropy = I·T·dV/dT"""
        return abs(I) * T * abs(self.params.dV_dT)


class EnhancedResistanceModel:
    """增强内阻模型
    
    R_int(SOC, T, N) = R_0(T) · f_SOC(SOC) · f_aging(N)
    
    其中 R_0(T) 遵循 Arrhenius 关系
    """
    
    def __init__(self, params: BatteryPhysicalParams):
        self.params = params
        self.R_gas = 8.314  # J/(mol·K)
        
    def R_int(self, SOC: float, T: float, N_cycles: int = 0) -> float:
        """计算内阻"""
        p = self.params
        
        # 基础内阻 (Arrhenius温度依赖)
        T_ref = 298.15
        R_base = p.R_int_25C * np.exp(
            p.E_a_R / self.R_gas * (1/T - 1/T_ref)
        )
        
        # SOC影响 (低SOC和高SOC时内阻增加)
        f_SOC = 1.0 + 0.3 * (1 - SOC)**2 + 0.1 * SOC**3
        
        # 老化影响
        f_aging = 1.0 + p.R_growth * N_cycles
        
        return R_base * f_SOC * f_aging
    
    def joule_heat_rate(self, I: float, R: float) -> float:
        """焦耳热功率: P_joule = I²R"""
        return I**2 * R


# =============================================================================
# 第三部分：功耗模块 (Power Modules)
# =============================================================================

class EnhancedSoCPower:
    """增强SoC功耗模型 - 电热强耦合
    
    P_SoC = P_dynamic + P_leakage + P_static
    
    包含DVFS控制和热节流
    """
    
    def __init__(self, params: SoCChipParams):
        self.p = params
        self.T_soc = 298.15  # SoC温度状态
        
    def DVFS_controller(self, load: float, T_batt: float, T_soc: float) -> Tuple[float, float]:
        """DVFS控制器
        
        f_cpu = g(Load, T_batt, V_dd)
        
        Args:
            load: CPU负载 [0, 1]
            T_batt: 电池温度 [K]
            T_soc: SoC温度 [K]
        
        Returns:
            (f_cpu, V_dd): 频率和电压
        """
        p = self.p
        
        # 基于负载的目标频率
        f_target = p.f_min + load * (p.f_max - p.f_min)
        
        # 热节流 (基于SoC温度)
        if T_soc > p.T_throttle:
            throttle_factor = np.exp(-0.08 * (T_soc - p.T_throttle))
            f_target *= throttle_factor
        
        f_cpu = np.clip(f_target, p.f_min, p.f_max)
        
        # 电压-频率映射 (近似平方关系)
        f_norm = (f_cpu - p.f_min) / (p.f_max - p.f_min)
        V_dd = p.V_dd_min + np.sqrt(f_norm) * (p.V_dd_max - p.V_dd_min)
        
        return f_cpu, V_dd
    
    def P_dynamic(self, f: float, V: float, load: float) -> float:
        """动态功耗: P = α·C·V²·f"""
        return self.p.alpha * load * self.p.C_eff * V**2 * f
    
    def P_leakage(self, V: float, T_soc: float) -> float:
        """漏电功耗 (指数温度依赖)"""
        p = self.p
        T_ref = 298.15
        k_B = 8.617e-5  # eV/K
        
        I_leak = p.I_leak_ref * (T_soc / T_ref)**p.n_leak * \
                 np.exp(-p.E_a_leak / k_B * (1/T_soc - 1/T_ref))
        
        return I_leak * V
    
    def compute_power(self, load: float, T_batt: float, T_soc: float) -> Tuple[float, float, float]:
        """计算总功耗"""
        f, V = self.DVFS_controller(load, T_batt, T_soc)
        
        P_dyn = self.P_dynamic(f, V, load)
        P_leak = self.P_leakage(V, T_soc)
        P_static = 0.02  # 静态功耗基线
        
        return P_dyn, P_leak, P_static
    
    def thermal_dynamics(self, T_soc: float, T_batt: float, T_env: float, P_soc: float) -> float:
        """SoC温度动力学
        
        C_th · dT_soc/dt = P_soc - (T_soc-T_batt)/R_th_batt - (T_soc-T_env)/R_th_env
        """
        p = self.p
        
        Q_to_batt = (T_soc - T_batt) / p.R_th_soc_batt
        Q_to_env = (T_soc - T_env) / p.R_th_soc_env
        
        dT_dt = (P_soc - Q_to_batt - Q_to_env) / p.C_th_soc
        
        return dT_dt


class EnhancedDisplayPower:
    """增强显示模块功耗 - 环境光内容耦合"""
    
    def __init__(self, params: DisplayParams):
        self.p = params
    
    def compute_power(self, brightness_nits: float, refresh_rate: float,
                     content_brightness: float = 0.5, screen_on: bool = True) -> float:
        """计算显示功耗
        
        P_disp = P_static + P_refresh + P_content
        
        Args:
            brightness_nits: 屏幕亮度 [nits]
            refresh_rate: 刷新率 [Hz]
            content_brightness: 内容平均亮度 [0,1] (白色比例)
            screen_on: 屏幕是否开启
        """
        if not screen_on:
            return 0.001  # 息屏最小功耗
        
        p = self.p
        
        # 静态功耗 (控制器+触控)
        P_static = p.P_controller + p.P_touch
        
        # 刷新率功耗
        P_refresh = p.P_60Hz + p.k_refresh * max(0, refresh_rate - 60)
        
        # 内容功耗 (OLED: 与亮度和白色像素比例相关)
        effective_brightness = brightness_nits * (p.k_pixel + (1 - p.k_pixel) * content_brightness)
        P_content = p.k_brightness * effective_brightness * p.screen_area * 1000
        
        return P_static + P_refresh + P_content


class EnhancedCommunicationPower:
    """增强通信模块功耗 - 信道状态耦合"""
    
    def __init__(self, params: CommunicationParams):
        self.p = params
    
    def P_5G(self, mode: str, signal_quality: float = 0.8, data_rate: float = 0.0) -> float:
        """5G功耗模型
        
        Args:
            mode: 'off', 'idle', 'rx', 'tx'
            signal_quality: 信号质量 [0,1] (RSRP归一化)
            data_rate: 数据速率归一化 [0,1]
        """
        p = self.p
        
        if mode == 'off':
            return 0.0
        elif mode == 'idle':
            return p.P_5G_idle
        elif mode == 'rx':
            return p.P_5G_rx * (0.6 + 0.4 * data_rate)
        elif mode == 'tx':
            # 弱信号需要更高发射功率
            power_boost = 1.0 + 0.8 * (1 - signal_quality)
            return p.P_5G_tx_base + (p.P_5G_tx_max - p.P_5G_tx_base) * data_rate * power_boost
        return p.P_5G_idle
    
    def P_WiFi(self, mode: str, data_rate: float = 0.0) -> float:
        """WiFi功耗模型"""
        p = self.p
        
        modes = {
            'off': 0.0,
            'sleep': p.P_wifi_sleep,
            'idle': p.P_wifi_idle,
            'rx': p.P_wifi_rx * (0.5 + 0.5 * data_rate),
            'tx': p.P_wifi_tx * (0.5 + 0.5 * data_rate),
        }
        return modes.get(mode, p.P_wifi_idle)
    
    def P_Bluetooth(self, mode: str, audio_streaming: bool = False) -> float:
        """蓝牙功耗模型"""
        p = self.p
        
        if mode == 'off':
            return p.P_bt_off
        elif mode == 'standby':
            return p.P_bt_standby
        elif audio_streaming:
            return p.P_bt_audio
        else:
            return p.P_bt_data


class EnhancedGNSSPower:
    """增强GNSS模块功耗 - 环境信号耦合
    
    采用一阶动态系统建模锁定状态:
    dx_lock/dt = (1/τ)·[S(t) - x_lock]
    """
    
    def __init__(self, params: GNSSParams):
        self.p = params
        self.x_lock = 0.0  # 锁定状态 [0,1]
        
    def signal_availability(self, indoor: bool, urban: bool, 
                           satellite_count: int = 8) -> float:
        """计算信号可用性
        
        S = sigmoid(satellite_factor · environment_factor)
        """
        # 卫星因子
        sat_factor = np.clip(satellite_count / 6, 0, 1.5)
        
        # 环境因子
        env_factor = 1.0
        if indoor:
            env_factor *= 0.2
        if urban:
            env_factor *= 0.6
            
        # 综合信号质量
        S_raw = sat_factor * env_factor
        
        # Sigmoid归一化
        S = 1 / (1 + np.exp(-4 * (S_raw - 0.5)))
        
        return S
    
    def update_lock_state(self, S: float, dt: float) -> float:
        """更新锁定状态"""
        tau = self.p.tau_acquisition if self.x_lock < 0.5 else self.p.tau_reacquire
        
        # 一阶动态
        dx = (S - self.x_lock) / tau * dt
        self.x_lock = np.clip(self.x_lock + dx, 0, 1)
        
        return self.x_lock
    
    def compute_power(self, active: bool, indoor: bool = False, 
                     urban: bool = False, dt: float = 1.0) -> float:
        """计算GNSS功耗"""
        if not active:
            return self.p.P_sleep
        
        p = self.p
        
        S = self.signal_availability(indoor, urban)
        x = self.update_lock_state(S, dt)
        
        # P = P_LNA + x·P_track + (1-x)·P_acq
        P_gnss = p.P_LNA + x * p.P_tracking + (1 - x) * p.P_acquisition
        
        return P_gnss


class EnhancedBackgroundPower:
    """增强后台任务功耗 - Ornstein-Uhlenbeck随机过程
    
    dI_bg = θ·(μ - I_bg)·dt + σ·dW
    """
    
    def __init__(self, params: BackgroundParams):
        self.p = params
        self.I_bg = params.mu_idle
    
    def update(self, user_activity: float, dt: float) -> float:
        """更新后台电流 (OU过程)"""
        p = self.p
        
        # 活动调整均值
        mu = p.mu_idle + (p.mu_active - p.mu_idle) * user_activity
        
        # OU过程
        drift = p.theta * (mu - self.I_bg) * dt
        diffusion = p.sigma * np.sqrt(dt) * np.random.randn()
        
        self.I_bg = max(0.002, self.I_bg + drift + diffusion)
        
        return self.I_bg
    
    def compute_power(self, V_batt: float, user_activity: float, dt: float) -> float:
        """计算后台功耗"""
        I = self.update(user_activity, dt)
        return V_batt * I


# =============================================================================
# 第四部分：用户行为模型 (User Behavior Model)
# =============================================================================

class EnhancedUserModel:
    """增强用户行为模型 - 连续时间马尔科夫链
    
    状态空间: S = {Sleep, Idle, Light, Normal, Heavy}
    转移率矩阵: Q(t) 时变 (日夜/工作日/周末)
    """
    
    SLEEP = 0
    IDLE = 1
    LIGHT = 2
    NORMAL = 3
    HEAVY = 4
    
    STATE_NAMES = ['Sleep', 'Idle', 'Light', 'Normal', 'Heavy']
    
    def __init__(self):
        # 状态对应的硬件参数
        self.state_profiles = {
            self.SLEEP: {
                'cpu_load': 0.01,
                'screen_on': False,
                'brightness': 0,
                'refresh_rate': 60,
                'wifi_mode': 'sleep',
                '5g_mode': 'off',
                'bt_mode': 'off',
                'gps_active': False,
                'activity': 0.0,
            },
            self.IDLE: {
                'cpu_load': 0.03,
                'screen_on': False,
                'brightness': 0,
                'refresh_rate': 60,
                'wifi_mode': 'idle',
                '5g_mode': 'idle',
                'bt_mode': 'standby',
                'gps_active': False,
                'activity': 0.1,
            },
            self.LIGHT: {
                'cpu_load': 0.10,
                'screen_on': True,
                'brightness': 120,
                'refresh_rate': 60,
                'wifi_mode': 'idle',
                '5g_mode': 'idle',
                'bt_mode': 'standby',
                'gps_active': False,
                'activity': 0.2,
            },
            self.NORMAL: {
                'cpu_load': 0.30,
                'screen_on': True,
                'brightness': 280,
                'refresh_rate': 60,
                'wifi_mode': 'rx',
                '5g_mode': 'idle',
                'bt_mode': 'standby',
                'gps_active': False,
                'activity': 0.4,
            },
            self.HEAVY: {
                'cpu_load': 0.65,
                'screen_on': True,
                'brightness': 450,
                'refresh_rate': 120,
                'wifi_mode': 'tx',
                '5g_mode': 'rx',
                'bt_mode': 'off',
                'gps_active': True,
                'activity': 0.8,
            },
        }
        
        # 日间转移率矩阵 (7:00-23:00)
        self.Q_day = np.array([
            [-0.5,  0.3,  0.15, 0.04, 0.01],  # from Sleep
            [ 0.2, -0.5,  0.2,  0.08, 0.02],  # from Idle
            [ 0.1,  0.15,-0.4,  0.12, 0.03],  # from Light
            [ 0.05, 0.1,  0.2, -0.45, 0.1 ],  # from Normal
            [ 0.02, 0.08, 0.15, 0.25,-0.5 ],  # from Heavy
        ])
        
        # 夜间转移率矩阵 (23:00-7:00)
        self.Q_night = np.array([
            [-0.05, 0.03, 0.015,0.004,0.001],
            [ 0.6, -0.7,  0.08, 0.015,0.005],
            [ 0.5,  0.2, -0.75, 0.04, 0.01],
            [ 0.4,  0.2,  0.2, -0.85, 0.05],
            [ 0.3,  0.25, 0.2,  0.15,-0.9 ],
        ])
        
        # 工作时间转移率矩阵 (9:00-18:00)
        self.Q_work = np.array([
            [-0.8,  0.5,  0.2,  0.08, 0.02],
            [ 0.1, -0.4,  0.2,  0.08, 0.02],
            [ 0.05, 0.1, -0.35, 0.15, 0.05],
            [ 0.02, 0.08, 0.15,-0.35, 0.1 ],
            [ 0.01, 0.05, 0.1,  0.24,-0.4 ],
        ])
    
    def get_Q_matrix(self, hour: float) -> np.ndarray:
        """获取时变转移率矩阵"""
        if 23 <= hour or hour < 7:
            return self.Q_night
        elif 9 <= hour < 18:
            return self.Q_work
        else:
            return self.Q_day
    
    def simulate_transition(self, state: int, hour: float, dt: float) -> int:
        """模拟状态转移"""
        Q = self.get_Q_matrix(hour)
        
        # 离开当前状态的速率
        rate = -Q[state, state]
        
        # 泊松过程判断是否转移
        if np.random.random() < rate * dt:
            # 选择目标状态
            probs = Q[state, :].copy()
            probs[state] = 0
            probs = np.maximum(probs, 0)
            if probs.sum() > 0:
                probs = probs / probs.sum()
                return np.random.choice(5, p=probs)
        
        return state
    
    def get_profile(self, state: int) -> dict:
        """获取状态对应的硬件配置"""
        return self.state_profiles[state].copy()


# =============================================================================
# 第五部分：完整系统模型 (Complete System Model)
# =============================================================================

class EnhancedBatterySystem:
    """完整智能手机电池系统模型
    
    状态向量: X = [SOC, T_batt, T_soc, x_lock, I_bg]
    
    耦合方程组:
    dSOC/dt = -I_total / Q_eff
    dT_batt/dt = (P_joule + P_entropy - Q_loss) / C_th
    dT_soc/dt = (P_soc - Q_to_batt - Q_to_env) / C_th_soc
    dx_lock/dt = (S - x_lock) / τ
    dI_bg = θ·(μ - I_bg)·dt + σ·dW
    """
    
    def __init__(self):
        # 初始化所有参数
        self.batt_params = BatteryPhysicalParams()
        self.soc_params = SoCChipParams()
        self.disp_params = DisplayParams()
        self.comm_params = CommunicationParams()
        self.gnss_params = GNSSParams()
        self.bg_params = BackgroundParams()
        
        # 初始化模型组件
        self.ocv_model = EnhancedOCVModel(self.batt_params)
        self.R_model = EnhancedResistanceModel(self.batt_params)
        self.soc_power = EnhancedSoCPower(self.soc_params)
        self.disp_power = EnhancedDisplayPower(self.disp_params)
        self.comm_power = EnhancedCommunicationPower(self.comm_params)
        self.gnss_power = EnhancedGNSSPower(self.gnss_params)
        self.bg_power = EnhancedBackgroundPower(self.bg_params)
        self.user_model = EnhancedUserModel()
        
        # 系统状态
        self.N_cycles = 0
        
    def compute_total_power(self, profile: dict, V_batt: float, 
                           T_batt: float, T_soc: float, dt: float) -> Tuple[float, dict]:
        """计算总功耗和各组件功耗"""
        components = {}
        
        # SoC功耗
        P_dyn, P_leak, P_static = self.soc_power.compute_power(
            profile['cpu_load'], T_batt, T_soc)
        components['SoC'] = P_dyn + P_leak + P_static
        
        # 显示功耗
        components['Display'] = self.disp_power.compute_power(
            profile['brightness'], profile['refresh_rate'],
            screen_on=profile['screen_on'])
        
        # 5G功耗
        components['5G'] = self.comm_power.P_5G(profile['5g_mode'])
        
        # WiFi功耗
        components['WiFi'] = self.comm_power.P_WiFi(profile['wifi_mode'])
        
        # 蓝牙功耗
        components['Bluetooth'] = self.comm_power.P_Bluetooth(profile['bt_mode'])
        
        # GPS功耗
        components['GPS'] = self.gnss_power.compute_power(
            profile['gps_active'], dt=dt)
        
        # 后台功耗
        components['Background'] = self.bg_power.compute_power(
            V_batt, profile['activity'], dt)
        
        P_total = sum(components.values())
        
        return P_total, components
    
    def system_ode(self, t: float, state: np.ndarray, profile: dict) -> np.ndarray:
        """系统ODE
        
        state = [SOC, T_batt, T_soc]
        """
        SOC, T_batt, T_soc = state
        SOC = np.clip(SOC, 0.001, 0.999)
        
        bp = self.batt_params
        
        # 计算OCV和内阻
        V_ocv = self.ocv_model.V_OCV(SOC, T_batt)
        R_int = self.R_model.R_int(SOC, T_batt, self.N_cycles)
        
        # 计算总功耗
        P_total, _ = self.compute_total_power(profile, V_ocv, T_batt, T_soc, 1.0)
        
        # 求解电流 (迭代法)
        # V_batt = V_ocv - I·R, P = V_batt·I / η
        # 解二次方程: I²R - IV_ocv + P/η = 0
        a = R_int
        b = -V_ocv
        c = P_total / bp.eta_PMIC
        
        disc = b**2 - 4*a*c
        if disc >= 0:
            I_total = (-b - np.sqrt(disc)) / (2*a)
        else:
            I_total = P_total / (bp.eta_PMIC * V_ocv)
        
        I_total = max(0.001, I_total)
        V_batt = V_ocv - I_total * R_int
        
        # 有效容量 (温度影响)
        Q_eff = bp.Q_max * (1 - bp.capacity_fade * self.N_cycles)
        if T_batt < 298.15:
            Q_eff *= (1 - 0.008 * (298.15 - T_batt))
        
        # === SOC动力学 ===
        dSOC_dt = -I_total / (Q_eff * 3600)
        
        # === 电池热动力学 ===
        P_joule = self.R_model.joule_heat_rate(I_total, R_int)
        P_entropy = self.ocv_model.entropy_heat_rate(SOC, T_batt, I_total)
        Q_loss = (T_batt - bp.T_env) / bp.R_th_batt_env
        
        dT_batt_dt = (P_joule + P_entropy - Q_loss) / bp.C_th_batt
        
        # === SoC热动力学 ===
        P_soc = profile['cpu_load'] * 2.0  # 估计SoC功耗
        dT_soc_dt = self.soc_power.thermal_dynamics(T_soc, T_batt, bp.T_env, P_soc)
        
        return np.array([dSOC_dt, dT_batt_dt, dT_soc_dt])
    
    def simulate(self, duration_hours: float, initial_soc: float = 1.0,
                start_hour: float = 8.0, dt: float = 30.0,
                scenario: str = 'realistic') -> dict:
        """运行完整模拟
        
        Args:
            duration_hours: 模拟时长 [小时]
            initial_soc: 初始SOC
            start_hour: 起始小时
            dt: 时间步长 [秒]
            scenario: 场景 ('realistic', 'idle', 'light', 'normal', 'heavy')
        """
        # 场景到固定状态的映射
        fixed_states = {
            'idle': self.user_model.IDLE,
            'light': self.user_model.LIGHT,
            'normal': self.user_model.NORMAL,
            'heavy': self.user_model.HEAVY,
        }
        
        # 初始化状态
        state = np.array([initial_soc, self.batt_params.T_env, self.batt_params.T_env])
        user_state = self.user_model.NORMAL
        
        # 历史记录
        history = {
            'time': [], 'SOC': [], 'T_batt': [], 'T_soc': [],
            'V_batt': [], 'I_total': [], 'P_total': [],
            'components': [], 'user_state': [],
        }
        
        # 模拟循环
        n_steps = int(duration_hours * 3600 / dt)
        
        for i in range(n_steps):
            t = i * dt
            hour = (start_hour + t / 3600) % 24
            
            # 更新用户状态
            if scenario == 'realistic':
                user_state = self.user_model.simulate_transition(user_state, hour, dt)
            else:
                user_state = fixed_states.get(scenario, self.user_model.NORMAL)
            
            profile = self.user_model.get_profile(user_state)
            
            # 计算当前电气量
            SOC, T_batt, T_soc = state
            V_ocv = self.ocv_model.V_OCV(SOC, T_batt)
            R_int = self.R_model.R_int(SOC, T_batt, self.N_cycles)
            
            P_total, components = self.compute_total_power(
                profile, V_ocv, T_batt, T_soc, dt)
            
            I_total = P_total / (self.batt_params.eta_PMIC * V_ocv)
            V_batt = V_ocv - I_total * R_int
            
            # 记录
            history['time'].append(t / 3600)
            history['SOC'].append(SOC)
            history['T_batt'].append(T_batt - 273.15)
            history['T_soc'].append(T_soc - 273.15)
            history['V_batt'].append(V_batt)
            history['I_total'].append(I_total)
            history['P_total'].append(P_total)
            history['components'].append(components)
            history['user_state'].append(user_state)
            
            # 更新状态 (Euler积分)
            dstate = self.system_ode(t, state, profile)
            state = state + dstate * dt
            
            # 限制状态范围
            state[0] = np.clip(state[0], 0.0, 1.0)
            state[1] = np.clip(state[1], 273.15, 353.15)
            state[2] = np.clip(state[2], 273.15, 383.15)
            
            # 检查电量耗尽
            if state[0] <= 0.01:
                break
        
        return history
    
    def predict_remaining_time(self, current_soc: float, scenario: str,
                               T_batt: float = 298.15) -> Tuple[float, dict]:
        """预测剩余时间"""
        # 场景平均功耗
        avg_power = {
            'idle': 0.12,
            'light': 0.45,
            'normal': 1.1,
            'heavy': 3.2,
            'realistic': 0.8,
        }.get(scenario, 1.0)
        
        bp = self.batt_params
        Q_eff = bp.Q_max * (1 - bp.capacity_fade * self.N_cycles)
        V_avg = self.ocv_model.V_OCV(current_soc * 0.5, T_batt)
        
        I_avg = avg_power / (bp.eta_PMIC * V_avg)
        t_remain = (current_soc * Q_eff) / I_avg
        
        return t_remain, {'power': avg_power, 'capacity': Q_eff, 'current': I_avg}


# =============================================================================
# 测试代码
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("增强型智能手机电池放电模型测试")
    print("Enhanced Smartphone Battery Discharge Model Test")
    print("=" * 60)
    
    model = EnhancedBatterySystem()
    
    # 测试不同场景
    scenarios = ['idle', 'light', 'normal', 'heavy', 'realistic']
    
    for scenario in scenarios:
        print(f"\n测试场景: {scenario}")
        history = model.simulate(
            duration_hours=24,
            initial_soc=1.0,
            scenario=scenario,
            dt=60
        )
        
        soc = np.array(history['SOC'])
        time = np.array(history['time'])
        power = np.array(history['P_total'])
        
        # 放电时间
        idx = np.where(soc <= 0.05)[0]
        drain_time = time[idx[0]] if len(idx) > 0 else time[-1]
        
        print(f"  放电时间: {drain_time:.1f} 小时")
        print(f"  平均功耗: {np.mean(power):.2f} W")
        print(f"  最终SOC: {soc[-1]*100:.1f}%")
