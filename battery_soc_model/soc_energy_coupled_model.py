"""
SOC与耗能关系的连续时间耦合模型
结合马尔科夫时间分区和微分耦合方程组

模型特点：
1. 连续时间马尔科夫链建模用户行为
2. 电池电化学-热耦合微分方程组
3. 各模块功耗耦合计算
4. SOC与能耗关系建模
5. 电池剩余使用时间预测

Author: MCM Expert
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, List, Dict, Callable
import warnings
warnings.filterwarnings('ignore')

# ============================================================================
# 第一部分：电池参数与常量定义
# ============================================================================

@dataclass
class BatteryParams:
    """电池物理参数"""
    Q_max: float = 4000.0       # 电池最大容量 (mAh)
    V_nominal: float = 3.7      # 标称电压 (V)
    R_int_25: float = 0.08      # 25°C时内阻 (Ω)
    C_th: float = 50.0          # 热容 (J/K)
    R_th: float = 10.0          # 热阻 (K/W)
    T_env: float = 25.0         # 环境温度 (°C)
    
    # OCV-SOC关系参数 (多项式拟合系数)
    ocv_coeffs: tuple = (3.0, 0.8, 0.2, -0.1, 0.1)
    
    # 内阻温度系数
    R_temp_coeff: float = 0.005  # R_int = R_int_25 * (1 + coeff * (T - 25))
    
    # 降额保护
    N_derate: float = 1.0       # 降额因子

@dataclass 
class HardwareParams:
    """硬件模块参数"""
    # SoC模块
    V_dd: float = 1.0           # 供电电压 (V)
    C_eff: float = 1e-9         # 等效电容 (F)
    I_leak_25: float = 0.01     # 25°C漏电流 (A)
    
    # 显示模块
    P_static_disp: float = 0.5  # 显示静态功耗 (W)
    k_drv: float = 0.001        # 驱动功耗系数
    
    # 通信模块
    P_5G_base: float = 2.0      # 5G基础功耗 (W)
    P_BT_active: float = 0.1    # 蓝牙活动功耗 (W)
    P_BT_sleep: float = 0.01    # 蓝牙睡眠功耗 (W)
    P_GNSS_acq: float = 0.15    # GNSS捕获功耗 (W)
    P_GNSS_track: float = 0.05  # GNSS跟踪功耗 (W)
    
    # 后台任务
    P_background_base: float = 0.1  # 基础后台功耗 (W)

# ============================================================================
# 第二部分：连续时间马尔科夫用户行为模型
# ============================================================================

class ContinuousTimeMarkovUserModel:
    """
    连续时间马尔科夫链用户行为模型
    
    状态定义:
    S1: Deep Sleep (深度睡眠)
    S2: Light Use (轻度使用 - 社交/消息)
    S3: Streaming (流媒体)
    S4: Gaming (游戏)
    """
    
    def __init__(self):
        self.states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        self.num_states = 4
        
        # 状态对应的硬件参数范围 [Min, Max]
        # APL (Average Picture Level) %
        self.P_APL = np.array([
            [0, 0],      # Deep Sleep
            [60, 95],    # Light Use
            [20, 50],    # Streaming
            [40, 75]     # Gaming
        ])
        
        # CPU利用率 %
        self.P_CPU_Util = np.array([
            [0, 2],      # Deep Sleep
            [5, 25],     # Light Use
            [15, 35],    # Streaming
            [70, 95]     # Gaming
        ])
        
        # CPU频率 GHz
        self.P_CPU_Freq = np.array([
            [0.3, 0.3],  # Deep Sleep
            [0.8, 1.8],  # Light Use
            [1.0, 2.0],  # Streaming
            [2.2, 3.0]   # Gaming
        ])
        
        # 亮度范围 nits
        self.P_Brightness = np.array([
            [0, 0],       # Deep Sleep
            [150, 500],   # Light Use
            [200, 800],   # Streaming
            [300, 1000]   # Gaming
        ])
        
        # 定义三种时段的转移速率矩阵 (Q矩阵)
        # Q矩阵: 对角线元素为负，使得行和为0
        self._define_transition_matrices()
        
    def _define_transition_matrices(self):
        """定义不同时段的转移速率矩阵"""
        
        # 睡眠模式转移概率矩阵 (离散)
        P_sleep = np.array([
            [0.995, 0.005, 0.000, 0.000],
            [0.600, 0.400, 0.000, 0.000],
            [0.100, 0.000, 0.900, 0.000],
            [0.100, 0.000, 0.000, 0.900]
        ])
        
        # 工作模式转移概率矩阵
        P_work = np.array([
            [0.850, 0.145, 0.003, 0.002],
            [0.250, 0.700, 0.040, 0.010],
            [0.100, 0.100, 0.800, 0.000],
            [0.200, 0.100, 0.000, 0.700]
        ])
        
        # 休闲模式转移概率矩阵
        P_leisure = np.array([
            [0.800, 0.150, 0.030, 0.020],
            [0.050, 0.650, 0.200, 0.100],
            [0.010, 0.040, 0.940, 0.010],
            [0.010, 0.010, 0.010, 0.970]
        ])
        
        # 转换为连续时间Q矩阵 (1分钟时间步长)
        # Q = (P - I) / dt, dt = 1 minute = 1/60 hour
        dt = 1.0 / 60.0  # 小时
        
        self.Q_sleep = (P_sleep - np.eye(4)) / dt
        self.Q_work = (P_work - np.eye(4)) / dt
        self.Q_leisure = (P_leisure - np.eye(4)) / dt
        
    def get_Q_matrix(self, hour: float) -> np.ndarray:
        """
        根据当前时间返回对应的转移速率矩阵
        
        Args:
            hour: 当天的小时数 (0-24)
            
        Returns:
            Q: 转移速率矩阵
        """
        hour = hour % 24
        
        if hour >= 23 or hour < 7:
            return self.Q_sleep
        elif (hour >= 9 and hour < 12) or (hour >= 14 and hour < 18):
            return self.Q_work
        else:
            return self.Q_leisure
    
    def get_time_mode(self, hour: float) -> str:
        """获取时段模式名称"""
        hour = hour % 24
        if hour >= 23 or hour < 7:
            return "sleep"
        elif (hour >= 9 and hour < 12) or (hour >= 14 and hour < 18):
            return "work"
        else:
            return "leisure"
    
    def kolmogorov_forward_ode(self, p: np.ndarray, t: float, 
                               start_hour: float = 0) -> np.ndarray:
        """
        Kolmogorov前向方程 (主方程)
        dp/dt = p * Q(t)
        
        Args:
            p: 状态概率分布向量
            t: 时间 (小时)
            start_hour: 开始时间
            
        Returns:
            dp/dt: 概率变化率
        """
        current_hour = (start_hour + t) % 24
        Q = self.get_Q_matrix(current_hour)
        return p @ Q
    
    def simulate_state_distribution(self, initial_dist: np.ndarray,
                                    duration_hours: float,
                                    start_hour: float = 0,
                                    dt_minutes: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        模拟状态概率分布的时间演化
        
        Args:
            initial_dist: 初始状态分布
            duration_hours: 仿真时长 (小时)
            start_hour: 开始时间
            dt_minutes: 时间步长 (分钟)
            
        Returns:
            time_axis: 时间轴
            prob_history: 概率分布历史
        """
        n_steps = int(duration_hours * 60 / dt_minutes)
        time_axis = np.linspace(0, duration_hours, n_steps)
        
        prob_history = np.zeros((n_steps, self.num_states))
        prob_history[0] = initial_dist
        
        dt_hours = dt_minutes / 60.0
        
        for i in range(1, n_steps):
            current_hour = (start_hour + time_axis[i-1]) % 24
            Q = self.get_Q_matrix(current_hour)
            
            # 使用矩阵指数的近似 P(dt) ≈ I + Q*dt
            P_dt = np.eye(self.num_states) + Q * dt_hours
            P_dt = np.clip(P_dt, 0, 1)  # 确保概率非负
            P_dt = P_dt / P_dt.sum(axis=1, keepdims=True)  # 归一化
            
            prob_history[i] = prob_history[i-1] @ P_dt
            
        return time_axis, prob_history
    
    def get_expected_power_params(self, state_dist: np.ndarray, 
                                  hour: float) -> Dict[str, float]:
        """
        根据状态分布计算期望硬件参数
        
        Args:
            state_dist: 当前状态分布
            hour: 当前时间
            
        Returns:
            params: 期望硬件参数字典
        """
        # 计算各参数的期望值
        expected_apl = np.sum(state_dist * np.mean(self.P_APL, axis=1))
        expected_cpu_util = np.sum(state_dist * np.mean(self.P_CPU_Util, axis=1))
        expected_cpu_freq = np.sum(state_dist * np.mean(self.P_CPU_Freq, axis=1))
        
        # 亮度受环境光影响
        sunlight_factor = np.exp(-((hour % 24 - 13)**2) / (2 * 3**2))
        base_brightness = np.sum(state_dist * np.mean(self.P_Brightness, axis=1))
        adjusted_brightness = base_brightness * (0.5 + 0.5 * sunlight_factor)
        
        return {
            'apl': expected_apl,
            'cpu_util': expected_cpu_util,
            'cpu_freq': expected_cpu_freq,
            'brightness': adjusted_brightness,
            'sunlight_factor': sunlight_factor
        }

# ============================================================================
# 第三部分：电池电化学-热耦合微分方程组
# ============================================================================

class BatteryElectrochemicalThermalModel:
    """
    电池电化学-热耦合模型
    
    核心微分方程:
    1. dSOC/dt = -I_total(t) / (Q_max * N)
    2. dT_batt/dt = (P_joule + P_entropy - (T_batt - T_env)/R_th) / C_th
    
    其中:
    - V_batt = V_OCV(SOC) - I_total * R_int(SOC, T)
    - P_joule = I_total^2 * R_int
    - P_entropy = I_total * T * dV_OCV/dT
    """
    
    def __init__(self, battery_params: BatteryParams = None):
        self.params = battery_params or BatteryParams()
        
    def V_OCV(self, soc: float) -> float:
        """
        开路电压与SOC的关系 (基于多项式拟合)
        
        V_OCV = a0 + a1*SOC + a2*SOC^2 + a3*SOC^3 + a4*SOC^4
        """
        coeffs = self.params.ocv_coeffs
        soc_clipped = np.clip(soc, 0.01, 0.99)
        
        V = (coeffs[0] + 
             coeffs[1] * soc_clipped + 
             coeffs[2] * soc_clipped**2 + 
             coeffs[3] * soc_clipped**3 +
             coeffs[4] * soc_clipped**4)
        
        return np.clip(V, 2.8, 4.2)
    
    def dV_OCV_dT(self, soc: float) -> float:
        """
        OCV的温度系数 (熵变效应)
        典型值: -0.2 ~ -0.5 mV/K
        """
        return -0.0003  # V/K
    
    def R_int(self, soc: float, T: float) -> float:
        """
        内阻模型: R = R_25 * f(SOC) * g(T)
        
        Args:
            soc: 电池SOC
            T: 电池温度 (°C)
            
        Returns:
            R: 内阻 (Ω)
        """
        # SOC影响 (低SOC时内阻增大)
        soc_factor = 1 + 0.5 * (1 - soc)**2
        
        # 温度影响 (Arrhenius关系简化)
        T_ref = 25.0
        temp_factor = 1 + self.params.R_temp_coeff * (T_ref - T)
        temp_factor = np.clip(temp_factor, 0.5, 3.0)
        
        return self.params.R_int_25 * soc_factor * temp_factor
    
    def calculate_battery_voltage(self, soc: float, T: float, 
                                  I_total: float) -> float:
        """
        计算电池端电压
        V_batt = V_OCV(SOC) - I_total * R_int(SOC, T) * N
        """
        V_ocv = self.V_OCV(soc)
        R = self.R_int(soc, T)
        N = self.params.N_derate
        
        V_batt = V_ocv - I_total * R * N
        return np.clip(V_batt, 2.5, 4.2)

# ============================================================================
# 第四部分：各模块功耗耦合计算
# ============================================================================

class ModulePowerCalculator:
    """
    各模块功耗计算器
    
    包括:
    1. SoC模块 (处理器)
    2. 显示模块
    3. 5G通信模块
    4. 蓝牙模块
    5. GNSS模块
    6. 后台任务模块
    """
    
    def __init__(self, hw_params: HardwareParams = None):
        self.params = hw_params or HardwareParams()
        
    def P_SoC(self, cpu_freq: float, cpu_util: float, T: float) -> float:
        """
        SoC模块功耗 (动态 + 漏电)
        
        P_dyn = C_eff * V_dd^2 * f * util
        P_leak = I_leak * V_dd * exp((T-25)/10)
        """
        # 动态功耗
        f_Hz = cpu_freq * 1e9  # GHz -> Hz
        P_dyn = (self.params.C_eff * self.params.V_dd**2 * 
                 f_Hz * (cpu_util / 100.0))
        
        # 漏电功耗 (指数温度依赖)
        leak_factor = np.exp((T - 25) / 10)
        P_leak = self.params.I_leak_25 * self.params.V_dd * leak_factor
        
        # 总功耗 (W)
        return P_dyn + P_leak
    
    def P_Display(self, brightness: float, apl: float, refresh_rate: float = 60) -> float:
        """
        显示模块功耗
        
        P_disp = P_static + k_drv * f_refresh * L * APL
        """
        P_static = self.params.P_static_disp
        
        # 动态功耗与亮度、APL、刷新率相关
        P_dynamic = (self.params.k_drv * refresh_rate * 
                     (brightness / 1000) * (apl / 100))
        
        return P_static + P_dynamic
    
    def P_5G(self, signal_strength: float, data_rate: float = 0) -> float:
        """
        5G通信模块功耗
        
        考虑信号强度和数据速率
        """
        if data_rate <= 0:
            return 0.05  # 待机功耗
        
        # 信号强度影响 (弱信号需要更大发射功率)
        signal_factor = 1 + 0.5 * (1 - signal_strength)
        
        # 数据速率影响
        rate_factor = 0.5 + 0.5 * (data_rate / 100)  # 假设最大100 Mbps
        
        return self.params.P_5G_base * signal_factor * rate_factor
    
    def P_Bluetooth(self, audio_active: bool, duty_cycle: float = 0.1) -> float:
        """
        蓝牙模块功耗 (事件驱动)
        """
        if audio_active:
            return self.params.P_BT_active
        else:
            return (duty_cycle * self.params.P_BT_active + 
                    (1 - duty_cycle) * self.params.P_BT_sleep)
    
    def P_GNSS(self, is_locked: bool, signal_quality: float = 0.5) -> float:
        """
        GNSS模块功耗
        
        捕获状态功耗 > 跟踪状态功耗
        """
        if not is_locked:
            # 捕获状态 (高功耗)
            return self.params.P_GNSS_acq
        else:
            # 跟踪状态 (功耗与信号质量相关)
            return self.params.P_GNSS_track * (0.5 + 0.5 / (signal_quality + 0.1))
    
    def P_Background(self, user_activity_level: float) -> float:
        """
        后台任务功耗 (随机过程)
        
        与用户活动水平相关
        """
        # 基础功耗 + 活动相关功耗
        return self.params.P_background_base * (1 + 0.5 * user_activity_level)

# ============================================================================
# 第五部分：SOC-能耗耦合系统模型
# ============================================================================

class SOCEnergyCoupledSystem:
    """
    SOC与能耗关系的完整耦合系统
    
    整合:
    1. 连续时间马尔科夫用户行为模型
    2. 电池电化学-热耦合模型
    3. 各模块功耗计算
    4. 电池剩余时间预测
    """
    
    def __init__(self, 
                 battery_params: BatteryParams = None,
                 hardware_params: HardwareParams = None):
        
        self.battery_params = battery_params or BatteryParams()
        self.hardware_params = hardware_params or HardwareParams()
        
        self.user_model = ContinuousTimeMarkovUserModel()
        self.battery_model = BatteryElectrochemicalThermalModel(self.battery_params)
        self.power_calculator = ModulePowerCalculator(self.hardware_params)
        
    def calculate_total_power(self, hw_params: Dict[str, float], 
                             T_batt: float,
                             enable_5G: bool = None,
                             enable_GNSS: bool = None,
                             enable_BT: bool = True) -> float:
        """
        计算总功耗
        
        Args:
            hw_params: 硬件参数 (来自用户状态)
            T_batt: 电池温度
            enable_*: 各模块使能状态
            
        Returns:
            P_total: 总功耗 (W)
        """
        cpu_util = hw_params['cpu_util']
        cpu_freq = hw_params['cpu_freq']
        brightness = hw_params['brightness']
        apl = hw_params['apl']
        
        # 根据CPU利用率推断是否需要5G和GNSS
        if enable_5G is None:
            enable_5G = cpu_util > 15  # 有活动时启用网络
        if enable_GNSS is None:
            enable_GNSS = cpu_util > 30  # 流媒体/游戏时可能启用GPS
        
        # SoC功耗 - 根据CPU频率和利用率动态计算
        # 基础功耗 + 动态功耗
        P_soc_base = 0.1  # 100mW基础功耗
        P_soc_dynamic = 0.5 * (cpu_freq / 3.0)**2 * (cpu_util / 100.0)  # 动态功耗
        P_soc_leak = 0.02 * np.exp((T_batt - 25) / 15)  # 漏电功耗
        P_soc = P_soc_base + P_soc_dynamic + P_soc_leak
        
        # 显示功耗 - 主要功耗来源
        if brightness < 10:  # 屏幕关闭
            P_disp = 0.01
        else:
            P_disp_static = 0.3  # 静态功耗
            P_disp_backlight = 0.8 * (brightness / 1000)  # 背光功耗
            P_disp_content = 0.2 * (apl / 100)  # 内容功耗
            P_disp = P_disp_static + P_disp_backlight + P_disp_content
        
        # 通信模块功耗
        if enable_5G:
            # 5G功耗与数据活动相关
            data_activity = min(cpu_util / 50.0, 1.0)  # 归一化
            P_5G = 0.2 + 1.5 * data_activity  # 200mW待机到1.7W满载
        else:
            P_5G = 0.05  # 50mW待机
        
        # 蓝牙功耗
        P_BT = 0.05 if cpu_util < 50 else 0.1  # 50-100mW
        
        # GNSS功耗
        P_GNSS = 0.15 if enable_GNSS else 0
        
        # 后台功耗
        P_bg = 0.1 * (1 + 0.3 * (cpu_util / 100.0))
        
        # 总功耗
        P_total = P_soc + P_disp + P_5G + P_BT + P_GNSS + P_bg
        
        return P_total
    
    def coupled_ode_system(self, y: np.ndarray, t: float,
                           state_dist_func: Callable,
                           start_hour: float = 0) -> np.ndarray:
        """
        耦合微分方程组
        
        状态变量 y = [SOC, T_batt]
        
        方程:
        dSOC/dt = -I_total / (Q_max * N)
        dT_batt/dt = (P_joule + P_entropy - (T_batt - T_env)/R_th) / C_th
        
        Args:
            y: 状态变量 [SOC, T_batt]
            t: 时间 (小时)
            state_dist_func: 状态分布函数
            start_hour: 开始时间
            
        Returns:
            dy/dt: 状态变量导数
        """
        SOC, T_batt = y
        
        # 当SOC降到最低阈值时停止放电
        if SOC <= 0.01:
            return np.array([0.0, 0.0])
        
        SOC = np.clip(SOC, 0.01, 0.99)
        
        # 获取当前时间
        current_hour = (start_hour + t) % 24
        
        # 获取用户状态分布
        state_dist = state_dist_func(t)
        
        # 计算期望硬件参数
        hw_params = self.user_model.get_expected_power_params(state_dist, current_hour)
        
        # 计算总功耗
        P_total = self.calculate_total_power(hw_params, T_batt)
        
        # 计算电池电压
        V_batt = self.battery_model.calculate_battery_voltage(SOC, T_batt, 0)
        
        # 计算总电流 (A) = P_total / V_batt
        I_total = P_total / V_batt
        
        # SOC方程: dSOC/dt = -I_total / (Q_max * N)
        # Q_max单位为mAh，需要转换
        Q_max_Ah = self.battery_params.Q_max / 1000.0
        N = self.battery_params.N_derate
        dSOC_dt = -I_total / (Q_max_Ah * N)
        
        # 温度方程
        R_int = self.battery_model.R_int(SOC, T_batt)
        
        # 焦耳热
        P_joule = I_total**2 * R_int
        
        # 熵变热 (通常为负，吸热)
        dV_dT = self.battery_model.dV_OCV_dT(SOC)
        P_entropy = I_total * (T_batt + 273.15) * dV_dT
        
        # 热平衡方程
        T_env = self.battery_params.T_env
        R_th = self.battery_params.R_th
        C_th = self.battery_params.C_th
        
        dT_dt = (P_joule + P_entropy - (T_batt - T_env) / R_th) / C_th
        
        return np.array([dSOC_dt, dT_dt])
    
    def simulate_battery_drain(self, 
                               initial_SOC: float = 1.0,
                               initial_T: float = 25.0,
                               duration_hours: float = 24,
                               start_hour: float = 8,
                               initial_state_dist: np.ndarray = None,
                               min_SOC: float = 0.01) -> Dict:
        """
        仿真电池放电过程
        
        Args:
            initial_SOC: 初始SOC
            initial_T: 初始温度 (°C)
            duration_hours: 仿真时长 (小时)
            start_hour: 开始时间 (小时)
            initial_state_dist: 初始用户状态分布
            min_SOC: 最低SOC阈值
            
        Returns:
            results: 仿真结果字典
        """
        if initial_state_dist is None:
            initial_state_dist = np.array([0.3, 0.4, 0.2, 0.1])
            
        # 首先模拟用户状态分布演化
        time_axis, prob_history = self.user_model.simulate_state_distribution(
            initial_state_dist, duration_hours, start_hour
        )
        
        # 创建状态分布插值函数
        state_dist_interp = [
            interp1d(time_axis, prob_history[:, i], 
                     kind='linear', fill_value='extrapolate')
            for i in range(self.user_model.num_states)
        ]
        
        def state_dist_func(t):
            dist = np.array([f(t) for f in state_dist_interp])
            dist = np.clip(dist, 0, 1)
            return dist / dist.sum()
        
        # 定义SOC耗尽事件函数
        def soc_depleted(t, y):
            return y[0] - min_SOC
        soc_depleted.terminal = True
        soc_depleted.direction = -1
        
        # 求解耦合ODE
        y0 = [initial_SOC, initial_T]
        t_span = (0, duration_hours)
        t_eval = time_axis
        
        # 使用较稳定的求解器，添加终止事件
        sol = solve_ivp(
            lambda t, y: self.coupled_ode_system(y, t, state_dist_func, start_hour),
            t_span, y0, t_eval=t_eval, method='RK45',
            max_step=0.1, rtol=1e-6, atol=1e-8,
            events=soc_depleted
        )
        
        # 确保SOC不为负
        sol_SOC = np.clip(sol.y[0], min_SOC, 1.0)
        
        # 调整状态分布历史以匹配实际仿真时间
        actual_time = sol.t
        actual_prob_history = np.zeros((len(actual_time), self.user_model.num_states))
        for i in range(self.user_model.num_states):
            interp_func = interp1d(time_axis, prob_history[:, i], 
                                   kind='linear', fill_value='extrapolate')
            actual_prob_history[:, i] = interp_func(actual_time)
        
        # 计算功耗历史
        power_history = []
        current_history = []
        voltage_history = []
        
        for i, t in enumerate(sol.t):
            SOC, T_batt = sol_SOC[i], sol.y[1, i]
            current_hour = (start_hour + t) % 24
            state_dist = state_dist_func(t)
            hw_params = self.user_model.get_expected_power_params(state_dist, current_hour)
            
            P = self.calculate_total_power(hw_params, T_batt)
            V = self.battery_model.calculate_battery_voltage(SOC, T_batt, 0)
            I = P / V
            
            power_history.append(P)
            current_history.append(I)
            voltage_history.append(V)
        
        return {
            'time': sol.t,
            'SOC': sol_SOC,
            'temperature': sol.y[1],
            'power': np.array(power_history),
            'current': np.array(current_history),
            'voltage': np.array(voltage_history),
            'state_distribution': actual_prob_history,
            'start_hour': start_hour,
            'battery_depleted': len(sol.t_events[0]) > 0 if sol.t_events else False
        }

# ============================================================================
# 第六部分：电池剩余使用时间预测
# ============================================================================

class BatteryLifePredictor:
    """
    电池剩余使用时间预测器
    
    基于:
    1. 当前SOC状态
    2. 用户使用模式
    3. 历史能耗数据
    4. 环境因素
    """
    
    def __init__(self, coupled_system: SOCEnergyCoupledSystem = None):
        self.system = coupled_system or SOCEnergyCoupledSystem()
        
    def predict_remaining_time(self,
                               current_SOC: float,
                               current_T: float,
                               current_hour: float,
                               current_state_dist: np.ndarray = None,
                               target_SOC: float = 0.05,
                               max_hours: float = 48) -> Dict:
        """
        预测电池剩余使用时间
        
        Args:
            current_SOC: 当前SOC
            current_T: 当前温度
            current_hour: 当前时间 (小时)
            current_state_dist: 当前用户状态分布
            target_SOC: 目标SOC (关机阈值)
            max_hours: 最大预测时长
            
        Returns:
            prediction: 预测结果字典
        """
        if current_state_dist is None:
            # 根据时间推断可能的状态分布
            mode = self.system.user_model.get_time_mode(current_hour)
            if mode == "sleep":
                current_state_dist = np.array([0.8, 0.15, 0.03, 0.02])
            elif mode == "work":
                current_state_dist = np.array([0.2, 0.5, 0.2, 0.1])
            else:  # leisure
                current_state_dist = np.array([0.1, 0.3, 0.35, 0.25])
        
        # 运行仿真
        results = self.system.simulate_battery_drain(
            initial_SOC=current_SOC,
            initial_T=current_T,
            duration_hours=max_hours,
            start_hour=current_hour,
            initial_state_dist=current_state_dist
        )
        
        # 找到SOC降到目标值的时间
        SOC_history = results['SOC']
        time_history = results['time']
        
        # 查找SOC首次低于target_SOC的时间
        below_target = np.where(SOC_history <= target_SOC)[0]
        
        if len(below_target) > 0:
            idx = below_target[0]
            remaining_time = time_history[idx]
        else:
            # 外推预测
            if SOC_history[-1] > target_SOC:
                # 使用最后一段的平均耗电速率外推
                avg_drain_rate = (current_SOC - SOC_history[-1]) / max_hours
                if avg_drain_rate > 0:
                    remaining_time = (current_SOC - target_SOC) / avg_drain_rate
                else:
                    remaining_time = float('inf')
            else:
                remaining_time = max_hours
        
        # 计算平均功耗
        avg_power = np.mean(results['power'])
        
        # 计算使用模式统计
        state_probs = np.mean(results['state_distribution'], axis=0)
        
        return {
            'remaining_hours': remaining_time,
            'remaining_minutes': remaining_time * 60,
            'average_power_W': avg_power,
            'simulation_results': results,
            'state_distribution': state_probs,
            'current_SOC': current_SOC,
            'target_SOC': target_SOC,
            'confidence': self._calculate_confidence(results)
        }
    
    def _calculate_confidence(self, results: Dict) -> float:
        """计算预测置信度"""
        # 基于数据平滑度和模型稳定性
        power_std = np.std(results['power'])
        power_mean = np.mean(results['power'])
        
        cv = power_std / (power_mean + 0.001)  # 变异系数
        confidence = np.exp(-cv)  # 变异越大，置信度越低
        
        return np.clip(confidence, 0, 1)
    
    def predict_with_scenarios(self,
                               current_SOC: float,
                               current_T: float,
                               current_hour: float) -> Dict:
        """
        多场景预测 (乐观/中性/悲观)
        """
        scenarios = {
            'optimistic': np.array([0.7, 0.2, 0.08, 0.02]),  # 主要睡眠/轻度使用
            'neutral': np.array([0.3, 0.35, 0.25, 0.1]),     # 混合使用
            'pessimistic': np.array([0.05, 0.15, 0.30, 0.50])  # 重度使用
        }
        
        predictions = {}
        for name, state_dist in scenarios.items():
            pred = self.predict_remaining_time(
                current_SOC, current_T, current_hour,
                current_state_dist=state_dist
            )
            predictions[name] = pred
            
        return predictions

# ============================================================================
# 第七部分：可视化工具
# ============================================================================

def visualize_simulation_results(results: Dict, figsize: Tuple = (14, 12)):
    """
    可视化仿真结果
    """
    fig, axes = plt.subplots(4, 1, figsize=figsize)
    
    time = results['time']
    start_hour = results['start_hour']
    time_of_day = (start_hour + time) % 24
    
    # 子图1: SOC变化
    ax1 = axes[0]
    ax1.plot(time, results['SOC'] * 100, 'b-', linewidth=2)
    ax1.set_ylabel('SOC (%)', fontsize=11)
    ax1.set_ylim([0, 105])
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Battery SOC Over Time', fontsize=12, fontweight='bold')
    ax1.axhline(y=20, color='r', linestyle='--', alpha=0.5, label='Low Battery (20%)')
    ax1.axhline(y=5, color='r', linestyle='-', alpha=0.7, label='Critical (5%)')
    ax1.legend(loc='upper right')
    
    # 子图2: 功耗和电流
    ax2 = axes[1]
    ax2.plot(time, results['power'], 'g-', linewidth=1.5, label='Power (W)')
    ax2.set_ylabel('Power (W)', color='g', fontsize=11)
    ax2.tick_params(axis='y', labelcolor='g')
    ax2.set_ylim([0, max(results['power']) * 1.2])
    ax2.grid(True, alpha=0.3)
    
    ax2b = ax2.twinx()
    ax2b.plot(time, results['current'] * 1000, 'r-', linewidth=1.5, 
              label='Current (mA)', alpha=0.7)
    ax2b.set_ylabel('Current (mA)', color='r', fontsize=11)
    ax2b.tick_params(axis='y', labelcolor='r')
    
    ax2.set_title('Power Consumption and Current', fontsize=12, fontweight='bold')
    
    # 子图3: 温度变化
    ax3 = axes[2]
    ax3.plot(time, results['temperature'], 'orange', linewidth=2)
    ax3.set_ylabel('Temperature (°C)', fontsize=11)
    ax3.axhline(y=45, color='r', linestyle='--', alpha=0.5, label='Warning (45°C)')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Battery Temperature', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right')
    
    # 子图4: 用户状态分布
    ax4 = axes[3]
    states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
    colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D']
    
    state_dist = results['state_distribution']
    ax4.stackplot(time, state_dist.T, labels=states, colors=colors, alpha=0.8)
    ax4.set_ylabel('Probability', fontsize=11)
    ax4.set_xlabel('Time (hours)', fontsize=11)
    ax4.set_ylim([0, 1])
    ax4.legend(loc='upper right', ncol=2)
    ax4.set_title('User State Distribution', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def visualize_prediction(prediction: Dict, figsize: Tuple = (12, 8)):
    """
    可视化预测结果
    """
    fig, axes = plt.subplots(2, 2, figsize=figsize)
    
    results = prediction['simulation_results']
    
    # SOC预测曲线
    ax1 = axes[0, 0]
    ax1.plot(results['time'], results['SOC'] * 100, 'b-', linewidth=2)
    ax1.axhline(y=prediction['target_SOC'] * 100, color='r', 
                linestyle='--', label=f'Target ({prediction["target_SOC"]*100:.0f}%)')
    ax1.axvline(x=prediction['remaining_hours'], color='g', 
                linestyle='--', label=f'Predicted: {prediction["remaining_hours"]:.1f}h')
    ax1.set_xlabel('Time (hours)')
    ax1.set_ylabel('SOC (%)')
    ax1.set_title('SOC Prediction')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 功耗分布
    ax2 = axes[0, 1]
    ax2.hist(results['power'], bins=30, color='green', alpha=0.7, edgecolor='black')
    ax2.axvline(x=prediction['average_power_W'], color='r', 
                linestyle='--', label=f'Average: {prediction["average_power_W"]:.2f}W')
    ax2.set_xlabel('Power (W)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Power Distribution')
    ax2.legend()
    
    # 状态分布饼图
    ax3 = axes[1, 0]
    states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
    colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D']
    ax3.pie(prediction['state_distribution'], labels=states, colors=colors,
            autopct='%1.1f%%', startangle=90)
    ax3.set_title('Average User State Distribution')
    
    # 预测信息文本
    ax4 = axes[1, 1]
    ax4.axis('off')
    info_text = f"""
    预测结果摘要 (Prediction Summary)
    ═══════════════════════════════════
    
    当前SOC: {prediction['current_SOC']*100:.1f}%
    目标SOC: {prediction['target_SOC']*100:.1f}%
    
    预测剩余时间: {prediction['remaining_hours']:.1f} 小时
                 ({prediction['remaining_minutes']:.0f} 分钟)
    
    平均功耗: {prediction['average_power_W']:.2f} W
    
    预测置信度: {prediction['confidence']*100:.1f}%
    """
    ax4.text(0.1, 0.5, info_text, transform=ax4.transAxes, fontsize=11,
             verticalalignment='center', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    return fig

# ============================================================================
# 主程序
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("SOC与能耗关系的连续时间耦合模型")
    print("Continuous-Time SOC-Energy Coupled Model")
    print("=" * 60)
    print()
    
    # 创建系统实例
    system = SOCEnergyCoupledSystem()
    predictor = BatteryLifePredictor(system)
    
    # 场景1: 早上8点开始，SOC 100%，模拟24小时
    print("场景1: 24小时电池放电仿真")
    print("-" * 40)
    
    results = system.simulate_battery_drain(
        initial_SOC=1.0,
        initial_T=25.0,
        duration_hours=24,
        start_hour=8
    )
    
    print(f"初始SOC: 100%")
    print(f"最终SOC: {results['SOC'][-1]*100:.1f}%")
    print(f"平均功耗: {np.mean(results['power']):.2f} W")
    print(f"温度范围: {results['temperature'].min():.1f}°C - {results['temperature'].max():.1f}°C")
    print()
    
    # 场景2: 电池剩余时间预测
    print("场景2: 电池剩余时间预测")
    print("-" * 40)
    
    prediction = predictor.predict_remaining_time(
        current_SOC=0.50,
        current_T=30.0,
        current_hour=14  # 下午2点
    )
    
    print(f"当前SOC: 50%")
    print(f"当前时间: 14:00")
    print(f"预测剩余时间: {prediction['remaining_hours']:.1f} 小时 ({prediction['remaining_minutes']:.0f} 分钟)")
    print(f"平均功耗: {prediction['average_power_W']:.2f} W")
    print(f"预测置信度: {prediction['confidence']*100:.1f}%")
    print()
    
    # 场景3: 多场景预测
    print("场景3: 多场景预测 (乐观/中性/悲观)")
    print("-" * 40)
    
    scenarios = predictor.predict_with_scenarios(
        current_SOC=0.60,
        current_T=28.0,
        current_hour=10
    )
    
    for name, pred in scenarios.items():
        hours = pred['remaining_hours']
        power = pred['average_power_W']
        print(f"  {name:12s}: {hours:5.1f} 小时 (平均功耗: {power:.2f} W)")
    print()
    
    # 可视化
    print("生成可视化图表...")
    
    fig1 = visualize_simulation_results(results)
    fig1.savefig('/workspace/battery_soc_model/simulation_results.png', 
                 dpi=150, bbox_inches='tight')
    print("  保存: simulation_results.png")
    
    fig2 = visualize_prediction(prediction)
    fig2.savefig('/workspace/battery_soc_model/prediction_results.png', 
                 dpi=150, bbox_inches='tight')
    print("  保存: prediction_results.png")
    
    print()
    print("=" * 60)
    print("模型运行完成!")
    print("=" * 60)
