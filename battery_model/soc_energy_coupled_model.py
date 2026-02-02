"""
SOC与能耗耦合连续时间模型
Coupled Continuous-Time Model: SOC-Energy Consumption Relationship

基于以下文档构建:
1. 电池电化学热耦合核心方程
2. 各模块功耗耦合表达式
3. 连续时间马尔科夫用户模型

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional, Callable
import warnings

# ==============================================================================
# 1. 物理参数配置
# ==============================================================================

@dataclass
class BatteryParameters:
    """电池物理参数"""
    Q_nom: float = 4000.0          # 标称容量 (mAh)
    V_nom: float = 3.85            # 标称电压 (V)
    R_int_ref: float = 0.08        # 参考内阻 (Ohm)
    C_th: float = 15.0             # 热容 (J/K)
    R_th: float = 8.0              # 热阻 (K/W)
    T_amb: float = 298.15          # 环境温度 (K)
    eta_conv: float = 0.92         # PMIC效率
    
    # 电化学参数
    alpha_temp: float = 0.004      # 温度对容量的影响系数
    beta_soc: float = 0.5          # SOC对内阻的影响系数
    
    # OCV-SOC关系参数 (多项式系数)
    ocv_coeffs: tuple = (3.0, 0.8, 0.3, -0.1)


@dataclass
class SoCParameters:
    """片上系统(SoC)参数 - 7nm FinFET"""
    C_eff: float = 2.5e-9          # 有效开关电容 (F)
    V_th: float = 0.25             # 阈值电压 (V)
    gamma: float = 1.2             # 速度饱和指数
    K: float = 1.2e9               # 频率-电压关系常数
    
    # 漏电参数
    I_ref: float = 1e-6            # 参考漏电流 (A)
    T_ref: float = 300.0           # 参考温度 (K)
    lambda_DIBL: float = 70e-3     # DIBL系数 (V/V)
    kappa: float = 2e-3            # 阈值电压温度系数
    n_VT: float = 1.2              # 亚阈值斜率因子
    
    # DVFS工作点
    V_dd_min: float = 0.6          # 最小电压 (V)
    V_dd_max: float = 1.1          # 最大电压 (V)
    f_min: float = 0.3e9           # 最小频率 (Hz)
    f_max: float = 3.0e9           # 最大频率 (Hz)


@dataclass
class DisplayParameters:
    """显示模块参数 - OLED LTPO"""
    P_static: float = 0.1          # 静态功耗 (W)
    k_drv: float = 0.0015          # 驱动系数
    gamma_oled: float = 2.2        # OLED伽马值
    f_refresh_min: float = 1.0     # 最小刷新率 (Hz)
    f_refresh_max: float = 120.0   # 最大刷新率 (Hz)
    P_per_nit: float = 0.002       # 每尼特功耗 (W/nit)


@dataclass
class WirelessParameters:
    """无线模块参数"""
    # 5G参数
    P_5G_base: float = 0.5         # 5G基础功耗 (W)
    P_5G_per_Mbps: float = 0.01    # 每Mbps功耗 (W)
    path_loss_exp: float = 3.5     # 路径损耗指数
    
    # GNSS参数
    P_GPS_acq: float = 0.15        # 捕获模式功耗 (W)
    P_GPS_track: float = 0.03      # 跟踪模式功耗 (W)
    snr_threshold: float = 30.0    # 锁定阈值 (dB)
    k_sigmoid: float = 0.5         # Sigmoid陡峭因子
    
    # 蓝牙参数
    P_BT_active: float = 0.08      # 活跃功耗 (W)
    P_BT_sleep: float = 0.001      # 休眠功耗 (W)
    conn_interval: float = 100.0   # 连接间隔 (ms)


# ==============================================================================
# 2. 核心耦合微分方程
# ==============================================================================

class CoupledSOCEnergyModel:
    """
    SOC-能耗耦合连续时间模型
    
    核心微分方程:
    dSOC/dt = -I_total(t) / (Q_nom * eta_temp(T) * N_cycle)
    dT/dt = (P_joule + P_entropy - (T - T_amb)/R_th) / C_th
    
    其中总电流耦合方程:
    I_total = (P_SoC + P_disp + P_5G + P_BT + P_GNSS + P_bg) / (eta_PMIC * V_batt)
    """
    
    def __init__(self,
                 battery_params: BatteryParameters = None,
                 soc_params: SoCParameters = None,
                 display_params: DisplayParameters = None,
                 wireless_params: WirelessParameters = None):
        
        self.bp = battery_params or BatteryParameters()
        self.sp = soc_params or SoCParameters()
        self.dp = display_params or DisplayParameters()
        self.wp = wireless_params or WirelessParameters()
        
        # 状态历史记录
        self.history = {
            'time': [],
            'soc': [],
            'temperature': [],
            'power_total': [],
            'current': [],
            'voltage': []
        }
    
    def ocv_from_soc(self, soc: float) -> float:
        """开路电压-SOC关系 (多项式模型)"""
        soc = np.clip(soc, 0.0, 1.0)
        a0, a1, a2, a3 = self.bp.ocv_coeffs
        return a0 + a1 * soc + a2 * soc**2 + a3 * soc**3
    
    def internal_resistance(self, soc: float, T: float) -> float:
        """内阻模型 (SOC和温度依赖)"""
        # R_int随SOC降低而增加,随温度升高而降低
        soc_factor = 1.0 + self.bp.beta_soc * (1.0 - soc)**2
        temp_factor = 1.0 - 0.005 * (T - self.bp.T_amb)
        return self.bp.R_int_ref * soc_factor * max(0.5, temp_factor)
    
    def temperature_efficiency(self, T: float) -> float:
        """温度效率因子"""
        T_celsius = T - 273.15
        if T_celsius < 0:
            return 0.7
        elif T_celsius < 10:
            return 0.7 + 0.03 * T_celsius
        elif T_celsius < 45:
            return 1.0
        else:
            return 1.0 - 0.01 * (T_celsius - 45)
    
    def compute_soc_power(self, frequency: float, T: float, load: float) -> float:
        """
        SoC功耗计算 (动态+漏电)
        P_SoC = P_dyn + P_leak
        """
        # DVFS控制: 根据负载确定工作点
        f = np.clip(frequency, self.sp.f_min, self.sp.f_max)
        
        # 反推电压 (Alpha-Power Law)
        f_norm = (f - self.sp.f_min) / (self.sp.f_max - self.sp.f_min)
        V_dd = self.sp.V_dd_min + (self.sp.V_dd_max - self.sp.V_dd_min) * f_norm**0.8
        
        # 动态功耗: P_dyn = alpha * C_eff * V_dd^2 * f
        alpha = 0.1 + 0.6 * load  # 活动因子
        P_dyn = alpha * self.sp.C_eff * V_dd**2 * f
        
        # 漏电功耗 (温度相关)
        k_B = 8.617e-5  # 玻尔兹曼常数 (eV/K)
        V_T = k_B * T   # 热电压
        
        exp_arg = (self.sp.lambda_DIBL * V_dd + self.sp.kappa * (T - self.sp.T_ref)) / (self.sp.n_VT * V_T)
        exp_arg = np.clip(exp_arg, -50, 50)  # 防止溢出
        
        I_sub = self.sp.I_ref * (T / self.sp.T_ref)**2 * np.exp(exp_arg)
        P_leak = V_dd * I_sub
        
        return P_dyn + P_leak
    
    def compute_display_power(self, brightness_nits: float, 
                               refresh_rate: float, apl: float) -> float:
        """
        显示模块功耗
        P_disp = P_static + k_drv * f_refresh * L^gamma * APL
        """
        if brightness_nits <= 0:
            return 0.0
        
        # LTPO自适应刷新率
        f_refresh = np.clip(refresh_rate, self.dp.f_refresh_min, self.dp.f_refresh_max)
        
        # 亮度非线性功耗 (伽马校正)
        L_normalized = brightness_nits / 1000.0
        P_emissive = self.dp.P_per_nit * brightness_nits * (apl / 100.0)**self.dp.gamma_oled
        
        # 驱动功耗
        P_driver = self.dp.k_drv * f_refresh * (apl / 100.0)
        
        return self.dp.P_static + P_emissive + P_driver
    
    def compute_5g_power(self, data_rate_mbps: float, distance_m: float) -> float:
        """
        5G模块功耗 (信道-距离耦合)
        基于Shannon定理和Friis传输方程
        """
        if data_rate_mbps <= 0:
            return 0.0
        
        # 路径损耗因子
        path_loss = (distance_m / 100.0)**self.wp.path_loss_exp
        
        # Shannon容量约束下的发射功率
        P_tx = (2**(data_rate_mbps / 100.0) - 1) * path_loss * 0.01
        
        return self.wp.P_5G_base + self.wp.P_5G_per_Mbps * data_rate_mbps + P_tx
    
    def compute_gnss_power(self, snr: float) -> float:
        """
        GNSS功耗 (状态机Sigmoid平滑)
        """
        # Sigmoid状态切换
        lock_prob = 1.0 / (1.0 + np.exp(-self.wp.k_sigmoid * (snr - self.wp.snr_threshold)))
        
        # 加权功耗
        return lock_prob * self.wp.P_GPS_track + (1.0 - lock_prob) * self.wp.P_GPS_acq
    
    def compute_bluetooth_power(self, is_audio: bool, conn_interval_ms: float) -> float:
        """
        蓝牙功耗 (占空比模型)
        """
        if conn_interval_ms <= 0:
            return self.wp.P_BT_sleep
        
        # 双曲线衰减: I_avg = I_sleep + Q_event / T_interval
        Q_event = 0.1  # mC per event
        I_avg = self.wp.P_BT_sleep + Q_event / conn_interval_ms
        
        if is_audio:
            I_avg += 0.05  # DSP编解码功耗
        
        return I_avg
    
    def compute_background_power(self, t: float, base_power: float = 0.2) -> float:
        """
        后台任务功耗 (O-U随机过程)
        dI(t) = theta * (mu - I(t)) * dt + sigma * dW
        """
        theta = 0.5    # 回归速率
        mu = base_power
        sigma = 0.05   # 波动幅度
        
        # 简化的O-U过程模拟
        noise = sigma * np.random.randn()
        return max(0.05, mu + noise)
    
    def compute_total_power(self, state: dict, T: float) -> float:
        """计算总功耗"""
        P_soc = self.compute_soc_power(state['frequency'], T, state['cpu_load'])
        P_disp = self.compute_display_power(state['brightness'], 
                                            state['refresh_rate'], 
                                            state['apl'])
        P_5g = self.compute_5g_power(state['data_rate'], state['distance'])
        P_gnss = self.compute_gnss_power(state['snr']) if state.get('gnss_on', False) else 0
        P_bt = self.compute_bluetooth_power(state.get('bt_audio', False), 
                                           state.get('bt_interval', 0))
        P_bg = self.compute_background_power(0)
        
        return P_soc + P_disp + P_5g + P_gnss + P_bt + P_bg
    
    def coupled_ode_system(self, t: float, y: np.ndarray, 
                           state_func: Callable) -> np.ndarray:
        """
        耦合ODE系统
        
        状态向量 y = [SOC, T]
        
        dSOC/dt = -I_batt / (Q_nom * eta_temp)
        dT/dt = (P_total - (T - T_amb)/R_th) / C_th
        """
        SOC, T = y
        # 约束状态在物理范围内
        SOC = np.clip(SOC, 0.0, 1.0)
        T = np.clip(T, 273.15, 373.15)
        
        # 如果SOC已经耗尽,停止消耗
        if SOC <= 0.0:
            return np.array([0.0, -(T - self.bp.T_amb) / (self.bp.R_th * self.bp.C_th)])
        
        # 获取当前硬件状态
        state = state_func(t)
        
        # 计算总功耗
        P_total = self.compute_total_power(state, T)
        
        # 电池电压
        V_ocv = self.ocv_from_soc(SOC)
        R_int = self.internal_resistance(SOC, T)
        
        # 电池电流 (迭代求解)
        # P = V * I = (V_ocv - I*R_int) * I
        # I^2 * R_int - I * V_ocv + P/eta = 0
        P_load = P_total / self.bp.eta_conv
        
        discriminant = V_ocv**2 - 4 * R_int * P_load
        if discriminant < 0:
            I_batt = V_ocv / (2 * R_int)  # 最大电流
        else:
            I_batt = (V_ocv - np.sqrt(discriminant)) / (2 * R_int)
        
        I_batt = max(0, I_batt)
        
        # 温度效率
        eta_temp = self.temperature_efficiency(T)
        
        # SOC变化率 (mAh -> Ah, 注意单位转换)
        # Q_nom 是 mAh, I_batt 是 A, 需要转换为每秒的SOC变化
        dSOC_dt = -I_batt / (self.bp.Q_nom / 1000 * 3600 * eta_temp)
        
        # 温度变化率
        # 热生成 = I^2 * R_int (焦耳热) + 熵热
        P_joule = I_batt**2 * R_int
        P_entropy = 0.01 * I_batt * T  # 简化的熵热
        dT_dt = (P_joule + P_entropy - (T - self.bp.T_amb) / self.bp.R_th) / self.bp.C_th
        
        return np.array([dSOC_dt, dT_dt])
    
    def simulate(self, t_span: Tuple[float, float], 
                 y0: np.ndarray,
                 state_func: Callable,
                 t_eval: np.ndarray = None) -> dict:
        """
        运行耦合仿真
        
        Parameters:
        -----------
        t_span : tuple
            仿真时间范围 (t_start, t_end) 单位:秒
        y0 : array
            初始状态 [SOC_0, T_0]
        state_func : callable
            硬件状态函数 state_func(t) -> dict
        t_eval : array, optional
            评估时间点
        
        Returns:
        --------
        result : dict
            仿真结果
        """
        if t_eval is None:
            t_eval = np.linspace(t_span[0], t_span[1], 1000)
        
        # 定义SOC耗尽事件
        def soc_depleted(t, y):
            return y[0] - 0.001  # SOC降至0.1%时停止
        soc_depleted.terminal = True
        soc_depleted.direction = -1
        
        # 求解ODE
        solution = solve_ivp(
            lambda t, y: self.coupled_ode_system(t, y, state_func),
            t_span,
            y0,
            method='RK45',
            t_eval=t_eval,
            max_step=60.0,  # 最大步长60秒
            rtol=1e-6,
            atol=1e-9,
            events=soc_depleted
        )
        
        # 计算附加信息
        P_total = []
        I_batt = []
        V_batt = []
        
        for i, t in enumerate(solution.t):
            SOC, T = solution.y[:, i]
            state = state_func(t)
            
            P = self.compute_total_power(state, T)
            V_ocv = self.ocv_from_soc(SOC)
            R_int = self.internal_resistance(SOC, T)
            
            P_load = P / self.bp.eta_conv
            discriminant = V_ocv**2 - 4 * R_int * P_load
            if discriminant < 0:
                I = V_ocv / (2 * R_int)
            else:
                I = (V_ocv - np.sqrt(discriminant)) / (2 * R_int)
            
            V = V_ocv - I * R_int
            
            P_total.append(P)
            I_batt.append(I)
            V_batt.append(V)
        
        # 确保SOC不为负
        soc_clipped = np.clip(solution.y[0], 0.0, 1.0)
        
        return {
            'time': solution.t,
            'soc': soc_clipped,
            'temperature': solution.y[1],
            'power_total': np.array(P_total),
            'current': np.array(I_batt),
            'voltage': np.array(V_batt),
            'success': solution.success,
            'depleted': len(solution.t_events[0]) > 0 if hasattr(solution, 't_events') and solution.t_events else False
        }
    
    def predict_remaining_time(self, current_soc: float, 
                                current_temp: float,
                                state_func: Callable,
                                soc_threshold: float = 0.05) -> float:
        """
        预测剩余使用时间
        
        使用二分搜索找到SOC降至阈值的时间
        """
        # 最大预测时间: 48小时
        t_max = 48 * 3600
        
        def check_soc(t_end):
            result = self.simulate(
                (0, t_end),
                np.array([current_soc, current_temp]),
                state_func,
                t_eval=np.array([t_end])
            )
            return result['soc'][-1]
        
        # 二分搜索
        t_low, t_high = 0, t_max
        
        while t_high - t_low > 60:  # 精度1分钟
            t_mid = (t_low + t_high) / 2
            soc_mid = check_soc(t_mid)
            
            if soc_mid > soc_threshold:
                t_low = t_mid
            else:
                t_high = t_mid
        
        return t_mid


# ==============================================================================
# 3. 便捷函数
# ==============================================================================

def create_default_model() -> CoupledSOCEnergyModel:
    """创建默认配置的模型"""
    return CoupledSOCEnergyModel(
        BatteryParameters(),
        SoCParameters(),
        DisplayParameters(),
        WirelessParameters()
    )


def example_state_function(t: float) -> dict:
    """示例硬件状态函数"""
    # 基于时间的状态变化
    hour = (t / 3600) % 24
    
    if 23 <= hour or hour < 7:  # 睡眠时段
        return {
            'frequency': 0.3e9,
            'cpu_load': 0.02,
            'brightness': 0,
            'refresh_rate': 1,
            'apl': 0,
            'data_rate': 0,
            'distance': 100,
            'snr': 40,
            'gnss_on': False,
            'bt_audio': False,
            'bt_interval': 1000
        }
    elif 9 <= hour < 12 or 14 <= hour < 18:  # 工作时段
        return {
            'frequency': 1.5e9,
            'cpu_load': 0.3,
            'brightness': 500,
            'refresh_rate': 60,
            'apl': 50,
            'data_rate': 50,
            'distance': 200,
            'snr': 35,
            'gnss_on': False,
            'bt_audio': False,
            'bt_interval': 100
        }
    else:  # 休闲时段
        return {
            'frequency': 2.0e9,
            'cpu_load': 0.5,
            'brightness': 700,
            'refresh_rate': 90,
            'apl': 60,
            'data_rate': 100,
            'distance': 150,
            'snr': 38,
            'gnss_on': True,
            'bt_audio': True,
            'bt_interval': 50
        }


if __name__ == "__main__":
    # 测试模型
    model = create_default_model()
    
    # 初始状态
    SOC_0 = 1.0
    T_0 = 298.15
    
    # 仿真8小时
    result = model.simulate(
        t_span=(0, 8 * 3600),
        y0=np.array([SOC_0, T_0]),
        state_func=example_state_function
    )
    
    print(f"仿真完成: {result['success']}")
    print(f"最终SOC: {result['soc'][-1]:.2%}")
    print(f"最终温度: {result['temperature'][-1] - 273.15:.1f}°C")
    print(f"平均功耗: {np.mean(result['power_total']):.2f} W")
