"""
智能手机电池连续时间数学模型
Smartphone Battery Continuous-Time Mathematical Model

基于锂离子电池的电化学特性和能量守恒原理建立的SOC（充电状态）耗电模型

模型基础：
1. 库仑计数法：SOC的变化与电流积分成正比
2. 能量守恒：电池释放的能量等于各组件消耗的能量之和
3. 温度效应：基于Arrhenius方程的温度修正因子
4. 老化效应：考虑循环次数对容量的影响

参考文献：
[1] Plett, G.L. "Battery Management Systems, Volume I: Battery Modeling" Artech House, 2015
[2] Tremblay, O., Dessaint, L.A. "Experimental validation of a battery dynamic model for EV applications" 
    World Electric Vehicle Journal, 2009
[3] He, H., et al. "State-of-Charge Estimation of the Lithium-Ion Battery Using an Adaptive Extended 
    Kalman Filter" IEEE Transactions on Vehicular Technology, 2011

作者: Battery Model System
日期: 2026
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
from scipy.optimize import minimize, differential_evolution
from dataclasses import dataclass, field
from typing import Callable, List, Tuple, Optional, Dict
import warnings


@dataclass
class BatteryParameters:
    """
    电池参数数据类
    
    参数来源与依据：
    - 典型智能手机锂离子电池规格（如Samsung SDI, LG Chem产品规格书）
    - 学术文献中的实测数据
    """
    # 电池基本参数
    nominal_capacity: float = 4000.0  # 标称容量 (mAh) - 典型智能手机电池
    nominal_voltage: float = 3.85  # 标称电压 (V) - 锂离子电池标准
    max_voltage: float = 4.35  # 最大电压 (V)
    min_voltage: float = 3.0  # 最小电压 (V) - 截止电压
    
    # 内阻参数 (Ω) - 基于等效电路模型
    internal_resistance: float = 0.08  # 内阻 - 典型值0.05-0.15Ω
    
    # 温度相关参数
    reference_temperature: float = 25.0  # 参考温度 (°C)
    activation_energy: float = 20000.0  # 活化能 (J/mol) - Arrhenius方程
    
    # 老化参数
    cycle_count: int = 0  # 充放电循环次数
    capacity_fade_rate: float = 0.0002  # 每次循环容量衰减率 (典型值0.01-0.03%/cycle)
    
    # 自放电参数
    self_discharge_rate: float = 0.0001  # 自放电率 (%/hour) - 锂离子电池约2-3%/month


@dataclass
class UsageProfile:
    """
    使用模式参数
    
    功耗数据来源：
    - Android Battery Historian分析数据
    - 学术论文中的实测功耗数据
    - 设备制造商规格参数
    """
    # 屏幕参数
    screen_size: float = 6.5  # 屏幕尺寸 (inches)
    screen_brightness: float = 0.5  # 亮度比例 (0-1)
    screen_on: bool = True  # 屏幕是否开启
    refresh_rate: int = 60  # 刷新率 (Hz)
    
    # 处理器参数
    cpu_load: float = 0.2  # CPU负载比例 (0-1)
    gpu_load: float = 0.0  # GPU负载比例 (0-1)
    
    # 网络参数
    wifi_active: bool = True  # WiFi是否活跃
    cellular_active: bool = True  # 蜂窝网络是否活跃
    cellular_signal_strength: float = 0.7  # 信号强度 (0-1, 1为最强)
    data_transfer_rate: float = 0.0  # 数据传输率 (Mbps)
    
    # GPS参数
    gps_active: bool = False  # GPS是否活跃
    gps_accuracy: str = 'high'  # GPS精度: 'high', 'balanced', 'low'
    
    # 其他参数
    bluetooth_active: bool = False  # 蓝牙是否活跃
    nfc_active: bool = False  # NFC是否活跃
    background_apps_count: int = 5  # 后台应用数量
    
    # 环境温度
    ambient_temperature: float = 25.0  # 环境温度 (°C)


@dataclass
class PowerConsumptionCoefficients:
    """
    功耗系数 - 基于实测数据和文献值
    
    数据来源：
    [1] Carroll, A., Heiser, G. "An Analysis of Power Consumption in a Smartphone" 
        USENIX Annual Technical Conference, 2010
    [2] Pathak, A., et al. "Fine-Grained Power Modeling for Smartphones Using System Call Tracing"
        EuroSys, 2011
    [3] Zhang, L., et al. "Accurate Online Power Estimation and Automatic Battery Behavior Based 
        Power Model Generation for Smartphones" CODES+ISSS, 2010
    """
    # 屏幕功耗系数 (mW)
    screen_base_power: float = 100.0  # 屏幕基础功耗
    screen_brightness_coeff: float = 400.0  # 亮度系数 (mW per brightness unit)
    screen_size_coeff: float = 50.0  # 尺寸系数 (mW per inch)
    screen_refresh_coeff: float = 2.0  # 刷新率系数 (mW per Hz above 60)
    
    # 处理器功耗系数 (mW)
    cpu_idle_power: float = 50.0  # CPU空闲功耗
    cpu_active_power: float = 1500.0  # CPU满载功耗
    gpu_idle_power: float = 20.0  # GPU空闲功耗  
    gpu_active_power: float = 2000.0  # GPU满载功耗
    
    # 网络功耗系数 (mW)
    wifi_idle_power: float = 10.0  # WiFi空闲功耗
    wifi_active_power: float = 300.0  # WiFi活跃功耗
    wifi_transfer_coeff: float = 20.0  # WiFi传输功耗系数 (mW per Mbps)
    
    cellular_idle_power: float = 20.0  # 蜂窝网络空闲功耗
    cellular_active_power: float = 500.0  # 蜂窝网络活跃功耗
    cellular_signal_coeff: float = 200.0  # 信号强度功耗系数 (弱信号增加功耗)
    cellular_transfer_coeff: float = 30.0  # 蜂窝传输功耗系数
    
    # GPS功耗系数 (mW)
    gps_high_power: float = 400.0  # 高精度GPS功耗
    gps_balanced_power: float = 200.0  # 平衡精度GPS功耗
    gps_low_power: float = 50.0  # 低精度GPS功耗
    
    # 其他组件功耗 (mW)
    bluetooth_idle_power: float = 5.0  # 蓝牙空闲功耗
    bluetooth_active_power: float = 50.0  # 蓝牙活跃功耗
    nfc_power: float = 30.0  # NFC功耗
    
    # 后台进程功耗
    background_app_power: float = 15.0  # 每个后台应用的平均功耗 (mW)
    
    # 系统基础功耗
    base_system_power: float = 30.0  # 系统基础功耗 (mW)


class SmartphoneBatteryModel:
    """
    智能手机电池连续时间数学模型
    
    核心方程：
    ────────────────────────────────────────────────────────────────────────
    
    1. SOC动力学方程（主方程）：
    
       dSOC/dt = -P_total(t) / (V(SOC) * Q_eff) - k_self * SOC
    
       其中：
       - SOC: 充电状态 (0-1)
       - P_total(t): 总功耗 (W)
       - V(SOC): 开路电压，是SOC的函数 (V)
       - Q_eff: 有效容量 (Ah)，考虑温度和老化效应
       - k_self: 自放电系数 (1/h)
    
    2. 开路电压-SOC关系（基于Shepherd模型改进）：
    
       V(SOC) = V0 - K * (1-SOC)/SOC - R*I + A*exp(-B*(1-SOC))
    
       其中：
       - V0: 电池常数电压
       - K: 极化常数
       - R: 内阻
       - I: 放电电流
       - A, B: 指数区参数
    
    3. 有效容量（温度和老化修正）：
    
       Q_eff = Q_nom * f_T(T) * f_age(n)
       
       f_T(T) = exp(-Ea/R * (1/T - 1/T_ref))  # Arrhenius温度因子
       f_age(n) = 1 - α * n                    # 老化因子
    
    4. 总功耗模型（组件叠加）：
    
       P_total = P_screen + P_cpu + P_gpu + P_network + P_gps + P_other
    
    ────────────────────────────────────────────────────────────────────────
    """
    
    # 物理常数
    R_GAS = 8.314  # 气体常数 (J/(mol·K))
    
    def __init__(self, 
                 battery_params: BatteryParameters = None,
                 power_coeffs: PowerConsumptionCoefficients = None):
        """初始化电池模型"""
        self.battery_params = battery_params or BatteryParameters()
        self.power_coeffs = power_coeffs or PowerConsumptionCoefficients()
        
        # 电压模型参数 (基于锂离子电池特性曲线拟合)
        # 参考: Tremblay & Dessaint (2009)
        self.V0 = 4.2  # 满充电压
        self.K = 0.1  # 极化电压常数
        self.A = 0.2  # 指数区振幅
        self.B = 3.0  # 指数区时间常数逆数
        
    def open_circuit_voltage(self, soc: float) -> float:
        """
        计算开路电压 (OCV) 作为SOC的函数
        
        使用改进的Shepherd模型：
        V(SOC) = V0 - K*(Q/(Q-q)) + A*exp(-B*q/Q)
        
        简化为SOC表示：
        V(SOC) = V0 - K/SOC + A*exp(-B*(1-SOC))
        
        参数:
            soc: 充电状态 (0-1)
            
        返回:
            开路电压 (V)
        """
        soc = np.clip(soc, 0.01, 1.0)  # 避免除零
        
        # 改进的Shepherd模型
        ocv = (self.V0 
               - self.K * (1 - soc) / soc  # 极化项
               + self.A * np.exp(-self.B * (1 - soc)))  # 指数项
        
        # 限制在物理范围内
        return np.clip(ocv, self.battery_params.min_voltage, 
                       self.battery_params.max_voltage)
    
    def temperature_factor(self, temperature: float) -> float:
        """
        计算温度修正因子 (基于Arrhenius方程)
        
        f_T = exp(-Ea/R * (1/T - 1/T_ref))
        
        在低温下电池容量降低，高温下可能加速老化
        
        参数:
            temperature: 温度 (°C)
            
        返回:
            温度修正因子 (0-1)
        """
        T = temperature + 273.15  # 转换为开尔文
        T_ref = self.battery_params.reference_temperature + 273.15
        Ea = self.battery_params.activation_energy
        
        factor = np.exp(-Ea / self.R_GAS * (1/T - 1/T_ref))
        
        # 低温下容量显著下降的额外修正
        if temperature < 0:
            factor *= np.exp(0.02 * temperature)  # 低温惩罚
        elif temperature > 45:
            factor *= np.exp(-0.01 * (temperature - 45))  # 高温惩罚
            
        return np.clip(factor, 0.3, 1.2)
    
    def aging_factor(self) -> float:
        """
        计算老化修正因子
        
        f_age = 1 - α * n
        
        其中 α 是每循环容量衰减率，n 是循环次数
        
        返回:
            老化修正因子 (0.7-1.0)
        """
        n = self.battery_params.cycle_count
        alpha = self.battery_params.capacity_fade_rate
        
        # 线性老化模型
        factor = 1 - alpha * n
        
        # 考虑非线性老化（深度放电和高温加速老化）
        if n > 500:
            factor *= np.exp(-0.0001 * (n - 500))
            
        return np.clip(factor, 0.7, 1.0)
    
    def effective_capacity(self, temperature: float) -> float:
        """
        计算有效容量
        
        Q_eff = Q_nom * f_T(T) * f_age(n)
        
        参数:
            temperature: 温度 (°C)
            
        返回:
            有效容量 (mAh)
        """
        Q_nom = self.battery_params.nominal_capacity
        f_T = self.temperature_factor(temperature)
        f_age = self.aging_factor()
        
        return Q_nom * f_T * f_age
    
    def calculate_screen_power(self, usage: UsageProfile) -> float:
        """
        计算屏幕功耗
        
        P_screen = (P_base + α_b * brightness + α_s * size + α_r * (refresh-60)) * I_on
        
        参数:
            usage: 使用配置
            
        返回:
            屏幕功耗 (mW)
        """
        if not usage.screen_on:
            return 0.0
        
        c = self.power_coeffs
        
        power = (c.screen_base_power 
                 + c.screen_brightness_coeff * usage.screen_brightness
                 + c.screen_size_coeff * (usage.screen_size - 5.0)  # 基准5英寸
                 + c.screen_refresh_coeff * max(0, usage.refresh_rate - 60))
        
        return power
    
    def calculate_cpu_power(self, usage: UsageProfile) -> float:
        """
        计算CPU功耗
        
        P_cpu = P_idle + (P_active - P_idle) * load^β
        
        β考虑DVFS（动态电压频率调节）的非线性效应
        
        参数:
            usage: 使用配置
            
        返回:
            CPU功耗 (mW)
        """
        c = self.power_coeffs
        load = usage.cpu_load
        
        # 非线性功耗模型 (考虑DVFS)
        beta = 1.5  # 功耗-负载非线性系数
        power = c.cpu_idle_power + (c.cpu_active_power - c.cpu_idle_power) * (load ** beta)
        
        return power
    
    def calculate_gpu_power(self, usage: UsageProfile) -> float:
        """
        计算GPU功耗
        
        参数:
            usage: 使用配置
            
        返回:
            GPU功耗 (mW)
        """
        c = self.power_coeffs
        load = usage.gpu_load
        
        power = c.gpu_idle_power + (c.gpu_active_power - c.gpu_idle_power) * (load ** 1.3)
        
        return power
    
    def calculate_network_power(self, usage: UsageProfile) -> float:
        """
        计算网络功耗 (WiFi + 蜂窝网络)
        
        网络功耗模型考虑：
        - 空闲功耗
        - 活跃功耗
        - 数据传输功耗
        - 信号强度影响（弱信号需要更高发射功率）
        
        参数:
            usage: 使用配置
            
        返回:
            网络总功耗 (mW)
        """
        c = self.power_coeffs
        power = 0.0
        
        # WiFi功耗
        if usage.wifi_active:
            wifi_power = c.wifi_active_power
            wifi_power += c.wifi_transfer_coeff * usage.data_transfer_rate
            power += wifi_power
        else:
            power += c.wifi_idle_power
        
        # 蜂窝网络功耗
        if usage.cellular_active:
            cell_power = c.cellular_active_power
            # 信号强度影响（弱信号增加功耗）
            signal_factor = 1 + c.cellular_signal_coeff * (1 - usage.cellular_signal_strength) / 500
            cell_power *= signal_factor
            cell_power += c.cellular_transfer_coeff * usage.data_transfer_rate
            power += cell_power
        else:
            power += c.cellular_idle_power
        
        return power
    
    def calculate_gps_power(self, usage: UsageProfile) -> float:
        """
        计算GPS功耗
        
        参数:
            usage: 使用配置
            
        返回:
            GPS功耗 (mW)
        """
        if not usage.gps_active:
            return 0.0
        
        c = self.power_coeffs
        
        if usage.gps_accuracy == 'high':
            return c.gps_high_power
        elif usage.gps_accuracy == 'balanced':
            return c.gps_balanced_power
        else:
            return c.gps_low_power
    
    def calculate_other_power(self, usage: UsageProfile) -> float:
        """
        计算其他组件功耗 (蓝牙、NFC、后台应用等)
        
        参数:
            usage: 使用配置
            
        返回:
            其他功耗 (mW)
        """
        c = self.power_coeffs
        power = c.base_system_power
        
        # 蓝牙
        if usage.bluetooth_active:
            power += c.bluetooth_active_power
        else:
            power += c.bluetooth_idle_power
        
        # NFC
        if usage.nfc_active:
            power += c.nfc_power
        
        # 后台应用
        power += c.background_app_power * usage.background_apps_count
        
        return power
    
    def total_power_consumption(self, usage: UsageProfile) -> float:
        """
        计算总功耗
        
        P_total = P_screen + P_cpu + P_gpu + P_network + P_gps + P_other
        
        参数:
            usage: 使用配置
            
        返回:
            总功耗 (mW)
        """
        P_screen = self.calculate_screen_power(usage)
        P_cpu = self.calculate_cpu_power(usage)
        P_gpu = self.calculate_gpu_power(usage)
        P_network = self.calculate_network_power(usage)
        P_gps = self.calculate_gps_power(usage)
        P_other = self.calculate_other_power(usage)
        
        return P_screen + P_cpu + P_gpu + P_network + P_gps + P_other
    
    def power_breakdown(self, usage: UsageProfile) -> Dict[str, float]:
        """
        获取功耗分解
        
        返回各组件的功耗详情
        """
        return {
            'screen': self.calculate_screen_power(usage),
            'cpu': self.calculate_cpu_power(usage),
            'gpu': self.calculate_gpu_power(usage),
            'network': self.calculate_network_power(usage),
            'gps': self.calculate_gps_power(usage),
            'other': self.calculate_other_power(usage),
            'total': self.total_power_consumption(usage)
        }
    
    def soc_dynamics(self, t: float, soc: float, usage: UsageProfile) -> float:
        """
        SOC动力学方程 - 核心微分方程
        
        dSOC/dt = -P_total / (V(SOC) * Q_eff) - k_self * SOC
        
        单位转换：
        - P_total: mW
        - V: V
        - Q_eff: mAh
        - t: hours
        
        dSOC/dt = -P_total(mW) / (V(V) * Q_eff(mAh)) * (1h/1h) * (1000mW/W) / 1000
                = -P_total / (V * Q_eff * 1000) * 1000
                = -P_total / (V * Q_eff)  [单位: 1/h]
        
        参数:
            t: 时间 (hours)
            soc: 当前SOC (0-1)
            usage: 使用配置
            
        返回:
            dSOC/dt (1/hours)
        """
        # 确保SOC在有效范围内
        soc = np.clip(soc, 0.01, 1.0)
        
        # 计算总功耗 (mW)
        P_total = self.total_power_consumption(usage)
        
        # 计算开路电压 (V)
        V = self.open_circuit_voltage(soc)
        
        # 计算有效容量 (mAh)
        Q_eff = self.effective_capacity(usage.ambient_temperature)
        
        # 自放电系数 (1/h)
        k_self = self.battery_params.self_discharge_rate
        
        # SOC变化率
        # P_total (mW) = P_total * 10^-3 W
        # Q_eff (mAh) = Q_eff * 10^-3 Ah
        # 电流 I = P/V (A)
        # dSOC/dt = -I/Q = -P/(V*Q)
        
        # 单位：mW / (V * mAh) = 10^-3 W / (V * 10^-3 Ah) = W / (V * Ah) = A / Ah = 1/h
        dSOC_dt = -P_total / (V * Q_eff) - k_self * soc
        
        return dSOC_dt
    
    def simulate(self, 
                 initial_soc: float,
                 usage: UsageProfile,
                 duration_hours: float,
                 time_points: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
        """
        模拟电池放电过程
        
        使用scipy.integrate.solve_ivp求解ODE
        
        参数:
            initial_soc: 初始SOC (0-1)
            usage: 使用配置
            duration_hours: 模拟时长 (hours)
            time_points: 时间点数量
            
        返回:
            时间数组 (hours), SOC数组
        """
        t_span = (0, duration_hours)
        t_eval = np.linspace(0, duration_hours, time_points)
        
        # 定义ODE函数
        def ode_func(t, y):
            return self.soc_dynamics(t, y[0], usage)
        
        # 设置事件：SOC降到0时停止
        def soc_zero(t, y):
            return y[0] - 0.01
        soc_zero.terminal = True
        soc_zero.direction = -1
        
        # 求解ODE
        sol = solve_ivp(ode_func, t_span, [initial_soc], 
                        t_eval=t_eval, events=soc_zero, 
                        method='RK45', dense_output=True)
        
        return sol.t, sol.y[0]
    
    def estimate_remaining_time(self, 
                                 current_soc: float, 
                                 usage: UsageProfile,
                                 target_soc: float = 0.05) -> float:
        """
        估计剩余使用时间
        
        通过求解SOC到达目标值的时间
        
        参数:
            current_soc: 当前SOC (0-1)
            usage: 使用配置
            target_soc: 目标SOC (默认5%，低电量警告阈值)
            
        返回:
            剩余时间 (hours)
        """
        if current_soc <= target_soc:
            return 0.0
        
        # 使用数值积分估计
        max_time = 48.0  # 最大模拟48小时
        t, soc = self.simulate(current_soc, usage, max_time, time_points=5000)
        
        # 找到SOC首次降到目标值的时间
        idx = np.where(soc <= target_soc)[0]
        if len(idx) > 0:
            return t[idx[0]]
        else:
            return max_time  # 超过最大时间
    
    def analytical_remaining_time(self, 
                                   current_soc: float,
                                   usage: UsageProfile,
                                   target_soc: float = 0.05) -> float:
        """
        剩余时间的解析近似
        
        假设电压近似恒定，忽略自放电：
        dSOC/dt ≈ -P / (V_avg * Q_eff)
        
        积分得：
        t = (SOC_0 - SOC_target) * V_avg * Q_eff / P
        
        参数:
            current_soc: 当前SOC
            usage: 使用配置
            target_soc: 目标SOC
            
        返回:
            剩余时间近似值 (hours)
        """
        if current_soc <= target_soc:
            return 0.0
        
        # 平均电压（取中点SOC对应的电压）
        avg_soc = (current_soc + target_soc) / 2
        V_avg = self.open_circuit_voltage(avg_soc)
        
        # 有效容量
        Q_eff = self.effective_capacity(usage.ambient_temperature)
        
        # 总功耗
        P_total = self.total_power_consumption(usage)
        
        # 解析解
        delta_soc = current_soc - target_soc
        remaining_time = delta_soc * V_avg * Q_eff / P_total
        
        return remaining_time


class TimeVaryingUsageModel:
    """
    时变使用模式模型
    
    支持模拟一天中不同时段的使用模式变化
    """
    
    def __init__(self, base_usage: UsageProfile = None):
        """初始化时变模型"""
        self.base_usage = base_usage or UsageProfile()
        self.usage_schedule: List[Tuple[float, UsageProfile]] = []
    
    def add_usage_period(self, start_hour: float, usage: UsageProfile):
        """
        添加使用时段
        
        参数:
            start_hour: 开始时间 (相对于模拟开始的小时数)
            usage: 该时段的使用配置
        """
        self.usage_schedule.append((start_hour, usage))
        self.usage_schedule.sort(key=lambda x: x[0])
    
    def get_usage_at_time(self, t: float) -> UsageProfile:
        """
        获取指定时间的使用配置
        
        参数:
            t: 时间 (hours)
            
        返回:
            对应的使用配置
        """
        if not self.usage_schedule:
            return self.base_usage
        
        current_usage = self.base_usage
        for start_time, usage in self.usage_schedule:
            if t >= start_time:
                current_usage = usage
            else:
                break
        
        return current_usage


class AdvancedBatteryModel(SmartphoneBatteryModel):
    """
    高级电池模型
    
    扩展基础模型，支持：
    1. 时变使用模式
    2. 动态温度变化
    3. RC等效电路模型
    4. 更精确的老化模型
    """
    
    def __init__(self, 
                 battery_params: BatteryParameters = None,
                 power_coeffs: PowerConsumptionCoefficients = None):
        super().__init__(battery_params, power_coeffs)
        
        # RC等效电路参数
        self.R1 = 0.02  # 第一RC网络电阻 (Ω)
        self.C1 = 5000  # 第一RC网络电容 (F)
        self.R2 = 0.01  # 第二RC网络电阻 (Ω)
        self.C2 = 50000  # 第二RC网络电容 (F)
    
    def extended_soc_dynamics(self, t: float, state: np.ndarray, 
                               usage_model: TimeVaryingUsageModel) -> np.ndarray:
        """
        扩展状态空间动力学
        
        状态向量: [SOC, V_RC1, V_RC2]
        
        dSOC/dt = -I / Q_eff
        dV_RC1/dt = I/C1 - V_RC1/(R1*C1)
        dV_RC2/dt = I/C2 - V_RC2/(R2*C2)
        
        参数:
            t: 时间
            state: 状态向量 [SOC, V_RC1, V_RC2]
            usage_model: 时变使用模型
            
        返回:
            状态导数向量
        """
        soc, V_RC1, V_RC2 = state
        soc = np.clip(soc, 0.01, 1.0)
        
        # 获取当前使用配置
        usage = usage_model.get_usage_at_time(t)
        
        # 计算功耗和电流
        P_total = self.total_power_consumption(usage)
        V_ocv = self.open_circuit_voltage(soc)
        
        # 端电压
        V_terminal = V_ocv - V_RC1 - V_RC2 - self.battery_params.internal_resistance * P_total / (V_ocv * 1000)
        
        # 电流 (A)
        I = P_total / (V_terminal * 1000) if V_terminal > 0 else P_total / (V_ocv * 1000)
        
        # 有效容量 (Ah)
        Q_eff = self.effective_capacity(usage.ambient_temperature) / 1000
        
        # 状态方程
        dSOC_dt = -I / Q_eff
        dV_RC1_dt = I / self.C1 - V_RC1 / (self.R1 * self.C1)
        dV_RC2_dt = I / self.C2 - V_RC2 / (self.R2 * self.C2)
        
        return np.array([dSOC_dt, dV_RC1_dt, dV_RC2_dt])
    
    def simulate_extended(self,
                          initial_soc: float,
                          usage_model: TimeVaryingUsageModel,
                          duration_hours: float,
                          time_points: int = 1000) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        扩展模拟，包含RC网络动态
        
        参数:
            initial_soc: 初始SOC
            usage_model: 时变使用模型
            duration_hours: 模拟时长
            time_points: 时间点数量
            
        返回:
            时间, SOC, V_RC1, V_RC2 数组
        """
        t_span = (0, duration_hours)
        t_eval = np.linspace(0, duration_hours, time_points)
        
        # 初始状态
        y0 = [initial_soc, 0.0, 0.0]
        
        def ode_func(t, y):
            return self.extended_soc_dynamics(t, y, usage_model)
        
        # SOC为零事件
        def soc_zero(t, y):
            return y[0] - 0.01
        soc_zero.terminal = True
        soc_zero.direction = -1
        
        sol = solve_ivp(ode_func, t_span, y0,
                        t_eval=t_eval, events=soc_zero,
                        method='RK45', dense_output=True)
        
        return sol.t, sol.y[0], sol.y[1], sol.y[2]


# 预定义使用场景
def create_idle_usage() -> UsageProfile:
    """待机模式"""
    return UsageProfile(
        screen_on=False,
        cpu_load=0.02,
        gpu_load=0.0,
        wifi_active=True,
        cellular_active=True,
        cellular_signal_strength=0.8,
        gps_active=False,
        bluetooth_active=False,
        background_apps_count=3
    )


def create_light_usage() -> UsageProfile:
    """轻度使用（浏览、社交媒体）"""
    return UsageProfile(
        screen_on=True,
        screen_brightness=0.4,
        cpu_load=0.15,
        gpu_load=0.05,
        wifi_active=True,
        cellular_active=False,
        data_transfer_rate=1.0,
        gps_active=False,
        bluetooth_active=False,
        background_apps_count=5
    )


def create_moderate_usage() -> UsageProfile:
    """中度使用（视频观看）"""
    return UsageProfile(
        screen_on=True,
        screen_brightness=0.6,
        cpu_load=0.3,
        gpu_load=0.2,
        wifi_active=True,
        cellular_active=False,
        data_transfer_rate=5.0,
        gps_active=False,
        bluetooth_active=True,
        background_apps_count=8
    )


def create_heavy_usage() -> UsageProfile:
    """重度使用（游戏）"""
    return UsageProfile(
        screen_on=True,
        screen_brightness=0.8,
        refresh_rate=120,
        cpu_load=0.8,
        gpu_load=0.7,
        wifi_active=True,
        cellular_active=True,
        data_transfer_rate=2.0,
        gps_active=False,
        bluetooth_active=True,
        background_apps_count=10
    )


def create_navigation_usage() -> UsageProfile:
    """导航模式"""
    return UsageProfile(
        screen_on=True,
        screen_brightness=0.9,
        cpu_load=0.4,
        gpu_load=0.3,
        wifi_active=False,
        cellular_active=True,
        cellular_signal_strength=0.6,
        data_transfer_rate=0.5,
        gps_active=True,
        gps_accuracy='high',
        bluetooth_active=True,
        background_apps_count=5
    )


if __name__ == "__main__":
    # 示例使用
    model = SmartphoneBatteryModel()
    
    print("=" * 60)
    print("智能手机电池连续时间模型 - 示例运行")
    print("=" * 60)
    
    # 测试不同使用场景
    scenarios = [
        ("待机模式", create_idle_usage()),
        ("轻度使用", create_light_usage()),
        ("中度使用", create_moderate_usage()),
        ("重度使用", create_heavy_usage()),
        ("导航模式", create_navigation_usage())
    ]
    
    for name, usage in scenarios:
        print(f"\n{name}:")
        print("-" * 40)
        
        # 功耗分解
        breakdown = model.power_breakdown(usage)
        print(f"  屏幕功耗: {breakdown['screen']:.1f} mW")
        print(f"  CPU功耗: {breakdown['cpu']:.1f} mW")
        print(f"  GPU功耗: {breakdown['gpu']:.1f} mW")
        print(f"  网络功耗: {breakdown['network']:.1f} mW")
        print(f"  GPS功耗: {breakdown['gps']:.1f} mW")
        print(f"  其他功耗: {breakdown['other']:.1f} mW")
        print(f"  总功耗: {breakdown['total']:.1f} mW")
        
        # 估计剩余时间
        remaining = model.analytical_remaining_time(1.0, usage, 0.05)
        print(f"  预计续航: {remaining:.1f} 小时")
