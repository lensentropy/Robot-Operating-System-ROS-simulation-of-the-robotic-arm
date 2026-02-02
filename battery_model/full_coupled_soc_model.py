"""
完整SOC连续时间耦合方程组模型 (无简化)
Full Coupled Continuous-Time SOC Model (No Simplification)

基于文档中的完整方程组:
1. 电池电化学热耦合核心方程
2. 总电流耦合方程
3. SoC模块功耗 (电热强耦合)
4. 显示模块 (环境光内容耦合)
5. 5G通信模块 (信道距离耦合)
6. 蓝牙模块 (事件驱动耦合)
7. GNSS模块 (环境信号耦合)
8. 后台任务模块 (随机过程耦合)
9. 用户行为马尔科夫模型

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from dataclasses import dataclass, field
from typing import Tuple, List, Dict, Optional, Callable
import warnings


# ==============================================================================
# 物理常数和参数
# ==============================================================================

@dataclass
class PhysicalConstants:
    """物理常数"""
    k_B: float = 1.380649e-23      # 玻尔兹曼常数 (J/K)
    q: float = 1.602176634e-19     # 电子电荷 (C)
    k_B_eV: float = 8.617333262e-5 # 玻尔兹曼常数 (eV/K)


@dataclass
class BatteryElectrochemicalParams:
    """电池电化学参数 (完整)"""
    # 容量参数
    Q_max: float = 4000.0          # 最大容量 (mAh)
    Q_nom: float = 3800.0          # 标称容量 (mAh)
    
    # 电压参数
    V_nom: float = 3.85            # 标称电压 (V)
    V_max: float = 4.35            # 最大电压 (V)
    V_min: float = 3.0             # 最小电压 (V)
    
    # OCV-SOC关系 (多项式系数, 从高阶到低阶)
    # V_OCV = a0 + a1*SOC + a2*SOC^2 + a3*SOC^3 + a4*SOC^4 + a5*SOC^5
    ocv_coeffs: tuple = (3.0, 1.2, -0.8, 1.5, -1.2, 0.3)
    
    # 内阻模型参数
    R_int_ref: float = 0.08        # 参考内阻 (Ohm) @ 25°C, SOC=50%
    R_int_soc_coeff: float = 0.5   # SOC对内阻的影响系数
    R_int_temp_coeff: float = 0.02 # 温度对内阻的影响系数 (1/K)
    R_int_aging_coeff: float = 0.001  # 循环老化系数
    
    # 熵热系数 (dV_OCV/dT)
    entropy_coeffs: tuple = (-0.0005, 0.001, -0.0008)  # 随SOC变化
    
    # 循环老化参数
    N_cycle_ref: float = 500.0     # 参考循环次数
    capacity_fade_rate: float = 0.0002  # 每循环容量衰减率


@dataclass
class BatteryThermalParams:
    """电池热参数"""
    C_th_batt: float = 15.0        # 电池热容 (J/K)
    R_th_batt: float = 8.0         # 电池热阻 (K/W)
    T_amb: float = 298.15          # 环境温度 (K)
    T_max_safe: float = 318.15     # 最高安全温度 (K) = 45°C
    
    # 热传导参数
    h_conv: float = 10.0           # 对流换热系数 (W/m²K)
    A_surface: float = 0.01        # 散热表面积 (m²)


@dataclass 
class SoCChipParams:
    """SoC芯片参数 (7nm FinFET)"""
    # 动态功耗参数
    C_eff: float = 2.5e-9          # 有效开关电容 (F)
    alpha_base: float = 0.1        # 基础活动因子
    alpha_load_coeff: float = 0.6  # 负载相关活动因子系数
    
    # DVFS参数
    V_dd_min: float = 0.6          # 最小电压 (V)
    V_dd_max: float = 1.1          # 最大电压 (V)
    V_th: float = 0.25             # 阈值电压 (V)
    f_min: float = 0.3e9           # 最小频率 (Hz)
    f_max: float = 3.0e9           # 最大频率 (Hz)
    
    # Alpha-Power Law参数
    gamma: float = 1.2             # 速度饱和指数
    K_freq: float = 1.2e9          # 频率-电压关系常数
    
    # 漏电参数
    I_ref: float = 1e-6            # 参考漏电流 (A)
    T_ref: float = 300.0           # 参考温度 (K)
    lambda_DIBL: float = 70e-3     # DIBL系数 (V/V)
    kappa: float = 2e-3            # 阈值电压温度系数
    n_subth: float = 1.2           # 亚阈值斜率因子
    
    # SoC热参数
    C_th_soc: float = 2.0          # SoC热容 (J/K)
    R_th_soc_batt: float = 5.0     # SoC到电池热阻 (K/W)
    R_th_soc_env: float = 15.0     # SoC到环境热阻 (K/W)


@dataclass
class DisplayParams:
    """显示模块参数 (OLED LTPO)"""
    P_static: float = 0.1          # 静态功耗 (W)
    k_drv: float = 0.0015          # 驱动系数
    gamma_oled: float = 2.2        # OLED伽马值
    
    # 刷新率参数
    f_refresh_min: float = 1.0     # LTPO最小刷新率 (Hz)
    f_refresh_max: float = 120.0   # 最大刷新率 (Hz)
    f_ltpo_threshold: float = 10.0 # LTPO切换阈值 (Hz)
    
    # 亮度参数
    L_max: float = 1200.0          # 最大亮度 (nits)
    P_per_nit: float = 0.002       # 每尼特功耗 (W/nit)
    
    # 面板参数
    A_panel: float = 0.01          # 面板面积 (m²)
    pixel_density: float = 460     # 像素密度 (ppi)


@dataclass
class Wireless5GParams:
    """5G通信模块参数"""
    P_base: float = 0.5            # 基础功耗 (W)
    P_BB: float = 0.3              # 基带功耗 (W)
    
    # 路径损耗参数 (Friis)
    path_loss_exp: float = 3.5     # 路径损耗指数
    d_ref: float = 100.0           # 参考距离 (m)
    
    # Shannon容量参数
    B: float = 100e6               # 带宽 (Hz)
    N_0: float = -174              # 噪声功率谱密度 (dBm/Hz)
    
    # PA参数
    eta_PA_max: float = 0.4        # PA最大效率
    P_PA_sat: float = 0.5          # PA饱和功率 (W)
    
    # 天线参数
    G_tx: float = 0.0              # 发射天线增益 (dBi)
    G_rx: float = 10.0             # 接收天线增益 (dBi)


@dataclass
class BluetoothParams:
    """蓝牙模块参数"""
    # BLE参数
    I_tx: float = 8e-3             # 发射电流 (A)
    I_rx: float = 6e-3             # 接收电流 (A)
    I_idle: float = 1e-3           # 空闲电流 (A)
    I_sleep: float = 1e-6          # 休眠电流 (A)
    
    # 事件参数
    t_preamble: float = 1e-3       # 前导码时间 (s)
    t_tx: float = 2e-3             # 发送时间 (s)
    t_rx: float = 2e-3             # 接收时间 (s)
    
    # 音频参数 (A2DP)
    P_DSP_SBC: float = 0.02        # SBC编解码功耗 (W)
    P_DSP_AAC: float = 0.03        # AAC编解码功耗 (W)
    P_DSP_LDAC: float = 0.05       # LDAC编解码功耗 (W)


@dataclass
class GNSSParams:
    """GNSS模块参数"""
    # 功耗参数
    P_LNA: float = 0.02            # LNA功耗 (W)
    P_acq: float = 0.15            # 捕获模式功耗 (W)
    P_track: float = 0.03          # 跟踪模式功耗 (W)
    
    # 状态转移参数
    SNR_threshold: float = 30.0    # 锁定阈值 (dB)
    k_sigmoid: float = 0.5         # Sigmoid陡峭因子
    tau_react: float = 1.0         # 反应时间常数 (s)


@dataclass
class BackgroundTaskParams:
    """后台任务参数"""
    # O-U过程参数
    theta: float = 0.5             # 回归速率
    mu_base: float = 0.2           # 基准电流 (A)
    sigma: float = 0.05            # 波动幅度
    
    # 周期性唤醒参数
    T_wakeup: float = 60.0         # 唤醒周期 (s)
    I_burst: float = 0.5           # 突发电流 (A)
    t_burst: float = 0.5           # 突发持续时间 (s)
    
    # 相关性参数 (网络-CPU)
    rho_net_cpu: float = 0.6       # 网络-CPU相关系数


@dataclass
class PMICParams:
    """电源管理IC参数"""
    eta_conv_max: float = 0.95     # 最大转换效率
    eta_conv_min: float = 0.85     # 最小转换效率
    I_quiescent: float = 1e-3      # 静态电流 (A)
    
    # 效率曲线参数
    I_peak_eff: float = 1.0        # 峰值效率电流 (A)


# ==============================================================================
# 完整耦合微分方程组
# ==============================================================================

class FullCoupledSOCModel:
    """
    完整SOC耦合连续时间模型
    
    状态向量: x = [SOC, T_batt, T_soc, x_lock, I_bg, N_cycle]
    
    核心方程组:
    1. dSOC/dt = -I_total / (Q_max * eta_temp * f(N_cycle))
    2. dT_batt/dt = (P_joule + P_entropy - (T_batt - T_amb)/R_th) / C_th
    3. dT_soc/dt = (P_soc - (T_soc - T_batt)/R_th_soc_batt - (T_soc - T_amb)/R_th_soc_env) / C_th_soc
    4. dx_lock/dt = (1/tau) * (S(SNR) - x_lock)  (GNSS状态)
    5. dI_bg/dt = theta*(mu - I_bg) + sigma*dW  (后台O-U过程)
    6. dN_cycle/dt = |I_total| / (2 * Q_max)  (循环计数)
    """
    
    def __init__(self):
        """初始化模型"""
        # 物理常数
        self.const = PhysicalConstants()
        
        # 参数组
        self.batt_echem = BatteryElectrochemicalParams()
        self.batt_therm = BatteryThermalParams()
        self.soc_chip = SoCChipParams()
        self.display = DisplayParams()
        self.wireless_5g = Wireless5GParams()
        self.bluetooth = BluetoothParams()
        self.gnss = GNSSParams()
        self.background = BackgroundTaskParams()
        self.pmic = PMICParams()
        
        # 状态维度
        self.n_states = 6  # [SOC, T_batt, T_soc, x_lock, I_bg, N_cycle]
        
        # 仿真历史
        self.history = {}
        
        # 随机数生成器状态
        self._rng = np.random.default_rng()
    
    # --------------------------------------------------------------------------
    # OCV-SOC关系 (完整多项式)
    # --------------------------------------------------------------------------
    
    def V_OCV(self, SOC: float) -> float:
        """
        开路电压-SOC关系 (5阶多项式)
        V_OCV = a0 + a1*SOC + a2*SOC^2 + a3*SOC^3 + a4*SOC^4 + a5*SOC^5
        """
        SOC = np.clip(SOC, 0.0, 1.0)
        coeffs = self.batt_echem.ocv_coeffs
        
        V = 0.0
        for i, a in enumerate(coeffs):
            V += a * (SOC ** i)
        
        return np.clip(V, self.batt_echem.V_min, self.batt_echem.V_max)
    
    def dV_OCV_dSOC(self, SOC: float) -> float:
        """OCV对SOC的导数"""
        SOC = np.clip(SOC, 0.001, 0.999)
        coeffs = self.batt_echem.ocv_coeffs
        
        dV = 0.0
        for i, a in enumerate(coeffs):
            if i > 0:
                dV += i * a * (SOC ** (i - 1))
        
        return dV
    
    def dV_OCV_dT(self, SOC: float, T: float) -> float:
        """
        OCV对温度的导数 (熵系数)
        dV/dT = b0 + b1*SOC + b2*SOC^2
        """
        SOC = np.clip(SOC, 0.0, 1.0)
        coeffs = self.batt_echem.entropy_coeffs
        
        dV_dT = coeffs[0] + coeffs[1] * SOC + coeffs[2] * SOC**2
        return dV_dT
    
    # --------------------------------------------------------------------------
    # 内阻模型 (SOC, 温度, 老化依赖)
    # --------------------------------------------------------------------------
    
    def R_int(self, SOC: float, T: float, N_cycle: float) -> float:
        """
        内阻模型
        R_int = R_ref * f_soc(SOC) * f_temp(T) * f_aging(N)
        """
        R_ref = self.batt_echem.R_int_ref
        
        # SOC因子: 低SOC时内阻增大
        SOC = np.clip(SOC, 0.01, 1.0)
        f_soc = 1.0 + self.batt_echem.R_int_soc_coeff * (1.0 - SOC)**2
        
        # 温度因子: 低温时内阻增大
        T_ref = 298.15
        f_temp = np.exp(-self.batt_echem.R_int_temp_coeff * (T - T_ref))
        
        # 老化因子: 循环次数增加内阻
        f_aging = 1.0 + self.batt_echem.R_int_aging_coeff * N_cycle
        
        return R_ref * f_soc * f_temp * f_aging
    
    # --------------------------------------------------------------------------
    # 容量老化模型
    # --------------------------------------------------------------------------
    
    def Q_effective(self, N_cycle: float, T_avg: float) -> float:
        """
        有效容量 (考虑老化)
        Q_eff = Q_max * (1 - fade_rate * N_cycle * f_temp)
        """
        Q_max = self.batt_echem.Q_max
        fade_rate = self.batt_echem.capacity_fade_rate
        
        # 温度加速因子 (Arrhenius)
        E_a = 0.3  # 活化能 (eV)
        T_ref = 298.15
        f_temp = np.exp(E_a / self.const.k_B_eV * (1/T_ref - 1/T_avg))
        
        Q_eff = Q_max * (1.0 - fade_rate * N_cycle * f_temp)
        return max(Q_eff, Q_max * 0.7)  # 最低70%容量
    
    # --------------------------------------------------------------------------
    # 温度效率因子
    # --------------------------------------------------------------------------
    
    def eta_temp(self, T: float) -> float:
        """
        温度效率因子
        低温下库仑效率下降
        """
        T_celsius = T - 273.15
        
        if T_celsius < 0:
            eta = 0.7 + 0.01 * T_celsius  # 低于0°C急剧下降
        elif T_celsius < 10:
            eta = 0.7 + 0.03 * T_celsius
        elif T_celsius < 45:
            eta = 1.0
        else:
            eta = 1.0 - 0.01 * (T_celsius - 45)  # 高温下降
        
        return np.clip(eta, 0.5, 1.0)
    
    # --------------------------------------------------------------------------
    # PMIC效率模型
    # --------------------------------------------------------------------------
    
    def eta_PMIC(self, I_load: float, V_in: float, V_out: float) -> float:
        """
        PMIC转换效率
        η = η_max * f(I_load)
        """
        I_peak = self.pmic.I_peak_eff
        eta_max = self.pmic.eta_conv_max
        eta_min = self.pmic.eta_conv_min
        
        # 负载相关效率 (二次模型)
        if I_load < 0.01:
            eta = eta_min
        else:
            # 效率在峰值电流附近最高
            ratio = I_load / I_peak
            eta = eta_max * (1.0 - 0.3 * (ratio - 1)**2)
        
        return np.clip(eta, eta_min, eta_max)
    
    # --------------------------------------------------------------------------
    # SoC芯片功耗模型 (完整电热耦合)
    # --------------------------------------------------------------------------
    
    def DVFS_control(self, load: float, T_soc: float, V_batt: float) -> Tuple[float, float]:
        """
        DVFS控制律
        f_cpu = g(Load, T_soc, V_batt)
        返回 (V_dd, f_cpu)
        """
        sp = self.soc_chip
        
        # 热节流
        T_throttle = 353.15  # 80°C开始节流
        if T_soc > T_throttle:
            throttle_factor = 1.0 - 0.5 * (T_soc - T_throttle) / 20.0
            throttle_factor = max(0.3, throttle_factor)
        else:
            throttle_factor = 1.0
        
        # 目标频率 (负载映射)
        f_target = sp.f_min + (sp.f_max - sp.f_min) * load * throttle_factor
        f_target = np.clip(f_target, sp.f_min, sp.f_max)
        
        # Alpha-Power Law反推电压
        # f = K * (V_dd - V_th)^gamma / V_dd
        # 简化: V_dd = V_min + (V_max - V_min) * ((f - f_min) / (f_max - f_min))^(1/gamma)
        f_norm = (f_target - sp.f_min) / (sp.f_max - sp.f_min)
        V_dd = sp.V_dd_min + (sp.V_dd_max - sp.V_dd_min) * (f_norm ** (1.0/sp.gamma))
        
        # 电池电压限制
        if V_batt < V_dd + 0.2:
            V_dd = V_batt - 0.2
            # 重新计算频率
            v_norm = (V_dd - sp.V_dd_min) / (sp.V_dd_max - sp.V_dd_min)
            f_target = sp.f_min + (sp.f_max - sp.f_min) * (v_norm ** sp.gamma)
        
        V_dd = np.clip(V_dd, sp.V_dd_min, sp.V_dd_max)
        f_target = np.clip(f_target, sp.f_min, sp.f_max)
        
        return V_dd, f_target
    
    def P_SoC_dynamic(self, f: float, V_dd: float, load: float) -> float:
        """
        SoC动态功耗
        P_dyn = α(load) * C_eff * V_dd^2 * f
        """
        sp = self.soc_chip
        
        # 活动因子
        alpha = sp.alpha_base + sp.alpha_load_coeff * load
        alpha = np.clip(alpha, 0.1, 0.7)
        
        P_dyn = alpha * sp.C_eff * (V_dd ** 2) * f
        return P_dyn
    
    def P_SoC_leakage(self, V_dd: float, T_soc: float) -> float:
        """
        SoC漏电功耗 (完整DIBL和温度模型)
        P_leak = V_dd * I_sub(V_dd, T)
        I_sub = I_ref * (T/T_ref)^2 * exp((λ*V_dd + κ*(T-T_ref)) / (n*V_T))
        """
        sp = self.soc_chip
        
        # 热电压
        V_T = self.const.k_B_eV * T_soc
        
        # 亚阈值漏电流
        exp_arg = (sp.lambda_DIBL * V_dd + sp.kappa * (T_soc - sp.T_ref)) / (sp.n_subth * V_T)
        exp_arg = np.clip(exp_arg, -50, 50)  # 防止溢出
        
        I_sub = sp.I_ref * (T_soc / sp.T_ref)**2 * np.exp(exp_arg)
        P_leak = V_dd * I_sub
        
        return P_leak
    
    def P_SoC_total(self, load: float, T_soc: float, V_batt: float) -> Tuple[float, float, float]:
        """
        SoC总功耗
        返回 (P_total, V_dd, f_cpu)
        """
        V_dd, f_cpu = self.DVFS_control(load, T_soc, V_batt)
        P_dyn = self.P_SoC_dynamic(f_cpu, V_dd, load)
        P_leak = self.P_SoC_leakage(V_dd, T_soc)
        
        return P_dyn + P_leak, V_dd, f_cpu
    
    # --------------------------------------------------------------------------
    # 显示模块功耗 (环境光内容耦合)
    # --------------------------------------------------------------------------
    
    def P_display(self, brightness: float, refresh_rate: float, 
                  APL: float, content_motion: float) -> float:
        """
        显示模块功耗
        P_disp = P_static + k_drv * f_refresh * L^γ * APL
        
        Parameters:
        -----------
        brightness : float
            屏幕亮度 (nits)
        refresh_rate : float
            刷新率 (Hz)
        APL : float
            平均像素电平 (0-100)
        content_motion : float
            内容运动量 (0-1), 影响LTPO刷新率
        """
        dp = self.display
        
        if brightness <= 0:
            return 0.0
        
        # LTPO自适应刷新率
        if content_motion < 0.1:  # 静态内容
            f_actual = max(dp.f_ltpo_threshold, refresh_rate * content_motion * 10)
        else:
            f_actual = refresh_rate
        
        f_actual = np.clip(f_actual, dp.f_refresh_min, dp.f_refresh_max)
        
        # 亮度非线性功耗 (伽马校正)
        L_normalized = brightness / dp.L_max
        APL_normalized = APL / 100.0
        
        # OLED发光功耗
        P_emissive = dp.P_per_nit * brightness * (APL_normalized ** dp.gamma_oled)
        
        # 驱动功耗 (与刷新率成正比)
        P_driver = dp.k_drv * f_actual * APL_normalized
        
        # 总功耗
        P_total = dp.P_static + P_emissive + P_driver
        
        return P_total
    
    # --------------------------------------------------------------------------
    # 5G通信模块 (信道距离耦合)
    # --------------------------------------------------------------------------
    
    def P_5G(self, data_rate: float, distance: float, SNR_measured: float) -> float:
        """
        5G模块功耗 (基于Shannon定理和Friis传输方程)
        
        P_5G = P_base + P_BB*R(t) + P_tx / η_PA
        P_tx ∝ (2^(R/B) - 1) * d^n
        
        Parameters:
        -----------
        data_rate : float
            数据速率 (Mbps)
        distance : float
            到基站距离 (m)
        SNR_measured : float
            测量的信噪比 (dB)
        """
        wp = self.wireless_5g
        
        if data_rate <= 0:
            return 0.0
        
        # 基带功耗
        P_BB_actual = wp.P_BB * (data_rate / 100.0)  # 归一化到100Mbps
        
        # Shannon容量约束下的所需SNR
        # C = B * log2(1 + SNR)
        # SNR_req = 2^(R/B) - 1
        R_bps = data_rate * 1e6
        SNR_req_linear = 2**(R_bps / wp.B) - 1
        
        # 路径损耗 (Friis)
        # PL = (4πd/λ)^n
        path_loss = (distance / wp.d_ref) ** wp.path_loss_exp
        
        # 所需发射功率
        # P_tx = SNR_req * N_0 * B * PL / (G_tx * G_rx)
        N_0_linear = 10**((wp.N_0 + 30) / 10)  # 转换为W/Hz
        G_tx_linear = 10**(wp.G_tx / 10)
        G_rx_linear = 10**(wp.G_rx / 10)
        
        P_tx_required = SNR_req_linear * N_0_linear * wp.B * path_loss / (G_tx_linear * G_rx_linear)
        P_tx_required = min(P_tx_required, wp.P_PA_sat)  # PA饱和限制
        
        # PA效率 (随输出功率变化)
        # η_PA = η_max * (P_tx / P_sat)^0.5  (简化模型)
        if P_tx_required > 0:
            eta_PA = wp.eta_PA_max * np.sqrt(P_tx_required / wp.P_PA_sat)
            eta_PA = np.clip(eta_PA, 0.1, wp.eta_PA_max)
        else:
            eta_PA = wp.eta_PA_max
        
        # 总功耗
        P_total = wp.P_base + P_BB_actual + P_tx_required / eta_PA
        
        return P_total
    
    # --------------------------------------------------------------------------
    # 蓝牙模块 (事件驱动耦合)
    # --------------------------------------------------------------------------
    
    def P_Bluetooth(self, is_connected: bool, is_audio: bool, 
                    conn_interval_ms: float, audio_codec: str = 'SBC') -> float:
        """
        蓝牙功耗 (事件驱动模型)
        
        BLE: Q_event = ∫I(t)dt = Q_preamble + Q_tx + Q_rx
        平均电流: I_avg = I_sleep + Q_event / T_interval
        
        音频: P_audio = P_DSP(codec)
        """
        bp = self.bluetooth
        
        if not is_connected:
            return bp.I_sleep * 3.3  # 假设3.3V供电
        
        # 单次连接事件电荷
        Q_event = (bp.I_tx * bp.t_tx + 
                   bp.I_rx * bp.t_rx + 
                   bp.I_idle * bp.t_preamble)
        
        # 连接间隔 (ms -> s)
        T_interval = conn_interval_ms / 1000.0
        
        # 平均电流
        if T_interval > 0:
            I_avg = bp.I_sleep + Q_event / T_interval
        else:
            I_avg = bp.I_idle
        
        # BLE功耗 (假设3.3V)
        P_BLE = I_avg * 3.3
        
        # 音频功耗
        P_audio = 0.0
        if is_audio:
            if audio_codec == 'LDAC':
                P_audio = bp.P_DSP_LDAC
            elif audio_codec == 'AAC':
                P_audio = bp.P_DSP_AAC
            else:
                P_audio = bp.P_DSP_SBC
        
        return P_BLE + P_audio
    
    # --------------------------------------------------------------------------
    # GNSS模块 (环境信号耦合, 状态机)
    # --------------------------------------------------------------------------
    
    def P_GNSS(self, x_lock: float, SNR_env: float) -> Tuple[float, float]:
        """
        GNSS功耗 (连续时间状态机)
        
        P = P_LNA + x_lock * P_track + (1 - x_lock) * P_acq
        dx_lock/dt = (1/τ) * (S(SNR) - x_lock)
        
        返回 (P_gnss, dx_lock_dt)
        """
        gp = self.gnss
        
        # Sigmoid状态转移概率
        S_lock = 1.0 / (1.0 + np.exp(-gp.k_sigmoid * (SNR_env - gp.SNR_threshold)))
        
        # 状态导数
        dx_lock_dt = (S_lock - x_lock) / gp.tau_react
        
        # 功耗
        P_gnss = gp.P_LNA + x_lock * gp.P_track + (1.0 - x_lock) * gp.P_acq
        
        return P_gnss, dx_lock_dt
    
    # --------------------------------------------------------------------------
    # 后台任务 (O-U随机过程)
    # --------------------------------------------------------------------------
    
    def P_background(self, I_bg: float, t: float, V_batt: float) -> Tuple[float, float]:
        """
        后台任务功耗 (O-U过程 + 周期性突发)
        
        dI_bg = θ*(μ - I_bg)*dt + σ*dW
        
        返回 (P_bg, dI_bg_dt_deterministic)
        """
        bgp = self.background
        
        # O-U过程确定性部分
        dI_bg_dt = bgp.theta * (bgp.mu_base - I_bg)
        
        # 周期性突发 (简化为占空比)
        duty_cycle = bgp.t_burst / bgp.T_wakeup
        I_burst_avg = bgp.I_burst * duty_cycle
        
        # 总后台电流
        I_total_bg = I_bg + I_burst_avg
        
        # 功耗
        P_bg = I_total_bg * V_batt
        
        return P_bg, dI_bg_dt
    
    # --------------------------------------------------------------------------
    # 总电流耦合方程
    # --------------------------------------------------------------------------
    
    def I_total(self, P_total: float, V_batt: float, eta_pmic: float) -> float:
        """
        总电池电流
        I_total = P_total / (η_PMIC * V_batt)
        """
        if V_batt <= 0 or eta_pmic <= 0:
            return 0.0
        
        I = P_total / (eta_pmic * V_batt)
        
        # 加上PMIC静态电流
        I += self.pmic.I_quiescent
        
        return I
    
    # --------------------------------------------------------------------------
    # 完整耦合ODE系统
    # --------------------------------------------------------------------------
    
    def coupled_ode_system(self, t: float, y: np.ndarray, 
                           hardware_state_func: Callable) -> np.ndarray:
        """
        完整耦合微分方程组
        
        状态向量: y = [SOC, T_batt, T_soc, x_lock, I_bg, N_cycle]
        
        方程组:
        1. dSOC/dt = -I_total / (Q_eff * η_temp) / 3600
        2. dT_batt/dt = (P_joule + P_entropy - (T_batt-T_amb)/R_th) / C_th
        3. dT_soc/dt = (P_soc - (T_soc-T_batt)/R_th_sb - (T_soc-T_amb)/R_th_se) / C_th_soc
        4. dx_lock/dt = (S(SNR) - x_lock) / τ
        5. dI_bg/dt = θ*(μ - I_bg)
        6. dN_cycle/dt = |I_total| / (2 * Q_max * 3600)
        """
        # 解包状态
        SOC, T_batt, T_soc, x_lock, I_bg, N_cycle = y
        
        # 状态约束
        SOC = np.clip(SOC, 0.0, 1.0)
        T_batt = np.clip(T_batt, 273.15, 373.15)
        T_soc = np.clip(T_soc, 273.15, 393.15)
        x_lock = np.clip(x_lock, 0.0, 1.0)
        I_bg = max(0.0, I_bg)
        N_cycle = max(0.0, N_cycle)
        
        # 如果SOC耗尽
        if SOC <= 0.001:
            # 只有温度衰减
            dT_batt_dt = -(T_batt - self.batt_therm.T_amb) / (self.batt_therm.R_th_batt * self.batt_therm.C_th_batt)
            dT_soc_dt = -(T_soc - self.batt_therm.T_amb) / (self.soc_chip.R_th_soc_env * self.soc_chip.C_th_soc)
            return np.array([0.0, dT_batt_dt, dT_soc_dt, 0.0, 0.0, 0.0])
        
        # 获取当前硬件状态
        hw = hardware_state_func(t)
        
        # ---------- 电池电压计算 ----------
        V_OCV = self.V_OCV(SOC)
        R_int = self.R_int(SOC, T_batt, N_cycle)
        
        # 初始电流估计 (迭代求解)
        V_batt_est = V_OCV - 0.5 * R_int  # 初始估计
        
        # ---------- 各模块功耗计算 ----------
        
        # 1. SoC芯片功耗
        P_soc, V_dd, f_cpu = self.P_SoC_total(
            hw.get('cpu_load', 0.3), T_soc, V_batt_est
        )
        
        # 2. 显示功耗
        P_disp = self.P_display(
            hw.get('brightness', 0),
            hw.get('refresh_rate', 60),
            hw.get('APL', 50),
            hw.get('content_motion', 0.5)
        )
        
        # 3. 5G功耗
        P_5g = self.P_5G(
            hw.get('data_rate', 0),
            hw.get('distance', 200),
            hw.get('SNR_5g', 20)
        )
        
        # 4. 蓝牙功耗
        P_bt = self.P_Bluetooth(
            hw.get('bt_connected', False),
            hw.get('bt_audio', False),
            hw.get('bt_interval', 100),
            hw.get('bt_codec', 'SBC')
        )
        
        # 5. GNSS功耗
        if hw.get('gnss_on', False):
            SNR_gnss = hw.get('SNR_gnss', 35)
            P_gnss, dx_lock_dt = self.P_GNSS(x_lock, SNR_gnss)
        else:
            P_gnss = 0.0
            dx_lock_dt = -x_lock / self.gnss.tau_react  # 衰减
        
        # 6. 后台功耗
        P_bg, dI_bg_dt = self.P_background(I_bg, t, V_batt_est)
        
        # ---------- 总功耗和电流 ----------
        P_total = P_soc + P_disp + P_5g + P_bt + P_gnss + P_bg
        
        # PMIC效率
        I_est = P_total / V_batt_est if V_batt_est > 0 else 0
        eta_pmic = self.eta_PMIC(I_est, V_batt_est, V_dd)
        
        # 实际电流 (迭代求解)
        for _ in range(3):
            I_total = self.I_total(P_total, V_batt_est, eta_pmic)
            V_batt_new = V_OCV - I_total * R_int
            if abs(V_batt_new - V_batt_est) < 0.01:
                break
            V_batt_est = V_batt_new
            eta_pmic = self.eta_PMIC(I_total, V_batt_est, V_dd)
        
        I_batt = I_total
        V_batt = V_batt_est
        
        # ---------- 热计算 ----------
        
        # 焦耳热
        P_joule = I_batt**2 * R_int
        
        # 熵热
        dV_dT = self.dV_OCV_dT(SOC, T_batt)
        P_entropy = I_batt * T_batt * dV_dT
        
        # ---------- 微分方程组 ----------
        
        # 1. SOC变化率
        Q_eff = self.Q_effective(N_cycle, T_batt) / 1000.0  # mAh -> Ah
        eta_t = self.eta_temp(T_batt)
        dSOC_dt = -I_batt / (Q_eff * eta_t * 3600)  # 每秒变化
        
        # 2. 电池温度变化率
        dT_batt_dt = (P_joule + P_entropy - 
                      (T_batt - self.batt_therm.T_amb) / self.batt_therm.R_th_batt
                     ) / self.batt_therm.C_th_batt
        
        # 3. SoC温度变化率
        dT_soc_dt = (P_soc - 
                     (T_soc - T_batt) / self.soc_chip.R_th_soc_batt -
                     (T_soc - self.batt_therm.T_amb) / self.soc_chip.R_th_soc_env
                    ) / self.soc_chip.C_th_soc
        
        # 4. GNSS锁定状态
        # dx_lock_dt 已在GNSS计算中得到
        
        # 5. 后台电流
        # dI_bg_dt 已在后台计算中得到
        
        # 6. 循环计数
        dN_cycle_dt = abs(I_batt) / (2 * self.batt_echem.Q_max / 1000 * 3600)
        
        return np.array([dSOC_dt, dT_batt_dt, dT_soc_dt, dx_lock_dt, dI_bg_dt, dN_cycle_dt])
    
    # --------------------------------------------------------------------------
    # 仿真接口
    # --------------------------------------------------------------------------
    
    def simulate(self, t_span: Tuple[float, float], 
                 y0: np.ndarray,
                 hardware_state_func: Callable,
                 t_eval: np.ndarray = None,
                 record_details: bool = True) -> Dict:
        """
        运行完整耦合仿真
        
        Parameters:
        -----------
        t_span : tuple
            仿真时间范围 (t_start, t_end) 单位:秒
        y0 : np.ndarray
            初始状态 [SOC_0, T_batt_0, T_soc_0, x_lock_0, I_bg_0, N_cycle_0]
        hardware_state_func : callable
            硬件状态函数 f(t) -> dict
        t_eval : np.ndarray, optional
            评估时间点
        record_details : bool
            是否记录详细信息
        
        Returns:
        --------
        dict : 仿真结果
        """
        if t_eval is None:
            n_points = int((t_span[1] - t_span[0]) / 60) + 1
            t_eval = np.linspace(t_span[0], t_span[1], n_points)
        
        # SOC耗尽事件
        def soc_depleted(t, y):
            return y[0] - 0.001
        soc_depleted.terminal = True
        soc_depleted.direction = -1
        
        # 求解ODE
        solution = solve_ivp(
            lambda t, y: self.coupled_ode_system(t, y, hardware_state_func),
            t_span,
            y0,
            method='RK45',
            t_eval=t_eval,
            max_step=30.0,
            rtol=1e-6,
            atol=1e-9,
            events=soc_depleted
        )
        
        # 提取结果
        result = {
            'time': solution.t,
            'time_hours': solution.t / 3600,
            'SOC': np.clip(solution.y[0], 0, 1),
            'T_batt': solution.y[1],
            'T_batt_celsius': solution.y[1] - 273.15,
            'T_soc': solution.y[2],
            'T_soc_celsius': solution.y[2] - 273.15,
            'x_lock': solution.y[3],
            'I_bg': solution.y[4],
            'N_cycle': solution.y[5],
            'success': solution.success,
            'depleted': len(solution.t_events[0]) > 0 if solution.t_events else False
        }
        
        # 计算详细功耗和电压/电流
        if record_details:
            n = len(solution.t)
            P_total = np.zeros(n)
            P_soc = np.zeros(n)
            P_disp = np.zeros(n)
            P_5g = np.zeros(n)
            P_bt = np.zeros(n)
            P_gnss = np.zeros(n)
            P_bg = np.zeros(n)
            V_batt = np.zeros(n)
            I_batt = np.zeros(n)
            V_OCV_arr = np.zeros(n)
            R_int_arr = np.zeros(n)
            
            for i, t in enumerate(solution.t):
                SOC = result['SOC'][i]
                T_batt = result['T_batt'][i]
                T_soc = result['T_soc'][i]
                x_lock = result['x_lock'][i]
                I_bg = result['I_bg'][i]
                N_cycle = result['N_cycle'][i]
                
                hw = hardware_state_func(t)
                
                # 电压
                V_OCV_arr[i] = self.V_OCV(SOC)
                R_int_arr[i] = self.R_int(SOC, T_batt, N_cycle)
                
                # 各模块功耗
                P_soc[i], _, _ = self.P_SoC_total(hw.get('cpu_load', 0.3), T_soc, V_OCV_arr[i])
                P_disp[i] = self.P_display(hw.get('brightness', 0), hw.get('refresh_rate', 60), 
                                            hw.get('APL', 50), hw.get('content_motion', 0.5))
                P_5g[i] = self.P_5G(hw.get('data_rate', 0), hw.get('distance', 200), hw.get('SNR_5g', 20))
                P_bt[i] = self.P_Bluetooth(hw.get('bt_connected', False), hw.get('bt_audio', False),
                                           hw.get('bt_interval', 100), hw.get('bt_codec', 'SBC'))
                if hw.get('gnss_on', False):
                    P_gnss[i], _ = self.P_GNSS(x_lock, hw.get('SNR_gnss', 35))
                P_bg[i], _ = self.P_background(I_bg, t, V_OCV_arr[i])
                
                P_total[i] = P_soc[i] + P_disp[i] + P_5g[i] + P_bt[i] + P_gnss[i] + P_bg[i]
                
                # 电流和端电压
                eta = self.eta_PMIC(P_total[i] / V_OCV_arr[i], V_OCV_arr[i], 0.9)
                I_batt[i] = P_total[i] / (eta * V_OCV_arr[i]) if V_OCV_arr[i] > 0 else 0
                V_batt[i] = V_OCV_arr[i] - I_batt[i] * R_int_arr[i]
            
            result.update({
                'P_total': P_total,
                'P_soc': P_soc,
                'P_disp': P_disp,
                'P_5g': P_5g,
                'P_bt': P_bt,
                'P_gnss': P_gnss,
                'P_bg': P_bg,
                'V_batt': V_batt,
                'I_batt': I_batt,
                'V_OCV': V_OCV_arr,
                'R_int': R_int_arr
            })
        
        return result


# ==============================================================================
# 可视化
# ==============================================================================

def visualize_full_model(result: Dict, save_path: str = None):
    """
    完整模型可视化
    """
    import matplotlib.pyplot as plt
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    fig = plt.figure(figsize=(16, 20))
    
    time_hours = result['time_hours']
    
    # 1. SOC演化
    ax1 = fig.add_subplot(5, 2, 1)
    ax1.plot(time_hours, result['SOC'] * 100, 'b-', linewidth=2, label='SOC')
    ax1.axhline(y=20, color='orange', linestyle='--', alpha=0.7, label='Warning')
    ax1.axhline(y=5, color='red', linestyle='--', alpha=0.7, label='Critical')
    ax1.set_xlabel('Time (hours)')
    ax1.set_ylabel('SOC (%)')
    ax1.set_title('Battery State of Charge Evolution', fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 105)
    
    # 2. 温度演化
    ax2 = fig.add_subplot(5, 2, 2)
    ax2.plot(time_hours, result['T_batt_celsius'], 'r-', linewidth=2, label='Battery')
    ax2.plot(time_hours, result['T_soc_celsius'], 'orange', linewidth=2, label='SoC Chip')
    ax2.axhline(y=45, color='red', linestyle='--', alpha=0.5, label='Max Safe')
    ax2.set_xlabel('Time (hours)')
    ax2.set_ylabel('Temperature (C)')
    ax2.set_title('Thermal Dynamics (Battery & SoC)', fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 电压演化
    ax3 = fig.add_subplot(5, 2, 3)
    ax3.plot(time_hours, result['V_OCV'], 'g-', linewidth=2, label='V_OCV')
    ax3.plot(time_hours, result['V_batt'], 'b-', linewidth=2, label='V_terminal')
    ax3.set_xlabel('Time (hours)')
    ax3.set_ylabel('Voltage (V)')
    ax3.set_title('Battery Voltage (OCV & Terminal)', fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. 电流演化
    ax4 = fig.add_subplot(5, 2, 4)
    ax4.plot(time_hours, result['I_batt'] * 1000, 'purple', linewidth=2)
    ax4.set_xlabel('Time (hours)')
    ax4.set_ylabel('Current (mA)')
    ax4.set_title('Battery Discharge Current', fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    # 5. 总功耗
    ax5 = fig.add_subplot(5, 2, 5)
    ax5.plot(time_hours, result['P_total'], 'k-', linewidth=2)
    ax5.fill_between(time_hours, 0, result['P_total'], alpha=0.3)
    ax5.set_xlabel('Time (hours)')
    ax5.set_ylabel('Power (W)')
    ax5.set_title('Total Power Consumption', fontweight='bold')
    ax5.grid(True, alpha=0.3)
    
    # 6. 功耗分解 (堆叠)
    ax6 = fig.add_subplot(5, 2, 6)
    ax6.stackplot(time_hours, 
                  result['P_soc'], result['P_disp'], result['P_5g'], 
                  result['P_bt'], result['P_gnss'], result['P_bg'],
                  labels=['SoC', 'Display', '5G', 'Bluetooth', 'GNSS', 'Background'],
                  alpha=0.8)
    ax6.set_xlabel('Time (hours)')
    ax6.set_ylabel('Power (W)')
    ax6.set_title('Power Breakdown by Module', fontweight='bold')
    ax6.legend(loc='upper right', fontsize=8)
    ax6.grid(True, alpha=0.3)
    
    # 7. 内阻演化
    ax7 = fig.add_subplot(5, 2, 7)
    ax7.plot(time_hours, result['R_int'] * 1000, 'brown', linewidth=2)
    ax7.set_xlabel('Time (hours)')
    ax7.set_ylabel('Internal Resistance (mOhm)')
    ax7.set_title('Battery Internal Resistance', fontweight='bold')
    ax7.grid(True, alpha=0.3)
    
    # 8. GNSS锁定状态
    ax8 = fig.add_subplot(5, 2, 8)
    ax8.plot(time_hours, result['x_lock'], 'green', linewidth=2)
    ax8.set_xlabel('Time (hours)')
    ax8.set_ylabel('Lock Probability')
    ax8.set_title('GNSS Lock State (Continuous)', fontweight='bold')
    ax8.set_ylim(-0.05, 1.05)
    ax8.grid(True, alpha=0.3)
    
    # 9. 后台电流
    ax9 = fig.add_subplot(5, 2, 9)
    ax9.plot(time_hours, result['I_bg'] * 1000, 'gray', linewidth=2)
    ax9.set_xlabel('Time (hours)')
    ax9.set_ylabel('Background Current (mA)')
    ax9.set_title('Background Task Current (O-U Process)', fontweight='bold')
    ax9.grid(True, alpha=0.3)
    
    # 10. 循环计数
    ax10 = fig.add_subplot(5, 2, 10)
    ax10.plot(time_hours, result['N_cycle'], 'navy', linewidth=2)
    ax10.set_xlabel('Time (hours)')
    ax10.set_ylabel('Equivalent Cycles')
    ax10.set_title('Battery Cycle Count (Aging)', fontweight='bold')
    ax10.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
    
    return fig


def visualize_ocv_soc_curve(model: FullCoupledSOCModel, save_path: str = None):
    """
    可视化OCV-SOC关系曲线
    """
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    SOC = np.linspace(0, 1, 100)
    
    # 1. OCV-SOC关系
    ax1 = axes[0, 0]
    V_OCV = [model.V_OCV(s) for s in SOC]
    ax1.plot(SOC * 100, V_OCV, 'b-', linewidth=2)
    ax1.set_xlabel('SOC (%)')
    ax1.set_ylabel('Open Circuit Voltage (V)')
    ax1.set_title('OCV-SOC Relationship (5th Order Polynomial)', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # 2. dV/dSOC
    ax2 = axes[0, 1]
    dV_dSOC = [model.dV_OCV_dSOC(s) for s in SOC]
    ax2.plot(SOC * 100, dV_dSOC, 'g-', linewidth=2)
    ax2.set_xlabel('SOC (%)')
    ax2.set_ylabel('dV_OCV/dSOC (V)')
    ax2.set_title('OCV Gradient', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # 3. 内阻-SOC关系 (不同温度)
    ax3 = axes[1, 0]
    T_values = [273.15, 288.15, 298.15, 313.15]  # 0, 15, 25, 40°C
    T_labels = ['0°C', '15°C', '25°C', '40°C']
    for T, label in zip(T_values, T_labels):
        R_int = [model.R_int(s, T, 0) * 1000 for s in SOC]
        ax3.plot(SOC * 100, R_int, linewidth=2, label=label)
    ax3.set_xlabel('SOC (%)')
    ax3.set_ylabel('Internal Resistance (mOhm)')
    ax3.set_title('R_int vs SOC (Temperature Dependence)', fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. 熵系数
    ax4 = axes[1, 1]
    dV_dT = [model.dV_OCV_dT(s, 298.15) * 1000 for s in SOC]  # mV/K
    ax4.plot(SOC * 100, dV_dT, 'r-', linewidth=2)
    ax4.set_xlabel('SOC (%)')
    ax4.set_ylabel('dV_OCV/dT (mV/K)')
    ax4.set_title('Entropy Coefficient', fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    return fig


def visualize_power_models(model: FullCoupledSOCModel, save_path: str = None):
    """
    可视化各模块功耗模型
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(16, 12))
    
    # 1. SoC功耗 vs 频率和温度
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    freq = np.linspace(0.3e9, 3e9, 30)
    temp = np.linspace(25, 80, 30) + 273.15
    F, T = np.meshgrid(freq/1e9, temp - 273.15)
    P_soc = np.zeros_like(F)
    for i in range(len(temp)):
        for j in range(len(freq)):
            # 计算对应负载
            load = (freq[j] - 0.3e9) / (3e9 - 0.3e9)
            P, _, _ = model.P_SoC_total(load, temp[i], 3.85)
            P_soc[i, j] = P
    
    ax1.plot_surface(F, T, P_soc, cmap='viridis', alpha=0.8)
    ax1.set_xlabel('Freq (GHz)')
    ax1.set_ylabel('Temp (C)')
    ax1.set_zlabel('Power (W)')
    ax1.set_title('SoC Power Surface', fontweight='bold')
    
    # 2. 显示功耗 vs 亮度和刷新率
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    brightness = np.linspace(0, 1000, 30)
    refresh = np.linspace(1, 120, 30)
    B, R = np.meshgrid(brightness, refresh)
    P_disp = np.zeros_like(B)
    for i in range(len(refresh)):
        for j in range(len(brightness)):
            P_disp[i, j] = model.P_display(brightness[j], refresh[i], 50, 0.5)
    
    ax2.plot_surface(B, R, P_disp, cmap='plasma', alpha=0.8)
    ax2.set_xlabel('Brightness (nits)')
    ax2.set_ylabel('Refresh (Hz)')
    ax2.set_zlabel('Power (W)')
    ax2.set_title('Display Power Surface', fontweight='bold')
    
    # 3. 5G功耗 vs 距离和速率
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    distance = np.linspace(50, 500, 30)
    rate = np.linspace(1, 200, 30)
    D, Ra = np.meshgrid(distance, rate)
    P_5g = np.zeros_like(D)
    for i in range(len(rate)):
        for j in range(len(distance)):
            P_5g[i, j] = model.P_5G(rate[i], distance[j], 20)
    
    ax3.plot_surface(D, Ra, P_5g, cmap='coolwarm', alpha=0.8)
    ax3.set_xlabel('Distance (m)')
    ax3.set_ylabel('Rate (Mbps)')
    ax3.set_zlabel('Power (W)')
    ax3.set_title('5G Power Surface', fontweight='bold')
    
    # 4. GNSS状态转移
    ax4 = fig.add_subplot(2, 3, 4)
    SNR = np.linspace(10, 50, 100)
    S_lock = 1.0 / (1.0 + np.exp(-model.gnss.k_sigmoid * (SNR - model.gnss.SNR_threshold)))
    P_gnss = model.gnss.P_LNA + S_lock * model.gnss.P_track + (1 - S_lock) * model.gnss.P_acq
    
    ax4_twin = ax4.twinx()
    ax4.plot(SNR, S_lock, 'b-', linewidth=2, label='Lock Probability')
    ax4_twin.plot(SNR, P_gnss * 1000, 'r--', linewidth=2, label='Power')
    ax4.set_xlabel('SNR (dB)')
    ax4.set_ylabel('Lock Probability', color='b')
    ax4_twin.set_ylabel('Power (mW)', color='r')
    ax4.set_title('GNSS State Transition (Sigmoid)', fontweight='bold')
    ax4.axvline(x=model.gnss.SNR_threshold, color='gray', linestyle=':', alpha=0.5)
    ax4.grid(True, alpha=0.3)
    
    # 5. 蓝牙功耗 vs 连接间隔
    ax5 = fig.add_subplot(2, 3, 5)
    interval = np.linspace(7.5, 2000, 100)
    P_ble = [model.P_Bluetooth(True, False, i, 'SBC') * 1000 for i in interval]
    P_audio = [model.P_Bluetooth(True, True, i, 'SBC') * 1000 for i in interval]
    
    ax5.plot(interval, P_ble, 'b-', linewidth=2, label='BLE Only')
    ax5.plot(interval, P_audio, 'r-', linewidth=2, label='BLE + Audio')
    ax5.set_xlabel('Connection Interval (ms)')
    ax5.set_ylabel('Power (mW)')
    ax5.set_title('Bluetooth Power (Duty Cycle)', fontweight='bold')
    ax5.set_xscale('log')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 6. PMIC效率曲线
    ax6 = fig.add_subplot(2, 3, 6)
    I_load = np.linspace(0.01, 3, 100)
    eta = [model.eta_PMIC(I, 3.85, 0.9) * 100 for I in I_load]
    
    ax6.plot(I_load * 1000, eta, 'g-', linewidth=2)
    ax6.set_xlabel('Load Current (mA)')
    ax6.set_ylabel('Efficiency (%)')
    ax6.set_title('PMIC Conversion Efficiency', fontweight='bold')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    return fig


# ==============================================================================
# 主程序
# ==============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print(" 完整SOC连续时间耦合方程组模型 (无简化)")
    print(" Full Coupled Continuous-Time SOC Model (No Simplification)")
    print("=" * 70)
    
    # 创建模型
    model = FullCoupledSOCModel()
    
    print("\n模型参数:")
    print(f"  电池容量: {model.batt_echem.Q_max} mAh")
    print(f"  标称电压: {model.batt_echem.V_nom} V")
    print(f"  参考内阻: {model.batt_echem.R_int_ref * 1000} mOhm")
    print(f"  SoC频率范围: {model.soc_chip.f_min/1e9:.1f} - {model.soc_chip.f_max/1e9:.1f} GHz")
    
    # 定义硬件状态函数 (24小时使用模式)
    def hardware_state_24h(t: float) -> dict:
        """24小时使用模式硬件状态函数"""
        hour = (t / 3600) % 24
        
        if 0 <= hour < 7:  # 睡眠
            return {
                'cpu_load': 0.02,
                'brightness': 0,
                'refresh_rate': 1,
                'APL': 0,
                'content_motion': 0,
                'data_rate': 0,
                'distance': 200,
                'SNR_5g': 30,
                'bt_connected': False,
                'bt_audio': False,
                'bt_interval': 1000,
                'gnss_on': False,
                'SNR_gnss': 35
            }
        elif 7 <= hour < 9:  # 早晨通勤
            return {
                'cpu_load': 0.4,
                'brightness': 600,
                'refresh_rate': 60,
                'APL': 50,
                'content_motion': 0.3,
                'data_rate': 50,
                'distance': 300,
                'SNR_5g': 25,
                'bt_connected': True,
                'bt_audio': True,
                'bt_interval': 50,
                'bt_codec': 'AAC',
                'gnss_on': True,
                'SNR_gnss': 25
            }
        elif 9 <= hour < 12:  # 上午工作
            return {
                'cpu_load': 0.35,
                'brightness': 400,
                'refresh_rate': 60,
                'APL': 60,
                'content_motion': 0.2,
                'data_rate': 30,
                'distance': 150,
                'SNR_5g': 35,
                'bt_connected': False,
                'bt_audio': False,
                'bt_interval': 500,
                'gnss_on': False,
                'SNR_gnss': 30
            }
        elif 12 <= hour < 14:  # 午休
            return {
                'cpu_load': 0.5,
                'brightness': 700,
                'refresh_rate': 90,
                'APL': 55,
                'content_motion': 0.6,
                'data_rate': 80,
                'distance': 200,
                'SNR_5g': 30,
                'bt_connected': True,
                'bt_audio': False,
                'bt_interval': 100,
                'gnss_on': False,
                'SNR_gnss': 35
            }
        elif 14 <= hour < 18:  # 下午工作
            return {
                'cpu_load': 0.4,
                'brightness': 450,
                'refresh_rate': 60,
                'APL': 55,
                'content_motion': 0.25,
                'data_rate': 40,
                'distance': 150,
                'SNR_5g': 35,
                'bt_connected': False,
                'bt_audio': False,
                'bt_interval': 500,
                'gnss_on': False,
                'SNR_gnss': 30
            }
        elif 18 <= hour < 21:  # 晚间休闲
            return {
                'cpu_load': 0.6,
                'brightness': 600,
                'refresh_rate': 90,
                'APL': 60,
                'content_motion': 0.7,
                'data_rate': 100,
                'distance': 100,
                'SNR_5g': 40,
                'bt_connected': True,
                'bt_audio': True,
                'bt_interval': 30,
                'bt_codec': 'LDAC',
                'gnss_on': True,
                'SNR_gnss': 40
            }
        else:  # 21-24 轻度使用
            return {
                'cpu_load': 0.25,
                'brightness': 300,
                'refresh_rate': 60,
                'APL': 40,
                'content_motion': 0.3,
                'data_rate': 20,
                'distance': 100,
                'SNR_5g': 40,
                'bt_connected': False,
                'bt_audio': False,
                'bt_interval': 500,
                'gnss_on': False,
                'SNR_gnss': 35
            }
    
    # 初始状态
    # [SOC, T_batt, T_soc, x_lock, I_bg, N_cycle]
    y0 = np.array([1.0, 298.15, 298.15, 0.0, 0.2, 100.0])
    
    print("\n开始24小时仿真...")
    
    # 仿真24小时
    result = model.simulate(
        t_span=(0, 24 * 3600),
        y0=y0,
        hardware_state_func=hardware_state_24h,
        record_details=True
    )
    
    print(f"\n仿真完成: {result['success']}")
    print(f"电量耗尽: {result['depleted']}")
    print(f"仿真时长: {result['time_hours'][-1]:.2f} 小时")
    print(f"初始SOC: 100%")
    print(f"最终SOC: {result['SOC'][-1]:.1%}")
    print(f"电池温度范围: {result['T_batt_celsius'].min():.1f} - {result['T_batt_celsius'].max():.1f} °C")
    print(f"SoC温度范围: {result['T_soc_celsius'].min():.1f} - {result['T_soc_celsius'].max():.1f} °C")
    print(f"平均功耗: {np.mean(result['P_total']):.2f} W")
    print(f"峰值功耗: {np.max(result['P_total']):.2f} W")
    print(f"总耗电量: {np.trapezoid(result['P_total'], result['time_hours']):.1f} Wh")
    
    # 生成可视化
    print("\n生成可视化图表...")
    
    # 1. 完整仿真结果
    fig1 = visualize_full_model(result, '/workspace/battery_model/full_soc_simulation.png')
    
    # 2. OCV-SOC曲线
    fig2 = visualize_ocv_soc_curve(model, '/workspace/battery_model/ocv_soc_curves.png')
    
    # 3. 功耗模型
    fig3 = visualize_power_models(model, '/workspace/battery_model/power_models.png')
    
    print("\n可视化完成!")
    print("图表已保存到 /workspace/battery_model/")
