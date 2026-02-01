#!/usr/bin/env python3
"""
智能手机电池耦合微分方程组建模与剩余时间预测
Smartphone Battery Coupled ODE Modeling and Remaining Time Prediction

严格按照文档2的耦合微分方程组进行推导求解:

【3.1 电池电化学热耦合核心】
dSOC(t)/dt = -I_total(t) / (Q_max · f(T_batt, N))
C_th · dT_batt/dt = P_joule(t) + P_entropy(t) - (T_batt - T_env)/R_th
V_batt(t) = V_OCV(SOC) - I_total(t) · R_int(SOC, T_batt, N)
P_joule = I_total² · R_int
P_entropy = I_total · T_batt · |∂V_OCV/∂T|

【3.2 总电流耦合方程（能量守恒）】
I_total(t) = [P_SoC(t) + P_disp(t) + P_5G(t) + P_BT(t) + P_GNSS(t) + P_bg(t)] / (η_PMIC · V_load · V_batt(t))

【4.1 SoC模块（电热强耦合）】
P_SoC(t) = f_cpu(t) · V_dd² + I_leak(T_soc) · V_dd
C_th,soc · dT_soc/dt = P_SoC(t) - (T_soc - T_batt)/R_th,soc-batt - (T_soc - T_env)/R_th,soc-env
f_cpu(t) = g(Load(t), T_batt, V_dd)  [DVFS映射]

【4.2 显示模块（环境光内容耦合）】
P_disp(t) = P_static + k_drv · f_refresh(t) + L_set(t) · A(t)

【4.5 GNSS模块（环境信号耦合）】
P_GNSS(t) = P_LNA + x_lock(t) · P_track + [1-x_lock(t)] · P_acq
dx_lock(t)/dt = [1/τ_react] · [S(t) - x_lock(t)]
S(t) = 1 / (1 + exp(-λ·(S_env - S_th)))

【4.6 后台任务模块（随机过程耦合）】
P_bg(t) = V_batt · I_bg(t)
dI_bg(t) = θ · [μ(t) - I_bg(t)]dt + σ · dW_t
μ(t) = μ_0 · exp(λ · UserActivity(t))

【5.1 连续时间马尔科夫用户模型】
UserState(t) ∈ {S1, S2, S3, S4}
dp(t)/dt = p(t) · Q(t)
Q(t) = Q_sleep  [23:00, 7:00)
       Q_work   [9:00, 18:00) 
       Q_leisure 其他时段
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.gridspec as gridspec
import os

# ============================================================================
# 柔和鲜亮配色方案
# ============================================================================
COLORS = {
    # 主色调 - 柔和渐变
    'primary': '#6C9BCF',      # 柔和蓝
    'secondary': '#F2994A',    # 柔和橙
    'accent': '#67B7A0',       # 柔和绿
    'highlight': '#E97171',    # 柔和红
    'purple': '#9B7ED9',       # 柔和紫
    'pink': '#F5A6C6',         # 柔和粉
    
    # 状态颜色
    'sleep': '#A8D8EA',        # 浅蓝 - 睡眠
    'light': '#98D9A8',        # 浅绿 - 轻度使用
    'stream': '#FFE5A0',       # 浅黄 - 流媒体
    'game': '#FFB5B5',         # 浅红 - 游戏
    
    # 时段背景
    'bg_sleep': '#E8F4F8',     # 睡眠时段
    'bg_work': '#FFF0E5',      # 工作时段
    'bg_leisure': '#E8F8E8',   # 休闲时段
    
    # 曲线颜色
    'soc': '#FF8C42',          # SOC - 温暖橙
    'current': '#4A90D9',      # 电流 - 明亮蓝
    'temp_batt': '#E96A6A',    # 电池温度 - 柔和红
    'temp_cpu': '#5DADE2',     # CPU温度 - 天蓝
    'power': '#67B7A0',        # 功率 - 青绿
    'brightness': '#F7DC6F',   # 亮度 - 明黄
}

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'figure.dpi': 150,
    'axes.facecolor': '#FAFBFC',
    'figure.facecolor': 'white',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.color': '#CCCCCC',
})


# ============================================================================
# 耦合微分方程组模型 (严格按照文档2)
# ============================================================================

class CoupledBatteryModel:
    """
    严格按照文档2的耦合微分方程组实现
    """
    
    def __init__(self):
        # ============ 3.1节 电池电化学热耦合参数 ============
        self.Q_max = 4.0 * 3600       # As, 最大容量
        self.V_nom = 3.7              # V, 标称电压
        self.V_max = 4.2              # V
        self.V_min = 3.0              # V
        
        # 内阻参数 R_int(SOC, T_batt, N)
        self.R_0 = 0.042              # Ω, 基准内阻
        self.E_a = 20000              # J/mol, 活化能
        self.R_gas = 8.314            # J/(mol·K)
        self.T_ref = 298.15           # K, 参考温度
        
        # 热参数
        self.C_th_batt = 48.0         # J/K, 电池热容 C_th
        self.R_th_batt = 11.0         # K/W, 电池热阻 R_th
        self.dV_dT = -0.0003          # V/K, 熵系数 ∂V_OCV/∂T
        
        # 老化参数
        self.N_cycles = 150           # 循环次数 N
        self.alpha_N = 0.0008         # 容量衰减率
        
        # PMIC效率
        self.eta_PMIC = 0.91          # η_PMIC
        
        # ============ 4.1节 SoC模块参数 ============
        self.C_th_soc = 2.0           # J/K, CPU热容
        self.R_th_soc_batt = 4.5      # K/W
        self.R_th_soc_env = 16.0      # K/W
        self.C_eff = 2e-9             # F, 等效开关电容
        self.P_leak_0 = 0.15          # W, 25°C泄漏功率
        self.k_leak = 0.04            # 1/K, 温度系数
        
        # ============ 4.2节 显示模块参数 ============
        self.P_disp_static = 0.08     # W, P_static
        self.k_drv = 8e-6             # W/Hz, 驱动系数
        self.k_brightness = 0.002     # W/nit
        
        # ============ 4.5节 GNSS模块参数 ============
        self.P_LNA = 0.015            # W
        self.P_acq = 0.12             # W, 捕获功率
        self.P_track = 0.06           # W, 跟踪功率
        self.tau_react = 25.0         # s, τ_react
        self.tau_unlock = 8.0         # s
        self.lambda_S = 5.0           # 信号响应参数
        self.S_th = 0.5               # 信号阈值
        
        # ============ 4.6节 后台任务O-U过程参数 ============
        self.theta_bg = 0.08          # 均值回复率 θ
        self.mu_0 = 0.03              # A, 基准均值
        self.sigma_bg = 0.008         # 波动率 σ
        self.lambda_activity = 0.5    # 活动系数 λ
        
        # ============ 5.1节 马尔科夫参数 ============
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
        
        # 5.2节 状态到硬件参数映射
        self.state_params = {
            0: {'f_cpu': 0.3, 'V_dd': 0.65, 'Load': 0.01, 'L_set': 0, 'f_refresh': 0},
            1: {'f_cpu': 1.3, 'V_dd': 0.80, 'Load': 0.15, 'L_set': 300, 'f_refresh': 60},
            2: {'f_cpu': 1.5, 'V_dd': 0.85, 'Load': 0.25, 'L_set': 450, 'f_refresh': 60},
            3: {'f_cpu': 2.6, 'V_dd': 1.00, 'Load': 0.85, 'L_set': 600, 'f_refresh': 90}
        }
    
    # ========== 3.1节方程实现 ==========
    
    def V_OCV(self, SOC: float) -> float:
        """开路电压 V_OCV(SOC) - 公式(3.1)"""
        SOC = np.clip(SOC, 0.001, 0.999)
        # 多项式+指数修正模型
        V = 3.0 + 0.9 * SOC + 0.15 * SOC**2 + 0.15 * SOC**3
        V += 0.15 * (1 - np.exp(-8 * (SOC - 0.85))) * (SOC > 0.85)
        V -= 0.25 * np.exp(-12 * SOC)
        return np.clip(V, self.V_min, self.V_max)
    
    def R_int(self, SOC: float, T_batt: float, N: int = None) -> float:
        """内阻 R_int(SOC, T_batt, N) - 公式(3.1)"""
        if N is None:
            N = self.N_cycles
        SOC = np.clip(SOC, 0.01, 0.99)
        
        # f(SOC): U型曲线
        f_SOC = 1.0 + 0.4 * (SOC - 0.5)**2 + 0.5 * np.exp(-8 * SOC)
        
        # g(T): Arrhenius温度依赖
        g_T = np.exp(self.E_a / self.R_gas * (1/T_batt - 1/self.T_ref))
        
        # h(N): 老化因子
        h_N = 1.0 + self.alpha_N * N * 1.5
        
        return self.R_0 * f_SOC * g_T * h_N
    
    def capacity_factor(self, T_batt: float, N: int = None) -> float:
        """容量因子 f(T_batt, N)"""
        if N is None:
            N = self.N_cycles
        T_c = T_batt - 273.15
        f_T = 1.0 - 0.012 * max(0, 15 - T_c) - 0.003 * max(0, T_c - 40)
        f_N = 1.0 - self.alpha_N * N
        return max(0.7, f_T * f_N)
    
    def P_joule(self, I_total: float, R_int: float) -> float:
        """焦耳热 P_joule = I²R - 公式(3.1)"""
        return I_total**2 * R_int
    
    def P_entropy(self, I_total: float, T_batt: float) -> float:
        """熵热 P_entropy = I·T·|∂V/∂T| - 公式(3.1)"""
        return abs(I_total) * T_batt * abs(self.dV_dT)
    
    # ========== 4.1节 SoC模块 ==========
    
    def P_SoC(self, f_cpu: float, V_dd: float, T_soc: float, Load: float) -> float:
        """
        SoC功耗 - 公式(4.1)
        P_SoC = f_cpu · V_dd² + I_leak(T_soc) · V_dd
        """
        f = f_cpu * 1e9  # GHz -> Hz
        P_dyn = self.C_eff * V_dd**2 * f * Load
        P_leak = self.P_leak_0 * np.exp(self.k_leak * (T_soc - self.T_ref)) * V_dd
        return P_dyn + P_leak
    
    # ========== 4.2节 显示模块 ==========
    
    def P_disp(self, L_set: float, f_refresh: float, APL: float = 0.5) -> float:
        """
        显示功耗 - 公式(4.2)
        P_disp = P_static + k_drv · f_refresh + L_set · A(t)
        """
        if L_set <= 0:
            return 0.0
        return self.P_disp_static + self.k_drv * f_refresh + self.k_brightness * L_set * APL
    
    # ========== 4.5节 GNSS模块 ==========
    
    def P_GNSS(self, x_lock: float, active: bool) -> float:
        """
        GNSS功耗 - 公式(4.5)
        P_GNSS = P_LNA + x_lock · P_track + (1-x_lock) · P_acq
        """
        if not active:
            return 0.0
        return self.P_LNA + x_lock * self.P_track + (1 - x_lock) * self.P_acq
    
    def S_signal(self, S_env: float) -> float:
        """
        信号质量函数 - 公式(4.5)
        S(t) = 1 / (1 + exp(-λ·(S_env - S_th)))
        """
        return 1.0 / (1.0 + np.exp(-self.lambda_S * (S_env - self.S_th)))
    
    # ========== 4.6节 后台任务 ==========
    
    def mu_bg(self, UserActivity: float) -> float:
        """
        后台均值 - 公式(4.6)
        μ(t) = μ_0 · exp(λ · UserActivity(t))
        """
        return self.mu_0 * np.exp(self.lambda_activity * UserActivity)
    
    def P_background(self, V_batt: float, I_bg: float) -> float:
        """后台功耗 P_bg = V_batt · I_bg"""
        return V_batt * I_bg
    
    # ========== 5.1节 马尔科夫模型 ==========
    
    def get_Q_matrix(self, hour: float) -> np.ndarray:
        """
        获取转移率矩阵 Q(t) - 公式(5.1)
        Q = (P - I) / Δt
        """
        if hour >= 23 or hour < 7:
            P = self.P_sleep
        elif (9 <= hour < 12) or (14 <= hour < 18):
            P = self.P_work
        else:
            P = self.P_leisure
        
        dt = 60.0  # 1分钟
        return (P - np.eye(4)) / dt
    
    def get_time_mode(self, hour: float) -> str:
        """获取时间模式"""
        if hour >= 23 or hour < 7:
            return 'sleep'
        elif (9 <= hour < 12) or (14 <= hour < 18):
            return 'work'
        return 'leisure'
    
    # ========== 5.2节 状态映射 ==========
    
    def state_to_params(self, p_states: np.ndarray, hour: float) -> dict:
        """
        状态到硬件参数映射 - 公式(5.2)
        [f_cpu, A_set, L_set, R_data, f_refresh] = M(UserState) + ε(t)
        """
        params = {k: 0.0 for k in ['f_cpu', 'V_dd', 'Load', 'L_set', 'f_refresh']}
        
        for i in range(4):
            for k in params:
                params[k] += p_states[i] * self.state_params[i][k]
        
        # 环境扰动 ε(t) - 阳光因子
        sunlight = np.exp(-((hour - 13)**2) / 18)
        params['L_set'] *= (1 + 0.3 * sunlight)
        
        # 微波动
        params['f_cpu'] += 0.03
        
        return params


# ============================================================================
# 耦合ODE求解器
# ============================================================================

def solve_coupled_ode(model: CoupledBatteryModel, T_hours: float = 24):
    """
    求解完整耦合微分方程组
    
    状态向量: y = [SOC, T_batt, T_soc, x_lock, I_bg, p0, p1, p2, p3]
    """
    print("\n" + "="*70)
    print("【耦合微分方程组求解】")
    print("="*70)
    
    print("\n状态向量: y = [SOC, T_batt, T_soc, x_lock, I_bg, p₀, p₁, p₂, p₃]")
    print("\n方程组:")
    print("  [3.1] dSOC/dt = -I_total / (Q_max · f(T,N))")
    print("  [3.1] C_th·dT_batt/dt = P_joule + P_entropy - (T_batt-T_env)/R_th")
    print("  [4.1] C_th,soc·dT_soc/dt = P_SoC - (T_soc-T_batt)/R₁ - (T_soc-T_env)/R₂")
    print("  [4.5] dx_lock/dt = (S(t) - x_lock) / τ_react")
    print("  [4.6] dI_bg = θ·(μ(t) - I_bg)dt + σ·dW")
    print("  [5.1] dp/dt = p · Q(t)")
    
    # 随机数序列
    np.random.seed(42)
    noise_seq = np.random.normal(0, 1, 2000)
    
    # 环境函数
    def T_env(hour):
        if 7 <= hour < 9 or 18 <= hour < 19:
            return 298.15 + 3 * np.sin(2 * np.pi * (hour - 6) / 24)
        return 295.15
    
    def is_charging(hour):
        return (2 <= hour < 3.5) or (13 <= hour < 13.5) or (22 <= hour < 23.5)
    
    def coupled_ode(t, y):
        # 解包状态
        SOC = np.clip(y[0], 0.01, 0.99)
        T_batt = np.clip(y[1], 270, 330)
        T_soc = np.clip(y[2], 270, 360)
        x_lock = np.clip(y[3], 0, 1)
        I_bg = np.clip(y[4], 0.001, 0.2)
        p_states = np.clip(y[5:9], 0.001, 0.999)
        p_states = p_states / np.sum(p_states)
        
        hour = (t / 3600) % 24
        T_e = T_env(hour)
        charging = is_charging(hour)
        
        # 5.2节: 状态到参数映射
        params = model.state_to_params(p_states, hour)
        
        # 计算各模块功耗
        # 4.1节: P_SoC
        P_soc = model.P_SoC(params['f_cpu'], params['V_dd'], T_soc, params['Load'])
        
        # 4.2节: P_disp
        P_disp = model.P_disp(params['L_set'], params['f_refresh'])
        
        # 其他模块
        P_5G = 0.05 + 0.3 * (1 - p_states[0])
        P_BT = 0.01 + 0.04 * (p_states[2] + p_states[3])
        
        # 4.5节: P_GNSS
        gps_active = p_states[3] > 0.1 or (7 <= hour < 9) or (18 <= hour < 19)
        P_gnss = model.P_GNSS(x_lock, gps_active)
        
        # 4.6节: P_bg
        V_batt = model.V_OCV(SOC)
        P_bg = model.P_background(V_batt, I_bg)
        
        # 3.2节: 总功耗和电流
        P_total = P_soc + P_disp + P_5G + P_BT + P_gnss + P_bg
        
        # 电压和内阻
        R_int = model.R_int(SOC, T_batt)
        
        if charging:
            if SOC < 0.80:
                I_total = -2.0  # CC
            elif SOC < 0.95:
                I_total = -2.0 * (0.98 - SOC) / 0.18  # CV
            else:
                I_total = -0.1
            if SOC > 0.99:
                I_total = 0
        else:
            # 3.2节: I_total = P_total / (η · V_batt)
            I_total = P_total / (model.eta_PMIC * max(V_batt - I_bg * R_int, model.V_min))
        
        # ========== 微分方程 ==========
        
        # 3.1节: dSOC/dt
        Q_eff = model.Q_max * model.capacity_factor(T_batt)
        dSOC = -I_total / Q_eff
        if (SOC >= 0.99 and dSOC > 0) or (SOC <= 0.05 and dSOC < 0):
            dSOC = 0
        
        # 3.1节: dT_batt/dt
        P_j = model.P_joule(I_total, R_int)
        P_e = model.P_entropy(I_total, T_batt)
        dT_batt = (P_j + P_e - (T_batt - T_e) / model.R_th_batt) / model.C_th_batt
        
        # 4.1节: dT_soc/dt
        Q_soc_batt = (T_soc - T_batt) / model.R_th_soc_batt
        Q_soc_env = (T_soc - T_e) / model.R_th_soc_env
        dT_soc = (P_soc - Q_soc_batt - Q_soc_env) / model.C_th_soc
        
        # 4.5节: dx_lock/dt
        if gps_active:
            S_env = 0.7 + 0.2 * np.sin(hour)
            S = model.S_signal(S_env)
            dx_lock = (S - x_lock) / model.tau_react
        else:
            dx_lock = -x_lock / model.tau_unlock
        
        # 4.6节: dI_bg (O-U过程)
        UserActivity = 1 - p_states[0]
        mu = model.mu_bg(UserActivity)
        xi = noise_seq[int(t / 60) % len(noise_seq)]
        dI_bg = model.theta_bg * (mu - I_bg) + model.sigma_bg * xi
        
        # 5.1节: dp/dt = p · Q(t)
        Q_mat = model.get_Q_matrix(hour)
        dp = p_states @ Q_mat
        
        return [dSOC, dT_batt, dT_soc, dx_lock, dI_bg, dp[0], dp[1], dp[2], dp[3]]
    
    # 初始条件
    y0 = [0.85, 294.15, 296.15, 0.0, 0.03, 0.90, 0.08, 0.01, 0.01]
    
    # 求解
    print("\n求解中...")
    t_span = (0, T_hours * 3600)
    t_eval = np.linspace(0, T_hours * 3600, int(T_hours * 60) + 1)
    
    solution = solve_ivp(coupled_ode, t_span, y0, method='RK45', t_eval=t_eval,
                        max_step=60, rtol=1e-4, atol=1e-6)
    
    print(f"求解状态: {'成功' if solution.success else '失败'}")
    print(f"函数评估次数: {solution.nfev}")
    
    # 后处理
    t_hours = solution.t / 3600
    n = len(t_hours)
    
    SOC = np.clip(solution.y[0], 0.05, 1.0) * 100
    T_batt = solution.y[1] - 273.15
    T_soc = solution.y[2] - 273.15
    x_lock = solution.y[3]
    I_bg = solution.y[4]
    p_states = solution.y[5:9]
    
    # 归一化概率
    for i in range(n):
        p_states[:, i] /= np.sum(p_states[:, i])
    
    # 计算派生量
    P_total = np.zeros(n)
    I_total = np.zeros(n)
    brightness = np.zeros(n)
    
    for i in range(n):
        hour = t_hours[i] % 24
        p = p_states[:, i]
        params = model.state_to_params(p, hour)
        
        brightness[i] = params['L_set']
        
        P_soc = model.P_SoC(params['f_cpu'], params['V_dd'], solution.y[2, i], params['Load'])
        P_disp = model.P_disp(params['L_set'], params['f_refresh'])
        P_5G = 0.05 + 0.3 * (1 - p[0])
        P_BT = 0.01 + 0.04 * (p[2] + p[3])
        P_gnss = model.P_GNSS(solution.y[3, i], p[3] > 0.1)
        P_bg = model.P_background(model.V_OCV(solution.y[0, i]), solution.y[4, i])
        P_total[i] = P_soc + P_disp + P_5G + P_BT + P_gnss + P_bg
        
        if is_charging(hour):
            soc = solution.y[0, i]
            I_total[i] = -2.0 if soc < 0.8 else (-2.0 * (0.98 - soc) / 0.18 if soc < 0.95 else -0.1)
        else:
            V = model.V_OCV(solution.y[0, i]) - 0.1
            I_total[i] = P_total[i] / (model.eta_PMIC * max(V, model.V_min))
    
    return {
        't_hours': t_hours,
        'SOC': SOC,
        'T_batt': T_batt,
        'T_soc': T_soc,
        'x_lock': x_lock,
        'I_bg': I_bg,
        'p_states': p_states,
        'P_total': P_total,
        'I_total': I_total,
        'brightness': brightness,
        'model': model
    }


# ============================================================================
# 剩余时间预测
# ============================================================================

def predict_remaining_time(model: CoupledBatteryModel, SOC_current: float, 
                           P_avg: float, T_batt: float = 298.15) -> float:
    """
    预测剩余时间
    t_remain = ∫_{SOC_cutoff}^{SOC_current} Q_eff·V(SOC) / P dSOC
    """
    SOC_cutoff = 0.05
    if SOC_current <= SOC_cutoff or P_avg <= 0:
        return 0.0
    
    Q_eff = model.Q_max * model.capacity_factor(T_batt)
    
    # 数值积分
    n_points = 100
    SOC_range = np.linspace(SOC_cutoff, SOC_current, n_points)
    V_range = np.array([model.V_OCV(s) for s in SOC_range])
    
    # 积分: ∫ Q_eff·V / P dSOC
    integrand = Q_eff * V_range / P_avg
    t_remain = np.trapz(integrand, SOC_range)  # seconds
    
    return t_remain / 3600  # hours


# ============================================================================
# 可视化
# ============================================================================

def create_visualization(data, save_path=None):
    """创建柔和鲜亮的综合可视化"""
    
    fig = plt.figure(figsize=(18, 22))
    gs = gridspec.GridSpec(6, 2, height_ratios=[1, 1, 1, 0.8, 1, 0.8], 
                          hspace=0.32, wspace=0.22)
    
    t = data['t_hours']
    model = data['model']
    
    # 时段信息
    periods = [
        (0, 7, 'bg_sleep', 'Sleep'), (7, 9, 'bg_leisure', 'Leisure'),
        (9, 12, 'bg_work', 'Work'), (12, 14, 'bg_leisure', 'Leisure'),
        (14, 18, 'bg_work', 'Work'), (18, 23, 'bg_leisure', 'Leisure'),
        (23, 24, 'bg_sleep', 'Sleep')
    ]
    
    def draw_bg(ax, y_min, y_max):
        for start, end, color_key, name in periods:
            rect = Rectangle((start, y_min), end - start, y_max - y_min,
                            facecolor=COLORS[color_key], edgecolor='none', 
                            alpha=0.7, zorder=0)
            ax.add_patch(rect)
            ax.text((start + end) / 2, y_max - (y_max - y_min) * 0.04, name,
                   ha='center', va='top', fontsize=9, fontweight='bold',
                   color='#666666', alpha=0.8)
    
    # ========== 1. 马尔科夫状态概率 ==========
    ax1 = fig.add_subplot(gs[0, :])
    draw_bg(ax1, 0, 1.08)
    
    p = data['p_states']
    ax1.stackplot(t, p[0], p[1], p[2], p[3],
                 colors=[COLORS['sleep'], COLORS['light'], COLORS['stream'], COLORS['game']],
                 labels=['Deep Sleep', 'Light Use', 'Streaming', 'Gaming'],
                 alpha=0.85)
    
    ax1.set_xlim(0, 24)
    ax1.set_ylim(0, 1.08)
    ax1.set_xlabel('Time (hour)', fontsize=11)
    ax1.set_ylabel('State Probability', fontsize=11)
    ax1.set_title('[Sec 5.1] User Behavior: Continuous-Time Markov Chain\n' +
                 r'$\frac{dp(t)}{dt} = p(t) \cdot Q(t)$, where $Q(t) \in \{Q_{sleep}, Q_{work}, Q_{leisure}\}$',
                 fontsize=13, fontweight='bold', color='#2C3E50')
    ax1.legend(loc='upper right', ncol=4, fontsize=9, framealpha=0.9)
    ax1.set_xticks(range(0, 25, 2))
    
    # ========== 2. SOC动力学 ==========
    ax2 = fig.add_subplot(gs[1, 0])
    draw_bg(ax2, -30, 115)
    
    ax2.plot(t, data['SOC'], color=COLORS['soc'], linewidth=2.5, label='SOC [%]')
    ax2.fill_between(t, 0, data['SOC'], color=COLORS['soc'], alpha=0.15)
    ax2.plot(t, data['I_total'] * 30, color=COLORS['current'], linewidth=1.8, 
            label='Current [A×30]', alpha=0.9)
    ax2.axhline(y=5, color='#E74C3C', linestyle='--', alpha=0.6, linewidth=1.5)
    
    ax2.set_xlim(0, 24)
    ax2.set_ylim(-30, 115)
    ax2.set_xlabel('Time (hour)')
    ax2.set_ylabel('SOC (%) / Current')
    ax2.set_title('[Sec 3.1] SOC Dynamics\n' +
                 r'$\frac{dSOC}{dt} = -\frac{I_{total}}{Q_{max} \cdot f(T_{batt}, N)}$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax2.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax2.set_xticks(range(0, 25, 2))
    
    # ========== 3. 温度动力学 ==========
    ax3 = fig.add_subplot(gs[1, 1])
    draw_bg(ax3, 18, 42)
    
    ax3.plot(t, data['T_batt'], color=COLORS['temp_batt'], linewidth=2.5, label='Battery T')
    ax3.plot(t, data['T_soc'], color=COLORS['temp_cpu'], linewidth=2.5, label='SoC T')
    ax3.axhline(y=22, color='#95A5A6', linestyle='--', alpha=0.6, label='Ambient')
    
    ax3.set_xlim(0, 24)
    ax3.set_ylim(18, 42)
    ax3.set_xlabel('Time (hour)')
    ax3.set_ylabel('Temperature (°C)')
    ax3.set_title('[Sec 3.1 & 4.1] Thermal Dynamics\n' +
                 r'$C_{th}\frac{dT}{dt} = P_{joule} + P_{entropy} - \frac{T-T_{env}}{R_{th}}$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax3.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax3.set_xticks(range(0, 25, 2))
    
    # ========== 4. SOC-功率关系 ==========
    ax4 = fig.add_subplot(gs[2, 0])
    
    scatter = ax4.scatter(data['SOC'], data['P_total'], c=t, cmap='viridis', 
                         s=8, alpha=0.7, edgecolors='none')
    cbar = plt.colorbar(scatter, ax=ax4)
    cbar.set_label('Time (hour)', fontsize=10)
    
    # 趋势线
    z = np.polyfit(data['SOC'], data['P_total'], 2)
    soc_fit = np.linspace(5, 100, 100)
    ax4.plot(soc_fit, np.polyval(z, soc_fit), color=COLORS['highlight'], 
            linewidth=2, linestyle='--', label='Trend')
    
    ax4.set_xlabel('SOC (%)')
    ax4.set_ylabel('Power (W)')
    ax4.set_title('[Sec 3.2] SOC-Power Relationship\n' +
                 r'$I_{total} = \frac{\sum P_i}{\eta_{PMIC} \cdot V_{batt}}$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax4.legend(loc='upper right', fontsize=9)
    ax4.set_xlim(0, 105)
    
    # ========== 5. 功率分解 ==========
    ax5 = fig.add_subplot(gs[2, 1])
    draw_bg(ax5, 0, 5.5)
    
    ax5.fill_between(t, 0, data['P_total'], color=COLORS['power'], alpha=0.4)
    ax5.plot(t, data['P_total'], color=COLORS['power'], linewidth=2, label='Total Power')
    
    ax5.set_xlim(0, 24)
    ax5.set_ylim(0, 5.5)
    ax5.set_xlabel('Time (hour)')
    ax5.set_ylabel('Power (W)')
    ax5.set_title('[Sec 3.2] Total Power Consumption\n' +
                 r'$P_{total} = P_{SoC} + P_{disp} + P_{5G} + P_{BT} + P_{GNSS} + P_{bg}$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax5.legend(loc='upper right', fontsize=9)
    ax5.set_xticks(range(0, 25, 2))
    
    # ========== 6. GNSS与后台 ==========
    ax6 = fig.add_subplot(gs[3, 0])
    draw_bg(ax6, 0, 1.1)
    
    ax6.fill_between(t, 0, data['x_lock'], color=COLORS['purple'], alpha=0.4)
    ax6.plot(t, data['x_lock'], color=COLORS['purple'], linewidth=2, label='GPS Lock')
    
    ax6b = ax6.twinx()
    ax6b.plot(t, data['I_bg'] * 1000, color=COLORS['accent'], linewidth=1.5, label='I_bg')
    ax6b.set_ylabel('I_bg (mA)', color=COLORS['accent'])
    
    ax6.set_xlim(0, 24)
    ax6.set_ylim(0, 1.1)
    ax6.set_xlabel('Time (hour)')
    ax6.set_ylabel('Lock State', color=COLORS['purple'])
    ax6.set_title('[Sec 4.5 & 4.6] GNSS Lock & Background O-U Process\n' +
                 r'$\frac{dx_{lock}}{dt} = \frac{S(t)-x_{lock}}{\tau}$, '
                 r'$dI_{bg} = \theta(\mu-I_{bg})dt + \sigma dW$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax6.legend(loc='upper left', fontsize=9)
    ax6b.legend(loc='upper right', fontsize=9)
    ax6.set_xticks(range(0, 25, 2))
    
    # ========== 7. 显示功耗 ==========
    ax7 = fig.add_subplot(gs[3, 1])
    draw_bg(ax7, 0, 700)
    
    ax7.fill_between(t, 0, data['brightness'], color=COLORS['brightness'], alpha=0.5)
    ax7.plot(t, data['brightness'], color='#D4A84B', linewidth=2)
    
    ax7.set_xlim(0, 24)
    ax7.set_ylim(0, 700)
    ax7.set_xlabel('Time (hour)')
    ax7.set_ylabel('Brightness (nits)')
    ax7.set_title('[Sec 4.2] Display Power Driver\n' +
                 r'$P_{disp} = P_{static} + k_{drv} \cdot f_{refresh} + L_{set} \cdot A(t)$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax7.set_xticks(range(0, 25, 2))
    
    # ========== 8. 剩余时间预测 ==========
    ax8 = fig.add_subplot(gs[4, 0])
    
    scenarios = ['Idle', 'Light Use', 'Streaming', 'Navigation', 'Gaming', 'Heavy']
    powers = [0.15, 0.8, 2.0, 2.8, 3.5, 4.0]
    times = [predict_remaining_time(model, 1.0, p) for p in powers]
    
    colors_bar = [COLORS['accent'], COLORS['primary'], COLORS['secondary'],
                 COLORS['highlight'], COLORS['purple'], COLORS['pink']]
    
    bars = ax8.barh(scenarios, times, color=colors_bar, alpha=0.85, 
                   edgecolor='white', linewidth=1.5)
    
    for bar, t_val, p in zip(bars, times, powers):
        ax8.text(bar.get_width() + 0.8, bar.get_y() + bar.get_height()/2,
                f'{t_val:.1f}h (P={p}W)', va='center', fontsize=9, color='#2C3E50')
    
    ax8.set_xlabel('Remaining Time (hours)')
    ax8.set_title('Remaining Time Prediction (from 100% SOC)\n' +
                 r'$t_{remain} = \int_{SOC_{cut}}^{SOC_0} \frac{Q_{eff} \cdot V(SOC)}{P} dSOC$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax8.set_xlim(0, max(times) * 1.35)
    
    # ========== 9. OCV-SOC曲线 ==========
    ax9 = fig.add_subplot(gs[4, 1])
    
    soc_range = np.linspace(0.01, 0.99, 100)
    V_OCV = [model.V_OCV(s) for s in soc_range]
    
    ax9.fill_between(soc_range * 100, model.V_min, V_OCV, color=COLORS['primary'], alpha=0.2)
    ax9.plot(soc_range * 100, V_OCV, color=COLORS['primary'], linewidth=3)
    ax9.axhline(y=model.V_min + 0.2, color=COLORS['highlight'], linestyle='--', 
               linewidth=1.5, label='Cutoff')
    
    ax9.set_xlabel('SOC (%)')
    ax9.set_ylabel('V_OCV (V)')
    ax9.set_title('[Sec 3.1] OCV-SOC Characteristic\n' +
                 r'$V_{batt} = V_{OCV}(SOC) - I_{total} \cdot R_{int}(SOC, T, N)$',
                 fontsize=11, fontweight='bold', color='#2C3E50')
    ax9.set_xlim(0, 100)
    ax9.set_ylim(2.9, 4.3)
    ax9.legend(loc='lower right', fontsize=9)
    
    # ========== 10. 放电预测曲线 ==========
    ax10 = fig.add_subplot(gs[5, :])
    
    scenario_info = [
        ('Idle', 0.15, COLORS['accent']),
        ('Light Use', 0.8, COLORS['primary']),
        ('Streaming', 2.0, COLORS['secondary']),
        ('Gaming', 3.5, COLORS['highlight'])
    ]
    
    for name, P, color in scenario_info:
        t_max = predict_remaining_time(model, 1.0, P)
        t_pred = np.linspace(0, min(t_max, 30), 150)
        SOC_pred = np.zeros_like(t_pred)
        SOC_pred[0] = 100
        
        Q_eff = model.Q_max * model.capacity_factor(298.15)
        
        for i in range(1, len(t_pred)):
            dt = (t_pred[i] - t_pred[i-1]) * 3600
            V = model.V_OCV(SOC_pred[i-1] / 100)
            I = P / (model.eta_PMIC * max(V, model.V_min))
            dSOC = -I * dt / Q_eff * 100
            SOC_pred[i] = max(5, SOC_pred[i-1] + dSOC)
        
        ax10.plot(t_pred, SOC_pred, color=color, linewidth=2.5, label=f'{name} (P={P}W)')
        ax10.fill_between(t_pred, 5, SOC_pred, color=color, alpha=0.08)
    
    ax10.axhline(y=5, color='#E74C3C', linestyle='--', alpha=0.7, linewidth=1.5)
    ax10.set_xlabel('Time (hours)', fontsize=11)
    ax10.set_ylabel('SOC (%)', fontsize=11)
    ax10.set_title('Predicted Discharge Curves: ' +
                  r'$SOC(t) = SOC_0 - \int_0^t \frac{P(\tau)}{\eta \cdot V(\tau) \cdot Q_{eff}} d\tau$',
                  fontsize=12, fontweight='bold', color='#2C3E50')
    ax10.set_xlim(0, 30)
    ax10.set_ylim(0, 105)
    ax10.legend(loc='upper right', fontsize=10, ncol=4, framealpha=0.9)
    ax10.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"\n图表已保存: {save_path}")
    
    return fig


# ============================================================================
# 主程序
# ============================================================================

def main():
    print("="*70)
    print("智能手机电池耦合微分方程组建模与剩余时间预测")
    print("="*70)
    
    # 初始化模型
    model = CoupledBatteryModel()
    
    # 求解ODE
    data = solve_coupled_ode(model)
    
    # 剩余时间预测
    print("\n【剩余时间预测】(从100% SOC)")
    for name, P in [('Idle', 0.15), ('Light', 0.8), ('Stream', 2.0), 
                    ('Nav', 2.8), ('Game', 3.5), ('Heavy', 4.0)]:
        t = predict_remaining_time(model, 1.0, P)
        print(f"  {name:8s}: {t:6.1f}h (P={P}W)")
    
    # 统计
    print("\n【仿真统计】")
    print(f"  初始SOC: {data['SOC'][0]:.1f}%")
    print(f"  最低SOC: {np.min(data['SOC']):.1f}%")
    print(f"  最高SOC: {np.max(data['SOC']):.1f}%")
    print(f"  最终SOC: {data['SOC'][-1]:.1f}%")
    print(f"  最高T_batt: {np.max(data['T_batt']):.1f}°C")
    print(f"  最高T_soc: {np.max(data['T_soc']):.1f}°C")
    print(f"  最大功耗: {np.max(data['P_total']):.2f}W")
    print(f"  平均功耗: {np.mean(data['P_total']):.2f}W")
    
    # 可视化
    print("\n【生成可视化】")
    output_dir = os.path.join(os.path.dirname(__file__), 'results')
    os.makedirs(output_dir, exist_ok=True)
    
    save_path = os.path.join(output_dir, 'battery_coupled_model_final.png')
    fig = create_visualization(data, save_path)
    plt.close(fig)
    
    print("\n" + "="*70)
    print("完成!")
    print("="*70)
    
    return data


if __name__ == "__main__":
    data = main()
