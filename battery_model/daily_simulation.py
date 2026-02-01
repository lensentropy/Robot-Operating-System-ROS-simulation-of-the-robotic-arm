#!/usr/bin/env python3
"""
24小时智能手机真实使用场景仿真
================================
Realistic 24-Hour Smartphone Usage Simulation

模拟一天中不同活动场景下的电池放电行为：
- 睡眠 (Sleep)
- 通勤 (Commute)  
- 办公 (Office)
- 娱乐 (Entertainment)
- 充电 (Charging)

输出变量：
- SOC: 荷电状态 [%]
- I: 负载电流 [A]
- T: 电池温度 [°C]
- L: 屏幕亮度 [%]
- S: 信号强度 [dBm]
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from scipy.integrate import solve_ivp
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 10

# =============================================================================
# 物理参数定义
# =============================================================================

class BatteryParameters:
    """电池物理参数"""
    # 标称参数
    Q_nom = 4.0          # 标称容量 [Ah]
    V_nom = 3.85         # 标称电压 [V]
    V_max = 4.35         # 最大电压 [V]
    V_min = 3.0          # 截止电压 [V]
    
    # 老化参数 (双指数模型)
    a_Q = -0.15
    b_Q = 0.025
    c_Q = 4.15
    d_Q = 0.0008
    
    # 阻抗参数 (幂律模型)
    a_R = 0.008
    b_R = 0.22
    c_R = 0.025
    
    # 温度修正参数
    S_Q_max = 1.02
    k_Q = 0.1
    T_0 = -12.0
    C_R = 0.75
    A_R = 1.2
    B_R = 0.055
    
    # OCV参数 (能斯特模型)
    K = [3.45, 0.18, -0.005, 0.06, -0.09]
    
    # RC电路参数
    R0_ratio = 0.35
    R1_ratio = 0.40
    R2_ratio = 0.25
    tau_1 = 25.0         # 电化学极化时间常数 [s]
    tau_2 = 250.0        # 浓差极化时间常数 [s]
    
    # 热参数
    C_c = 50.0           # 核心热容 [J/K]
    C_s = 12.0           # 表面热容 [J/K]
    R_cs = 2.5           # 核心-表面热阻 [K/W]
    R_se = 20.0          # 表面-环境热阻 [K/W]
    dV_dT = 0.0004       # 熵系数 [V/K]
    
    # 充电参数
    I_charge_max = 3.0   # 最大充电电流 [A]
    V_charge = 4.35      # 充电截止电压 [V]


# =============================================================================
# 活动场景定义
# =============================================================================

class ActivityScenarios:
    """活动场景参数定义"""
    
    @staticmethod
    def sleep():
        """睡眠模式 - 最低功耗"""
        return {
            'name': '睡眠',
            'name_en': 'Sleep',
            'color': '#E8F4FD',
            # 5G参数
            '5g_active': True,
            '5g_data_rate': 50e3,      # 50 kbps (仅保持连接)
            '5g_distance': 400,
            # 蓝牙
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 2.0,
            # 后台
            'bg_wake_rate': 0.3,
            'bg_activity': 0.2,
            # GPS
            'gnss_active': False,
            'gnss_environment': 'indoor',
            'gnss_satellites': 0,
            # 显示
            'display_on': False,
            'display_brightness': 0.0,
            'display_refresh': 1,
            'display_content': 'dark',
            # CPU
            'cpu_load': 0.02,
            # 环境
            'T_env': 22.0,
            'signal_strength': -75,     # dBm
        }
    
    @staticmethod
    def commute():
        """通勤模式 - 导航+音乐"""
        return {
            'name': '通勤',
            'name_en': 'Commute',
            'color': '#FFF3E0',
            '5g_active': True,
            '5g_data_rate': 8e6,        # 8 Mbps (音乐流)
            '5g_distance': 800,
            'bt_active': True,
            'bt_streaming': True,       # 蓝牙音频
            'bt_interval': 0.02,
            'bg_wake_rate': 1.5,
            'bg_activity': 0.6,
            'gnss_active': True,        # 导航开启
            'gnss_environment': 'urban',
            'gnss_satellites': 6,
            'display_on': True,
            'display_brightness': 0.7,
            'display_refresh': 60,
            'display_content': 'mixed',
            'cpu_load': 0.45,
            'T_env': 28.0,              # 车内温度
            'signal_strength': -85,     # 移动中信号较弱
        }
    
    @staticmethod
    def office():
        """办公模式 - 邮件+文档"""
        return {
            'name': '办公',
            'name_en': 'Office',
            'color': '#E8F5E9',
            '5g_active': True,
            '5g_data_rate': 3e6,        # 3 Mbps
            '5g_distance': 300,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 0.5,
            'bg_wake_rate': 2.0,
            'bg_activity': 0.7,
            'gnss_active': False,
            'gnss_environment': 'indoor',
            'gnss_satellites': 0,
            'display_on': True,
            'display_brightness': 0.5,
            'display_refresh': 60,
            'display_content': 'text',
            'cpu_load': 0.25,
            'T_env': 24.0,
            'signal_strength': -65,     # 室内信号良好
        }
    
    @staticmethod
    def entertainment():
        """娱乐模式 - 视频/游戏"""
        return {
            'name': '娱乐',
            'name_en': 'Entertainment',
            'color': '#FCE4EC',
            '5g_active': True,
            '5g_data_rate': 25e6,       # 25 Mbps (高清视频)
            '5g_distance': 400,
            'bt_active': True,
            'bt_streaming': True,
            'bt_interval': 0.05,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.4,
            'gnss_active': False,
            'gnss_environment': 'indoor',
            'gnss_satellites': 0,
            'display_on': True,
            'display_brightness': 0.85,
            'display_refresh': 90,
            'display_content': 'video',
            'cpu_load': 0.65,
            'T_env': 25.0,
            'signal_strength': -70,
        }
    
    @staticmethod
    def charging():
        """充电状态"""
        return {
            'name': '充电',
            'name_en': 'Charging',
            'color': '#C8E6C9',
            'is_charging': True,
            '5g_active': True,
            '5g_data_rate': 1e6,
            '5g_distance': 300,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 1.0,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.5,
            'gnss_active': False,
            'gnss_environment': 'indoor',
            'gnss_satellites': 0,
            'display_on': False,
            'display_brightness': 0.0,
            'display_refresh': 60,
            'display_content': 'dark',
            'cpu_load': 0.1,
            'T_env': 25.0,
            'signal_strength': -65,
        }


def create_daily_schedule():
    """创建24小时活动时间表"""
    # 格式: (开始时间小时, 结束时间小时, 场景函数)
    schedule = [
        (0.0, 2.0, ActivityScenarios.sleep),       # 睡眠
        (2.0, 2.5, ActivityScenarios.charging),    # 夜间充电
        (2.5, 7.0, ActivityScenarios.sleep),       # 继续睡眠
        (7.0, 8.0, ActivityScenarios.commute),     # 早通勤
        (8.0, 10.0, ActivityScenarios.office),     # 上午办公
        (10.0, 11.0, ActivityScenarios.entertainment),  # 休息娱乐
        (11.0, 12.5, ActivityScenarios.office),    # 午前办公
        (12.5, 13.0, ActivityScenarios.charging),  # 午餐充电
        (13.0, 14.0, ActivityScenarios.entertainment),  # 午休
        (14.0, 17.0, ActivityScenarios.office),    # 下午办公
        (17.0, 18.0, ActivityScenarios.commute),   # 晚通勤
        (18.0, 20.0, ActivityScenarios.entertainment),  # 晚间娱乐
        (20.0, 21.0, ActivityScenarios.office),    # 晚间工作
        (21.0, 22.0, ActivityScenarios.entertainment),  # 睡前娱乐
        (22.0, 22.5, ActivityScenarios.charging),  # 睡前充电
        (22.5, 24.0, ActivityScenarios.sleep),     # 睡眠
    ]
    return schedule


# =============================================================================
# 耦合模型核心
# =============================================================================

class CoupledBatteryModel:
    """
    耦合电-热-老化-负载模型
    
    状态向量: x = [z, V1, V2, Tc, Ts]
    其中:
        z  - 荷电状态 SOC ∈ [0, 1]
        V1 - 电化学极化电压 [V]
        V2 - 浓差极化电压 [V]
        Tc - 核心温度 [°C]
        Ts - 表面温度 [°C]
    """
    
    def __init__(self, cycle_number=150):
        self.params = BatteryParameters()
        self.cycle_number = cycle_number
        self._update_aging_params()
    
    def _update_aging_params(self):
        """更新老化参数"""
        N = self.cycle_number
        p = self.params
        
        # 容量衰减
        self.Q_max_base = (p.a_Q * np.exp(-p.b_Q * N) + 
                          p.c_Q * np.exp(-p.d_Q * N))
        
        # 阻抗增长
        self.R_total_base = (p.a_R * np.power(max(N, 1), p.b_R) + p.c_R)
    
    def capacity_temp_factor(self, T):
        """温度容量修正因子 S_Q(T)"""
        p = self.params
        return p.S_Q_max / (1 + np.exp(-p.k_Q * (T - p.T_0)))
    
    def resistance_temp_factor(self, T):
        """温度阻抗修正因子 S_R(T)"""
        p = self.params
        return p.C_R + p.A_R * np.exp(-p.B_R * T)
    
    def get_capacity(self, Tc):
        """获取温度修正后的容量 Q_max(N, T)"""
        return self.Q_max_base * self.capacity_temp_factor(Tc)
    
    def get_resistance(self, Tc):
        """获取温度修正后的总内阻 R_total(N, T)"""
        return self.R_total_base * self.resistance_temp_factor(Tc)
    
    def ocv(self, z):
        """开路电压 V_OCV(z) - 能斯特组合模型"""
        z = np.clip(z, 1e-6, 1 - 1e-6)
        K = self.params.K
        return (K[0] + K[1]*z + K[2]/z + 
                K[3]*np.log(z) + K[4]*np.log(1-z))
    
    def load_power(self, profile, V_bat, Tc):
        """计算负载功率 P_load(t)"""
        p = profile
        
        # 5G功率
        rate_ratio = np.clip(p['5g_data_rate'] / 100e6, 0, 8)
        snr = 2**rate_ratio - 1
        P_tx = 1e-9 * (p['5g_distance'] ** 3.2) * snr
        P_tx = np.clip(P_tx, 0, 2.0)
        P_5g = 0.15 + P_tx / 0.35 if p['5g_active'] else 0.001
        
        # 蓝牙功率
        if p['bt_streaming']:
            P_bt = 0.035
        elif p['bt_active']:
            P_bt = V_bat * (5e-6 + 50e-6 / p['bt_interval'])
        else:
            P_bt = 0
        
        # 后台功率
        duty = p['bg_wake_rate'] * 10.5 / 60
        duty = np.clip(duty, 0, 1)
        P_bg = 0.005 + duty * 0.08 + (1 - duty) * 0.015
        
        # GNSS功率
        if p['gnss_active']:
            lock_prob = 1 / (1 + np.exp(-10 * (p['gnss_satellites']/12 * 0.7 - 0.3)))
            P_gnss = lock_prob * 0.04 + (1 - lock_prob) * 0.15 + 0.01
        else:
            P_gnss = 0
        
        # 显示功率
        if p['display_on']:
            apl_map = {'dark': 0.1, 'text': 0.25, 'mixed': 0.4, 'video': 0.55}
            apl = apl_map.get(p['display_content'], 0.4)
            P_disp = (0.08 + 0.004 * p['display_refresh'] + 
                      2.2 * (p['display_brightness'] ** 1.4) * apl)
        else:
            P_disp = 0.008
        
        # SoC功率
        f = 3e8 + (3e9 - 3e8) * p['cpu_load']
        P_dyn = 1.2e-28 * (f ** 3)
        T_k = Tc + 273.15
        I_leak = 0.004 * (T_k / 298) ** 2 * np.exp(0.015 * (Tc - 25))
        I_leak = np.clip(I_leak, 0, 0.3)
        P_soc = P_dyn + 1.0 * I_leak
        
        P_total = P_5g + P_bt + P_bg + P_gnss + P_disp + P_soc
        
        return P_total, {
            '5G': P_5g, 'BT': P_bt, 'BG': P_bg, 
            'GNSS': P_gnss, 'Display': P_disp, 'SoC': P_soc
        }
    
    def state_derivatives(self, t, state, profile):
        """
        计算状态导数 dx/dt
        
        连续时间微分方程组:
        ┌─────────────────────────────────────────────────────────────┐
        │  dz/dt = -I(t)·η(Tc) / (Q_max(N,Tc)·3600)                  │
        │  dV1/dt = -V1/(R1·C1) + I(t)/C1                            │
        │  dV2/dt = -V2/(R2·C2) + I(t)/C2                            │
        │  dTc/dt = (Q_gen - (Tc-Ts)/R_cs) / C_c                     │
        │  dTs/dt = ((Tc-Ts)/R_cs - (Ts-Tenv)/R_se) / C_s            │
        └─────────────────────────────────────────────────────────────┘
        """
        z, V1, V2, Tc, Ts = state
        z = np.clip(z, 0.01, 0.99)
        p = self.params
        
        # 获取温度相关参数
        Q_max = self.get_capacity(Tc)
        R_total = self.get_resistance(Tc)
        
        R0 = R_total * p.R0_ratio
        R1 = R_total * p.R1_ratio
        R2 = R_total * p.R2_ratio
        C1 = p.tau_1 / R1 if R1 > 0 else 1
        C2 = p.tau_2 / R2 if R2 > 0 else 1
        
        # 计算电流
        V_ocv = self.ocv(z)
        is_charging = profile.get('is_charging', False)
        
        if is_charging and z < 0.95:
            # CC-CV充电
            I_charge = min(p.I_charge_max, (p.V_charge - V_ocv) / R_total)
            I_charge = max(I_charge, 0.1)
            I = -I_charge  # 充电为负电流
        else:
            # 放电
            V_term_est = V_ocv - V1 - V2
            P_load, _ = self.load_power(profile, V_term_est, Tc)
            I = P_load / max(V_term_est, 3.0)
        
        # 库伦效率
        eta = 0.85 + 0.148 / (1 + np.exp(-0.1 * (Tc + 5)))
        
        # 状态方程
        dz_dt = -I * eta / (Q_max * 3600)
        dV1_dt = -V1 / (R1 * C1) + I / C1
        dV2_dt = -V2 / (R2 * C2) + I / C2
        
        # 热生成 (Bernardi方程)
        Q_gen = I**2 * R_total + abs(I) * (Tc + 273.15) * p.dV_dT
        
        # 热动力学
        T_env = profile.get('T_env', 25.0)
        dTc_dt = (Q_gen - (Tc - Ts) / p.R_cs) / p.C_c
        dTs_dt = ((Tc - Ts) / p.R_cs - (Ts - T_env) / p.R_se) / p.C_s
        
        return [dz_dt, dV1_dt, dV2_dt, dTc_dt, dTs_dt]
    
    def terminal_voltage(self, state, profile):
        """计算端电压"""
        z, V1, V2, Tc, Ts = state
        z = np.clip(z, 0.01, 0.99)
        
        R_total = self.get_resistance(Tc)
        R0 = R_total * self.params.R0_ratio
        V_ocv = self.ocv(z)
        
        is_charging = profile.get('is_charging', False)
        if is_charging and z < 0.95:
            I = -min(self.params.I_charge_max, 
                     (self.params.V_charge - V_ocv) / R_total)
        else:
            V_est = V_ocv - V1 - V2
            P_load, _ = self.load_power(profile, V_est, Tc)
            I = P_load / max(V_est, 3.0)
        
        return V_ocv - V1 - V2 - I * R0, I


def run_daily_simulation(model, schedule, dt=30.0):
    """
    运行24小时仿真
    
    参数:
        model: CoupledBatteryModel实例
        schedule: 活动时间表
        dt: 时间步长 [s]
    
    返回:
        results: 仿真结果字典
    """
    # 初始状态
    state = [0.85, 0.0, 0.0, 22.0, 22.0]  # z, V1, V2, Tc, Ts
    
    # 结果存储
    t_all = []
    soc_all = []
    current_all = []
    voltage_all = []
    temp_all = []
    brightness_all = []
    signal_all = []
    power_all = []
    scenario_all = []
    
    t = 0
    t_end = 24 * 3600  # 24小时
    
    while t < t_end:
        # 确定当前场景
        t_hours = t / 3600
        current_scenario = None
        for start_h, end_h, scenario_func in schedule:
            if start_h <= t_hours < end_h:
                current_scenario = scenario_func()
                break
        
        if current_scenario is None:
            current_scenario = ActivityScenarios.sleep()
        
        # 计算状态导数并更新 (欧拉法)
        dstate = model.state_derivatives(t, state, current_scenario)
        state = [state[i] + dstate[i] * dt for i in range(5)]
        
        # 约束SOC
        state[0] = np.clip(state[0], 0.01, 0.99)
        
        # 计算输出
        V_term, I = model.terminal_voltage(state, current_scenario)
        P_load, _ = model.load_power(current_scenario, V_term, state[3])
        
        # 存储结果
        t_all.append(t / 3600)
        soc_all.append(state[0] * 100)
        current_all.append(I)
        voltage_all.append(V_term)
        temp_all.append(state[3])
        brightness_all.append(current_scenario['display_brightness'] * 100)
        signal_all.append(current_scenario['signal_strength'])
        power_all.append(P_load)
        scenario_all.append(current_scenario)
        
        t += dt
    
    return {
        't': np.array(t_all),
        'soc': np.array(soc_all),
        'current': np.array(current_all),
        'voltage': np.array(voltage_all),
        'temperature': np.array(temp_all),
        'brightness': np.array(brightness_all),
        'signal': np.array(signal_all),
        'power': np.array(power_all),
        'scenarios': scenario_all,
    }


def plot_daily_results(results, schedule, save_path=None):
    """
    绘制24小时仿真结果 - 类似目标图片风格
    """
    fig, ax1 = plt.subplots(figsize=(14, 7))
    
    t = results['t']
    
    # 绘制场景背景色带
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        ax1.axvspan(start_h, end_h, alpha=0.3, 
                   color=scenario['color'], zorder=0)
    
    # 添加场景标签
    labeled_scenarios = set()
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        mid = (start_h + end_h) / 2
        name = scenario['name']
        if name not in labeled_scenarios:
            ax1.text(mid, 102, name, ha='center', va='bottom', 
                    fontsize=9, fontweight='bold')
            labeled_scenarios.add(name)
        elif end_h - start_h > 1.5:
            ax1.text(mid, 102, name, ha='center', va='bottom', 
                    fontsize=9, fontweight='bold')
    
    # 主Y轴: SOC和电流
    line_soc, = ax1.plot(t, results['soc'], 'darkorange', linewidth=2.5, 
                         label='SOC [%]', zorder=5)
    
    # 电流 (放大显示)
    current_scaled = np.abs(results['current']) * 30  # 放大倍数
    line_current, = ax1.plot(t, current_scaled, 'steelblue', linewidth=1.2, 
                             alpha=0.8, label='电流 I [A×30]', zorder=4)
    
    ax1.set_xlabel('时间 (小时)', fontsize=12)
    ax1.set_ylabel('SOC (%) / 电流 (A)', fontsize=12, color='darkorange')
    ax1.tick_params(axis='y', labelcolor='darkorange')
    ax1.set_xlim([0, 24])
    ax1.set_ylim([0, 105])
    ax1.set_xticks(np.arange(0, 25, 3))
    ax1.set_xticklabels([f'{int(h)}:00' for h in np.arange(0, 25, 3)])
    ax1.grid(True, alpha=0.2)
    
    # 副Y轴: 温度、亮度、信号强度
    ax2 = ax1.twinx()
    
    # 温度
    line_temp, = ax2.plot(t, results['temperature'], 'purple', 
                         linewidth=1.5, alpha=0.9, label='温度 T [°C]', zorder=3)
    
    # 亮度
    line_bright, = ax2.plot(t, results['brightness'], 'goldenrod', 
                           linewidth=1.2, alpha=0.8, label='亮度 [%]', zorder=3)
    
    # 信号强度 (归一化到0-100)
    signal_norm = (results['signal'] + 100) * 1.2  # 归一化
    line_signal, = ax2.plot(t, signal_norm, 'forestgreen', 
                           linewidth=1.0, alpha=0.7, label='信号 [dBm]', zorder=2)
    
    ax2.set_ylabel('温度 (°C) / 亮度 (%) / 信号强度 [dBm]', fontsize=11)
    ax2.set_ylim([0, 120])
    
    # 标注充电区间
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        if scenario.get('is_charging', False) or scenario['name'] == '充电':
            mid = (start_h + end_h) / 2
            ax1.annotate('充电', xy=(mid, 20), fontsize=8, 
                        ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.3', 
                                 facecolor='lightgreen', alpha=0.8))
    
    # 图例
    lines = [line_soc, line_current, line_temp, line_bright, line_signal]
    labels = ['SOC [%]', '电流 I [A×30]', '温度 T [°C]', 
              '亮度 [%]', '信号强度 [dBm]']
    ax1.legend(lines, labels, loc='upper right', fontsize=9, 
              framealpha=0.9, ncol=2)
    
    plt.title('24小时智能手机电池耦合模型仿真', fontsize=14, fontweight='bold', pad=15)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'图片已保存: {save_path}')
    
    return fig


def print_model_equations_chinese():
    """打印中文版耦合模型方程"""
    equations = """
╔══════════════════════════════════════════════════════════════════════════════╗
║           耦合电-热-老化-负载 连续时间数学模型                                  ║
║        Coupled Electro-Thermal-Aging-Load Continuous-Time Model              ║
╚══════════════════════════════════════════════════════════════════════════════╝

┌──────────────────────────────────────────────────────────────────────────────┐
│ 1. 状态变量定义 (State Variables)                                             │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   x = [z, V₁, V₂, Tc, Ts]ᵀ                                                  │
│                                                                              │
│   其中:                                                                      │
│     z   - 荷电状态 SOC ∈ [0, 1]                                              │
│     V₁  - 电化学极化电压 [V]                                                  │
│     V₂  - 浓差极化电压 [V]                                                    │
│     Tc  - 电池核心温度 [°C]                                                   │
│     Ts  - 电池表面温度 [°C]                                                   │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 2. 容量衰减模型 (Capacity Fade Model) - 双指数函数                            │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Qmax(N) = aQ·exp(-bQ·N) + cQ·exp(-dQ·N)                                    │
│                                                                              │
│   温度修正:                                                                   │
│                      SQ,max                                                  │
│   Qmax(N,T) = Qmax(N) · ────────────────────                                 │
│                      1 + exp(-kQ·(T - T₀))                                   │
│                                                                              │
│   参数: aQ=-0.15, bQ=0.025, cQ=4.15, dQ=0.0008                               │
│         SQ,max=1.02, kQ=0.1, T₀=-12°C                                        │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 3. 阻抗增长模型 (Impedance Growth Model) - 幂律函数                           │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Rtotal(N) = aR·N^bR + cR                                                   │
│                                                                              │
│   温度修正 (阿伦尼乌斯定律):                                                  │
│   Rtotal(N,T) = Rtotal(N) · (CR + AR·exp(-BR·T))                             │
│                                                                              │
│   参数: aR=0.008, bR=0.22, cR=0.025                                          │
│         CR=0.75, AR=1.2, BR=0.055                                            │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 4. 开路电压模型 (OCV Model) - 能斯特组合方程                                  │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   VOCV(z) = K₀ + K₁·z + K₂/z + K₃·ln(z) + K₄·ln(1-z)                        │
│                                                                              │
│   参数: K₀=3.45V, K₁=0.18V, K₂=-0.005V, K₃=0.06V, K₄=-0.09V                 │
│                                                                              │
│   物理意义:                                                                   │
│     - K₂/z 项: 放电末端电压骤降                                               │
│     - ln(z) 项: 电化学平衡熵效应                                              │
│     - ln(1-z) 项: 满充状态非线性                                              │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 5. 二阶RC等效电路动力学 (2nd-Order Thevenin ECM)                              │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   电化学极化回路:                                                             │
│         dV₁      V₁      I(t)                                                │
│        ──── = - ──── + ────                                                  │
│         dt     R₁C₁     C₁                                                   │
│                                                                              │
│   浓差极化回路:                                                               │
│         dV₂      V₂      I(t)                                                │
│        ──── = - ──── + ────                                                  │
│         dt     R₂C₂     C₂                                                   │
│                                                                              │
│   端电压输出方程:                                                             │
│        Vterm(t) = VOCV(z) - V₁(t) - V₂(t) - I(t)·R₀                         │
│                                                                              │
│   参数: τ₁=R₁C₁=25s (电化学极化), τ₂=R₂C₂=250s (浓差极化)                   │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 6. 荷电状态动力学 (SOC Dynamics) - 安时积分法                                 │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│         dz        I(t) · η(Tc)                                               │
│        ──── = - ─────────────────                                            │
│         dt      Qmax(N,Tc) · 3600                                            │
│                                                                              │
│   库伦效率:                                                                   │
│                        0.148                                                 │
│        η(Tc) = 0.85 + ────────────────────                                   │
│                      1 + exp(-0.1·(Tc + 5))                                  │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 7. 双状态热动力学模型 (Two-State Thermal Model)                               │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   产热方程 (Bernardi):                                                        │
│        Qgen = I²·Rtotal + |I|·(Tc + 273.15)·(∂VOCV/∂T)                       │
│              ╰───────╯   ╰─────────────────────────────╯                     │
│              焦耳热(不可逆)      熵热(可逆)                                    │
│                                                                              │
│   核心温度:                                                                   │
│            dTc     1  ⎛            Tc - Ts  ⎞                                │
│           ──── = ─── ⎜ Qgen  -  ────────── ⎟                                │
│            dt    Cc  ⎝            Rcs      ⎠                                │
│                                                                              │
│   表面温度:                                                                   │
│            dTs     1  ⎛  Tc - Ts     Ts - Tenv  ⎞                            │
│           ──── = ─── ⎜ ────────  - ──────────  ⎟                            │
│            dt    Cs  ⎝   Rcs         Rse       ⎠                            │
│                                                                              │
│   参数: Cc=50 J/K, Cs=12 J/K, Rcs=2.5 K/W, Rse=20 K/W                        │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 8. 多物理场负载功率模型 (Multi-Physics Load Models)                           │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ◆ 5G通信 (Shannon-Friis):                                                  │
│                              Λenv · d^n · (2^(R/B) - 1)                      │
│        P5G(t) = Pstatic + αbb·R(t) + ─────────────────────                   │
│                                            ηPA                               │
│                                                                              │
│   ◆ 蓝牙/BLE (离散事件积分):                                                  │
│                            Qevent(L)                                         │
│        IBLE(τ) ≈ Isleep + ───────────                                        │
│                               τ                                              │
│                                                                              │
│   ◆ GNSS导航 (双模状态机):                                                    │
│        PGNSS(t) = xlock·Ptrack + (1-xlock)·Pacq + PLNA                       │
│                                                                              │
│        dxlock     Ψ(Senv) - xlock                                            │
│        ────── = ─────────────────                                            │
│          dt          τreact                                                  │
│                                                                              │
│   ◆ OLED显示 (内容感知+LTPO):                                                 │
│        Pdisp(t) = Pbase + kdrv·frefresh(t) + βpanel·Θ(Lset)·APL(t)          │
│                                                                              │
│   ◆ SoC处理器 (DVFS+热耦合):                                                  │
│        PSoC(t) = κdvfs·f(t)³ + Vdd·Ileak(TSoC)                               │
│                  ╰─────────╯   ╰─────────────╯                               │
│                   动态功耗        静态漏电                                     │
│                                                                              │
│   总负载功率:                                                                 │
│        Pload(t) = P5G + PBT + Pbg + PGNSS + Pdisp + PSoC                     │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 9. 电-热-负载耦合方程 (Electro-Thermal-Load Coupling)                         │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   自洽电流-电压关系:                                                          │
│                                                                              │
│        I(t) = Pload(t, Vterm) / Vterm(t)                                     │
│                                                                              │
│        Vterm(t) = VOCV(z) - V₁(t) - V₂(t) - I(t)·R₀(Tc)                     │
│                                                                              │
│   闭环反馈机制:                                                               │
│                                                                              │
│        电流 ──→ 焦耳热 ──→ 温度↑ ──→ 内阻↓ ──→ 电压↑ ──→ 电流↓              │
│          │                                                                   │
│          └──────────── 温度↑ ──→ 漏电↑ ──→ 功耗↑ ─────────────┘              │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ 10. 完整状态空间表示 (Complete State-Space Representation)                    │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌     ┐   ┌                                                          ┐    │
│   │ dz  │   │  -I(t)·η(Tc) / (Qmax(N,Tc)·3600)                        │    │
│   │ dt  │   │                                                          │    │
│   │     │   │                                                          │    │
│   │ dV₁ │   │  -V₁/(R₁C₁) + I(t)/C₁                                   │    │
│   │ dt  │   │                                                          │    │
│   │     │   │                                                          │    │
│   │ dV₂ │ = │  -V₂/(R₂C₂) + I(t)/C₂                                   │    │
│   │ dt  │   │                                                          │    │
│   │     │   │                                                          │    │
│   │ dTc │   │  (Qgen - (Tc-Ts)/Rcs) / Cc                               │    │
│   │ dt  │   │                                                          │    │
│   │     │   │                                                          │    │
│   │ dTs │   │  ((Tc-Ts)/Rcs - (Ts-Tenv)/Rse) / Cs                      │    │
│   │ dt  │   │                                                          │    │
│   └     ┘   └                                                          ┘    │
│                                                                              │
│   输出方程:                                                                   │
│        y(t) = [SOC(t), Vterm(t), Tc(t), I(t)]ᵀ                              │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
"""
    print(equations)


# =============================================================================
# 主程序
# =============================================================================

if __name__ == "__main__":
    import os
    
    print("="*70)
    print("  24小时智能手机电池耦合模型仿真")
    print("  24-Hour Smartphone Battery Coupled Model Simulation")
    print("="*70)
    
    # 打印模型方程
    print_model_equations_chinese()
    
    # 创建模型
    print("\n初始化耦合电-热-老化模型...")
    model = CoupledBatteryModel(cycle_number=150)
    
    # 创建时间表
    schedule = create_daily_schedule()
    
    # 运行仿真
    print("运行24小时仿真...")
    results = run_daily_simulation(model, schedule, dt=30.0)
    
    # 输出统计
    print("\n" + "="*50)
    print("仿真结果统计")
    print("="*50)
    print(f"  初始SOC: {results['soc'][0]:.1f}%")
    print(f"  最终SOC: {results['soc'][-1]:.1f}%")
    print(f"  最低SOC: {min(results['soc']):.1f}%")
    print(f"  最高SOC: {max(results['soc']):.1f}%")
    print(f"  平均功率: {np.mean(results['power'])*1000:.1f} mW")
    print(f"  峰值功率: {max(results['power'])*1000:.1f} mW")
    print(f"  温度范围: {min(results['temperature']):.1f}°C - {max(results['temperature']):.1f}°C")
    print(f"  总能量消耗: {np.trapz(results['power'], results['t']*3600)/3600:.2f} Wh")
    
    # 绘制结果
    print("\n生成可视化图表...")
    os.makedirs('output/results', exist_ok=True)
    fig = plot_daily_results(results, schedule, 'output/results/daily_simulation_24h.png')
    
    print("\n仿真完成!")
