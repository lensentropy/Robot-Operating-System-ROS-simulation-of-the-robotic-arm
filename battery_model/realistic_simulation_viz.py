"""
基于严格微分方程的真实仿真可视化
Realistic Simulation Visualization Based on Rigorous Differential Equations

生成与用户示例相似的高保真图形:
1. Display Dynamics (APL & Brightness)
2. Processor Dynamics (Util & Freq)
3. Battery Drain (SOC & Power)
4. Power Composition (堆叠图)

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from dataclasses import dataclass
from typing import Tuple, List, Dict, Callable
import warnings

# 设置绘图风格
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['axes.facecolor'] = 'white'
plt.rcParams['axes.grid'] = False


# ==============================================================================
# 完整物理参数
# ==============================================================================

@dataclass
class FullBatteryParams:
    """完整电池参数"""
    Q_max: float = 4000.0          # mAh
    V_nom: float = 3.85            # V
    R_int_ref: float = 0.08        # Ohm
    C_th: float = 15.0             # J/K
    R_th: float = 8.0              # K/W
    T_amb: float = 298.15          # K
    eta_pmic: float = 0.92
    
    # OCV多项式系数 (5阶)
    ocv_coeffs: tuple = (3.0, 1.2, -0.8, 1.5, -1.2, 0.3)
    
    # 熵系数
    entropy_coeffs: tuple = (-0.0005, 0.001, -0.0008)


@dataclass
class FullSoCParams:
    """完整SoC芯片参数"""
    C_eff: float = 2.5e-9          # F
    V_dd_min: float = 0.6          # V
    V_dd_max: float = 1.1          # V
    f_min: float = 0.3e9           # Hz
    f_max: float = 3.0e9           # Hz
    gamma: float = 1.2             # 速度饱和指数
    
    # 漏电参数
    I_ref: float = 1e-6            # A
    T_ref: float = 300.0           # K
    lambda_DIBL: float = 70e-3     # V/V
    kappa: float = 2e-3
    n_subth: float = 1.2
    
    # 热参数
    C_th_soc: float = 2.0          # J/K
    R_th_soc: float = 5.0          # K/W


# ==============================================================================
# 高保真硬件状态生成器 (马尔科夫+随机波动)
# ==============================================================================

class RealisticHardwareStateGenerator:
    """
    生成真实感的硬件状态轨迹
    结合马尔科夫链状态转移和高频随机波动
    """
    
    def __init__(self, seed: int = 42):
        np.random.seed(seed)
        
        # 用户状态
        self.states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        self.n_states = 4
        
        # 时间模式转移矩阵
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
        
        # 状态对应的硬件参数范围
        self.state_params = {
            0: {  # Deep Sleep
                'cpu_util': (0, 5), 'cpu_freq': (0.3, 0.4),
                'brightness': (0, 50), 'apl': (0, 10),
                'base_power': 0.3
            },
            1: {  # Light Use
                'cpu_util': (10, 40), 'cpu_freq': (0.8, 1.8),
                'brightness': (200, 600), 'apl': (30, 70),
                'base_power': 0.5
            },
            2: {  # Streaming
                'cpu_util': (20, 50), 'cpu_freq': (1.0, 2.2),
                'brightness': (400, 900), 'apl': (40, 80),
                'base_power': 0.6
            },
            3: {  # Gaming
                'cpu_util': (60, 95), 'cpu_freq': (2.0, 3.2),
                'brightness': (600, 1200), 'apl': (50, 90),
                'base_power': 0.8
            }
        }
    
    def get_time_mode(self, hour: float) -> int:
        """获取时间模式: 0=sleep, 1=work, 2=leisure"""
        hour = hour % 24
        if hour >= 23 or hour < 7:
            return 0  # Sleep
        elif (9 <= hour < 12) or (14 <= hour < 18):
            return 1  # Work
        else:
            return 2  # Leisure
    
    def get_transition_matrix(self, mode: int) -> np.ndarray:
        if mode == 0:
            return self.P_sleep
        elif mode == 1:
            return self.P_work
        else:
            return self.P_leisure
    
    def generate_trajectory(self, start_hour: float, duration_hours: float, 
                           dt_seconds: float = 1.0) -> Dict:
        """
        生成完整的硬件状态轨迹
        
        Parameters:
        -----------
        start_hour : float
            起始时间 (小时)
        duration_hours : float
            持续时间 (小时)
        dt_seconds : float
            时间步长 (秒)
        
        Returns:
        --------
        dict : 包含所有硬件参数时间序列
        """
        n_steps = int(duration_hours * 3600 / dt_seconds)
        
        # 初始化数组
        time_seconds = np.arange(n_steps) * dt_seconds
        time_hours = start_hour + time_seconds / 3600
        
        user_state = np.zeros(n_steps, dtype=int)
        time_mode = np.zeros(n_steps, dtype=int)
        
        cpu_util = np.zeros(n_steps)
        cpu_freq = np.zeros(n_steps)
        brightness = np.zeros(n_steps)
        apl = np.zeros(n_steps)
        base_power = np.zeros(n_steps)
        
        # 初始状态
        current_state = 0  # 从Deep Sleep开始
        
        # O-U过程状态变量
        ou_cpu = 0.0
        ou_brightness = 0.0
        ou_apl = 0.0
        
        theta = 0.1  # O-U回归速率
        sigma_cpu = 5.0
        sigma_brightness = 50.0
        sigma_apl = 5.0
        
        for i in range(n_steps):
            hour = time_hours[i] % 24
            mode = self.get_time_mode(hour)
            time_mode[i] = mode
            
            # 状态转移 (每分钟检查一次)
            if i % 60 == 0:
                P = self.get_transition_matrix(mode)
                probs = P[current_state]
                current_state = np.random.choice(4, p=probs)
            
            user_state[i] = current_state
            
            # 获取状态参数范围
            params = self.state_params[current_state]
            
            # 基础值 (状态中心)
            cpu_util_base = np.mean(params['cpu_util'])
            cpu_freq_base = np.mean(params['cpu_freq'])
            brightness_base = np.mean(params['brightness'])
            apl_base = np.mean(params['apl'])
            
            # O-U过程更新 (添加随机波动)
            dW = np.random.randn() * np.sqrt(dt_seconds)
            ou_cpu += theta * (0 - ou_cpu) * dt_seconds + sigma_cpu * dW
            ou_brightness += theta * (0 - ou_brightness) * dt_seconds + sigma_brightness * np.random.randn() * np.sqrt(dt_seconds)
            ou_apl += theta * (0 - ou_apl) * dt_seconds + sigma_apl * np.random.randn() * np.sqrt(dt_seconds)
            
            # 添加高频噪声
            noise_cpu = np.random.randn() * 3
            noise_freq = np.random.randn() * 0.1
            noise_brightness = np.random.randn() * 30
            noise_apl = np.random.randn() * 3
            
            # 组合: 基础值 + O-U漂移 + 高频噪声
            cpu_util[i] = np.clip(cpu_util_base + ou_cpu + noise_cpu, 
                                  params['cpu_util'][0], params['cpu_util'][1])
            cpu_freq[i] = np.clip(cpu_freq_base + noise_freq,
                                  params['cpu_freq'][0], params['cpu_freq'][1])
            brightness[i] = np.clip(brightness_base + ou_brightness + noise_brightness,
                                    params['brightness'][0], params['brightness'][1])
            apl[i] = np.clip(apl_base + ou_apl + noise_apl,
                            params['apl'][0], params['apl'][1])
            base_power[i] = params['base_power']
            
            # 环境光影响亮度 (日间更亮)
            sunlight = np.exp(-((hour - 13)**2) / 18)
            brightness[i] *= (0.7 + 0.6 * sunlight)
            brightness[i] = np.clip(brightness[i], 0, 1500)
        
        # 平滑处理 (模拟DVFS响应延迟)
        cpu_freq = gaussian_filter1d(cpu_freq, sigma=5)
        
        return {
            'time_seconds': time_seconds,
            'time_hours': time_hours,
            'user_state': user_state,
            'time_mode': time_mode,
            'cpu_util': cpu_util,
            'cpu_freq': cpu_freq,
            'brightness': brightness,
            'apl': apl,
            'base_power': base_power
        }


# ==============================================================================
# 严格微分方程求解器
# ==============================================================================

class RigorousBatterySimulator:
    """
    基于严格微分方程的电池仿真器
    
    状态方程:
    dSOC/dt = -I_total / (Q_eff * η_temp * 3600)
    dT_batt/dt = (P_joule + P_entropy - (T-T_amb)/R_th) / C_th
    dT_soc/dt = (P_soc - (T_soc-T_batt)/R_th_sb - (T_soc-T_amb)/R_th_se) / C_th_soc
    """
    
    def __init__(self):
        self.bp = FullBatteryParams()
        self.sp = FullSoCParams()
    
    def V_OCV(self, SOC: float) -> float:
        """5阶多项式OCV-SOC关系"""
        SOC = np.clip(SOC, 0.0, 1.0)
        V = sum(c * (SOC ** i) for i, c in enumerate(self.bp.ocv_coeffs))
        return np.clip(V, 3.0, 4.35)
    
    def dV_OCV_dT(self, SOC: float) -> float:
        """熵系数"""
        SOC = np.clip(SOC, 0.0, 1.0)
        c = self.bp.entropy_coeffs
        return c[0] + c[1] * SOC + c[2] * SOC**2
    
    def R_int(self, SOC: float, T: float) -> float:
        """内阻模型"""
        SOC = np.clip(SOC, 0.01, 1.0)
        f_soc = 1.0 + 0.5 * (1.0 - SOC)**2
        f_temp = np.exp(-0.02 * (T - 298.15))
        return self.bp.R_int_ref * f_soc * f_temp
    
    def eta_temp(self, T: float) -> float:
        """温度效率"""
        T_c = T - 273.15
        if T_c < 0:
            return 0.7
        elif T_c < 10:
            return 0.7 + 0.03 * T_c
        elif T_c < 45:
            return 1.0
        else:
            return 1.0 - 0.01 * (T_c - 45)
    
    def P_SoC(self, cpu_util: float, cpu_freq: float, T_soc: float) -> float:
        """
        SoC功耗 (完整DVFS + 漏电模型)
        P = α*C_eff*V_dd²*f + V_dd*I_sub
        """
        # DVFS
        f_norm = (cpu_freq - 0.3) / (3.0 - 0.3)
        f_norm = np.clip(f_norm, 0, 1)
        V_dd = self.sp.V_dd_min + (self.sp.V_dd_max - self.sp.V_dd_min) * (f_norm ** (1/self.sp.gamma))
        
        # 动态功耗
        alpha = 0.1 + 0.5 * (cpu_util / 100.0)
        f_hz = cpu_freq * 1e9
        P_dyn = alpha * self.sp.C_eff * (V_dd ** 2) * f_hz
        
        # 漏电功耗 (DIBL + 温度)
        k_B_eV = 8.617e-5
        V_T = k_B_eV * T_soc
        exp_arg = (self.sp.lambda_DIBL * V_dd + self.sp.kappa * (T_soc - self.sp.T_ref)) / (self.sp.n_subth * V_T)
        exp_arg = np.clip(exp_arg, -50, 50)
        I_sub = self.sp.I_ref * (T_soc / self.sp.T_ref)**2 * np.exp(exp_arg)
        P_leak = V_dd * I_sub
        
        return P_dyn + P_leak
    
    def P_Display(self, brightness: float, apl: float) -> float:
        """
        显示功耗 (OLED伽马模型)
        P = P_static + k*L^γ*APL
        """
        if brightness <= 0:
            return 0.0
        
        gamma = 2.2
        P_static = 0.1
        k = 0.002
        
        L_norm = brightness / 1000.0
        APL_norm = apl / 100.0
        
        P = P_static + k * brightness * (APL_norm ** gamma)
        return P
    
    def simulate(self, hw_trajectory: Dict) -> Dict:
        """
        运行完整仿真
        
        使用显式欧拉法求解微分方程组
        """
        n_steps = len(hw_trajectory['time_seconds'])
        dt = hw_trajectory['time_seconds'][1] - hw_trajectory['time_seconds'][0]
        
        # 状态变量
        SOC = np.zeros(n_steps)
        T_batt = np.zeros(n_steps)
        T_soc = np.zeros(n_steps)
        
        # 功耗和电流
        P_soc = np.zeros(n_steps)
        P_display = np.zeros(n_steps)
        P_base = np.zeros(n_steps)
        P_total = np.zeros(n_steps)
        I_batt = np.zeros(n_steps)
        V_batt = np.zeros(n_steps)
        
        # 初始条件
        SOC[0] = 1.0
        T_batt[0] = 298.15
        T_soc[0] = 298.15
        
        for i in range(n_steps):
            # 当前硬件状态
            cpu_util = hw_trajectory['cpu_util'][i]
            cpu_freq = hw_trajectory['cpu_freq'][i]
            brightness = hw_trajectory['brightness'][i]
            apl = hw_trajectory['apl'][i]
            base_pwr = hw_trajectory['base_power'][i]
            
            # 计算功耗
            P_soc[i] = self.P_SoC(cpu_util, cpu_freq, T_soc[i] if i > 0 else 298.15)
            P_display[i] = self.P_Display(brightness, apl)
            P_base[i] = base_pwr
            P_total[i] = P_soc[i] + P_display[i] + P_base[i]
            
            # 电压和电流
            V_ocv = self.V_OCV(SOC[i] if i > 0 else 1.0)
            R_int = self.R_int(SOC[i] if i > 0 else 1.0, T_batt[i] if i > 0 else 298.15)
            
            # 求解电流 (P = V*I = (V_ocv - I*R)*I)
            if V_ocv > 0:
                discriminant = V_ocv**2 - 4 * R_int * P_total[i] / self.bp.eta_pmic
                if discriminant > 0:
                    I_batt[i] = (V_ocv - np.sqrt(discriminant)) / (2 * R_int)
                else:
                    I_batt[i] = V_ocv / (2 * R_int)
            
            V_batt[i] = V_ocv - I_batt[i] * R_int
            
            if i < n_steps - 1:
                # 微分方程积分
                
                # 1. SOC方程
                Q_ah = self.bp.Q_max / 1000.0
                eta_t = self.eta_temp(T_batt[i])
                dSOC_dt = -I_batt[i] / (Q_ah * eta_t * 3600)
                SOC[i+1] = np.clip(SOC[i] + dSOC_dt * dt, 0.0, 1.0)
                
                # 2. 电池温度方程
                P_joule = I_batt[i]**2 * R_int
                P_entropy = I_batt[i] * T_batt[i] * self.dV_OCV_dT(SOC[i])
                dT_batt_dt = (P_joule + P_entropy - (T_batt[i] - self.bp.T_amb) / self.bp.R_th) / self.bp.C_th
                T_batt[i+1] = T_batt[i] + dT_batt_dt * dt
                
                # 3. SoC温度方程
                dT_soc_dt = (P_soc[i] - (T_soc[i] - T_batt[i]) / self.sp.R_th_soc - 
                            (T_soc[i] - self.bp.T_amb) / 15.0) / self.sp.C_th_soc
                T_soc[i+1] = T_soc[i] + dT_soc_dt * dt
        
        return {
            'SOC': SOC,
            'T_batt': T_batt,
            'T_soc': T_soc,
            'P_soc': P_soc,
            'P_display': P_display,
            'P_base': P_base,
            'P_total': P_total,
            'I_batt': I_batt,
            'V_batt': V_batt
        }


# ==============================================================================
# 可视化 (类似用户示例的风格)
# ==============================================================================

def create_realistic_visualization(hw_traj: Dict, sim_result: Dict, 
                                   save_path: str = None):
    """
    创建类似用户示例的可视化
    """
    # 提取数据
    time_hours = hw_traj['time_hours']
    time_mode = hw_traj['time_mode']
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.patch.set_facecolor('white')
    
    # 定义时间模式颜色
    mode_colors = {0: '#90EE90', 1: '#F0E68C', 2: '#FFA500'}  # 绿、黄、橙
    
    def add_time_mode_background(ax, time_h, modes):
        """添加时间模式背景色块"""
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        # 找出模式变化点
        mode_changes = [0]
        for i in range(1, len(modes)):
            if modes[i] != modes[i-1]:
                mode_changes.append(i)
        mode_changes.append(len(modes)-1)
        
        for j in range(len(mode_changes)-1):
            start_idx = mode_changes[j]
            end_idx = mode_changes[j+1]
            mode = modes[start_idx]
            
            # 在底部绘制色条
            height = (ylim[1] - ylim[0]) * 0.03
            ax.axhspan(ylim[0], ylim[0] + height, 
                      xmin=(time_h[start_idx] - xlim[0])/(xlim[1]-xlim[0]),
                      xmax=(time_h[end_idx] - xlim[0])/(xlim[1]-xlim[0]),
                      color=mode_colors[mode], alpha=0.8)
    
    # -------------------------------------------------------------------------
    # 子图1: Display Dynamics (APL & Brightness)
    # -------------------------------------------------------------------------
    ax1 = axes[0, 0]
    
    # APL (左Y轴)
    color_apl = '#1E90FF'
    ax1.plot(time_hours, hw_traj['apl'], color=color_apl, linewidth=0.5, alpha=0.8)
    ax1.set_ylabel('APL (%)', color=color_apl, fontsize=12)
    ax1.tick_params(axis='y', labelcolor=color_apl)
    ax1.set_ylim(0, 100)
    
    # Brightness (右Y轴, 填充)
    ax1_twin = ax1.twinx()
    color_bright = '#DAA520'
    ax1_twin.fill_between(time_hours, 0, hw_traj['brightness'], 
                          color=color_bright, alpha=0.4, label='Brightness')
    ax1_twin.plot(time_hours, hw_traj['brightness'], color=color_bright, 
                  linewidth=0.3, alpha=0.6)
    ax1_twin.set_ylabel('Brightness (nits)', color=color_bright, fontsize=12)
    ax1_twin.tick_params(axis='y', labelcolor=color_bright)
    ax1_twin.set_ylim(0, 1500)
    
    ax1.set_xlabel('Time of Day (h)', fontsize=11)
    ax1.set_title('Display Dynamics', fontsize=14, fontweight='bold')
    ax1.set_xlim(time_hours[0], time_hours[-1])
    
    # 添加图例
    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], color=color_apl, linewidth=2, label='APL'),
                      mpatches.Patch(facecolor=color_bright, alpha=0.4, label='Brightness')]
    ax1.legend(handles=legend_elements, loc='upper left', fontsize=10)
    
    # -------------------------------------------------------------------------
    # 子图2: Processor Dynamics (Util & Freq)
    # -------------------------------------------------------------------------
    ax2 = axes[0, 1]
    
    # CPU Utilization (左Y轴)
    color_util = '#228B22'
    ax2.plot(time_hours, hw_traj['cpu_util'], color=color_util, linewidth=0.5, alpha=0.8)
    ax2.set_ylabel('Util (%)', color=color_util, fontsize=12)
    ax2.tick_params(axis='y', labelcolor=color_util)
    ax2.set_ylim(0, 100)
    
    # CPU Frequency (右Y轴)
    ax2_twin = ax2.twinx()
    color_freq = '#8B0000'
    ax2_twin.plot(time_hours, hw_traj['cpu_freq'], color=color_freq, linewidth=0.5, alpha=0.8)
    ax2_twin.set_ylabel('Freq (GHz)', color=color_freq, fontsize=12)
    ax2_twin.tick_params(axis='y', labelcolor=color_freq)
    ax2_twin.set_ylim(0, 3.5)
    
    ax2.set_xlabel('Time of Day (h)', fontsize=11)
    ax2.set_title('Processor Dynamics', fontsize=14, fontweight='bold')
    ax2.set_xlim(time_hours[0], time_hours[-1])
    
    legend_elements = [Line2D([0], [0], color=color_util, linewidth=2, label='Util'),
                      Line2D([0], [0], color=color_freq, linewidth=2, label='Freq')]
    ax2.legend(handles=legend_elements, loc='upper left', fontsize=10)
    
    # -------------------------------------------------------------------------
    # 子图3: Battery Drain (SOC & Power)
    # -------------------------------------------------------------------------
    ax3 = axes[1, 0]
    
    # SOC (左Y轴)
    color_soc = '#006400'
    ax3.plot(time_hours, sim_result['SOC'] * 100, color=color_soc, linewidth=2.5)
    ax3.set_ylabel('SoC (%)', color=color_soc, fontsize=12)
    ax3.tick_params(axis='y', labelcolor=color_soc)
    ax3.set_ylim(0, 105)
    
    # Power (右Y轴)
    ax3_twin = ax3.twinx()
    color_power = '#CD853F'
    ax3_twin.plot(time_hours, sim_result['P_total'] * 1000, color=color_power, 
                  linewidth=0.5, alpha=0.7)
    ax3_twin.set_ylabel('Power (mW)', color=color_power, fontsize=12)
    ax3_twin.tick_params(axis='y', labelcolor=color_power)
    ax3_twin.set_ylim(0, max(sim_result['P_total'] * 1000) * 1.2)
    
    ax3.set_xlabel('Time of Day (h)', fontsize=11)
    ax3.set_title('Battery Drain', fontsize=14, fontweight='bold')
    ax3.set_xlim(time_hours[0], time_hours[-1])
    
    legend_elements = [Line2D([0], [0], color=color_soc, linewidth=2, label='SoC'),
                      Line2D([0], [0], color=color_power, linewidth=1, label='Power')]
    ax3.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    # -------------------------------------------------------------------------
    # 子图4: Power Composition (堆叠图)
    # -------------------------------------------------------------------------
    ax4 = axes[1, 1]
    
    # 转换为mW
    P_base_mw = sim_result['P_base'] * 1000
    P_display_mw = sim_result['P_display'] * 1000
    P_soc_mw = sim_result['P_soc'] * 1000
    
    # 堆叠面积图
    ax4.stackplot(time_hours, P_base_mw, P_display_mw, P_soc_mw,
                  labels=['Base/Net', 'Display', 'SoC'],
                  colors=['#808080', '#DAA520', '#CD5C5C'],
                  alpha=0.8)
    
    # 添加总功耗线
    ax4.plot(time_hours, sim_result['P_total'] * 1000, 'k-', linewidth=0.5, alpha=0.5)
    
    ax4.set_xlabel('Time of Day (h)', fontsize=11)
    ax4.set_ylabel('Power (mW)', fontsize=12)
    ax4.set_title('Power Composition', fontsize=14, fontweight='bold')
    ax4.set_xlim(time_hours[0], time_hours[-1])
    ax4.set_ylim(0, max(sim_result['P_total'] * 1000) * 1.1)
    ax4.legend(loc='upper right', fontsize=10)
    
    # -------------------------------------------------------------------------
    # 为每个子图添加时间模式色条
    # -------------------------------------------------------------------------
    for ax in [ax1, ax2, ax3, ax4]:
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        # 绘制底部色条
        prev_mode = time_mode[0]
        start_h = time_hours[0]
        
        for i in range(1, len(time_mode)):
            if time_mode[i] != prev_mode or i == len(time_mode) - 1:
                end_h = time_hours[i]
                color = mode_colors[prev_mode]
                
                # 底部色条
                bar_height = (ylim[1] - ylim[0]) * 0.025
                ax.fill_between([start_h, end_h], ylim[0], ylim[0] + bar_height,
                              color=color, alpha=0.9, zorder=5)
                
                prev_mode = time_mode[i]
                start_h = time_hours[i]
        
        ax.set_ylim(ylim)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to {save_path}")
    
    return fig


def create_extended_visualization(hw_traj: Dict, sim_result: Dict, 
                                  save_path: str = None):
    """
    创建扩展可视化 (6个子图)
    """
    time_hours = hw_traj['time_hours']
    time_mode = hw_traj['time_mode']
    mode_colors = {0: '#90EE90', 1: '#F0E68C', 2: '#FFA500'}
    
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.patch.set_facecolor('white')
    
    # 1. Display Dynamics
    ax1 = axes[0, 0]
    ax1.plot(time_hours, hw_traj['apl'], '#1E90FF', linewidth=0.5, alpha=0.8, label='APL')
    ax1_t = ax1.twinx()
    ax1_t.fill_between(time_hours, 0, hw_traj['brightness'], color='#DAA520', alpha=0.4)
    ax1_t.plot(time_hours, hw_traj['brightness'], '#DAA520', linewidth=0.3, alpha=0.6)
    ax1.set_ylabel('APL (%)', color='#1E90FF')
    ax1_t.set_ylabel('Brightness (nits)', color='#DAA520')
    ax1.set_title('Display Dynamics', fontweight='bold')
    ax1.set_ylim(0, 100)
    ax1_t.set_ylim(0, 1500)
    
    # 2. Processor Dynamics
    ax2 = axes[0, 1]
    ax2.plot(time_hours, hw_traj['cpu_util'], '#228B22', linewidth=0.5, alpha=0.8)
    ax2_t = ax2.twinx()
    ax2_t.plot(time_hours, hw_traj['cpu_freq'], '#8B0000', linewidth=0.5, alpha=0.8)
    ax2.set_ylabel('Util (%)', color='#228B22')
    ax2_t.set_ylabel('Freq (GHz)', color='#8B0000')
    ax2.set_title('Processor Dynamics', fontweight='bold')
    ax2.set_ylim(0, 100)
    ax2_t.set_ylim(0, 3.5)
    
    # 3. Battery SOC & Power
    ax3 = axes[1, 0]
    ax3.plot(time_hours, sim_result['SOC'] * 100, '#006400', linewidth=2.5, label='SoC')
    ax3_t = ax3.twinx()
    ax3_t.plot(time_hours, sim_result['P_total'] * 1000, '#CD853F', linewidth=0.5, alpha=0.7)
    ax3.set_ylabel('SoC (%)', color='#006400')
    ax3_t.set_ylabel('Power (mW)', color='#CD853F')
    ax3.set_title('Battery Drain', fontweight='bold')
    ax3.set_ylim(0, 105)
    
    # 4. Power Composition
    ax4 = axes[1, 1]
    ax4.stackplot(time_hours, 
                  sim_result['P_base'] * 1000,
                  sim_result['P_display'] * 1000,
                  sim_result['P_soc'] * 1000,
                  labels=['Base/Net', 'Display', 'SoC'],
                  colors=['#808080', '#DAA520', '#CD5C5C'], alpha=0.8)
    ax4.set_ylabel('Power (mW)')
    ax4.set_title('Power Composition', fontweight='bold')
    ax4.legend(loc='upper right')
    
    # 5. Voltage & Current
    ax5 = axes[2, 0]
    ax5.plot(time_hours, sim_result['V_batt'], '#4169E1', linewidth=1.5, label='V_batt')
    ax5_t = ax5.twinx()
    ax5_t.plot(time_hours, sim_result['I_batt'] * 1000, '#DC143C', linewidth=0.5, alpha=0.7)
    ax5.set_ylabel('Voltage (V)', color='#4169E1')
    ax5_t.set_ylabel('Current (mA)', color='#DC143C')
    ax5.set_title('Battery Voltage & Current', fontweight='bold')
    ax5.set_xlabel('Time of Day (h)')
    
    # 6. Temperature
    ax6 = axes[2, 1]
    ax6.plot(time_hours, sim_result['T_batt'] - 273.15, '#FF4500', linewidth=1.5, label='Battery')
    ax6.plot(time_hours, sim_result['T_soc'] - 273.15, '#FF8C00', linewidth=1.5, label='SoC Chip')
    ax6.axhline(y=45, color='red', linestyle='--', alpha=0.5, label='Max Safe')
    ax6.set_ylabel('Temperature (°C)')
    ax6.set_title('Thermal Dynamics', fontweight='bold')
    ax6.set_xlabel('Time of Day (h)')
    ax6.legend(loc='upper right')
    
    # 添加时间模式色条
    for ax in axes.flatten():
        xlim = (time_hours[0], time_hours[-1])
        ax.set_xlim(xlim)
        ylim = ax.get_ylim()
        
        prev_mode = time_mode[0]
        start_h = time_hours[0]
        
        for i in range(1, len(time_mode)):
            if time_mode[i] != prev_mode or i == len(time_mode) - 1:
                end_h = time_hours[i]
                bar_height = (ylim[1] - ylim[0]) * 0.02
                ax.fill_between([start_h, end_h], ylim[0], ylim[0] + bar_height,
                              color=mode_colors[prev_mode], alpha=0.9, zorder=5)
                prev_mode = time_mode[i]
                start_h = time_hours[i]
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to {save_path}")
    
    return fig


# ==============================================================================
# 主程序
# ==============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print(" 基于严格微分方程的真实电池仿真")
    print(" Rigorous Differential Equation Based Realistic Battery Simulation")
    print("=" * 70)
    
    # 生成硬件状态轨迹
    print("\n生成硬件状态轨迹 (马尔科夫链 + O-U过程)...")
    generator = RealisticHardwareStateGenerator(seed=42)
    
    # 从早上7点到下午3点 (8小时)
    hw_trajectory = generator.generate_trajectory(
        start_hour=7.0,
        duration_hours=8.0,
        dt_seconds=1.0  # 1秒采样
    )
    
    print(f"  时间范围: {hw_trajectory['time_hours'][0]:.1f} - {hw_trajectory['time_hours'][-1]:.1f} 小时")
    print(f"  数据点数: {len(hw_trajectory['time_seconds'])}")
    
    # 运行电池仿真
    print("\n运行严格微分方程仿真...")
    simulator = RigorousBatterySimulator()
    sim_result = simulator.simulate(hw_trajectory)
    
    print(f"  初始SOC: 100%")
    print(f"  最终SOC: {sim_result['SOC'][-1]*100:.1f}%")
    print(f"  平均功耗: {np.mean(sim_result['P_total'])*1000:.0f} mW")
    print(f"  峰值功耗: {np.max(sim_result['P_total'])*1000:.0f} mW")
    print(f"  温度范围: {sim_result['T_batt'].min()-273.15:.1f} - {sim_result['T_batt'].max()-273.15:.1f} °C")
    
    # 生成可视化
    print("\n生成可视化...")
    
    # 4子图版本 (类似用户示例)
    fig1 = create_realistic_visualization(
        hw_trajectory, sim_result,
        save_path='/workspace/battery_model/realistic_battery_simulation.png'
    )
    
    # 6子图扩展版本
    fig2 = create_extended_visualization(
        hw_trajectory, sim_result,
        save_path='/workspace/battery_model/extended_battery_simulation.png'
    )
    
    print("\n可视化完成!")
    print("=" * 70)
