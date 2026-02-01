"""
高级可视化模块 - 美观新颖的电池模型可视化
Advanced Visualization Module - Beautiful and Innovative Battery Model Visualizations

特色:
1. 动态渐变色彩映射
2. 3D曲面和等高线图
3. 能量流Sankey图
4. 极坐标雷达图
5. 热力图矩阵
6. 实时仪表盘风格
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch, Wedge
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize, TwoSlopeNorm
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import gaussian_filter1d
from typing import Dict, List, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# 高级配色方案
# =============================================================================

# 电池状态渐变色
BATTERY_CMAP = LinearSegmentedColormap.from_list('battery', [
    '#FF4136',  # 红色 (低电量)
    '#FF851B',  # 橙色
    '#FFDC00',  # 黄色
    '#2ECC40',  # 绿色 (高电量)
    '#0074D9',  # 蓝色 (充满)
], N=256)

# 热力渐变色
THERMAL_CMAP = LinearSegmentedColormap.from_list('thermal', [
    '#3498DB',  # 冷蓝
    '#2ECC71',  # 正常绿
    '#F1C40F',  # 温暖黄
    '#E67E22',  # 橙色
    '#E74C3C',  # 热红
    '#9B59B6',  # 过热紫
], N=256)

# 功耗组件配色
POWER_COLORS = {
    'SoC': '#E74C3C',
    'Display': '#3498DB',
    '5G': '#9B59B6',
    'WiFi': '#1ABC9C',
    'Bluetooth': '#F39C12',
    'GPS': '#E91E63',
    'Background': '#95A5A6',
}

# 用户状态配色
STATE_COLORS = {
    0: '#34495E',   # Sleep - 深灰蓝
    1: '#7F8C8D',   # Idle - 灰色
    2: '#3498DB',   # Light - 蓝色
    3: '#2ECC71',   # Normal - 绿色
    4: '#E74C3C',   # Heavy - 红色
}

STATE_NAMES = ['Sleep', 'Idle', 'Light', 'Normal', 'Heavy']


# =============================================================================
# 美化工具函数
# =============================================================================

def set_aesthetic_style():
    """设置美观的绘图风格"""
    plt.rcParams.update({
        'figure.facecolor': '#FAFAFA',
        'axes.facecolor': '#FFFFFF',
        'axes.edgecolor': '#CCCCCC',
        'axes.labelcolor': '#333333',
        'axes.titlecolor': '#222222',
        'text.color': '#333333',
        'xtick.color': '#666666',
        'ytick.color': '#666666',
        'grid.color': '#E0E0E0',
        'grid.linestyle': '-',
        'grid.linewidth': 0.5,
        'font.family': 'sans-serif',
        'font.size': 10,
        'axes.titlesize': 13,
        'axes.labelsize': 11,
        'figure.dpi': 150,
        'savefig.dpi': 300,
    })


def add_shadow_effect(ax, rect, offset=(3, -3), alpha=0.15):
    """为矩形添加阴影效果"""
    shadow = FancyBboxPatch(
        (rect.get_x() + offset[0], rect.get_y() + offset[1]),
        rect.get_width(), rect.get_height(),
        boxstyle=rect.get_boxstyle(),
        facecolor='black', alpha=alpha
    )
    ax.add_patch(shadow)


def smooth_data(data, sigma=2):
    """高斯平滑数据"""
    return gaussian_filter1d(data, sigma=sigma)


# =============================================================================
# 高级可视化类
# =============================================================================

class AdvancedBatteryVisualizer:
    """高级电池可视化器"""
    
    def __init__(self):
        set_aesthetic_style()
    
    # =========================================================================
    # 1. 动态SOC放电曲线 (渐变发光效果)
    # =========================================================================
    
    def plot_soc_glow_curve(self, history: dict, save_path: str = None) -> plt.Figure:
        """SOC放电曲线 - 带发光渐变效果"""
        fig, ax = plt.subplots(figsize=(14, 8))
        
        time = np.array(history['time'])
        soc = np.array(history['SOC']) * 100
        
        # 平滑数据
        soc_smooth = smooth_data(soc, sigma=3)
        
        # 创建多层发光效果
        for width, alpha in [(12, 0.05), (8, 0.1), (5, 0.2), (3, 0.4), (2, 0.8)]:
            points = np.array([time, soc_smooth]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            
            lc = LineCollection(segments, cmap=BATTERY_CMAP, 
                              norm=Normalize(0, 100),
                              linewidth=width, alpha=alpha)
            lc.set_array(soc_smooth[:-1])
            ax.add_collection(lc)
        
        # 添加临界区域
        ax.axhspan(0, 15, color='#FF4136', alpha=0.08, label='Critical (<15%)')
        ax.axhspan(15, 30, color='#FF851B', alpha=0.05, label='Low (15-30%)')
        
        # 标记关键点
        critical_idx = np.where(soc_smooth <= 15)[0]
        if len(critical_idx) > 0:
            t_crit = time[critical_idx[0]]
            ax.axvline(x=t_crit, color='#FF4136', linestyle='--', linewidth=2, alpha=0.7)
            ax.scatter([t_crit], [15], s=200, c='#FF4136', zorder=5, marker='v')
            ax.annotate(f'Critical: {t_crit:.1f}h', 
                       xy=(t_crit, 15), xytext=(t_crit + 0.5, 25),
                       fontsize=11, fontweight='bold', color='#FF4136',
                       arrowprops=dict(arrowstyle='->', color='#FF4136', lw=2))
        
        # 起点和终点标记
        ax.scatter([time[0]], [soc_smooth[0]], s=150, c='#0074D9', zorder=5, 
                  marker='o', edgecolors='white', linewidths=2)
        ax.scatter([time[-1]], [soc_smooth[-1]], s=150, c='#FF4136', zorder=5,
                  marker='s', edgecolors='white', linewidths=2)
        
        ax.set_xlim(time.min() - 0.1, time.max() + 0.1)
        ax.set_ylim(-2, 105)
        ax.set_xlabel('Time (hours)', fontsize=12, fontweight='bold')
        ax.set_ylabel('State of Charge (%)', fontsize=12, fontweight='bold')
        ax.set_title('Battery Discharge Curve with Glow Effect', fontsize=14, fontweight='bold', pad=15)
        
        # 添加colorbar
        sm = plt.cm.ScalarMappable(cmap=BATTERY_CMAP, norm=Normalize(0, 100))
        cbar = plt.colorbar(sm, ax=ax, shrink=0.8, aspect=30)
        cbar.set_label('SOC Level (%)', fontsize=10)
        
        ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig
    
    # =========================================================================
    # 2. 功耗分解河流图 (Streamgraph)
    # =========================================================================
    
    def plot_power_streamgraph(self, history: dict, save_path: str = None) -> plt.Figure:
        """功耗分解 - 河流图样式"""
        fig, ax = plt.subplots(figsize=(16, 9))
        
        time = np.array(history['time'])
        
        # 提取组件功耗
        components = list(history['components'][0].keys())
        power_data = {comp: smooth_data(np.array([h[comp] for h in history['components']]), 5) 
                      for comp in components}
        
        # 计算河流图的上下边界
        n = len(time)
        stack_pos = np.zeros(n)
        stack_neg = np.zeros(n)
        
        colors = [POWER_COLORS.get(comp, '#888888') for comp in components]
        
        # 交替向上下堆叠,形成河流效果
        for i, comp in enumerate(components):
            data = power_data[comp]
            if i % 2 == 0:
                ax.fill_between(time, stack_pos, stack_pos + data, 
                               color=colors[i], alpha=0.85, label=comp,
                               edgecolor='white', linewidth=0.5)
                stack_pos += data
            else:
                ax.fill_between(time, stack_neg - data, stack_neg,
                               color=colors[i], alpha=0.85, label=comp,
                               edgecolor='white', linewidth=0.5)
                stack_neg -= data
        
        # 添加中心线
        ax.axhline(y=0, color='white', linewidth=3, zorder=10)
        
        # 添加总功耗线
        total = np.array(history['P_total'])
        ax.plot(time, total / 2, 'k-', linewidth=2, alpha=0.5, label='Total/2')
        ax.plot(time, -total / 2, 'k-', linewidth=2, alpha=0.5)
        
        ax.set_xlim(time.min(), time.max())
        ax.set_xlabel('Time (hours)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Power Flow (W)', fontsize=12, fontweight='bold')
        ax.set_title('Power Consumption Streamgraph', fontsize=14, fontweight='bold', pad=15)
        
        ax.legend(loc='upper right', ncol=4, fontsize=9, framealpha=0.9)
        ax.grid(True, alpha=0.2, axis='x')
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig
    
    # =========================================================================
    # 3. 3D剩余时间预测曲面
    # =========================================================================
    
    def plot_3d_prediction_surface(self, model, save_path: str = None) -> plt.Figure:
        """3D剩余时间预测曲面"""
        fig = plt.figure(figsize=(16, 7))
        
        # 创建数据
        soc_range = np.linspace(0.05, 1.0, 40)
        power_range = np.linspace(0.1, 4.0, 40)
        SOC, POWER = np.meshgrid(soc_range, power_range)
        
        Q = 4.0  # Ah
        V = 3.7  # V
        eta = 0.92
        
        TIME = (SOC * Q) / (POWER / (eta * V))
        
        # 3D曲面图
        ax1 = fig.add_subplot(121, projection='3d')
        
        surf = ax1.plot_surface(SOC * 100, POWER, TIME, cmap='viridis',
                               alpha=0.9, antialiased=True,
                               linewidth=0, edgecolor='none')
        
        # 添加等时间线
        for t in [2, 5, 10, 20]:
            mask = np.abs(TIME - t) < 0.5
            if mask.any():
                ax1.scatter(SOC[mask] * 100, POWER[mask], TIME[mask],
                           c='white', s=2, alpha=0.5)
        
        ax1.set_xlabel('SOC (%)', fontsize=10, labelpad=10)
        ax1.set_ylabel('Power (W)', fontsize=10, labelpad=10)
        ax1.set_zlabel('Time (h)', fontsize=10, labelpad=10)
        ax1.set_title('3D Remaining Time Surface', fontsize=12, fontweight='bold')
        ax1.view_init(elev=25, azim=-60)
        
        fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=15, label='Hours')
        
        # 2D等高线图
        ax2 = fig.add_subplot(122)
        
        levels = [0.5, 1, 2, 3, 5, 8, 12, 18, 24, 36]
        cf = ax2.contourf(SOC * 100, POWER, TIME, levels=levels, cmap='viridis', alpha=0.9)
        cs = ax2.contour(SOC * 100, POWER, TIME, levels=levels, colors='white', 
                        linewidths=1, alpha=0.7)
        ax2.clabel(cs, inline=True, fontsize=8, fmt='%.0fh')
        
        # 添加使用场景标记
        scenarios = {
            'Idle': (80, 0.15, '#2ECC40'),
            'Light': (60, 0.5, '#0074D9'),
            'Normal': (50, 1.2, '#FFDC00'),
            'Heavy': (30, 3.0, '#FF4136'),
        }
        
        for name, (soc_val, power_val, color) in scenarios.items():
            ax2.scatter([soc_val], [power_val], s=200, c=color, 
                       edgecolors='white', linewidths=2, zorder=5)
            ax2.annotate(name, (soc_val, power_val), 
                        textcoords='offset points', xytext=(10, 5),
                        fontsize=10, fontweight='bold', color=color)
        
        ax2.set_xlabel('SOC (%)', fontsize=11, fontweight='bold')
        ax2.set_ylabel('Power (W)', fontsize=11, fontweight='bold')
        ax2.set_title('Remaining Time Contour Map', fontsize=12, fontweight='bold')
        
        cbar = fig.colorbar(cf, ax=ax2, shrink=0.9)
        cbar.set_label('Remaining Time (hours)', fontsize=10)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig
    
    # =========================================================================
    # 4. 热力耦合动态图
    # =========================================================================
    
    def plot_thermal_dynamics(self, history: dict, save_path: str = None) -> plt.Figure:
        """热动力学可视化"""
        fig = plt.figure(figsize=(16, 10))
        gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        time = np.array(history['time'])
        T_batt = np.array(history['T_batt'])
        T_soc = np.array(history['T_soc'])
        soc = np.array(history['SOC']) * 100
        power = np.array(history['P_total'])
        
        # 平滑
        T_batt_s = smooth_data(T_batt, 3)
        T_soc_s = smooth_data(T_soc, 3)
        
        # 1. 温度时间演化 (带发光)
        ax1 = fig.add_subplot(gs[0, :2])
        
        for width, alpha in [(6, 0.1), (4, 0.2), (2, 0.6)]:
            points = np.array([time, T_batt_s]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=THERMAL_CMAP,
                              norm=Normalize(20, 50), linewidth=width, alpha=alpha)
            lc.set_array(T_batt_s)
            ax1.add_collection(lc)
        
        ax1.plot(time, T_soc_s, '--', color='#9B59B6', linewidth=2, alpha=0.7, label='SoC Temp')
        
        # 温度区域
        ax1.axhspan(20, 35, color='#2ECC71', alpha=0.1, label='Optimal')
        ax1.axhspan(35, 45, color='#F1C40F', alpha=0.1, label='Warm')
        ax1.axhspan(45, 60, color='#E74C3C', alpha=0.1, label='Hot')
        
        ax1.set_xlim(time.min(), time.max())
        ax1.set_ylim(18, max(55, T_batt.max() + 5))
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('Temperature (°C)', fontsize=11)
        ax1.set_title('Temperature Evolution', fontsize=12, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=9)
        ax1.grid(True, alpha=0.3)
        
        # 2. 温度仪表盘
        ax2 = fig.add_subplot(gs[0, 2])
        
        current_temp = T_batt[-1]
        
        # 绘制温度计
        theta = np.linspace(0.75 * np.pi, 0.25 * np.pi, 100)
        r = 0.8
        
        # 背景弧
        for i, (t_min, t_max, color) in enumerate([
            (20, 35, '#2ECC71'), (35, 45, '#F1C40F'), (45, 60, '#E74C3C')
        ]):
            t1 = 0.75 - (t_min - 20) / 40 * 0.5
            t2 = 0.75 - (t_max - 20) / 40 * 0.5
            theta_seg = np.linspace(t1 * np.pi, t2 * np.pi, 30)
            ax2.fill_between(theta_seg, 0.6, 1.0, alpha=0.3, color=color)
        
        # 指针
        temp_angle = 0.75 - (current_temp - 20) / 40 * 0.5
        temp_angle = np.clip(temp_angle, 0.25, 0.75)
        ax2.annotate('', xy=(np.cos(temp_angle * np.pi) * 0.7, np.sin(temp_angle * np.pi) * 0.7),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle='->', lw=3, color='#2C3E50'))
        
        ax2.add_patch(Circle((0, 0), 0.1, color='#2C3E50'))
        ax2.text(0, -0.3, f'{current_temp:.1f}°C', ha='center', fontsize=16, fontweight='bold')
        ax2.text(0, -0.5, 'Battery Temperature', ha='center', fontsize=10)
        
        ax2.set_xlim(-1.2, 1.2)
        ax2.set_ylim(-0.7, 1.2)
        ax2.set_aspect('equal')
        ax2.axis('off')
        ax2.set_title('Current Temperature', fontsize=12, fontweight='bold')
        
        # 3. 温度-功耗相关性
        ax3 = fig.add_subplot(gs[1, 0])
        
        scatter = ax3.scatter(power, T_batt, c=soc, cmap=BATTERY_CMAP, 
                             s=40, alpha=0.6, edgecolors='white', linewidths=0.5)
        
        # 趋势线
        z = np.polyfit(power, T_batt, 2)
        p_fit = np.linspace(power.min(), power.max(), 100)
        ax3.plot(p_fit, np.polyval(z, p_fit), 'r--', linewidth=2, alpha=0.7)
        
        ax3.set_xlabel('Power (W)', fontsize=11)
        ax3.set_ylabel('Temperature (°C)', fontsize=11)
        ax3.set_title('Temperature vs Power', fontsize=12, fontweight='bold')
        cbar = plt.colorbar(scatter, ax=ax3)
        cbar.set_label('SOC (%)')
        ax3.grid(True, alpha=0.3)
        
        # 4. 热量分布饼图
        ax4 = fig.add_subplot(gs[1, 1])
        
        # 估算热量来源
        avg_power = np.mean(power)
        heat_sources = {
            'Joule (I²R)': avg_power * 0.08,
            'Entropy': avg_power * 0.02,
            'SoC': avg_power * 0.5,
            'Display': avg_power * 0.25,
            'Comm': avg_power * 0.15,
        }
        
        colors = ['#E74C3C', '#9B59B6', '#3498DB', '#F39C12', '#1ABC9C']
        wedges, texts, autotexts = ax4.pie(
            heat_sources.values(), labels=heat_sources.keys(),
            colors=colors, autopct='%1.1f%%',
            explode=[0.05] * 5, shadow=True, startangle=90
        )
        ax4.set_title('Heat Generation Distribution', fontsize=12, fontweight='bold')
        
        # 5. 温度-SOC相关性
        ax5 = fig.add_subplot(gs[1, 2])
        
        # 六边形bin图
        hb = ax5.hexbin(soc, T_batt, gridsize=20, cmap='YlOrRd', mincnt=1)
        ax5.set_xlabel('SOC (%)', fontsize=11)
        ax5.set_ylabel('Temperature (°C)', fontsize=11)
        ax5.set_title('Temperature-SOC Distribution', fontsize=12, fontweight='bold')
        plt.colorbar(hb, ax=ax5, label='Count')
        
        plt.suptitle('Thermal Dynamics Analysis', fontsize=14, fontweight='bold', y=1.02)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig
    
    # =========================================================================
    # 5. 用户状态马尔科夫转移可视化
    # =========================================================================
    
    def plot_user_state_markov(self, history: dict, save_path: str = None) -> plt.Figure:
        """用户状态马尔科夫链可视化"""
        fig = plt.figure(figsize=(16, 10))
        gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        time = np.array(history['time'])
        states = np.array(history['user_state'])
        soc = np.array(history['SOC']) * 100
        
        # 1. 状态时间线 (热力带)
        ax1 = fig.add_subplot(gs[0, :])
        
        # 创建状态热力带
        for i in range(len(time) - 1):
            state = states[i]
            color = STATE_COLORS[state]
            ax1.axvspan(time[i], time[i+1], color=color, alpha=0.6)
        
        # 叠加SOC曲线
        ax1_twin = ax1.twinx()
        ax1_twin.fill_between(time, 0, soc, alpha=0.3, color='#3498DB')
        ax1_twin.plot(time, soc, 'k-', linewidth=2, alpha=0.7)
        ax1_twin.set_ylabel('SOC (%)', fontsize=11)
        ax1_twin.set_ylim(0, 105)
        
        # 图例
        legend_patches = [mpatches.Patch(color=STATE_COLORS[i], label=STATE_NAMES[i], alpha=0.6)
                         for i in range(5)]
        ax1.legend(handles=legend_patches, loc='upper center', ncol=5, fontsize=9,
                  bbox_to_anchor=(0.5, 1.15))
        
        ax1.set_xlim(time.min(), time.max())
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('User State', fontsize=11)
        ax1.set_yticks([])
        ax1.set_title('User State Timeline with SOC Overlay', fontsize=12, fontweight='bold')
        
        # 2. 状态分布饼图
        ax2 = fig.add_subplot(gs[1, 0])
        
        state_counts = [np.sum(states == i) for i in range(5)]
        colors = [STATE_COLORS[i] for i in range(5)]
        
        wedges, texts, autotexts = ax2.pie(
            state_counts, labels=STATE_NAMES, colors=colors,
            autopct='%1.1f%%', explode=[0.03] * 5, shadow=True,
            startangle=90, textprops={'fontsize': 9}
        )
        ax2.set_title('State Distribution', fontsize=12, fontweight='bold')
        
        # 3. 状态转移计数矩阵
        ax3 = fig.add_subplot(gs[1, 1])
        
        transition_matrix = np.zeros((5, 5))
        for i in range(len(states) - 1):
            transition_matrix[states[i], states[i+1]] += 1
        
        # 归一化
        row_sums = transition_matrix.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1
        transition_prob = transition_matrix / row_sums
        
        im = ax3.imshow(transition_prob, cmap='YlOrRd', vmin=0, vmax=1)
        
        ax3.set_xticks(range(5))
        ax3.set_yticks(range(5))
        ax3.set_xticklabels(STATE_NAMES, rotation=45, ha='right', fontsize=9)
        ax3.set_yticklabels(STATE_NAMES, fontsize=9)
        
        for i in range(5):
            for j in range(5):
                text = ax3.text(j, i, f'{transition_prob[i, j]:.2f}',
                              ha='center', va='center', fontsize=8,
                              color='white' if transition_prob[i, j] > 0.5 else 'black')
        
        ax3.set_title('Transition Probability Matrix', fontsize=12, fontweight='bold')
        plt.colorbar(im, ax=ax3, shrink=0.8)
        
        # 4. 各状态放电率对比
        ax4 = fig.add_subplot(gs[1, 2])
        
        discharge_rates = []
        for state in range(5):
            mask = states[:-1] == state
            if mask.sum() > 1:
                dt = np.diff(time)[mask]
                dsoc = np.diff(soc)[mask]
                rate = -np.mean(dsoc / dt) if dt.sum() > 0 else 0
            else:
                rate = 0
            discharge_rates.append(rate)
        
        bars = ax4.bar(range(5), discharge_rates, color=colors, alpha=0.8,
                      edgecolor='white', linewidth=2)
        
        ax4.set_xticks(range(5))
        ax4.set_xticklabels(STATE_NAMES, fontsize=10)
        ax4.set_ylabel('Discharge Rate (%/hour)', fontsize=11)
        ax4.set_title('Discharge Rate by State', fontsize=12, fontweight='bold')
        
        for bar, rate in zip(bars, discharge_rates):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{rate:.1f}', ha='center', fontsize=10, fontweight='bold')
        
        ax4.grid(True, alpha=0.3, axis='y')
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig
    
    # =========================================================================
    # 6. 综合仪表盘
    # =========================================================================
    
    def plot_dashboard(self, history: dict, model, save_path: str = None) -> plt.Figure:
        """综合仪表盘"""
        fig = plt.figure(figsize=(20, 14))
        fig.patch.set_facecolor('#1A1A2E')  # 深色背景
        
        gs = GridSpec(3, 4, figure=fig, hspace=0.35, wspace=0.35)
        
        time = np.array(history['time'])
        soc = np.array(history['SOC']) * 100
        power = np.array(history['P_total'])
        T_batt = np.array(history['T_batt'])
        V_batt = np.array(history['V_batt'])
        states = np.array(history['user_state'])
        
        # 通用样式设置
        text_color = '#FFFFFF'
        grid_color = '#3A3A5C'
        
        # 1. 主SOC曲线
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.set_facecolor('#16213E')
        
        points = np.array([time, soc]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        for w, a in [(8, 0.15), (5, 0.3), (3, 0.6), (2, 1.0)]:
            lc = LineCollection(segments, cmap=BATTERY_CMAP, norm=Normalize(0, 100),
                              linewidth=w, alpha=a)
            lc.set_array(soc[:-1])
            ax1.add_collection(lc)
        
        ax1.set_xlim(time.min(), time.max())
        ax1.set_ylim(0, 105)
        ax1.set_xlabel('Time (hours)', color=text_color, fontsize=11)
        ax1.set_ylabel('SOC (%)', color=text_color, fontsize=11)
        ax1.set_title('🔋 Battery State of Charge', color=text_color, fontsize=13, fontweight='bold')
        ax1.tick_params(colors=text_color)
        ax1.grid(True, color=grid_color, alpha=0.3)
        for spine in ax1.spines.values():
            spine.set_color(grid_color)
        
        # 2. 功耗堆叠图
        ax2 = fig.add_subplot(gs[0, 2:])
        ax2.set_facecolor('#16213E')
        
        components = list(history['components'][0].keys())
        power_data = {comp: np.array([h[comp] for h in history['components']]) for comp in components}
        
        stack = np.vstack([power_data[c] for c in components])
        colors_stack = [POWER_COLORS.get(c, '#888888') for c in components]
        
        ax2.stackplot(time, stack, labels=components, colors=colors_stack, alpha=0.85)
        ax2.plot(time, power, 'w-', linewidth=2, alpha=0.5, label='Total')
        
        ax2.set_xlim(time.min(), time.max())
        ax2.set_xlabel('Time (hours)', color=text_color, fontsize=11)
        ax2.set_ylabel('Power (W)', color=text_color, fontsize=11)
        ax2.set_title('⚡ Power Consumption', color=text_color, fontsize=13, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=8, ncol=3, facecolor='#16213E', 
                  edgecolor=grid_color, labelcolor=text_color)
        ax2.tick_params(colors=text_color)
        ax2.grid(True, color=grid_color, alpha=0.3)
        for spine in ax2.spines.values():
            spine.set_color(grid_color)
        
        # 3-6. 仪表盘指示器
        for idx, (title, value, unit, vmin, vmax, color) in enumerate([
            ('SOC', soc[-1], '%', 0, 100, '#2ECC71' if soc[-1] > 20 else '#E74C3C'),
            ('Power', power[-1], 'W', 0, 5, '#3498DB'),
            ('Temp', T_batt[-1], '°C', 20, 60, '#F1C40F' if T_batt[-1] < 40 else '#E74C3C'),
            ('Voltage', V_batt[-1], 'V', 3.0, 4.2, '#9B59B6'),
        ]):
            ax = fig.add_subplot(gs[1, idx])
            ax.set_facecolor('#16213E')
            ax.axis('off')
            
            # 绘制圆形进度条
            progress = (value - vmin) / (vmax - vmin)
            progress = np.clip(progress, 0, 1)
            
            # 背景圆环
            theta = np.linspace(0, 2*np.pi, 100)
            ax.plot(np.cos(theta), np.sin(theta), color=grid_color, linewidth=20, alpha=0.3)
            
            # 进度圆环
            theta_prog = np.linspace(np.pi/2, np.pi/2 - 2*np.pi*progress, 100)
            ax.plot(np.cos(theta_prog), np.sin(theta_prog), color=color, linewidth=20, solid_capstyle='round')
            
            # 中心文字
            ax.text(0, 0.1, f'{value:.1f}', ha='center', va='center', 
                   fontsize=28, fontweight='bold', color=text_color)
            ax.text(0, -0.3, unit, ha='center', va='center', fontsize=14, color=text_color, alpha=0.7)
            ax.text(0, 1.4, title, ha='center', va='center', fontsize=12, 
                   fontweight='bold', color=text_color)
            
            ax.set_xlim(-1.5, 1.5)
            ax.set_ylim(-1.5, 1.5)
            ax.set_aspect('equal')
        
        # 7. 用户状态时间线
        ax7 = fig.add_subplot(gs[2, :2])
        ax7.set_facecolor('#16213E')
        
        for i in range(len(time) - 1):
            state = states[i]
            ax7.barh(0, time[i+1] - time[i], left=time[i], 
                    color=STATE_COLORS[state], height=0.6, alpha=0.8)
        
        ax7.set_xlim(time.min(), time.max())
        ax7.set_ylim(-0.5, 0.5)
        ax7.set_yticks([])
        ax7.set_xlabel('Time (hours)', color=text_color, fontsize=11)
        ax7.set_title('👤 User Activity', color=text_color, fontsize=13, fontweight='bold')
        ax7.tick_params(colors=text_color)
        for spine in ax7.spines.values():
            spine.set_color(grid_color)
        
        # 图例
        legend_patches = [mpatches.Patch(color=STATE_COLORS[i], label=STATE_NAMES[i], alpha=0.8)
                         for i in range(5)]
        ax7.legend(handles=legend_patches, loc='upper right', ncol=5, fontsize=8,
                  facecolor='#16213E', edgecolor=grid_color, labelcolor=text_color)
        
        # 8. 剩余时间预测
        ax8 = fig.add_subplot(gs[2, 2:])
        ax8.set_facecolor('#16213E')
        
        current_soc = soc[-1] / 100
        scenarios = ['idle', 'light', 'normal', 'heavy']
        times = [model.predict_remaining_time(current_soc, s)[0] for s in scenarios]
        colors_pred = ['#2ECC71', '#3498DB', '#F1C40F', '#E74C3C']
        
        bars = ax8.barh(range(4), times, color=colors_pred, alpha=0.8, height=0.6,
                       edgecolor='white', linewidth=1)
        
        ax8.set_yticks(range(4))
        ax8.set_yticklabels([s.capitalize() for s in scenarios], color=text_color, fontsize=11)
        ax8.set_xlabel('Remaining Time (hours)', color=text_color, fontsize=11)
        ax8.set_title(f'⏱️ Time Remaining (SOC={current_soc*100:.0f}%)', 
                     color=text_color, fontsize=13, fontweight='bold')
        ax8.tick_params(colors=text_color)
        ax8.grid(True, color=grid_color, alpha=0.3, axis='x')
        for spine in ax8.spines.values():
            spine.set_color(grid_color)
        
        for bar, t in zip(bars, times):
            ax8.text(bar.get_width() + 0.2, bar.get_y() + bar.get_height()/2,
                    f'{t:.1f}h', va='center', color=text_color, fontsize=11, fontweight='bold')
        
        plt.suptitle('📱 Smartphone Battery Dashboard', fontsize=18, fontweight='bold', 
                    color='white', y=0.98)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#1A1A2E')
        
        return fig
    
    # =========================================================================
    # 7. 多场景雷达对比图
    # =========================================================================
    
    def plot_scenario_radar(self, histories: Dict[str, dict], save_path: str = None) -> plt.Figure:
        """多场景雷达对比图"""
        fig = plt.figure(figsize=(14, 12))
        
        # 计算各场景指标
        categories = ['Battery Life', 'Efficiency', 'Temperature', 'Stability', 'Avg Power']
        n_cats = len(categories)
        angles = np.linspace(0, 2*np.pi, n_cats, endpoint=False).tolist()
        angles += angles[:1]
        
        colors = {
            'idle': '#2ECC71',
            'light': '#3498DB', 
            'normal': '#F1C40F',
            'heavy': '#E74C3C',
            'realistic': '#9B59B6'
        }
        
        ax = fig.add_subplot(111, projection='polar')
        ax.set_facecolor('#FAFAFA')
        
        for scenario, history in histories.items():
            soc = np.array(history['SOC'])
            time = np.array(history['time'])
            power = np.array(history['P_total'])
            temp = np.array(history['T_batt'])
            
            # 计算指标 (归一化到0-1)
            idx = np.where(soc <= 0.05)[0]
            drain_time = time[idx[0]] if len(idx) > 0 else time[-1]
            
            battery_life = min(1, drain_time / 30)
            efficiency = 1 - np.mean(power) / 5
            temperature = 1 - (np.max(temp) - 25) / 35
            stability = 1 - np.std(np.diff(soc)) * 50
            avg_power = 1 - np.mean(power) / 5
            
            values = [battery_life, efficiency, temperature, stability, avg_power]
            values = [max(0, min(1, v)) for v in values]
            values += values[:1]
            
            color = colors.get(scenario, '#888888')
            ax.plot(angles, values, 'o-', linewidth=2.5, label=scenario.capitalize(), 
                   color=color, markersize=8)
            ax.fill(angles, values, alpha=0.15, color=color)
        
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=11, fontweight='bold')
        ax.set_ylim(0, 1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['20%', '40%', '60%', '80%', '100%'], fontsize=9, alpha=0.7)
        
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=11)
        ax.set_title('Scenario Performance Comparison', fontsize=14, fontweight='bold', pad=20)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='#FAFAFA')
        
        return fig


# =============================================================================
# 完整可视化生成
# =============================================================================

def generate_advanced_visualizations(model, histories: Dict[str, dict], output_dir: str = './'):
    """生成所有高级可视化"""
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    viz = AdvancedBatteryVisualizer()
    
    # 选择主场景
    main_hist = histories.get('realistic', histories.get('normal', list(histories.values())[0]))
    
    print("Generating advanced visualizations...")
    
    # 1. SOC发光曲线
    print("  1/7 SOC glow curve...")
    viz.plot_soc_glow_curve(main_hist, f'{output_dir}/adv_01_soc_glow.png')
    plt.close()
    
    # 2. 功耗河流图
    print("  2/7 Power streamgraph...")
    viz.plot_power_streamgraph(main_hist, f'{output_dir}/adv_02_power_stream.png')
    plt.close()
    
    # 3. 3D预测曲面
    print("  3/7 3D prediction surface...")
    viz.plot_3d_prediction_surface(model, f'{output_dir}/adv_03_3d_surface.png')
    plt.close()
    
    # 4. 热动力学
    print("  4/7 Thermal dynamics...")
    viz.plot_thermal_dynamics(main_hist, f'{output_dir}/adv_04_thermal.png')
    plt.close()
    
    # 5. 用户状态马尔科夫
    print("  5/7 User state Markov...")
    viz.plot_user_state_markov(main_hist, f'{output_dir}/adv_05_markov.png')
    plt.close()
    
    # 6. 综合仪表盘
    print("  6/7 Dashboard...")
    viz.plot_dashboard(main_hist, model, f'{output_dir}/adv_06_dashboard.png')
    plt.close()
    
    # 7. 场景雷达图
    print("  7/7 Scenario radar...")
    viz.plot_scenario_radar(histories, f'{output_dir}/adv_07_radar.png')
    plt.close()
    
    print(f"Advanced visualizations saved to {output_dir}")


if __name__ == "__main__":
    from enhanced_model import EnhancedBatterySystem
    
    print("Testing advanced visualization...")
    
    model = EnhancedBatterySystem()
    
    scenarios = ['idle', 'light', 'normal', 'heavy', 'realistic']
    histories = {}
    
    for scenario in scenarios:
        print(f"  Simulating {scenario}...")
        histories[scenario] = model.simulate(
            duration_hours=24,
            initial_soc=1.0,
            scenario=scenario,
            dt=30
        )
    
    generate_advanced_visualizations(model, histories, './advanced_output')
