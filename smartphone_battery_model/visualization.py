"""
智能手机电池模型可视化模块
Smartphone Battery Model Visualization Module

提供创新且美观的数据可视化
Provides innovative and aesthetically pleasing visualizations
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from typing import Dict, List, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体和样式
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['figure.facecolor'] = 'white'

# 自定义配色方案
COLORS = {
    'primary': '#2E86AB',
    'secondary': '#A23B72',
    'accent': '#F18F01',
    'success': '#C73E1D',
    'background': '#F5F5F5',
    'dark': '#1A1A2E',
    'gradient_start': '#667eea',
    'gradient_end': '#764ba2',
}

# 功耗组件颜色
COMPONENT_COLORS = {
    'P_SoC': '#FF6B6B',
    'P_display': '#4ECDC4',
    'P_5G': '#45B7D1',
    'P_WiFi': '#96CEB4',
    'P_BT': '#FFEAA7',
    'P_GNSS': '#DDA0DD',
    'P_background': '#98D8C8',
}

# 用户状态颜色
STATE_COLORS = {
    0: '#2C3E50',  # Sleep - dark blue
    1: '#3498DB',  # Work - blue
    2: '#2ECC71',  # Leisure - green
    3: '#E74C3C',  # Heavy Use - red
}


def create_gradient_colormap(name='battery'):
    """创建自定义渐变色图"""
    if name == 'battery':
        colors = ['#FF4757', '#FFA502', '#2ED573', '#1E90FF']
        return LinearSegmentedColormap.from_list('battery', colors, N=256)
    elif name == 'thermal':
        colors = ['#00D2D3', '#54A0FF', '#FFA502', '#FF6B6B', '#EE5A24']
        return LinearSegmentedColormap.from_list('thermal', colors, N=256)
    elif name == 'power':
        colors = ['#74B9FF', '#0984E3', '#6C5CE7', '#A29BFE']
        return LinearSegmentedColormap.from_list('power', colors, N=256)
    return plt.cm.viridis


class BatteryVisualizer:
    """电池模型可视化类"""
    
    def __init__(self, style='modern'):
        """初始化可视化器
        
        Args:
            style: 风格 ('modern', 'classic', 'dark')
        """
        self.style = style
        self._setup_style()
    
    def _setup_style(self):
        """设置绘图风格"""
        if self.style == 'dark':
            plt.style.use('dark_background')
        else:
            plt.style.use('seaborn-v0_8-whitegrid')
    
    def plot_soc_discharge_curve(self, history: dict, 
                                 title: str = "Battery SOC Discharge Curve",
                                 save_path: Optional[str] = None) -> plt.Figure:
        """绘制SOC放电曲线 - 带渐变色和状态指示
        
        创新点：使用渐变色线条表示SOC状态，添加临界区域标注
        """
        fig, ax = plt.subplots(figsize=(14, 8))
        
        time = np.array(history['time'])
        soc = np.array(history['SOC']) * 100
        
        # 创建渐变色线条
        points = np.array([time, soc]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # 根据SOC值着色
        norm = Normalize(0, 100)
        cmap = create_gradient_colormap('battery')
        lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=3, alpha=0.9)
        lc.set_array(soc[:-1])
        
        ax.add_collection(lc)
        
        # 添加临界区域
        ax.axhspan(0, 20, alpha=0.2, color='#FF6B6B', label='Critical Zone (<20%)')
        ax.axhspan(20, 50, alpha=0.1, color='#FFA502', label='Low Zone (20-50%)')
        
        # 添加关键点标注
        critical_indices = np.where(soc <= 20)[0]
        if len(critical_indices) > 0:
            critical_time = time[critical_indices[0]]
            ax.axvline(x=critical_time, color='#FF4757', linestyle='--', 
                      alpha=0.7, linewidth=2)
            ax.annotate(f'Critical at {critical_time:.1f}h', 
                       xy=(critical_time, 20),
                       xytext=(critical_time + 0.5, 35),
                       fontsize=10, color='#FF4757',
                       arrowprops=dict(arrowstyle='->', color='#FF4757'))
        
        # 设置轴
        ax.set_xlim(time.min(), time.max())
        ax.set_ylim(0, 105)
        ax.set_xlabel('Time (hours)', fontsize=12, fontweight='bold')
        ax.set_ylabel('State of Charge (%)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
        
        # 添加colorbar
        cbar = plt.colorbar(lc, ax=ax, shrink=0.8)
        cbar.set_label('SOC Level (%)', fontsize=10)
        
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_power_breakdown_sankey_style(self, history: dict,
                                          save_path: Optional[str] = None) -> plt.Figure:
        """功耗分解图 - Sankey风格堆叠面积图
        
        创新点：结合面积图和能量流动概念
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 12), 
                                        gridspec_kw={'height_ratios': [2, 1]})
        
        time = np.array(history['time'])
        
        # 提取各组件功耗
        components = ['P_SoC', 'P_display', 'P_5G', 'P_WiFi', 'P_BT', 'P_GNSS', 'P_background']
        power_data = {comp: [] for comp in components}
        
        for p_comp in history['P_components']:
            for comp in components:
                power_data[comp].append(p_comp.get(comp, 0))
        
        # 转换为numpy数组
        for comp in components:
            power_data[comp] = np.array(power_data[comp])
        
        # 绘制堆叠面积图
        stack_data = np.vstack([power_data[comp] for comp in components])
        colors = [COMPONENT_COLORS[comp] for comp in components]
        
        ax1.stackplot(time, stack_data, labels=components, colors=colors, alpha=0.85)
        
        # 添加总功耗线
        total_power = np.array(history['P_total'])
        ax1.plot(time, total_power, 'k-', linewidth=2, label='Total Power', alpha=0.7)
        
        ax1.set_xlabel('Time (hours)', fontsize=12)
        ax1.set_ylabel('Power (W)', fontsize=12)
        ax1.set_title('Power Consumption Breakdown Over Time', fontsize=14, fontweight='bold')
        ax1.legend(loc='upper right', ncol=4, fontsize=9)
        ax1.set_xlim(time.min(), time.max())
        ax1.grid(True, alpha=0.3)
        
        # 绘制平均功耗饼图
        avg_power = {comp: np.mean(power_data[comp]) for comp in components}
        total_avg = sum(avg_power.values())
        
        # 过滤掉太小的组件
        significant_comps = {k: v for k, v in avg_power.items() if v/total_avg > 0.01}
        other_power = sum(v for k, v in avg_power.items() if k not in significant_comps)
        if other_power > 0:
            significant_comps['Other'] = other_power
        
        wedges, texts, autotexts = ax2.pie(
            significant_comps.values(),
            labels=significant_comps.keys(),
            colors=[COMPONENT_COLORS.get(k, '#888888') for k in significant_comps.keys()],
            autopct='%1.1f%%',
            startangle=90,
            explode=[0.02] * len(significant_comps),
            shadow=True
        )
        
        ax2.set_title('Average Power Distribution', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_thermal_behavior(self, history: dict,
                             save_path: Optional[str] = None) -> plt.Figure:
        """温度行为可视化 - 热力图风格
        
        创新点：使用热力图表示温度分布
        """
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(2, 2, figure=fig, height_ratios=[1.5, 1])
        
        time = np.array(history['time'])
        temp = np.array(history['T_batt'])
        soc = np.array(history['SOC']) * 100
        power = np.array(history['P_total'])
        
        # 子图1：温度-时间曲线（带渐变）
        ax1 = fig.add_subplot(gs[0, :])
        
        points = np.array([time, temp]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        norm = Normalize(temp.min(), temp.max())
        cmap = create_gradient_colormap('thermal')
        lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=4)
        lc.set_array(temp[:-1])
        ax1.add_collection(lc)
        
        # 添加安全区域
        ax1.axhspan(20, 35, alpha=0.2, color='#2ECC71', label='Optimal (20-35°C)')
        ax1.axhspan(35, 45, alpha=0.2, color='#FFA502', label='Warm (35-45°C)')
        ax1.axhspan(45, 60, alpha=0.2, color='#FF6B6B', label='Hot (>45°C)')
        
        ax1.set_xlim(time.min(), time.max())
        ax1.set_ylim(temp.min() - 2, max(temp.max() + 2, 50))
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('Temperature (°C)', fontsize=11)
        ax1.set_title('Battery Temperature Evolution', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=9)
        
        cbar = plt.colorbar(lc, ax=ax1, shrink=0.8)
        cbar.set_label('Temperature (°C)', fontsize=10)
        
        # 子图2：温度-功耗关系散点图
        ax2 = fig.add_subplot(gs[1, 0])
        scatter = ax2.scatter(power, temp, c=soc, cmap=create_gradient_colormap('battery'),
                             s=30, alpha=0.6, edgecolors='none')
        ax2.set_xlabel('Power (W)', fontsize=11)
        ax2.set_ylabel('Temperature (°C)', fontsize=11)
        ax2.set_title('Temperature vs Power', fontsize=12, fontweight='bold')
        cbar2 = plt.colorbar(scatter, ax=ax2)
        cbar2.set_label('SOC (%)', fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # 子图3：温度-SOC关系
        ax3 = fig.add_subplot(gs[1, 1])
        scatter2 = ax3.scatter(soc, temp, c=time, cmap='viridis',
                              s=30, alpha=0.6, edgecolors='none')
        ax3.set_xlabel('SOC (%)', fontsize=11)
        ax3.set_ylabel('Temperature (°C)', fontsize=11)
        ax3.set_title('Temperature vs SOC', fontsize=12, fontweight='bold')
        cbar3 = plt.colorbar(scatter2, ax=ax3)
        cbar3.set_label('Time (hours)', fontsize=9)
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_user_state_timeline(self, history: dict,
                                save_path: Optional[str] = None) -> plt.Figure:
        """用户状态时间线 - 甘特图风格
        
        创新点：结合状态转换和SOC变化
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), 
                                        gridspec_kw={'height_ratios': [1, 2]},
                                        sharex=True)
        
        time = np.array(history['time'])
        states = np.array(history['user_state'])
        soc = np.array(history['SOC']) * 100
        
        state_names = ['Sleep', 'Work', 'Leisure', 'Heavy Use']
        
        # 绘制状态时间线
        for i in range(len(time) - 1):
            state = states[i]
            ax1.barh(0, time[i+1] - time[i], left=time[i], 
                    color=STATE_COLORS[state], height=0.8, alpha=0.8)
        
        ax1.set_yticks([])
        ax1.set_title('User Activity State Timeline', fontsize=13, fontweight='bold')
        
        # 添加图例
        legend_patches = [mpatches.Patch(color=STATE_COLORS[i], label=state_names[i], alpha=0.8)
                         for i in range(4)]
        ax1.legend(handles=legend_patches, loc='upper right', ncol=4, fontsize=9)
        
        # 绘制SOC曲线，颜色随状态变化
        for i in range(len(time) - 1):
            state = states[i]
            ax2.fill_between(time[i:i+2], 0, soc[i:i+2], 
                            color=STATE_COLORS[state], alpha=0.3)
            ax2.plot(time[i:i+2], soc[i:i+2], 
                    color=STATE_COLORS[state], linewidth=2)
        
        ax2.set_xlabel('Time (hours)', fontsize=12)
        ax2.set_ylabel('SOC (%)', fontsize=12)
        ax2.set_title('SOC Evolution with User State', fontsize=13, fontweight='bold')
        ax2.set_ylim(0, 105)
        ax2.grid(True, alpha=0.3)
        
        # 计算各状态下的平均放电率
        discharge_rates = {}
        for state in range(4):
            mask = states[:-1] == state
            if mask.sum() > 0:
                dt = np.diff(time)[mask]
                dsoc = np.diff(soc)[mask]
                avg_rate = -np.mean(dsoc / dt) if dt.sum() > 0 else 0
                discharge_rates[state_names[state]] = avg_rate
        
        # 添加统计信息文本框
        stats_text = "Avg Discharge Rate:\n"
        for name, rate in discharge_rates.items():
            stats_text += f"  {name}: {rate:.1f}%/h\n"
        
        ax2.text(0.02, 0.02, stats_text, transform=ax2.transAxes,
                fontsize=9, verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_remaining_time_surface(self, model, 
                                   save_path: Optional[str] = None) -> plt.Figure:
        """剩余时间预测3D曲面图
        
        创新点：展示SOC、功耗与剩余时间的三维关系
        """
        fig = plt.figure(figsize=(14, 10))
        
        # 创建数据网格
        soc_range = np.linspace(0.05, 1.0, 50)
        power_range = np.linspace(0.2, 4.0, 50)
        SOC, POWER = np.meshgrid(soc_range, power_range)
        
        # 计算剩余时间
        Q_eff = model.batt_params.Q_max
        V_avg = 3.7
        eta = model.batt_params.eta_PMIC
        
        I_avg = POWER / (eta * V_avg)
        REMAINING_TIME = (SOC * Q_eff) / I_avg
        
        # 3D曲面图
        ax1 = fig.add_subplot(121, projection='3d')
        
        surf = ax1.plot_surface(SOC * 100, POWER, REMAINING_TIME,
                               cmap=create_gradient_colormap('battery'),
                               alpha=0.9, linewidth=0, antialiased=True)
        
        ax1.set_xlabel('SOC (%)', fontsize=11)
        ax1.set_ylabel('Power (W)', fontsize=11)
        ax1.set_zlabel('Remaining Time (h)', fontsize=11)
        ax1.set_title('Remaining Time Prediction Surface', fontsize=13, fontweight='bold')
        ax1.view_init(elev=25, azim=45)
        
        fig.colorbar(surf, ax=ax1, shrink=0.5, label='Time (h)')
        
        # 2D等高线图
        ax2 = fig.add_subplot(122)
        
        levels = [0.5, 1, 2, 4, 6, 8, 10, 15, 20, 30]
        contour = ax2.contourf(SOC * 100, POWER, REMAINING_TIME, 
                              levels=levels, cmap=create_gradient_colormap('battery'))
        ax2.contour(SOC * 100, POWER, REMAINING_TIME, levels=levels, 
                   colors='white', linewidths=0.5, alpha=0.5)
        
        # 添加典型使用场景标记
        scenarios = {
            'Idle': (80, 0.3),
            'Normal': (60, 1.0),
            'Work': (50, 1.5),
            'Heavy': (40, 3.0),
        }
        
        for name, (soc_val, power_val) in scenarios.items():
            ax2.plot(soc_val, power_val, 'ko', markersize=10)
            ax2.annotate(name, (soc_val, power_val), 
                        textcoords="offset points", xytext=(5, 5),
                        fontsize=9, fontweight='bold')
        
        ax2.set_xlabel('SOC (%)', fontsize=11)
        ax2.set_ylabel('Power (W)', fontsize=11)
        ax2.set_title('Remaining Time Contour Map', fontsize=13, fontweight='bold')
        
        cbar = fig.colorbar(contour, ax=ax2)
        cbar.set_label('Remaining Time (hours)', fontsize=10)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_scenario_comparison(self, histories: Dict[str, dict],
                                save_path: Optional[str] = None) -> plt.Figure:
        """多场景对比图
        
        创新点：雷达图结合时间曲线对比
        """
        fig = plt.figure(figsize=(16, 10))
        gs = GridSpec(2, 3, figure=fig)
        
        scenario_colors = {
            'idle': '#2ECC71',
            'work': '#3498DB',
            'leisure': '#9B59B6',
            'heavy': '#E74C3C',
            'normal': '#F39C12',
        }
        
        # 子图1：SOC对比曲线
        ax1 = fig.add_subplot(gs[0, :2])
        
        for scenario, history in histories.items():
            time = np.array(history['time'])
            soc = np.array(history['SOC']) * 100
            color = scenario_colors.get(scenario, '#888888')
            ax1.plot(time, soc, label=scenario.capitalize(), 
                    color=color, linewidth=2.5)
        
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('SOC (%)', fontsize=11)
        ax1.set_title('SOC Comparison Across Scenarios', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, 105)
        
        # 子图2：雷达图 - 场景特性对比
        ax2 = fig.add_subplot(gs[0, 2], projection='polar')
        
        categories = ['Battery Life', 'Avg Power', 'Max Temp', 'SOC Stability', 'Efficiency']
        num_vars = len(categories)
        angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
        angles += angles[:1]
        
        for scenario, history in histories.items():
            # 计算各指标
            soc = np.array(history['SOC'])
            power = np.array(history['P_total'])
            temp = np.array(history['T_batt'])
            time = np.array(history['time'])
            
            # 归一化指标 (0-1范围)
            battery_life = max(0, min(1, time[-1] / 24))  # 相对于24小时
            avg_power = max(0, min(1, 1 - np.mean(power) / 5))  # 功耗越低越好
            max_temp = max(0, min(1, 1 - (np.max(temp) - 20) / 40))  # 温度越低越好
            stability = max(0, min(1, 1 - np.std(np.diff(soc)) * 100))  # 变化越稳定越好
            efficiency = max(0, min(1, soc[-1] * battery_life))  # 综合效率
            
            values = [battery_life, avg_power, max_temp, stability, efficiency]
            values += values[:1]
            
            color = scenario_colors.get(scenario, '#888888')
            ax2.plot(angles, values, 'o-', linewidth=2, label=scenario.capitalize(), color=color)
            ax2.fill(angles, values, alpha=0.15, color=color)
        
        ax2.set_xticks(angles[:-1])
        ax2.set_xticklabels(categories, fontsize=9)
        ax2.set_title('Scenario Characteristics', fontsize=12, fontweight='bold', pad=20)
        ax2.legend(loc='upper right', bbox_to_anchor=(1.3, 1), fontsize=9)
        
        # 子图3：功耗对比柱状图
        ax3 = fig.add_subplot(gs[1, 0])
        
        scenarios_list = list(histories.keys())
        avg_powers = [np.mean(histories[s]['P_total']) for s in scenarios_list]
        colors = [scenario_colors.get(s, '#888888') for s in scenarios_list]
        
        bars = ax3.bar(range(len(scenarios_list)), avg_powers, color=colors, alpha=0.8)
        ax3.set_xticks(range(len(scenarios_list)))
        ax3.set_xticklabels([s.capitalize() for s in scenarios_list], fontsize=10)
        ax3.set_ylabel('Average Power (W)', fontsize=11)
        ax3.set_title('Average Power by Scenario', fontsize=12, fontweight='bold')
        
        for bar, power in zip(bars, avg_powers):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                    f'{power:.2f}W', ha='center', va='bottom', fontsize=9)
        
        # 子图4：电池寿命对比
        ax4 = fig.add_subplot(gs[1, 1])
        
        drain_times = []
        for s in scenarios_list:
            soc = np.array(histories[s]['SOC'])
            time = np.array(histories[s]['time'])
            idx = np.where(soc <= 0.05)[0]
            drain_time = time[idx[0]] if len(idx) > 0 else time[-1]
            drain_times.append(drain_time)
        
        bars2 = ax4.barh(range(len(scenarios_list)), drain_times, color=colors, alpha=0.8)
        ax4.set_yticks(range(len(scenarios_list)))
        ax4.set_yticklabels([s.capitalize() for s in scenarios_list], fontsize=10)
        ax4.set_xlabel('Battery Life (hours)', fontsize=11)
        ax4.set_title('Battery Drain Time', fontsize=12, fontweight='bold')
        
        for bar, time_val in zip(bars2, drain_times):
            ax4.text(bar.get_width() + 0.2, bar.get_y() + bar.get_height()/2,
                    f'{time_val:.1f}h', ha='left', va='center', fontsize=9)
        
        # 子图5：温度对比箱线图
        ax5 = fig.add_subplot(gs[1, 2])
        
        temp_data = [histories[s]['T_batt'] for s in scenarios_list]
        bp = ax5.boxplot(temp_data, labels=[s.capitalize() for s in scenarios_list],
                        patch_artist=True)
        
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        
        ax5.set_ylabel('Temperature (°C)', fontsize=11)
        ax5.set_title('Temperature Distribution', fontsize=12, fontweight='bold')
        ax5.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_energy_flow_diagram(self, history: dict, time_index: int = -1,
                                save_path: Optional[str] = None) -> plt.Figure:
        """能量流动图 - 桑基图风格
        
        创新点：展示能量从电池到各组件的流动
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # 获取指定时刻的功耗数据
        p_components = history['P_components'][time_index]
        total_power = history['P_total'][time_index]
        v_batt = history['V_batt'][time_index]
        soc = history['SOC'][time_index] * 100
        
        # 定义节点位置
        left_x = 0.1
        middle_x = 0.5
        right_x = 0.9
        
        # 绘制电池节点
        battery_height = 0.4
        battery = mpatches.FancyBboxPatch((left_x - 0.08, 0.3), 0.16, battery_height,
                                          boxstyle="round,pad=0.02",
                                          facecolor='#3498DB', edgecolor='#2C3E50',
                                          linewidth=2, alpha=0.9)
        ax.add_patch(battery)
        ax.text(left_x, 0.5, f'Battery\n{soc:.0f}%\n{v_batt:.2f}V',
               ha='center', va='center', fontsize=11, fontweight='bold', color='white')
        
        # 绘制PMIC节点
        pmic = mpatches.FancyBboxPatch((middle_x - 0.06, 0.4), 0.12, 0.2,
                                       boxstyle="round,pad=0.02",
                                       facecolor='#9B59B6', edgecolor='#2C3E50',
                                       linewidth=2, alpha=0.9)
        ax.add_patch(pmic)
        ax.text(middle_x, 0.5, f'PMIC\n{total_power:.2f}W',
               ha='center', va='center', fontsize=10, fontweight='bold', color='white')
        
        # 绘制各组件节点和连接
        components = list(p_components.keys())
        n_comps = len(components)
        comp_y_positions = np.linspace(0.1, 0.9, n_comps)
        
        for i, (comp, y_pos) in enumerate(zip(components, comp_y_positions)):
            power = p_components[comp]
            ratio = power / total_power if total_power > 0 else 0
            
            # 组件节点
            color = COMPONENT_COLORS.get(comp, '#888888')
            node_height = max(0.05, 0.15 * ratio + 0.03)
            
            node = mpatches.FancyBboxPatch((right_x - 0.08, y_pos - node_height/2),
                                           0.16, node_height,
                                           boxstyle="round,pad=0.01",
                                           facecolor=color, edgecolor='#2C3E50',
                                           linewidth=1, alpha=0.8)
            ax.add_patch(node)
            
            # 组件标签
            label = comp.replace('P_', '').upper()
            ax.text(right_x, y_pos, f'{label}\n{power:.3f}W\n({ratio*100:.1f}%)',
                   ha='center', va='center', fontsize=8, fontweight='bold')
            
            # 绘制流动箭头
            arrow_width = max(0.002, 0.02 * ratio)
            arrow = mpatches.FancyArrowPatch(
                (middle_x + 0.06, 0.5),
                (right_x - 0.08, y_pos),
                arrowstyle='->,head_length=0.15,head_width=0.1',
                mutation_scale=15,
                color=color,
                linewidth=arrow_width * 100,
                alpha=0.7
            )
            ax.add_patch(arrow)
        
        # 电池到PMIC的主连接
        main_arrow = mpatches.FancyArrowPatch(
            (left_x + 0.08, 0.5),
            (middle_x - 0.06, 0.5),
            arrowstyle='->,head_length=0.2,head_width=0.15',
            mutation_scale=20,
            color='#2C3E50',
            linewidth=4,
            alpha=0.8
        )
        ax.add_patch(main_arrow)
        
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title('Energy Flow Diagram', fontsize=14, fontweight='bold', pad=20)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_voltage_current_characteristics(self, history: dict,
                                            save_path: Optional[str] = None) -> plt.Figure:
        """电压-电流特性曲线
        
        创新点：展示V-I关系随SOC变化
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        
        time = np.array(history['time'])
        voltage = np.array(history['V_batt'])
        current = np.array(history['I_total'])
        soc = np.array(history['SOC']) * 100
        power = np.array(history['P_total'])
        
        # 子图1：V-I散点图
        ax1 = axes[0, 0]
        scatter = ax1.scatter(current, voltage, c=soc, cmap=create_gradient_colormap('battery'),
                             s=20, alpha=0.6)
        ax1.set_xlabel('Current (A)', fontsize=11)
        ax1.set_ylabel('Voltage (V)', fontsize=11)
        ax1.set_title('Voltage-Current Characteristics', fontsize=12, fontweight='bold')
        cbar = plt.colorbar(scatter, ax=ax1)
        cbar.set_label('SOC (%)', fontsize=10)
        ax1.grid(True, alpha=0.3)
        
        # 添加趋势线
        z = np.polyfit(current, voltage, 1)
        p = np.poly1d(z)
        ax1.plot(sorted(current), p(sorted(current)), 'r--', linewidth=2, 
                alpha=0.7, label=f'V = {z[1]:.2f} - {-z[0]:.3f}·I')
        ax1.legend(fontsize=9)
        
        # 子图2：电压随时间变化
        ax2 = axes[0, 1]
        ax2.plot(time, voltage, 'b-', linewidth=2, label='Voltage')
        ax2.set_xlabel('Time (hours)', fontsize=11)
        ax2.set_ylabel('Voltage (V)', fontsize=11, color='blue')
        ax2.tick_params(axis='y', labelcolor='blue')
        
        ax2_twin = ax2.twinx()
        ax2_twin.plot(time, current, 'r-', linewidth=2, label='Current', alpha=0.7)
        ax2_twin.set_ylabel('Current (A)', fontsize=11, color='red')
        ax2_twin.tick_params(axis='y', labelcolor='red')
        
        ax2.set_title('Voltage & Current over Time', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        # 子图3：功率因子分析
        ax3 = axes[1, 0]
        power_factor = voltage * current / power
        power_factor = np.clip(power_factor, 0, 2)
        
        ax3.plot(time, power_factor, 'g-', linewidth=2)
        ax3.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='Ideal')
        ax3.set_xlabel('Time (hours)', fontsize=11)
        ax3.set_ylabel('Power Transfer Efficiency', fontsize=11)
        ax3.set_title('Power Transfer Efficiency over Time', fontsize=12, fontweight='bold')
        ax3.legend(fontsize=10)
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(0.5, 1.5)
        
        # 子图4：能量累积曲线
        ax4 = axes[1, 1]
        dt = np.diff(time) if len(time) > 1 else [1]
        energy_consumed = np.cumsum(power[:-1] * np.array(dt)) if len(dt) > 0 else [0]
        
        ax4.fill_between(time[:-1], 0, energy_consumed, alpha=0.3, color='#3498DB')
        ax4.plot(time[:-1], energy_consumed, 'b-', linewidth=2)
        ax4.set_xlabel('Time (hours)', fontsize=11)
        ax4.set_ylabel('Cumulative Energy (Wh)', fontsize=11)
        ax4.set_title('Cumulative Energy Consumption', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        # 添加总能量标注
        if len(energy_consumed) > 0:
            total_energy = energy_consumed[-1]
            ax4.annotate(f'Total: {total_energy:.2f} Wh',
                        xy=(time[-2], energy_consumed[-1]),
                        xytext=(time[-2] * 0.7, energy_consumed[-1] * 0.8),
                        fontsize=11, fontweight='bold',
                        arrowprops=dict(arrowstyle='->', color='#2C3E50'))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def create_dashboard(self, history: dict, model,
                        save_path: Optional[str] = None) -> plt.Figure:
        """创建综合仪表板
        
        创新点：一图展示所有关键指标
        """
        fig = plt.figure(figsize=(20, 14))
        gs = GridSpec(3, 4, figure=fig, hspace=0.35, wspace=0.3)
        
        time = np.array(history['time'])
        soc = np.array(history['SOC']) * 100
        temp = np.array(history['T_batt'])
        power = np.array(history['P_total'])
        voltage = np.array(history['V_batt'])
        states = np.array(history['user_state'])
        
        # 1. 主SOC曲线 (大图)
        ax1 = fig.add_subplot(gs[0, :2])
        points = np.array([time, soc]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=create_gradient_colormap('battery'),
                           norm=Normalize(0, 100), linewidth=4)
        lc.set_array(soc[:-1])
        ax1.add_collection(lc)
        ax1.axhspan(0, 20, alpha=0.15, color='#FF6B6B')
        ax1.set_xlim(time.min(), time.max())
        ax1.set_ylim(0, 105)
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('SOC (%)', fontsize=11)
        ax1.set_title('🔋 State of Charge', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # 2. 功耗堆叠图
        ax2 = fig.add_subplot(gs[0, 2:])
        components = ['P_SoC', 'P_display', 'P_5G', 'P_WiFi', 'P_BT', 'P_GNSS', 'P_background']
        power_data = {comp: [p.get(comp, 0) for p in history['P_components']] for comp in components}
        stack_data = np.vstack([power_data[comp] for comp in components])
        colors = [COMPONENT_COLORS[comp] for comp in components]
        ax2.stackplot(time, stack_data, labels=components, colors=colors, alpha=0.85)
        ax2.set_xlabel('Time (hours)', fontsize=11)
        ax2.set_ylabel('Power (W)', fontsize=11)
        ax2.set_title('⚡ Power Consumption', fontsize=13, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=8, ncol=2)
        ax2.set_xlim(time.min(), time.max())
        
        # 3. 温度仪表
        ax3 = fig.add_subplot(gs[1, 0])
        current_temp = temp[-1]
        theta = np.linspace(0, np.pi, 100)
        r_inner, r_outer = 0.6, 1.0
        
        ax3.fill_between(theta, r_inner, r_outer, alpha=0.1, color='gray')
        temp_angle = np.pi * (1 - (current_temp - 20) / 40)
        temp_angle = np.clip(temp_angle, 0, np.pi)
        ax3.fill_between(np.linspace(temp_angle, np.pi, 50), r_inner, r_outer,
                        alpha=0.8, color='#3498DB' if current_temp < 35 else '#E74C3C')
        ax3.plot([0, np.cos(temp_angle)], [0, np.sin(temp_angle)], 'k-', linewidth=3)
        ax3.set_xlim(-1.2, 1.2)
        ax3.set_ylim(-0.2, 1.2)
        ax3.set_aspect('equal')
        ax3.axis('off')
        ax3.set_title(f'🌡️ Temperature: {current_temp:.1f}°C', fontsize=12, fontweight='bold')
        
        # 4. 电压指示
        ax4 = fig.add_subplot(gs[1, 1])
        current_v = voltage[-1]
        v_range = [3.0, 4.2]
        v_pct = (current_v - v_range[0]) / (v_range[1] - v_range[0])
        
        ax4.barh(0, v_pct, height=0.5, color='#2ECC71' if v_pct > 0.3 else '#E74C3C', alpha=0.8)
        ax4.barh(0, 1, height=0.5, color='gray', alpha=0.2)
        ax4.set_xlim(0, 1)
        ax4.set_ylim(-0.5, 0.5)
        ax4.set_yticks([])
        ax4.set_xticks([0, 0.5, 1])
        ax4.set_xticklabels([f'{v_range[0]:.1f}V', f'{(v_range[0]+v_range[1])/2:.1f}V', f'{v_range[1]:.1f}V'])
        ax4.set_title(f'⚡ Voltage: {current_v:.2f}V', fontsize=12, fontweight='bold')
        
        # 5. 用户状态分布
        ax5 = fig.add_subplot(gs[1, 2])
        state_counts = [np.sum(states == i) for i in range(4)]
        state_names = ['Sleep', 'Work', 'Leisure', 'Heavy']
        colors_state = [STATE_COLORS[i] for i in range(4)]
        wedges, texts, autotexts = ax5.pie(state_counts, labels=state_names,
                                           colors=colors_state, autopct='%1.0f%%',
                                           startangle=90, explode=[0.02]*4)
        ax5.set_title('👤 User State Distribution', fontsize=12, fontweight='bold')
        
        # 6. 剩余时间预测
        ax6 = fig.add_subplot(gs[1, 3])
        current_soc_val = soc[-1] / 100
        predictions = {}
        for scenario in ['idle', 'normal', 'heavy']:
            remaining, _ = model.predict_remaining_time(current_soc_val, scenario)
            predictions[scenario] = remaining
        
        bars = ax6.bar(predictions.keys(), predictions.values(),
                      color=['#2ECC71', '#F39C12', '#E74C3C'], alpha=0.8)
        ax6.set_ylabel('Hours', fontsize=11)
        ax6.set_title(f'⏱️ Remaining Time (SOC={current_soc_val*100:.0f}%)', 
                     fontsize=12, fontweight='bold')
        for bar, val in zip(bars, predictions.values()):
            ax6.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{val:.1f}h', ha='center', fontsize=10)
        
        # 7. 关键统计数据
        ax7 = fig.add_subplot(gs[2, :2])
        ax7.axis('off')
        
        stats_data = [
            ['Metric', 'Value', 'Status'],
            ['Initial SOC', f'{history["SOC"][0]*100:.0f}%', '✓'],
            ['Final SOC', f'{soc[-1]:.0f}%', '⚠️' if soc[-1] < 20 else '✓'],
            ['Avg Power', f'{np.mean(power):.2f} W', '✓'],
            ['Max Power', f'{np.max(power):.2f} W', '⚠️' if np.max(power) > 3 else '✓'],
            ['Avg Temp', f'{np.mean(temp):.1f}°C', '✓'],
            ['Max Temp', f'{np.max(temp):.1f}°C', '⚠️' if np.max(temp) > 40 else '✓'],
            ['Runtime', f'{time[-1]:.1f} hours', '✓'],
        ]
        
        table = ax7.table(cellText=stats_data[1:], colLabels=stats_data[0],
                         loc='center', cellLoc='center',
                         colColours=['#E8E8E8']*3)
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1.2, 1.8)
        ax7.set_title('📊 Key Statistics', fontsize=13, fontweight='bold', pad=20)
        
        # 8. 功耗影响因子排名
        ax8 = fig.add_subplot(gs[2, 2:])
        
        avg_power_by_comp = {comp: np.mean([p.get(comp, 0) for p in history['P_components']])
                           for comp in components}
        sorted_comps = sorted(avg_power_by_comp.items(), key=lambda x: x[1], reverse=True)
        
        comp_names = [c[0].replace('P_', '').upper() for c in sorted_comps]
        comp_values = [c[1] for c in sorted_comps]
        comp_colors = [COMPONENT_COLORS[c[0]] for c in sorted_comps]
        
        bars = ax8.barh(range(len(comp_names)), comp_values, color=comp_colors, alpha=0.8)
        ax8.set_yticks(range(len(comp_names)))
        ax8.set_yticklabels(comp_names)
        ax8.set_xlabel('Average Power (W)', fontsize=11)
        ax8.set_title('📈 Power Impact Ranking', fontsize=13, fontweight='bold')
        ax8.invert_yaxis()
        
        for bar, val in zip(bars, comp_values):
            ax8.text(bar.get_width() + 0.01, bar.get_y() + bar.get_height()/2,
                    f'{val:.3f}W', ha='left', va='center', fontsize=9)
        
        plt.suptitle('Smartphone Battery Model Dashboard', fontsize=16, fontweight='bold', y=0.98)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig


def generate_all_visualizations(model, histories: Dict[str, dict], output_dir: str = './'):
    """生成所有可视化图表
    
    Args:
        model: SmartphoneBatteryModel实例
        histories: 各场景的历史数据字典
        output_dir: 输出目录
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    viz = BatteryVisualizer()
    
    # 选择一个代表性的历史记录
    main_history = histories.get('normal', list(histories.values())[0])
    
    print("Generating visualizations...")
    
    # 1. SOC放电曲线
    print("  1/8 SOC discharge curve...")
    viz.plot_soc_discharge_curve(main_history, 
                                save_path=f'{output_dir}/01_soc_discharge.png')
    
    # 2. 功耗分解图
    print("  2/8 Power breakdown...")
    viz.plot_power_breakdown_sankey_style(main_history,
                                         save_path=f'{output_dir}/02_power_breakdown.png')
    
    # 3. 热行为
    print("  3/8 Thermal behavior...")
    viz.plot_thermal_behavior(main_history,
                             save_path=f'{output_dir}/03_thermal_behavior.png')
    
    # 4. 用户状态时间线
    print("  4/8 User state timeline...")
    viz.plot_user_state_timeline(main_history,
                                save_path=f'{output_dir}/04_user_state.png')
    
    # 5. 剩余时间预测曲面
    print("  5/8 Remaining time surface...")
    viz.plot_remaining_time_surface(model,
                                   save_path=f'{output_dir}/05_remaining_time.png')
    
    # 6. 场景对比
    print("  6/8 Scenario comparison...")
    if len(histories) > 1:
        viz.plot_scenario_comparison(histories,
                                    save_path=f'{output_dir}/06_scenario_comparison.png')
    
    # 7. 能量流动图
    print("  7/8 Energy flow diagram...")
    viz.plot_energy_flow_diagram(main_history,
                                save_path=f'{output_dir}/07_energy_flow.png')
    
    # 8. 电压电流特性
    print("  8/8 V-I characteristics...")
    viz.plot_voltage_current_characteristics(main_history,
                                            save_path=f'{output_dir}/08_vi_characteristics.png')
    
    # 9. 综合仪表板
    print("  9/9 Dashboard...")
    viz.create_dashboard(main_history, model,
                        save_path=f'{output_dir}/09_dashboard.png')
    
    print(f"All visualizations saved to {output_dir}")
    
    return viz


if __name__ == "__main__":
    # 测试可视化
    from battery_model import SmartphoneBatteryModel
    
    model = SmartphoneBatteryModel()
    
    # 运行不同场景模拟
    scenarios = ['idle', 'work', 'leisure', 'heavy', 'normal']
    histories = {}
    
    for scenario in scenarios:
        print(f"Simulating {scenario} scenario...")
        histories[scenario] = model.simulate(
            duration_hours=12,
            initial_soc=1.0,
            scenario=scenario,
            start_hour=8.0
        )
    
    # 生成所有可视化
    generate_all_visualizations(model, histories, output_dir='./visualization_output')
