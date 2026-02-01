"""
智能手机电池模型可视化模块
Smartphone Battery Model Visualization Module

提供美观、现代的可视化图表：
1. SOC放电曲线
2. 功耗分解饼图和条形图
3. 剩余时间预测对比
4. 温度和老化效应
5. 卡尔曼滤波效果展示
6. 敏感性分析热力图
7. 不确定性区间图
8. 多场景对比图
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.ticker import PercentFormatter
import matplotlib.font_manager as fm
from typing import List, Dict, Tuple, Optional
import warnings

# 尝试设置中文字体
try:
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Noto Sans CJK SC']
    plt.rcParams['axes.unicode_minus'] = False
except:
    pass

# 现代配色方案
COLORS = {
    'primary': '#2196F3',      # 蓝色
    'secondary': '#FF9800',    # 橙色
    'success': '#4CAF50',      # 绿色
    'danger': '#F44336',       # 红色
    'warning': '#FFC107',      # 黄色
    'info': '#00BCD4',         # 青色
    'dark': '#37474F',         # 深灰
    'light': '#ECEFF1',        # 浅灰
    'purple': '#9C27B0',       # 紫色
    'pink': '#E91E63',         # 粉色
    'teal': '#009688',         # 青绿
    'indigo': '#3F51B5'        # 靛蓝
}

# 组件颜色映射
COMPONENT_COLORS = {
    'screen': '#FF6B6B',
    'cpu': '#4ECDC4',
    'gpu': '#45B7D1',
    'network': '#96CEB4',
    'gps': '#FFEAA7',
    'other': '#DDA0DD',
    'total': '#2C3E50'
}

# 渐变色映射
GRADIENT_COLORS = ['#667eea', '#764ba2']


class BatteryVisualizer:
    """
    电池模型可视化器
    
    提供多种美观的可视化方法
    """
    
    def __init__(self, figsize_base: Tuple[int, int] = (12, 8), dpi: int = 150):
        """
        初始化可视化器
        
        参数:
            figsize_base: 基础图形尺寸
            dpi: 图像分辨率
        """
        self.figsize_base = figsize_base
        self.dpi = dpi
        
        # 设置全局样式
        self._setup_style()
    
    def _setup_style(self):
        """设置matplotlib全局样式"""
        plt.style.use('seaborn-v0_8-whitegrid')
        
        plt.rcParams.update({
            'figure.facecolor': 'white',
            'axes.facecolor': '#FAFAFA',
            'axes.edgecolor': '#E0E0E0',
            'axes.labelsize': 12,
            'axes.titlesize': 14,
            'axes.titleweight': 'bold',
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'legend.fontsize': 10,
            'legend.framealpha': 0.9,
            'grid.alpha': 0.3,
            'grid.color': '#CCCCCC',
            'lines.linewidth': 2,
            'figure.dpi': self.dpi
        })
    
    def _add_gradient_background(self, ax, color1='#667eea', color2='#764ba2', alpha=0.1):
        """添加渐变背景"""
        gradient = np.linspace(0, 1, 256).reshape(1, -1)
        gradient = np.vstack((gradient, gradient))
        
        cmap = LinearSegmentedColormap.from_list('gradient', [color1, color2])
        
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        ax.imshow(gradient, aspect='auto', cmap=cmap, alpha=alpha,
                  extent=[xlim[0], xlim[1], ylim[0], ylim[1]], origin='lower', zorder=0)
    
    def plot_soc_discharge(self, 
                           time: np.ndarray, 
                           soc: np.ndarray,
                           title: str = 'SOC放电曲线',
                           scenario_name: str = None,
                           show_threshold: bool = True,
                           save_path: str = None) -> plt.Figure:
        """
        绘制SOC放电曲线
        
        参数:
            time: 时间数组 (hours)
            soc: SOC数组 (0-1)
            title: 图表标题
            scenario_name: 使用场景名称
            show_threshold: 是否显示低电量阈值
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=self.figsize_base)
        
        # 绘制主曲线
        ax.plot(time, soc * 100, color=COLORS['primary'], linewidth=2.5, 
                label='SOC' if not scenario_name else scenario_name)
        
        # 填充曲线下方区域
        ax.fill_between(time, 0, soc * 100, alpha=0.3, color=COLORS['primary'])
        
        # 添加低电量阈值线
        if show_threshold:
            ax.axhline(y=20, color=COLORS['warning'], linestyle='--', linewidth=1.5, 
                       label='低电量警告 (20%)')
            ax.axhline(y=5, color=COLORS['danger'], linestyle='--', linewidth=1.5,
                       label='关机阈值 (5%)')
        
        # 标记关键点
        if soc[-1] <= 0.05:
            # 标记关机时间
            shutdown_idx = np.where(soc <= 0.05)[0]
            if len(shutdown_idx) > 0:
                shutdown_time = time[shutdown_idx[0]]
                ax.axvline(x=shutdown_time, color=COLORS['danger'], linestyle=':', alpha=0.7)
                ax.annotate(f'关机时间\n{shutdown_time:.1f}h',
                           xy=(shutdown_time, 5),
                           xytext=(shutdown_time + 0.5, 25),
                           fontsize=10,
                           arrowprops=dict(arrowstyle='->', color=COLORS['danger']))
        
        # 设置坐标轴
        ax.set_xlim(0, time[-1])
        ax.set_ylim(0, 105)
        ax.set_xlabel('时间 (小时)', fontsize=12, fontweight='bold')
        ax.set_ylabel('充电状态 SOC (%)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        # 添加网格
        ax.grid(True, alpha=0.3)
        
        # 图例
        ax.legend(loc='upper right', framealpha=0.9)
        
        # 添加电池图标标注
        self._add_battery_icon(ax, soc[0], 0.85, 0.85)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def _add_battery_icon(self, ax, soc: float, x: float, y: float, size: float = 0.1):
        """添加电池图标"""
        # 电池外框
        transform = ax.transAxes
        
        battery_width = 0.08
        battery_height = 0.04
        
        # 外框
        rect = plt.Rectangle((x, y), battery_width, battery_height,
                              fill=False, edgecolor=COLORS['dark'], linewidth=2,
                              transform=transform)
        ax.add_patch(rect)
        
        # 电池头
        head_width = 0.008
        head_height = battery_height * 0.6
        head_y = y + (battery_height - head_height) / 2
        
        head = plt.Rectangle((x + battery_width, head_y), head_width, head_height,
                              fill=True, facecolor=COLORS['dark'],
                              transform=transform)
        ax.add_patch(head)
        
        # 电量填充
        fill_width = battery_width * soc * 0.95
        
        if soc > 0.5:
            fill_color = COLORS['success']
        elif soc > 0.2:
            fill_color = COLORS['warning']
        else:
            fill_color = COLORS['danger']
        
        fill = plt.Rectangle((x + 0.002, y + 0.002), fill_width, battery_height - 0.004,
                              fill=True, facecolor=fill_color, alpha=0.8,
                              transform=transform)
        ax.add_patch(fill)
        
        # 百分比文字
        ax.text(x + battery_width / 2, y + battery_height + 0.01, f'{soc*100:.0f}%',
                ha='center', fontsize=9, fontweight='bold', transform=transform)
    
    def plot_power_breakdown(self,
                             power_breakdown: Dict[str, float],
                             title: str = '功耗分解',
                             save_path: str = None) -> plt.Figure:
        """
        绘制功耗分解图（饼图+条形图组合）
        
        参数:
            power_breakdown: 功耗分解字典
            title: 图表标题
            save_path: 保存路径
        """
        fig = plt.figure(figsize=(14, 6))
        gs = GridSpec(1, 2, width_ratios=[1, 1.2])
        
        # 移除总功耗，只显示组件
        components = {k: v for k, v in power_breakdown.items() if k != 'total'}
        total_power = power_breakdown.get('total', sum(components.values()))
        
        labels = list(components.keys())
        values = list(components.values())
        
        # 标签翻译
        label_map = {
            'screen': '屏幕',
            'cpu': 'CPU',
            'gpu': 'GPU',
            'network': '网络',
            'gps': 'GPS',
            'other': '其他'
        }
        
        labels_cn = [label_map.get(l, l) for l in labels]
        colors = [COMPONENT_COLORS.get(l, COLORS['info']) for l in labels]
        
        # 饼图
        ax1 = fig.add_subplot(gs[0])
        
        # 突出最大功耗组件
        max_idx = np.argmax(values)
        explode = [0.05 if i == max_idx else 0 for i in range(len(values))]
        
        wedges, texts, autotexts = ax1.pie(values, labels=labels_cn, colors=colors,
                                            autopct='%1.1f%%', explode=explode,
                                            shadow=True, startangle=90,
                                            textprops={'fontsize': 10})
        
        # 美化文字
        for autotext in autotexts:
            autotext.set_color('white')
            autotext.set_fontweight('bold')
        
        ax1.set_title('功耗占比', fontsize=14, fontweight='bold')
        
        # 条形图
        ax2 = fig.add_subplot(gs[1])
        
        y_pos = np.arange(len(labels))
        bars = ax2.barh(y_pos, values, color=colors, edgecolor='white', linewidth=1.5)
        
        # 添加数值标签
        for i, (bar, val) in enumerate(zip(bars, values)):
            width = bar.get_width()
            percentage = val / total_power * 100
            ax2.text(width + max(values) * 0.02, bar.get_y() + bar.get_height() / 2,
                     f'{val:.0f} mW ({percentage:.1f}%)',
                     ha='left', va='center', fontsize=10)
        
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels_cn)
        ax2.set_xlabel('功耗 (mW)', fontsize=12)
        ax2.set_title(f'组件功耗明细 (总计: {total_power:.0f} mW)', fontsize=14, fontweight='bold')
        ax2.set_xlim(0, max(values) * 1.4)
        
        # 添加网格
        ax2.grid(True, axis='x', alpha=0.3)
        ax2.set_axisbelow(True)
        
        fig.suptitle(title, fontsize=16, fontweight='bold', y=1.02)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_multi_scenario_comparison(self,
                                       scenarios: Dict[str, Tuple[np.ndarray, np.ndarray]],
                                       title: str = '多场景SOC对比',
                                       save_path: str = None) -> plt.Figure:
        """
        绘制多使用场景SOC对比图
        
        参数:
            scenarios: 场景字典 {name: (time, soc)}
            title: 图表标题
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # 场景颜色
        scenario_colors = [
            '#2196F3', '#4CAF50', '#FF9800', '#F44336', '#9C27B0',
            '#00BCD4', '#795548', '#607D8B'
        ]
        
        for i, (name, (time, soc)) in enumerate(scenarios.items()):
            color = scenario_colors[i % len(scenario_colors)]
            ax.plot(time, soc * 100, label=name, color=color, linewidth=2.5, alpha=0.9)
        
        # 阈值线
        ax.axhline(y=20, color='#FFC107', linestyle='--', linewidth=1.5, 
                   label='低电量警告', alpha=0.7)
        ax.axhline(y=5, color='#F44336', linestyle='--', linewidth=1.5,
                   label='关机阈值', alpha=0.7)
        
        # 设置
        ax.set_xlim(0, max(t[-1] for t, s in scenarios.values()))
        ax.set_ylim(0, 105)
        ax.set_xlabel('时间 (小时)', fontsize=12, fontweight='bold')
        ax.set_ylabel('充电状态 SOC (%)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', ncol=2, framealpha=0.9)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_remaining_time_comparison(self,
                                       scenarios: Dict[str, float],
                                       title: str = '剩余使用时间预测',
                                       save_path: str = None) -> plt.Figure:
        """
        绘制剩余时间对比条形图
        
        参数:
            scenarios: 场景剩余时间字典 {name: hours}
            title: 图表标题
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        
        names = list(scenarios.keys())
        times = list(scenarios.values())
        
        # 颜色渐变
        colors = plt.cm.RdYlGn(np.linspace(0.2, 0.8, len(names)))[::-1]
        
        # 排序
        sorted_indices = np.argsort(times)[::-1]
        names = [names[i] for i in sorted_indices]
        times = [times[i] for i in sorted_indices]
        colors = [colors[i] for i in sorted_indices]
        
        y_pos = np.arange(len(names))
        bars = ax.barh(y_pos, times, color=colors, edgecolor='white', linewidth=1.5)
        
        # 添加数值标签
        for bar, time in zip(bars, times):
            width = bar.get_width()
            hours = int(time)
            minutes = int((time - hours) * 60)
            ax.text(width + max(times) * 0.02, bar.get_y() + bar.get_height() / 2,
                    f'{hours}小时 {minutes}分钟',
                    ha='left', va='center', fontsize=11, fontweight='bold')
        
        ax.set_yticks(y_pos)
        ax.set_yticklabels(names, fontsize=11)
        ax.set_xlabel('预计续航时间 (小时)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        ax.set_xlim(0, max(times) * 1.25)
        
        ax.grid(True, axis='x', alpha=0.3)
        ax.set_axisbelow(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_temperature_effect(self,
                                temperatures: np.ndarray,
                                capacity_factors: np.ndarray,
                                remaining_times: np.ndarray,
                                title: str = '温度对电池性能的影响',
                                save_path: str = None) -> plt.Figure:
        """
        绘制温度效应图
        
        参数:
            temperatures: 温度数组 (°C)
            capacity_factors: 容量因子数组
            remaining_times: 剩余时间数组
            title: 图表标题
            save_path: 保存路径
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # 容量因子曲线
        ax1.plot(temperatures, capacity_factors * 100, color=COLORS['primary'], 
                 linewidth=2.5, marker='o', markersize=6)
        ax1.fill_between(temperatures, 0, capacity_factors * 100, alpha=0.3, 
                        color=COLORS['primary'])
        
        # 标记最优温度区间
        optimal_mask = (temperatures >= 20) & (temperatures <= 30)
        ax1.fill_between(temperatures[optimal_mask], 0, 
                        capacity_factors[optimal_mask] * 100,
                        alpha=0.3, color=COLORS['success'], label='最优温度区间')
        
        ax1.set_xlabel('温度 (°C)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('有效容量因子 (%)', fontsize=12, fontweight='bold')
        ax1.set_title('温度-容量关系', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # 剩余时间曲线
        ax2.plot(temperatures, remaining_times, color=COLORS['secondary'],
                 linewidth=2.5, marker='s', markersize=6)
        ax2.fill_between(temperatures, 0, remaining_times, alpha=0.3,
                        color=COLORS['secondary'])
        
        ax2.set_xlabel('温度 (°C)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('预计续航时间 (小时)', fontsize=12, fontweight='bold')
        ax2.set_title('温度-续航时间关系', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        fig.suptitle(title, fontsize=16, fontweight='bold', y=1.02)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_kalman_filter_results(self,
                                   time: np.ndarray,
                                   true_soc: np.ndarray,
                                   measured_voltage: np.ndarray,
                                   estimated_soc: np.ndarray,
                                   uncertainty: np.ndarray = None,
                                   title: str = '卡尔曼滤波SOC估计',
                                   save_path: str = None) -> plt.Figure:
        """
        绘制卡尔曼滤波结果
        
        参数:
            time: 时间数组
            true_soc: 真实SOC
            measured_voltage: 测量电压
            estimated_soc: 估计SOC
            uncertainty: 估计不确定性（可选）
            title: 图表标题
            save_path: 保存路径
        """
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(3, 1, height_ratios=[2, 1, 1])
        
        # SOC对比图
        ax1 = fig.add_subplot(gs[0])
        
        ax1.plot(time, true_soc * 100, 'b-', linewidth=2, label='真实SOC', alpha=0.8)
        ax1.plot(time, estimated_soc * 100, 'r--', linewidth=2, label='估计SOC', alpha=0.9)
        
        if uncertainty is not None:
            upper = (estimated_soc + 2 * uncertainty) * 100
            lower = (estimated_soc - 2 * uncertainty) * 100
            ax1.fill_between(time, lower, upper, alpha=0.2, color='red',
                            label='95%置信区间')
        
        ax1.set_ylabel('SOC (%)', fontsize=12, fontweight='bold')
        ax1.set_title('SOC估计对比', fontsize=14, fontweight='bold')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # 电压图
        ax2 = fig.add_subplot(gs[1])
        ax2.plot(time, measured_voltage, 'g-', linewidth=1.5, label='测量电压', alpha=0.8)
        ax2.set_ylabel('电压 (V)', fontsize=12, fontweight='bold')
        ax2.set_title('电压测量值', fontsize=14, fontweight='bold')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        # 误差图
        ax3 = fig.add_subplot(gs[2])
        error = (estimated_soc - true_soc) * 100
        ax3.plot(time, error, 'm-', linewidth=1.5)
        ax3.fill_between(time, 0, error, alpha=0.3, color='magenta')
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        
        ax3.set_xlabel('时间', fontsize=12, fontweight='bold')
        ax3.set_ylabel('估计误差 (%)', fontsize=12, fontweight='bold')
        ax3.set_title(f'估计误差 (RMSE: {np.sqrt(np.mean(error**2)):.2f}%)', 
                     fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        fig.suptitle(title, fontsize=16, fontweight='bold', y=0.98)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_sensitivity_heatmap(self,
                                 sensitivity_data: Dict[str, Dict[str, float]],
                                 title: str = '参数敏感性分析',
                                 save_path: str = None) -> plt.Figure:
        """
        绘制敏感性分析热力图
        
        参数:
            sensitivity_data: 敏感性数据字典
            title: 图表标题
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        params = list(sensitivity_data.keys())
        metrics = ['sensitivity', 'mean_power']
        
        # 参数翻译
        param_map = {
            'screen_brightness': '屏幕亮度',
            'cpu_load': 'CPU负载',
            'wifi_active': 'WiFi',
            'cellular_signal': '蜂窝信号',
            'temperature': '环境温度'
        }
        
        params_cn = [param_map.get(p, p) for p in params]
        
        # 创建数据矩阵
        data = np.array([[sensitivity_data[p]['sensitivity'] * 100 for p in params]])
        
        # 绘制热力图
        im = ax.imshow(data, cmap='YlOrRd', aspect='auto')
        
        # 设置坐标轴
        ax.set_xticks(np.arange(len(params)))
        ax.set_xticklabels(params_cn, rotation=45, ha='right', fontsize=11)
        ax.set_yticks([0])
        ax.set_yticklabels(['敏感性指数'], fontsize=11)
        
        # 添加数值标注
        for i in range(len(params)):
            text = ax.text(i, 0, f'{data[0, i]:.1f}',
                          ha='center', va='center', color='white', fontweight='bold', fontsize=12)
        
        # 颜色条
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label('敏感性 (%)', fontsize=11)
        
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_uncertainty_band(self,
                              time: np.ndarray,
                              soc_mean: np.ndarray,
                              soc_percentiles: Dict[str, np.ndarray],
                              title: str = 'SOC预测不确定性区间',
                              save_path: str = None) -> plt.Figure:
        """
        绘制不确定性区间图
        
        参数:
            time: 时间数组
            soc_mean: 平均SOC
            soc_percentiles: 百分位数字典
            title: 图表标题
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # 95%置信区间
        ax.fill_between(time, soc_percentiles['5%'] * 100, soc_percentiles['95%'] * 100,
                       alpha=0.2, color=COLORS['primary'], label='95%置信区间')
        
        # 50%置信区间
        ax.fill_between(time, soc_percentiles['25%'] * 100, soc_percentiles['75%'] * 100,
                       alpha=0.3, color=COLORS['primary'], label='50%置信区间')
        
        # 中位数
        ax.plot(time, soc_percentiles['50%'] * 100, 'b-', linewidth=2.5, 
                label='中位数预测', alpha=0.9)
        
        # 平均值
        ax.plot(time, soc_mean * 100, 'r--', linewidth=2, label='均值预测', alpha=0.8)
        
        # 阈值线
        ax.axhline(y=20, color=COLORS['warning'], linestyle=':', linewidth=1.5, alpha=0.7)
        ax.axhline(y=5, color=COLORS['danger'], linestyle=':', linewidth=1.5, alpha=0.7)
        
        ax.set_xlim(0, time[-1])
        ax.set_ylim(0, 105)
        ax.set_xlabel('时间 (小时)', fontsize=12, fontweight='bold')
        ax.set_ylabel('充电状态 SOC (%)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        ax.legend(loc='upper right', framealpha=0.9)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_comprehensive_dashboard(self,
                                      time: np.ndarray,
                                      soc: np.ndarray,
                                      power_breakdown: Dict[str, float],
                                      scenarios_comparison: Dict[str, float],
                                      temperature_data: Tuple[np.ndarray, np.ndarray],
                                      title: str = '智能手机电池综合分析仪表板',
                                      save_path: str = None) -> plt.Figure:
        """
        绘制综合分析仪表板
        
        参数:
            time, soc: SOC放电数据
            power_breakdown: 功耗分解
            scenarios_comparison: 场景剩余时间对比
            temperature_data: 温度效应数据
            title: 标题
            save_path: 保存路径
        """
        fig = plt.figure(figsize=(18, 12))
        gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # 1. SOC放电曲线
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(time, soc * 100, color=COLORS['primary'], linewidth=2.5)
        ax1.fill_between(time, 0, soc * 100, alpha=0.3, color=COLORS['primary'])
        ax1.axhline(y=20, color=COLORS['warning'], linestyle='--', linewidth=1.5, alpha=0.7)
        ax1.axhline(y=5, color=COLORS['danger'], linestyle='--', linewidth=1.5, alpha=0.7)
        ax1.set_xlabel('时间 (小时)')
        ax1.set_ylabel('SOC (%)')
        ax1.set_title('SOC放电曲线', fontweight='bold')
        ax1.set_ylim(0, 105)
        ax1.grid(True, alpha=0.3)
        
        # 2. 功耗分解饼图
        ax2 = fig.add_subplot(gs[0, 1])
        components = {k: v for k, v in power_breakdown.items() if k != 'total'}
        labels = list(components.keys())
        values = list(components.values())
        
        label_map = {'screen': '屏幕', 'cpu': 'CPU', 'gpu': 'GPU',
                     'network': '网络', 'gps': 'GPS', 'other': '其他'}
        labels_cn = [label_map.get(l, l) for l in labels]
        colors = [COMPONENT_COLORS.get(l, COLORS['info']) for l in labels]
        
        ax2.pie(values, labels=labels_cn, colors=colors, autopct='%1.1f%%',
                shadow=True, startangle=90)
        ax2.set_title(f'功耗分解 (总计: {power_breakdown.get("total", sum(values)):.0f} mW)',
                     fontweight='bold')
        
        # 3. 剩余时间对比
        ax3 = fig.add_subplot(gs[0, 2])
        names = list(scenarios_comparison.keys())
        times = list(scenarios_comparison.values())
        
        colors_bar = plt.cm.RdYlGn(np.linspace(0.2, 0.8, len(names)))[::-1]
        sorted_idx = np.argsort(times)[::-1]
        names = [names[i] for i in sorted_idx]
        times = [times[i] for i in sorted_idx]
        colors_bar = [colors_bar[i] for i in sorted_idx]
        
        y_pos = np.arange(len(names))
        ax3.barh(y_pos, times, color=colors_bar)
        ax3.set_yticks(y_pos)
        ax3.set_yticklabels(names)
        ax3.set_xlabel('续航时间 (小时)')
        ax3.set_title('各场景续航预测', fontweight='bold')
        ax3.grid(True, axis='x', alpha=0.3)
        
        # 4. 温度效应
        ax4 = fig.add_subplot(gs[1, 0])
        temps, factors = temperature_data
        ax4.plot(temps, factors * 100, color=COLORS['secondary'], linewidth=2.5, marker='o')
        ax4.fill_between(temps, 0, factors * 100, alpha=0.3, color=COLORS['secondary'])
        ax4.set_xlabel('温度 (°C)')
        ax4.set_ylabel('有效容量 (%)')
        ax4.set_title('温度-容量关系', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        # 5. 功耗分解条形图
        ax5 = fig.add_subplot(gs[1, 1])
        components = {k: v for k, v in power_breakdown.items() if k != 'total'}
        labels = list(components.keys())
        values = list(components.values())
        labels_cn = [label_map.get(l, l) for l in labels]
        colors = [COMPONENT_COLORS.get(l, COLORS['info']) for l in labels]
        
        ax5.bar(labels_cn, values, color=colors)
        ax5.set_ylabel('功耗 (mW)')
        ax5.set_title('组件功耗明细', fontweight='bold')
        ax5.tick_params(axis='x', rotation=45)
        ax5.grid(True, axis='y', alpha=0.3)
        
        # 6. 文字摘要
        ax6 = fig.add_subplot(gs[1, 2])
        ax6.axis('off')
        
        total_power = power_breakdown.get('total', sum(components.values()))
        max_component = max(components, key=components.get)
        max_power = components[max_component]
        
        summary_text = f"""
        模型摘要
        ─────────────────────
        
        当前功耗: {total_power:.0f} mW
        
        主要耗电组件: {label_map.get(max_component, max_component)}
        ({max_power:.0f} mW, {max_power/total_power*100:.1f}%)
        
        最优温度范围: 20-30°C
        
        预计最长续航: {max(scenarios_comparison.values()):.1f} 小时
        (场景: {names[0]})
        
        预计最短续航: {min(times):.1f} 小时
        (场景: {names[-1]})
        """
        
        ax6.text(0.1, 0.9, summary_text, transform=ax6.transAxes,
                fontsize=11, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='#F5F5F5', alpha=0.9))
        
        fig.suptitle(title, fontsize=18, fontweight='bold', y=0.98)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_ocv_soc_curve(self,
                           soc_range: np.ndarray,
                           ocv_values: np.ndarray,
                           title: str = '开路电压-SOC特性曲线',
                           save_path: str = None) -> plt.Figure:
        """
        绘制OCV-SOC特性曲线
        
        参数:
            soc_range: SOC范围
            ocv_values: 对应OCV值
            title: 标题
            save_path: 保存路径
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        
        ax.plot(soc_range * 100, ocv_values, color=COLORS['primary'], 
                linewidth=2.5, label='OCV-SOC曲线')
        ax.fill_between(soc_range * 100, 3.0, ocv_values, alpha=0.2, 
                       color=COLORS['primary'])
        
        # 标记关键点
        ax.axhline(y=4.2, color=COLORS['success'], linestyle='--', 
                   linewidth=1.5, label='满充电压 (4.2V)', alpha=0.7)
        ax.axhline(y=3.0, color=COLORS['danger'], linestyle='--',
                   linewidth=1.5, label='截止电压 (3.0V)', alpha=0.7)
        
        ax.set_xlabel('充电状态 SOC (%)', fontsize=12, fontweight='bold')
        ax.set_ylabel('开路电压 OCV (V)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        ax.set_xlim(0, 100)
        ax.set_ylim(2.8, 4.4)
        ax.legend(loc='lower right')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig


def create_all_visualizations(model, usage_scenarios, save_dir: str = './figures'):
    """
    创建所有可视化图表
    
    参数:
        model: 电池模型
        usage_scenarios: 使用场景字典
        save_dir: 保存目录
    """
    import os
    os.makedirs(save_dir, exist_ok=True)
    
    viz = BatteryVisualizer()
    
    # 1. 各场景SOC曲线
    scenarios_data = {}
    remaining_times = {}
    
    for name, usage in usage_scenarios.items():
        t, soc = model.simulate(1.0, usage, 24.0, 1000)
        scenarios_data[name] = (t, soc)
        remaining_times[name] = model.analytical_remaining_time(1.0, usage, 0.05)
    
    # 2. 多场景对比
    viz.plot_multi_scenario_comparison(
        scenarios_data,
        title='不同使用场景SOC放电对比',
        save_path=f'{save_dir}/multi_scenario_comparison.png'
    )
    
    # 3. 剩余时间对比
    viz.plot_remaining_time_comparison(
        remaining_times,
        title='各场景预计续航时间',
        save_path=f'{save_dir}/remaining_time_comparison.png'
    )
    
    # 4. 功耗分解
    for name, usage in usage_scenarios.items():
        breakdown = model.power_breakdown(usage)
        viz.plot_power_breakdown(
            breakdown,
            title=f'{name} - 功耗分解',
            save_path=f'{save_dir}/power_breakdown_{name}.png'
        )
    
    print(f"所有图表已保存到 {save_dir}/")


if __name__ == "__main__":
    # 示例使用
    import sys
    sys.path.insert(0, '.')
    
    from battery_model import (
        SmartphoneBatteryModel, 
        create_idle_usage, create_light_usage, 
        create_moderate_usage, create_heavy_usage, create_navigation_usage
    )
    
    print("=" * 60)
    print("电池模型可视化示例")
    print("=" * 60)
    
    # 创建模型和可视化器
    model = SmartphoneBatteryModel()
    viz = BatteryVisualizer()
    
    # 准备数据
    usage = create_moderate_usage()
    t, soc = model.simulate(1.0, usage, 10.0, 500)
    
    # 绘制SOC曲线
    fig1 = viz.plot_soc_discharge(t, soc, scenario_name='中度使用')
    
    # 功耗分解
    breakdown = model.power_breakdown(usage)
    fig2 = viz.plot_power_breakdown(breakdown)
    
    # 多场景对比
    scenarios = {
        '待机': (model.simulate(1.0, create_idle_usage(), 48.0, 500)),
        '轻度使用': (model.simulate(1.0, create_light_usage(), 20.0, 500)),
        '中度使用': (t, soc),
        '重度使用': (model.simulate(1.0, create_heavy_usage(), 8.0, 500)),
        '导航模式': (model.simulate(1.0, create_navigation_usage(), 8.0, 500))
    }
    
    scenarios_dict = {k: v for k, v in scenarios.items()}
    fig3 = viz.plot_multi_scenario_comparison(scenarios_dict)
    
    # 剩余时间对比
    remaining = {
        '待机': model.analytical_remaining_time(1.0, create_idle_usage()),
        '轻度使用': model.analytical_remaining_time(1.0, create_light_usage()),
        '中度使用': model.analytical_remaining_time(1.0, create_moderate_usage()),
        '重度使用': model.analytical_remaining_time(1.0, create_heavy_usage()),
        '导航模式': model.analytical_remaining_time(1.0, create_navigation_usage())
    }
    fig4 = viz.plot_remaining_time_comparison(remaining)
    
    # OCV-SOC曲线
    soc_range = np.linspace(0.01, 1.0, 100)
    ocv_values = np.array([model.open_circuit_voltage(s) for s in soc_range])
    fig5 = viz.plot_ocv_soc_curve(soc_range, ocv_values)
    
    plt.show()
    
    print("\n可视化完成!")
