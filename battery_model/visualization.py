"""
可视化模块
Visualization for Battery Model Analysis

提供:
1. SOC-时间曲线
2. 用户行为状态图
3. 功耗分布图
4. 温度演化图
5. 马尔科夫链状态转移图
6. Pareto前沿可视化

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from typing import Dict, List, Optional
import warnings

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class BatteryModelVisualizer:
    """电池模型可视化器"""
    
    def __init__(self, figsize: tuple = (12, 8), dpi: int = 100):
        """
        初始化可视化器
        
        Parameters:
        -----------
        figsize : tuple
            图形大小
        dpi : int
            分辨率
        """
        self.figsize = figsize
        self.dpi = dpi
        
        # 配色方案
        self.colors = {
            'soc': '#2E86AB',
            'temperature': '#E94F37',
            'power': '#F39237',
            'current': '#1B998B',
            'voltage': '#7B2CBF',
            'sleep': '#C0C0C0',
            'work': '#E8D4B4',
            'leisure': '#B8E0D2'
        }
        
        # 状态颜色
        self.state_colors = {
            0: '#1a1a2e',  # Deep Sleep - 深蓝
            1: '#16537e',  # Light Use - 蓝
            2: '#3c8dad',  # Streaming - 青
            3: '#ff6b6b'   # Gaming - 红
        }
    
    def plot_soc_evolution(self, time_hours: np.ndarray, 
                           soc: np.ndarray,
                           soc_std: np.ndarray = None,
                           ax: plt.Axes = None,
                           title: str = 'SOC Evolution Over Time') -> plt.Axes:
        """
        绘制SOC演化曲线
        
        Parameters:
        -----------
        time_hours : np.ndarray
            时间 (小时)
        soc : np.ndarray
            SOC值
        soc_std : np.ndarray, optional
            SOC标准差 (不确定性)
        ax : plt.Axes, optional
            绑定的坐标轴
        title : str
            图标题
        
        Returns:
        --------
        plt.Axes : 坐标轴对象
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 5), dpi=self.dpi)
        
        # 主曲线
        ax.plot(time_hours, soc * 100, color=self.colors['soc'], 
                linewidth=2, label='SOC')
        
        # 不确定性带
        if soc_std is not None:
            ax.fill_between(time_hours, 
                           (soc - 2*soc_std) * 100, 
                           (soc + 2*soc_std) * 100,
                           color=self.colors['soc'], alpha=0.2,
                           label='95% CI')
        
        # 警告线
        ax.axhline(y=20, color='orange', linestyle='--', 
                   linewidth=1, alpha=0.7, label='Warning (20%)')
        ax.axhline(y=5, color='red', linestyle='--', 
                   linewidth=1, alpha=0.7, label='Critical (5%)')
        
        ax.set_xlabel('Time (hours)', fontsize=11)
        ax.set_ylabel('SOC (%)', fontsize=11)
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_ylim(0, 105)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        return ax
    
    def plot_coupled_dynamics(self, simulation_result: Dict,
                               save_path: str = None) -> plt.Figure:
        """
        绘制耦合动力学图
        
        包含SOC、温度、功耗、电流四个子图
        
        Parameters:
        -----------
        simulation_result : dict
            仿真结果
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize, dpi=self.dpi)
        
        time_hours = simulation_result['time'] / 3600
        
        # SOC
        ax1 = axes[0, 0]
        ax1.plot(time_hours, simulation_result['soc'] * 100, 
                color=self.colors['soc'], linewidth=2)
        ax1.axhline(y=20, color='orange', linestyle='--', alpha=0.5)
        ax1.set_xlabel('Time (hours)')
        ax1.set_ylabel('SOC (%)')
        ax1.set_title('State of Charge', fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # Temperature
        ax2 = axes[0, 1]
        temp_celsius = simulation_result['temperature'] - 273.15
        ax2.plot(time_hours, temp_celsius, 
                color=self.colors['temperature'], linewidth=2)
        ax2.axhline(y=45, color='red', linestyle='--', alpha=0.5, label='Max Safe')
        ax2.set_xlabel('Time (hours)')
        ax2.set_ylabel('Temperature (°C)')
        ax2.set_title('Battery Temperature', fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Power
        ax3 = axes[1, 0]
        ax3.plot(time_hours, simulation_result['power_total'], 
                color=self.colors['power'], linewidth=2)
        ax3.fill_between(time_hours, 0, simulation_result['power_total'],
                        color=self.colors['power'], alpha=0.3)
        ax3.set_xlabel('Time (hours)')
        ax3.set_ylabel('Power (W)')
        ax3.set_title('Total Power Consumption', fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # Current
        ax4 = axes[1, 1]
        ax4.plot(time_hours, simulation_result['current'], 
                color=self.colors['current'], linewidth=2)
        ax4.set_xlabel('Time (hours)')
        ax4.set_ylabel('Current (A)')
        ax4.set_title('Battery Current', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_user_behavior_timeline(self, time_hours: np.ndarray,
                                     states: np.ndarray,
                                     state_names: List[str] = None,
                                     save_path: str = None) -> plt.Figure:
        """
        绘制用户行为时间线
        
        Parameters:
        -----------
        time_hours : np.ndarray
            时间 (小时)
        states : np.ndarray
            状态序列
        state_names : List[str], optional
            状态名称
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        if state_names is None:
            state_names = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), dpi=self.dpi)
        
        # 子图1: 状态时间线
        ax1 = axes[0]
        
        # 绘制时间模式背景
        for start, end, color, label in [
            (0, 7, self.colors['sleep'], 'Sleep'),
            (7, 9, self.colors['leisure'], 'Leisure'),
            (9, 12, self.colors['work'], 'Work'),
            (12, 14, self.colors['leisure'], 'Leisure'),
            (14, 18, self.colors['work'], 'Work'),
            (18, 23, self.colors['leisure'], 'Leisure'),
            (23, 24, self.colors['sleep'], 'Sleep')
        ]:
            ax1.axvspan(start, end, alpha=0.3, color=color)
        
        ax1.stairs(states + 1, np.append(time_hours, time_hours[-1] + 1/60),
                  color='black', linewidth=1.5)
        
        ax1.set_xlim(0, 24)
        ax1.set_ylim(0.5, 4.5)
        ax1.set_yticks([1, 2, 3, 4])
        ax1.set_yticklabels(state_names)
        ax1.set_xlabel('Time of Day (hour)', fontsize=11)
        ax1.set_title('User Behavior State (Markov Chain)', fontsize=12, fontweight='bold')
        ax1.set_xticks(range(0, 25, 2))
        ax1.grid(True, axis='y', alpha=0.3)
        
        # 子图2: 状态分布
        ax2 = axes[1]
        state_counts = np.bincount(states, minlength=4)
        state_ratios = state_counts / len(states) * 100
        
        bars = ax2.bar(state_names, state_ratios, 
                      color=[self.state_colors[i] for i in range(4)])
        ax2.set_ylabel('Time Percentage (%)', fontsize=11)
        ax2.set_title('State Distribution', fontsize=12, fontweight='bold')
        
        for bar, ratio in zip(bars, state_ratios):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                    f'{ratio:.1f}%', ha='center', va='bottom', fontsize=10)
        
        # 子图3: 状态转移热力图
        ax3 = axes[2]
        
        # 计算转移次数
        n_states = 4
        transitions = np.zeros((n_states, n_states))
        for i in range(len(states) - 1):
            transitions[states[i], states[i+1]] += 1
        
        # 归一化为概率
        row_sums = transitions.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1
        transition_probs = transitions / row_sums
        
        im = ax3.imshow(transition_probs, cmap='Blues', vmin=0, vmax=1)
        ax3.set_xticks(range(n_states))
        ax3.set_yticks(range(n_states))
        ax3.set_xticklabels(state_names, rotation=45, ha='right')
        ax3.set_yticklabels(state_names)
        ax3.set_xlabel('To State', fontsize=11)
        ax3.set_ylabel('From State', fontsize=11)
        ax3.set_title('Observed State Transition Probabilities', fontsize=12, fontweight='bold')
        
        # 添加数值标注
        for i in range(n_states):
            for j in range(n_states):
                text = ax3.text(j, i, f'{transition_probs[i, j]:.2f}',
                               ha='center', va='center', fontsize=9,
                               color='white' if transition_probs[i, j] > 0.5 else 'black')
        
        plt.colorbar(im, ax=ax3, label='Probability')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_power_breakdown(self, power_components: Dict[str, np.ndarray],
                              time_hours: np.ndarray = None,
                              save_path: str = None) -> plt.Figure:
        """
        绘制功耗分解图
        
        Parameters:
        -----------
        power_components : dict
            各组件功耗 {'SoC': [...], 'Display': [...], ...}
        time_hours : np.ndarray, optional
            时间
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        fig, axes = plt.subplots(1, 2, figsize=(14, 5), dpi=self.dpi)
        
        # 左图: 堆叠面积图
        ax1 = axes[0]
        
        if time_hours is None:
            time_hours = np.arange(len(list(power_components.values())[0]))
        
        colors_cycle = plt.cm.Set2(np.linspace(0, 1, len(power_components)))
        
        arrays = np.array(list(power_components.values()))
        ax1.stackplot(time_hours, arrays, labels=list(power_components.keys()),
                     colors=colors_cycle, alpha=0.8)
        
        ax1.set_xlabel('Time (hours)', fontsize=11)
        ax1.set_ylabel('Power (W)', fontsize=11)
        ax1.set_title('Power Consumption Breakdown', fontsize=12, fontweight='bold')
        ax1.legend(loc='upper left')
        ax1.grid(True, alpha=0.3)
        
        # 右图: 平均功耗饼图
        ax2 = axes[1]
        
        avg_powers = {k: np.mean(v) for k, v in power_components.items()}
        total_power = sum(avg_powers.values())
        
        sizes = list(avg_powers.values())
        labels = [f'{k}\n{v:.2f}W ({v/total_power*100:.1f}%)' 
                 for k, v in avg_powers.items()]
        
        wedges, texts = ax2.pie(sizes, colors=colors_cycle, startangle=90)
        ax2.legend(wedges, labels, loc='center left', bbox_to_anchor=(1, 0.5))
        ax2.set_title(f'Average Power Distribution\nTotal: {total_power:.2f}W', 
                     fontsize=12, fontweight='bold')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_remaining_time_prediction(self, prediction_result: Dict,
                                        save_path: str = None) -> plt.Figure:
        """
        绘制剩余时间预测结果
        
        Parameters:
        -----------
        prediction_result : dict
            预测结果
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize, dpi=self.dpi)
        
        # 子图1: SOC轨迹
        ax1 = axes[0, 0]
        if 'soc_trace' in prediction_result:
            time = prediction_result.get('time_trace', 
                   np.arange(len(prediction_result['soc_trace'])))
            ax1.plot(time, prediction_result['soc_trace'] * 100,
                    color=self.colors['soc'], linewidth=2)
            ax1.axhline(y=20, color='orange', linestyle='--', alpha=0.7)
            ax1.axhline(y=5, color='red', linestyle='--', alpha=0.7)
        ax1.set_xlabel('Time (hours)')
        ax1.set_ylabel('SOC (%)')
        ax1.set_title('Predicted SOC Trajectory', fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # 子图2: 时间区块消耗
        ax2 = axes[0, 1]
        if 'block_predictions' in prediction_result.get('block_prediction', {}):
            blocks = prediction_result['block_prediction']['block_predictions']
            names = list(blocks.keys())
            consumptions = [b.get('soc_consumed', 0) * 100 for b in blocks.values()]
            
            bars = ax2.barh(names, consumptions, color=self.colors['power'])
            ax2.set_xlabel('SOC Consumed (%)')
            ax2.set_title('SOC Consumption by Time Block', fontweight='bold')
        
        # 子图3: 预测对比
        ax3 = axes[1, 0]
        methods = ['Basic', 'Block-wise', 'Optimized']
        
        basic_h = prediction_result.get('basic_prediction', {}).get('remaining_hours', 0)
        block_h = prediction_result.get('block_prediction', {}).get('remaining_hours', 0)
        opt = prediction_result.get('optimized_prediction', {})
        opt_h = opt.get('remaining_hours', 0)
        opt_err = opt.get('uncertainty', 0)
        
        hours = [basic_h, block_h, opt_h]
        errors = [0, 0, opt_err]
        
        bars = ax3.bar(methods, hours, color=[self.colors['soc'], 
                      self.colors['power'], self.colors['temperature']],
                      yerr=errors, capsize=5)
        ax3.set_ylabel('Remaining Time (hours)')
        ax3.set_title('Prediction Comparison', fontweight='bold')
        
        for bar, h in zip(bars, hours):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{h:.1f}h', ha='center', va='bottom')
        
        # 子图4: 当前状态仪表盘
        ax4 = axes[1, 1]
        current = prediction_result.get('current_state', {})
        soc = current.get('soc', 0.5)
        temp = current.get('temperature_celsius', 25)
        
        # SOC仪表
        theta = np.linspace(0.75*np.pi, 0.25*np.pi, 100)
        r = 0.8
        ax4.plot(r*np.cos(theta), r*np.sin(theta), 'k-', linewidth=3)
        
        # SOC指针
        angle = 0.75*np.pi - soc * 0.5*np.pi
        ax4.arrow(0, 0, 0.6*np.cos(angle), 0.6*np.sin(angle),
                 head_width=0.1, head_length=0.05, fc='red', ec='red')
        
        ax4.text(0, -0.3, f'SOC: {soc:.1%}', ha='center', fontsize=14, fontweight='bold')
        ax4.text(0, -0.5, f'Temp: {temp:.1f}°C', ha='center', fontsize=12)
        
        ax4.set_xlim(-1.2, 1.2)
        ax4.set_ylim(-0.8, 1.2)
        ax4.set_aspect('equal')
        ax4.axis('off')
        ax4.set_title('Current Battery State', fontweight='bold')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_pareto_front(self, pareto_solutions: np.ndarray,
                          pareto_objectives: np.ndarray,
                          recommended_idx: int = None,
                          save_path: str = None) -> plt.Figure:
        """
        绘制Pareto前沿
        
        Parameters:
        -----------
        pareto_solutions : np.ndarray
            Pareto解集
        pareto_objectives : np.ndarray
            目标函数值
        recommended_idx : int, optional
            推荐解索引
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        fig = plt.figure(figsize=(12, 5), dpi=self.dpi)
        
        # 2D投影
        ax1 = fig.add_subplot(121)
        
        # 负号转换回原始目标 (最大化)
        obj1 = -pareto_objectives[:, 0]  # Battery Life
        obj2 = -pareto_objectives[:, 1]  # Performance
        
        scatter = ax1.scatter(obj1, obj2, c=pareto_objectives[:, 2],
                             cmap='RdYlGn_r', s=50, alpha=0.7)
        
        if recommended_idx is not None:
            ax1.scatter(obj1[recommended_idx], obj2[recommended_idx],
                       c='red', s=200, marker='*', label='Recommended')
        
        ax1.set_xlabel('Battery Life (hours)', fontsize=11)
        ax1.set_ylabel('Performance Score', fontsize=11)
        ax1.set_title('Pareto Front (2D Projection)', fontsize=12, fontweight='bold')
        plt.colorbar(scatter, ax=ax1, label='Thermal Score')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 3D视图
        ax2 = fig.add_subplot(122, projection='3d')
        
        ax2.scatter(obj1, obj2, pareto_objectives[:, 2],
                   c=pareto_objectives[:, 2], cmap='RdYlGn_r', s=50, alpha=0.7)
        
        if recommended_idx is not None:
            ax2.scatter([obj1[recommended_idx]], [obj2[recommended_idx]],
                       [pareto_objectives[recommended_idx, 2]],
                       c='red', s=200, marker='*')
        
        ax2.set_xlabel('Battery Life')
        ax2.set_ylabel('Performance')
        ax2.set_zlabel('Thermal')
        ax2.set_title('Pareto Front (3D)', fontsize=12, fontweight='bold')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig
    
    def plot_kalman_filter_results(self, ekf_history: Dict,
                                    true_soc: np.ndarray = None,
                                    save_path: str = None) -> plt.Figure:
        """
        绘制卡尔曼滤波结果
        
        Parameters:
        -----------
        ekf_history : dict
            EKF历史记录
        true_soc : np.ndarray, optional
            真实SOC (如有)
        save_path : str, optional
            保存路径
        
        Returns:
        --------
        plt.Figure : 图形对象
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize, dpi=self.dpi)
        
        soc_est = np.array(ekf_history['soc_est'])
        soc_std = np.array(ekf_history['soc_std'])
        time_idx = np.arange(len(soc_est))
        
        # SOC估计
        ax1 = axes[0, 0]
        ax1.plot(time_idx, soc_est * 100, color=self.colors['soc'], 
                linewidth=2, label='EKF Estimate')
        ax1.fill_between(time_idx, 
                        (soc_est - 2*soc_std) * 100,
                        (soc_est + 2*soc_std) * 100,
                        color=self.colors['soc'], alpha=0.2, label='95% CI')
        
        if true_soc is not None:
            ax1.plot(time_idx, true_soc * 100, 'k--', 
                    linewidth=1.5, label='True SOC')
        
        ax1.set_xlabel('Time Step')
        ax1.set_ylabel('SOC (%)')
        ax1.set_title('SOC Estimation', fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 估计误差 (如有真值)
        ax2 = axes[0, 1]
        if true_soc is not None:
            error = (soc_est - true_soc[:len(soc_est)]) * 100
            ax2.plot(time_idx, error, color='red', linewidth=1)
            ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5)
            ax2.fill_between(time_idx, -2*soc_std*100, 2*soc_std*100,
                            color='gray', alpha=0.2, label='2σ bound')
        ax2.set_xlabel('Time Step')
        ax2.set_ylabel('Error (%)')
        ax2.set_title('Estimation Error', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        # 温度估计
        ax3 = axes[1, 0]
        temp_est = np.array(ekf_history['temp_est']) - 273.15
        ax3.plot(time_idx, temp_est, color=self.colors['temperature'], linewidth=2)
        ax3.set_xlabel('Time Step')
        ax3.set_ylabel('Temperature (°C)')
        ax3.set_title('Temperature Estimation', fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # 内阻估计
        ax4 = axes[1, 1]
        rint_est = np.array(ekf_history['rint_est']) * 1000  # mOhm
        ax4.plot(time_idx, rint_est, color=self.colors['voltage'], linewidth=2)
        ax4.set_xlabel('Time Step')
        ax4.set_ylabel('Internal Resistance (mΩ)')
        ax4.set_title('Internal Resistance Estimation', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches='tight')
        
        return fig


def demo_visualization():
    """演示可视化功能"""
    print("生成演示可视化...")
    
    viz = BatteryModelVisualizer()
    
    # 示例数据
    np.random.seed(42)
    time_hours = np.linspace(0, 24, 1440)
    
    # 模拟SOC
    soc = 1.0 - 0.03 * time_hours + 0.005 * np.sin(2*np.pi*time_hours/12)
    soc = np.clip(soc, 0.05, 1.0)
    soc_std = 0.01 * np.ones_like(soc)
    
    # 模拟用户状态
    states = np.zeros(len(time_hours), dtype=int)
    for i, h in enumerate(time_hours % 24):
        if h < 7:
            states[i] = 0
        elif h < 9:
            states[i] = np.random.choice([1, 2])
        elif h < 18:
            states[i] = np.random.choice([1, 2, 3], p=[0.5, 0.3, 0.2])
        else:
            states[i] = np.random.choice([1, 2, 3], p=[0.3, 0.4, 0.3])
    
    # 绘制SOC演化
    fig1, ax = plt.subplots(figsize=(10, 5))
    viz.plot_soc_evolution(time_hours, soc, soc_std, ax=ax)
    plt.savefig('/workspace/battery_model/demo_soc_evolution.png', dpi=100, bbox_inches='tight')
    plt.close()
    
    # 绘制用户行为时间线
    fig2 = viz.plot_user_behavior_timeline(time_hours, states)
    plt.savefig('/workspace/battery_model/demo_user_behavior.png', dpi=100, bbox_inches='tight')
    plt.close()
    
    print("演示图已保存到 /workspace/battery_model/")


if __name__ == "__main__":
    demo_visualization()
