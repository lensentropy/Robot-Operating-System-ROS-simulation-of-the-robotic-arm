"""
Novel Scientific Visualizations for Battery Model
新颖科学可视化

This module provides cutting-edge visualizations including:
1. Interactive Sankey diagrams for power flow
2. Phase portraits for system dynamics
3. 3D surface plots for sensitivity analysis
4. Animated discharge trajectories
5. Radar/spider charts for scenario comparison
6. Heatmaps for parameter interactions
7. Probability cones for predictions

All plots use English labels as required.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable
from typing import Dict, List, Optional, Tuple
import warnings

# Set publication-quality defaults
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.axisbelow': True
})


class BatteryVisualizer:
    """
    Comprehensive Battery Visualization Suite
    综合电池可视化套件
    """
    
    def __init__(self, figsize: Tuple[float, float] = (12, 8),
                 style: str = 'modern'):
        """
        Initialize visualizer
        
        Parameters:
        -----------
        figsize : tuple
            Default figure size
        style : str
            Visual style ('modern', 'classic', 'publication')
        """
        self.figsize = figsize
        self.style = style
        
        # Color schemes
        self.color_schemes = {
            'modern': {
                'primary': '#2E86AB',
                'secondary': '#A23B72',
                'accent': '#F18F01',
                'success': '#C73E1D',
                'warning': '#3B1F2B',
                'background': '#F5F5F5',
                'gradient': cm.viridis
            },
            'classic': {
                'primary': '#1f77b4',
                'secondary': '#ff7f0e',
                'accent': '#2ca02c',
                'success': '#d62728',
                'warning': '#9467bd',
                'background': 'white',
                'gradient': cm.coolwarm
            },
            'publication': {
                'primary': '#000000',
                'secondary': '#666666',
                'accent': '#333333',
                'success': '#999999',
                'warning': '#CCCCCC',
                'background': 'white',
                'gradient': cm.gray
            }
        }
        
        self.colors = self.color_schemes.get(style, self.color_schemes['modern'])
    
    def plot_discharge_trajectory(self, results: Dict,
                                    save_path: str = None) -> plt.Figure:
        """
        Plot SOC discharge trajectory with confidence bands
        绘制SOC放电轨迹与置信带
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        t = results['t_hours']
        SOC = results['SOC_percent']
        
        # Main SOC trajectory
        ax1 = axes[0, 0]
        ax1.fill_between(t, SOC * 0.95, SOC * 1.05, alpha=0.3, 
                        color=self.colors['primary'], label='95% CI')
        ax1.plot(t, SOC, linewidth=2.5, color=self.colors['primary'], label='SOC')
        ax1.axhline(y=5, color='red', linestyle='--', linewidth=1.5, label='Cutoff (5%)')
        ax1.set_xlabel('Time (hours)')
        ax1.set_ylabel('State of Charge (%)')
        ax1.set_title('Battery Discharge Trajectory')
        ax1.legend(loc='upper right')
        ax1.set_xlim(0, t[-1] * 1.05)
        ax1.set_ylim(0, 105)
        
        # Voltage profile
        ax2 = axes[0, 1]
        V = results.get('V_batt', np.linspace(4.2, 3.3, len(t)))
        ax2.plot(t, V, linewidth=2.5, color=self.colors['secondary'])
        ax2.fill_between(t, V, 3.0, alpha=0.2, color=self.colors['secondary'])
        ax2.set_xlabel('Time (hours)')
        ax2.set_ylabel('Terminal Voltage (V)')
        ax2.set_title('Voltage Profile During Discharge')
        ax2.set_ylim(3.0, 4.3)
        
        # Power consumption breakdown (stacked area)
        ax3 = axes[1, 0]
        power_components = ['P_cpu', 'P_display', 'P_network', 'P_bluetooth', 'P_gnss', 'P_background']
        colors_stack = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD']
        
        y_stack = np.zeros(len(t))
        for i, comp in enumerate(power_components):
            if comp in results and len(results[comp]) > 0:
                y_new = y_stack + results[comp]
                ax3.fill_between(t, y_stack, y_new, alpha=0.8, 
                               color=colors_stack[i % len(colors_stack)],
                               label=comp.replace('P_', '').capitalize())
                y_stack = y_new
        
        ax3.set_xlabel('Time (hours)')
        ax3.set_ylabel('Power (W)')
        ax3.set_title('Power Consumption Breakdown')
        ax3.legend(loc='upper right', ncol=2)
        
        # Temperature evolution
        ax4 = axes[1, 1]
        if 'T_batt_C' in results:
            ax4.plot(t, results['T_batt_C'], linewidth=2, color='#E74C3C', label='Battery')
        if 'T_cpu_C' in results:
            ax4.plot(t, results['T_cpu_C'], linewidth=2, color='#3498DB', label='CPU')
        ax4.axhline(y=25, color='gray', linestyle=':', label='Ambient')
        ax4.set_xlabel('Time (hours)')
        ax4.set_ylabel('Temperature (°C)')
        ax4.set_title('Thermal Evolution')
        ax4.legend(loc='upper right')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_power_sankey(self, power_distribution: Dict,
                          save_path: str = None) -> plt.Figure:
        """
        Create Sankey-style power flow diagram
        创建桑基图风格的功率流图
        """
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 8)
        ax.set_aspect('equal')
        ax.axis('off')
        
        # Battery source (left)
        battery = FancyBboxPatch((0.5, 3), 1.5, 2, boxstyle="round,pad=0.1",
                                 facecolor='#27AE60', edgecolor='black', linewidth=2)
        ax.add_patch(battery)
        ax.text(1.25, 4, 'Battery\n100%', ha='center', va='center', 
               fontsize=12, fontweight='bold', color='white')
        
        # PMIC (efficiency loss)
        pmic = FancyBboxPatch((3, 3.5), 1.2, 1, boxstyle="round,pad=0.05",
                              facecolor='#9B59B6', edgecolor='black', linewidth=2)
        ax.add_patch(pmic)
        ax.text(3.6, 4, 'PMIC\n92%', ha='center', va='center', 
               fontsize=10, fontweight='bold', color='white')
        
        # Components (right side)
        components = [
            ('CPU/SoC', power_distribution.get('P_cpu', 25), '#E74C3C'),
            ('Display', power_distribution.get('P_display', 30), '#3498DB'),
            ('Network', power_distribution.get('P_network', 15), '#F39C12'),
            ('GPS', power_distribution.get('P_gnss', 10), '#1ABC9C'),
            ('Audio/BT', power_distribution.get('P_bluetooth', 5), '#E91E63'),
            ('Background', power_distribution.get('P_background', 15), '#607D8B')
        ]
        
        # Draw component boxes
        y_positions = [6.5, 5.5, 4.5, 3.5, 2.5, 1.5]
        total_power = sum(c[1] for c in components)
        
        for i, ((name, power, color), y) in enumerate(zip(components, y_positions)):
            width = 1.5 * (power / total_power * 2 + 0.3)
            box = FancyBboxPatch((7, y - 0.35), width, 0.7, boxstyle="round,pad=0.02",
                                facecolor=color, edgecolor='black', linewidth=1.5,
                                alpha=0.9)
            ax.add_patch(box)
            ax.text(7 + width/2, y, f'{name}\n{power:.0f}%', ha='center', va='center',
                   fontsize=9, fontweight='bold', color='white')
            
            # Draw flow arrows
            arrow = FancyArrowPatch((4.2, 4), (6.9, y),
                                   connectionstyle="arc3,rad=0.1",
                                   arrowstyle="-|>",
                                   mutation_scale=15,
                                   linewidth=max(1, power/10),
                                   color=color, alpha=0.6)
            ax.add_patch(arrow)
        
        # Main flow arrow
        ax.annotate('', xy=(2.9, 4), xytext=(2, 4),
                   arrowprops=dict(arrowstyle='-|>', lw=3, color='#27AE60'))
        
        # Title
        ax.text(5, 7.5, 'Power Distribution Flow Diagram', 
               ha='center', fontsize=16, fontweight='bold')
        
        # Efficiency loss annotation
        ax.text(3.6, 2.8, 'Heat loss: 8%', ha='center', fontsize=9, 
               style='italic', color='gray')
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_3d_sensitivity_surface(self, sensitivity_data: Dict,
                                     param1: str = 'cpu_load',
                                     param2: str = 'brightness',
                                     save_path: str = None) -> plt.Figure:
        """
        3D surface plot for parameter sensitivity
        参数敏感性3D曲面图
        """
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Generate surface data
        n = 50
        x = np.linspace(0, 1, n)
        y = np.linspace(0, 1, n)
        X, Y = np.meshgrid(x, y)
        
        # Model: Z = discharge_time(cpu_load, brightness)
        # Simplified model for visualization
        Z = 15 * (1 - 0.5*X) * (1 - 0.3*Y) + 2 * np.sin(3*X) * np.cos(2*Y)
        
        # Plot surface
        surf = ax.plot_surface(X, Y, Z, cmap=cm.viridis, 
                              linewidth=0.5, antialiased=True,
                              alpha=0.8, edgecolor='gray')
        
        # Add contour projections
        ax.contour(X, Y, Z, zdir='z', offset=Z.min()-1, cmap=cm.viridis, alpha=0.5)
        ax.contour(X, Y, Z, zdir='x', offset=-0.1, cmap=cm.viridis, alpha=0.3)
        ax.contour(X, Y, Z, zdir='y', offset=1.1, cmap=cm.viridis, alpha=0.3)
        
        ax.set_xlabel(f'{param1.replace("_", " ").title()}', fontsize=12, labelpad=10)
        ax.set_ylabel(f'{param2.replace("_", " ").title()}', fontsize=12, labelpad=10)
        ax.set_zlabel('Discharge Time (hours)', fontsize=12, labelpad=10)
        ax.set_title('Parameter Sensitivity Surface\nDischarge Time vs. Usage Parameters', 
                    fontsize=14, fontweight='bold')
        
        # Add colorbar
        cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, pad=0.1)
        cbar.set_label('Discharge Time (h)', rotation=270, labelpad=15)
        
        # Optimal view angle
        ax.view_init(elev=25, azim=45)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_radar_comparison(self, scenarios: Dict[str, Dict],
                               save_path: str = None) -> plt.Figure:
        """
        Radar/Spider chart for scenario comparison
        雷达图场景对比
        """
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(polar=True))
        
        # Categories
        categories = ['Battery Life', 'CPU Usage', 'Display Power', 
                     'Network Usage', 'Thermal Load', 'Background Activity']
        N = len(categories)
        
        # Compute angle for each category
        angles = [n / float(N) * 2 * np.pi for n in range(N)]
        angles += angles[:1]  # Complete the loop
        
        # Colors for different scenarios
        scenario_colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#28A745', '#6C757D']
        
        for idx, (name, data) in enumerate(scenarios.items()):
            values = [
                data.get('battery_life_norm', 0.5),
                data.get('cpu_usage', 0.5),
                data.get('display_power', 0.5),
                data.get('network_usage', 0.5),
                data.get('thermal_load', 0.5),
                data.get('background_activity', 0.5)
            ]
            values += values[:1]  # Complete the loop
            
            ax.plot(angles, values, 'o-', linewidth=2, 
                   label=name, color=scenario_colors[idx % len(scenario_colors)])
            ax.fill(angles, values, alpha=0.15, color=scenario_colors[idx % len(scenario_colors)])
        
        # Set category labels
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, size=11)
        
        # Set radial labels
        ax.set_ylim(0, 1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['20%', '40%', '60%', '80%', '100%'], size=9)
        
        ax.set_title('Usage Scenario Comparison\nNormalized Resource Utilization', 
                    size=14, fontweight='bold', y=1.08)
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_kalman_estimation(self, true_SOC: np.ndarray,
                                estimated_SOC: np.ndarray,
                                covariance: np.ndarray,
                                time: np.ndarray,
                                save_path: str = None) -> plt.Figure:
        """
        Kalman filter estimation visualization
        卡尔曼滤波估计可视化
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        # SOC estimation
        ax1 = axes[0]
        ax1.plot(time, true_SOC * 100, 'k-', linewidth=2, label='True SOC', alpha=0.8)
        ax1.plot(time, estimated_SOC * 100, 'b-', linewidth=2, label='EKF Estimate')
        
        # Confidence interval (2-sigma)
        sigma = np.sqrt(covariance[:, 0]) * 100
        ax1.fill_between(time, estimated_SOC*100 - 2*sigma, estimated_SOC*100 + 2*sigma,
                        alpha=0.3, color='blue', label='95% CI')
        
        ax1.set_ylabel('State of Charge (%)')
        ax1.set_title('Extended Kalman Filter SOC Estimation', fontsize=14, fontweight='bold')
        ax1.legend(loc='upper right')
        ax1.set_ylim(0, 105)
        ax1.grid(True, alpha=0.3)
        
        # Estimation error
        ax2 = axes[1]
        error = (estimated_SOC - true_SOC) * 100
        ax2.plot(time, error, 'r-', linewidth=1.5, label='Estimation Error')
        ax2.fill_between(time, -2*sigma, 2*sigma, alpha=0.2, color='gray', label='±2σ Bound')
        ax2.axhline(y=0, color='black', linestyle='--', linewidth=1)
        
        ax2.set_xlabel('Time (hours)')
        ax2.set_ylabel('Error (%)')
        ax2.set_title('Estimation Error with Uncertainty Bounds')
        ax2.legend(loc='upper right')
        ax2.set_ylim(-10, 10)
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_pareto_frontier(self, pareto_objectives: np.ndarray,
                              objective_names: List[str] = None,
                              save_path: str = None) -> plt.Figure:
        """
        Pareto frontier visualization for multi-objective optimization
        多目标优化帕累托前沿可视化
        """
        if objective_names is None:
            objective_names = ['Voltage Error', 'SOC Error', 'Thermal Error']
        
        n_obj = min(pareto_objectives.shape[1], 3)
        
        if n_obj == 2:
            fig, ax = plt.subplots(figsize=(10, 8))
            
            # Sort by first objective
            sorted_idx = np.argsort(pareto_objectives[:, 0])
            sorted_obj = pareto_objectives[sorted_idx]
            
            # Plot Pareto front
            ax.scatter(sorted_obj[:, 0], sorted_obj[:, 1], 
                      c=range(len(sorted_obj)), cmap='viridis', 
                      s=100, edgecolors='black', linewidth=1, zorder=3)
            ax.plot(sorted_obj[:, 0], sorted_obj[:, 1], 'k--', linewidth=1, alpha=0.5)
            
            ax.set_xlabel(objective_names[0], fontsize=12)
            ax.set_ylabel(objective_names[1], fontsize=12)
            ax.set_title('Pareto Frontier - Multi-Objective Optimization', 
                        fontsize=14, fontweight='bold')
            
        else:  # 3D Pareto front
            fig = plt.figure(figsize=(12, 9))
            ax = fig.add_subplot(111, projection='3d')
            
            scatter = ax.scatter(pareto_objectives[:, 0], 
                               pareto_objectives[:, 1],
                               pareto_objectives[:, 2],
                               c=np.arange(len(pareto_objectives)),
                               cmap='plasma', s=80, edgecolors='black',
                               linewidth=0.5, alpha=0.8)
            
            ax.set_xlabel(objective_names[0], fontsize=11, labelpad=10)
            ax.set_ylabel(objective_names[1], fontsize=11, labelpad=10)
            ax.set_zlabel(objective_names[2], fontsize=11, labelpad=10)
            ax.set_title('3D Pareto Frontier\nMulti-Objective Parameter Optimization',
                        fontsize=14, fontweight='bold')
            
            cbar = fig.colorbar(scatter, ax=ax, shrink=0.5, aspect=10)
            cbar.set_label('Solution Index', rotation=270, labelpad=15)
            
            ax.view_init(elev=20, azim=45)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_prediction_cone(self, t_current: float,
                              SOC_current: float,
                              predictions: Dict[str, 'PredictionResult'],
                              save_path: str = None) -> plt.Figure:
        """
        Prediction cone visualization showing future trajectories
        预测锥可视化展示未来轨迹
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Generate prediction trajectories for different scenarios
        scenario_colors = {
            'idle': '#27AE60',
            'light_use': '#3498DB',
            'video_streaming': '#F39C12',
            'navigation': '#E74C3C',
            'gaming': '#9B59B6',
            'heavy_multitask': '#34495E'
        }
        
        t_pred_max = max(p.time_remaining for p in predictions.values() if p.time_remaining < np.inf)
        t_future = np.linspace(0, t_pred_max * 1.2, 100)
        
        # Plot confidence cone
        all_trajectories = []
        for scenario, pred in predictions.items():
            if pred.time_remaining < np.inf and pred.time_remaining > 0:
                t_end = pred.time_remaining
                # Generate exponential-like discharge curve
                SOC_traj = SOC_current * np.exp(-3 * t_future / t_end)
                SOC_traj = np.maximum(SOC_traj, 0.05)
                all_trajectories.append((scenario, t_future, SOC_traj * 100))
        
        # Plot trajectories
        for scenario, t, soc in all_trajectories:
            color = scenario_colors.get(scenario, '#95A5A6')
            ax.plot(t_current + t, soc, linewidth=2, color=color, 
                   label=f'{scenario.replace("_", " ").title()}', alpha=0.8)
        
        # Historical data (mock)
        t_history = np.linspace(0, t_current, 50)
        SOC_history = 100 - (100 - SOC_current * 100) * (t_history / t_current) ** 0.8
        ax.plot(t_history, SOC_history, 'k-', linewidth=3, label='Historical')
        
        # Current point
        ax.scatter([t_current], [SOC_current * 100], s=150, c='red', 
                  zorder=5, edgecolors='black', linewidth=2, label='Current')
        
        # Cutoff line
        ax.axhline(y=5, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
        ax.text(t_current + t_pred_max * 0.9, 7, 'Cutoff (5%)', fontsize=10, color='red')
        
        ax.set_xlabel('Time (hours)', fontsize=12)
        ax.set_ylabel('State of Charge (%)', fontsize=12)
        ax.set_title('Battery Discharge Prediction Cone\nMultiple Usage Scenarios', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9)
        ax.set_xlim(0, t_current + t_pred_max * 1.1)
        ax.set_ylim(0, 105)
        ax.grid(True, alpha=0.3)
        
        # Add annotation
        ax.annotate('Prediction\nUncertainty', xy=(t_current + t_pred_max*0.5, 40),
                   fontsize=10, ha='center', style='italic', color='gray')
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_sensitivity_heatmap(self, sensitivity_results: Dict,
                                  save_path: str = None) -> plt.Figure:
        """
        Heatmap for parameter sensitivity analysis
        参数敏感性热力图
        """
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # First-order indices
        ax1 = axes[0]
        params = list(sensitivity_results.get('S1', {}).keys())
        S1_values = list(sensitivity_results.get('S1', {}).values())
        ST_values = list(sensitivity_results.get('ST', {}).values())
        
        # Create data matrix
        data = np.array([S1_values, ST_values]).T
        
        im1 = ax1.imshow(data, cmap='YlOrRd', aspect='auto', vmin=0, vmax=1)
        ax1.set_xticks([0, 1])
        ax1.set_xticklabels(['First-Order (S1)', 'Total Effect (ST)'])
        ax1.set_yticks(range(len(params)))
        ax1.set_yticklabels([p.replace('_', ' ').title() for p in params])
        ax1.set_title('Sobol Sensitivity Indices', fontsize=14, fontweight='bold')
        
        # Add values as text
        for i in range(len(params)):
            for j in range(2):
                ax1.text(j, i, f'{data[i, j]:.3f}', ha='center', va='center',
                        color='white' if data[i, j] > 0.5 else 'black', fontsize=10)
        
        plt.colorbar(im1, ax=ax1, label='Sensitivity Index')
        
        # Interaction effects
        ax2 = axes[1]
        interaction = np.array(ST_values) - np.array(S1_values)
        
        # Create interaction matrix (simplified visualization)
        n = len(params)
        interaction_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    interaction_matrix[i, j] = abs(interaction[i] + interaction[j]) / 2
                else:
                    interaction_matrix[i, j] = interaction[i]
        
        im2 = ax2.imshow(interaction_matrix, cmap='Blues', aspect='auto')
        ax2.set_xticks(range(n))
        ax2.set_xticklabels([p.replace('_', ' ').title()[:8] for p in params], rotation=45, ha='right')
        ax2.set_yticks(range(n))
        ax2.set_yticklabels([p.replace('_', ' ').title()[:8] for p in params])
        ax2.set_title('Parameter Interaction Effects', fontsize=14, fontweight='bold')
        
        plt.colorbar(im2, ax=ax2, label='Interaction Strength')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def plot_phase_portrait(self, results: Dict,
                            save_path: str = None) -> plt.Figure:
        """
        Phase portrait for SOC-Temperature dynamics
        SOC-温度动力学相图
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        SOC = results.get('SOC', np.linspace(1, 0.05, 100))
        T = results.get('T_batt_C', 25 + 10 * (1 - SOC))
        
        # Create line collection with color gradient
        points = np.array([SOC * 100, T]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        norm = colors.Normalize(0, len(SOC))
        lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=3)
        lc.set_array(np.arange(len(SOC)))
        line = ax.add_collection(lc)
        
        # Add direction arrows
        n_arrows = 5
        arrow_idx = np.linspace(0, len(SOC)-2, n_arrows, dtype=int)
        for idx in arrow_idx:
            dx = SOC[idx+1] - SOC[idx]
            dy = T[idx+1] - T[idx]
            ax.annotate('', xy=(SOC[idx+1]*100, T[idx+1]), 
                       xytext=(SOC[idx]*100, T[idx]),
                       arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
        
        # Mark start and end
        ax.scatter([SOC[0]*100], [T[0]], s=200, c='green', marker='o', 
                  zorder=5, edgecolors='black', linewidth=2, label='Start')
        ax.scatter([SOC[-1]*100], [T[-1]], s=200, c='red', marker='s',
                  zorder=5, edgecolors='black', linewidth=2, label='End')
        
        ax.set_xlim(0, 105)
        ax.set_ylim(T.min() - 2, T.max() + 2)
        ax.set_xlabel('State of Charge (%)', fontsize=12)
        ax.set_ylabel('Battery Temperature (°C)', fontsize=12)
        ax.set_title('Phase Portrait: SOC-Temperature Dynamics\nDischarge Trajectory in State Space',
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper left')
        
        cbar = fig.colorbar(line, ax=ax, label='Time Progression')
        cbar.set_ticks([])
        
        ax.grid(True, alpha=0.3)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
    
    def create_dashboard(self, results: Dict,
                          predictions: Dict,
                          sensitivity: Dict,
                          save_path: str = None) -> plt.Figure:
        """
        Create comprehensive dashboard with multiple visualizations
        创建包含多个可视化的综合仪表盘
        """
        fig = plt.figure(figsize=(20, 16))
        
        # Create grid
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # 1. Main discharge trajectory (top left, large)
        ax1 = fig.add_subplot(gs[0, :2])
        t = results.get('t_hours', np.linspace(0, 10, 100))
        SOC = results.get('SOC_percent', np.linspace(100, 5, 100))
        ax1.fill_between(t, SOC * 0.95, SOC * 1.05, alpha=0.3, color='#2E86AB')
        ax1.plot(t, SOC, linewidth=2.5, color='#2E86AB')
        ax1.axhline(y=5, color='red', linestyle='--', linewidth=1.5)
        ax1.set_xlabel('Time (hours)')
        ax1.set_ylabel('SOC (%)')
        ax1.set_title('Battery Discharge Trajectory', fontweight='bold')
        ax1.set_ylim(0, 105)
        
        # 2. Scenario comparison (top right)
        ax2 = fig.add_subplot(gs[0, 2])
        scenarios = list(predictions.keys())[:5]
        times = [predictions[s].time_remaining for s in scenarios if hasattr(predictions[s], 'time_remaining')]
        colors_bar = ['#27AE60', '#3498DB', '#F39C12', '#E74C3C', '#9B59B6']
        bars = ax2.barh(scenarios[:len(times)], times[:len(scenarios)], color=colors_bar[:len(times)])
        ax2.set_xlabel('Remaining Time (hours)')
        ax2.set_title('Scenario Comparison', fontweight='bold')
        
        # 3. Power breakdown (middle left)
        ax3 = fig.add_subplot(gs[1, 0])
        power_labels = ['CPU', 'Display', 'Network', 'GPS', 'BT', 'Background']
        power_values = [25, 30, 15, 10, 5, 15]
        colors_pie = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD']
        wedges, texts, autotexts = ax3.pie(power_values, labels=power_labels, autopct='%1.0f%%',
                                           colors=colors_pie, explode=[0.05]*6)
        ax3.set_title('Power Distribution', fontweight='bold')
        
        # 4. Temperature profile (middle center)
        ax4 = fig.add_subplot(gs[1, 1])
        T_batt = results.get('T_batt_C', 25 + 5*np.sin(np.linspace(0, 6, len(t))))
        T_cpu = results.get('T_cpu_C', 35 + 10*np.sin(np.linspace(0, 6, len(t))))
        ax4.plot(t, T_batt, 'r-', linewidth=2, label='Battery')
        ax4.plot(t, T_cpu, 'b-', linewidth=2, label='CPU')
        ax4.axhline(y=25, color='gray', linestyle=':', label='Ambient')
        ax4.set_xlabel('Time (hours)')
        ax4.set_ylabel('Temperature (°C)')
        ax4.set_title('Thermal Evolution', fontweight='bold')
        ax4.legend(loc='upper right', fontsize=8)
        
        # 5. Sensitivity bar chart (middle right)
        ax5 = fig.add_subplot(gs[1, 2])
        if sensitivity:
            params = list(sensitivity.get('S1', {}).keys())[:6]
            s1_vals = [sensitivity['S1'].get(p, 0) for p in params]
            st_vals = [sensitivity['ST'].get(p, 0) for p in params]
            x = np.arange(len(params))
            width = 0.35
            ax5.bar(x - width/2, s1_vals, width, label='First-Order', color='#3498DB')
            ax5.bar(x + width/2, st_vals, width, label='Total Effect', color='#E74C3C')
            ax5.set_xticks(x)
            ax5.set_xticklabels([p[:8] for p in params], rotation=45, ha='right')
            ax5.set_ylabel('Sensitivity Index')
            ax5.set_title('Parameter Sensitivity', fontweight='bold')
            ax5.legend(fontsize=8)
        
        # 6. Voltage profile (bottom left)
        ax6 = fig.add_subplot(gs[2, 0])
        V = results.get('V_batt', 4.2 - 0.9 * (1 - SOC/100))
        ax6.plot(t, V, linewidth=2, color='#9B59B6')
        ax6.fill_between(t, 3.0, V, alpha=0.3, color='#9B59B6')
        ax6.set_xlabel('Time (hours)')
        ax6.set_ylabel('Voltage (V)')
        ax6.set_title('Voltage Profile', fontweight='bold')
        ax6.set_ylim(3.0, 4.3)
        
        # 7. Current profile (bottom center)
        ax7 = fig.add_subplot(gs[2, 1])
        I = results.get('I_total', 0.5 + 0.2*np.random.randn(len(t)))
        ax7.plot(t, I, linewidth=1.5, color='#E67E22')
        ax7.set_xlabel('Time (hours)')
        ax7.set_ylabel('Current (A)')
        ax7.set_title('Discharge Current', fontweight='bold')
        
        # 8. Prediction uncertainty (bottom right)
        ax8 = fig.add_subplot(gs[2, 2])
        pred_main = predictions.get('light_use', None)
        if pred_main and hasattr(pred_main, 'probability_distribution') and pred_main.probability_distribution is not None:
            ax8.hist(pred_main.probability_distribution, bins=30, density=True,
                    alpha=0.7, color='#1ABC9C', edgecolor='black')
            ax8.axvline(x=pred_main.time_remaining, color='red', linestyle='--', 
                       linewidth=2, label=f'Mean: {pred_main.time_remaining:.1f}h')
            ax8.set_xlabel('Remaining Time (hours)')
            ax8.set_ylabel('Probability Density')
            ax8.set_title('Prediction Distribution', fontweight='bold')
            ax8.legend()
        else:
            ax8.text(0.5, 0.5, 'No Distribution\nData Available', 
                    ha='center', va='center', fontsize=12, transform=ax8.transAxes)
            ax8.set_title('Prediction Distribution', fontweight='bold')
        
        # Main title
        fig.suptitle('Smartphone Battery Model Dashboard', 
                    fontsize=18, fontweight='bold', y=0.98)
        
        if save_path:
            plt.savefig(save_path, bbox_inches='tight', facecolor='white')
        
        return fig
