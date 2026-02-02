"""
敏感性分析可视化模块
=======================
创建新颖的集成可视化图表，包括：
1. 龙卷风图（Tornado Diagram）
2. Sobol指数雷达图
3. Morris μ*-σ散点图
4. 蒙特卡洛分布小提琴图
5. 鲁棒性热图
6. 参数交互效应矩阵
7. 综合仪表板
8. 时变敏感性动画帧
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec
import matplotlib.cm as cm
from typing import Dict, List, Tuple, Optional
import warnings

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'SimHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# 自定义颜色方案
COLORS = {
    'primary': '#2E86AB',
    'secondary': '#A23B72', 
    'tertiary': '#F18F01',
    'quaternary': '#C73E1D',
    'success': '#3A9D5F',
    'warning': '#FFB400',
    'danger': '#E63946',
    'neutral': '#6C757D',
    'light': '#F8F9FA',
    'dark': '#212529',
}

# 渐变色映射
GRADIENT_CMAP = LinearSegmentedColormap.from_list(
    'sensitivity_cmap',
    ['#3A9D5F', '#FFB400', '#E63946']
)


def create_tornado_chart(sensitivity_data: Dict[str, Dict[str, float]],
                        output_metric: str = 'avg_power',
                        title: str = None,
                        figsize: Tuple[int, int] = (12, 8)) -> plt.Figure:
    """
    创建龙卷风图（Tornado Diagram）
    
    显示各参数对输出的影响程度，正负方向分别显示
    
    Parameters:
    -----------
    sensitivity_data : Dict
        局部敏感性分析结果
    output_metric : str
        目标输出指标
    title : str
        图表标题
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    data = sensitivity_data.get(output_metric, sensitivity_data)
    
    # 排序
    sorted_params = sorted(data.items(), key=lambda x: abs(x[1]), reverse=True)
    params = [p[0] for p in sorted_params]
    values = [p[1] for p in sorted_params]
    
    fig, ax = plt.subplots(figsize=figsize)
    
    y_pos = np.arange(len(params))
    colors = [COLORS['success'] if v >= 0 else COLORS['danger'] for v in values]
    
    # 绘制水平条形图
    bars = ax.barh(y_pos, values, color=colors, alpha=0.8, edgecolor='white', linewidth=0.5)
    
    # 添加数值标签
    for i, (bar, val) in enumerate(zip(bars, values)):
        width = bar.get_width()
        label_x = width + 0.01 if width >= 0 else width - 0.01
        ha = 'left' if width >= 0 else 'right'
        ax.annotate(f'{val:.3f}',
                   xy=(label_x, bar.get_y() + bar.get_height()/2),
                   ha=ha, va='center', fontsize=9, fontweight='bold')
    
    ax.set_yticks(y_pos)
    ax.set_yticklabels(params)
    ax.axvline(x=0, color=COLORS['dark'], linewidth=1.5, linestyle='-')
    ax.set_xlabel('Normalized Sensitivity Index', fontsize=12, fontweight='bold')
    ax.set_title(title or f'Parameter Sensitivity - {output_metric}', 
                fontsize=14, fontweight='bold', pad=20)
    
    # 添加图例
    positive_patch = mpatches.Patch(color=COLORS['success'], label='Positive Effect')
    negative_patch = mpatches.Patch(color=COLORS['danger'], label='Negative Effect')
    ax.legend(handles=[positive_patch, negative_patch], loc='lower right')
    
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(axis='x', alpha=0.3, linestyle='--')
    
    plt.tight_layout()
    return fig


def create_sobol_radar_chart(sobol_data: Dict[str, Dict[str, float]],
                             figsize: Tuple[int, int] = (10, 10)) -> plt.Figure:
    """
    创建Sobol指数雷达图
    
    同时显示一阶和全阶Sobol指数
    
    Parameters:
    -----------
    sobol_data : Dict
        Sobol分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    params = list(sobol_data.keys())
    n = len(params)
    
    # 提取S1和ST
    s1_values = [sobol_data[p]['S1'] for p in params]
    st_values = [sobol_data[p]['ST'] for p in params]
    
    # 计算角度
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False).tolist()
    angles += angles[:1]  # 闭合
    
    s1_values += s1_values[:1]
    st_values += st_values[:1]
    
    fig, ax = plt.subplots(figsize=figsize, subplot_kw=dict(projection='polar'))
    
    # 绘制S1（一阶）
    ax.fill(angles, s1_values, alpha=0.25, color=COLORS['primary'], label='First-order (S1)')
    ax.plot(angles, s1_values, 'o-', linewidth=2, color=COLORS['primary'], markersize=8)
    
    # 绘制ST（全阶）
    ax.fill(angles, st_values, alpha=0.15, color=COLORS['secondary'], label='Total-order (ST)')
    ax.plot(angles, st_values, 's--', linewidth=2, color=COLORS['secondary'], markersize=8)
    
    # 设置刻度
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(params, fontsize=9)
    ax.set_ylim(0, max(max(st_values), 1.0))
    
    # 添加同心圆标签
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(['0.25', '0.50', '0.75', '1.00'], fontsize=8)
    
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
    ax.set_title('Sobol Sensitivity Indices\n(Global Sensitivity Analysis)', 
                fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    return fig


def create_morris_scatter(morris_data: Dict[str, Dict[str, float]],
                          figsize: Tuple[int, int] = (10, 8)) -> plt.Figure:
    """
    创建Morris μ*-σ散点图
    
    显示参数重要性（μ*）和非线性/交互效应（σ）
    
    Parameters:
    -----------
    morris_data : Dict
        Morris筛选结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    params = list(morris_data.keys())
    mu_star = [morris_data[p]['mu_star'] for p in params]
    sigma = [morris_data[p]['sigma'] for p in params]
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # 颜色映射：根据重要性
    max_mu = max(mu_star) if max(mu_star) > 0 else 1
    colors = [plt.cm.RdYlGn_r(m / max_mu) for m in mu_star]
    
    # 气泡大小：根据sigma
    sizes = [max(50, s * 1000) for s in sigma]
    
    scatter = ax.scatter(mu_star, sigma, c=colors, s=sizes, alpha=0.7, edgecolors='white', linewidth=2)
    
    # 添加参数标签
    for i, param in enumerate(params):
        ax.annotate(param, (mu_star[i], sigma[i]), 
                   xytext=(5, 5), textcoords='offset points',
                   fontsize=8, alpha=0.8)
    
    # 添加参考线（σ = μ* 表示高度非线性）
    max_val = max(max(mu_star), max(sigma)) * 1.1
    ax.plot([0, max_val], [0, max_val], 'k--', alpha=0.3, label='σ = μ* (Highly Nonlinear)')
    ax.plot([0, max_val], [0, max_val * 0.5], 'g--', alpha=0.3, label='σ = 0.5μ* (Moderate)')
    ax.plot([0, max_val], [0, max_val * 0.1], 'b--', alpha=0.3, label='σ = 0.1μ* (Nearly Linear)')
    
    ax.set_xlabel('μ* (Mean Absolute Elementary Effect)', fontsize=12, fontweight='bold')
    ax.set_ylabel('σ (Standard Deviation)', fontsize=12, fontweight='bold')
    ax.set_title('Morris Screening Analysis\n(Parameter Importance & Nonlinearity)', 
                fontsize=14, fontweight='bold', pad=20)
    
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)
    
    plt.tight_layout()
    return fig


def create_monte_carlo_violin(mc_data: Dict[str, Dict],
                              figsize: Tuple[int, int] = (14, 6)) -> plt.Figure:
    """
    创建蒙特卡洛分布小提琴图
    
    显示输出变量的概率分布
    
    Parameters:
    -----------
    mc_data : Dict
        蒙特卡洛分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    metrics = list(mc_data.keys())
    n_metrics = len(metrics)
    
    fig, axes = plt.subplots(1, n_metrics, figsize=figsize)
    if n_metrics == 1:
        axes = [axes]
    
    colors = [COLORS['primary'], COLORS['secondary'], COLORS['tertiary'], COLORS['quaternary']]
    
    for i, (metric, data) in enumerate(mc_data.items()):
        ax = axes[i]
        samples = data['samples']
        
        # 小提琴图
        parts = ax.violinplot([samples], positions=[0], showmeans=True, showmedians=True)
        
        # 自定义颜色
        for pc in parts['bodies']:
            pc.set_facecolor(colors[i % len(colors)])
            pc.set_alpha(0.7)
        
        parts['cmeans'].set_color(COLORS['danger'])
        parts['cmedians'].set_color(COLORS['dark'])
        
        # 添加箱线图元素
        q1, q3 = data['p25'], data['p75']
        median = data['median']
        ax.vlines(0, q1, q3, color=COLORS['dark'], linestyle='-', lw=5, alpha=0.7)
        ax.scatter([0], [median], color='white', s=30, zorder=3)
        
        # 添加统计标注
        stats_text = f"μ = {data['mean']:.2f}\nσ = {data['std']:.2f}\nCV = {data['cv']:.2%}"
        ax.text(0.95, 0.95, stats_text, transform=ax.transAxes, 
               fontsize=9, verticalalignment='top', horizontalalignment='right',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # 标记5%和95%置信区间
        ax.axhline(y=data['p5'], color=COLORS['danger'], linestyle='--', alpha=0.5)
        ax.axhline(y=data['p95'], color=COLORS['danger'], linestyle='--', alpha=0.5)
        ax.text(0.3, data['p5'], '5%', fontsize=8, color=COLORS['danger'])
        ax.text(0.3, data['p95'], '95%', fontsize=8, color=COLORS['danger'])
        
        ax.set_title(metric, fontsize=12, fontweight='bold')
        ax.set_xticks([])
        ax.grid(axis='y', alpha=0.3, linestyle='--')
    
    fig.suptitle('Monte Carlo Uncertainty Propagation\n(Output Distribution Analysis)', 
                fontsize=14, fontweight='bold', y=1.02)
    
    plt.tight_layout()
    return fig


def create_robustness_heatmap(robustness_data: Dict[str, Dict],
                              figsize: Tuple[int, int] = (14, 10)) -> plt.Figure:
    """
    创建鲁棒性热图
    
    显示参数扰动对输出的影响
    
    Parameters:
    -----------
    robustness_data : Dict
        鲁棒性分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    params = list(robustness_data.keys())
    n_params = len(params)
    
    # 获取扰动级别
    sample_data = robustness_data[params[0]]
    perturbation_levels = sample_data['perturbation_levels']
    n_levels = len(perturbation_levels)
    
    # 构建热图矩阵
    heatmap_data = np.zeros((n_params, n_levels))
    base_values = []
    
    for i, param in enumerate(params):
        data = robustness_data[param]
        base = data['base_value']
        base_values.append(base)
        
        # 计算相对变化
        if base != 0:
            relative_change = (data['outputs'] - base) / abs(base) * 100
        else:
            relative_change = data['outputs'] - base
        
        heatmap_data[i, :] = relative_change
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # 创建对称的颜色映射
    max_abs = np.max(np.abs(heatmap_data))
    im = ax.imshow(heatmap_data, cmap='RdBu_r', aspect='auto',
                   vmin=-max_abs, vmax=max_abs)
    
    # 设置刻度
    ax.set_xticks(range(n_levels))
    ax.set_xticklabels([f'{p*100:.0f}%' for p in perturbation_levels], rotation=45, ha='right')
    ax.set_yticks(range(n_params))
    ax.set_yticklabels(params)
    
    # 添加颜色条
    cbar = plt.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label('Output Change (%)', fontsize=11, fontweight='bold')
    
    # 添加数值标注
    for i in range(n_params):
        for j in range(n_levels):
            val = heatmap_data[i, j]
            color = 'white' if abs(val) > max_abs * 0.5 else 'black'
            ax.text(j, i, f'{val:.1f}', ha='center', va='center', 
                   color=color, fontsize=7)
    
    ax.set_xlabel('Parameter Perturbation Level', fontsize=12, fontweight='bold')
    ax.set_ylabel('Parameters', fontsize=12, fontweight='bold')
    ax.set_title('Parameter Robustness Analysis\n(Sensitivity to Perturbations)', 
                fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    return fig


def create_scenario_comparison(scenario_data: Dict[str, Dict[str, float]],
                               figsize: Tuple[int, int] = (12, 8)) -> plt.Figure:
    """
    创建场景对比图
    
    显示不同使用场景下的性能指标
    
    Parameters:
    -----------
    scenario_data : Dict
        场景分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    scenarios = list(scenario_data.keys())
    metrics = list(scenario_data[scenarios[0]].keys())
    
    n_scenarios = len(scenarios)
    n_metrics = len(metrics)
    
    fig, ax = plt.subplots(figsize=figsize)
    
    x = np.arange(n_scenarios)
    width = 0.8 / n_metrics
    
    colors = [COLORS['primary'], COLORS['secondary'], COLORS['tertiary'], 
              COLORS['quaternary'], COLORS['success']]
    
    # 获取基准值用于归一化
    baseline = scenario_data.get('baseline', scenario_data[scenarios[0]])
    
    for i, metric in enumerate(metrics):
        values = []
        for scenario in scenarios:
            val = scenario_data[scenario][metric]
            base_val = baseline[metric]
            # 归一化到基准值
            normalized = val / base_val * 100 if base_val != 0 else 100
            values.append(normalized)
        
        offset = (i - n_metrics/2 + 0.5) * width
        bars = ax.bar(x + offset, values, width, label=metric, 
                     color=colors[i % len(colors)], alpha=0.8)
    
    ax.axhline(y=100, color=COLORS['dark'], linestyle='--', alpha=0.5, label='Baseline')
    
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=45, ha='right')
    ax.set_ylabel('Relative Value (Baseline = 100%)', fontsize=12, fontweight='bold')
    ax.set_title('Scenario Robustness Comparison\n(Performance Under Different Conditions)', 
                fontsize=14, fontweight='bold', pad=20)
    
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    
    plt.tight_layout()
    return fig


def create_integrated_dashboard(all_results: Dict,
                                figsize: Tuple[int, int] = (20, 16)) -> plt.Figure:
    """
    创建综合分析仪表板
    
    将所有分析结果整合到一个图表中
    
    Parameters:
    -----------
    all_results : Dict
        完整分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    fig = plt.figure(figsize=figsize, facecolor='white')
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. 龙卷风图 (左上)
    ax1 = fig.add_subplot(gs[0, 0])
    if 'local' in all_results and 'avg_power' in all_results['local']:
        data = all_results['local']['avg_power']
        sorted_params = sorted(data.items(), key=lambda x: abs(x[1]), reverse=True)[:10]
        params = [p[0] for p in sorted_params]
        values = [p[1] for p in sorted_params]
        
        colors = [COLORS['success'] if v >= 0 else COLORS['danger'] for v in values]
        ax1.barh(range(len(params)), values, color=colors, alpha=0.8)
        ax1.set_yticks(range(len(params)))
        ax1.set_yticklabels(params, fontsize=8)
        ax1.axvline(x=0, color='black', linewidth=1)
        ax1.set_title('Local Sensitivity (Top 10)', fontsize=11, fontweight='bold')
        ax1.set_xlabel('Sensitivity Index', fontsize=9)
    
    # 2. Morris散点图 (中上)
    ax2 = fig.add_subplot(gs[0, 1])
    if 'morris' in all_results:
        morris = all_results['morris']
        params = list(morris.keys())
        mu_star = [morris[p]['mu_star'] for p in params]
        sigma = [morris[p]['sigma'] for p in params]
        
        ax2.scatter(mu_star, sigma, c=mu_star, cmap='YlOrRd', s=100, alpha=0.7, edgecolors='white')
        for i, p in enumerate(params):
            ax2.annotate(p[:8], (mu_star[i], sigma[i]), fontsize=6, alpha=0.7)
        ax2.set_xlabel('μ* (Importance)', fontsize=9)
        ax2.set_ylabel('σ (Nonlinearity)', fontsize=9)
        ax2.set_title('Morris Screening', fontsize=11, fontweight='bold')
        ax2.grid(True, alpha=0.3)
    
    # 3. Sobol条形图 (右上)
    ax3 = fig.add_subplot(gs[0, 2])
    if 'sobol' in all_results:
        sobol = all_results['sobol']
        params = list(sobol.keys())
        s1 = [sobol[p]['S1'] for p in params]
        st = [sobol[p]['ST'] for p in params]
        
        x = np.arange(len(params))
        width = 0.35
        ax3.bar(x - width/2, s1, width, label='S1 (First-order)', color=COLORS['primary'], alpha=0.8)
        ax3.bar(x + width/2, st, width, label='ST (Total-order)', color=COLORS['secondary'], alpha=0.8)
        ax3.set_xticks(x)
        ax3.set_xticklabels(params, rotation=45, ha='right', fontsize=7)
        ax3.set_title('Sobol Indices', fontsize=11, fontweight='bold')
        ax3.legend(fontsize=8)
    
    # 4. 蒙特卡洛分布 (左中)
    ax4 = fig.add_subplot(gs[1, 0])
    if 'monte_carlo' in all_results and 'avg_power' in all_results['monte_carlo']:
        mc = all_results['monte_carlo']['avg_power']
        samples = mc['samples']
        
        ax4.hist(samples, bins=30, density=True, color=COLORS['primary'], alpha=0.7, edgecolor='white')
        ax4.axvline(mc['mean'], color=COLORS['danger'], linestyle='--', linewidth=2, label=f"Mean: {mc['mean']:.1f}")
        ax4.axvline(mc['p5'], color=COLORS['warning'], linestyle=':', linewidth=1.5, label=f"5%: {mc['p5']:.1f}")
        ax4.axvline(mc['p95'], color=COLORS['warning'], linestyle=':', linewidth=1.5, label=f"95%: {mc['p95']:.1f}")
        ax4.set_xlabel('Average Power (mW)', fontsize=9)
        ax4.set_ylabel('Density', fontsize=9)
        ax4.set_title('Monte Carlo - Power Distribution', fontsize=11, fontweight='bold')
        ax4.legend(fontsize=8)
    
    # 5. 鲁棒性热图 (中中)
    ax5 = fig.add_subplot(gs[1, 1])
    if 'robustness_param' in all_results:
        rob = all_results['robustness_param']
        params = list(rob.keys())[:8]  # 取前8个参数
        
        perturbation = rob[params[0]]['perturbation_levels']
        heatmap_data = []
        for p in params:
            base = rob[p]['base_value']
            if base != 0:
                rel_change = (rob[p]['outputs'] - base) / abs(base) * 100
            else:
                rel_change = rob[p]['outputs']
            heatmap_data.append(rel_change)
        
        heatmap_data = np.array(heatmap_data)
        max_abs = np.max(np.abs(heatmap_data))
        im = ax5.imshow(heatmap_data, cmap='RdBu_r', aspect='auto', vmin=-max_abs, vmax=max_abs)
        ax5.set_yticks(range(len(params)))
        ax5.set_yticklabels(params, fontsize=7)
        ax5.set_xticks(range(len(perturbation)))
        ax5.set_xticklabels([f'{p*100:.0f}%' for p in perturbation], fontsize=7, rotation=45)
        ax5.set_title('Robustness Heatmap', fontsize=11, fontweight='bold')
        plt.colorbar(im, ax=ax5, shrink=0.8)
    
    # 6. 场景对比 (右中)
    ax6 = fig.add_subplot(gs[1, 2])
    if 'robustness_scenario' in all_results:
        scenarios = all_results['robustness_scenario']
        names = list(scenarios.keys())
        
        if 'avg_power' in list(scenarios.values())[0]:
            baseline = scenarios.get('baseline', scenarios[names[0]])['avg_power']
            values = [scenarios[s]['avg_power'] / baseline * 100 for s in names]
            
            colors = [COLORS['success'] if v <= 100 else COLORS['danger'] for v in values]
            ax6.barh(range(len(names)), values, color=colors, alpha=0.8)
            ax6.axvline(x=100, color='black', linestyle='--', linewidth=1)
            ax6.set_yticks(range(len(names)))
            ax6.set_yticklabels(names, fontsize=8)
            ax6.set_xlabel('Relative Power (%)', fontsize=9)
            ax6.set_title('Scenario Comparison', fontsize=11, fontweight='bold')
    
    # 7. 参数重要性排名 (左下)
    ax7 = fig.add_subplot(gs[2, 0])
    if 'local' in all_results:
        # 综合多个指标的重要性
        metrics = all_results['local']
        all_params = set()
        for m in metrics.values():
            all_params.update(m.keys())
        
        importance_scores = {}
        for p in all_params:
            scores = []
            for m in metrics.values():
                if p in m:
                    scores.append(abs(m[p]))
            importance_scores[p] = np.mean(scores) if scores else 0
        
        sorted_imp = sorted(importance_scores.items(), key=lambda x: x[1], reverse=True)[:10]
        params = [p[0] for p in sorted_imp]
        scores = [p[1] for p in sorted_imp]
        
        ax7.barh(range(len(params)), scores, color=COLORS['tertiary'], alpha=0.8)
        ax7.set_yticks(range(len(params)))
        ax7.set_yticklabels(params, fontsize=8)
        ax7.set_xlabel('Average |Sensitivity|', fontsize=9)
        ax7.set_title('Overall Parameter Importance', fontsize=11, fontweight='bold')
    
    # 8. 鲁棒性分数 (中下)
    ax8 = fig.add_subplot(gs[2, 1])
    if 'robustness_param' in all_results:
        rob = all_results['robustness_param']
        params = list(rob.keys())
        scores = [rob[p]['robustness_score'] for p in params]
        
        sorted_data = sorted(zip(params, scores), key=lambda x: x[1], reverse=True)[:10]
        params = [d[0] for d in sorted_data]
        scores = [d[1] for d in sorted_data]
        
        colors = [plt.cm.RdYlGn(s) for s in scores]
        ax8.barh(range(len(params)), scores, color=colors, alpha=0.8)
        ax8.set_yticks(range(len(params)))
        ax8.set_yticklabels(params, fontsize=8)
        ax8.set_xlabel('Robustness Score (0-1)', fontsize=9)
        ax8.set_xlim(0, 1)
        ax8.set_title('Robustness Scores', fontsize=11, fontweight='bold')
    
    # 9. 不确定性汇总 (右下)
    ax9 = fig.add_subplot(gs[2, 2])
    if 'monte_carlo' in all_results:
        mc = all_results['monte_carlo']
        metrics = list(mc.keys())
        cvs = [mc[m]['cv'] * 100 for m in metrics]
        
        colors = [COLORS['success'] if cv < 20 else COLORS['warning'] if cv < 50 else COLORS['danger'] for cv in cvs]
        ax9.bar(range(len(metrics)), cvs, color=colors, alpha=0.8)
        ax9.set_xticks(range(len(metrics)))
        ax9.set_xticklabels(metrics, rotation=45, ha='right', fontsize=8)
        ax9.set_ylabel('Coefficient of Variation (%)', fontsize=9)
        ax9.set_title('Output Uncertainty (CV)', fontsize=11, fontweight='bold')
        ax9.axhline(y=20, color=COLORS['success'], linestyle='--', alpha=0.5)
        ax9.axhline(y=50, color=COLORS['warning'], linestyle='--', alpha=0.5)
    
    fig.suptitle('Smartphone Power Model - Comprehensive Sensitivity Analysis Dashboard', 
                fontsize=16, fontweight='bold', y=0.98)
    
    return fig


def create_interaction_matrix(sensitivity_results: Dict,
                              figsize: Tuple[int, int] = (12, 10)) -> plt.Figure:
    """
    创建参数交互效应矩阵
    
    Parameters:
    -----------
    sensitivity_results : Dict
        敏感性分析结果
    figsize : Tuple[int, int]
        图表尺寸
    
    Returns:
    --------
    plt.Figure
        Matplotlib图表对象
    """
    if 'sobol' not in sensitivity_results:
        print("需要Sobol分析结果来计算交互效应")
        return None
    
    sobol = sensitivity_results['sobol']
    params = list(sobol.keys())
    n = len(params)
    
    # 估计交互效应: ST - S1
    interaction = np.zeros((n, n))
    
    for i, p in enumerate(params):
        s1 = sobol[p]['S1']
        st = sobol[p]['ST']
        interaction_effect = max(0, st - s1)
        
        # 将交互效应分配到对角线
        interaction[i, i] = s1
        
        # 非对角线元素表示与其他参数的潜在交互
        for j in range(n):
            if i != j:
                interaction[i, j] = interaction_effect / (n - 1)
    
    fig, ax = plt.subplots(figsize=figsize)
    
    im = ax.imshow(interaction, cmap='YlOrRd', aspect='equal')
    
    ax.set_xticks(range(n))
    ax.set_xticklabels(params, rotation=45, ha='right', fontsize=8)
    ax.set_yticks(range(n))
    ax.set_yticklabels(params, fontsize=8)
    
    # 添加数值标注
    for i in range(n):
        for j in range(n):
            val = interaction[i, j]
            color = 'white' if val > 0.3 else 'black'
            ax.text(j, i, f'{val:.2f}', ha='center', va='center', 
                   color=color, fontsize=7)
    
    cbar = plt.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label('Effect Size', fontsize=11)
    
    ax.set_title('Parameter Interaction Matrix\n(Diagonal: First-order, Off-diagonal: Interaction estimates)', 
                fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    return fig


def save_all_figures(all_results: Dict, output_dir: str = '.') -> None:
    """
    保存所有可视化图表
    
    Parameters:
    -----------
    all_results : Dict
        完整分析结果
    output_dir : str
        输出目录
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    figures = []
    
    # 1. 龙卷风图
    if 'local' in all_results:
        for metric in all_results['local'].keys():
            fig = create_tornado_chart(all_results['local'], metric, 
                                      title=f'Tornado Chart - {metric}')
            fig.savefig(f'{output_dir}/tornado_{metric}.png', dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f"Saved: tornado_{metric}.png")
    
    # 2. Sobol雷达图
    if 'sobol' in all_results:
        fig = create_sobol_radar_chart(all_results['sobol'])
        fig.savefig(f'{output_dir}/sobol_radar.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("Saved: sobol_radar.png")
    
    # 3. Morris散点图
    if 'morris' in all_results:
        fig = create_morris_scatter(all_results['morris'])
        fig.savefig(f'{output_dir}/morris_scatter.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("Saved: morris_scatter.png")
    
    # 4. 蒙特卡洛小提琴图
    if 'monte_carlo' in all_results:
        fig = create_monte_carlo_violin(all_results['monte_carlo'])
        fig.savefig(f'{output_dir}/monte_carlo_violin.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("Saved: monte_carlo_violin.png")
    
    # 5. 鲁棒性热图
    if 'robustness_param' in all_results:
        fig = create_robustness_heatmap(all_results['robustness_param'])
        fig.savefig(f'{output_dir}/robustness_heatmap.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("Saved: robustness_heatmap.png")
    
    # 6. 场景对比图
    if 'robustness_scenario' in all_results:
        fig = create_scenario_comparison(all_results['robustness_scenario'])
        fig.savefig(f'{output_dir}/scenario_comparison.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("Saved: scenario_comparison.png")
    
    # 7. 综合仪表板
    fig = create_integrated_dashboard(all_results)
    fig.savefig(f'{output_dir}/integrated_dashboard.png', dpi=200, bbox_inches='tight')
    plt.close(fig)
    print("Saved: integrated_dashboard.png")
    
    # 8. 交互矩阵
    if 'sobol' in all_results:
        fig = create_interaction_matrix(all_results)
        if fig:
            fig.savefig(f'{output_dir}/interaction_matrix.png', dpi=150, bbox_inches='tight')
            plt.close(fig)
            print("Saved: interaction_matrix.png")
    
    print(f"\nAll figures saved to: {output_dir}")


if __name__ == "__main__":
    # 测试可视化模块
    print("可视化模块测试...")
    
    # 创建模拟数据进行测试
    test_local = {
        'param_a': 0.5,
        'param_b': -0.3,
        'param_c': 0.8,
        'param_d': -0.1,
    }
    
    fig = create_tornado_chart({'test': test_local}, 'test')
    plt.show()
