#!/usr/bin/env python3
"""
用户建议可视化模块
================

生成用户行为建议和操作系统策略的可视化图表。

Author: Battery Modeling Framework
Date: 2026-02-01
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import matplotlib.patches as mpatches
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from user_behavior_analysis import UserBehaviorAnalysis, AgingImpactAnalysis
from os_power_strategies import OSPowerStrategies, CrossDeviceGeneralization
from load_subsystems import ModelSoC

plt.rcParams['font.size'] = 10


def generate_user_recommendations_figure():
    """生成用户建议可视化图表"""
    
    analyzer = UserBehaviorAnalysis()
    aging = AgingImpactAnalysis()
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 获取所有行为并排名
    all_behaviors = analyzer.generate_comprehensive_ranking()
    
    # (a) 综合节能排名条形图
    ax = axes[0, 0]
    
    names = [b['name'][:20] + '...' if len(b['name']) > 20 else b['name'] 
             for b in all_behaviors[:10]]
    savings = [b['savings_mW'] for b in all_behaviors[:10]]
    categories = [b['category'] for b in all_behaviors[:10]]
    
    category_colors = {
        '显示设置优化': '#3498DB',
        '连接管理优化': '#2ECC71',
        '后台活动管理': '#F39C12',
        '性能与散热管理': '#E74C3C'
    }
    colors = [category_colors.get(c, 'gray') for c in categories]
    
    bars = ax.barh(range(len(names)), savings, color=colors)
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names)
    ax.set_xlabel('Savings (mW)', fontsize=11)
    ax.set_title('(a) User Behavior Power Saving Ranking', fontsize=12, fontweight='bold')
    ax.invert_yaxis()
    
    legend_patches = [mpatches.Patch(color=c, label=l) for l, c in category_colors.items()]
    ax.legend(handles=legend_patches, loc='lower right', fontsize=9)
    
    for bar, s in zip(bars, savings):
        ax.text(bar.get_width() + 20, bar.get_y() + bar.get_height()/2,
                f'{s:.0f}mW', va='center', fontsize=9)
    ax.grid(True, alpha=0.3, axis='x')
    
    # (b) 显示优化效果对比
    ax = axes[0, 1]
    
    display_data = analyzer.analyze_display_behaviors()
    names_d = [b['name'] for b in display_data['behaviors']]
    baseline_d = [b['baseline_mW'] for b in display_data['behaviors']]
    optimized_d = [b['optimized_mW'] for b in display_data['behaviors']]
    
    x = np.arange(len(names_d))
    width = 0.35
    
    ax.bar(x - width/2, baseline_d, width, label='Before', color='#E74C3C', alpha=0.8)
    ax.bar(x + width/2, optimized_d, width, label='After', color='#2ECC71', alpha=0.8)
    
    for i, (b, o) in enumerate(zip(baseline_d, optimized_d)):
        pct = (b - o) / b * 100
        ax.annotate(f'-{pct:.0f}%', xy=(i, max(b, o) + 30), 
                    ha='center', fontsize=9, color='green', fontweight='bold')
    
    ax.set_xticks(x)
    ax.set_xticklabels([n[:12] + '...' if len(n) > 12 else n for n in names_d], 
                       rotation=15, ha='right')
    ax.set_ylabel('Power (mW)', fontsize=11)
    ax.set_title('(b) Display Optimization Comparison', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3, axis='y')
    
    # (c) 续航时间影响（5000mAh电池）
    ax = axes[1, 0]
    
    scenarios = ['Light Use\n(Reading)', 'Normal Use\n(Browsing)', 
                 'Heavy Use\n(Video)', 'Extreme Use\n(Gaming+5G)']
    power_baseline = [500, 1500, 3500, 5000]
    power_optimized = [300, 900, 2000, 3000]
    
    runtime_baseline = [analyzer.energy_Wh / (p/1000) for p in power_baseline]
    runtime_optimized = [analyzer.energy_Wh / (p/1000) for p in power_optimized]
    
    x = np.arange(len(scenarios))
    
    ax.bar(x - width/2, runtime_baseline, width, label='Default', color='#95A5A6')
    ax.bar(x + width/2, runtime_optimized, width, label='Optimized', color='#27AE60')
    
    for i, (rb, ro) in enumerate(zip(runtime_baseline, runtime_optimized)):
        improvement = (ro - rb) / rb * 100
        ax.annotate(f'+{improvement:.0f}%', xy=(i + width/2, ro + 0.3),
                    ha='center', fontsize=9, color='green', fontweight='bold')
    
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios)
    ax.set_ylabel('Runtime (hours)', fontsize=11)
    ax.set_title('(c) Battery Life Improvement (5000mAh)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3, axis='y')
    
    # (d) 电池老化对续航的影响
    ax = axes[1, 1]
    
    aging_timeline = aging.analyze_capacity_degradation_timeline()
    N_values = aging_timeline['N_values']
    capacity_values = aging_timeline['capacity_Ah']
    retention = aging_timeline['retention_pct']
    
    # 计算续航时间（正常使用1.5W）
    V_avg = 3.7
    P_normal = 1.5
    runtimes = [Q * V_avg / P_normal for Q in capacity_values]
    
    ax2 = ax.twinx()
    
    line1, = ax.plot(N_values, runtimes, 'b-o', linewidth=2, markersize=6, label='Runtime')
    line2, = ax2.plot(N_values, retention, 'r--s', linewidth=2, markersize=5, label='Capacity Retention')
    
    ax2.axhline(y=80, color='red', linestyle=':', alpha=0.7)
    ax2.annotate('80% EOL Threshold', xy=(600, 81), fontsize=9, color='red')
    
    ax.set_xlabel('Cycle Number', fontsize=11)
    ax.set_ylabel('Runtime (hours)', fontsize=11, color='blue')
    ax2.set_ylabel('Capacity Retention (%)', fontsize=11, color='red')
    ax.set_title('(d) Battery Aging Impact on Runtime', fontsize=12, fontweight='bold')
    
    ax.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    
    lines = [line1, line2]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='center right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/battery_model/fig_user_recommendations_detailed.png', 
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_user_recommendations_detailed.png")


def generate_os_strategies_figure():
    """生成操作系统策略可视化图表"""
    
    aging = AgingImpactAnalysis()
    
    fig = plt.figure(figsize=(18, 12))
    gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # (a) DVFS策略对比
    ax = fig.add_subplot(gs[0, 0])
    
    soc = ModelSoC()
    
    # Race-to-idle: 3GHz运行1秒
    soc.reset(25)
    race_results = [soc.step(3e9, 25, 0.1) for _ in range(10)]
    race_energy = sum([r['P_total'] * 0.1 for r in race_results])
    
    # Pace-to-idle: 1GHz运行3秒
    soc.reset(25)
    pace_results = [soc.step(1e9, 25, 0.1) for _ in range(30)]
    pace_energy = sum([r['P_total'] * 0.1 for r in pace_results])
    
    strategies = ['Race-to-Idle\n(3GHz x 1s)', 'Pace-to-Idle\n(1GHz x 3s)']
    energies = [race_energy, pace_energy]
    colors_bar = ['#E74C3C', '#27AE60']
    
    bars = ax.bar(strategies, energies, color=colors_bar, width=0.5)
    
    savings_pct = (race_energy - pace_energy) / race_energy * 100
    ax.annotate(f'Saves {savings_pct:.0f}%', xy=(1, pace_energy + 50),
                ha='center', fontsize=11, color='green', fontweight='bold')
    
    ax.set_ylabel('Total Energy (mJ)', fontsize=11)
    ax.set_title('(a) DVFS Strategy Energy Comparison\n(Same workload)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # (b) 网络功耗对比
    ax = fig.add_subplot(gs[0, 1])
    
    networks = ['WiFi\n(Home)', '5G Good\n(Near Tower)', '5G Weak\n(Cell Edge)', 'LTE\n(Indoor)']
    powers = [200, 800, 1700, 500]
    colors_net = ['#27AE60', '#3498DB', '#E74C3C', '#F39C12']
    
    ax.bar(networks, powers, color=colors_net, width=0.6)
    
    for i, p in enumerate(powers):
        ax.text(i, p + 50, f'{p}mW', ha='center', fontsize=10)
    
    ax.set_ylabel('Power (mW)', fontsize=11)
    ax.set_title('(b) Network Interface Power Comparison', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # (c) 唤醒对齐效果
    ax = fig.add_subplot(gs[0, 2])
    
    np.random.seed(42)
    t_unaligned = np.sort(np.random.uniform(0, 60, 20))
    t_aligned = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55])
    
    ax.eventplot([t_unaligned], lineoffsets=1, colors='red', linewidths=2, label='Unaligned')
    ax.eventplot([t_aligned], lineoffsets=0.5, colors='green', linewidths=2, label='Aligned')
    
    ax.set_xlim([0, 60])
    ax.set_ylim([0, 1.5])
    ax.set_xlabel('Time (seconds)', fontsize=11)
    ax.set_yticks([0.5, 1.0])
    ax.set_yticklabels(['Aligned\n(12 wakeups)', 'Unaligned\n(20 wakeups)'])
    ax.set_title('(c) Wake Alignment Strategy', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3, axis='x')
    
    # (d) 温度老化加速
    ax = fig.add_subplot(gs[1, 0])
    
    temp_analysis = aging.analyze_temperature_acceleration()
    temps = temp_analysis['temperature_C']
    factors = temp_analysis['acceleration_factor']
    lifespan = temp_analysis['equivalent_lifespan_cycles']
    
    ax2 = ax.twinx()
    
    ax.bar(temps, factors, color='#E74C3C', alpha=0.7, label='Aging Factor', width=4)
    ax2.plot(temps, lifespan, 'b-o', linewidth=2, markersize=6, label='Equivalent Lifespan')
    
    ax.axvspan(15, 35, alpha=0.2, color='green', label='Optimal Range')
    
    ax.set_xlabel('Temperature (C)', fontsize=11)
    ax.set_ylabel('Aging Acceleration Factor (x)', fontsize=11, color='red')
    ax2.set_ylabel('Equivalent Cycle Life', fontsize=11, color='blue')
    ax.set_title('(d) Temperature Impact on Battery Life', fontsize=12, fontweight='bold')
    
    ax.tick_params(axis='y', labelcolor='red')
    ax2.tick_params(axis='y', labelcolor='red')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # (e) 设备功耗分布对比
    ax = fig.add_subplot(gs[1, 1])
    
    devices = CrossDeviceGeneralization.get_device_profiles()
    device_names = ['Smartphone', 'Tablet', 'Smartwatch', 'Laptop', 'Earbuds']
    battery_sizes = [17.5, 40, 1.5, 75, 0.2]
    runtimes = [12, 12, 48, 10, 6]
    
    avg_powers = [b*1000/r for b, r in zip(battery_sizes, runtimes)]
    colors_dev = ['#3498DB', '#9B59B6', '#1ABC9C', '#E74C3C', '#F39C12']
    
    ax.barh(device_names, avg_powers, color=colors_dev)
    
    for i, (p, b) in enumerate(zip(avg_powers, battery_sizes)):
        ax.text(p + 100, i, f'{p:.0f}mW\n({b}Wh)', va='center', fontsize=9)
    
    ax.set_xlabel('Average Power (mW)', fontsize=11)
    ax.set_title('(e) Device Power Profiles', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='x')
    
    # (f) 建议优先级矩阵
    ax = fig.add_subplot(gs[1, 2])
    ax.axis('off')
    
    table_data = [
        ['Dark Mode', 'HIGH', 'Display', '35-50%'],
        ['Prefer WiFi', 'HIGH', 'Connectivity', '50-70%'],
        ['Power Saver', 'HIGH', 'Performance', '40-60%'],
        ['Limit Background', 'MEDIUM', 'Background', '30-50%'],
        ['60Hz Refresh', 'MEDIUM', 'Display', '10-20%'],
        ['Disable GPS', 'MEDIUM', 'Location', '100%'],
        ['BT Management', 'LOW', 'Connectivity', '5-15%'],
    ]
    
    table = ax.table(
        cellText=table_data,
        colLabels=['Optimization', 'Priority', 'Category', 'Savings'],
        loc='center',
        cellLoc='center',
        colWidths=[0.35, 0.15, 0.2, 0.2]
    )
    
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.8)
    
    for i in range(len(table_data) + 1):
        for j in range(4):
            cell = table[(i, j)]
            if i == 0:
                cell.set_facecolor('#2C3E50')
                cell.set_text_props(color='white', fontweight='bold')
            elif j == 1:
                if i <= 3:
                    cell.set_facecolor('#FFCCCC')
                elif i <= 6:
                    cell.set_facecolor('#FFFFCC')
                else:
                    cell.set_facecolor('#CCFFCC')
    
    ax.set_title('(f) User Optimization Priority Matrix', fontsize=12, fontweight='bold', pad=20)
    
    plt.suptitle('OS Power Management Strategies Analysis', fontsize=14, fontweight='bold', y=0.98)
    
    plt.savefig('/workspace/battery_model/fig_os_strategies_comprehensive.png',
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_os_strategies_comprehensive.png")


def generate_cross_device_figure():
    """生成跨设备分析图表"""
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    
    # (a) 设备能耗结构对比
    ax = axes[0]
    
    devices_data = {
        'Smartphone': {'Display': 35, 'SoC': 30, 'Comm': 20, 'Other': 15},
        'Tablet': {'Display': 50, 'SoC': 25, 'Comm': 15, 'Other': 10},
        'Smartwatch': {'Display': 35, 'Sensors': 25, 'BLE': 25, 'Other': 15},
        'Laptop': {'CPU/GPU': 50, 'Display': 25, 'Storage': 15, 'Other': 10},
        'Earbuds': {'Audio DSP': 45, 'BLE': 35, 'Amp': 20, 'Other': 0}
    }
    
    device_list = list(devices_data.keys())
    categories = ['Display', 'SoC', 'Comm', 'CPU/GPU', 'Sensors', 'BLE', 
                  'Audio DSP', 'Amp', 'Storage', 'Other']
    category_colors = {
        'Display': '#3498DB', 'SoC': '#E74C3C', 'Comm': '#2ECC71',
        'CPU/GPU': '#9B59B6', 'Sensors': '#F39C12', 'BLE': '#1ABC9C',
        'Audio DSP': '#E67E22', 'Amp': '#34495E', 'Storage': '#95A5A6', 'Other': '#BDC3C7'
    }
    
    bottom = np.zeros(len(device_list))
    
    for cat in categories:
        values = [devices_data[dev].get(cat, 0) for dev in device_list]
        if sum(values) > 0:
            ax.barh(device_list, values, left=bottom, label=cat, color=category_colors[cat])
            bottom += values
    
    ax.set_xlabel('Power Distribution (%)', fontsize=11)
    ax.set_title('(a) Power Structure by Device Type', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9, ncol=2)
    ax.grid(True, alpha=0.3, axis='x')
    
    # (b) 模型适配指南表格
    ax = axes[1]
    ax.axis('off')
    
    adapt_data = [
        ['Smartphone', 'Full compatibility', 'No changes needed'],
        ['Tablet', 'Scale display weight', 'Simplify cellular'],
        ['Smartwatch', 'Add sensor models', 'Remove 5G/GNSS'],
        ['Laptop', 'Extend CPU/GPU', 'Add fan model'],
        ['Earbuds', 'BLE + Audio only', 'Adjust thermal'],
        ['E-Reader', 'E-ink bistable model', 'Remove most subsystems']
    ]
    
    table = ax.table(
        cellText=adapt_data,
        colLabels=['Device', 'Core Adaptation', 'Key Changes'],
        loc='center',
        cellLoc='center',
        colWidths=[0.25, 0.35, 0.35]
    )
    
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.3, 2.0)
    
    for i in range(len(adapt_data) + 1):
        for j in range(3):
            cell = table[(i, j)]
            if i == 0:
                cell.set_facecolor('#2C3E50')
                cell.set_text_props(color='white', fontweight='bold')
    
    ax.set_title('(b) Model Adaptation Guide', fontsize=12, fontweight='bold', pad=20)
    
    plt.suptitle('Cross-Device Framework Generalization', fontsize=14, fontweight='bold', y=0.98)
    
    plt.savefig('/workspace/battery_model/fig_cross_device_analysis.png',
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Generated: fig_cross_device_analysis.png")


def main():
    """主函数"""
    print("=" * 60)
    print("Generating Recommendation Visualizations")
    print("=" * 60)
    
    print("\n[1/3] User recommendations figure...")
    generate_user_recommendations_figure()
    
    print("[2/3] OS strategies figure...")
    generate_os_strategies_figure()
    
    print("[3/3] Cross-device analysis figure...")
    generate_cross_device_figure()
    
    print("\n" + "=" * 60)
    print("All recommendation figures generated!")
    print("=" * 60)


if __name__ == "__main__":
    main()
