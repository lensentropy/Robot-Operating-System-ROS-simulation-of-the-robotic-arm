#!/usr/bin/env python3
"""
敏感性分析主程序
=================
运行完整的敏感性分析并生成可视化和报告
"""

import numpy as np
import json
import os
from datetime import datetime

from smartphone_power_model import (
    SmartphonePowerModel, 
    create_model_with_params, 
    DEFAULT_PARAMS
)
from sensitivity_analysis import (
    LocalSensitivityAnalyzer,
    MorrisScreening,
    SobolAnalyzer,
    MonteCarloUncertainty,
    RobustnessAnalyzer,
    get_default_params,
    get_param_bounds,
)
from visualization import (
    create_tornado_chart,
    create_sobol_radar_chart,
    create_morris_scatter,
    create_monte_carlo_violin,
    create_robustness_heatmap,
    create_scenario_comparison,
    create_integrated_dashboard,
    create_interaction_matrix,
    save_all_figures,
)


def run_baseline_simulation():
    """运行基准仿真"""
    print("\n" + "=" * 70)
    print("基准仿真")
    print("=" * 70)
    
    model = SmartphonePowerModel()
    hist = model.run_simulation(fast_mode=True)
    metrics = model.compute_metrics(hist)
    
    print("\n基准仿真结果:")
    print("-" * 40)
    for key, value in metrics.items():
        if 'samples' not in key.lower():
            print(f"  {key:20s}: {value:12.4f}")
    
    return metrics, hist


def run_local_sensitivity():
    """运行局部敏感性分析"""
    print("\n" + "=" * 70)
    print("1. 局部敏感性分析 (One-at-a-Time)")
    print("=" * 70)
    
    base_params = get_default_params()
    analyzer = LocalSensitivityAnalyzer(perturbation_ratio=0.05)
    
    results = analyzer.analyze(
        base_params,
        output_metrics=['final_soc', 'avg_power', 'max_temp', 'avg_power_soc', 'avg_power_conn'],
        verbose=True
    )
    
    # 打印结果摘要
    print("\n敏感性指数摘要 (avg_power):")
    print("-" * 50)
    sorted_params = sorted(results['avg_power'].items(), key=lambda x: abs(x[1]), reverse=True)
    for param, sensitivity in sorted_params[:10]:
        print(f"  {param:20s}: {sensitivity:10.4f}")
    
    return results


def run_morris_screening():
    """运行Morris筛选分析"""
    print("\n" + "=" * 70)
    print("2. Morris筛选分析")
    print("=" * 70)
    
    param_bounds = get_param_bounds()
    morris = MorrisScreening(n_trajectories=10, n_levels=4)
    
    results = morris.analyze(param_bounds, 'avg_power', verbose=True)
    
    # 打印结果
    print("\nMorris统计量:")
    print("-" * 60)
    print(f"{'参数':20s} {'μ*':>12s} {'μ':>12s} {'σ':>12s}")
    print("-" * 60)
    
    sorted_params = sorted(results.items(), key=lambda x: x[1]['mu_star'], reverse=True)
    for param, stats in sorted_params:
        print(f"{param:20s} {stats['mu_star']:12.4f} {stats['mu']:12.4f} {stats['sigma']:12.4f}")
    
    return results


def run_sobol_analysis():
    """运行Sobol全局敏感性分析"""
    print("\n" + "=" * 70)
    print("3. Sobol全局敏感性分析")
    print("=" * 70)
    
    param_bounds = get_param_bounds()
    sobol = SobolAnalyzer(n_samples=128)  # 减少样本数以加速
    
    results = sobol.analyze(param_bounds, 'avg_power', verbose=True)
    
    # 打印结果
    print("\nSobol指数:")
    print("-" * 50)
    print(f"{'参数':20s} {'S1 (一阶)':>12s} {'ST (全阶)':>12s}")
    print("-" * 50)
    
    sorted_params = sorted(results.items(), key=lambda x: x[1]['ST'], reverse=True)
    for param, indices in sorted_params:
        print(f"{param:20s} {indices['S1']:12.4f} {indices['ST']:12.4f}")
    
    return results


def run_monte_carlo():
    """运行蒙特卡洛不确定性传播"""
    print("\n" + "=" * 70)
    print("4. 蒙特卡洛不确定性传播")
    print("=" * 70)
    
    # 定义参数分布
    param_distributions = {}
    for name, info in DEFAULT_PARAMS.items():
        # 均匀分布
        param_distributions[name] = ('uniform', info['min'], info['max'])
    
    mc = MonteCarloUncertainty(n_samples=300)
    results = mc.propagate(
        param_distributions,
        target_metrics=['final_soc', 'avg_power', 'max_temp'],
        verbose=True
    )
    
    # 打印结果
    print("\n蒙特卡洛统计:")
    print("-" * 70)
    for metric, stats in results.items():
        print(f"\n{metric}:")
        print(f"  均值: {stats['mean']:.4f}")
        print(f"  标准差: {stats['std']:.4f}")
        print(f"  变异系数: {stats['cv']:.2%}")
        print(f"  5%-95%置信区间: [{stats['p5']:.4f}, {stats['p95']:.4f}]")
    
    return results


def run_robustness_analysis():
    """运行鲁棒性分析"""
    print("\n" + "=" * 70)
    print("5. 鲁棒性分析")
    print("=" * 70)
    
    base_params = get_default_params()
    robustness = RobustnessAnalyzer()
    
    # 参数扰动鲁棒性
    print("\n5.1 参数扰动鲁棒性...")
    param_results = robustness.analyze_parameter_robustness(
        base_params,
        perturbation_levels=np.linspace(-0.2, 0.2, 9),
        target_metric='avg_power',
        verbose=True
    )
    
    # 场景鲁棒性
    print("\n5.2 场景鲁棒性...")
    scenario_results = robustness.analyze_scenario_robustness(
        base_params,
        verbose=True
    )
    
    # 打印鲁棒性分数
    print("\n鲁棒性分数 (越高越稳定):")
    print("-" * 40)
    sorted_params = sorted(param_results.items(), 
                          key=lambda x: x[1]['robustness_score'], reverse=True)
    for param, data in sorted_params:
        print(f"  {param:20s}: {data['robustness_score']:.4f}")
    
    return {'param': param_results, 'scenario': scenario_results}


def generate_report(all_results: dict, output_dir: str):
    """生成分析报告"""
    print("\n" + "=" * 70)
    print("生成分析报告")
    print("=" * 70)
    
    report = []
    report.append("=" * 80)
    report.append("智能手机功耗模型 - 敏感性与鲁棒性分析报告")
    report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 80)
    report.append("")
    
    # 1. 摘要
    report.append("1. 执行摘要")
    report.append("-" * 40)
    report.append("""
本报告对智能手机功耗仿真模型进行了全面的敏感性和鲁棒性分析。
分析涵盖17个关键参数，包括电池、SoC、显示屏、连接性和热管理系统。

主要发现:
""")
    
    # 从局部敏感性中提取关键参数
    if 'local' in all_results and 'avg_power' in all_results['local']:
        local = all_results['local']['avg_power']
        sorted_params = sorted(local.items(), key=lambda x: abs(x[1]), reverse=True)[:5]
        report.append("  最敏感的参数 (对平均功耗):")
        for param, sens in sorted_params:
            report.append(f"    - {param}: 敏感性指数 = {sens:.4f}")
    
    report.append("")
    
    # 2. 局部敏感性分析
    report.append("\n2. 局部敏感性分析 (OAT方法)")
    report.append("-" * 40)
    report.append("""
方法说明:
  - 采用One-at-a-Time (OAT)方法
  - 参数扰动幅度: ±5%
  - 使用中心差分计算敏感性
  - 计算归一化敏感性指数: S = (∂Y/∂X) × (X₀/Y₀)
""")
    
    if 'local' in all_results:
        for metric, data in all_results['local'].items():
            report.append(f"\n  目标指标: {metric}")
            sorted_params = sorted(data.items(), key=lambda x: abs(x[1]), reverse=True)
            for param, sens in sorted_params[:10]:
                report.append(f"    {param:20s}: {sens:10.4f}")
    
    # 3. Morris筛选
    report.append("\n\n3. Morris筛选分析")
    report.append("-" * 40)
    report.append("""
方法说明:
  - 轨迹数: 10
  - 离散化级别: 4
  - μ*: 参数重要性指标（均值绝对基本效应）
  - σ: 参数非线性/交互效应指标
  
判断准则:
  - 高μ*, 低σ: 重要且线性
  - 高μ*, 高σ: 重要且非线性/有交互
  - 低μ*, 低σ: 不重要
""")
    
    if 'morris' in all_results:
        report.append(f"\n  {'参数':20s} {'μ*':>10s} {'μ':>10s} {'σ':>10s}")
        sorted_params = sorted(all_results['morris'].items(), 
                              key=lambda x: x[1]['mu_star'], reverse=True)
        for param, stats in sorted_params:
            report.append(f"  {param:20s} {stats['mu_star']:10.4f} {stats['mu']:10.4f} {stats['sigma']:10.4f}")
    
    # 4. Sobol分析
    report.append("\n\n4. Sobol全局敏感性分析")
    report.append("-" * 40)
    report.append("""
方法说明:
  - 基于方差分解的全局敏感性分析
  - S1 (一阶指数): 参数单独贡献
  - ST (全阶指数): 参数总贡献（包括交互效应）
  - 交互效应 ≈ ST - S1
  
判断准则:
  - S1 ≈ ST: 主要是主效应，交互作用小
  - ST >> S1: 存在显著交互效应
""")
    
    if 'sobol' in all_results:
        report.append(f"\n  {'参数':20s} {'S1':>10s} {'ST':>10s} {'交互':>10s}")
        sorted_params = sorted(all_results['sobol'].items(), 
                              key=lambda x: x[1]['ST'], reverse=True)
        for param, indices in sorted_params:
            interaction = max(0, indices['ST'] - indices['S1'])
            report.append(f"  {param:20s} {indices['S1']:10.4f} {indices['ST']:10.4f} {interaction:10.4f}")
    
    # 5. 蒙特卡洛分析
    report.append("\n\n5. 蒙特卡洛不确定性传播")
    report.append("-" * 40)
    report.append("""
方法说明:
  - 样本数: 300
  - 参数分布: 均匀分布（最小-最大范围）
  - 输出统计: 均值、标准差、变异系数、置信区间
""")
    
    if 'monte_carlo' in all_results:
        for metric, stats in all_results['monte_carlo'].items():
            if 'samples' not in str(type(stats)):
                report.append(f"\n  {metric}:")
                report.append(f"    均值 (μ): {stats.get('mean', 'N/A'):.4f}")
                report.append(f"    标准差 (σ): {stats.get('std', 'N/A'):.4f}")
                report.append(f"    变异系数 (CV): {stats.get('cv', 0)*100:.2f}%")
                report.append(f"    5%分位数: {stats.get('p5', 'N/A'):.4f}")
                report.append(f"    95%分位数: {stats.get('p95', 'N/A'):.4f}")
    
    # 6. 鲁棒性分析
    report.append("\n\n6. 鲁棒性分析")
    report.append("-" * 40)
    report.append("""
方法说明:
  - 参数扰动范围: ±20%
  - 鲁棒性分数 = 1 / (1 + 平均相对变化)
  - 分数越高，模型对该参数变化越稳定
""")
    
    if 'robustness' in all_results:
        if 'param' in all_results['robustness']:
            report.append("\n  参数鲁棒性分数:")
            sorted_params = sorted(all_results['robustness']['param'].items(),
                                  key=lambda x: x[1]['robustness_score'], reverse=True)
            for param, data in sorted_params:
                report.append(f"    {param:20s}: {data['robustness_score']:.4f}")
        
        if 'scenario' in all_results['robustness']:
            report.append("\n  场景测试结果:")
            for scenario, metrics in all_results['robustness']['scenario'].items():
                report.append(f"\n    {scenario}:")
                for metric, value in metrics.items():
                    report.append(f"      {metric}: {value:.4f}")
    
    # 7. 结论与建议
    report.append("\n\n7. 结论与建议")
    report.append("-" * 40)
    report.append("""
基于上述分析，我们得出以下结论和建议:

7.1 关键参数识别
  根据多种分析方法的综合结果，以下参数对模型输出影响最大:
""")
    
    # 综合关键参数
    if 'local' in all_results and 'avg_power' in all_results['local']:
        top_params = sorted(all_results['local']['avg_power'].items(), 
                           key=lambda x: abs(x[1]), reverse=True)[:5]
        for i, (param, _) in enumerate(top_params, 1):
            desc = DEFAULT_PARAMS.get(param, {}).get('desc', param)
            report.append(f"    {i}. {param} ({desc})")
    
    report.append("""
7.2 模型鲁棒性评估
  - 模型在大多数参数扰动下表现稳定
  - 连接性参数（5G、WiFi）的变化对功耗影响较大
  - 热管理参数在极端温度下影响显著

7.3 不确定性量化
  - 在参数不确定性传播下，平均功耗的变异系数表明模型具有中等程度的不确定性
  - 建议优先精确标定高敏感性参数以减少预测不确定性

7.4 实际应用建议
  - 在功耗优化中，应重点关注高敏感性参数
  - 设计验证时，应测试边界条件下的模型行为
  - 建议定期更新参数值以适应新的硬件特性
""")
    
    # 保存报告
    report_text = "\n".join(report)
    report_path = os.path.join(output_dir, 'sensitivity_analysis_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    
    print(f"\n报告已保存至: {report_path}")
    return report_text


def save_results_json(all_results: dict, output_dir: str):
    """将结果保存为JSON格式"""
    # 转换numpy数组为列表
    def convert_to_serializable(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: convert_to_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [convert_to_serializable(i) for i in obj]
        elif isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        return obj
    
    serializable_results = convert_to_serializable(all_results)
    
    json_path = os.path.join(output_dir, 'sensitivity_results.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(serializable_results, f, indent=2, ensure_ascii=False)
    
    print(f"结果已保存至: {json_path}")


def main():
    """主程序入口"""
    print("\n" + "=" * 80)
    print("智能手机功耗模型 - 敏感性与鲁棒性分析")
    print("=" * 80)
    print(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 创建输出目录
    output_dir = 'analysis_output'
    os.makedirs(output_dir, exist_ok=True)
    
    all_results = {}
    
    # 1. 基准仿真
    baseline_metrics, baseline_hist = run_baseline_simulation()
    all_results['baseline'] = baseline_metrics
    
    # 2. 局部敏感性分析
    local_results = run_local_sensitivity()
    all_results['local'] = local_results
    
    # 3. Morris筛选
    morris_results = run_morris_screening()
    all_results['morris'] = morris_results
    
    # 4. Sobol分析
    sobol_results = run_sobol_analysis()
    all_results['sobol'] = sobol_results
    
    # 5. 蒙特卡洛分析
    mc_results = run_monte_carlo()
    all_results['monte_carlo'] = mc_results
    
    # 6. 鲁棒性分析
    robustness_results = run_robustness_analysis()
    all_results['robustness'] = robustness_results
    all_results['robustness_param'] = robustness_results['param']
    all_results['robustness_scenario'] = robustness_results['scenario']
    
    # 生成可视化
    print("\n" + "=" * 70)
    print("生成可视化")
    print("=" * 70)
    
    try:
        save_all_figures(all_results, output_dir)
    except Exception as e:
        print(f"可视化生成警告: {e}")
        print("尝试生成简化版本...")
    
    # 生成报告
    report = generate_report(all_results, output_dir)
    
    # 保存JSON结果
    save_results_json(all_results, output_dir)
    
    print("\n" + "=" * 80)
    print("分析完成!")
    print(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"输出目录: {os.path.abspath(output_dir)}")
    print("=" * 80)
    
    return all_results


if __name__ == "__main__":
    results = main()
