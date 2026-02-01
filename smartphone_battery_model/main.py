#!/usr/bin/env python3
"""
智能手机电池耗电建模主程序
Smartphone Battery Discharge Modeling Main Program

运行完整的电池建模、模拟、分析和可视化流程
Runs the complete battery modeling, simulation, analysis, and visualization pipeline

Usage:
    python main.py
    python main.py --output-dir ./results
    python main.py --scenarios idle,normal,heavy --duration 12
"""

import os
import sys
import argparse
import json
import numpy as np
from datetime import datetime

# 导入模块
from battery_model import SmartphoneBatteryModel, BatteryParameters
from visualization import BatteryVisualizer, generate_all_visualizations
from analysis import BatteryAnalyzer


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='Smartphone Battery Discharge Modeling System'
    )
    parser.add_argument(
        '--output-dir', '-o',
        default='./output',
        help='Output directory for results (default: ./output)'
    )
    parser.add_argument(
        '--scenarios', '-s',
        default='idle,work,leisure,heavy,normal',
        help='Comma-separated list of scenarios (default: idle,work,leisure,heavy,normal)'
    )
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=24,
        help='Simulation duration in hours (default: 24)'
    )
    parser.add_argument(
        '--battery-capacity', '-c',
        type=float,
        default=4.0,
        help='Battery capacity in Ah (default: 4.0)'
    )
    parser.add_argument(
        '--no-visualization',
        action='store_true',
        help='Skip visualization generation'
    )
    
    return parser.parse_args()


def print_header():
    """打印程序头部信息"""
    header = """
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║     📱 智能手机电池耗电建模系统                                              ║
║     Smartphone Battery Discharge Modeling System                             ║
║                                                                              ║
║     基于电化学-热耦合方程的连续时间SOC预测模型                               ║
║     Continuous-time SOC prediction based on electrochemical-thermal coupling ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
    """
    print(header)


def run_simulations(model: SmartphoneBatteryModel, 
                   scenarios: list,
                   duration: float) -> dict:
    """运行多场景模拟
    
    Args:
        model: 电池模型实例
        scenarios: 场景列表
        duration: 模拟时长
    
    Returns:
        各场景历史数据字典
    """
    histories = {}
    
    print("\n📊 Running Simulations...")
    print("-" * 50)
    
    for i, scenario in enumerate(scenarios, 1):
        print(f"  [{i}/{len(scenarios)}] Simulating '{scenario}' scenario...", end=" ")
        
        history = model.simulate(
            duration_hours=duration,
            initial_soc=1.0,
            scenario=scenario,
            start_hour=8.0,
            dt=60  # 1分钟步长
        )
        
        # 计算关键指标
        soc = np.array(history['SOC'])
        time = np.array(history['time'])
        idx = np.where(soc <= 0.05)[0]
        drain_time = time[idx[0]] if len(idx) > 0 else time[-1]
        
        print(f"✓ (Drain time: {drain_time:.1f}h, Avg power: {np.mean(history['P_total']):.2f}W)")
        
        histories[scenario] = history
    
    return histories


def print_summary(histories: dict):
    """打印模拟结果摘要"""
    print("\n📈 Simulation Summary")
    print("=" * 70)
    print(f"{'Scenario':<12} {'Drain Time':>12} {'Avg Power':>12} {'Max Temp':>12} {'Final SOC':>12}")
    print("-" * 70)
    
    for scenario, history in histories.items():
        soc = np.array(history['SOC'])
        time = np.array(history['time'])
        power = np.array(history['P_total'])
        temp = np.array(history['T_batt'])
        
        idx = np.where(soc <= 0.05)[0]
        drain_time = time[idx[0]] if len(idx) > 0 else time[-1]
        
        print(f"{scenario:<12} {drain_time:>10.1f}h {np.mean(power):>11.2f}W {np.max(temp):>10.1f}°C {soc[-1]*100:>10.1f}%")
    
    print("-" * 70)


def print_predictions(model: SmartphoneBatteryModel):
    """打印剩余时间预测"""
    print("\n⏱️  Remaining Time Predictions")
    print("=" * 70)
    print(f"{'SOC':<10} {'Idle':>12} {'Normal':>12} {'Work':>12} {'Heavy':>12}")
    print("-" * 70)
    
    for soc in [1.0, 0.8, 0.6, 0.4, 0.2]:
        row = f"{soc*100:.0f}%"
        for scenario in ['idle', 'normal', 'work', 'heavy']:
            remaining, _ = model.predict_remaining_time(soc, scenario)
            row += f"{remaining:>12.1f}h"
        print(row)
    
    print("-" * 70)


def save_results(histories: dict, output_dir: str):
    """保存模拟结果数据"""
    data_dir = os.path.join(output_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)
    
    # 保存为JSON
    results = {}
    for scenario, history in histories.items():
        results[scenario] = {
            'time': [float(t) for t in history['time']],
            'SOC': [float(s) for s in history['SOC']],
            'T_batt': [float(t) for t in history['T_batt']],
            'V_batt': [float(v) for v in history['V_batt']],
            'I_total': [float(i) for i in history['I_total']],
            'P_total': [float(p) for p in history['P_total']],
        }
    
    with open(os.path.join(data_dir, 'simulation_results.json'), 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n💾 Data saved to {data_dir}/simulation_results.json")


def generate_report(analyzer: BatteryAnalyzer, histories: dict, output_dir: str):
    """生成分析报告"""
    report = analyzer.generate_analysis_report(histories)
    
    report_path = os.path.join(output_dir, 'analysis_report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)
    
    print(f"📄 Report saved to {report_path}")
    
    return report


def main():
    """主函数"""
    # 解析参数
    args = parse_args()
    
    # 打印头部
    print_header()
    
    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    viz_dir = os.path.join(args.output_dir, 'visualizations')
    os.makedirs(viz_dir, exist_ok=True)
    
    # 解析场景
    scenarios = [s.strip() for s in args.scenarios.split(',')]
    
    # 打印配置
    print("\n⚙️  Configuration:")
    print(f"   Output Directory: {args.output_dir}")
    print(f"   Scenarios: {', '.join(scenarios)}")
    print(f"   Duration: {args.duration} hours")
    print(f"   Battery Capacity: {args.battery_capacity} Ah")
    
    # 创建电池参数
    battery_params = BatteryParameters(Q_max=args.battery_capacity)
    
    # 创建模型
    print("\n🔋 Initializing Battery Model...")
    model = SmartphoneBatteryModel(battery_params)
    print("   ✓ Model initialized")
    
    # 运行模拟
    histories = run_simulations(model, scenarios, args.duration)
    
    # 打印摘要
    print_summary(histories)
    
    # 打印预测
    print_predictions(model)
    
    # 保存数据
    save_results(histories, args.output_dir)
    
    # 分析
    print("\n🔬 Running Analysis...")
    analyzer = BatteryAnalyzer(model)
    
    # 敏感性分析
    sensitivity = analyzer.sensitivity_analysis()
    print("\n   Sensitivity Analysis Results:")
    for param, value in sorted(sensitivity.items(), key=lambda x: abs(x[1]), reverse=True):
        print(f"   - {param}: {value:.3f}")
    
    # 模型验证
    print("\n   Model Validation:")
    validation = analyzer.validate_against_typical_values()
    for test in validation['tests']:
        status = "✓" if test['passed'] else "✗"
        print(f"   {status} {test['name']}: {test['actual']}")
    print(f"\n   Validation Pass Rate: {validation['pass_rate']:.0f}%")
    
    # 生成报告
    report = generate_report(analyzer, histories, args.output_dir)
    
    # 生成可视化
    if not args.no_visualization:
        print("\n🎨 Generating Visualizations...")
        try:
            generate_all_visualizations(model, histories, output_dir=viz_dir)
            print(f"   ✓ Visualizations saved to {viz_dir}/")
        except Exception as e:
            print(f"   ⚠ Visualization generation failed: {e}")
    
    # 完成
    print("\n" + "=" * 70)
    print("✅ Analysis Complete!")
    print(f"   Results saved to: {os.path.abspath(args.output_dir)}")
    print("=" * 70)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
