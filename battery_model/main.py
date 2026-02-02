"""
电池SOC-能耗耦合连续时间模型主程序
Main Program: Coupled SOC-Energy Consumption Continuous-Time Model

功能:
1. SOC与耗能关系的连续时间建模
2. 基于马尔科夫链的用户行为建模和时间区块划分
3. 卡尔曼滤波SOC状态估计
4. 多目标优化电池寿命预测
5. 剩余使用时间预测

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
import sys
import os

# 确保模块可以被导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from soc_energy_coupled_model import CoupledSOCEnergyModel, create_default_model, BatteryParameters
from markov_user_behavior import MarkovUserBehaviorModel, MarkovTimeBlockPredictor, TimeMode
from kalman_filter_soc import ExtendedKalmanFilter, UnscentedKalmanFilter, AdaptiveEKF
from multi_objective_optimizer import WeightedSumOptimizer, NSGA2Optimizer, BatteryLifePredictor
from remaining_time_predictor import IntegratedRemainingTimePredictor, create_predictor, PredictionConfig


def print_header(title: str):
    """打印标题"""
    print("\n" + "=" * 70)
    print(f" {title}")
    print("=" * 70)


def print_section(title: str):
    """打印小节标题"""
    print(f"\n--- {title} ---")


def demo_coupled_model():
    """演示SOC-能耗耦合模型"""
    print_header("1. SOC-能耗耦合连续时间模型演示")
    
    # 创建模型
    model = create_default_model()
    
    print("电池参数:")
    print(f"  标称容量: {model.bp.Q_nom} mAh")
    print(f"  标称电压: {model.bp.V_nom} V")
    print(f"  PMIC效率: {model.bp.eta_conv:.1%}")
    
    # 定义简单状态函数
    def simple_state(t):
        hour = (t / 3600) % 24
        if hour < 7:  # 睡眠
            return {
                'frequency': 0.3e9, 'cpu_load': 0.02, 'brightness': 0,
                'refresh_rate': 1, 'apl': 0, 'data_rate': 0,
                'distance': 100, 'snr': 40, 'gnss_on': False,
                'bt_audio': False, 'bt_interval': 1000
            }
        elif hour < 18:  # 日间使用
            return {
                'frequency': 1.5e9, 'cpu_load': 0.4, 'brightness': 500,
                'refresh_rate': 60, 'apl': 50, 'data_rate': 30,
                'distance': 200, 'snr': 35, 'gnss_on': False,
                'bt_audio': False, 'bt_interval': 100
            }
        else:  # 晚间休闲
            return {
                'frequency': 2.0e9, 'cpu_load': 0.6, 'brightness': 600,
                'refresh_rate': 90, 'apl': 60, 'data_rate': 80,
                'distance': 150, 'snr': 38, 'gnss_on': True,
                'bt_audio': True, 'bt_interval': 50
            }
    
    print_section("运行24小时仿真")
    
    # 仿真24小时
    result = model.simulate(
        t_span=(0, 24 * 3600),
        y0=np.array([1.0, 298.15]),  # 初始SOC=100%, T=25°C
        state_func=simple_state,
        t_eval=np.linspace(0, 24*3600, 1440)
    )
    
    print(f"仿真状态: {'成功' if result['success'] else '失败'}")
    print(f"初始SOC: 100%")
    print(f"最终SOC: {result['soc'][-1]:.1%}")
    print(f"初始温度: 25.0°C")
    print(f"最终温度: {result['temperature'][-1] - 273.15:.1f}°C")
    print(f"最高温度: {np.max(result['temperature']) - 273.15:.1f}°C")
    print(f"平均功耗: {np.mean(result['power_total']):.2f} W")
    print(f"最大功耗: {np.max(result['power_total']):.2f} W")
    print(f"总能量消耗: {np.trapz(result['power_total'], result['time']/3600):.1f} Wh")
    
    return result


def demo_markov_chain():
    """演示马尔科夫链用户行为模型"""
    print_header("2. 时间非齐次马尔科夫链用户行为模型")
    
    model = MarkovUserBehaviorModel(seed=42)
    
    print("时间区块划分:")
    print("  睡眠模式: 23:00 - 7:00")
    print("  工作模式: 9:00-12:00, 14:00-18:00")
    print("  休闲模式: 7:00-9:00, 12:00-14:00, 18:00-23:00")
    
    print_section("转移矩阵")
    
    print("\n睡眠模式转移矩阵:")
    print("         Deep   Light  Stream Gaming")
    for i, name in enumerate(['Deep', 'Light', 'Stream', 'Gaming']):
        row = model.P_sleep[i]
        print(f"{name:7} [{row[0]:.3f}  {row[1]:.3f}  {row[2]:.3f}  {row[3]:.3f}]")
    
    print("\n工作模式转移矩阵:")
    for i, name in enumerate(['Deep', 'Light', 'Stream', 'Gaming']):
        row = model.P_work[i]
        print(f"{name:7} [{row[0]:.3f}  {row[1]:.3f}  {row[2]:.3f}  {row[3]:.3f}]")
    
    print("\n休闲模式转移矩阵:")
    for i, name in enumerate(['Deep', 'Light', 'Stream', 'Gaming']):
        row = model.P_leisure[i]
        print(f"{name:7} [{row[0]:.3f}  {row[1]:.3f}  {row[2]:.3f}  {row[3]:.3f}]")
    
    print_section("稳态分布")
    
    for mode in TimeMode:
        pi = model.compute_stationary_distribution(mode)
        print(f"\n{mode.name}模式稳态分布:")
        for i, state_name in enumerate(model.states):
            print(f"  {state_name}: {pi[i]:.1%}")
    
    print_section("24小时用户行为仿真")
    
    sim_result = model.simulate(duration_hours=24, start_hour=0)
    
    states = sim_result['states']
    print("\n各状态时间占比:")
    for i, name in enumerate(model.states):
        ratio = np.mean(states == i)
        print(f"  {name}: {ratio:.1%}")
    
    print_section("时间区块能耗预测")
    
    predictor = MarkovTimeBlockPredictor(model)
    
    # 预测各区块消耗
    blocks = ['morning_work', 'lunch_leisure', 'afternoon_work', 'evening_leisure']
    
    print("\n区块能耗统计 (20次蒙特卡罗):")
    for block in blocks:
        stats = predictor.predict_block_consumption(block, num_samples=20)
        print(f"  {block}:")
        print(f"    时长: {stats['duration_hours']:.1f}h")
        print(f"    平均能耗: {stats['mean_energy_wh']:.2f} ± {stats['std_energy_wh']:.2f} Wh")
    
    return sim_result


def demo_kalman_filter():
    """演示卡尔曼滤波SOC估计"""
    print_header("3. 扩展卡尔曼滤波SOC状态估计")
    
    print("卡尔曼滤波器类型:")
    print("  - EKF: 扩展卡尔曼滤波器")
    print("  - UKF: 无迹卡尔曼滤波器")
    print("  - AEKF: 自适应扩展卡尔曼滤波器")
    
    # 测试EKF
    ekf = ExtendedKalmanFilter(battery_capacity_mah=4000)
    ekf.initialize(soc_init=0.95, temp_init=298.15, rint_init=0.08)
    
    print_section("EKF性能测试")
    
    # 模拟1小时放电
    dt = 1.0  # 1秒步长
    I_load = 1.5  # 1.5A负载
    true_soc = 0.95
    
    for t in range(3600):  # 1小时
        # 真实SOC变化
        true_soc -= I_load / 4.0 / 3600
        true_soc = max(0, true_soc)
        
        # 模拟测量 (带噪声)
        V_true = 3.0 + 0.8 * true_soc + 0.3 * true_soc**2 - 0.1 * true_soc**3
        V_meas = V_true - I_load * 0.08 + np.random.randn() * 0.01
        I_meas = I_load + np.random.randn() * 0.01
        
        ekf.step(V_meas, I_meas, dt)
    
    soc_est, soc_std = ekf.get_soc_estimate()
    
    print(f"\n1小时放电测试 (1.5A恒流):")
    print(f"  真实SOC: {true_soc:.4f} ({true_soc:.2%})")
    print(f"  估计SOC: {soc_est:.4f} ({soc_est:.2%})")
    print(f"  估计标准差: {soc_std:.4f}")
    print(f"  绝对误差: {abs(true_soc - soc_est):.4f}")
    print(f"  相对误差: {abs(true_soc - soc_est) / true_soc * 100:.2f}%")
    
    state = ekf.get_state_estimate()
    print(f"\n完整状态估计:")
    print(f"  SOC: {state['soc']:.2%} ± {state['soc_std']:.4f}")
    print(f"  温度: {state['temperature'] - 273.15:.2f}°C ± {state['temperature_std']:.2f}K")
    print(f"  内阻: {state['internal_resistance']*1000:.2f} mΩ ± {state['rint_std']*1000:.2f} mΩ")
    
    return ekf


def demo_multi_objective():
    """演示多目标优化"""
    print_header("4. 多目标优化电池寿命预测")
    
    print("优化目标:")
    print("  1. 最大化剩余使用时间")
    print("  2. 最大化用户体验 (性能)")
    print("  3. 最小化温度峰值 (保护电池)")
    
    print_section("加权求和法优化")
    
    weighted_opt = WeightedSumOptimizer()
    
    # 不同优先级测试
    scenarios = [
        ('balanced', '平衡模式'),
        ('battery', '省电模式'),
        ('performance', '性能模式')
    ]
    
    print("\n当前SOC: 80%, 温度: 27°C")
    
    for priority, name in scenarios:
        result = weighted_opt.optimize(
            soc_current=0.8,
            temperature=300,
            user_priority=priority
        )
        
        print(f"\n{name}:")
        print(f"  推荐CPU频率: {result['optimal_cpu_freq']:.2f} GHz")
        print(f"  推荐亮度: {result['optimal_brightness']:.0f} nits")
        print(f"  推荐刷新率: {result['optimal_refresh_rate']:.0f} Hz")
        print(f"  预测功耗: {result['predicted_power']:.2f} W")
        print(f"  预测电池寿命: {result['predicted_battery_life_hours']:.1f} 小时")
    
    print_section("NSGA-II Pareto优化")
    
    nsga2 = NSGA2Optimizer()
    nsga2.config.n_generations = 30
    nsga2.config.population_size = 20
    
    result = nsga2.optimize(soc_current=0.8, temperature=300)
    
    print(f"\nPareto最优解数量: {result['n_pareto_solutions']}")
    print(f"\n推荐解 (TOPSIS选择):")
    print(f"  CPU频率: {result['recommended_solution']['cpu_freq']:.2f} GHz")
    print(f"  亮度: {result['recommended_solution']['brightness']:.0f} nits")
    print(f"  刷新率: {result['recommended_solution']['refresh_rate']:.0f} Hz")
    print(f"  预测电池寿命: {result['predicted_battery_life_hours']:.1f} 小时")
    
    return result


def demo_integrated_predictor():
    """演示集成剩余时间预测"""
    print_header("5. 集成剩余使用时间预测")
    
    # 创建预测器
    predictor = create_predictor(battery_capacity_mah=4000, use_kalman=True)
    predictor.initialize(soc_init=0.75, temp_init=300)
    
    current_hour = 14.0  # 下午2点
    
    print(f"当前状态:")
    print(f"  SOC: {predictor.current_soc:.1%}")
    print(f"  温度: {predictor.current_temperature - 273.15:.1f}°C")
    print(f"  当前时间: {current_hour:.0f}:00")
    
    print_section("确定性预测")
    
    det_result = predictor.predict_remaining_time_deterministic(current_hour)
    print(f"预测剩余时间: {det_result['remaining_hours']:.2f} 小时")
    print(f"低电量警告时间: {det_result['warning_hours']:.2f} 小时")
    print(f"平均功耗: {det_result['average_power']:.2f} W")
    
    print_section("时间区块预测")
    
    block_result = predictor.predict_by_time_blocks(current_hour)
    print(f"总预测剩余时间: {block_result['total_remaining_hours']:.2f} 小时")
    print("\n各时间区块消耗:")
    for name, details in block_result['block_predictions'].items():
        depleted = " [电量耗尽]" if details.get('depleted_in_block', False) else ""
        print(f"  {name}:")
        print(f"    时长: {details['duration_hours']:.1f} 小时")
        print(f"    功耗: {details['expected_power']:.1f} W")
        print(f"    SOC消耗: {details['soc_consumed']:.1%}{depleted}")
    
    print_section("优化预测 (含不确定性)")
    
    opt_result = predictor.get_optimized_prediction(current_hour, 'balanced')
    
    # 打印格式化报告
    report = predictor.format_prediction_report(opt_result)
    print(report)
    
    return predictor, opt_result


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print(" 电池SOC-能耗耦合连续时间模型")
    print(" Battery SOC-Energy Consumption Coupled Continuous-Time Model")
    print("=" * 70)
    
    print("\n本程序集成了以下功能:")
    print("1. SOC-能耗耦合连续时间微分方程模型")
    print("2. 时间非齐次马尔科夫链用户行为建模")
    print("3. 扩展卡尔曼滤波SOC状态估计")
    print("4. 多目标优化电池寿命预测")
    print("5. 剩余使用时间预测 (含时间区块划分)")
    
    # 运行各模块演示
    try:
        # 1. 耦合模型演示
        coupled_result = demo_coupled_model()
        
        # 2. 马尔科夫链演示
        markov_result = demo_markov_chain()
        
        # 3. 卡尔曼滤波演示
        ekf = demo_kalman_filter()
        
        # 4. 多目标优化演示
        opt_result = demo_multi_objective()
        
        # 5. 集成预测演示
        predictor, prediction = demo_integrated_predictor()
        
        print_header("总结")
        print("""
本系统成功实现了:

1. SOC-能耗耦合模型:
   - 电化学-热耦合微分方程
   - 各模块功耗物理建模
   - 温度效率因子

2. 用户行为建模:
   - 时间非齐次马尔科夫链
   - 睡眠/工作/休闲模式切换
   - 时间区块能耗统计

3. 状态估计:
   - 扩展卡尔曼滤波 (EKF)
   - 无迹卡尔曼滤波 (UKF)
   - 自适应噪声协方差

4. 多目标优化:
   - 加权求和法
   - NSGA-II Pareto优化
   - 电池寿命/性能/温度权衡

5. 剩余时间预测:
   - 确定性预测
   - 概率预测 (蒙特卡罗)
   - 时间区块分解预测
   - 置信区间估计
""")
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
