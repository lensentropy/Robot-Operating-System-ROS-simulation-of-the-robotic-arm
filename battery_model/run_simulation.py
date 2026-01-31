import sys
import os
import time
import numpy as np

# 确保可以导入 src 目录下的模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from coupled_system import (
    CoupledBatterySystem, 
    IdleScenario, 
    VideoStreamingScenario, 
    NavigationScenario, 
    GamingScenario,
    MixedUsageScenario
)
from visualizations import (
    figure_5g_power_analysis,
    figure_gnss_state_dynamics,
    figure_background_stochastic,
    figure_coupled_system_simulation
)

def main():
    print("==================================================")
    print("      智能手机电池放电连续时间建模 (MCM 2026)")
    print("==================================================")
    
    # 1. 确保输出目录存在
    output_dir = 'results'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # 2. 生成组件级分析图表
    print("\n[1/3] 生成组件物理模型分析图表...")
    
    print("  - 生成 5G 功耗热力图 (Shannon-Hartley & Friis)...")
    figure_5g_power_analysis(f'{output_dir}/fig1_5g_physics.png')
    
    print("  - 生成 GNSS 状态机动态图 (Sigmoid Transitions)...")
    figure_gnss_state_dynamics(f'{output_dir}/fig2_gnss_dynamics.png')
    
    print("  - 生成后台随机过程图 (O-U Process + Bursts)...")
    figure_background_stochastic(f'{output_dir}/fig3_background_stochastic.png')
    
    # 3. 运行全系统耦合仿真
    print("\n[2/3] 运行耦合系统场景仿真...")
    system = CoupledBatterySystem()
    
    scenarios = [
        (IdleScenario(), 48),          # 待机 48小时
        (VideoStreamingScenario(), 6), # 视频 6小时
        (NavigationScenario(), 6),     # 导航 6小时
        (GamingScenario(), 4),         # 游戏 4小时
        (MixedUsageScenario(), 24)     # 混合 24小时
    ]
    
    results = {}
    
    for scenario, hours in scenarios:
        print(f"  正在模拟场景: {scenario.name} (时长: {hours}h)...")
        start_time = time.time()
        
        # 运行仿真
        res = system.simulate(scenario, S0=1.0, t_span=(0, hours), dt=0.005)
        results[scenario.name] = res
        
        duration = res['time'][-1]
        final_soc = res['SOC'][-1] * 100
        avg_temp = np.mean(res['temperature']) - 273.15
        
        print(f"    -> 实际续航: {duration:.2f} 小时")
        print(f"    -> 最终 SOC: {final_soc:.1f}%")
        print(f"    -> 平均温度: {avg_temp:.1f}°C")
        print(f"    -> 耗时: {time.time() - start_time:.2f}s")

    # 4. 生成综合结果图
    print("\n[3/3] 生成综合仿真结果可视化...")
    figure_coupled_system_simulation(results, f'{output_dir}/fig4_system_results.png')
    
    print(f"\n✅ 全部完成！结果保存在 '{output_dir}' 目录中。")

if __name__ == "__main__":
    main()
