#!/usr/bin/env python3
"""
用户行为量化分析模块
==================

基于物理模型，计算各种用户操作带来的功耗变化和续航时间影响。

Author: Battery Modeling Framework
Date: 2026-02-01
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery_electro_thermal_aging import BatteryElectroThermalAgingModel
from load_subsystems import (
    Model5G, ModelBluetooth, ModelBackground,
    ModelGNSS, ModelOLED, ModelSoC, IntegratedPowerModel
)


class UserBehaviorAnalysis:
    """用户行为对电池续航影响的量化分析"""
    
    def __init__(self):
        self.battery = BatteryElectroThermalAgingModel()
        self.integrated = IntegratedPowerModel()
        self.battery_capacity_mAh = 5000
        self.V_nominal = 3.8
        self.energy_Wh = self.battery_capacity_mAh * self.V_nominal / 1000
        
    def calculate_runtime(self, power_mW: float) -> float:
        """计算给定功耗下的续航时间（小时）"""
        power_W = power_mW / 1000
        if power_W <= 0:
            return 24.0
        return min(self.energy_Wh / power_W, 24.0)
    
    def analyze_display_behaviors(self) -> Dict:
        """分析显示相关用户行为的影响"""
        oled = self.integrated.model_oled
        
        results = {'title': '显示设置优化', 'behaviors': []}
        
        # 基准场景
        baseline_APL = oled.calculate_APL(240, 240, 240)
        baseline = oled.get_power(baseline_APL, 500, 120)
        baseline_power = baseline['P_total']
        
        # 1. 深色模式
        dark_APL = oled.calculate_APL(30, 30, 30)
        dark = oled.get_power(dark_APL, 500, 120)
        dark_savings = baseline_power - dark['P_total']
        
        results['behaviors'].append({
            'name': '启用深色模式',
            'baseline_mW': baseline_power,
            'optimized_mW': dark['P_total'],
            'savings_mW': dark_savings,
            'savings_pct': dark_savings / baseline_power * 100,
            'physics': 'OLED像素显示黑色时完全关闭，零功耗发光',
            'equation': r'$P_{emit} = \beta \cdot L \cdot APL$',
            'implementation': '系统设置 → 显示 → 深色主题',
            'priority': 'HIGH'
        })
        
        # 2. 亮度降低
        dim = oled.get_power(baseline_APL, 300, 120)
        dim_savings = baseline_power - dim['P_total']
        
        results['behaviors'].append({
            'name': '降低屏幕亮度(500→300nits)',
            'baseline_mW': baseline_power,
            'optimized_mW': dim['P_total'],
            'savings_mW': dim_savings,
            'savings_pct': dim_savings / baseline_power * 100,
            'physics': 'OLED发光功率与亮度近似线性关系',
            'equation': r'$P_{emit} \propto L_{set}$',
            'implementation': '下拉快捷设置调整，或启用自动亮度',
            'priority': 'HIGH'
        })
        
        # 3. 刷新率降低
        lowfps = oled.get_power(baseline_APL, 500, 60)
        lowfps_savings = baseline_power - lowfps['P_total']
        
        results['behaviors'].append({
            'name': '降低刷新率(120→60Hz)',
            'baseline_mW': baseline_power,
            'optimized_mW': lowfps['P_total'],
            'savings_mW': lowfps_savings,
            'savings_pct': lowfps_savings / baseline_power * 100,
            'physics': '驱动IC动态功耗与刷新率线性相关',
            'equation': r'$P_{driver} = k_{drv} \cdot f_{refresh}$',
            'implementation': '设置 → 显示 → 刷新率 → 标准(60Hz)',
            'priority': 'MEDIUM'
        })
        
        # 4. 综合优化
        optimal = oled.get_power(dark_APL, 300, 60)
        optimal_savings = baseline_power - optimal['P_total']
        
        results['behaviors'].append({
            'name': '综合优化(深色+低亮度+60Hz)',
            'baseline_mW': baseline_power,
            'optimized_mW': optimal['P_total'],
            'savings_mW': optimal_savings,
            'savings_pct': optimal_savings / baseline_power * 100,
            'physics': '三种优化效果可叠加',
            'equation': r'$P_{total} = P_{base} + k_{drv}f + \beta L \cdot APL$',
            'implementation': '同时启用以上三项设置',
            'priority': 'HIGH'
        })
        
        return results
    
    def analyze_connectivity_behaviors(self) -> Dict:
        """分析连接相关用户行为的影响"""
        model_5g = self.integrated.model_5g
        model_gnss = self.integrated.model_gnss
        model_bt = self.integrated.model_bt
        
        results = {'title': '连接管理优化', 'behaviors': []}
        
        # 1. WiFi vs 5G
        p_5g_weak = model_5g.get_power(50e6, 600)['P_total'] * 1000
        p_wifi = 200
        
        results['behaviors'].append({
            'name': '优先使用WiFi(代替弱信号5G)',
            'baseline_mW': p_5g_weak,
            'optimized_mW': p_wifi,
            'savings_mW': p_5g_weak - p_wifi,
            'savings_pct': (p_5g_weak - p_wifi) / p_5g_weak * 100,
            'physics': '5G发射功率随距离呈幂律增长',
            'equation': r'$P_{tx} = \Lambda \cdot d^n \cdot (2^{R/B}-1)$',
            'implementation': '优先连接已知WiFi网络',
            'priority': 'HIGH'
        })
        
        # 2. 关闭GPS
        model_gnss.reset(locked=True)
        p_gnss = model_gnss.get_power(35)['P_total']
        
        results['behaviors'].append({
            'name': '关闭不必要的GPS定位',
            'baseline_mW': p_gnss,
            'optimized_mW': 0,
            'savings_mW': p_gnss,
            'savings_pct': 100,
            'physics': 'GNSS接收机持续消耗45-115mW',
            'equation': r'$P_{GNSS} = P_{LNA} + x_{lock}P_{track}$',
            'implementation': '设置 → 位置 → 仅在使用时允许',
            'priority': 'MEDIUM'
        })
        
        # 3. 飞行模式
        results['behaviors'].append({
            'name': '无信号区域启用飞行模式',
            'baseline_mW': 2500,
            'optimized_mW': 0,
            'savings_mW': 2500,
            'savings_pct': 100,
            'physics': '手机在无信号区域以最大功率搜索',
            'equation': r'$P_{tx,max} \approx 2-3W$',
            'implementation': '进入隧道/地下室前开启飞行模式',
            'priority': 'HIGH'
        })
        
        return results
    
    def analyze_background_behaviors(self) -> Dict:
        """分析后台活动相关用户行为的影响"""
        model_bg = self.integrated.model_bg
        
        results = {'title': '后台活动管理', 'behaviors': []}
        
        p_high = model_bg.get_average_power(10, interface='cell')['P_total']
        p_low = model_bg.get_average_power(1, interface='cell')['P_total']
        
        results['behaviors'].append({
            'name': '限制后台刷新频率',
            'baseline_mW': p_high,
            'optimized_mW': p_low,
            'savings_mW': p_high - p_low,
            'savings_pct': (p_high - p_low) / p_high * 100,
            'physics': '频繁唤醒触发尾时间机制',
            'equation': r'$P_{bg} = P_{leak} + (P_{idle}-P_{leak})(1-e^{-\lambda\tau})$',
            'implementation': '设置 → 应用 → 后台刷新 → 关闭',
            'priority': 'MEDIUM'
        })
        
        return results
    
    def analyze_performance_behaviors(self) -> Dict:
        """分析性能相关用户行为的影响"""
        model_soc = self.integrated.model_soc
        
        results = {'title': '性能与散热管理', 'behaviors': []}
        
        model_soc.reset(25)
        high_perf = model_soc.step(2.8e9, 25, 0.1)
        model_soc.reset(25)
        power_save = model_soc.step(1.5e9, 25, 0.1)
        
        results['behaviors'].append({
            'name': '启用省电模式(2.8→1.5GHz)',
            'baseline_mW': high_perf['P_total'],
            'optimized_mW': power_save['P_total'],
            'savings_mW': high_perf['P_total'] - power_save['P_total'],
            'savings_pct': (high_perf['P_total'] - power_save['P_total']) / high_perf['P_total'] * 100,
            'physics': 'DVFS功耗与频率呈立方关系',
            'equation': r'$P_{dyn} \propto f^3$',
            'implementation': '设置 → 电池 → 省电模式',
            'priority': 'HIGH'
        })
        
        return results
    
    def generate_comprehensive_ranking(self) -> List[Dict]:
        """生成所有优化措施的综合排名"""
        all_behaviors = []
        
        categories = [
            self.analyze_display_behaviors(),
            self.analyze_connectivity_behaviors(),
            self.analyze_background_behaviors(),
            self.analyze_performance_behaviors()
        ]
        
        for category in categories:
            for behavior in category['behaviors']:
                behavior['category'] = category['title']
                all_behaviors.append(behavior)
        
        all_behaviors.sort(key=lambda x: x['savings_mW'], reverse=True)
        return all_behaviors


class AgingImpactAnalysis:
    """电池老化对有效容量和续航的长期影响分析"""
    
    def __init__(self):
        self.battery = BatteryElectroThermalAgingModel()
        
    def analyze_capacity_degradation_timeline(self) -> Dict:
        """分析容量随循环次数的衰减"""
        N_values = np.arange(0, 801, 50)
        Q_values = [self.battery.get_capacity(N, 25) for N in N_values]
        Q_ref = Q_values[0]
        retention = [Q / Q_ref * 100 for Q in Q_values]
        
        N_80 = None
        for i, r in enumerate(retention):
            if r < 80:
                N_80 = N_values[i]
                break
        
        return {
            'N_values': N_values.tolist(),
            'capacity_Ah': Q_values,
            'retention_pct': retention,
            'N_80_threshold': N_80,
            'physics': {
                'model': r'$Q_{max}(N) = a_Q e^{-b_Q N} + c_Q e^{-d_Q N}$',
                'interpretation': [
                    '前100次循环：SEI膜快速形成，容量损失约5%',
                    '100-300次：线性缓慢衰减阶段',
                    f'{N_80}次后：容量低于80%，建议更换'
                ]
            }
        }
    
    def analyze_temperature_acceleration(self) -> Dict:
        """分析温度对老化加速的影响"""
        E_a = 55000
        R = 8.314
        T_ref = 298.15
        T_range = np.array([0, 10, 20, 25, 30, 35, 40, 45, 50])
        
        def arrhenius_factor(T_c):
            return np.exp(E_a / R * (1/T_ref - 1/(T_c + 273.15)))
        
        AF = [arrhenius_factor(T) for T in T_range]
        base_cycles = 800
        equiv_cycles = [int(base_cycles / af) for af in AF]
        
        return {
            'temperature_C': T_range.tolist(),
            'acceleration_factor': [round(af, 2) for af in AF],
            'equivalent_lifespan_cycles': equiv_cycles,
            'physics': {
                'model': r'$AF(T) = \exp[\frac{E_a}{R}(\frac{1}{T_{ref}} - \frac{1}{T})]$',
                'interpretation': [
                    '每升高10°C，老化速度约加快2倍',
                    '45°C充电老化速度是25°C的4倍',
                    '最佳温度：15-35°C'
                ]
            }
        }


if __name__ == "__main__":
    analyzer = UserBehaviorAnalysis()
    aging = AgingImpactAnalysis()
    
    print("=" * 60)
    print("用户行为节能排名")
    print("=" * 60)
    
    ranking = analyzer.generate_comprehensive_ranking()
    
    print(f"\n{'排名':<4} {'措施':<35} {'节能(mW)':<10} {'优先级':<8}")
    print("-" * 60)
    
    for i, b in enumerate(ranking[:10], 1):
        name = b['name'][:33] if len(b['name']) <= 33 else b['name'][:30] + '...'
        print(f"{i:<4} {name:<35} {b['savings_mW']:<10.0f} {b['priority']:<8}")
    
    print("\n" + "=" * 60)
    print("电池老化分析")
    print("=" * 60)
    
    aging_data = aging.analyze_capacity_degradation_timeline()
    print(f"\n80%容量阈值: 约{aging_data['N_80_threshold']}次循环")
    
    temp_data = aging.analyze_temperature_acceleration()
    print("\n温度对老化的加速:")
    for t, af in zip(temp_data['temperature_C'][:5], temp_data['acceleration_factor'][:5]):
        print(f"  {t:>3}°C: {af:>5.2f}× 加速")
