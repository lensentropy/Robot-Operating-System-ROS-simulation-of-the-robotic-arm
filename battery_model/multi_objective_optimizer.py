"""
多目标优化模块
Multi-Objective Optimization for Battery Life Prediction and Optimization

目标:
1. 最大化剩余使用时间
2. 最小化温度峰值 (保护电池健康)
3. 最大化用户体验 (性能与续航平衡)

方法:
- NSGA-II (Non-dominated Sorting Genetic Algorithm II)
- 加权聚合法
- Pareto前沿分析

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional, Callable
from scipy.optimize import minimize, differential_evolution
import warnings


@dataclass
class OptimizationConfig:
    """优化配置"""
    # 决策变量边界
    cpu_freq_bounds: Tuple[float, float] = (0.3, 3.0)     # GHz
    brightness_bounds: Tuple[float, float] = (0, 1000)     # nits
    refresh_rate_bounds: Tuple[float, float] = (1, 120)    # Hz
    
    # 优化目标权重
    weight_battery_life: float = 0.5
    weight_performance: float = 0.3
    weight_thermal: float = 0.2
    
    # 约束条件
    max_temperature: float = 318.15    # 最高温度 45°C (K)
    min_soc_threshold: float = 0.05    # 最低SOC阈值
    
    # 算法参数
    population_size: int = 50
    n_generations: int = 100
    mutation_rate: float = 0.1
    crossover_rate: float = 0.8


class ObjectiveFunctions:
    """
    多目标函数定义
    """
    
    @staticmethod
    def battery_life_objective(power_consumption: float, 
                                soc_current: float,
                                capacity_mah: float = 4000) -> float:
        """
        电池寿命目标 (最大化)
        
        返回预计剩余时间 (小时)
        """
        if power_consumption <= 0:
            return float('inf')
        
        # 可用能量 (Wh)
        V_avg = 3.7
        usable_energy = soc_current * capacity_mah / 1000 * V_avg
        
        # 剩余时间
        remaining_hours = usable_energy / power_consumption
        
        return remaining_hours
    
    @staticmethod
    def performance_objective(cpu_freq: float, 
                               brightness: float,
                               refresh_rate: float) -> float:
        """
        性能/用户体验目标 (最大化)
        
        归一化到 [0, 1]
        """
        # CPU性能分数 (频率归一化)
        cpu_score = (cpu_freq - 0.3) / (3.0 - 0.3)
        
        # 显示体验分数
        brightness_score = min(brightness / 800, 1.0)  # 800 nits为满分
        refresh_score = (refresh_rate - 30) / (120 - 30)  # 30Hz为基准
        
        display_score = 0.6 * brightness_score + 0.4 * refresh_score
        
        # 综合性能分数
        performance = 0.5 * cpu_score + 0.5 * display_score
        
        return np.clip(performance, 0, 1)
    
    @staticmethod
    def thermal_objective(temperature: float, max_temp: float = 318.15) -> float:
        """
        热管理目标 (最小化温度)
        
        返回温度惩罚分数 [0, 1], 0为最优
        """
        T_ambient = 298.15
        T_range = max_temp - T_ambient
        
        if temperature <= T_ambient:
            return 0.0
        elif temperature >= max_temp:
            return 1.0
        else:
            return (temperature - T_ambient) / T_range
    
    @staticmethod
    def power_consumption_model(cpu_freq: float,
                                 cpu_load: float,
                                 brightness: float,
                                 refresh_rate: float,
                                 temperature: float = 300) -> float:
        """
        功耗模型 (简化版)
        
        Returns:
        --------
        float : 总功耗 (W)
        """
        # CPU功耗 (立方缩放)
        f_normalized = np.clip((cpu_freq - 0.3) / (3.0 - 0.3), 0, 1)
        V_dd = 0.6 + 0.5 * (f_normalized**0.8 if f_normalized > 0 else 0)
        P_cpu_dyn = 0.1 * cpu_load * 2.5e-9 * V_dd**2 * cpu_freq * 1e9
        
        # 漏电功耗
        P_leak = 0.3 * (temperature / 300)**2 * np.exp(0.05 * (V_dd - 0.6))
        
        P_cpu = P_cpu_dyn + P_leak
        
        # 显示功耗
        if brightness > 0:
            P_display = 0.1 + 0.002 * brightness * (1 + 0.001 * refresh_rate)
        else:
            P_display = 0
        
        # 基础功耗 (通信、后台等)
        P_base = 0.5
        
        return P_cpu + P_display + P_base


class WeightedSumOptimizer:
    """
    加权求和法多目标优化
    
    将多目标转化为单目标: f = w1*f1 + w2*f2 + w3*f3
    """
    
    def __init__(self, config: OptimizationConfig = None):
        self.config = config or OptimizationConfig()
        self.objectives = ObjectiveFunctions()
        
        # 记录
        self.optimization_history = []
    
    def combined_objective(self, x: np.ndarray, 
                           soc_current: float,
                           temperature: float) -> float:
        """
        组合目标函数 (最小化)
        
        Parameters:
        -----------
        x : np.ndarray
            决策变量 [cpu_freq, brightness, refresh_rate]
        soc_current : float
            当前SOC
        temperature : float
            当前温度 (K)
        
        Returns:
        --------
        float : 组合目标值 (越小越好)
        """
        cpu_freq, brightness, refresh_rate = x
        
        # 计算功耗
        power = self.objectives.power_consumption_model(
            cpu_freq, 0.5, brightness, refresh_rate, temperature
        )
        
        # 目标1: 电池寿命 (最大化 -> 负号)
        battery_life = self.objectives.battery_life_objective(power, soc_current)
        f1 = -battery_life / 24.0  # 归一化到24小时
        
        # 目标2: 性能 (最大化 -> 负号)
        performance = self.objectives.performance_objective(
            cpu_freq, brightness, refresh_rate
        )
        f2 = -performance
        
        # 目标3: 热管理 (最小化)
        # 估计温度增量
        P_joule = power * 0.1  # 简化
        T_est = temperature + P_joule * 5  # 简化热模型
        thermal = self.objectives.thermal_objective(T_est, self.config.max_temperature)
        f3 = thermal
        
        # 加权求和
        combined = (self.config.weight_battery_life * f1 + 
                   self.config.weight_performance * f2 + 
                   self.config.weight_thermal * f3)
        
        return combined
    
    def optimize(self, soc_current: float, 
                 temperature: float,
                 user_priority: str = 'balanced') -> Dict:
        """
        执行优化
        
        Parameters:
        -----------
        soc_current : float
            当前SOC (0-1)
        temperature : float
            当前温度 (K)
        user_priority : str
            用户优先级: 'battery', 'performance', 'balanced'
        
        Returns:
        --------
        dict : 最优解
        """
        # 根据用户优先级调整权重
        if user_priority == 'battery':
            self.config.weight_battery_life = 0.7
            self.config.weight_performance = 0.15
            self.config.weight_thermal = 0.15
        elif user_priority == 'performance':
            self.config.weight_battery_life = 0.2
            self.config.weight_performance = 0.6
            self.config.weight_thermal = 0.2
        else:  # balanced
            self.config.weight_battery_life = 0.5
            self.config.weight_performance = 0.3
            self.config.weight_thermal = 0.2
        
        # 边界
        bounds = [
            self.config.cpu_freq_bounds,
            self.config.brightness_bounds,
            self.config.refresh_rate_bounds
        ]
        
        # 差分进化优化
        result = differential_evolution(
            lambda x: self.combined_objective(x, soc_current, temperature),
            bounds,
            seed=42,
            maxiter=100,
            tol=1e-6,
            polish=True
        )
        
        optimal_x = result.x
        
        # 计算各目标值
        power = self.objectives.power_consumption_model(
            optimal_x[0], 0.5, optimal_x[1], optimal_x[2], temperature
        )
        
        battery_life = self.objectives.battery_life_objective(power, soc_current)
        performance = self.objectives.performance_objective(
            optimal_x[0], optimal_x[1], optimal_x[2]
        )
        thermal = self.objectives.thermal_objective(temperature, self.config.max_temperature)
        
        return {
            'optimal_cpu_freq': optimal_x[0],
            'optimal_brightness': optimal_x[1],
            'optimal_refresh_rate': optimal_x[2],
            'predicted_power': power,
            'predicted_battery_life_hours': battery_life,
            'performance_score': performance,
            'thermal_score': 1 - thermal,
            'optimization_success': result.success
        }


class NSGA2Optimizer:
    """
    NSGA-II 多目标优化算法
    
    用于寻找Pareto最优解集
    """
    
    def __init__(self, config: OptimizationConfig = None):
        self.config = config or OptimizationConfig()
        self.objectives = ObjectiveFunctions()
        
        # 决策变量数
        self.n_vars = 3  # cpu_freq, brightness, refresh_rate
        
        # Pareto前沿
        self.pareto_front = []
    
    def initialize_population(self) -> np.ndarray:
        """初始化种群"""
        pop_size = self.config.population_size
        
        population = np.zeros((pop_size, self.n_vars))
        
        # 随机初始化
        population[:, 0] = np.random.uniform(
            self.config.cpu_freq_bounds[0],
            self.config.cpu_freq_bounds[1],
            pop_size
        )
        population[:, 1] = np.random.uniform(
            self.config.brightness_bounds[0],
            self.config.brightness_bounds[1],
            pop_size
        )
        population[:, 2] = np.random.uniform(
            self.config.refresh_rate_bounds[0],
            self.config.refresh_rate_bounds[1],
            pop_size
        )
        
        return population
    
    def evaluate_objectives(self, x: np.ndarray, 
                            soc_current: float,
                            temperature: float) -> np.ndarray:
        """
        评估所有目标函数
        
        Returns:
        --------
        np.ndarray : [f1, f2, f3] 目标值 (全部最小化)
        """
        cpu_freq, brightness, refresh_rate = x
        
        power = self.objectives.power_consumption_model(
            cpu_freq, 0.5, brightness, refresh_rate, temperature
        )
        
        # f1: 负电池寿命 (最小化 -> 最大化寿命)
        battery_life = self.objectives.battery_life_objective(power, soc_current)
        f1 = -battery_life
        
        # f2: 负性能 (最小化 -> 最大化性能)
        performance = self.objectives.performance_objective(
            cpu_freq, brightness, refresh_rate
        )
        f2 = -performance
        
        # f3: 温度惩罚 (最小化)
        T_est = temperature + power * 0.5
        f3 = self.objectives.thermal_objective(T_est, self.config.max_temperature)
        
        return np.array([f1, f2, f3])
    
    def dominates(self, obj1: np.ndarray, obj2: np.ndarray) -> bool:
        """判断obj1是否支配obj2"""
        return np.all(obj1 <= obj2) and np.any(obj1 < obj2)
    
    def fast_non_dominated_sort(self, population: np.ndarray,
                                 objectives: np.ndarray) -> List[List[int]]:
        """
        快速非支配排序
        
        Returns:
        --------
        List[List[int]] : 各层的个体索引
        """
        n = len(population)
        domination_count = np.zeros(n, dtype=int)
        dominated_set = [[] for _ in range(n)]
        fronts = [[]]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    if self.dominates(objectives[i], objectives[j]):
                        dominated_set[i].append(j)
                    elif self.dominates(objectives[j], objectives[i]):
                        domination_count[i] += 1
            
            if domination_count[i] == 0:
                fronts[0].append(i)
        
        current_front = 0
        while fronts[current_front]:
            next_front = []
            for i in fronts[current_front]:
                for j in dominated_set[i]:
                    domination_count[j] -= 1
                    if domination_count[j] == 0:
                        next_front.append(j)
            current_front += 1
            fronts.append(next_front)
        
        return fronts[:-1]  # 移除最后的空层
    
    def crowding_distance(self, objectives: np.ndarray, 
                          front: List[int]) -> np.ndarray:
        """计算拥挤距离"""
        n = len(front)
        if n <= 2:
            return np.full(n, float('inf'))
        
        distances = np.zeros(n)
        n_obj = objectives.shape[1]
        
        for m in range(n_obj):
            sorted_indices = np.argsort(objectives[front, m])
            distances[sorted_indices[0]] = float('inf')
            distances[sorted_indices[-1]] = float('inf')
            
            obj_range = (objectives[front[sorted_indices[-1]], m] - 
                        objectives[front[sorted_indices[0]], m])
            
            if obj_range > 0:
                for i in range(1, n - 1):
                    distances[sorted_indices[i]] += (
                        objectives[front[sorted_indices[i + 1]], m] -
                        objectives[front[sorted_indices[i - 1]], m]
                    ) / obj_range
        
        return distances
    
    def selection(self, population: np.ndarray, 
                  objectives: np.ndarray,
                  fronts: List[List[int]],
                  n_select: int) -> np.ndarray:
        """选择操作"""
        selected = []
        
        for front in fronts:
            if len(selected) + len(front) <= n_select:
                selected.extend(front)
            else:
                # 基于拥挤距离选择
                distances = self.crowding_distance(objectives, front)
                sorted_indices = np.argsort(-distances)  # 降序
                remaining = n_select - len(selected)
                selected.extend([front[i] for i in sorted_indices[:remaining]])
                break
        
        return population[selected]
    
    def crossover(self, parent1: np.ndarray, parent2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """模拟二进制交叉 (SBX)"""
        if np.random.random() > self.config.crossover_rate:
            return parent1.copy(), parent2.copy()
        
        eta = 20  # 分布指数
        child1 = np.zeros_like(parent1)
        child2 = np.zeros_like(parent2)
        
        for i in range(len(parent1)):
            if np.random.random() < 0.5:
                if abs(parent1[i] - parent2[i]) > 1e-10:
                    if parent1[i] < parent2[i]:
                        y1, y2 = parent1[i], parent2[i]
                    else:
                        y1, y2 = parent2[i], parent1[i]
                    
                    beta = 1.0 + 2.0 * (y1 - 0) / (y2 - y1)
                    alpha = 2.0 - beta**-(eta + 1)
                    
                    rand = np.random.random()
                    if rand <= 1.0 / alpha:
                        betaq = (rand * alpha)**(1.0 / (eta + 1))
                    else:
                        betaq = (1.0 / (2.0 - rand * alpha))**(1.0 / (eta + 1))
                    
                    child1[i] = 0.5 * ((y1 + y2) - betaq * (y2 - y1))
                    child2[i] = 0.5 * ((y1 + y2) + betaq * (y2 - y1))
                else:
                    child1[i] = parent1[i]
                    child2[i] = parent2[i]
            else:
                child1[i] = parent1[i]
                child2[i] = parent2[i]
        
        return child1, child2
    
    def mutation(self, individual: np.ndarray) -> np.ndarray:
        """多项式变异"""
        mutated = individual.copy()
        eta_m = 20  # 变异分布指数
        
        bounds = [
            self.config.cpu_freq_bounds,
            self.config.brightness_bounds,
            self.config.refresh_rate_bounds
        ]
        
        for i in range(len(individual)):
            if np.random.random() < self.config.mutation_rate:
                y = individual[i]
                yl, yu = bounds[i]
                
                delta1 = (y - yl) / (yu - yl)
                delta2 = (yu - y) / (yu - yl)
                
                rand = np.random.random()
                
                if rand <= 0.5:
                    xy = 1.0 - delta1
                    val = 2.0 * rand + (1.0 - 2.0 * rand) * (xy**(eta_m + 1))
                    deltaq = val**(1.0 / (eta_m + 1)) - 1.0
                else:
                    xy = 1.0 - delta2
                    val = 2.0 * (1.0 - rand) + 2.0 * (rand - 0.5) * (xy**(eta_m + 1))
                    deltaq = 1.0 - val**(1.0 / (eta_m + 1))
                
                mutated[i] = y + deltaq * (yu - yl)
                mutated[i] = np.clip(mutated[i], yl, yu)
        
        return mutated
    
    def optimize(self, soc_current: float, 
                 temperature: float) -> Dict:
        """
        执行NSGA-II优化
        
        Returns:
        --------
        dict : Pareto最优解集和推荐解
        """
        # 初始化种群
        population = self.initialize_population()
        pop_size = self.config.population_size
        
        # 主循环
        for generation in range(self.config.n_generations):
            # 评估目标函数
            objectives = np.array([
                self.evaluate_objectives(ind, soc_current, temperature)
                for ind in population
            ])
            
            # 非支配排序
            fronts = self.fast_non_dominated_sort(population, objectives)
            
            # 创建子代
            offspring = []
            while len(offspring) < pop_size:
                # 锦标赛选择
                i1, i2 = np.random.choice(pop_size, 2, replace=False)
                i3, i4 = np.random.choice(pop_size, 2, replace=False)
                
                parent1 = population[i1] if objectives[i1].sum() < objectives[i2].sum() else population[i2]
                parent2 = population[i3] if objectives[i3].sum() < objectives[i4].sum() else population[i4]
                
                # 交叉
                child1, child2 = self.crossover(parent1, parent2)
                
                # 变异
                child1 = self.mutation(child1)
                child2 = self.mutation(child2)
                
                offspring.extend([child1, child2])
            
            offspring = np.array(offspring[:pop_size])
            
            # 合并父代和子代
            combined = np.vstack([population, offspring])
            combined_obj = np.array([
                self.evaluate_objectives(ind, soc_current, temperature)
                for ind in combined
            ])
            
            # 环境选择
            combined_fronts = self.fast_non_dominated_sort(combined, combined_obj)
            population = self.selection(combined, combined_obj, combined_fronts, pop_size)
        
        # 获取最终Pareto前沿
        final_objectives = np.array([
            self.evaluate_objectives(ind, soc_current, temperature)
            for ind in population
        ])
        final_fronts = self.fast_non_dominated_sort(population, final_objectives)
        
        pareto_indices = final_fronts[0] if final_fronts else list(range(len(population)))
        pareto_solutions = population[pareto_indices]
        pareto_objectives = final_objectives[pareto_indices]
        
        # 选择推荐解 (最平衡的解)
        # 使用TOPSIS方法
        normalized_obj = pareto_objectives - pareto_objectives.min(axis=0)
        max_range = pareto_objectives.max(axis=0) - pareto_objectives.min(axis=0)
        max_range[max_range == 0] = 1
        normalized_obj /= max_range
        
        ideal = normalized_obj.min(axis=0)
        nadir = normalized_obj.max(axis=0)
        
        dist_ideal = np.sqrt(((normalized_obj - ideal)**2).sum(axis=1))
        dist_nadir = np.sqrt(((normalized_obj - nadir)**2).sum(axis=1))
        
        closeness = dist_nadir / (dist_ideal + dist_nadir + 1e-10)
        best_idx = np.argmax(closeness)
        
        recommended = pareto_solutions[best_idx]
        
        # 计算推荐解的详细信息
        power = self.objectives.power_consumption_model(
            recommended[0], 0.5, recommended[1], recommended[2], temperature
        )
        
        return {
            'pareto_solutions': pareto_solutions.tolist(),
            'pareto_objectives': pareto_objectives.tolist(),
            'recommended_solution': {
                'cpu_freq': recommended[0],
                'brightness': recommended[1],
                'refresh_rate': recommended[2]
            },
            'predicted_power': power,
            'predicted_battery_life_hours': self.objectives.battery_life_objective(power, soc_current),
            'n_pareto_solutions': len(pareto_solutions)
        }


class BatteryLifePredictor:
    """
    电池寿命预测器
    
    结合多目标优化和用户行为模式
    """
    
    def __init__(self):
        self.weighted_optimizer = WeightedSumOptimizer()
        self.nsga2_optimizer = NSGA2Optimizer()
    
    def predict_remaining_time(self, soc_current: float,
                                temperature: float,
                                user_behavior: str = 'balanced',
                                method: str = 'weighted') -> Dict:
        """
        预测剩余使用时间
        
        Parameters:
        -----------
        soc_current : float
            当前SOC
        temperature : float
            当前温度 (K)
        user_behavior : str
            用户行为模式: 'light', 'normal', 'heavy'
        method : str
            优化方法: 'weighted' or 'nsga2'
        
        Returns:
        --------
        dict : 预测结果
        """
        if method == 'weighted':
            result = self.weighted_optimizer.optimize(
                soc_current, temperature, user_priority=user_behavior
            )
        else:
            result = self.nsga2_optimizer.optimize(soc_current, temperature)
        
        # 添加不确定性估计
        base_life = result.get('predicted_battery_life_hours', 0)
        
        # 基于用户行为的不确定性
        if user_behavior == 'light':
            uncertainty_factor = 0.1
        elif user_behavior == 'heavy':
            uncertainty_factor = 0.3
        else:
            uncertainty_factor = 0.2
        
        result['remaining_time_hours'] = base_life
        result['uncertainty_hours'] = base_life * uncertainty_factor
        result['confidence_interval'] = (
            base_life * (1 - uncertainty_factor),
            base_life * (1 + uncertainty_factor)
        )
        
        return result


if __name__ == "__main__":
    # 测试多目标优化
    print("测试加权求和优化器...")
    weighted_opt = WeightedSumOptimizer()
    
    result = weighted_opt.optimize(
        soc_current=0.8,
        temperature=300,
        user_priority='balanced'
    )
    
    print(f"最优CPU频率: {result['optimal_cpu_freq']:.2f} GHz")
    print(f"最优亮度: {result['optimal_brightness']:.0f} nits")
    print(f"最优刷新率: {result['optimal_refresh_rate']:.0f} Hz")
    print(f"预测功耗: {result['predicted_power']:.2f} W")
    print(f"预测电池寿命: {result['predicted_battery_life_hours']:.1f} 小时")
    
    print("\n测试NSGA-II优化器...")
    nsga2_opt = NSGA2Optimizer()
    nsga2_opt.config.n_generations = 50  # 减少迭代次数加快测试
    nsga2_opt.config.population_size = 30
    
    result = nsga2_opt.optimize(soc_current=0.8, temperature=300)
    
    print(f"Pareto解数量: {result['n_pareto_solutions']}")
    print(f"推荐解:")
    print(f"  CPU频率: {result['recommended_solution']['cpu_freq']:.2f} GHz")
    print(f"  亮度: {result['recommended_solution']['brightness']:.0f} nits")
    print(f"  刷新率: {result['recommended_solution']['refresh_rate']:.0f} Hz")
    print(f"预测电池寿命: {result['predicted_battery_life_hours']:.1f} 小时")
