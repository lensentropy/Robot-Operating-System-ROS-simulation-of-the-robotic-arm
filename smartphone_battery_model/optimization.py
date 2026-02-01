"""
多目标优化用于电池模型参数估计
Multi-Objective Optimization for Battery Model Parameter Estimation

本模块实现：
1. NSGA-II (非支配排序遗传算法) 用于多目标优化
2. 粒子群优化 (PSO) 用于参数搜索
3. 差分进化算法用于全局优化
4. 敏感性分析
5. 模型验证与不确定性量化

优化目标：
────────────────────────────────────────────────────────────────────────
1. 最小化SOC预测误差
   J1 = ∫(SOC_model(t) - SOC_measured(t))² dt

2. 最小化电压预测误差
   J2 = ∫(V_model(t) - V_measured(t))² dt

3. 最小化剩余时间预测误差
   J3 = |t_remaining_model - t_remaining_actual|² / t_remaining_actual²

约束条件：
- 物理参数范围约束
- 模型稳定性约束
────────────────────────────────────────────────────────────────────────

参考文献:
[1] Deb, K., et al. "A fast and elitist multiobjective genetic algorithm: NSGA-II"
    IEEE Transactions on Evolutionary Computation, 2002
[2] Kennedy, J., Eberhart, R. "Particle swarm optimization"
    IEEE International Conference on Neural Networks, 1995
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution, dual_annealing
from scipy.integrate import solve_ivp
from typing import List, Tuple, Dict, Callable, Optional
from dataclasses import dataclass
import copy
import warnings

from battery_model import (
    SmartphoneBatteryModel, BatteryParameters, 
    PowerConsumptionCoefficients, UsageProfile
)


@dataclass
class OptimizationConfig:
    """优化配置"""
    # 种群大小
    population_size: int = 50
    
    # 最大迭代次数
    max_generations: int = 100
    
    # 交叉概率
    crossover_prob: float = 0.9
    
    # 变异概率
    mutation_prob: float = 0.1
    
    # 精英保留比例
    elite_ratio: float = 0.1
    
    # 收敛阈值
    convergence_tol: float = 1e-6
    
    # 随机种子
    random_seed: Optional[int] = 42


@dataclass
class ParameterBounds:
    """参数边界定义"""
    # 电池参数边界
    nominal_capacity: Tuple[float, float] = (2000, 6000)  # mAh
    internal_resistance: Tuple[float, float] = (0.03, 0.2)  # Ω
    
    # 电压模型参数边界
    V0: Tuple[float, float] = (3.9, 4.3)  # V
    K: Tuple[float, float] = (0.01, 0.3)  # 极化常数
    A: Tuple[float, float] = (0.05, 0.5)  # 指数区振幅
    B: Tuple[float, float] = (1.0, 10.0)  # 指数区常数
    
    # 功耗系数边界
    screen_brightness_coeff: Tuple[float, float] = (200, 600)  # mW
    cpu_active_power: Tuple[float, float] = (800, 2500)  # mW
    cellular_active_power: Tuple[float, float] = (300, 800)  # mW


class Individual:
    """
    遗传算法个体
    """
    def __init__(self, genes: np.ndarray = None, bounds: List[Tuple[float, float]] = None):
        """
        初始化个体
        
        参数:
            genes: 基因向量（参数值）
            bounds: 参数边界列表
        """
        self.bounds = bounds
        
        if genes is not None:
            self.genes = genes
        elif bounds is not None:
            # 随机初始化
            self.genes = np.array([
                np.random.uniform(low, high) for low, high in bounds
            ])
        else:
            self.genes = None
        
        self.objectives: np.ndarray = None  # 目标函数值
        self.rank: int = 0  # Pareto等级
        self.crowding_distance: float = 0.0  # 拥挤度距离
        self.fitness: float = 0.0  # 适应度值（单目标优化用）
    
    def __lt__(self, other):
        """用于排序"""
        if self.rank != other.rank:
            return self.rank < other.rank
        return self.crowding_distance > other.crowding_distance


class NSGAII:
    """
    NSGA-II 多目标优化算法
    
    非支配排序遗传算法II，用于多目标优化问题
    """
    
    def __init__(self, 
                 objective_functions: List[Callable],
                 bounds: List[Tuple[float, float]],
                 config: OptimizationConfig = None):
        """
        初始化NSGA-II
        
        参数:
            objective_functions: 目标函数列表
            bounds: 参数边界
            config: 优化配置
        """
        self.objectives = objective_functions
        self.bounds = bounds
        self.config = config or OptimizationConfig()
        self.n_objectives = len(objective_functions)
        self.n_params = len(bounds)
        
        # 设置随机种子
        if self.config.random_seed is not None:
            np.random.seed(self.config.random_seed)
        
        # 种群
        self.population: List[Individual] = []
        self.pareto_front: List[Individual] = []
        
        # 历史记录
        self.history = {
            'generations': [],
            'best_objectives': [],
            'average_objectives': [],
            'pareto_front_size': []
        }
    
    def initialize_population(self):
        """初始化种群"""
        self.population = [
            Individual(bounds=self.bounds) 
            for _ in range(self.config.population_size)
        ]
    
    def evaluate_population(self):
        """评估种群中所有个体的目标函数"""
        for ind in self.population:
            if ind.objectives is None:
                ind.objectives = np.array([
                    obj(ind.genes) for obj in self.objectives
                ])
    
    def dominates(self, ind1: Individual, ind2: Individual) -> bool:
        """
        判断ind1是否支配ind2
        
        ind1支配ind2当且仅当:
        - ind1在所有目标上不差于ind2
        - ind1在至少一个目标上优于ind2
        """
        better_in_any = False
        
        for i in range(self.n_objectives):
            if ind1.objectives[i] > ind2.objectives[i]:
                return False
            elif ind1.objectives[i] < ind2.objectives[i]:
                better_in_any = True
        
        return better_in_any
    
    def fast_non_dominated_sort(self):
        """
        快速非支配排序
        
        将种群分成多个Pareto前沿层
        """
        fronts = [[]]
        
        for p in self.population:
            p.dominated_count = 0
            p.dominated_solutions = []
            
            for q in self.population:
                if self.dominates(p, q):
                    p.dominated_solutions.append(q)
                elif self.dominates(q, p):
                    p.dominated_count += 1
            
            if p.dominated_count == 0:
                p.rank = 0
                fronts[0].append(p)
        
        i = 0
        while len(fronts[i]) > 0:
            next_front = []
            for p in fronts[i]:
                for q in p.dominated_solutions:
                    q.dominated_count -= 1
                    if q.dominated_count == 0:
                        q.rank = i + 1
                        next_front.append(q)
            i += 1
            fronts.append(next_front)
        
        return fronts[:-1]  # 移除最后一个空前沿
    
    def calculate_crowding_distance(self, front: List[Individual]):
        """
        计算拥挤度距离
        
        拥挤度用于在同一Pareto层中选择个体
        """
        if len(front) <= 2:
            for ind in front:
                ind.crowding_distance = float('inf')
            return
        
        for ind in front:
            ind.crowding_distance = 0
        
        for m in range(self.n_objectives):
            # 按目标m排序
            front.sort(key=lambda x: x.objectives[m])
            
            # 边界点设为无穷大
            front[0].crowding_distance = float('inf')
            front[-1].crowding_distance = float('inf')
            
            # 计算范围
            obj_range = front[-1].objectives[m] - front[0].objectives[m]
            if obj_range == 0:
                continue
            
            # 计算中间点的拥挤度
            for i in range(1, len(front) - 1):
                front[i].crowding_distance += (
                    front[i + 1].objectives[m] - front[i - 1].objectives[m]
                ) / obj_range
    
    def selection(self) -> Tuple[Individual, Individual]:
        """
        二元锦标赛选择
        """
        def tournament(pop):
            i1, i2 = np.random.choice(len(pop), 2, replace=False)
            if pop[i1] < pop[i2]:
                return pop[i1]
            return pop[i2]
        
        parent1 = tournament(self.population)
        parent2 = tournament(self.population)
        
        return parent1, parent2
    
    def crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        """
        模拟二进制交叉 (SBX)
        """
        if np.random.random() > self.config.crossover_prob:
            return copy.deepcopy(parent1), copy.deepcopy(parent2)
        
        eta = 20  # 分布指数
        
        child1_genes = np.zeros(self.n_params)
        child2_genes = np.zeros(self.n_params)
        
        for i in range(self.n_params):
            if np.random.random() < 0.5:
                if abs(parent1.genes[i] - parent2.genes[i]) > 1e-10:
                    if parent1.genes[i] < parent2.genes[i]:
                        y1, y2 = parent1.genes[i], parent2.genes[i]
                    else:
                        y1, y2 = parent2.genes[i], parent1.genes[i]
                    
                    low, high = self.bounds[i]
                    
                    beta = 1.0 + (2.0 * (y1 - low) / (y2 - y1))
                    alpha = 2.0 - beta ** (-(eta + 1))
                    
                    u = np.random.random()
                    if u <= 1.0 / alpha:
                        betaq = (u * alpha) ** (1.0 / (eta + 1))
                    else:
                        betaq = (1.0 / (2.0 - u * alpha)) ** (1.0 / (eta + 1))
                    
                    c1 = 0.5 * ((y1 + y2) - betaq * (y2 - y1))
                    c2 = 0.5 * ((y1 + y2) + betaq * (y2 - y1))
                    
                    child1_genes[i] = np.clip(c1, low, high)
                    child2_genes[i] = np.clip(c2, low, high)
                else:
                    child1_genes[i] = parent1.genes[i]
                    child2_genes[i] = parent2.genes[i]
            else:
                child1_genes[i] = parent1.genes[i]
                child2_genes[i] = parent2.genes[i]
        
        return Individual(child1_genes, self.bounds), Individual(child2_genes, self.bounds)
    
    def mutation(self, individual: Individual) -> Individual:
        """
        多项式变异
        """
        eta = 20  # 分布指数
        
        for i in range(self.n_params):
            if np.random.random() < self.config.mutation_prob:
                low, high = self.bounds[i]
                y = individual.genes[i]
                
                delta1 = (y - low) / (high - low)
                delta2 = (high - y) / (high - low)
                
                u = np.random.random()
                
                if u < 0.5:
                    xy = 1.0 - delta1
                    val = 2.0 * u + (1.0 - 2.0 * u) * (xy ** (eta + 1))
                    deltaq = val ** (1.0 / (eta + 1)) - 1.0
                else:
                    xy = 1.0 - delta2
                    val = 2.0 * (1.0 - u) + 2.0 * (u - 0.5) * (xy ** (eta + 1))
                    deltaq = 1.0 - val ** (1.0 / (eta + 1))
                
                individual.genes[i] = np.clip(y + deltaq * (high - low), low, high)
        
        return individual
    
    def evolve(self) -> List[Individual]:
        """
        执行NSGA-II优化
        
        返回:
            Pareto最优解集
        """
        # 初始化
        self.initialize_population()
        self.evaluate_population()
        
        for gen in range(self.config.max_generations):
            # 创建子代
            offspring = []
            
            while len(offspring) < self.config.population_size:
                parent1, parent2 = self.selection()
                child1, child2 = self.crossover(parent1, parent2)
                child1 = self.mutation(child1)
                child2 = self.mutation(child2)
                offspring.extend([child1, child2])
            
            # 评估子代
            for ind in offspring:
                ind.objectives = np.array([obj(ind.genes) for obj in self.objectives])
            
            # 合并种群
            combined = self.population + offspring[:self.config.population_size]
            
            # 非支配排序
            fronts = self.fast_non_dominated_sort()
            
            # 选择下一代
            self.population = []
            front_idx = 0
            
            while len(self.population) + len(fronts[front_idx]) <= self.config.population_size:
                self.calculate_crowding_distance(fronts[front_idx])
                self.population.extend(fronts[front_idx])
                front_idx += 1
                if front_idx >= len(fronts):
                    break
            
            # 如果还需要补充
            if len(self.population) < self.config.population_size and front_idx < len(fronts):
                self.calculate_crowding_distance(fronts[front_idx])
                fronts[front_idx].sort(key=lambda x: x.crowding_distance, reverse=True)
                remaining = self.config.population_size - len(self.population)
                self.population.extend(fronts[front_idx][:remaining])
            
            # 记录历史
            best_obj = np.min([ind.objectives for ind in self.population], axis=0)
            avg_obj = np.mean([ind.objectives for ind in self.population], axis=0)
            
            self.history['generations'].append(gen)
            self.history['best_objectives'].append(best_obj)
            self.history['average_objectives'].append(avg_obj)
            self.history['pareto_front_size'].append(len(fronts[0]) if fronts else 0)
            
            # 打印进度
            if gen % 10 == 0:
                print(f"Generation {gen}: Best objectives = {best_obj}")
        
        # 获取Pareto前沿
        fronts = self.fast_non_dominated_sort()
        self.pareto_front = fronts[0] if fronts else []
        
        return self.pareto_front


class ParticleSwarmOptimizer:
    """
    粒子群优化 (PSO)
    
    用于单目标或加权多目标优化
    """
    
    def __init__(self,
                 objective_function: Callable,
                 bounds: List[Tuple[float, float]],
                 n_particles: int = 30,
                 max_iterations: int = 100,
                 w: float = 0.7,  # 惯性权重
                 c1: float = 1.5,  # 认知系数
                 c2: float = 1.5,  # 社会系数
                 random_seed: int = 42):
        """
        初始化PSO
        
        参数:
            objective_function: 目标函数（最小化）
            bounds: 参数边界
            n_particles: 粒子数量
            max_iterations: 最大迭代次数
            w: 惯性权重
            c1: 认知系数（个体最优）
            c2: 社会系数（全局最优）
        """
        self.objective = objective_function
        self.bounds = np.array(bounds)
        self.n_particles = n_particles
        self.max_iterations = max_iterations
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.n_params = len(bounds)
        
        np.random.seed(random_seed)
        
        # 初始化粒子
        self.positions = np.random.uniform(
            self.bounds[:, 0], self.bounds[:, 1],
            (n_particles, self.n_params)
        )
        
        # 初始化速度
        velocity_range = self.bounds[:, 1] - self.bounds[:, 0]
        self.velocities = np.random.uniform(
            -velocity_range, velocity_range,
            (n_particles, self.n_params)
        ) * 0.1
        
        # 个体最优
        self.personal_best_positions = self.positions.copy()
        self.personal_best_fitness = np.full(n_particles, np.inf)
        
        # 全局最优
        self.global_best_position = None
        self.global_best_fitness = np.inf
        
        # 历史
        self.history = {
            'iterations': [],
            'best_fitness': [],
            'average_fitness': []
        }
    
    def optimize(self) -> Tuple[np.ndarray, float]:
        """
        执行PSO优化
        
        返回:
            最优参数, 最优目标值
        """
        for iteration in range(self.max_iterations):
            # 评估所有粒子
            fitness_values = np.array([
                self.objective(pos) for pos in self.positions
            ])
            
            # 更新个体最优
            improved = fitness_values < self.personal_best_fitness
            self.personal_best_positions[improved] = self.positions[improved]
            self.personal_best_fitness[improved] = fitness_values[improved]
            
            # 更新全局最优
            best_idx = np.argmin(fitness_values)
            if fitness_values[best_idx] < self.global_best_fitness:
                self.global_best_fitness = fitness_values[best_idx]
                self.global_best_position = self.positions[best_idx].copy()
            
            # 更新速度和位置
            r1, r2 = np.random.random((2, self.n_particles, self.n_params))
            
            cognitive = self.c1 * r1 * (self.personal_best_positions - self.positions)
            social = self.c2 * r2 * (self.global_best_position - self.positions)
            
            self.velocities = self.w * self.velocities + cognitive + social
            
            # 速度限制
            velocity_limit = (self.bounds[:, 1] - self.bounds[:, 0]) * 0.2
            self.velocities = np.clip(self.velocities, -velocity_limit, velocity_limit)
            
            # 位置更新
            self.positions = self.positions + self.velocities
            
            # 边界处理
            self.positions = np.clip(self.positions, self.bounds[:, 0], self.bounds[:, 1])
            
            # 记录历史
            self.history['iterations'].append(iteration)
            self.history['best_fitness'].append(self.global_best_fitness)
            self.history['average_fitness'].append(np.mean(fitness_values))
            
            # 自适应惯性权重
            self.w = max(0.4, self.w * 0.99)
            
            if iteration % 20 == 0:
                print(f"Iteration {iteration}: Best fitness = {self.global_best_fitness:.6f}")
        
        return self.global_best_position, self.global_best_fitness


class BatteryModelOptimizer:
    """
    电池模型参数优化器
    
    整合多种优化方法，用于电池模型参数估计
    """
    
    def __init__(self, battery_model: SmartphoneBatteryModel):
        """
        初始化优化器
        
        参数:
            battery_model: 电池模型实例
        """
        self.model = battery_model
        
        # 参数边界
        self.param_bounds = ParameterBounds()
        
        # 测量数据（用于优化）
        self.measurement_data: Dict = {}
    
    def set_measurement_data(self, time: np.ndarray, soc: np.ndarray,
                             voltage: np.ndarray = None,
                             usage: UsageProfile = None):
        """
        设置测量数据用于参数估计
        
        参数:
            time: 时间数组 (hours)
            soc: SOC测量值数组
            voltage: 电压测量值数组 (可选)
            usage: 使用配置
        """
        self.measurement_data = {
            'time': time,
            'soc': soc,
            'voltage': voltage,
            'usage': usage or UsageProfile()
        }
    
    def _simulate_soc(self, params: np.ndarray) -> np.ndarray:
        """
        使用给定参数模拟SOC
        """
        # 更新模型参数
        model_copy = copy.deepcopy(self.model)
        model_copy.battery_params.nominal_capacity = params[0]
        model_copy.battery_params.internal_resistance = params[1]
        model_copy.V0 = params[2]
        model_copy.K = params[3]
        
        # 模拟
        usage = self.measurement_data['usage']
        time = self.measurement_data['time']
        
        initial_soc = self.measurement_data['soc'][0]
        duration = time[-1] - time[0]
        
        t_sim, soc_sim = model_copy.simulate(initial_soc, usage, duration, len(time))
        
        return soc_sim
    
    def objective_soc_rmse(self, params: np.ndarray) -> float:
        """
        SOC均方根误差目标函数
        """
        try:
            soc_sim = self._simulate_soc(params)
            soc_measured = self.measurement_data['soc']
            
            # 插值到相同时间点
            if len(soc_sim) != len(soc_measured):
                from scipy.interpolate import interp1d
                t_sim = np.linspace(0, 1, len(soc_sim))
                t_meas = np.linspace(0, 1, len(soc_measured))
                f = interp1d(t_sim, soc_sim, kind='linear', fill_value='extrapolate')
                soc_sim = f(t_meas)
            
            rmse = np.sqrt(np.mean((soc_sim - soc_measured) ** 2))
            return rmse
        except Exception:
            return 1e10
    
    def objective_voltage_rmse(self, params: np.ndarray) -> float:
        """
        电压均方根误差目标函数
        """
        if self.measurement_data.get('voltage') is None:
            return 0.0
        
        try:
            soc_sim = self._simulate_soc(params)
            
            # 计算预测电压
            model_copy = copy.deepcopy(self.model)
            model_copy.V0 = params[2]
            model_copy.K = params[3]
            
            voltage_pred = np.array([model_copy.open_circuit_voltage(s) for s in soc_sim])
            voltage_measured = self.measurement_data['voltage']
            
            if len(voltage_pred) != len(voltage_measured):
                from scipy.interpolate import interp1d
                t_sim = np.linspace(0, 1, len(voltage_pred))
                t_meas = np.linspace(0, 1, len(voltage_measured))
                f = interp1d(t_sim, voltage_pred, kind='linear', fill_value='extrapolate')
                voltage_pred = f(t_meas)
            
            rmse = np.sqrt(np.mean((voltage_pred - voltage_measured) ** 2))
            return rmse
        except Exception:
            return 1e10
    
    def objective_combined(self, params: np.ndarray, 
                          weights: Tuple[float, float] = (0.7, 0.3)) -> float:
        """
        组合目标函数
        
        J = w1 * J_soc + w2 * J_voltage
        """
        J_soc = self.objective_soc_rmse(params)
        J_voltage = self.objective_voltage_rmse(params)
        
        return weights[0] * J_soc + weights[1] * J_voltage
    
    def optimize_single_objective(self, method: str = 'differential_evolution') -> Dict:
        """
        单目标优化
        
        参数:
            method: 优化方法 ('differential_evolution', 'pso', 'dual_annealing')
            
        返回:
            优化结果字典
        """
        bounds = [
            self.param_bounds.nominal_capacity,
            self.param_bounds.internal_resistance,
            self.param_bounds.V0,
            self.param_bounds.K
        ]
        
        if method == 'differential_evolution':
            result = differential_evolution(
                self.objective_combined,
                bounds,
                maxiter=100,
                seed=42,
                disp=True
            )
            return {
                'optimal_params': result.x,
                'optimal_value': result.fun,
                'success': result.success,
                'message': result.message
            }
        
        elif method == 'pso':
            pso = ParticleSwarmOptimizer(
                self.objective_combined,
                bounds,
                n_particles=30,
                max_iterations=100
            )
            opt_params, opt_value = pso.optimize()
            return {
                'optimal_params': opt_params,
                'optimal_value': opt_value,
                'history': pso.history
            }
        
        elif method == 'dual_annealing':
            result = dual_annealing(
                self.objective_combined,
                bounds,
                maxiter=1000,
                seed=42
            )
            return {
                'optimal_params': result.x,
                'optimal_value': result.fun,
                'success': result.success
            }
        
        else:
            raise ValueError(f"未知的优化方法: {method}")
    
    def optimize_multi_objective(self, config: OptimizationConfig = None) -> List[Individual]:
        """
        多目标优化
        
        返回:
            Pareto最优解集
        """
        bounds = [
            self.param_bounds.nominal_capacity,
            self.param_bounds.internal_resistance,
            self.param_bounds.V0,
            self.param_bounds.K
        ]
        
        objectives = [
            self.objective_soc_rmse,
            self.objective_voltage_rmse
        ]
        
        nsga = NSGAII(objectives, bounds, config)
        pareto_front = nsga.evolve()
        
        return pareto_front, nsga.history


class SensitivityAnalyzer:
    """
    敏感性分析
    
    分析模型参数对输出的影响程度
    """
    
    def __init__(self, battery_model: SmartphoneBatteryModel):
        self.model = battery_model
    
    def sobol_indices(self, usage: UsageProfile, n_samples: int = 1000) -> Dict:
        """
        Sobol敏感性指数（简化版）
        
        分析各参数对SOC预测的敏感性
        """
        # 参数范围
        param_ranges = {
            'screen_brightness': (0.0, 1.0),
            'cpu_load': (0.0, 1.0),
            'wifi_active': (0, 1),
            'cellular_signal': (0.2, 1.0),
            'temperature': (0, 45)
        }
        
        # 基准功耗
        base_power = self.model.total_power_consumption(usage)
        
        results = {}
        
        for param_name, (low, high) in param_ranges.items():
            variations = []
            
            for val in np.linspace(low, high, 10):
                test_usage = copy.deepcopy(usage)
                
                if param_name == 'screen_brightness':
                    test_usage.screen_brightness = val
                elif param_name == 'cpu_load':
                    test_usage.cpu_load = val
                elif param_name == 'wifi_active':
                    test_usage.wifi_active = bool(val)
                elif param_name == 'cellular_signal':
                    test_usage.cellular_signal_strength = val
                elif param_name == 'temperature':
                    test_usage.ambient_temperature = val
                
                power = self.model.total_power_consumption(test_usage)
                variations.append(power)
            
            # 计算敏感性（归一化方差）
            var = np.var(variations)
            mean = np.mean(variations)
            sensitivity = var / (mean ** 2) if mean > 0 else 0
            
            results[param_name] = {
                'sensitivity': sensitivity,
                'mean_power': mean,
                'power_range': (min(variations), max(variations))
            }
        
        return results
    
    def local_sensitivity(self, usage: UsageProfile, param_name: str, 
                          delta: float = 0.01) -> float:
        """
        局部敏感性分析（偏导数近似）
        
        S = (∂f/∂x) * (x/f)
        """
        test_usage = copy.deepcopy(usage)
        base_power = self.model.total_power_consumption(usage)
        
        # 获取原始参数值
        if param_name == 'screen_brightness':
            x = usage.screen_brightness
            test_usage.screen_brightness = x + delta
        elif param_name == 'cpu_load':
            x = usage.cpu_load
            test_usage.cpu_load = x + delta
        else:
            return 0.0
        
        perturbed_power = self.model.total_power_consumption(test_usage)
        
        # 计算归一化敏感性
        df_dx = (perturbed_power - base_power) / delta
        sensitivity = (df_dx * x / base_power) if base_power > 0 else 0
        
        return sensitivity


class UncertaintyQuantifier:
    """
    不确定性量化
    
    量化模型预测的不确定性
    """
    
    def __init__(self, battery_model: SmartphoneBatteryModel):
        self.model = battery_model
    
    def monte_carlo_uncertainty(self, 
                                 initial_soc: float,
                                 usage: UsageProfile,
                                 duration: float,
                                 n_samples: int = 100,
                                 param_std: Dict[str, float] = None) -> Dict:
        """
        蒙特卡洛不确定性分析
        
        参数:
            initial_soc: 初始SOC
            usage: 使用配置
            duration: 模拟时长
            n_samples: 采样数量
            param_std: 参数标准差字典
            
        返回:
            不确定性分析结果
        """
        if param_std is None:
            param_std = {
                'capacity': 200,  # mAh
                'resistance': 0.01,  # Ω
                'power_error': 0.1  # 10% 功耗误差
            }
        
        soc_trajectories = []
        remaining_times = []
        
        for _ in range(n_samples):
            # 扰动模型参数
            model_copy = copy.deepcopy(self.model)
            
            # 容量扰动
            cap_noise = np.random.normal(0, param_std.get('capacity', 200))
            model_copy.battery_params.nominal_capacity += cap_noise
            
            # 内阻扰动
            res_noise = np.random.normal(0, param_std.get('resistance', 0.01))
            model_copy.battery_params.internal_resistance += res_noise
            model_copy.battery_params.internal_resistance = max(0.01, 
                model_copy.battery_params.internal_resistance)
            
            # 模拟
            try:
                t, soc = model_copy.simulate(initial_soc, usage, duration, 100)
                soc_trajectories.append(soc)
                
                # 计算剩余时间
                remaining = model_copy.estimate_remaining_time(initial_soc, usage)
                remaining_times.append(remaining)
            except Exception:
                continue
        
        if not soc_trajectories:
            return {'error': 'All simulations failed'}
        
        # 统计分析
        soc_array = np.array(soc_trajectories)
        
        return {
            'soc_mean': np.mean(soc_array, axis=0),
            'soc_std': np.std(soc_array, axis=0),
            'soc_percentiles': {
                '5%': np.percentile(soc_array, 5, axis=0),
                '25%': np.percentile(soc_array, 25, axis=0),
                '50%': np.percentile(soc_array, 50, axis=0),
                '75%': np.percentile(soc_array, 75, axis=0),
                '95%': np.percentile(soc_array, 95, axis=0)
            },
            'remaining_time_mean': np.mean(remaining_times),
            'remaining_time_std': np.std(remaining_times),
            'remaining_time_95CI': (
                np.percentile(remaining_times, 2.5),
                np.percentile(remaining_times, 97.5)
            ),
            'n_valid_samples': len(soc_trajectories)
        }


if __name__ == "__main__":
    print("=" * 60)
    print("电池模型参数优化示例")
    print("=" * 60)
    
    from battery_model import SmartphoneBatteryModel, create_moderate_usage
    
    # 创建模型
    model = SmartphoneBatteryModel()
    usage = create_moderate_usage()
    
    # 敏感性分析
    print("\n敏感性分析:")
    print("-" * 40)
    
    analyzer = SensitivityAnalyzer(model)
    sensitivity_results = analyzer.sobol_indices(usage)
    
    for param, result in sensitivity_results.items():
        print(f"{param}:")
        print(f"  敏感性指数: {result['sensitivity']:.4f}")
        print(f"  平均功耗: {result['mean_power']:.1f} mW")
        print(f"  功耗范围: {result['power_range'][0]:.1f} - {result['power_range'][1]:.1f} mW")
    
    # 不确定性量化
    print("\n不确定性量化 (蒙特卡洛):")
    print("-" * 40)
    
    uq = UncertaintyQuantifier(model)
    uncertainty = uq.monte_carlo_uncertainty(0.9, usage, 5.0, n_samples=50)
    
    print(f"有效样本数: {uncertainty['n_valid_samples']}")
    print(f"剩余时间均值: {uncertainty['remaining_time_mean']:.2f} 小时")
    print(f"剩余时间标准差: {uncertainty['remaining_time_std']:.2f} 小时")
    print(f"剩余时间95%置信区间: {uncertainty['remaining_time_95CI'][0]:.2f} - {uncertainty['remaining_time_95CI'][1]:.2f} 小时")
