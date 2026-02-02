"""
敏感性分析模块
================
实现多种敏感性分析方法：
1. 局部敏感性分析 (One-at-a-Time, OAT)
2. 全局敏感性分析 (Sobol指数)
3. Morris筛选法
4. 蒙特卡洛不确定性传播
5. 鲁棒性分析
"""

import numpy as np
from typing import Dict, List, Tuple, Callable, Optional
from dataclasses import dataclass
import warnings
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import itertools
from smartphone_power_model import (
    SmartphonePowerModel, create_model_with_params, DEFAULT_PARAMS
)


@dataclass
class SensitivityResult:
    """敏感性分析结果"""
    param_name: str
    sensitivity_index: float
    normalized_sensitivity: float
    rank: int = 0


class LocalSensitivityAnalyzer:
    """
    局部敏感性分析器 (One-at-a-Time方法)
    
    通过逐个扰动参数来评估输出对各参数的敏感程度
    """
    
    def __init__(self, perturbation_ratio: float = 0.05):
        """
        Parameters:
        -----------
        perturbation_ratio : float
            参数扰动比例 (默认5%)
        """
        self.perturbation_ratio = perturbation_ratio
    
    def analyze(self, 
                base_params: Dict[str, float],
                output_metrics: List[str] = None,
                verbose: bool = True) -> Dict[str, Dict[str, float]]:
        """
        执行OAT敏感性分析
        
        Parameters:
        -----------
        base_params : Dict[str, float]
            基准参数值
        output_metrics : List[str]
            要分析的输出指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, Dict[str, float]]
            每个输出指标对应的参数敏感性字典
        """
        if output_metrics is None:
            output_metrics = ['final_soc', 'avg_power', 'max_temp']
        
        # 基准仿真
        if verbose:
            print("运行基准仿真...")
        
        base_model = create_model_with_params(base_params)
        base_hist = base_model.run_simulation(fast_mode=True)
        base_metrics = base_model.compute_metrics(base_hist)
        
        results = {metric: {} for metric in output_metrics}
        
        param_names = list(base_params.keys())
        n_params = len(param_names)
        
        for i, param_name in enumerate(param_names):
            if verbose:
                print(f"分析参数 {i+1}/{n_params}: {param_name}")
            
            base_val = base_params[param_name]
            delta = base_val * self.perturbation_ratio
            
            if delta == 0:
                delta = 0.001  # 处理零值参数
            
            # 正向扰动
            perturbed_params = base_params.copy()
            perturbed_params[param_name] = base_val + delta
            model_plus = create_model_with_params(perturbed_params)
            hist_plus = model_plus.run_simulation(fast_mode=True)
            metrics_plus = model_plus.compute_metrics(hist_plus)
            
            # 负向扰动
            perturbed_params[param_name] = base_val - delta
            model_minus = create_model_with_params(perturbed_params)
            hist_minus = model_minus.run_simulation(fast_mode=True)
            metrics_minus = model_minus.compute_metrics(hist_minus)
            
            # 计算敏感性（中心差分）
            for metric in output_metrics:
                dy = metrics_plus[metric] - metrics_minus[metric]
                dx = 2 * delta
                
                # 归一化敏感性指数
                if base_metrics[metric] != 0:
                    sensitivity = (dy / dx) * (base_val / base_metrics[metric])
                else:
                    sensitivity = dy / dx
                
                results[metric][param_name] = sensitivity
        
        return results
    
    def compute_elasticity(self,
                          base_params: Dict[str, float],
                          target_metric: str = 'avg_power') -> Dict[str, float]:
        """
        计算弹性系数（百分比变化响应）
        
        Parameters:
        -----------
        base_params : Dict[str, float]
            基准参数
        target_metric : str
            目标指标
        
        Returns:
        --------
        Dict[str, float]
            各参数的弹性系数
        """
        base_model = create_model_with_params(base_params)
        base_hist = base_model.run_simulation(fast_mode=True)
        base_metrics = base_model.compute_metrics(base_hist)
        base_value = base_metrics[target_metric]
        
        elasticities = {}
        
        for param_name, base_val in base_params.items():
            delta_p = base_val * 0.01  # 1%变化
            if delta_p == 0:
                delta_p = 0.001
            
            perturbed_params = base_params.copy()
            perturbed_params[param_name] = base_val + delta_p
            
            model = create_model_with_params(perturbed_params)
            hist = model.run_simulation(fast_mode=True)
            metrics = model.compute_metrics(hist)
            
            delta_y = metrics[target_metric] - base_value
            
            # 弹性 = (ΔY/Y) / (ΔX/X)
            if base_value != 0 and base_val != 0:
                elasticity = (delta_y / base_value) / (delta_p / base_val)
            else:
                elasticity = 0
            
            elasticities[param_name] = elasticity
        
        return elasticities


class MorrisScreening:
    """
    Morris筛选方法
    
    用于识别最重要的参数，适合高维参数空间的初步筛选
    """
    
    def __init__(self, n_trajectories: int = 10, n_levels: int = 4):
        """
        Parameters:
        -----------
        n_trajectories : int
            轨迹数量
        n_levels : int
            参数离散化级别
        """
        self.n_trajectories = n_trajectories
        self.n_levels = n_levels
    
    def _generate_trajectory(self, 
                            param_bounds: Dict[str, Tuple[float, float]],
                            seed: int) -> List[Dict[str, float]]:
        """生成一条Morris轨迹"""
        np.random.seed(seed)
        
        param_names = list(param_bounds.keys())
        n_params = len(param_names)
        
        # 生成起始点
        levels = np.linspace(0, 1, self.n_levels)
        start_indices = np.random.randint(0, self.n_levels - 1, n_params)
        
        trajectory = []
        current_point = {name: levels[idx] for name, idx in zip(param_names, start_indices)}
        trajectory.append(current_point.copy())
        
        # 随机排列参数
        order = np.random.permutation(n_params)
        delta = 1.0 / (self.n_levels - 1)
        
        for i in order:
            param_name = param_names[i]
            current_val = current_point[param_name]
            
            # 决定增加还是减少
            if current_val + delta <= 1.0:
                current_point[param_name] = current_val + delta
            else:
                current_point[param_name] = current_val - delta
            
            trajectory.append(current_point.copy())
        
        # 转换到实际参数范围
        actual_trajectory = []
        for point in trajectory:
            actual_point = {}
            for name, normalized_val in point.items():
                low, high = param_bounds[name]
                actual_point[name] = low + normalized_val * (high - low)
            actual_trajectory.append(actual_point)
        
        return actual_trajectory
    
    def analyze(self,
                param_bounds: Dict[str, Tuple[float, float]],
                target_metric: str = 'avg_power',
                verbose: bool = True) -> Dict[str, Dict[str, float]]:
        """
        执行Morris筛选分析
        
        Parameters:
        -----------
        param_bounds : Dict[str, Tuple[float, float]]
            参数边界
        target_metric : str
            目标指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, Dict[str, float]]
            包含μ*, μ, σ的结果
        """
        param_names = list(param_bounds.keys())
        elementary_effects = {name: [] for name in param_names}
        
        for r in range(self.n_trajectories):
            if verbose:
                print(f"Morris轨迹 {r+1}/{self.n_trajectories}")
            
            trajectory = self._generate_trajectory(param_bounds, seed=r)
            
            # 评估轨迹上的每个点
            outputs = []
            for point in trajectory:
                model = create_model_with_params(point)
                hist = model.run_simulation(fast_mode=True)
                metrics = model.compute_metrics(hist)
                outputs.append(metrics[target_metric])
            
            # 计算基本效应
            for i, param_name in enumerate(param_names):
                if i + 1 < len(outputs):
                    effect = outputs[i + 1] - outputs[i]
                    elementary_effects[param_name].append(effect)
        
        # 计算统计量
        results = {}
        for param_name in param_names:
            effects = np.array(elementary_effects[param_name])
            if len(effects) > 0:
                results[param_name] = {
                    'mu': np.mean(effects),
                    'mu_star': np.mean(np.abs(effects)),
                    'sigma': np.std(effects),
                }
            else:
                results[param_name] = {'mu': 0, 'mu_star': 0, 'sigma': 0}
        
        return results


class SobolAnalyzer:
    """
    Sobol全局敏感性分析
    
    基于方差分解的全局敏感性分析方法
    """
    
    def __init__(self, n_samples: int = 512):
        """
        Parameters:
        -----------
        n_samples : int
            基础样本数量 (总评估次数为 n_samples * (2*k + 2))
        """
        self.n_samples = n_samples
    
    def _generate_sobol_sequence(self, n: int, d: int) -> np.ndarray:
        """生成Sobol序列（简化版本使用准随机数）"""
        # 使用Halton序列作为替代
        def halton(index, base):
            result = 0.0
            f = 1.0 / base
            i = index
            while i > 0:
                result += f * (i % base)
                i //= base
                f /= base
            return result
        
        primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71]
        
        sequence = np.zeros((n, d))
        for i in range(n):
            for j in range(d):
                sequence[i, j] = halton(i + 1, primes[j % len(primes)])
        
        return sequence
    
    def analyze(self,
                param_bounds: Dict[str, Tuple[float, float]],
                target_metric: str = 'avg_power',
                verbose: bool = True) -> Dict[str, Dict[str, float]]:
        """
        执行Sobol敏感性分析
        
        Parameters:
        -----------
        param_bounds : Dict[str, Tuple[float, float]]
            参数边界
        target_metric : str
            目标指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, Dict[str, float]]
            包含一阶和全阶Sobol指数
        """
        param_names = list(param_bounds.keys())
        n_params = len(param_names)
        
        # 生成两个独立的采样矩阵
        A = self._generate_sobol_sequence(self.n_samples, n_params)
        B = self._generate_sobol_sequence(self.n_samples, n_params)
        # 移动B矩阵以确保不同
        B = np.roll(B, self.n_samples // 2, axis=0)
        
        if verbose:
            print("评估基础矩阵A...")
        
        # 评估A矩阵
        f_A = np.zeros(self.n_samples)
        for i in range(self.n_samples):
            if verbose and i % 100 == 0:
                print(f"  A: {i}/{self.n_samples}")
            
            params = {}
            for j, name in enumerate(param_names):
                low, high = param_bounds[name]
                params[name] = low + A[i, j] * (high - low)
            
            model = create_model_with_params(params)
            hist = model.run_simulation(fast_mode=True)
            metrics = model.compute_metrics(hist)
            f_A[i] = metrics[target_metric]
        
        if verbose:
            print("评估基础矩阵B...")
        
        # 评估B矩阵
        f_B = np.zeros(self.n_samples)
        for i in range(self.n_samples):
            if verbose and i % 100 == 0:
                print(f"  B: {i}/{self.n_samples}")
            
            params = {}
            for j, name in enumerate(param_names):
                low, high = param_bounds[name]
                params[name] = low + B[i, j] * (high - low)
            
            model = create_model_with_params(params)
            hist = model.run_simulation(fast_mode=True)
            metrics = model.compute_metrics(hist)
            f_B[i] = metrics[target_metric]
        
        # 计算Sobol指数
        f_0 = np.mean(f_A)
        V_total = np.var(np.concatenate([f_A, f_B]))
        
        results = {}
        
        for k, param_name in enumerate(param_names):
            if verbose:
                print(f"计算参数 {param_name} 的Sobol指数...")
            
            # 构造AB_k矩阵（A矩阵但第k列来自B）
            f_AB_k = np.zeros(self.n_samples)
            for i in range(self.n_samples):
                params = {}
                for j, name in enumerate(param_names):
                    low, high = param_bounds[name]
                    if j == k:
                        params[name] = low + B[i, j] * (high - low)
                    else:
                        params[name] = low + A[i, j] * (high - low)
                
                model = create_model_with_params(params)
                hist = model.run_simulation(fast_mode=True)
                metrics = model.compute_metrics(hist)
                f_AB_k[i] = metrics[target_metric]
            
            # 一阶Sobol指数
            V_i = np.mean(f_B * (f_AB_k - f_A))
            S_i = V_i / V_total if V_total > 0 else 0
            
            # 全阶Sobol指数（Jansen估计器）
            V_Ti = 0.5 * np.mean((f_A - f_AB_k) ** 2)
            S_Ti = V_Ti / V_total if V_total > 0 else 0
            
            results[param_name] = {
                'S1': max(0, min(1, S_i)),
                'ST': max(0, min(1, S_Ti)),
            }
        
        return results


class MonteCarloUncertainty:
    """
    蒙特卡洛不确定性传播分析
    
    通过随机采样评估参数不确定性对输出的影响
    """
    
    def __init__(self, n_samples: int = 500):
        """
        Parameters:
        -----------
        n_samples : int
            蒙特卡洛采样数量
        """
        self.n_samples = n_samples
    
    def propagate(self,
                  param_distributions: Dict[str, Tuple[str, float, float]],
                  target_metrics: List[str] = None,
                  verbose: bool = True) -> Dict[str, Dict[str, float]]:
        """
        执行蒙特卡洛不确定性传播
        
        Parameters:
        -----------
        param_distributions : Dict[str, Tuple[str, float, float]]
            参数分布定义: {参数名: (分布类型, 参数1, 参数2)}
            支持: 'uniform' (min, max), 'normal' (mean, std), 'triangular' (mode, scale)
        target_metrics : List[str]
            目标指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, Dict[str, float]]
            包含均值、标准差、置信区间等统计量
        """
        if target_metrics is None:
            target_metrics = ['final_soc', 'avg_power', 'max_temp']
        
        # 存储所有采样结果
        all_results = {metric: [] for metric in target_metrics}
        
        for i in range(self.n_samples):
            if verbose and i % 50 == 0:
                print(f"蒙特卡洛采样 {i}/{self.n_samples}")
            
            # 从分布中采样参数
            params = {}
            for param_name, (dist_type, p1, p2) in param_distributions.items():
                if dist_type == 'uniform':
                    params[param_name] = np.random.uniform(p1, p2)
                elif dist_type == 'normal':
                    params[param_name] = np.random.normal(p1, p2)
                elif dist_type == 'triangular':
                    # mode=p1, scale=p2 -> triangular(mode-scale, mode, mode+scale)
                    params[param_name] = np.random.triangular(p1 - p2, p1, p1 + p2)
                else:
                    params[param_name] = p1  # 默认使用第一个参数
            
            # 运行模型
            model = create_model_with_params(params)
            hist = model.run_simulation(fast_mode=True)
            metrics = model.compute_metrics(hist)
            
            for metric in target_metrics:
                all_results[metric].append(metrics[metric])
        
        # 计算统计量
        statistics = {}
        for metric in target_metrics:
            values = np.array(all_results[metric])
            statistics[metric] = {
                'mean': np.mean(values),
                'std': np.std(values),
                'cv': np.std(values) / np.mean(values) if np.mean(values) != 0 else 0,
                'min': np.min(values),
                'max': np.max(values),
                'p5': np.percentile(values, 5),
                'p25': np.percentile(values, 25),
                'median': np.median(values),
                'p75': np.percentile(values, 75),
                'p95': np.percentile(values, 95),
                'samples': values,
            }
        
        return statistics


class RobustnessAnalyzer:
    """
    鲁棒性分析器
    
    评估模型在参数变化和使用模式波动下的稳定性
    """
    
    def __init__(self):
        pass
    
    def analyze_parameter_robustness(self,
                                     base_params: Dict[str, float],
                                     perturbation_levels: List[float] = None,
                                     target_metric: str = 'avg_power',
                                     verbose: bool = True) -> Dict[str, np.ndarray]:
        """
        分析参数扰动下的鲁棒性
        
        Parameters:
        -----------
        base_params : Dict[str, float]
            基准参数
        perturbation_levels : List[float]
            扰动级别列表（相对于基准值的比例）
        target_metric : str
            目标指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, np.ndarray]
            各参数在不同扰动级别下的输出值
        """
        if perturbation_levels is None:
            perturbation_levels = np.linspace(-0.3, 0.3, 13)
        
        results = {}
        param_names = list(base_params.keys())
        
        # 基准值
        base_model = create_model_with_params(base_params)
        base_hist = base_model.run_simulation(fast_mode=True)
        base_metrics = base_model.compute_metrics(base_hist)
        base_value = base_metrics[target_metric]
        
        for i, param_name in enumerate(param_names):
            if verbose:
                print(f"分析参数 {i+1}/{len(param_names)}: {param_name}")
            
            outputs = []
            base_param_val = base_params[param_name]
            
            for level in perturbation_levels:
                perturbed_params = base_params.copy()
                perturbed_params[param_name] = base_param_val * (1 + level)
                
                model = create_model_with_params(perturbed_params)
                hist = model.run_simulation(fast_mode=True)
                metrics = model.compute_metrics(hist)
                outputs.append(metrics[target_metric])
            
            results[param_name] = {
                'perturbation_levels': perturbation_levels,
                'outputs': np.array(outputs),
                'base_value': base_value,
                'robustness_score': self._compute_robustness_score(
                    np.array(outputs), base_value
                ),
            }
        
        return results
    
    def _compute_robustness_score(self, outputs: np.ndarray, base: float) -> float:
        """
        计算鲁棒性分数
        
        基于输出变化相对于基准值的稳定性
        """
        if base == 0:
            return 0
        
        relative_changes = np.abs(outputs - base) / abs(base)
        # 鲁棒性分数：变化越小，分数越高
        return 1 / (1 + np.mean(relative_changes))
    
    def analyze_scenario_robustness(self,
                                    base_params: Dict[str, float],
                                    scenarios: Dict[str, Dict[str, float]] = None,
                                    target_metrics: List[str] = None,
                                    verbose: bool = True) -> Dict[str, Dict[str, float]]:
        """
        分析不同使用场景下的鲁棒性
        
        Parameters:
        -----------
        base_params : Dict[str, float]
            基准参数
        scenarios : Dict[str, Dict[str, float]]
            场景定义
        target_metrics : List[str]
            目标指标
        verbose : bool
            是否打印进度
        
        Returns:
        --------
        Dict[str, Dict[str, float]]
            各场景下的指标值
        """
        if target_metrics is None:
            target_metrics = ['final_soc', 'avg_power', 'max_temp']
        
        if scenarios is None:
            scenarios = {
                'baseline': {},
                'high_5G': {'conn_5G_active': 1.8, 'conn_5G_idle': 0.12},
                'low_battery': {'batt_capacity': 3500},
                'hot_environment': {'therm_T_amb': 308.15},
                'cold_environment': {'therm_T_amb': 278.15},
                'old_battery': {'batt_R_internal': 0.08},
                'high_display': {'disp_Beta': 3.5e-3, 'disp_P_static': 0.08},
            }
        
        results = {}
        
        for scenario_name, modifications in scenarios.items():
            if verbose:
                print(f"评估场景: {scenario_name}")
            
            # 应用场景修改
            scenario_params = base_params.copy()
            scenario_params.update(modifications)
            
            model = create_model_with_params(scenario_params)
            hist = model.run_simulation(fast_mode=True)
            metrics = model.compute_metrics(hist)
            
            results[scenario_name] = {metric: metrics[metric] for metric in target_metrics}
        
        return results


def get_default_params() -> Dict[str, float]:
    """获取默认参数值"""
    return {name: info['default'] for name, info in DEFAULT_PARAMS.items()}


def get_param_bounds() -> Dict[str, Tuple[float, float]]:
    """获取参数边界"""
    return {name: (info['min'], info['max']) for name, info in DEFAULT_PARAMS.items()}


def run_comprehensive_analysis(verbose: bool = True) -> Dict:
    """
    运行完整的敏感性分析套件
    
    Returns:
    --------
    Dict
        包含所有分析结果的字典
    """
    base_params = get_default_params()
    param_bounds = get_param_bounds()
    
    results = {}
    
    # 1. 局部敏感性分析
    if verbose:
        print("\n" + "=" * 60)
        print("1. 局部敏感性分析 (OAT)")
        print("=" * 60)
    
    local_analyzer = LocalSensitivityAnalyzer(perturbation_ratio=0.05)
    results['local'] = local_analyzer.analyze(
        base_params, 
        output_metrics=['final_soc', 'avg_power', 'max_temp', 'avg_power_soc', 'avg_power_conn'],
        verbose=verbose
    )
    
    # 2. Morris筛选
    if verbose:
        print("\n" + "=" * 60)
        print("2. Morris筛选分析")
        print("=" * 60)
    
    morris = MorrisScreening(n_trajectories=8, n_levels=4)
    results['morris'] = morris.analyze(param_bounds, 'avg_power', verbose=verbose)
    
    # 3. Sobol分析（降低样本数以加速）
    if verbose:
        print("\n" + "=" * 60)
        print("3. Sobol全局敏感性分析")
        print("=" * 60)
    
    sobol = SobolAnalyzer(n_samples=128)
    results['sobol'] = sobol.analyze(param_bounds, 'avg_power', verbose=verbose)
    
    # 4. 蒙特卡洛不确定性传播
    if verbose:
        print("\n" + "=" * 60)
        print("4. 蒙特卡洛不确定性传播")
        print("=" * 60)
    
    param_distributions = {}
    for name, info in DEFAULT_PARAMS.items():
        # 假设参数服从均匀分布
        param_distributions[name] = ('uniform', info['min'], info['max'])
    
    mc = MonteCarloUncertainty(n_samples=200)
    results['monte_carlo'] = mc.propagate(
        param_distributions,
        target_metrics=['final_soc', 'avg_power', 'max_temp'],
        verbose=verbose
    )
    
    # 5. 鲁棒性分析
    if verbose:
        print("\n" + "=" * 60)
        print("5. 鲁棒性分析")
        print("=" * 60)
    
    robustness = RobustnessAnalyzer()
    results['robustness_param'] = robustness.analyze_parameter_robustness(
        base_params,
        perturbation_levels=np.linspace(-0.2, 0.2, 9),
        target_metric='avg_power',
        verbose=verbose
    )
    
    results['robustness_scenario'] = robustness.analyze_scenario_robustness(
        base_params,
        verbose=verbose
    )
    
    return results


if __name__ == "__main__":
    print("开始综合敏感性分析...")
    results = run_comprehensive_analysis(verbose=True)
    
    print("\n" + "=" * 60)
    print("分析完成！")
    print("=" * 60)
