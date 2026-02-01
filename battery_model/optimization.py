"""
Multi-Objective Optimization for Parameter Estimation
多目标优化用于参数估计

This module implements:
1. NSGA-II based multi-objective optimization
2. Pareto frontier analysis
3. Parameter sensitivity analysis
4. Uncertainty quantification

Objectives:
- Minimize voltage prediction error
- Minimize SOC estimation error
- Minimize thermal prediction error
- Maximize model robustness

References:
[1] Deb et al., "A Fast Elitist Multi-Objective Genetic Algorithm: NSGA-II"
[2] Ramadesigan et al., "Parameter Estimation of Lithium-ion Battery Models"
"""

import numpy as np
from typing import Tuple, List, Dict, Optional, Callable
from dataclasses import dataclass, field
from scipy.optimize import minimize, differential_evolution
import warnings


@dataclass
class OptimizationResult:
    """Result of optimization"""
    parameters: np.ndarray
    objectives: np.ndarray
    pareto_front: np.ndarray = None
    pareto_solutions: np.ndarray = None
    convergence_history: List = field(default_factory=list)


class MultiObjectiveOptimizer:
    """
    Multi-Objective Parameter Optimizer using NSGA-II
    基于NSGA-II的多目标参数优化器
    
    Estimates battery model parameters by minimizing multiple objectives
    simultaneously to find Pareto-optimal solutions.
    """
    
    def __init__(self, model_func: Callable,
                 measurement_data: Dict,
                 param_bounds: Dict):
        """
        Initialize optimizer
        
        Parameters:
        -----------
        model_func : callable
            Function that takes parameters and returns model predictions
        measurement_data : dict
            Dictionary with 't', 'V', 'I', 'T' arrays
        param_bounds : dict
            Dictionary with parameter names and (min, max) tuples
        """
        self.model_func = model_func
        self.data = measurement_data
        self.param_bounds = param_bounds
        self.param_names = list(param_bounds.keys())
        
        # Derived quantities
        self.n_params = len(param_bounds)
        self.bounds_array = np.array([param_bounds[k] for k in self.param_names])
        
        # Population parameters
        self.pop_size = 50
        self.n_generations = 100
        self.crossover_prob = 0.9
        self.mutation_prob = 0.1
        
        # History
        self.generation_history = []
        self.pareto_history = []
    
    def _evaluate_objectives(self, params: np.ndarray) -> np.ndarray:
        """
        Evaluate all objectives for a parameter set
        评估参数集的所有目标函数
        
        Returns array of objective values (to be minimized)
        """
        try:
            # Run model with parameters
            predictions = self.model_func(params)
            
            # Objective 1: Voltage RMSE
            V_pred = predictions.get('V', np.zeros_like(self.data['V']))
            V_error = np.sqrt(np.mean((V_pred - self.data['V'])**2))
            
            # Objective 2: SOC tracking error (if ground truth available)
            if 'SOC_true' in self.data:
                SOC_pred = predictions.get('SOC', np.zeros_like(self.data['SOC_true']))
                SOC_error = np.sqrt(np.mean((SOC_pred - self.data['SOC_true'])**2))
            else:
                # Use Coulomb counting as reference
                SOC_error = V_error * 0.1  # Proxy
            
            # Objective 3: Thermal prediction error
            if 'T' in self.data and 'T' in predictions:
                T_pred = predictions['T']
                T_error = np.sqrt(np.mean((T_pred - self.data['T'])**2))
            else:
                T_error = 0.0
            
            # Objective 4: Model complexity penalty (regularization)
            # Penalize parameters far from nominal values
            param_deviation = np.sum((params - 0.5 * (self.bounds_array[:, 0] + self.bounds_array[:, 1]))**2 / 
                                     (self.bounds_array[:, 1] - self.bounds_array[:, 0])**2)
            
            return np.array([V_error, SOC_error, T_error, param_deviation * 0.01])
            
        except Exception as e:
            # Return large values for failed evaluations
            return np.array([1e6, 1e6, 1e6, 1e6])
    
    def _dominates(self, obj1: np.ndarray, obj2: np.ndarray) -> bool:
        """Check if obj1 dominates obj2 (Pareto dominance)"""
        return np.all(obj1 <= obj2) and np.any(obj1 < obj2)
    
    def _non_dominated_sort(self, population: np.ndarray, 
                            objectives: np.ndarray) -> List[List[int]]:
        """
        Non-dominated sorting for NSGA-II
        非支配排序
        """
        n = len(population)
        domination_count = np.zeros(n, dtype=int)
        dominated_solutions = [[] for _ in range(n)]
        fronts = [[]]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    if self._dominates(objectives[i], objectives[j]):
                        dominated_solutions[i].append(j)
                    elif self._dominates(objectives[j], objectives[i]):
                        domination_count[i] += 1
            
            if domination_count[i] == 0:
                fronts[0].append(i)
        
        k = 0
        while len(fronts[k]) > 0:
            next_front = []
            for i in fronts[k]:
                for j in dominated_solutions[i]:
                    domination_count[j] -= 1
                    if domination_count[j] == 0:
                        next_front.append(j)
            k += 1
            fronts.append(next_front)
        
        return fronts[:-1]  # Remove empty last front
    
    def _crowding_distance(self, objectives: np.ndarray, 
                           front: List[int]) -> np.ndarray:
        """
        Calculate crowding distance for diversity preservation
        计算拥挤距离
        """
        n = len(front)
        if n <= 2:
            return np.full(n, np.inf)
        
        distances = np.zeros(n)
        n_objectives = objectives.shape[1]
        
        for m in range(n_objectives):
            sorted_idx = np.argsort(objectives[front, m])
            distances[sorted_idx[0]] = np.inf
            distances[sorted_idx[-1]] = np.inf
            
            obj_range = objectives[front[sorted_idx[-1]], m] - objectives[front[sorted_idx[0]], m]
            if obj_range > 0:
                for i in range(1, n - 1):
                    distances[sorted_idx[i]] += (
                        objectives[front[sorted_idx[i + 1]], m] - 
                        objectives[front[sorted_idx[i - 1]], m]
                    ) / obj_range
        
        return distances
    
    def _selection(self, population: np.ndarray, objectives: np.ndarray,
                   fronts: List[List[int]], crowding: Dict[int, float]) -> np.ndarray:
        """Tournament selection based on rank and crowding distance"""
        selected = []
        n = len(population)
        
        # Create rank mapping
        ranks = np.zeros(n)
        for rank, front in enumerate(fronts):
            for idx in front:
                ranks[idx] = rank
        
        for _ in range(self.pop_size):
            # Tournament selection (binary)
            i, j = np.random.choice(n, 2, replace=False)
            
            if ranks[i] < ranks[j]:
                winner = i
            elif ranks[j] < ranks[i]:
                winner = j
            elif crowding.get(i, 0) > crowding.get(j, 0):
                winner = i
            else:
                winner = j
            
            selected.append(population[winner])
        
        return np.array(selected)
    
    def _crossover(self, parent1: np.ndarray, parent2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Simulated Binary Crossover (SBX)"""
        if np.random.random() > self.crossover_prob:
            return parent1.copy(), parent2.copy()
        
        eta = 20  # Distribution index
        child1 = np.zeros_like(parent1)
        child2 = np.zeros_like(parent2)
        
        for i in range(self.n_params):
            if np.random.random() < 0.5:
                if abs(parent1[i] - parent2[i]) > 1e-10:
                    y1 = min(parent1[i], parent2[i])
                    y2 = max(parent1[i], parent2[i])
                    
                    rand = np.random.random()
                    beta = 1 + (2 * (y1 - self.bounds_array[i, 0]) / (y2 - y1))
                    alpha = 2 - beta ** (-(eta + 1))
                    
                    if rand <= 1 / alpha:
                        betaq = (rand * alpha) ** (1 / (eta + 1))
                    else:
                        betaq = (1 / (2 - rand * alpha)) ** (1 / (eta + 1))
                    
                    child1[i] = 0.5 * ((y1 + y2) - betaq * (y2 - y1))
                    child2[i] = 0.5 * ((y1 + y2) + betaq * (y2 - y1))
                else:
                    child1[i] = parent1[i]
                    child2[i] = parent2[i]
            else:
                child1[i] = parent1[i]
                child2[i] = parent2[i]
        
        # Clip to bounds
        child1 = np.clip(child1, self.bounds_array[:, 0], self.bounds_array[:, 1])
        child2 = np.clip(child2, self.bounds_array[:, 0], self.bounds_array[:, 1])
        
        return child1, child2
    
    def _mutation(self, individual: np.ndarray) -> np.ndarray:
        """Polynomial mutation"""
        mutant = individual.copy()
        eta = 20  # Distribution index
        
        for i in range(self.n_params):
            if np.random.random() < self.mutation_prob:
                y = individual[i]
                lb, ub = self.bounds_array[i]
                delta1 = (y - lb) / (ub - lb)
                delta2 = (ub - y) / (ub - lb)
                
                rand = np.random.random()
                if rand < 0.5:
                    xy = 1 - delta1
                    val = 2 * rand + (1 - 2 * rand) * (xy ** (eta + 1))
                    deltaq = val ** (1 / (eta + 1)) - 1
                else:
                    xy = 1 - delta2
                    val = 2 * (1 - rand) + 2 * (rand - 0.5) * (xy ** (eta + 1))
                    deltaq = 1 - val ** (1 / (eta + 1))
                
                mutant[i] = y + deltaq * (ub - lb)
                mutant[i] = np.clip(mutant[i], lb, ub)
        
        return mutant
    
    def optimize(self, n_generations: int = None,
                 verbose: bool = True) -> OptimizationResult:
        """
        Run NSGA-II optimization
        运行NSGA-II优化
        """
        if n_generations is not None:
            self.n_generations = n_generations
        
        # Initialize population using Latin Hypercube Sampling
        population = self._latin_hypercube_sampling(self.pop_size)
        
        # Evaluate initial population
        objectives = np.array([self._evaluate_objectives(ind) for ind in population])
        
        for gen in range(self.n_generations):
            # Non-dominated sorting
            fronts = self._non_dominated_sort(population, objectives)
            
            # Calculate crowding distance
            crowding = {}
            for front in fronts:
                distances = self._crowding_distance(objectives, front)
                for i, idx in enumerate(front):
                    crowding[idx] = distances[i]
            
            # Selection
            selected = self._selection(population, objectives, fronts, crowding)
            
            # Create offspring
            offspring = []
            for i in range(0, self.pop_size, 2):
                p1 = selected[i]
                p2 = selected[min(i + 1, self.pop_size - 1)]
                c1, c2 = self._crossover(p1, p2)
                offspring.extend([self._mutation(c1), self._mutation(c2)])
            
            offspring = np.array(offspring[:self.pop_size])
            
            # Evaluate offspring
            offspring_objectives = np.array([self._evaluate_objectives(ind) for ind in offspring])
            
            # Combine parent and offspring
            combined_pop = np.vstack([population, offspring])
            combined_obj = np.vstack([objectives, offspring_objectives])
            
            # Select next generation
            fronts = self._non_dominated_sort(combined_pop, combined_obj)
            
            new_population = []
            new_objectives = []
            
            for front in fronts:
                if len(new_population) + len(front) <= self.pop_size:
                    for idx in front:
                        new_population.append(combined_pop[idx])
                        new_objectives.append(combined_obj[idx])
                else:
                    # Select based on crowding distance
                    distances = self._crowding_distance(combined_obj, front)
                    sorted_idx = np.argsort(distances)[::-1]
                    remaining = self.pop_size - len(new_population)
                    for i in sorted_idx[:remaining]:
                        idx = front[i]
                        new_population.append(combined_pop[idx])
                        new_objectives.append(combined_obj[idx])
                    break
            
            population = np.array(new_population)
            objectives = np.array(new_objectives)
            
            # Record history
            pareto_front = objectives[list(fronts[0])] if fronts else objectives
            self.generation_history.append({
                'generation': gen,
                'pareto_size': len(fronts[0]) if fronts else 0,
                'best_objectives': np.min(objectives, axis=0)
            })
            
            if verbose and gen % 10 == 0:
                print(f"Generation {gen}: Pareto front size = {len(fronts[0]) if fronts else 0}, "
                      f"Best V_error = {np.min(objectives[:, 0]):.6f}")
        
        # Extract final Pareto front
        final_fronts = self._non_dominated_sort(population, objectives)
        pareto_indices = final_fronts[0] if final_fronts else list(range(len(population)))
        pareto_solutions = population[pareto_indices]
        pareto_objectives = objectives[pareto_indices]
        
        # Select compromise solution (minimum weighted sum)
        weights = np.array([0.4, 0.3, 0.2, 0.1])  # Prioritize voltage accuracy
        weighted_sum = np.sum(pareto_objectives * weights, axis=1)
        best_idx = np.argmin(weighted_sum)
        
        return OptimizationResult(
            parameters=pareto_solutions[best_idx],
            objectives=pareto_objectives[best_idx],
            pareto_front=pareto_objectives,
            pareto_solutions=pareto_solutions,
            convergence_history=self.generation_history
        )
    
    def _latin_hypercube_sampling(self, n_samples: int) -> np.ndarray:
        """Generate initial population using Latin Hypercube Sampling"""
        samples = np.zeros((n_samples, self.n_params))
        
        for i in range(self.n_params):
            lb, ub = self.bounds_array[i]
            perm = np.random.permutation(n_samples)
            for j in range(n_samples):
                samples[perm[j], i] = lb + (ub - lb) * (j + np.random.random()) / n_samples
        
        return samples


class SensitivityAnalyzer:
    """
    Global Sensitivity Analysis using Sobol indices
    基于Sobol指数的全局敏感性分析
    """
    
    def __init__(self, model_func: Callable, param_bounds: Dict):
        self.model_func = model_func
        self.param_bounds = param_bounds
        self.param_names = list(param_bounds.keys())
        self.n_params = len(param_bounds)
        self.bounds_array = np.array([param_bounds[k] for k in self.param_names])
    
    def sobol_analysis(self, n_samples: int = 1024,
                       output_func: Callable = None) -> Dict:
        """
        Compute Sobol sensitivity indices
        计算Sobol敏感性指数
        
        Parameters:
        -----------
        n_samples : int
            Number of base samples (actual evaluations = n_samples * (2*n_params + 2))
        output_func : callable
            Function to extract scalar output from model results
        """
        if output_func is None:
            output_func = lambda x: x.get('discharge_time', 0)
        
        # Generate Sobol sequence
        n = n_samples
        d = self.n_params
        
        # Base samples A and B
        A = self._sobol_samples(n)
        B = self._sobol_samples(n)
        
        # Scale to bounds
        A_scaled = self._scale_samples(A)
        B_scaled = self._scale_samples(B)
        
        # Evaluate base samples
        f_A = np.array([output_func(self.model_func(params)) for params in A_scaled])
        f_B = np.array([output_func(self.model_func(params)) for params in B_scaled])
        
        # Compute first-order and total-effect indices
        S1 = np.zeros(d)  # First-order
        ST = np.zeros(d)  # Total effect
        
        for i in range(d):
            # Create AB_i matrix (A with i-th column from B)
            AB_i = A_scaled.copy()
            AB_i[:, i] = B_scaled[:, i]
            
            # Create BA_i matrix (B with i-th column from A)
            BA_i = B_scaled.copy()
            BA_i[:, i] = A_scaled[:, i]
            
            f_AB_i = np.array([output_func(self.model_func(params)) for params in AB_i])
            f_BA_i = np.array([output_func(self.model_func(params)) for params in BA_i])
            
            # Variance estimates
            f_0 = np.mean(f_A)
            var_total = np.var(np.concatenate([f_A, f_B]))
            
            if var_total > 0:
                # First-order index (Saltelli formula)
                S1[i] = np.mean(f_B * (f_AB_i - f_A)) / var_total
                
                # Total effect index
                ST[i] = np.mean((f_A - f_AB_i)**2) / (2 * var_total)
        
        # Normalize and clip
        S1 = np.clip(S1, 0, 1)
        ST = np.clip(ST, 0, 1)
        
        return {
            'S1': dict(zip(self.param_names, S1)),
            'ST': dict(zip(self.param_names, ST)),
            'interaction': dict(zip(self.param_names, ST - S1))
        }
    
    def _sobol_samples(self, n: int) -> np.ndarray:
        """Generate Sobol sequence samples in [0, 1]^d"""
        # Simple quasi-random generation (approximate Sobol)
        samples = np.zeros((n, self.n_params))
        for i in range(n):
            for j in range(self.n_params):
                samples[i, j] = self._van_der_corput(i + 1, 2 + j)
        return samples
    
    def _van_der_corput(self, n: int, base: int) -> float:
        """Van der Corput sequence for quasi-random sampling"""
        q, bk = 0, 1 / base
        while n > 0:
            q += (n % base) * bk
            n //= base
            bk /= base
        return q
    
    def _scale_samples(self, samples: np.ndarray) -> np.ndarray:
        """Scale samples from [0,1] to parameter bounds"""
        scaled = np.zeros_like(samples)
        for i in range(self.n_params):
            lb, ub = self.bounds_array[i]
            scaled[:, i] = lb + (ub - lb) * samples[:, i]
        return scaled


class BayesianOptimizer:
    """
    Bayesian Optimization for efficient parameter search
    贝叶斯优化用于高效参数搜索
    
    Uses Gaussian Process surrogate model with Expected Improvement acquisition
    """
    
    def __init__(self, objective_func: Callable, param_bounds: Dict):
        self.objective = objective_func
        self.param_bounds = param_bounds
        self.param_names = list(param_bounds.keys())
        self.n_params = len(param_bounds)
        self.bounds_array = np.array([param_bounds[k] for k in self.param_names])
        
        # Observation history
        self.X_observed = []
        self.y_observed = []
        
        # GP hyperparameters
        self.length_scale = 0.5
        self.signal_var = 1.0
        self.noise_var = 0.01
    
    def _rbf_kernel(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        """RBF (Gaussian) kernel"""
        # Normalize to [0, 1]
        X1_norm = (X1 - self.bounds_array[:, 0]) / (self.bounds_array[:, 1] - self.bounds_array[:, 0])
        X2_norm = (X2 - self.bounds_array[:, 0]) / (self.bounds_array[:, 1] - self.bounds_array[:, 0])
        
        # Squared distances
        sq_dist = np.sum((X1_norm[:, np.newaxis, :] - X2_norm[np.newaxis, :, :])**2, axis=2)
        return self.signal_var * np.exp(-0.5 * sq_dist / self.length_scale**2)
    
    def _gp_predict(self, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """GP posterior prediction"""
        if len(self.X_observed) == 0:
            return np.zeros(len(X_test)), np.ones(len(X_test)) * self.signal_var
        
        X_train = np.array(self.X_observed)
        y_train = np.array(self.y_observed)
        
        K = self._rbf_kernel(X_train, X_train) + self.noise_var * np.eye(len(X_train))
        K_star = self._rbf_kernel(X_test, X_train)
        K_star_star = self._rbf_kernel(X_test, X_test)
        
        try:
            K_inv = np.linalg.inv(K)
        except np.linalg.LinAlgError:
            K_inv = np.linalg.pinv(K)
        
        mu = K_star @ K_inv @ y_train
        var = np.diag(K_star_star - K_star @ K_inv @ K_star.T)
        var = np.maximum(var, 1e-6)
        
        return mu, var
    
    def _expected_improvement(self, X: np.ndarray, xi: float = 0.01) -> np.ndarray:
        """Expected Improvement acquisition function"""
        mu, var = self._gp_predict(X)
        sigma = np.sqrt(var)
        
        if len(self.y_observed) == 0:
            return sigma
        
        y_best = np.min(self.y_observed)
        
        with np.errstate(divide='warn'):
            Z = (y_best - mu - xi) / sigma
            ei = (y_best - mu - xi) * self._norm_cdf(Z) + sigma * self._norm_pdf(Z)
            ei[sigma < 1e-6] = 0
        
        return ei
    
    def _norm_cdf(self, x: np.ndarray) -> np.ndarray:
        """Standard normal CDF"""
        return 0.5 * (1 + np.erf(x / np.sqrt(2)))
    
    def _norm_pdf(self, x: np.ndarray) -> np.ndarray:
        """Standard normal PDF"""
        return np.exp(-0.5 * x**2) / np.sqrt(2 * np.pi)
    
    def optimize(self, n_iterations: int = 50,
                 n_initial: int = 10,
                 verbose: bool = True) -> OptimizationResult:
        """
        Run Bayesian optimization
        运行贝叶斯优化
        """
        # Initial random sampling
        for i in range(n_initial):
            x = np.random.uniform(self.bounds_array[:, 0], self.bounds_array[:, 1])
            y = self.objective(x)
            self.X_observed.append(x)
            self.y_observed.append(y)
        
        # Optimization loop
        for i in range(n_iterations):
            # Find next point by maximizing EI
            best_ei = -np.inf
            best_x = None
            
            # Grid search for EI maximum
            n_candidates = 1000
            X_candidates = np.random.uniform(
                self.bounds_array[:, 0], 
                self.bounds_array[:, 1],
                size=(n_candidates, self.n_params)
            )
            ei_values = self._expected_improvement(X_candidates)
            best_idx = np.argmax(ei_values)
            x_next = X_candidates[best_idx]
            
            # Evaluate objective
            y_next = self.objective(x_next)
            self.X_observed.append(x_next)
            self.y_observed.append(y_next)
            
            if verbose and i % 10 == 0:
                print(f"Iteration {i}: Best = {np.min(self.y_observed):.6f}")
        
        # Return best solution
        best_idx = np.argmin(self.y_observed)
        
        return OptimizationResult(
            parameters=np.array(self.X_observed[best_idx]),
            objectives=np.array([self.y_observed[best_idx]]),
            convergence_history=list(zip(range(len(self.y_observed)), self.y_observed))
        )
