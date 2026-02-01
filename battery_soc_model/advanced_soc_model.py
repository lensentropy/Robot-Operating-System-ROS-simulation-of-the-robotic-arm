"""
高级SOC-能耗耦合模型
Advanced SOC-Energy Coupled Model with Kalman Filter & Multi-Objective Optimization

特性:
1. 扩展卡尔曼滤波 (EKF) - 非线性状态估计与噪声滤波
2. 多目标优化 (NSGA-II) - 电池寿命/性能/温度帕累托优化
3. 真实场景建模 - 传感器噪声、环境不确定性、用户行为随机性
4. 完整微分方程组 - 电化学-热-电耦合系统
5. 高端可视化 - 3D曲面、热力图、实时仪表盘

Author: MCM Expert
Date: 2026
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from scipy.optimize import minimize, differential_evolution
from scipy.linalg import expm
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
from dataclasses import dataclass, field
from typing import Tuple, List, Dict, Callable, Optional
import warnings
warnings.filterwarnings('ignore')

# ============================================================================
# 第一部分：系统参数定义
# ============================================================================

@dataclass
class BatterySystemParams:
    """电池系统完整参数"""
    # 电池电化学参数
    Q_max: float = 4000.0           # 最大容量 (mAh)
    V_nominal: float = 3.7          # 标称电压 (V)
    V_max: float = 4.2              # 最大电压 (V)
    V_min: float = 2.8              # 最小电压 (V)
    R_int_25: float = 0.08          # 25°C内阻 (Ω)
    
    # 热参数
    C_th_batt: float = 50.0         # 电池热容 (J/K)
    R_th_batt: float = 10.0         # 电池热阻 (K/W)
    C_th_soc: float = 5.0           # SoC模块热容 (J/K)
    R_th_soc_batt: float = 15.0     # SoC到电池热阻 (K/W)
    R_th_soc_env: float = 20.0      # SoC到环境热阻 (K/W)
    T_env: float = 25.0             # 环境温度 (°C)
    
    # OCV-SOC关系 (多项式系数)
    ocv_coeffs: tuple = (3.0, 0.8, 0.2, -0.1, 0.1)
    
    # 降额与老化参数
    N_derate: float = 1.0           # 降额因子
    aging_factor: float = 0.0       # 老化因子 (0-1)
    
    # PMIC效率
    eta_PMIC: float = 0.92          # PMIC转换效率

@dataclass
class SensorNoiseParams:
    """传感器噪声参数"""
    sigma_SOC: float = 0.02         # SOC测量噪声标准差
    sigma_V: float = 0.05           # 电压测量噪声标准差 (V)
    sigma_I: float = 0.02           # 电流测量噪声标准差 (A)
    sigma_T: float = 0.5            # 温度测量噪声标准差 (°C)
    sigma_P: float = 0.05           # 功率测量噪声标准差 (W)

@dataclass
class ProcessNoiseParams:
    """过程噪声参数"""
    sigma_SOC_process: float = 0.001    # SOC过程噪声
    sigma_T_process: float = 0.1        # 温度过程噪声
    sigma_T_soc_process: float = 0.2    # SoC温度过程噪声

# ============================================================================
# 第二部分：扩展卡尔曼滤波 (EKF)
# ============================================================================

class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器用于电池状态估计
    
    状态向量: x = [SOC, T_batt, T_soc, I_bg]^T
    观测向量: z = [V_batt, T_batt_meas, P_total]^T
    """
    
    def __init__(self, 
                 system_params: BatterySystemParams,
                 sensor_noise: SensorNoiseParams = None,
                 process_noise: ProcessNoiseParams = None):
        
        self.params = system_params
        self.sensor_noise = sensor_noise or SensorNoiseParams()
        self.process_noise = process_noise or ProcessNoiseParams()
        
        # 状态维度
        self.n_states = 4  # [SOC, T_batt, T_soc, I_bg]
        self.n_obs = 3     # [V_batt, T_batt, P_total]
        
        # 初始化状态和协方差
        self.x = np.array([1.0, 25.0, 25.0, 0.1])  # 初始状态
        self.P = np.diag([0.01, 1.0, 1.0, 0.01])   # 初始协方差
        
        # 过程噪声协方差 Q
        self.Q = np.diag([
            self.process_noise.sigma_SOC_process**2,
            self.process_noise.sigma_T_process**2,
            self.process_noise.sigma_T_soc_process**2,
            0.001**2  # I_bg过程噪声
        ])
        
        # 观测噪声协方差 R
        self.R = np.diag([
            self.sensor_noise.sigma_V**2,
            self.sensor_noise.sigma_T**2,
            self.sensor_noise.sigma_P**2
        ])
        
    def V_OCV(self, soc: float) -> float:
        """开路电压模型"""
        coeffs = self.params.ocv_coeffs
        soc = np.clip(soc, 0.01, 0.99)
        V = sum(c * soc**i for i, c in enumerate(coeffs))
        return np.clip(V, self.params.V_min, self.params.V_max)
    
    def dV_OCV_dSOC(self, soc: float) -> float:
        """OCV对SOC的导数"""
        coeffs = self.params.ocv_coeffs
        soc = np.clip(soc, 0.01, 0.99)
        return sum(i * c * soc**(i-1) for i, c in enumerate(coeffs) if i > 0)
    
    def R_int(self, soc: float, T: float) -> float:
        """内阻模型"""
        soc_factor = 1 + 0.5 * (1 - soc)**2
        T_ref = 25.0
        temp_factor = 1 + 0.005 * (T_ref - T)
        temp_factor = np.clip(temp_factor, 0.5, 3.0)
        return self.params.R_int_25 * soc_factor * temp_factor
    
    def state_transition(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """
        状态转移函数 f(x, u)
        
        Args:
            x: 当前状态 [SOC, T_batt, T_soc, I_bg]
            u: 控制输入 [P_load, user_activity]
            dt: 时间步长 (小时)
            
        Returns:
            x_next: 下一状态
        """
        SOC, T_batt, T_soc, I_bg = x
        P_load, user_activity = u
        
        # 计算电池电压和电流
        V_batt = self.V_OCV(SOC) - I_bg * self.R_int(SOC, T_batt)
        V_batt = np.clip(V_batt, self.params.V_min, self.params.V_max)
        
        # 总电流
        I_total = P_load / (V_batt * self.params.eta_PMIC) + I_bg
        
        # SOC动态
        Q_max_Ah = self.params.Q_max / 1000.0
        dSOC = -I_total / (Q_max_Ah * self.params.N_derate)
        
        # 电池温度动态
        R_int = self.R_int(SOC, T_batt)
        P_joule = I_total**2 * R_int
        dT_batt = (P_joule - (T_batt - self.params.T_env) / self.params.R_th_batt) / self.params.C_th_batt
        
        # SoC模块温度动态
        P_soc = P_load * 0.3  # 假设30%功耗在SoC模块
        dT_soc = (P_soc - (T_soc - T_batt) / self.params.R_th_soc_batt 
                  - (T_soc - self.params.T_env) / self.params.R_th_soc_env) / self.params.C_th_soc
        
        # 后台电流动态 (Ornstein-Uhlenbeck过程)
        theta = 0.5  # 回归速率
        mu_bg = 0.05 + 0.1 * user_activity  # 均值
        dI_bg = theta * (mu_bg - I_bg)
        
        # 欧拉积分
        x_next = np.array([
            np.clip(SOC + dSOC * dt, 0.01, 0.99),
            T_batt + dT_batt * dt * 3600,  # dt是小时，需要转换
            T_soc + dT_soc * dt * 3600,
            np.clip(I_bg + dI_bg * dt, 0.01, 0.5)
        ])
        
        return x_next
    
    def observation_model(self, x: np.ndarray) -> np.ndarray:
        """
        观测函数 h(x)
        
        Args:
            x: 状态 [SOC, T_batt, T_soc, I_bg]
            
        Returns:
            z: 观测 [V_batt, T_batt, P_total]
        """
        SOC, T_batt, T_soc, I_bg = x
        
        V_batt = self.V_OCV(SOC) - I_bg * self.R_int(SOC, T_batt)
        P_total = I_bg * V_batt
        
        return np.array([V_batt, T_batt, P_total])
    
    def jacobian_F(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """状态转移雅可比矩阵 ∂f/∂x"""
        eps = 1e-6
        n = len(x)
        F = np.zeros((n, n))
        
        f0 = self.state_transition(x, u, dt)
        for i in range(n):
            x_pert = x.copy()
            x_pert[i] += eps
            f_pert = self.state_transition(x_pert, u, dt)
            F[:, i] = (f_pert - f0) / eps
            
        return F
    
    def jacobian_H(self, x: np.ndarray) -> np.ndarray:
        """观测雅可比矩阵 ∂h/∂x"""
        eps = 1e-6
        n_x = len(x)
        n_z = self.n_obs
        H = np.zeros((n_z, n_x))
        
        h0 = self.observation_model(x)
        for i in range(n_x):
            x_pert = x.copy()
            x_pert[i] += eps
            h_pert = self.observation_model(x_pert)
            H[:, i] = (h_pert - h0) / eps
            
        return H
    
    def predict(self, u: np.ndarray, dt: float):
        """EKF预测步骤"""
        # 状态预测
        self.x = self.state_transition(self.x, u, dt)
        
        # 协方差预测
        F = self.jacobian_F(self.x, u, dt)
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x.copy()
    
    def update(self, z: np.ndarray):
        """EKF更新步骤"""
        # 观测预测
        z_pred = self.observation_model(self.x)
        
        # 创新
        y = z - z_pred
        
        # 创新协方差
        H = self.jacobian_H(self.x)
        S = H @ self.P @ H.T + self.R
        
        # 卡尔曼增益
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 状态更新
        self.x = self.x + K @ y
        self.x[0] = np.clip(self.x[0], 0.01, 0.99)  # SOC限制
        
        # 协方差更新
        I = np.eye(self.n_states)
        self.P = (I - K @ H) @ self.P
        
        return self.x.copy(), K
    
    def filter_step(self, z: np.ndarray, u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """完整的滤波步骤"""
        x_pred = self.predict(u, dt)
        x_est, K = self.update(z)
        return x_est, self.P.copy()

# ============================================================================
# 第三部分：多目标优化 (NSGA-II 简化版)
# ============================================================================

class MultiObjectiveOptimizer:
    """
    多目标优化器 - 优化电池使用策略
    
    目标:
    1. 最大化电池使用时间
    2. 最小化平均温度 (延长寿命)
    3. 最大化用户体验 (性能)
    
    决策变量:
    - CPU频率缩放因子
    - 屏幕亮度缩放因子
    - 5G功率控制
    """
    
    def __init__(self, system_model):
        self.system = system_model
        self.population_size = 50
        self.n_generations = 30
        self.n_objectives = 3
        self.n_variables = 3
        
        # 决策变量边界
        self.bounds = [
            (0.5, 1.0),   # CPU频率缩放 (50%-100%)
            (0.3, 1.0),   # 亮度缩放 (30%-100%)
            (0.0, 1.0)    # 5G功率控制 (0-100%)
        ]
        
    def evaluate_objectives(self, decision_vars: np.ndarray, 
                           initial_SOC: float = 1.0,
                           duration_hours: float = 12) -> np.ndarray:
        """
        评估目标函数
        
        Args:
            decision_vars: [cpu_scale, brightness_scale, power_5g]
            initial_SOC: 初始SOC
            duration_hours: 评估时长
            
        Returns:
            objectives: [使用时间, 平均温度, 性能得分]
        """
        cpu_scale, brightness_scale, power_5g = decision_vars
        
        # 简化仿真
        P_base = 1.5  # 基础功耗
        P_cpu = 0.8 * cpu_scale**2
        P_disp = 0.6 * brightness_scale
        P_5g = 1.5 * power_5g
        P_total = P_base + P_cpu + P_disp + P_5g
        
        # 目标1: 使用时间 (要最大化，取负值)
        Q_max_Wh = self.system.battery_params.Q_max * self.system.battery_params.V_nominal / 1000
        battery_life = Q_max_Wh * initial_SOC / P_total
        
        # 目标2: 平均温度 (要最小化)
        T_base = 25.0
        T_rise = 5 * P_total / 3.0  # 功耗导致温升
        avg_temp = T_base + T_rise
        
        # 目标3: 性能得分 (要最大化，取负值)
        performance = 0.5 * cpu_scale + 0.3 * brightness_scale + 0.2 * power_5g
        
        return np.array([-battery_life, avg_temp, -performance])
    
    def dominates(self, obj1: np.ndarray, obj2: np.ndarray) -> bool:
        """判断obj1是否支配obj2"""
        return np.all(obj1 <= obj2) and np.any(obj1 < obj2)
    
    def fast_non_dominated_sort(self, objectives: np.ndarray) -> List[List[int]]:
        """快速非支配排序"""
        n = len(objectives)
        domination_count = np.zeros(n, dtype=int)
        dominated_solutions = [[] for _ in range(n)]
        fronts = [[]]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    if self.dominates(objectives[i], objectives[j]):
                        dominated_solutions[i].append(j)
                    elif self.dominates(objectives[j], objectives[i]):
                        domination_count[i] += 1
            
            if domination_count[i] == 0:
                fronts[0].append(i)
        
        i = 0
        while len(fronts[i]) > 0:
            next_front = []
            for p in fronts[i]:
                for q in dominated_solutions[p]:
                    domination_count[q] -= 1
                    if domination_count[q] == 0:
                        next_front.append(q)
            i += 1
            fronts.append(next_front)
        
        return fronts[:-1]
    
    def crowding_distance(self, objectives: np.ndarray, front: List[int]) -> np.ndarray:
        """计算拥挤距离"""
        n = len(front)
        if n <= 2:
            return np.full(n, np.inf)
        
        distances = np.zeros(n)
        
        for m in range(self.n_objectives):
            sorted_indices = np.argsort(objectives[front, m])
            distances[sorted_indices[0]] = np.inf
            distances[sorted_indices[-1]] = np.inf
            
            obj_range = objectives[front[sorted_indices[-1]], m] - objectives[front[sorted_indices[0]], m]
            if obj_range > 0:
                for i in range(1, n - 1):
                    distances[sorted_indices[i]] += (
                        objectives[front[sorted_indices[i + 1]], m] - 
                        objectives[front[sorted_indices[i - 1]], m]
                    ) / obj_range
        
        return distances
    
    def optimize(self) -> Dict:
        """运行NSGA-II优化"""
        # 初始化种群
        population = np.random.uniform(
            low=[b[0] for b in self.bounds],
            high=[b[1] for b in self.bounds],
            size=(self.population_size, self.n_variables)
        )
        
        history = {'generations': [], 'pareto_fronts': []}
        
        for gen in range(self.n_generations):
            # 评估目标
            objectives = np.array([self.evaluate_objectives(ind) for ind in population])
            
            # 非支配排序
            fronts = self.fast_non_dominated_sort(objectives)
            
            # 记录帕累托前沿
            pareto_indices = fronts[0]
            history['generations'].append(gen)
            history['pareto_fronts'].append({
                'solutions': population[pareto_indices].copy(),
                'objectives': objectives[pareto_indices].copy()
            })
            
            # 选择与生成新种群
            new_population = []
            
            for front in fronts:
                if len(new_population) + len(front) <= self.population_size:
                    new_population.extend(front)
                else:
                    distances = self.crowding_distance(objectives, front)
                    sorted_by_dist = np.argsort(-distances)
                    remaining = self.population_size - len(new_population)
                    new_population.extend([front[i] for i in sorted_by_dist[:remaining]])
                    break
            
            # 交叉和变异
            offspring = []
            for _ in range(self.population_size):
                # 锦标赛选择
                i1, i2 = np.random.choice(new_population, 2, replace=False)
                
                # SBX交叉
                parent1, parent2 = population[i1], population[i2]
                child = 0.5 * (parent1 + parent2) + 0.1 * np.random.randn(self.n_variables)
                
                # 多项式变异
                if np.random.rand() < 0.1:
                    mut_idx = np.random.randint(self.n_variables)
                    child[mut_idx] += 0.1 * np.random.randn()
                
                # 边界约束
                for v in range(self.n_variables):
                    child[v] = np.clip(child[v], self.bounds[v][0], self.bounds[v][1])
                
                offspring.append(child)
            
            population = np.array(offspring)
        
        # 最终评估
        final_objectives = np.array([self.evaluate_objectives(ind) for ind in population])
        final_fronts = self.fast_non_dominated_sort(final_objectives)
        
        return {
            'pareto_solutions': population[final_fronts[0]],
            'pareto_objectives': final_objectives[final_fronts[0]],
            'history': history
        }

# ============================================================================
# 第四部分：完整耦合微分方程系统
# ============================================================================

class FullCoupledSystem:
    """
    完整耦合微分方程系统
    
    状态变量: [SOC, T_batt, T_soc, x_lock, I_bg]
    
    包含:
    1. 电池电化学-热耦合
    2. SoC模块电热耦合
    3. GNSS信号锁定动态
    4. 后台任务随机过程
    """
    
    def __init__(self, params: BatterySystemParams = None):
        self.params = params or BatterySystemParams()
        self.user_model = ContinuousTimeMarkovModel()
        
    def V_OCV(self, soc: float) -> float:
        """开路电压"""
        coeffs = self.params.ocv_coeffs
        soc = np.clip(soc, 0.01, 0.99)
        V = sum(c * soc**i for i, c in enumerate(coeffs))
        return np.clip(V, self.params.V_min, self.params.V_max)
    
    def R_int(self, soc: float, T: float) -> float:
        """内阻模型"""
        soc_factor = 1 + 0.5 * (1 - soc)**2
        temp_factor = 1 + 0.005 * (25.0 - T)
        temp_factor = np.clip(temp_factor, 0.5, 3.0)
        aging_factor = 1 + 0.5 * self.params.aging_factor
        return self.params.R_int_25 * soc_factor * temp_factor * aging_factor
    
    def coupled_ode(self, t: float, y: np.ndarray, 
                    hw_params_func: Callable,
                    env_params_func: Callable) -> np.ndarray:
        """
        完整耦合微分方程组
        
        Args:
            t: 时间 (小时)
            y: 状态 [SOC, T_batt, T_soc, x_lock, I_bg]
            hw_params_func: 硬件参数函数
            env_params_func: 环境参数函数
        """
        SOC, T_batt, T_soc, x_lock, I_bg = y
        
        # 边界限制
        SOC = np.clip(SOC, 0.01, 0.99)
        x_lock = np.clip(x_lock, 0, 1)
        I_bg = np.clip(I_bg, 0.01, 0.5)
        
        # 获取参数
        hw = hw_params_func(t)
        env = env_params_func(t)
        
        # === 各模块功耗计算 ===
        
        # 1. SoC模块功耗 (动态 + 漏电)
        cpu_freq = hw.get('cpu_freq', 1.5)
        cpu_util = hw.get('cpu_util', 30)
        V_dd = 1.0
        C_eff = 1e-9
        I_leak_25 = 0.01
        
        P_soc_dyn = C_eff * V_dd**2 * (cpu_freq * 1e9) * (cpu_util / 100)
        leak_factor = np.exp((T_soc - 25) / 10)
        P_soc_leak = I_leak_25 * V_dd * leak_factor
        P_soc = P_soc_dyn + P_soc_leak
        
        # 2. 显示模块功耗
        brightness = hw.get('brightness', 500)
        apl = hw.get('apl', 50)
        refresh_rate = hw.get('refresh_rate', 60)
        
        P_disp_static = 0.3
        P_disp_backlight = 0.8 * (brightness / 1000)
        P_disp_content = 0.002 * refresh_rate * (apl / 100)
        P_disp = P_disp_static + P_disp_backlight + P_disp_content if brightness > 10 else 0.01
        
        # 3. 5G通信模块功耗
        signal_strength = env.get('signal_strength', 0.8)
        data_rate = hw.get('data_rate', 10)
        
        if data_rate > 0:
            path_loss_factor = 1 + 0.3 * (1 - signal_strength)
            P_5G = 0.2 + 1.5 * (data_rate / 100) * path_loss_factor
        else:
            P_5G = 0.05
        
        # 4. 蓝牙模块功耗 (事件驱动)
        bt_active = hw.get('bt_active', False)
        audio_streaming = hw.get('audio_streaming', False)
        
        if audio_streaming:
            P_BT = 0.12
        elif bt_active:
            P_BT = 0.06
        else:
            P_BT = 0.02
        
        # 5. GNSS模块功耗 (信号耦合)
        gnss_enabled = hw.get('gnss_enabled', False)
        
        if gnss_enabled:
            P_LNA = 0.02
            P_acq = 0.15
            P_track = 0.05
            P_GNSS = P_LNA + x_lock * P_track + (1 - x_lock) * P_acq
        else:
            P_GNSS = 0
        
        # 6. 后台任务功耗
        V_batt = self.V_OCV(SOC) - I_bg * self.R_int(SOC, T_batt)
        V_batt = np.clip(V_batt, self.params.V_min, self.params.V_max)
        P_bg = V_batt * I_bg
        
        # === 总功耗与电流 ===
        P_total = P_soc + P_disp + P_5G + P_BT + P_GNSS + P_bg
        I_total = P_total / (V_batt * self.params.eta_PMIC)
        
        # === 微分方程 ===
        
        # 1. SOC方程
        Q_max_Ah = self.params.Q_max / 1000.0
        dSOC_dt = -I_total / (Q_max_Ah * self.params.N_derate)
        
        # 2. 电池温度方程
        R_int = self.R_int(SOC, T_batt)
        P_joule = I_total**2 * R_int
        dV_dT = -0.0003  # 熵变系数
        P_entropy = I_total * (T_batt + 273.15) * dV_dT
        
        dT_batt_dt = (P_joule + P_entropy - 
                     (T_batt - self.params.T_env) / self.params.R_th_batt) / self.params.C_th_batt
        
        # 3. SoC模块温度方程
        dT_soc_dt = (P_soc - 
                    (T_soc - T_batt) / self.params.R_th_soc_batt -
                    (T_soc - self.params.T_env) / self.params.R_th_soc_env) / self.params.C_th_soc
        
        # 4. GNSS锁定状态方程
        S_env = env.get('gnss_signal', 0.7)
        tau_react = 0.5  # 反应时间常数
        dx_lock_dt = (S_env - x_lock) / tau_react if gnss_enabled else -x_lock / tau_react
        
        # 5. 后台电流方程 (Ornstein-Uhlenbeck)
        user_activity = hw.get('user_activity', 0.3)
        theta_bg = 0.5
        mu_bg = 0.05 + 0.1 * user_activity
        sigma_bg = 0.02
        # 确定性部分
        dI_bg_dt = theta_bg * (mu_bg - I_bg)
        
        # 时间单位转换 (小时 -> 秒的导数)
        return np.array([
            dSOC_dt,
            dT_batt_dt * 3600,  # 转换为/小时
            dT_soc_dt * 3600,
            dx_lock_dt,
            dI_bg_dt
        ])

class ContinuousTimeMarkovModel:
    """连续时间马尔科夫用户行为模型"""
    
    def __init__(self):
        self.states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        self.n_states = 4
        
        # 转移速率矩阵
        self.Q_sleep = self._prob_to_rate(np.array([
            [0.995, 0.005, 0.000, 0.000],
            [0.600, 0.400, 0.000, 0.000],
            [0.100, 0.000, 0.900, 0.000],
            [0.100, 0.000, 0.000, 0.900]
        ]))
        
        self.Q_work = self._prob_to_rate(np.array([
            [0.850, 0.145, 0.003, 0.002],
            [0.250, 0.700, 0.040, 0.010],
            [0.100, 0.100, 0.800, 0.000],
            [0.200, 0.100, 0.000, 0.700]
        ]))
        
        self.Q_leisure = self._prob_to_rate(np.array([
            [0.800, 0.150, 0.030, 0.020],
            [0.050, 0.650, 0.200, 0.100],
            [0.010, 0.040, 0.940, 0.010],
            [0.010, 0.010, 0.010, 0.970]
        ]))
        
    def _prob_to_rate(self, P, dt=1/60):
        """转移概率矩阵转换为速率矩阵"""
        return (P - np.eye(self.n_states)) / dt
    
    def get_Q(self, hour: float) -> np.ndarray:
        """根据时间获取转移速率矩阵"""
        hour = hour % 24
        if hour >= 23 or hour < 7:
            return self.Q_sleep
        elif (hour >= 9 and hour < 12) or (hour >= 14 and hour < 18):
            return self.Q_work
        else:
            return self.Q_leisure

# ============================================================================
# 第五部分：真实场景仿真器
# ============================================================================

class RealisticScenarioSimulator:
    """真实场景仿真器"""
    
    def __init__(self, 
                 system_params: BatterySystemParams = None,
                 sensor_noise: SensorNoiseParams = None):
        
        self.params = system_params or BatterySystemParams()
        self.noise = sensor_noise or SensorNoiseParams()
        
        self.coupled_system = FullCoupledSystem(self.params)
        self.ekf = ExtendedKalmanFilter(self.params, self.noise)
        self.user_model = ContinuousTimeMarkovModel()
        
    def generate_environment(self, t: float) -> Dict:
        """生成环境参数"""
        hour = t % 24
        
        # 环境温度 (日变化)
        T_env = 20 + 8 * np.sin(2 * np.pi * (hour - 6) / 24)
        
        # 信号强度 (位置相关随机性)
        base_signal = 0.7 + 0.2 * np.sin(2 * np.pi * hour / 24)
        signal_strength = np.clip(base_signal + 0.1 * np.random.randn(), 0.2, 1.0)
        
        # GNSS信号 (室内/室外)
        is_indoor = hour >= 9 and hour < 18  # 工作时间假设室内
        gnss_signal = 0.3 if is_indoor else 0.8
        gnss_signal += 0.1 * np.random.randn()
        gnss_signal = np.clip(gnss_signal, 0.1, 0.95)
        
        return {
            'T_env': T_env,
            'signal_strength': signal_strength,
            'gnss_signal': gnss_signal,
            'is_indoor': is_indoor
        }
    
    def generate_hw_params(self, t: float, user_state_dist: np.ndarray) -> Dict:
        """根据用户状态生成硬件参数"""
        
        # 状态到参数的映射
        cpu_freq_map = [0.3, 1.2, 1.5, 2.5]
        cpu_util_map = [1, 15, 25, 85]
        brightness_map = [0, 400, 600, 800]
        apl_map = [0, 70, 40, 60]
        
        # 期望值
        cpu_freq = np.sum(user_state_dist * cpu_freq_map)
        cpu_util = np.sum(user_state_dist * cpu_util_map)
        brightness = np.sum(user_state_dist * brightness_map)
        apl = np.sum(user_state_dist * apl_map)
        
        # 添加随机波动
        cpu_freq += 0.1 * np.random.randn()
        cpu_util += 5 * np.random.randn()
        brightness += 50 * np.random.randn()
        
        # 其他参数
        hour = t % 24
        data_rate = 20 * (1 - user_state_dist[0]) * (0.8 + 0.4 * np.random.rand())
        bt_active = user_state_dist[1] + user_state_dist[2] > 0.3
        audio_streaming = user_state_dist[2] > 0.3
        gnss_enabled = hour >= 7 and hour < 23 and np.random.rand() > 0.7
        user_activity = 1 - user_state_dist[0]
        
        return {
            'cpu_freq': np.clip(cpu_freq, 0.2, 3.0),
            'cpu_util': np.clip(cpu_util, 0, 100),
            'brightness': np.clip(brightness, 0, 1200),
            'apl': np.clip(apl, 0, 100),
            'refresh_rate': 60 if user_state_dist[3] < 0.3 else 120,
            'data_rate': np.clip(data_rate, 0, 100),
            'bt_active': bt_active,
            'audio_streaming': audio_streaming,
            'gnss_enabled': gnss_enabled,
            'user_activity': user_activity
        }
    
    def simulate(self, 
                 duration_hours: float = 24,
                 start_hour: float = 8,
                 initial_SOC: float = 1.0,
                 dt_minutes: float = 1.0) -> Dict:
        """
        运行完整仿真
        """
        n_steps = int(duration_hours * 60 / dt_minutes)
        dt_hours = dt_minutes / 60
        
        # 初始化
        # 真实状态: [SOC, T_batt, T_soc, x_lock, I_bg]
        y_true = np.array([initial_SOC, 25.0, 25.0, 0.0, 0.1])
        
        # 用户状态分布
        user_state_dist = np.array([0.3, 0.4, 0.2, 0.1])
        
        # 重置EKF
        self.ekf.x = np.array([initial_SOC, 25.0, 25.0, 0.1])
        self.ekf.P = np.diag([0.01, 1.0, 1.0, 0.01])
        
        # 存储历史
        time_history = np.zeros(n_steps)
        true_state_history = np.zeros((n_steps, 5))
        estimated_state_history = np.zeros((n_steps, 4))
        measured_history = np.zeros((n_steps, 3))
        power_history = np.zeros(n_steps)
        user_state_history = np.zeros((n_steps, 4))
        env_history = []
        hw_history = []
        
        for i in range(n_steps):
            t = i * dt_hours
            current_hour = (start_hour + t) % 24
            time_history[i] = t
            
            # 更新用户状态分布
            Q = self.user_model.get_Q(current_hour)
            P_dt = expm(Q * dt_hours)
            user_state_dist = user_state_dist @ P_dt
            user_state_dist = np.clip(user_state_dist, 0, 1)
            user_state_dist /= user_state_dist.sum()
            user_state_history[i] = user_state_dist
            
            # 生成环境和硬件参数
            env = self.generate_environment(current_hour)
            hw = self.generate_hw_params(current_hour, user_state_dist)
            env_history.append(env)
            hw_history.append(hw)
            
            # 真实系统演化 (添加过程噪声)
            hw_params_func = lambda t: hw
            env_params_func = lambda t: env
            
            dy = self.coupled_system.coupled_ode(t, y_true, hw_params_func, env_params_func)
            
            # 添加过程噪声
            process_noise = np.array([
                self.noise.sigma_SOC * np.random.randn() * 0.01,
                0.1 * np.random.randn(),
                0.2 * np.random.randn(),
                0.05 * np.random.randn(),
                0.01 * np.random.randn()
            ])
            
            y_true = y_true + dy * dt_hours + process_noise * np.sqrt(dt_hours)
            y_true[0] = np.clip(y_true[0], 0.01, 0.99)
            y_true[3] = np.clip(y_true[3], 0, 1)
            y_true[4] = np.clip(y_true[4], 0.01, 0.5)
            
            true_state_history[i] = y_true
            
            # 生成带噪声的观测
            SOC_true, T_batt_true, T_soc_true, x_lock_true, I_bg_true = y_true
            V_batt_true = self.coupled_system.V_OCV(SOC_true) - I_bg_true * self.coupled_system.R_int(SOC_true, T_batt_true)
            P_total_true = V_batt_true * I_bg_true * 5  # 近似总功率
            
            z = np.array([
                V_batt_true + self.noise.sigma_V * np.random.randn(),
                T_batt_true + self.noise.sigma_T * np.random.randn(),
                P_total_true + self.noise.sigma_P * np.random.randn()
            ])
            measured_history[i] = z
            
            # EKF滤波
            u = np.array([P_total_true, hw['user_activity']])
            x_est, _ = self.ekf.filter_step(z, u, dt_hours)
            estimated_state_history[i] = x_est
            
            power_history[i] = P_total_true
            
            # 检查电池耗尽
            if y_true[0] <= 0.02:
                # 截断历史
                time_history = time_history[:i+1]
                true_state_history = true_state_history[:i+1]
                estimated_state_history = estimated_state_history[:i+1]
                measured_history = measured_history[:i+1]
                power_history = power_history[:i+1]
                user_state_history = user_state_history[:i+1]
                env_history = env_history[:i+1]
                hw_history = hw_history[:i+1]
                break
        
        return {
            'time': time_history,
            'true_state': true_state_history,
            'estimated_state': estimated_state_history,
            'measurements': measured_history,
            'power': power_history,
            'user_state': user_state_history,
            'env_history': env_history,
            'hw_history': hw_history,
            'start_hour': start_hour
        }

# ============================================================================
# 第六部分：高端可视化
# ============================================================================

class AdvancedVisualization:
    """高端可视化工具"""
    
    def __init__(self):
        self.colors = {
            'primary': '#2E86AB',
            'secondary': '#A23B72',
            'accent': '#F18F01',
            'danger': '#C73E1D',
            'success': '#28A745',
            'dark': '#1A1A2E',
            'light': '#E8E8E8'
        }
        
    def create_dashboard(self, results: Dict, figsize: Tuple = (20, 16)):
        """创建综合仪表盘"""
        fig = plt.figure(figsize=figsize, facecolor='#1A1A2E')
        gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.3, wspace=0.3)
        
        time = results['time']
        true_state = results['true_state']
        est_state = results['estimated_state']
        measurements = results['measurements']
        power = results['power']
        user_state = results['user_state']
        
        # 1. SOC对比图 (真实 vs 估计)
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.set_facecolor('#16213E')
        ax1.plot(time, true_state[:, 0] * 100, color='#00D9FF', 
                linewidth=2, label='True SOC', alpha=0.8)
        ax1.plot(time, est_state[:, 0] * 100, color='#FF6B6B', 
                linewidth=2, linestyle='--', label='EKF Estimated', alpha=0.8)
        ax1.fill_between(time, true_state[:, 0] * 100, est_state[:, 0] * 100,
                        alpha=0.2, color='#FF6B6B')
        ax1.axhline(y=20, color='#FFD93D', linestyle=':', alpha=0.5, label='Low Battery')
        ax1.axhline(y=5, color='#FF6B6B', linestyle=':', alpha=0.7, label='Critical')
        ax1.set_ylabel('SOC (%)', color='white', fontsize=11)
        ax1.set_title('SOC: True vs EKF Estimated', color='white', fontsize=12, fontweight='bold')
        ax1.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', labelcolor='white')
        ax1.tick_params(colors='white')
        ax1.grid(True, alpha=0.2, color='white')
        ax1.set_xlim([0, time[-1]])
        
        # 2. 温度分布图
        ax2 = fig.add_subplot(gs[0, 2:])
        ax2.set_facecolor('#16213E')
        ax2.plot(time, true_state[:, 1], color='#FF6B6B', linewidth=2, label='Battery Temp')
        ax2.plot(time, true_state[:, 2], color='#4ECDC4', linewidth=2, label='SoC Temp')
        ax2.fill_between(time, true_state[:, 1], true_state[:, 2], alpha=0.2, color='#FFD93D')
        ax2.axhline(y=45, color='#FF6B6B', linestyle='--', alpha=0.5, label='Warning (45°C)')
        ax2.set_ylabel('Temperature (°C)', color='white', fontsize=11)
        ax2.set_title('Thermal Dynamics', color='white', fontsize=12, fontweight='bold')
        ax2.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', labelcolor='white')
        ax2.tick_params(colors='white')
        ax2.grid(True, alpha=0.2, color='white')
        
        # 3. 功耗时间序列
        ax3 = fig.add_subplot(gs[1, :2])
        ax3.set_facecolor('#16213E')
        ax3.fill_between(time, 0, power, color='#4ECDC4', alpha=0.3)
        ax3.plot(time, power, color='#4ECDC4', linewidth=1.5)
        
        # 移动平均
        window = min(30, len(power) // 5)
        if window > 1:
            power_ma = np.convolve(power, np.ones(window)/window, mode='valid')
            time_ma = time[(window-1)//2:-(window-1)//2] if window > 1 else time
            if len(time_ma) > len(power_ma):
                time_ma = time_ma[:len(power_ma)]
            ax3.plot(time_ma, power_ma, color='#FFD93D', linewidth=2, label='Moving Average')
        
        ax3.set_ylabel('Power (W)', color='white', fontsize=11)
        ax3.set_title('Power Consumption Profile', color='white', fontsize=12, fontweight='bold')
        ax3.tick_params(colors='white')
        ax3.grid(True, alpha=0.2, color='white')
        ax3.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', labelcolor='white')
        
        # 4. 用户状态堆叠图
        ax4 = fig.add_subplot(gs[1, 2:])
        ax4.set_facecolor('#16213E')
        colors_state = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D']
        labels = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        ax4.stackplot(time, user_state.T, labels=labels, colors=colors_state, alpha=0.8)
        ax4.set_ylabel('Probability', color='white', fontsize=11)
        ax4.set_title('User State Distribution', color='white', fontsize=12, fontweight='bold')
        ax4.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', 
                   labelcolor='white', ncol=2)
        ax4.tick_params(colors='white')
        ax4.set_ylim([0, 1])
        
        # 5. 估计误差分析
        ax5 = fig.add_subplot(gs[2, :2])
        ax5.set_facecolor('#16213E')
        soc_error = (true_state[:, 0] - est_state[:, 0]) * 100
        ax5.plot(time, soc_error, color='#FF6B6B', linewidth=1.5)
        ax5.fill_between(time, 0, soc_error, where=soc_error >= 0, 
                        color='#4ECDC4', alpha=0.3, label='Overestimate')
        ax5.fill_between(time, 0, soc_error, where=soc_error < 0, 
                        color='#FF6B6B', alpha=0.3, label='Underestimate')
        ax5.axhline(y=0, color='white', linestyle='-', alpha=0.3)
        ax5.set_ylabel('SOC Error (%)', color='white', fontsize=11)
        ax5.set_xlabel('Time (hours)', color='white', fontsize=11)
        ax5.set_title('EKF Estimation Error', color='white', fontsize=12, fontweight='bold')
        ax5.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', labelcolor='white')
        ax5.tick_params(colors='white')
        ax5.grid(True, alpha=0.2, color='white')
        
        # 6. 电压与电流
        ax6 = fig.add_subplot(gs[2, 2:])
        ax6.set_facecolor('#16213E')
        ax6.plot(time, measurements[:, 0], color='#00D9FF', linewidth=1.5, label='Voltage (V)')
        ax6.set_ylabel('Voltage (V)', color='#00D9FF', fontsize=11)
        ax6.tick_params(axis='y', colors='#00D9FF')
        
        ax6b = ax6.twinx()
        current = power / measurements[:, 0]
        ax6b.plot(time, current * 1000, color='#FFD93D', linewidth=1.5, label='Current (mA)')
        ax6b.set_ylabel('Current (mA)', color='#FFD93D', fontsize=11)
        ax6b.tick_params(axis='y', colors='#FFD93D')
        
        ax6.set_xlabel('Time (hours)', color='white', fontsize=11)
        ax6.set_title('Voltage & Current', color='white', fontsize=12, fontweight='bold')
        ax6.tick_params(axis='x', colors='white')
        ax6.grid(True, alpha=0.2, color='white')
        
        # 7. SOC-功耗相空间图
        ax7 = fig.add_subplot(gs[3, :2])
        ax7.set_facecolor('#16213E')
        scatter = ax7.scatter(true_state[:, 0] * 100, power, 
                             c=time, cmap='plasma', s=10, alpha=0.6)
        cbar = plt.colorbar(scatter, ax=ax7)
        cbar.ax.yaxis.set_tick_params(color='white')
        cbar.ax.set_ylabel('Time (h)', color='white')
        plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')
        ax7.set_xlabel('SOC (%)', color='white', fontsize=11)
        ax7.set_ylabel('Power (W)', color='white', fontsize=11)
        ax7.set_title('SOC-Power Phase Space', color='white', fontsize=12, fontweight='bold')
        ax7.tick_params(colors='white')
        ax7.grid(True, alpha=0.2, color='white')
        
        # 8. 统计信息面板
        ax8 = fig.add_subplot(gs[3, 2:])
        ax8.set_facecolor('#16213E')
        ax8.axis('off')
        
        # 计算统计量
        total_time = time[-1]
        initial_soc = true_state[0, 0] * 100
        final_soc = true_state[-1, 0] * 100
        avg_power = np.mean(power)
        max_power = np.max(power)
        avg_temp = np.mean(true_state[:, 1])
        max_temp = np.max(true_state[:, 1])
        rmse_soc = np.sqrt(np.mean((true_state[:, 0] - est_state[:, 0])**2)) * 100
        
        stats_text = f"""
        ╔══════════════════════════════════════════════════╗
        ║           SIMULATION STATISTICS                  ║
        ╠══════════════════════════════════════════════════╣
        ║  Total Simulation Time:  {total_time:>8.1f} hours            ║
        ║  Initial SOC:            {initial_soc:>8.1f} %               ║
        ║  Final SOC:              {final_soc:>8.1f} %               ║
        ║  SOC Consumed:           {initial_soc - final_soc:>8.1f} %               ║
        ╠══════════════════════════════════════════════════╣
        ║  Average Power:          {avg_power:>8.2f} W               ║
        ║  Peak Power:             {max_power:>8.2f} W               ║
        ║  Average Temperature:    {avg_temp:>8.1f} °C              ║
        ║  Peak Temperature:       {max_temp:>8.1f} °C              ║
        ╠══════════════════════════════════════════════════╣
        ║  EKF RMSE (SOC):         {rmse_soc:>8.3f} %               ║
        ╚══════════════════════════════════════════════════╝
        """
        
        ax8.text(0.05, 0.5, stats_text, transform=ax8.transAxes, fontsize=10,
                verticalalignment='center', fontfamily='monospace', color='#00D9FF')
        
        plt.suptitle('Advanced Battery SOC-Energy Coupled System Dashboard', 
                    fontsize=16, fontweight='bold', color='white', y=0.98)
        
        return fig
    
    def create_3d_visualization(self, results: Dict, figsize: Tuple = (16, 12)):
        """创建3D可视化"""
        fig = plt.figure(figsize=figsize, facecolor='#1A1A2E')
        
        time = results['time']
        true_state = results['true_state']
        power = results['power']
        
        # 1. SOC-温度-功耗 3D曲面
        ax1 = fig.add_subplot(2, 2, 1, projection='3d', facecolor='#16213E')
        
        SOC = true_state[:, 0] * 100
        T = true_state[:, 1]
        P = power
        
        # 创建网格数据
        n_points = min(100, len(time))
        idx = np.linspace(0, len(time)-1, n_points).astype(int)
        
        scatter = ax1.scatter(SOC[idx], T[idx], P[idx], c=time[idx], 
                             cmap='viridis', s=30, alpha=0.8)
        ax1.plot(SOC, T, P, color='cyan', alpha=0.3, linewidth=0.5)
        
        ax1.set_xlabel('SOC (%)', color='white', fontsize=10)
        ax1.set_ylabel('Temperature (°C)', color='white', fontsize=10)
        ax1.set_zlabel('Power (W)', color='white', fontsize=10)
        ax1.set_title('SOC-Temperature-Power Phase Space', color='white', fontsize=11)
        ax1.tick_params(colors='white')
        ax1.xaxis.pane.fill = False
        ax1.yaxis.pane.fill = False
        ax1.zaxis.pane.fill = False
        
        # 2. 时间-SOC-功耗 3D轨迹
        ax2 = fig.add_subplot(2, 2, 2, projection='3d', facecolor='#16213E')
        
        ax2.plot(time, SOC, P, color='#FF6B6B', linewidth=2, alpha=0.8)
        ax2.scatter(time[idx], SOC[idx], P[idx], c=T[idx], cmap='coolwarm', s=20, alpha=0.6)
        
        ax2.set_xlabel('Time (h)', color='white', fontsize=10)
        ax2.set_ylabel('SOC (%)', color='white', fontsize=10)
        ax2.set_zlabel('Power (W)', color='white', fontsize=10)
        ax2.set_title('Temporal Evolution Trajectory', color='white', fontsize=11)
        ax2.tick_params(colors='white')
        
        # 3. 用户状态3D分布
        ax3 = fig.add_subplot(2, 2, 3, projection='3d', facecolor='#16213E')
        
        user_state = results['user_state']
        colors_state = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D']
        labels = ['Sleep', 'Light', 'Stream', 'Game']
        
        for i in range(4):
            ax3.bar3d(time[idx], [i] * len(idx), np.zeros(len(idx)),
                     np.diff(time[idx], append=time[idx[-1]]),
                     0.8, user_state[idx, i],
                     color=colors_state[i], alpha=0.7)
        
        ax3.set_xlabel('Time (h)', color='white', fontsize=10)
        ax3.set_ylabel('State', color='white', fontsize=10)
        ax3.set_zlabel('Probability', color='white', fontsize=10)
        ax3.set_title('User State Evolution 3D', color='white', fontsize=11)
        ax3.tick_params(colors='white')
        ax3.set_yticks(range(4))
        ax3.set_yticklabels(labels)
        
        # 4. 估计误差3D分布
        ax4 = fig.add_subplot(2, 2, 4, projection='3d', facecolor='#16213E')
        
        est_state = results['estimated_state']
        soc_error = (true_state[:, 0] - est_state[:, 0]) * 100
        
        ax4.plot(time, SOC, soc_error, color='#4ECDC4', linewidth=2)
        ax4.scatter(time[idx], SOC[idx], soc_error[idx], 
                   c=np.abs(soc_error[idx]), cmap='Reds', s=30, alpha=0.7)
        
        ax4.set_xlabel('Time (h)', color='white', fontsize=10)
        ax4.set_ylabel('SOC (%)', color='white', fontsize=10)
        ax4.set_zlabel('Error (%)', color='white', fontsize=10)
        ax4.set_title('EKF Estimation Error Distribution', color='white', fontsize=11)
        ax4.tick_params(colors='white')
        
        plt.suptitle('3D Visualization of Battery System Dynamics', 
                    fontsize=14, fontweight='bold', color='white', y=0.98)
        plt.tight_layout()
        
        return fig
    
    def create_heatmap_analysis(self, results: Dict, figsize: Tuple = (16, 10)):
        """创建热力图分析"""
        fig = plt.figure(figsize=figsize, facecolor='#1A1A2E')
        
        time = results['time']
        true_state = results['true_state']
        power = results['power']
        user_state = results['user_state']
        
        # 1. SOC-时间热力图
        ax1 = fig.add_subplot(2, 2, 1)
        ax1.set_facecolor('#16213E')
        
        # 创建2D数据
        n_bins = 50
        time_bins = np.linspace(0, time[-1], n_bins)
        soc_bins = np.linspace(0, 100, n_bins)
        
        H, xedges, yedges = np.histogram2d(time, true_state[:, 0] * 100, 
                                           bins=[time_bins, soc_bins])
        
        im1 = ax1.imshow(H.T, origin='lower', aspect='auto',
                        extent=[0, time[-1], 0, 100], cmap='plasma')
        ax1.plot(time, true_state[:, 0] * 100, 'w-', linewidth=1, alpha=0.5)
        plt.colorbar(im1, ax=ax1, label='Density')
        ax1.set_xlabel('Time (hours)', color='white')
        ax1.set_ylabel('SOC (%)', color='white')
        ax1.set_title('SOC Trajectory Density', color='white', fontweight='bold')
        ax1.tick_params(colors='white')
        
        # 2. 功耗-温度相关性热力图
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_facecolor('#16213E')
        
        power_bins = np.linspace(power.min(), power.max(), 30)
        temp_bins = np.linspace(true_state[:, 1].min(), true_state[:, 1].max(), 30)
        
        H2, xe2, ye2 = np.histogram2d(power, true_state[:, 1], bins=[power_bins, temp_bins])
        
        im2 = ax2.imshow(H2.T, origin='lower', aspect='auto',
                        extent=[power.min(), power.max(), 
                               true_state[:, 1].min(), true_state[:, 1].max()],
                        cmap='inferno')
        plt.colorbar(im2, ax=ax2, label='Frequency')
        ax2.set_xlabel('Power (W)', color='white')
        ax2.set_ylabel('Temperature (°C)', color='white')
        ax2.set_title('Power-Temperature Correlation', color='white', fontweight='bold')
        ax2.tick_params(colors='white')
        
        # 3. 用户状态转移热力图
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.set_facecolor('#16213E')
        
        # 计算状态转移频率
        state_seq = np.argmax(user_state, axis=1)
        transition_matrix = np.zeros((4, 4))
        for i in range(len(state_seq) - 1):
            transition_matrix[state_seq[i], state_seq[i+1]] += 1
        
        # 归一化
        row_sums = transition_matrix.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1
        transition_matrix = transition_matrix / row_sums
        
        states = ['Sleep', 'Light', 'Stream', 'Game']
        im3 = ax3.imshow(transition_matrix, cmap='Blues')
        
        # 添加数值标注
        for i in range(4):
            for j in range(4):
                text = ax3.text(j, i, f'{transition_matrix[i, j]:.2f}',
                               ha='center', va='center', color='white' if transition_matrix[i, j] > 0.5 else 'black')
        
        ax3.set_xticks(range(4))
        ax3.set_yticks(range(4))
        ax3.set_xticklabels(states, color='white')
        ax3.set_yticklabels(states, color='white')
        ax3.set_xlabel('To State', color='white')
        ax3.set_ylabel('From State', color='white')
        ax3.set_title('State Transition Probability', color='white', fontweight='bold')
        plt.colorbar(im3, ax=ax3, label='Probability')
        
        # 4. 时间段功耗分布
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_facecolor('#16213E')
        
        start_hour = results['start_hour']
        hours = (start_hour + time) % 24
        
        hour_bins = np.arange(0, 25)
        power_by_hour = np.zeros((24, 20))
        
        for h in range(24):
            mask = (hours >= h) & (hours < h + 1)
            if mask.sum() > 0:
                hist, _ = np.histogram(power[mask], bins=20, 
                                      range=(power.min(), power.max()))
                power_by_hour[h] = hist
        
        im4 = ax4.imshow(power_by_hour.T, origin='lower', aspect='auto',
                        extent=[0, 24, power.min(), power.max()], cmap='viridis')
        plt.colorbar(im4, ax=ax4, label='Count')
        ax4.set_xlabel('Hour of Day', color='white')
        ax4.set_ylabel('Power (W)', color='white')
        ax4.set_title('Hourly Power Distribution', color='white', fontweight='bold')
        ax4.tick_params(colors='white')
        ax4.set_xticks(range(0, 25, 4))
        
        plt.suptitle('Heatmap Analysis of System Behavior', 
                    fontsize=14, fontweight='bold', color='white', y=0.98)
        plt.tight_layout()
        
        return fig
    
    def create_pareto_visualization(self, opt_results: Dict, figsize: Tuple = (14, 10)):
        """创建帕累托前沿可视化"""
        fig = plt.figure(figsize=figsize, facecolor='#1A1A2E')
        
        pareto_obj = opt_results['pareto_objectives']
        pareto_sol = opt_results['pareto_solutions']
        
        # 转换为正向目标
        battery_life = -pareto_obj[:, 0]
        temperature = pareto_obj[:, 1]
        performance = -pareto_obj[:, 2]
        
        # 1. 3D帕累托前沿
        ax1 = fig.add_subplot(2, 2, 1, projection='3d', facecolor='#16213E')
        scatter = ax1.scatter(battery_life, temperature, performance,
                             c=performance, cmap='viridis', s=100, alpha=0.8)
        ax1.set_xlabel('Battery Life (h)', color='white')
        ax1.set_ylabel('Avg Temp (°C)', color='white')
        ax1.set_zlabel('Performance', color='white')
        ax1.set_title('Pareto Front 3D', color='white', fontweight='bold')
        ax1.tick_params(colors='white')
        
        # 2. 电池寿命 vs 性能
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_facecolor('#16213E')
        scatter2 = ax2.scatter(battery_life, performance, c=temperature, 
                               cmap='coolwarm', s=100, alpha=0.8)
        plt.colorbar(scatter2, ax=ax2, label='Temperature (°C)')
        ax2.set_xlabel('Battery Life (hours)', color='white')
        ax2.set_ylabel('Performance Score', color='white')
        ax2.set_title('Battery Life vs Performance Trade-off', color='white', fontweight='bold')
        ax2.tick_params(colors='white')
        ax2.grid(True, alpha=0.2, color='white')
        
        # 3. 决策变量分布
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.set_facecolor('#16213E')
        
        labels = ['CPU Scale', 'Brightness', '5G Power']
        x = np.arange(len(labels))
        width = 0.25
        
        # 选择几个代表性解
        n_repr = min(5, len(pareto_sol))
        colors = plt.cm.viridis(np.linspace(0, 1, n_repr))
        
        for i in range(n_repr):
            ax3.bar(x + i * width, pareto_sol[i], width, 
                   color=colors[i], alpha=0.8, label=f'Solution {i+1}')
        
        ax3.set_xticks(x + width * (n_repr - 1) / 2)
        ax3.set_xticklabels(labels, color='white')
        ax3.set_ylabel('Scale Factor', color='white')
        ax3.set_title('Pareto Solution Decision Variables', color='white', fontweight='bold')
        ax3.legend(loc='upper right', facecolor='#16213E', edgecolor='gray', labelcolor='white')
        ax3.tick_params(colors='white')
        
        # 4. 目标函数雷达图
        ax4 = fig.add_subplot(2, 2, 4, projection='polar')
        ax4.set_facecolor('#16213E')
        
        categories = ['Battery\nLife', 'Low\nTemp', 'Performance']
        n_cats = len(categories)
        
        # 归一化目标
        bl_norm = (battery_life - battery_life.min()) / (battery_life.max() - battery_life.min() + 1e-6)
        temp_norm = 1 - (temperature - temperature.min()) / (temperature.max() - temperature.min() + 1e-6)
        perf_norm = (performance - performance.min()) / (performance.max() - performance.min() + 1e-6)
        
        angles = np.linspace(0, 2 * np.pi, n_cats, endpoint=False).tolist()
        angles += angles[:1]
        
        for i in range(min(5, len(pareto_sol))):
            values = [bl_norm[i], temp_norm[i], perf_norm[i]]
            values += values[:1]
            ax4.plot(angles, values, 'o-', linewidth=2, alpha=0.7, label=f'Sol {i+1}')
            ax4.fill(angles, values, alpha=0.1)
        
        ax4.set_xticks(angles[:-1])
        ax4.set_xticklabels(categories, color='white')
        ax4.set_title('Normalized Objective Radar', color='white', fontweight='bold', pad=20)
        ax4.legend(loc='upper right', bbox_to_anchor=(1.3, 1), 
                  facecolor='#16213E', edgecolor='gray', labelcolor='white')
        
        plt.suptitle('Multi-Objective Optimization Results (NSGA-II)', 
                    fontsize=14, fontweight='bold', color='white', y=0.98)
        plt.tight_layout()
        
        return fig

# ============================================================================
# 主程序
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("高级SOC-能耗耦合模型 with 卡尔曼滤波 & 多目标优化")
    print("Advanced SOC-Energy Model with EKF & Multi-Objective Optimization")
    print("=" * 70)
    print()
    
    # 创建系统参数
    system_params = BatterySystemParams(
        Q_max=4000.0,
        V_nominal=3.7,
        T_env=25.0
    )
    
    sensor_noise = SensorNoiseParams(
        sigma_SOC=0.02,
        sigma_V=0.05,
        sigma_T=0.5
    )
    
    # 创建仿真器
    print("1. 初始化系统...")
    simulator = RealisticScenarioSimulator(system_params, sensor_noise)
    
    # 运行仿真
    print("2. 运行24小时真实场景仿真...")
    results = simulator.simulate(
        duration_hours=24,
        start_hour=8,
        initial_SOC=1.0,
        dt_minutes=1.0
    )
    
    print(f"   - 仿真时长: {results['time'][-1]:.1f} 小时")
    print(f"   - 最终SOC: {results['true_state'][-1, 0]*100:.1f}%")
    print(f"   - EKF估计SOC: {results['estimated_state'][-1, 0]*100:.1f}%")
    print(f"   - RMSE: {np.sqrt(np.mean((results['true_state'][:, 0] - results['estimated_state'][:, 0])**2))*100:.3f}%")
    
    # 多目标优化
    print("\n3. 运行多目标优化 (NSGA-II)...")
    
    # 创建一个简单的系统引用用于优化
    class SimpleSystem:
        def __init__(self, params):
            self.battery_params = params
    
    optimizer = MultiObjectiveOptimizer(SimpleSystem(system_params))
    opt_results = optimizer.optimize()
    
    print(f"   - 帕累托最优解数量: {len(opt_results['pareto_solutions'])}")
    
    # 高端可视化
    print("\n4. 生成高端可视化...")
    viz = AdvancedVisualization()
    
    # 仪表盘
    fig1 = viz.create_dashboard(results)
    fig1.savefig('/workspace/battery_soc_model/advanced_dashboard.png', 
                 dpi=150, bbox_inches='tight', facecolor='#1A1A2E')
    print("   - 保存: advanced_dashboard.png")
    
    # 3D可视化
    fig2 = viz.create_3d_visualization(results)
    fig2.savefig('/workspace/battery_soc_model/3d_visualization.png', 
                 dpi=150, bbox_inches='tight', facecolor='#1A1A2E')
    print("   - 保存: 3d_visualization.png")
    
    # 热力图分析
    fig3 = viz.create_heatmap_analysis(results)
    fig3.savefig('/workspace/battery_soc_model/heatmap_analysis.png', 
                 dpi=150, bbox_inches='tight', facecolor='#1A1A2E')
    print("   - 保存: heatmap_analysis.png")
    
    # 帕累托前沿
    fig4 = viz.create_pareto_visualization(opt_results)
    fig4.savefig('/workspace/battery_soc_model/pareto_front.png', 
                 dpi=150, bbox_inches='tight', facecolor='#1A1A2E')
    print("   - 保存: pareto_front.png")
    
    plt.close('all')
    
    print("\n" + "=" * 70)
    print("模型运行完成!")
    print("=" * 70)
