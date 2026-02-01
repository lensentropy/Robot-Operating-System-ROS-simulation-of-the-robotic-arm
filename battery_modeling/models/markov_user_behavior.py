"""
Time-Inhomogeneous Markov Chain Model for User Behavior

Based on the MCM 2026 Problem A model structure:
- 4 States: Deep Sleep, Light Use, Streaming, Gaming
- 3 Transition Modes: Sleep, Work, Leisure
- Time partitioning based on daily schedule

This module provides:
1. Markov chain state transition dynamics
2. Hardware parameter generation based on state
3. Differential equation formulation for power analysis
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, List, Dict, Optional
from scipy.integrate import odeint
from scipy.linalg import expm
import warnings


# =============================================================================
# State and Parameter Definitions
# =============================================================================

@dataclass
class MarkovStates:
    """User behavior states"""
    DEEP_SLEEP = 0
    LIGHT_USE = 1
    STREAMING = 2
    GAMING = 3
    
    names = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
    num_states = 4


@dataclass 
class HardwareParameters:
    """Hardware parameter ranges for each state [Min, Max]"""
    # Average Pixel Level (APL) for OLED display (%)
    APL: np.ndarray = field(default_factory=lambda: np.array([
        [0, 0],      # Deep Sleep - screen off
        [60, 95],    # Light Use - social media, messaging
        [20, 50],    # Streaming - video content (darker)
        [40, 75]     # Gaming - varied content
    ]))
    
    # CPU Utilization (%)
    CPU_Util: np.ndarray = field(default_factory=lambda: np.array([
        [0, 2],      # Deep Sleep - minimal background
        [5, 25],     # Light Use - light apps
        [15, 35],    # Streaming - decoding
        [70, 95]     # Gaming - heavy computation
    ]))
    
    # CPU Frequency (GHz)
    CPU_Freq: np.ndarray = field(default_factory=lambda: np.array([
        [0.3, 0.3],  # Deep Sleep - idle frequency
        [0.8, 1.8],  # Light Use - moderate DVFS
        [1.0, 2.0],  # Streaming - sustained
        [2.2, 3.0]   # Gaming - high performance
    ]))
    
    # 5G Data Rate (Mbps)
    Data_Rate: np.ndarray = field(default_factory=lambda: np.array([
        [0, 0.1],    # Deep Sleep - sync only
        [5, 50],     # Light Use - browsing
        [20, 100],   # Streaming - video
        [10, 150]    # Gaming - online gaming
    ]))


# =============================================================================
# Time Partition Definitions
# =============================================================================

@dataclass
class TimePartition:
    """
    24-hour day partitioned into behavioral modes
    
    时间分区（基于马尔科夫链模式切换）:
    - Sleep Mode (睡眠模式): 23:00 - 07:00 (8 hours)
    - Work Mode (工作模式): 09:00-12:00, 14:00-18:00 (7 hours)
    - Leisure Mode (休闲模式): 07:00-09:00, 12:00-14:00, 18:00-23:00 (9 hours)
    """
    
    # Time boundaries (hours)
    sleep_start: float = 23.0
    sleep_end: float = 7.0
    
    work_periods: List[Tuple[float, float]] = field(default_factory=lambda: [
        (9.0, 12.0),   # Morning work
        (14.0, 18.0)   # Afternoon work
    ])
    
    leisure_periods: List[Tuple[float, float]] = field(default_factory=lambda: [
        (7.0, 9.0),    # Morning leisure
        (12.0, 14.0),  # Lunch break
        (18.0, 23.0)   # Evening leisure
    ])
    
    def get_mode(self, hour: float) -> str:
        """Determine behavioral mode for given hour"""
        hour = hour % 24
        
        # Sleep mode: 23:00 - 07:00
        if hour >= self.sleep_start or hour < self.sleep_end:
            return 'sleep'
        
        # Work mode
        for start, end in self.work_periods:
            if start <= hour < end:
                return 'work'
        
        # Default to leisure
        return 'leisure'
    
    def get_mode_duration(self, mode: str) -> float:
        """Get total duration of a mode in hours"""
        if mode == 'sleep':
            return (24 - self.sleep_start) + self.sleep_end  # 8 hours
        elif mode == 'work':
            return sum(end - start for start, end in self.work_periods)  # 7 hours
        else:  # leisure
            return sum(end - start for start, end in self.leisure_periods)  # 9 hours


# =============================================================================
# Transition Matrices (Generator Matrices for Continuous-Time)
# =============================================================================

class TransitionMatrices:
    """
    Transition probability matrices for each mode
    
    离散时间马尔科夫链转移概率矩阵:
    P[i,j] = P(X_{t+1} = j | X_t = i)
    
    对于微分方程推导，需要转换为连续时间生成矩阵 Q:
    Q = (P - I) / Δt
    其中 dπ/dt = π·Q (概率分布演化方程)
    """
    
    def __init__(self, dt_minutes: float = 1.0):
        self.dt = dt_minutes / 60  # Convert to hours
        
        # --- Mode A: Sleep Mode (睡眠模式) ---
        # High probability of staying in deep sleep
        self.P_sleep = np.array([
            [0.995, 0.005, 0.000, 0.000],  # Deep Sleep: very stable
            [0.600, 0.400, 0.000, 0.000],  # Light Use: tends to sleep
            [0.100, 0.000, 0.900, 0.000],  # Streaming: unlikely at night
            [0.100, 0.000, 0.000, 0.900]   # Gaming: unlikely at night
        ])
        
        # --- Mode B: Work Mode (工作模式) ---
        # Moderate activity, focused usage
        self.P_work = np.array([
            [0.850, 0.145, 0.003, 0.002],  # Deep Sleep: quick wake
            [0.250, 0.700, 0.040, 0.010],  # Light Use: main work state
            [0.100, 0.100, 0.800, 0.000],  # Streaming: work breaks
            [0.200, 0.100, 0.000, 0.700]   # Gaming: rare at work
        ])
        
        # --- Mode C: Leisure Mode (休闲模式) ---
        # High entertainment activity
        self.P_leisure = np.array([
            [0.800, 0.150, 0.030, 0.020],  # Deep Sleep: occasional rest
            [0.050, 0.650, 0.200, 0.100],  # Light Use: social media
            [0.010, 0.040, 0.940, 0.010],  # Streaming: binge watching
            [0.010, 0.010, 0.010, 0.970]   # Gaming: heavy engagement
        ])
        
    def get_transition_matrix(self, mode: str) -> np.ndarray:
        """Get transition matrix for specified mode"""
        if mode == 'sleep':
            return self.P_sleep
        elif mode == 'work':
            return self.P_work
        else:
            return self.P_leisure
    
    def get_generator_matrix(self, mode: str) -> np.ndarray:
        """
        Convert discrete transition matrix to continuous-time generator matrix
        
        连续时间生成矩阵 Q 的推导:
        对于离散时间步长 Δt，转移概率矩阵 P 对应的连续时间生成矩阵为:
        Q = ln(P) / Δt  (矩阵对数)
        
        简化近似（当 Δt 较小时）:
        Q ≈ (P - I) / Δt
        
        生成矩阵性质:
        - 对角元素 q_ii < 0 (离开状态 i 的速率)
        - 非对角元素 q_ij ≥ 0 (从 i 到 j 的转移速率)
        - 每行和为 0: Σ_j q_ij = 0
        """
        P = self.get_transition_matrix(mode)
        I = np.eye(4)
        Q = (P - I) / self.dt
        return Q
    
    def get_stationary_distribution(self, mode: str) -> np.ndarray:
        """
        Calculate stationary distribution π* for given mode
        
        稳态分布满足: π* · P = π*
        即 π* 是转移矩阵 P 对应特征值 1 的左特征向量
        
        等价地，对于生成矩阵: π* · Q = 0
        """
        P = self.get_transition_matrix(mode)
        
        # Solve (P^T - I) * π = 0 with constraint sum(π) = 1
        n = P.shape[0]
        A = np.vstack([P.T - np.eye(n), np.ones(n)])
        b = np.zeros(n + 1)
        b[-1] = 1
        
        # Least squares solution
        pi_star, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        pi_star = np.maximum(pi_star, 0)  # Ensure non-negative
        pi_star /= np.sum(pi_star)  # Normalize
        
        return pi_star


# =============================================================================
# Differential Equation Formulation
# =============================================================================

class MarkovDifferentialEquations:
    """
    Differential equation system for Markov chain evolution
    
    概率分布演化方程 (Kolmogorov Forward Equation):
    dπ(t)/dt = π(t) · Q(t)
    
    其中:
    - π(t) = [π_1(t), π_2(t), π_3(t), π_4(t)] 是状态概率分布
    - Q(t) 是时变生成矩阵（依赖于当前时间的模式）
    
    期望功率演化方程:
    E[P(t)] = Σ_i π_i(t) · P_i
    dE[P]/dt = Σ_i (dπ_i/dt) · P_i
    """
    
    def __init__(self):
        self.transition = TransitionMatrices()
        self.partition = TimePartition()
        self.hw_params = HardwareParameters()
        
        # Power consumption for each state (mW)
        # These are derived from the hardware parameters
        self.state_power = self._calculate_state_power()
        
    def _calculate_state_power(self) -> np.ndarray:
        """
        Calculate average power consumption for each state
        
        状态功率模型:
        P_state = P_display + P_cpu + P_network + P_base
        
        P_display = f(APL, brightness)
        P_cpu = f(frequency, utilization)
        P_network = f(data_rate)
        """
        power = np.zeros(4)
        
        # State 0: Deep Sleep
        power[0] = 50  # mW (base standby)
        
        # State 1: Light Use
        power[1] = 800  # mW (screen on, light CPU)
        
        # State 2: Streaming
        power[2] = 1500  # mW (video decode, network)
        
        # State 3: Gaming
        power[3] = 4000  # mW (high CPU, GPU, screen)
        
        return power
    
    def probability_evolution_ode(self, pi: np.ndarray, t: float, 
                                   start_hour: float = 0) -> np.ndarray:
        """
        Kolmogorov forward equation for probability evolution
        
        dπ/dt = π · Q(t)
        
        Args:
            pi: Current probability distribution [π_0, π_1, π_2, π_3]
            t: Time in hours since start
            start_hour: Starting hour of day
        Returns:
            dπ/dt: Time derivative of probability distribution
        """
        current_hour = (start_hour + t) % 24
        mode = self.partition.get_mode(current_hour)
        Q = self.transition.get_generator_matrix(mode)
        
        # dπ/dt = π · Q
        dpidt = np.dot(pi, Q)
        return dpidt
    
    def solve_probability_evolution(self, 
                                     pi0: np.ndarray,
                                     t_span: Tuple[float, float],
                                     start_hour: float = 0,
                                     num_points: int = 1440) -> Dict:
        """
        Solve probability evolution ODE over time span
        
        求解概率分布演化微分方程组:
        dπ/dt = π · Q(t)
        初始条件: π(0) = π_0
        
        Args:
            pi0: Initial probability distribution
            t_span: Time span (start, end) in hours
            start_hour: Starting hour of day
            num_points: Number of time points
        Returns:
            Dictionary with time, probabilities, and derived quantities
        """
        t_eval = np.linspace(t_span[0], t_span[1], num_points)
        
        # Solve ODE
        solution = odeint(
            self.probability_evolution_ode,
            pi0,
            t_eval,
            args=(start_hour,)
        )
        
        # Ensure valid probabilities (non-negative, sum to 1)
        solution = np.maximum(solution, 0)
        solution = solution / solution.sum(axis=1, keepdims=True)
        
        # Calculate expected power
        expected_power = np.dot(solution, self.state_power)
        
        # Calculate mode schedule
        modes = [self.partition.get_mode((start_hour + t) % 24) for t in t_eval]
        
        return {
            'time': t_eval,
            'hour': (start_hour + t_eval) % 24,
            'probabilities': solution,
            'expected_power': expected_power,
            'modes': modes
        }
    
    def power_evolution_ode(self, state: np.ndarray, t: float,
                            start_hour: float = 0) -> np.ndarray:
        """
        Coupled ODE system for power and SOC evolution
        
        耦合微分方程组:
        1. dπ/dt = π · Q(t)           (概率演化)
        2. dSOC/dt = -E[P(t)] / (V·Q) (电量消耗)
        3. dT/dt = (Q_gen - Q_diss) / C (温度演化)
        
        其中:
        - E[P(t)] = Σ_i π_i(t) · P_i 是期望功率
        - Q_gen = E[P(t)] · η 是产热率
        - Q_diss = (T - T_env) / R_th 是散热率
        
        State vector: [π_0, π_1, π_2, π_3, SOC, T]
        """
        # Unpack state
        pi = state[:4]
        SOC = state[4]
        T = state[5]
        
        # Parameters
        V_bat = 3.7  # Battery voltage (V)
        Q_bat = 4.5  # Battery capacity (Ah)
        C_th = 10.0  # Thermal capacity (J/K)
        R_th = 5.0   # Thermal resistance (K/W)
        T_env = 25.0 # Environment temperature (°C)
        eta_heat = 0.3  # Heat generation efficiency
        
        # Get current mode
        current_hour = (start_hour + t) % 24
        mode = self.partition.get_mode(current_hour)
        Q_matrix = self.transition.get_generator_matrix(mode)
        
        # 1. Probability evolution: dπ/dt = π · Q
        pi = np.maximum(pi, 0)
        pi = pi / (np.sum(pi) + 1e-10)
        dpidt = np.dot(pi, Q_matrix)
        
        # 2. Expected power
        E_power = np.dot(pi, self.state_power) / 1000  # Convert to W
        
        # 3. SOC evolution: dSOC/dt = -P / (V·Q)
        # Ensure SOC doesn't go negative
        if SOC > 0:
            dSOCdt = -E_power / (V_bat * Q_bat)
        else:
            dSOCdt = 0
        
        # 4. Temperature evolution
        Q_gen = E_power * eta_heat
        Q_diss = (T - T_env) / R_th
        dTdt = (Q_gen - Q_diss) / C_th
        
        return np.concatenate([dpidt, [dSOCdt, dTdt]])
    
    def solve_coupled_system(self,
                              pi0: np.ndarray,
                              SOC0: float = 1.0,
                              T0: float = 25.0,
                              t_span: Tuple[float, float] = (0, 24),
                              start_hour: float = 0,
                              num_points: int = 1440) -> Dict:
        """
        Solve coupled power-thermal-Markov system
        
        求解耦合系统:
        状态向量: [π_0, π_1, π_2, π_3, SOC, T]
        
        微分方程组:
        dπ/dt = π · Q(t)
        dSOC/dt = -E[P]/（V·Q）
        dT/dt = (η·E[P] - (T-T_env)/R) / C
        """
        # Initial state
        state0 = np.concatenate([pi0, [SOC0, T0]])
        
        t_eval = np.linspace(t_span[0], t_span[1], num_points)
        
        # Solve ODE
        solution = odeint(
            self.power_evolution_ode,
            state0,
            t_eval,
            args=(start_hour,)
        )
        
        # Extract components
        probabilities = solution[:, :4]
        probabilities = np.maximum(probabilities, 0)
        probabilities = probabilities / probabilities.sum(axis=1, keepdims=True)
        
        SOC = np.clip(solution[:, 4], 0, 1)
        temperature = solution[:, 5]
        
        # Calculate expected power
        expected_power = np.dot(probabilities, self.state_power)
        
        # Mode schedule
        modes = [self.partition.get_mode((start_hour + t) % 24) for t in t_eval]
        
        return {
            'time': t_eval,
            'hour': (start_hour + t_eval) % 24,
            'probabilities': probabilities,
            'SOC': SOC,
            'temperature': temperature,
            'expected_power': expected_power,
            'modes': modes
        }


# =============================================================================
# Markov Chain Simulator (Monte Carlo)
# =============================================================================

class MarkovChainSimulator:
    """
    Monte Carlo simulation of user behavior
    
    蒙特卡洛仿真:
    在每个时间步，根据当前状态和转移概率进行随机状态转移
    同时生成对应的硬件参数
    """
    
    def __init__(self, dt_minutes: float = 1.0, seed: Optional[int] = None):
        self.dt = dt_minutes
        self.transition = TransitionMatrices(dt_minutes)
        self.partition = TimePartition()
        self.hw_params = HardwareParameters()
        
        if seed is not None:
            np.random.seed(seed)
    
    def simulate(self, 
                 duration_hours: float = 24,
                 start_hour: float = 0,
                 initial_state: int = 0) -> Dict:
        """
        Run Monte Carlo simulation
        
        Args:
            duration_hours: Simulation duration
            start_hour: Starting hour of day
            initial_state: Initial user state
        Returns:
            Dictionary with simulation history
        """
        num_steps = int(duration_hours * 60 / self.dt)
        
        # Initialize history
        history = {
            'state': np.zeros(num_steps, dtype=int),
            'apl': np.zeros(num_steps),
            'brightness': np.zeros(num_steps),
            'cpu_util': np.zeros(num_steps),
            'cpu_freq': np.zeros(num_steps),
            'data_rate': np.zeros(num_steps),
            'mode': [],
            'power': np.zeros(num_steps)
        }
        
        time_axis = np.linspace(0, duration_hours, num_steps)
        current_state = initial_state
        
        for t in range(num_steps):
            # Current time of day
            current_hour = (start_hour + time_axis[t]) % 24
            
            # Determine mode
            mode = self.partition.get_mode(current_hour)
            history['mode'].append(mode)
            
            # Get transition matrix
            P = self.transition.get_transition_matrix(mode)
            
            # Record current state
            history['state'][t] = current_state
            
            # State transition
            probs = P[current_state, :]
            next_state = np.random.choice(4, p=probs)
            
            # Generate hardware parameters
            s = current_state
            
            # APL
            min_a, max_a = self.hw_params.APL[s]
            history['apl'][t] = min_a + (max_a - min_a) * np.random.rand()
            
            # Brightness (with sunlight model)
            sunlight_factor = np.exp(-((current_hour - 13)**2) / (2 * 3**2))
            base_ambient = 100 + 800 * sunlight_factor
            
            if s == 0:  # Deep Sleep
                history['brightness'][t] = 0
            else:
                target_nits = base_ambient * (0.8 + 0.4 * np.random.rand())
                history['brightness'][t] = np.clip(target_nits, 150, 1200)
            
            # CPU Utilization
            min_u, max_u = self.hw_params.CPU_Util[s]
            base_util = min_u + (max_u - min_u) * np.random.rand()
            if mode == 'leisure':
                base_util += 5 * np.random.rand()
            history['cpu_util'][t] = np.clip(base_util, 0, 100)
            
            # CPU Frequency (with micro-jitter)
            min_f, max_f = self.hw_params.CPU_Freq[s]
            base_freq = min_f + (max_f - min_f) * np.random.rand()
            micro_jitter = 0.03 * np.random.randn()
            leisure_jitter = 0.15 * np.random.rand() if mode == 'leisure' else 0
            history['cpu_freq'][t] = np.clip(base_freq + micro_jitter + leisure_jitter, 0.2, 3.2)
            
            # Data Rate
            min_r, max_r = self.hw_params.Data_Rate[s]
            history['data_rate'][t] = min_r + (max_r - min_r) * np.random.rand()
            
            # Estimate power
            history['power'][t] = self._estimate_power(
                history['apl'][t],
                history['brightness'][t],
                history['cpu_freq'][t],
                history['cpu_util'][t],
                history['data_rate'][t]
            )
            
            current_state = next_state
        
        history['time'] = time_axis
        history['hour'] = (start_hour + time_axis) % 24
        
        return history
    
    def _estimate_power(self, apl, brightness, freq, util, data_rate) -> float:
        """Estimate instantaneous power consumption (mW)"""
        # Display power
        if brightness == 0:
            P_display = 5  # Minimal standby
        else:
            P_display = 65 + 1.25 * 60 + 3.8 * (brightness / 1000) * (apl / 100) * brightness
        
        # CPU power (simplified DVFS model)
        P_cpu = 150 + 2500 * (freq / 3.0)**2.5 * (util / 100)
        
        # Network power
        if data_rate < 1:
            P_network = 50
        else:
            P_network = 200 + 20 * np.log2(1 + data_rate / 10)
        
        return P_display + P_cpu + P_network


# =============================================================================
# Analysis Functions
# =============================================================================

def analyze_stationary_distributions():
    """
    分析各模式下的稳态分布
    
    稳态分布 π* 满足 π* · P = π*
    物理意义：长时间运行后，系统处于各状态的概率趋于稳定值
    """
    tm = TransitionMatrices()
    
    results = {}
    for mode in ['sleep', 'work', 'leisure']:
        pi_star = tm.get_stationary_distribution(mode)
        results[mode] = {
            'distribution': pi_star,
            'expected_state': np.dot(pi_star, np.arange(4)),
            'entropy': -np.sum(pi_star * np.log(pi_star + 1e-10))
        }
        
    return results


def analyze_mode_transitions():
    """
    分析模式切换时的瞬态响应
    
    当从一个模式切换到另一个模式时，概率分布需要时间从旧稳态过渡到新稳态
    松弛时间 τ ≈ 1 / |λ_2| 其中 λ_2 是生成矩阵的第二大特征值
    """
    tm = TransitionMatrices()
    
    results = {}
    for mode in ['sleep', 'work', 'leisure']:
        Q = tm.get_generator_matrix(mode)
        eigenvalues = np.linalg.eigvals(Q)
        eigenvalues_sorted = np.sort(np.real(eigenvalues))[::-1]
        
        # Second largest eigenvalue determines relaxation time
        if len(eigenvalues_sorted) > 1 and eigenvalues_sorted[1] < 0:
            tau = -1.0 / eigenvalues_sorted[1]
        else:
            tau = np.inf
            
        results[mode] = {
            'eigenvalues': eigenvalues_sorted,
            'relaxation_time_hours': tau,
            'relaxation_time_minutes': tau * 60
        }
        
    return results


if __name__ == "__main__":
    print("=" * 60)
    print("Markov Chain User Behavior Model Analysis")
    print("=" * 60)
    
    # Analyze stationary distributions
    print("\n1. Stationary Distributions:")
    stat_dist = analyze_stationary_distributions()
    states = MarkovStates.names
    
    for mode, data in stat_dist.items():
        print(f"\n  {mode.upper()} Mode:")
        for i, (name, prob) in enumerate(zip(states, data['distribution'])):
            print(f"    {name}: {prob:.3f} ({prob*100:.1f}%)")
    
    # Analyze relaxation times
    print("\n2. Relaxation Times (mode transition dynamics):")
    relax = analyze_mode_transitions()
    for mode, data in relax.items():
        print(f"  {mode}: τ = {data['relaxation_time_minutes']:.1f} minutes")
    
    # Run coupled system simulation
    print("\n3. Coupled System ODE Solution:")
    ode_system = MarkovDifferentialEquations()
    
    # Initial distribution (start in Deep Sleep at midnight)
    pi0 = np.array([1.0, 0.0, 0.0, 0.0])
    
    result = ode_system.solve_coupled_system(
        pi0=pi0,
        SOC0=1.0,
        T0=25.0,
        t_span=(0, 24),
        start_hour=0
    )
    
    print(f"  Final SOC: {result['SOC'][-1]*100:.1f}%")
    print(f"  Final Temperature: {result['temperature'][-1]:.1f}°C")
    print(f"  Average Power: {np.mean(result['expected_power']):.0f} mW")
    
    # Run Monte Carlo simulation
    print("\n4. Monte Carlo Simulation (single trajectory):")
    simulator = MarkovChainSimulator(seed=42)
    mc_result = simulator.simulate(duration_hours=24, start_hour=0, initial_state=0)
    
    print(f"  Average Power: {np.mean(mc_result['power']):.0f} mW")
    print(f"  Time in each state:")
    for i, name in enumerate(states):
        time_frac = np.mean(mc_result['state'] == i)
        print(f"    {name}: {time_frac*100:.1f}%")
