"""
时间非齐次马尔科夫链用户行为模型
Time-Inhomogeneous Markov Chain for User Behavior Modeling

基于b.m文件中的马尔科夫链结果进行时间区块划分:
- 睡眠模式 (23:00 - 7:00)
- 工作模式 (9:00-12:00, 14:00-18:00)
- 休闲模式 (其他时间)

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional
from enum import IntEnum


class UserState(IntEnum):
    """用户状态枚举"""
    DEEP_SLEEP = 0      # 深度睡眠
    LIGHT_USE = 1       # 轻度使用 (社交/消息)
    STREAMING = 2       # 流媒体
    GAMING = 3          # 游戏


class TimeMode(IntEnum):
    """时间模式枚举"""
    SLEEP = 0           # 睡眠模式 (23:00 - 7:00)
    WORK = 1            # 工作模式 (9:00-12:00, 14:00-18:00)
    LEISURE = 2         # 休闲模式 (其他时间)


@dataclass
class HardwareState:
    """硬件状态数据类"""
    apl: float = 0.0                # 屏幕平均像素电平 (0-100)
    brightness: float = 0.0         # 屏幕亮度 (nits)
    cpu_util: float = 0.0           # CPU利用率 (%)
    cpu_freq: float = 0.3           # CPU频率 (GHz)
    refresh_rate: float = 60.0      # 刷新率 (Hz)
    data_rate: float = 0.0          # 数据速率 (Mbps)
    gnss_on: bool = False           # GNSS开启
    bt_audio: bool = False          # 蓝牙音频
    
    def to_dict(self) -> dict:
        """转换为字典格式"""
        return {
            'frequency': self.cpu_freq * 1e9,
            'cpu_load': self.cpu_util / 100.0,
            'brightness': self.brightness,
            'refresh_rate': self.refresh_rate,
            'apl': self.apl,
            'data_rate': self.data_rate,
            'distance': 150.0,
            'snr': 35.0,
            'gnss_on': self.gnss_on,
            'bt_audio': self.bt_audio,
            'bt_interval': 100 if self.bt_audio else 1000
        }


class MarkovUserBehaviorModel:
    """
    时间非齐次马尔科夫链用户行为模型
    
    状态空间:
    - S1: Deep Sleep (深度睡眠)
    - S2: Light Use (轻度使用)
    - S3: Streaming (流媒体)
    - S4: Gaming (游戏)
    
    时间模式:
    - 睡眠模式: 23:00 - 7:00
    - 工作模式: 9:00-12:00, 14:00-18:00
    - 休闲模式: 其他时间
    """
    
    def __init__(self, seed: int = None):
        """
        初始化模型
        
        Parameters:
        -----------
        seed : int, optional
            随机种子
        """
        if seed is not None:
            np.random.seed(seed)
        
        # 状态名称
        self.states = ['Deep Sleep', 'Light Use', 'Streaming', 'Gaming']
        self.num_states = 4
        
        # 转移矩阵定义
        self._init_transition_matrices()
        
        # 硬件参数范围
        self._init_hardware_params()
        
        # 当前状态
        self.current_state = UserState.DEEP_SLEEP
        
        # 历史记录
        self.history = {
            'time': [],
            'state': [],
            'hardware': []
        }
    
    def _init_transition_matrices(self):
        """初始化转移矩阵"""
        # 睡眠模式转移矩阵
        self.P_sleep = np.array([
            [0.995, 0.005, 0.000, 0.000],
            [0.600, 0.400, 0.000, 0.000],
            [0.100, 0.000, 0.900, 0.000],
            [0.100, 0.000, 0.000, 0.900]
        ])
        
        # 工作模式转移矩阵
        self.P_work = np.array([
            [0.850, 0.145, 0.003, 0.002],
            [0.250, 0.700, 0.040, 0.010],
            [0.100, 0.100, 0.800, 0.000],
            [0.200, 0.100, 0.000, 0.700]
        ])
        
        # 休闲模式转移矩阵
        self.P_leisure = np.array([
            [0.800, 0.150, 0.030, 0.020],
            [0.050, 0.650, 0.200, 0.100],
            [0.010, 0.040, 0.940, 0.010],
            [0.010, 0.010, 0.010, 0.970]
        ])
    
    def _init_hardware_params(self):
        """初始化硬件参数范围"""
        # APL范围 [Min, Max] 按状态
        self.P_APL = np.array([
            [0, 0],       # Deep Sleep
            [60, 95],     # Light Use
            [20, 50],     # Streaming
            [40, 75]      # Gaming
        ])
        
        # CPU利用率范围 [Min, Max] 按状态
        self.P_CPU_Util = np.array([
            [0, 2],       # Deep Sleep
            [5, 25],      # Light Use
            [15, 35],     # Streaming
            [70, 95]      # Gaming
        ])
        
        # CPU频率范围 [Min, Max] (GHz) 按状态
        self.P_CPU_Freq = np.array([
            [0.3, 0.3],   # Deep Sleep
            [0.8, 1.8],   # Light Use
            [1.0, 2.0],   # Streaming
            [2.2, 3.0]    # Gaming
        ])
        
        # 刷新率范围
        self.P_Refresh = np.array([
            [1, 1],       # Deep Sleep
            [30, 60],     # Light Use
            [60, 90],     # Streaming
            [90, 120]     # Gaming
        ])
    
    def get_time_mode(self, hour: float) -> TimeMode:
        """
        根据时间获取模式
        
        Parameters:
        -----------
        hour : float
            当前小时 (0-24)
        
        Returns:
        --------
        TimeMode : 时间模式
        """
        hour = hour % 24
        
        if hour >= 23 or hour < 7:
            return TimeMode.SLEEP
        elif (9 <= hour < 12) or (14 <= hour < 18):
            return TimeMode.WORK
        else:
            return TimeMode.LEISURE
    
    def get_transition_matrix(self, time_mode: TimeMode) -> np.ndarray:
        """
        获取对应时间模式的转移矩阵
        
        Parameters:
        -----------
        time_mode : TimeMode
            时间模式
        
        Returns:
        --------
        np.ndarray : 4x4转移矩阵
        """
        if time_mode == TimeMode.SLEEP:
            return self.P_sleep
        elif time_mode == TimeMode.WORK:
            return self.P_work
        else:
            return self.P_leisure
    
    def transition(self, hour: float) -> UserState:
        """
        执行一次状态转移
        
        Parameters:
        -----------
        hour : float
            当前小时
        
        Returns:
        --------
        UserState : 新状态
        """
        time_mode = self.get_time_mode(hour)
        P = self.get_transition_matrix(time_mode)
        
        # 获取当前状态的转移概率
        probs = P[self.current_state]
        
        # 随机选择下一状态
        cumulative_probs = np.cumsum(probs)
        r = np.random.random()
        next_state = np.searchsorted(cumulative_probs, r)
        
        self.current_state = UserState(next_state)
        return self.current_state
    
    def generate_hardware_state(self, hour: float) -> HardwareState:
        """
        根据当前用户状态生成硬件参数
        
        Parameters:
        -----------
        hour : float
            当前小时
        
        Returns:
        --------
        HardwareState : 硬件状态
        """
        s_idx = int(self.current_state)
        time_mode = self.get_time_mode(hour)
        is_leisure = (time_mode == TimeMode.LEISURE)
        
        # 1. APL
        min_a, max_a = self.P_APL[s_idx]
        apl = min_a + (max_a - min_a) * np.random.random()
        
        # 2. 亮度 (基于环境光)
        sunlight_factor = np.exp(-((hour - 13)**2) / (2 * 3**2))
        base_ambient_nits = 100 + 800 * sunlight_factor
        
        if s_idx == 0:  # Deep Sleep
            brightness = 0
        else:
            target_nits = base_ambient_nits * (0.8 + 0.4 * np.random.random())
            brightness = np.clip(target_nits, 150, 1200)
        
        # 3. CPU利用率
        min_u, max_u = self.P_CPU_Util[s_idx]
        cpu_util = min_u + (max_u - min_u) * np.random.random()
        if is_leisure:
            cpu_util = min(cpu_util + 5 * np.random.random(), 100)
        
        # 4. CPU频率
        min_f, max_f = self.P_CPU_Freq[s_idx]
        base_freq = min_f + (max_f - min_f) * np.random.random()
        
        # 微波动 (电路噪声)
        micro_jitter = 0.03 * np.random.randn()
        
        # 休闲时段额外波动
        leisure_jitter = 0.15 * np.random.random() if is_leisure else 0
        
        cpu_freq = np.clip(base_freq + micro_jitter + leisure_jitter, 0.2, 3.2)
        
        # 5. 刷新率
        min_r, max_r = self.P_Refresh[s_idx]
        refresh_rate = min_r + (max_r - min_r) * np.random.random()
        
        # 6. 数据速率
        if s_idx == 0:
            data_rate = 0
        elif s_idx == 1:
            data_rate = np.random.random() * 10
        elif s_idx == 2:
            data_rate = 20 + np.random.random() * 30  # 流媒体
        else:
            data_rate = 50 + np.random.random() * 50  # 游戏
        
        # 7. GNSS和蓝牙
        gnss_on = (s_idx >= 2) and (np.random.random() < 0.3)
        bt_audio = (s_idx == 2) or ((s_idx == 3) and (np.random.random() < 0.5))
        
        return HardwareState(
            apl=apl,
            brightness=brightness,
            cpu_util=cpu_util,
            cpu_freq=cpu_freq,
            refresh_rate=refresh_rate,
            data_rate=data_rate,
            gnss_on=gnss_on,
            bt_audio=bt_audio
        )
    
    def simulate(self, duration_hours: float, 
                 start_hour: float = 0.0,
                 dt_minutes: float = 1.0,
                 initial_state: UserState = None) -> Dict:
        """
        运行用户行为仿真
        
        Parameters:
        -----------
        duration_hours : float
            仿真时长 (小时)
        start_hour : float
            起始时间 (小时)
        dt_minutes : float
            时间步长 (分钟)
        initial_state : UserState, optional
            初始状态
        
        Returns:
        --------
        dict : 仿真结果
        """
        if initial_state is not None:
            self.current_state = initial_state
        
        T_steps = int(duration_hours * 60 / dt_minutes)
        
        # 初始化历史记录
        time_axis = np.linspace(start_hour, start_hour + duration_hours, T_steps)
        states = np.zeros(T_steps, dtype=int)
        hardware_states = []
        
        for t in range(T_steps):
            current_hour = time_axis[t] % 24
            
            # 记录当前状态
            states[t] = int(self.current_state)
            
            # 生成硬件状态
            hw_state = self.generate_hardware_state(current_hour)
            hardware_states.append(hw_state)
            
            # 状态转移
            self.transition(current_hour)
        
        return {
            'time': time_axis,
            'time_seconds': time_axis * 3600,
            'states': states,
            'state_names': [self.states[s] for s in states],
            'hardware': hardware_states
        }
    
    def get_state_function(self, simulation_result: Dict):
        """
        创建状态函数用于耦合模型
        
        Parameters:
        -----------
        simulation_result : dict
            simulate()的返回结果
        
        Returns:
        --------
        callable : 状态函数 f(t) -> dict
        """
        time_seconds = simulation_result['time_seconds']
        hardware_states = simulation_result['hardware']
        
        def state_func(t: float) -> dict:
            # 找到最近的时间点
            idx = np.searchsorted(time_seconds, t)
            idx = min(idx, len(hardware_states) - 1)
            return hardware_states[idx].to_dict()
        
        return state_func
    
    def compute_stationary_distribution(self, time_mode: TimeMode) -> np.ndarray:
        """
        计算稳态分布
        
        Parameters:
        -----------
        time_mode : TimeMode
            时间模式
        
        Returns:
        --------
        np.ndarray : 稳态概率分布
        """
        P = self.get_transition_matrix(time_mode)
        
        # 求解 pi * P = pi, sum(pi) = 1
        # 等价于 (P^T - I) * pi = 0
        n = P.shape[0]
        A = P.T - np.eye(n)
        A[-1, :] = 1  # 归一化约束
        b = np.zeros(n)
        b[-1] = 1
        
        try:
            pi = np.linalg.solve(A, b)
            pi = np.clip(pi, 0, 1)
            pi /= pi.sum()
        except np.linalg.LinAlgError:
            pi = np.ones(n) / n
        
        return pi
    
    def expected_power_by_mode(self) -> Dict[str, float]:
        """
        计算各模式下的期望功耗
        
        Returns:
        --------
        dict : 各模式的期望功耗
        """
        # 简化的功耗模型 (W)
        state_power = np.array([0.5, 2.0, 4.0, 8.0])  # Deep Sleep, Light, Streaming, Gaming
        
        result = {}
        for mode in TimeMode:
            pi = self.compute_stationary_distribution(mode)
            expected_power = np.dot(pi, state_power)
            result[mode.name] = expected_power
        
        return result


class MarkovTimeBlockPredictor:
    """
    基于马尔科夫链的时间区块预测器
    
    用于预测不同时间段的电池消耗
    """
    
    def __init__(self, user_model: MarkovUserBehaviorModel):
        self.user_model = user_model
        
        # 时间区块定义 (小时)
        self.time_blocks = {
            'night_sleep': (23, 7),      # 夜间睡眠
            'morning_leisure': (7, 9),   # 早晨休闲
            'morning_work': (9, 12),     # 上午工作
            'lunch_leisure': (12, 14),   # 午休休闲
            'afternoon_work': (14, 18),  # 下午工作
            'evening_leisure': (18, 23)  # 晚间休闲
        }
    
    def get_block_duration(self, block_name: str) -> float:
        """获取时间区块时长 (小时)"""
        start, end = self.time_blocks[block_name]
        if end < start:  # 跨午夜
            return (24 - start) + end
        return end - start
    
    def predict_block_consumption(self, block_name: str, 
                                   num_samples: int = 100) -> Dict:
        """
        预测某时间区块的电池消耗
        
        Parameters:
        -----------
        block_name : str
            时间区块名称
        num_samples : int
            蒙特卡罗采样数
        
        Returns:
        --------
        dict : 消耗统计
        """
        start_hour, end_hour = self.time_blocks[block_name]
        duration = self.get_block_duration(block_name)
        
        consumption_samples = []
        
        for _ in range(num_samples):
            # 仿真该时间段
            result = self.user_model.simulate(
                duration_hours=duration,
                start_hour=start_hour,
                dt_minutes=1.0
            )
            
            # 计算平均功耗
            state_power = np.array([0.5, 2.0, 4.0, 8.0])
            power_trace = state_power[result['states']]
            avg_power = np.mean(power_trace)
            
            # 能量消耗 (Wh)
            energy = avg_power * duration
            consumption_samples.append(energy)
        
        consumption_samples = np.array(consumption_samples)
        
        return {
            'block_name': block_name,
            'duration_hours': duration,
            'mean_energy_wh': np.mean(consumption_samples),
            'std_energy_wh': np.std(consumption_samples),
            'min_energy_wh': np.min(consumption_samples),
            'max_energy_wh': np.max(consumption_samples),
            'percentile_25': np.percentile(consumption_samples, 25),
            'percentile_75': np.percentile(consumption_samples, 75)
        }
    
    def predict_daily_consumption(self, num_samples: int = 100) -> Dict:
        """
        预测24小时电池消耗
        
        Returns:
        --------
        dict : 每日消耗统计
        """
        daily_results = {}
        total_energy = np.zeros(num_samples)
        
        for block_name in self.time_blocks:
            block_result = self.predict_block_consumption(block_name, num_samples)
            daily_results[block_name] = block_result
            # 累加 (简化处理,实际应该保持样本对应关系)
            total_energy += block_result['mean_energy_wh']
        
        daily_results['total_daily'] = {
            'mean_energy_wh': np.sum([r['mean_energy_wh'] for r in daily_results.values() if isinstance(r, dict) and 'mean_energy_wh' in r]),
            'description': 'Total daily energy consumption'
        }
        
        return daily_results


if __name__ == "__main__":
    # 测试用户行为模型
    model = MarkovUserBehaviorModel(seed=42)
    
    # 仿真24小时
    result = model.simulate(duration_hours=24, start_hour=0)
    
    print("用户行为仿真结果:")
    print(f"总时间步: {len(result['time'])}")
    
    # 统计各状态占比
    states = result['states']
    for i, name in enumerate(model.states):
        ratio = np.mean(states == i)
        print(f"  {name}: {ratio:.1%}")
    
    # 计算各模式期望功耗
    expected_power = model.expected_power_by_mode()
    print("\n各模式期望功耗:")
    for mode, power in expected_power.items():
        print(f"  {mode}: {power:.2f} W")
    
    # 时间区块预测
    predictor = MarkovTimeBlockPredictor(model)
    daily_consumption = predictor.predict_daily_consumption(num_samples=50)
    
    print("\n每日能量消耗预测:")
    for block_name, stats in daily_consumption.items():
        if isinstance(stats, dict) and 'mean_energy_wh' in stats:
            print(f"  {block_name}: {stats['mean_energy_wh']:.2f} Wh")
