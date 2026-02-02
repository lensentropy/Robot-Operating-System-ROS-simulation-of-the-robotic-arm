"""
剩余使用时间预测模块
Remaining Battery Usage Time Prediction

集成:
1. SOC-能耗耦合连续时间模型
2. 时间非齐次马尔科夫链用户行为模型
3. 扩展卡尔曼滤波SOC估计
4. 多目标优化

基于马尔科夫链结果进行时间区块划分

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional
from scipy.integrate import solve_ivp
import warnings

from soc_energy_coupled_model import CoupledSOCEnergyModel, create_default_model
from markov_user_behavior import MarkovUserBehaviorModel, MarkovTimeBlockPredictor, TimeMode, UserState
from kalman_filter_soc import ExtendedKalmanFilter, UnscentedKalmanFilter, EKFParameters
from multi_objective_optimizer import WeightedSumOptimizer, NSGA2Optimizer, BatteryLifePredictor


@dataclass
class PredictionConfig:
    """预测配置"""
    # 电池参数
    battery_capacity_mah: float = 4000.0
    battery_voltage_nominal: float = 3.85
    
    # SOC阈值
    soc_min_threshold: float = 0.05
    soc_warning_threshold: float = 0.20
    
    # 预测参数
    prediction_horizon_hours: float = 48.0
    monte_carlo_samples: int = 100
    
    # 卡尔曼滤波参数
    use_kalman_filter: bool = True
    kalman_type: str = 'ekf'  # 'ekf' or 'ukf'
    
    # 时间区块 (小时)
    time_blocks: Dict = None
    
    def __post_init__(self):
        if self.time_blocks is None:
            self.time_blocks = {
                'night_sleep': (23, 7),      # 夜间睡眠
                'morning_commute': (7, 9),   # 早晨通勤
                'morning_work': (9, 12),     # 上午工作
                'lunch_break': (12, 14),     # 午休
                'afternoon_work': (14, 18),  # 下午工作
                'evening_leisure': (18, 23)  # 晚间休闲
            }


class IntegratedRemainingTimePredictor:
    """
    集成剩余时间预测器
    
    将SOC-能耗耦合模型、用户行为马尔科夫链、卡尔曼滤波和多目标优化
    整合为统一的预测系统
    """
    
    def __init__(self, config: PredictionConfig = None):
        """
        初始化预测器
        
        Parameters:
        -----------
        config : PredictionConfig
            预测配置
        """
        self.config = config or PredictionConfig()
        
        # 初始化子模块
        self.soc_model = create_default_model()
        self.user_model = MarkovUserBehaviorModel()
        self.time_block_predictor = MarkovTimeBlockPredictor(self.user_model)
        
        # 卡尔曼滤波器
        ekf_params = EKFParameters()
        if self.config.kalman_type == 'ukf':
            self.kalman_filter = UnscentedKalmanFilter(
                battery_capacity_mah=self.config.battery_capacity_mah,
                params=ekf_params
            )
        else:
            self.kalman_filter = ExtendedKalmanFilter(
                battery_capacity_mah=self.config.battery_capacity_mah,
                params=ekf_params
            )
        
        # 多目标优化器
        self.optimizer = BatteryLifePredictor()
        
        # 状态
        self.current_soc = 1.0
        self.current_temperature = 298.15
        self.soc_history = []
        self.prediction_history = []
    
    def initialize(self, soc_init: float = 1.0, 
                   temp_init: float = 298.15):
        """
        初始化预测器状态
        
        Parameters:
        -----------
        soc_init : float
            初始SOC
        temp_init : float
            初始温度 (K)
        """
        self.current_soc = soc_init
        self.current_temperature = temp_init
        
        self.kalman_filter.initialize(
            soc_init=soc_init,
            temp_init=temp_init,
            rint_init=0.08
        )
        
        self.soc_history = [soc_init]
        self.prediction_history = []
    
    def update_soc_estimate(self, V_measured: float, 
                            I_measured: float, 
                            dt: float):
        """
        使用卡尔曼滤波更新SOC估计
        
        Parameters:
        -----------
        V_measured : float
            测量电压 (V)
        I_measured : float
            测量电流 (A)
        dt : float
            时间步长 (s)
        """
        if self.config.use_kalman_filter:
            self.kalman_filter.step(V_measured, I_measured, dt)
            self.current_soc, _ = self.kalman_filter.get_soc_estimate()
            
            state = self.kalman_filter.get_state_estimate()
            self.current_temperature = state['temperature']
        else:
            # 简单库仑计数
            Q_ah = self.config.battery_capacity_mah / 1000
            dSOC = -I_measured * dt / (Q_ah * 3600)
            self.current_soc = np.clip(self.current_soc + dSOC, 0, 1)
        
        self.soc_history.append(self.current_soc)
    
    def get_time_block_info(self, current_hour: float) -> Dict:
        """
        获取当前时间区块信息
        
        Parameters:
        -----------
        current_hour : float
            当前小时 (0-24)
        
        Returns:
        --------
        dict : 时间区块信息
        """
        current_hour = current_hour % 24
        
        for block_name, (start, end) in self.config.time_blocks.items():
            if start <= end:
                if start <= current_hour < end:
                    time_remaining = end - current_hour
                    return {
                        'block_name': block_name,
                        'hours_remaining_in_block': time_remaining,
                        'is_sleep': 'sleep' in block_name.lower(),
                        'is_work': 'work' in block_name.lower()
                    }
            else:  # 跨午夜
                if current_hour >= start or current_hour < end:
                    if current_hour >= start:
                        time_remaining = (24 - current_hour) + end
                    else:
                        time_remaining = end - current_hour
                    return {
                        'block_name': block_name,
                        'hours_remaining_in_block': time_remaining,
                        'is_sleep': 'sleep' in block_name.lower(),
                        'is_work': 'work' in block_name.lower()
                    }
        
        return {
            'block_name': 'unknown',
            'hours_remaining_in_block': 1.0,
            'is_sleep': False,
            'is_work': False
        }
    
    def simulate_future_soc(self, current_hour: float,
                            prediction_hours: float = 24) -> Dict:
        """
        仿真未来SOC变化
        
        使用马尔科夫链用户行为模型
        
        Parameters:
        -----------
        current_hour : float
            当前小时
        prediction_hours : float
            预测时长 (小时)
        
        Returns:
        --------
        dict : 仿真结果
        """
        # 生成用户行为轨迹
        user_sim = self.user_model.simulate(
            duration_hours=prediction_hours,
            start_hour=current_hour,
            dt_minutes=1.0
        )
        
        # 获取状态函数
        state_func = self.user_model.get_state_function(user_sim)
        
        # 运行耦合模型仿真
        t_span = (0, prediction_hours * 3600)
        y0 = np.array([self.current_soc, self.current_temperature])
        
        result = self.soc_model.simulate(
            t_span=t_span,
            y0=y0,
            state_func=state_func,
            t_eval=np.linspace(0, prediction_hours * 3600, int(prediction_hours * 60))
        )
        
        return {
            'time_hours': result['time'] / 3600,
            'soc': result['soc'],
            'temperature': result['temperature'],
            'power': result['power_total'],
            'user_states': user_sim['states']
        }
    
    def predict_remaining_time_deterministic(self, current_hour: float) -> Dict:
        """
        确定性剩余时间预测
        
        Parameters:
        -----------
        current_hour : float
            当前小时
        
        Returns:
        --------
        dict : 预测结果
        """
        # 仿真直到SOC达到阈值
        sim_result = self.simulate_future_soc(current_hour, self.config.prediction_horizon_hours)
        
        soc_trace = sim_result['soc']
        time_hours = sim_result['time_hours']
        
        # 找到SOC降至阈值的时间
        threshold_idx = np.where(soc_trace <= self.config.soc_min_threshold)[0]
        
        if len(threshold_idx) > 0:
            remaining_hours = time_hours[threshold_idx[0]]
        else:
            # SOC在预测期内未降至阈值
            remaining_hours = self.config.prediction_horizon_hours
        
        # 找到警告阈值时间
        warning_idx = np.where(soc_trace <= self.config.soc_warning_threshold)[0]
        warning_hours = time_hours[warning_idx[0]] if len(warning_idx) > 0 else remaining_hours
        
        return {
            'remaining_hours': remaining_hours,
            'warning_hours': warning_hours,
            'soc_trace': soc_trace,
            'time_trace': time_hours,
            'final_soc': soc_trace[-1],
            'average_power': np.mean(sim_result['power'])
        }
    
    def predict_remaining_time_probabilistic(self, current_hour: float,
                                              n_samples: int = None) -> Dict:
        """
        概率性剩余时间预测 (蒙特卡罗)
        
        Parameters:
        -----------
        current_hour : float
            当前小时
        n_samples : int
            蒙特卡罗样本数
        
        Returns:
        --------
        dict : 预测结果统计
        """
        n_samples = n_samples or self.config.monte_carlo_samples
        
        remaining_times = []
        warning_times = []
        final_socs = []
        
        for i in range(n_samples):
            # 每次运行重置用户模型以获得不同轨迹
            self.user_model = MarkovUserBehaviorModel(seed=None)  # 随机种子
            
            result = self.predict_remaining_time_deterministic(current_hour)
            remaining_times.append(result['remaining_hours'])
            warning_times.append(result['warning_hours'])
            final_socs.append(result['final_soc'])
        
        remaining_times = np.array(remaining_times)
        warning_times = np.array(warning_times)
        
        return {
            'remaining_hours_mean': np.mean(remaining_times),
            'remaining_hours_std': np.std(remaining_times),
            'remaining_hours_median': np.median(remaining_times),
            'remaining_hours_p10': np.percentile(remaining_times, 10),
            'remaining_hours_p90': np.percentile(remaining_times, 90),
            'warning_hours_mean': np.mean(warning_times),
            'confidence_interval_90': (
                np.percentile(remaining_times, 5),
                np.percentile(remaining_times, 95)
            ),
            'average_final_soc': np.mean(final_socs)
        }
    
    def predict_by_time_blocks(self, current_hour: float) -> Dict:
        """
        按时间区块预测剩余时间
        
        基于马尔科夫链转移矩阵的稳态分布计算各区块能耗
        
        Parameters:
        -----------
        current_hour : float
            当前小时
        
        Returns:
        --------
        dict : 分区块预测结果
        """
        # 各状态的平均功耗 (W)
        state_power = {
            UserState.DEEP_SLEEP: 0.5,
            UserState.LIGHT_USE: 2.0,
            UserState.STREAMING: 4.0,
            UserState.GAMING: 8.0
        }
        
        predictions = {}
        remaining_soc = self.current_soc
        total_remaining_hours = 0
        
        # 按时间顺序遍历区块
        blocks_order = [
            ('morning_work', TimeMode.WORK),
            ('lunch_break', TimeMode.LEISURE),
            ('afternoon_work', TimeMode.WORK),
            ('evening_leisure', TimeMode.LEISURE),
            ('night_sleep', TimeMode.SLEEP),
            ('morning_commute', TimeMode.LEISURE)
        ]
        
        # 从当前时间区块开始
        block_info = self.get_time_block_info(current_hour)
        
        for block_name, time_mode in blocks_order:
            start, end = self.config.time_blocks[block_name]
            duration = (end - start) if end > start else (24 - start + end)
            
            # 获取稳态分布
            pi = self.user_model.compute_stationary_distribution(time_mode)
            
            # 计算期望功耗
            expected_power = sum(
                pi[s.value] * state_power[s] 
                for s in UserState
            )
            
            # 计算能量消耗 (Wh)
            energy_wh = expected_power * duration
            
            # 计算SOC消耗
            battery_wh = (self.config.battery_capacity_mah / 1000 * 
                         self.config.battery_voltage_nominal)
            soc_consumed = energy_wh / battery_wh
            
            # 检查是否在此区块耗尽
            if remaining_soc - soc_consumed <= self.config.soc_min_threshold:
                # 计算精确耗尽时间
                usable_soc = remaining_soc - self.config.soc_min_threshold
                hours_in_block = usable_soc * battery_wh / expected_power
                total_remaining_hours += hours_in_block
                
                predictions[block_name] = {
                    'duration_hours': hours_in_block,
                    'expected_power': expected_power,
                    'soc_consumed': usable_soc,
                    'depleted_in_block': True
                }
                break
            else:
                remaining_soc -= soc_consumed
                total_remaining_hours += duration
                
                predictions[block_name] = {
                    'duration_hours': duration,
                    'expected_power': expected_power,
                    'soc_consumed': soc_consumed,
                    'remaining_soc_after': remaining_soc,
                    'depleted_in_block': False
                }
        
        return {
            'block_predictions': predictions,
            'total_remaining_hours': total_remaining_hours,
            'current_soc': self.current_soc,
            'final_soc': remaining_soc
        }
    
    def get_optimized_prediction(self, current_hour: float,
                                  user_priority: str = 'balanced') -> Dict:
        """
        获取优化后的预测
        
        结合多目标优化给出最优操作建议
        
        Parameters:
        -----------
        current_hour : float
            当前小时
        user_priority : str
            用户优先级
        
        Returns:
        --------
        dict : 优化预测结果
        """
        # 基础预测
        basic_pred = self.predict_remaining_time_deterministic(current_hour)
        block_pred = self.predict_by_time_blocks(current_hour)
        
        # 多目标优化
        opt_result = self.optimizer.predict_remaining_time(
            soc_current=self.current_soc,
            temperature=self.current_temperature,
            user_behavior=user_priority,
            method='weighted'
        )
        
        return {
            'basic_prediction': {
                'remaining_hours': basic_pred['remaining_hours'],
                'average_power': basic_pred['average_power']
            },
            'block_prediction': {
                'remaining_hours': block_pred['total_remaining_hours'],
                'block_details': block_pred['block_predictions']
            },
            'optimized_prediction': {
                'remaining_hours': opt_result['remaining_time_hours'],
                'uncertainty': opt_result['uncertainty_hours'],
                'confidence_interval': opt_result['confidence_interval']
            },
            'recommendations': {
                'optimal_brightness': opt_result.get('optimal_brightness', 'N/A'),
                'optimal_cpu_freq': opt_result.get('optimal_cpu_freq', 'N/A'),
                'optimal_refresh_rate': opt_result.get('optimal_refresh_rate', 'N/A')
            },
            'current_state': {
                'soc': self.current_soc,
                'temperature_celsius': self.current_temperature - 273.15
            }
        }
    
    def format_prediction_report(self, prediction: Dict) -> str:
        """
        格式化预测报告
        
        Parameters:
        -----------
        prediction : dict
            预测结果
        
        Returns:
        --------
        str : 格式化报告
        """
        report = []
        report.append("=" * 60)
        report.append("电池剩余使用时间预测报告")
        report.append("=" * 60)
        
        # 当前状态
        state = prediction['current_state']
        report.append(f"\n当前状态:")
        report.append(f"  SOC: {state['soc']:.1%}")
        report.append(f"  温度: {state['temperature_celsius']:.1f}°C")
        
        # 基础预测
        basic = prediction['basic_prediction']
        report.append(f"\n基础预测 (确定性模型):")
        report.append(f"  剩余时间: {basic['remaining_hours']:.1f} 小时")
        report.append(f"  平均功耗: {basic['average_power']:.2f} W")
        
        # 区块预测
        block = prediction['block_prediction']
        report.append(f"\n时间区块预测:")
        report.append(f"  总剩余时间: {block['remaining_hours']:.1f} 小时")
        report.append("  各区块详情:")
        for name, details in block['block_details'].items():
            depleted = " [耗尽]" if details.get('depleted_in_block', False) else ""
            report.append(f"    {name}: {details['duration_hours']:.1f}h, "
                         f"{details['expected_power']:.1f}W{depleted}")
        
        # 优化预测
        opt = prediction['optimized_prediction']
        report.append(f"\n优化预测 (含不确定性):")
        report.append(f"  剩余时间: {opt['remaining_hours']:.1f} ± {opt['uncertainty']:.1f} 小时")
        ci = opt['confidence_interval']
        report.append(f"  90%置信区间: [{ci[0]:.1f}, {ci[1]:.1f}] 小时")
        
        # 建议
        rec = prediction['recommendations']
        report.append(f"\n省电建议:")
        if rec['optimal_brightness'] != 'N/A':
            report.append(f"  建议亮度: {rec['optimal_brightness']:.0f} nits")
        if rec['optimal_refresh_rate'] != 'N/A':
            report.append(f"  建议刷新率: {rec['optimal_refresh_rate']:.0f} Hz")
        
        report.append("\n" + "=" * 60)
        
        return "\n".join(report)


def create_predictor(battery_capacity_mah: float = 4000,
                     use_kalman: bool = True) -> IntegratedRemainingTimePredictor:
    """
    创建预测器实例
    
    Parameters:
    -----------
    battery_capacity_mah : float
        电池容量
    use_kalman : bool
        是否使用卡尔曼滤波
    
    Returns:
    --------
    IntegratedRemainingTimePredictor : 预测器实例
    """
    config = PredictionConfig(
        battery_capacity_mah=battery_capacity_mah,
        use_kalman_filter=use_kalman
    )
    return IntegratedRemainingTimePredictor(config)


if __name__ == "__main__":
    print("电池剩余使用时间预测器测试")
    print("=" * 60)
    
    # 创建预测器
    predictor = create_predictor(battery_capacity_mah=4000, use_kalman=True)
    predictor.initialize(soc_init=0.85, temp_init=300)
    
    # 当前时间假设为上午10点
    current_hour = 10.0
    
    print(f"\n当前SOC: {predictor.current_soc:.1%}")
    print(f"当前时间: {current_hour:.0f}:00")
    
    # 确定性预测
    print("\n进行确定性预测...")
    det_result = predictor.predict_remaining_time_deterministic(current_hour)
    print(f"预测剩余时间: {det_result['remaining_hours']:.2f} 小时")
    print(f"平均功耗: {det_result['average_power']:.2f} W")
    
    # 区块预测
    print("\n进行时间区块预测...")
    block_result = predictor.predict_by_time_blocks(current_hour)
    print(f"区块预测剩余时间: {block_result['total_remaining_hours']:.2f} 小时")
    
    # 优化预测
    print("\n进行优化预测...")
    opt_result = predictor.get_optimized_prediction(current_hour, 'balanced')
    
    # 输出报告
    report = predictor.format_prediction_report(opt_result)
    print(report)
    
    # 概率预测 (减少样本数以加快测试)
    print("\n进行概率预测 (蒙特卡罗, 20样本)...")
    prob_result = predictor.predict_remaining_time_probabilistic(current_hour, n_samples=20)
    print(f"剩余时间均值: {prob_result['remaining_hours_mean']:.2f} 小时")
    print(f"剩余时间标准差: {prob_result['remaining_hours_std']:.2f} 小时")
    print(f"90%置信区间: [{prob_result['confidence_interval_90'][0]:.2f}, "
          f"{prob_result['confidence_interval_90'][1]:.2f}] 小时")
