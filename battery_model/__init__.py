"""
电池SOC-能耗耦合连续时间模型包
Battery SOC-Energy Consumption Coupled Continuous-Time Model Package

模块:
- soc_energy_coupled_model: SOC-能耗耦合微分方程模型
- markov_user_behavior: 时间非齐次马尔科夫链用户行为模型
- kalman_filter_soc: 扩展卡尔曼滤波SOC状态估计
- multi_objective_optimizer: 多目标优化电池寿命预测
- remaining_time_predictor: 剩余使用时间预测
- visualization: 可视化工具

Author: Battery Model Expert
Date: February 2026
"""

from .soc_energy_coupled_model import (
    CoupledSOCEnergyModel,
    BatteryParameters,
    SoCParameters,
    DisplayParameters,
    WirelessParameters,
    create_default_model
)

from .markov_user_behavior import (
    MarkovUserBehaviorModel,
    MarkovTimeBlockPredictor,
    UserState,
    TimeMode,
    HardwareState
)

from .kalman_filter_soc import (
    ExtendedKalmanFilter,
    UnscentedKalmanFilter,
    AdaptiveEKF,
    EKFParameters
)

from .multi_objective_optimizer import (
    WeightedSumOptimizer,
    NSGA2Optimizer,
    BatteryLifePredictor,
    OptimizationConfig,
    ObjectiveFunctions
)

from .remaining_time_predictor import (
    IntegratedRemainingTimePredictor,
    PredictionConfig,
    create_predictor
)

__version__ = '1.0.0'
__author__ = 'Battery Model Expert'

__all__ = [
    # 耦合模型
    'CoupledSOCEnergyModel',
    'BatteryParameters',
    'SoCParameters',
    'DisplayParameters',
    'WirelessParameters',
    'create_default_model',
    
    # 马尔科夫链
    'MarkovUserBehaviorModel',
    'MarkovTimeBlockPredictor',
    'UserState',
    'TimeMode',
    'HardwareState',
    
    # 卡尔曼滤波
    'ExtendedKalmanFilter',
    'UnscentedKalmanFilter',
    'AdaptiveEKF',
    'EKFParameters',
    
    # 多目标优化
    'WeightedSumOptimizer',
    'NSGA2Optimizer',
    'BatteryLifePredictor',
    'OptimizationConfig',
    'ObjectiveFunctions',
    
    # 剩余时间预测
    'IntegratedRemainingTimePredictor',
    'PredictionConfig',
    'create_predictor'
]
