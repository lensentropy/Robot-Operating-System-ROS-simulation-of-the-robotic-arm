"""
智能手机电池连续时间耗电建模
Smartphone Battery Continuous-Time Modeling

本模块提供基于锂离子电池的智能手机电池建模工具：

核心组件:
---------
- battery_model: 电池连续时间数学模型
- kalman_filter: 扩展卡尔曼滤波器SOC估计
- optimization: 多目标优化和敏感性分析
- visualization: 综合可视化工具

使用示例:
--------
>>> from smartphone_battery_model import SmartphoneBatteryModel, create_moderate_usage
>>> model = SmartphoneBatteryModel()
>>> usage = create_moderate_usage()
>>> t, soc = model.simulate(1.0, usage, 10.0)
>>> remaining_time = model.estimate_remaining_time(1.0, usage)

作者: Battery Model System
版本: 1.0.0
"""

from .battery_model import (
    SmartphoneBatteryModel,
    BatteryParameters,
    UsageProfile,
    PowerConsumptionCoefficients,
    AdvancedBatteryModel,
    TimeVaryingUsageModel,
    create_idle_usage,
    create_light_usage,
    create_moderate_usage,
    create_heavy_usage,
    create_navigation_usage
)

from .kalman_filter import (
    ExtendedKalmanFilter,
    UnscentedKalmanFilter,
    AdaptiveKalmanFilter,
    SOCEstimator,
    KalmanFilterConfig
)

from .optimization import (
    SensitivityAnalyzer,
    UncertaintyQuantifier,
    ParticleSwarmOptimizer,
    BatteryModelOptimizer,
    NSGAII,
    OptimizationConfig
)

from .visualization import (
    BatteryVisualizer
)

__version__ = "1.0.0"
__author__ = "Battery Model System"

__all__ = [
    # Battery Model
    "SmartphoneBatteryModel",
    "BatteryParameters",
    "UsageProfile",
    "PowerConsumptionCoefficients",
    "AdvancedBatteryModel",
    "TimeVaryingUsageModel",
    
    # Usage Profiles
    "create_idle_usage",
    "create_light_usage",
    "create_moderate_usage",
    "create_heavy_usage",
    "create_navigation_usage",
    
    # Kalman Filter
    "ExtendedKalmanFilter",
    "UnscentedKalmanFilter",
    "AdaptiveKalmanFilter",
    "SOCEstimator",
    "KalmanFilterConfig",
    
    # Optimization
    "SensitivityAnalyzer",
    "UncertaintyQuantifier",
    "ParticleSwarmOptimizer",
    "BatteryModelOptimizer",
    "NSGAII",
    "OptimizationConfig",
    
    # Visualization
    "BatteryVisualizer"
]
