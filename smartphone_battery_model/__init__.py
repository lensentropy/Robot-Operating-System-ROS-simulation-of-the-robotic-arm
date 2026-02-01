"""
智能手机电池耗电建模系统
Smartphone Battery Discharge Modeling System

基于电化学-热耦合方程的连续时间SOC预测模型
Continuous-time SOC prediction based on electrochemical-thermal coupling equations
"""

from .battery_model import (
    SmartphoneBatteryModel,
    BatteryParameters,
    SoCModuleParameters,
    DisplayParameters,
    CommunicationParameters,
    GNSSParameters,
    BackgroundParameters,
    OCVModel,
    InternalResistanceModel,
    SoCPowerModule,
    DisplayPowerModule,
    CommunicationPowerModule,
    GNSSPowerModule,
    BackgroundPowerModule,
    UserBehaviorModel,
)

from .analysis import (
    BatteryAnalyzer,
    AnalysisResults,
)

from .visualization import (
    BatteryVisualizer,
    generate_all_visualizations,
)

__version__ = '1.0.0'
__author__ = 'Battery Modeling System'

__all__ = [
    'SmartphoneBatteryModel',
    'BatteryParameters',
    'BatteryAnalyzer',
    'BatteryVisualizer',
    'generate_all_visualizations',
]
