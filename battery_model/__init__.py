"""
Smartphone Battery Discharge Model
基于耦合微分方程组的智能手机电池耗电模型

This package implements a continuous-time mathematical model for smartphone
battery State of Charge (SOC) prediction under realistic usage conditions.

Key Features:
- Coupled electrochemical-thermal battery dynamics
- Multi-component power consumption modeling (CPU, Display, 5G, BT, GNSS, Background)
- Kalman filtering for state estimation
- Multi-objective optimization for parameter identification
- Remaining discharge time prediction

Author: Battery Modeling Research
"""

from .coupled_equations import BatteryCoupledModel
from .kalman_filter import ExtendedKalmanFilter, UnscentedKalmanFilter
from .optimization import MultiObjectiveOptimizer
from .prediction import RemainingTimePredictor
from .visualization import BatteryVisualizer
from .parameters import BatteryParameters, UsageScenarios

__version__ = "1.0.0"
__all__ = [
    'BatteryCoupledModel',
    'ExtendedKalmanFilter', 
    'UnscentedKalmanFilter',
    'MultiObjectiveOptimizer',
    'RemainingTimePredictor',
    'BatteryVisualizer',
    'BatteryParameters',
    'UsageScenarios'
]
