"""
Battery and Smartphone Power Modeling Package

This package provides physics-based models for:
- Lithium-ion battery electro-thermal-aging dynamics
- Smartphone multi-physics power consumption
- Time-inhomogeneous Markov chain user behavior
"""

from .battery_model import BatteryModel, BatterySimulator, BatteryParameters
from .smartphone_power import (
    FiveGModel, BluetoothModel, BackgroundModel,
    GNSSModel, OLEDModel, SoCModel, SmartphonePowerModel
)
from .markov_user_behavior import (
    MarkovChainSimulator, MarkovDifferentialEquations,
    TransitionMatrices, TimePartition, MarkovStates,
    HardwareParameters
)

__all__ = [
    'BatteryModel', 'BatterySimulator', 'BatteryParameters',
    'FiveGModel', 'BluetoothModel', 'BackgroundModel',
    'GNSSModel', 'OLEDModel', 'SoCModel', 'SmartphonePowerModel',
    'MarkovChainSimulator', 'MarkovDifferentialEquations',
    'TransitionMatrices', 'TimePartition', 'MarkovStates',
    'HardwareParameters'
]
