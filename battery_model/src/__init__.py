"""
Smartphone Battery Discharge Model
==================================

A comprehensive continuous-time mathematical model for smartphone 
battery state-of-charge (SOC) prediction under realistic usage conditions.

Modules:
--------
- battery_core: Core battery electrochemical model
- network_5g_module: 5G/LTE power consumption with link budget
- gnss_module: GPS/GNSS state machine model
- background_tasks_module: Stochastic background task model
- bluetooth_module: Bluetooth Classic and BLE power model
- coupled_system: Integrated system simulation
- visualizations: Publication-quality figure generation

2026 MCM Problem A Solution
"""

__version__ = '1.0.0'
__author__ = 'Battery Model Team'

from .battery_core import BatteryCore, BatteryParameters, ThermalModel
from .network_5g_module import Network5GModule, Network5GParameters
from .gnss_module import GNSSModule, GNSSParameters
from .background_tasks_module import BackgroundTasksModule, BackgroundTaskParameters
from .bluetooth_module import BluetoothModule, BluetoothParameters
from .coupled_system import CoupledBatterySystem, CoupledSystemParameters

__all__ = [
    'BatteryCore', 'BatteryParameters', 'ThermalModel',
    'Network5GModule', 'Network5GParameters',
    'GNSSModule', 'GNSSParameters', 
    'BackgroundTasksModule', 'BackgroundTaskParameters',
    'BluetoothModule', 'BluetoothParameters',
    'CoupledBatterySystem', 'CoupledSystemParameters'
]
