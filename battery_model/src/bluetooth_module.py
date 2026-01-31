"""
Bluetooth Module Power Consumption Model
========================================

This module implements a comprehensive continuous-time model for Bluetooth
power consumption, covering:

1. Bluetooth Classic (BR/EDR) - for audio streaming
2. Bluetooth Low Energy (BLE) - for IoT, wearables
3. Dual-mode operation

Key features:
- Connection interval and latency modeling
- Advertising/scanning state machines
- Audio codec power (A2DP, LC3)
- Multi-device connection overhead

References:
- Bluetooth Core Specification v5.3
- Nordic Semiconductor power profiling data
- Qualcomm WCN3990/WCN6855 specifications
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, List, Dict
from enum import IntEnum


class BTClassicState(IntEnum):
    """Bluetooth Classic operational states."""
    OFF = 0
    STANDBY = 1
    INQUIRY = 2
    PAGE = 3
    CONNECTED_IDLE = 4
    CONNECTED_ACTIVE = 5  # Data transfer
    CONNECTED_AUDIO = 6   # A2DP/HFP streaming


class BLEState(IntEnum):
    """Bluetooth Low Energy operational states."""
    OFF = 0
    STANDBY = 1
    ADVERTISING = 2
    SCANNING = 3
    INITIATING = 4
    CONNECTED_PERIPHERAL = 5
    CONNECTED_CENTRAL = 6


@dataclass
class BluetoothParameters:
    """
    Bluetooth module parameters based on modern smartphone implementations.
    
    Reference: Qualcomm FastConnect 6900, Broadcom BCM4389
    """
    # === Bluetooth Classic Parameters ===
    # Power states (W)
    P_classic_off: float = 0.0001  # Leakage
    P_classic_standby: float = 0.002  # Standby
    P_classic_inquiry: float = 0.080  # Device discovery
    P_classic_page: float = 0.060  # Connection setup
    P_classic_idle: float = 0.008  # Connected but idle
    P_classic_active: float = 0.040  # Data transfer (SPP)
    P_classic_audio: float = 0.035  # A2DP streaming
    
    # Audio codec power additions
    P_sbc_codec: float = 0.005  # SBC encode/decode
    P_aac_codec: float = 0.008  # AAC encode/decode
    P_aptx_codec: float = 0.010  # aptX encode/decode
    P_ldac_codec: float = 0.015  # LDAC (high-res)
    P_lc3_codec: float = 0.006  # LC3 (LE Audio)
    
    # === BLE Parameters ===
    P_ble_off: float = 0.00005
    P_ble_standby: float = 0.0003
    P_ble_advertising: float = 0.012  # Advertising at 100ms interval
    P_ble_scanning: float = 0.025  # Active scanning
    P_ble_initiating: float = 0.015
    P_ble_connected: float = 0.003  # Per connection at 100ms interval
    
    # BLE connection parameters
    conn_interval_min: float = 0.0075  # 7.5 ms
    conn_interval_max: float = 4.0     # 4 s
    slave_latency_max: int = 500
    
    # Advertising parameters
    adv_interval_min: float = 0.02   # 20 ms
    adv_interval_max: float = 10.24  # 10.24 s
    
    # Tx power levels (dBm) and corresponding power consumption
    tx_power_levels: Dict = None  # Will be set in __init__
    
    # Multi-connection overhead factor
    multi_conn_factor: float = 1.3  # 30% overhead per additional connection
    
    def __post_init__(self):
        if self.tx_power_levels is None:
            self.tx_power_levels = {
                -20: 0.003,  # -20 dBm: 3 mW
                -10: 0.005,  # -10 dBm: 5 mW
                0: 0.010,    # 0 dBm: 10 mW
                4: 0.018,    # 4 dBm: 18 mW (typical max)
                8: 0.030     # 8 dBm: 30 mW (extended range)
            }


class BluetoothClassicModule:
    """
    Bluetooth Classic (BR/EDR) power model.
    
    Focuses on audio streaming (A2DP) which is the dominant use case.
    """
    
    def __init__(self, params: Optional[BluetoothParameters] = None):
        self.params = params or BluetoothParameters()
        self.current_state = BTClassicState.OFF
        self.audio_codec = 'sbc'
        self.n_connections = 0
        
    def codec_power(self, codec: str) -> float:
        """Get codec-specific power consumption."""
        p = self.params
        codecs = {
            'sbc': p.P_sbc_codec,
            'aac': p.P_aac_codec,
            'aptx': p.P_aptx_codec,
            'aptx_hd': p.P_aptx_codec * 1.3,
            'ldac': p.P_ldac_codec,
            'lc3': p.P_lc3_codec
        }
        return codecs.get(codec.lower(), p.P_sbc_codec)
    
    def state_power(self, state: BTClassicState, codec: str = 'sbc') -> float:
        """
        Calculate power for given state.
        """
        p = self.params
        
        base_power = {
            BTClassicState.OFF: p.P_classic_off,
            BTClassicState.STANDBY: p.P_classic_standby,
            BTClassicState.INQUIRY: p.P_classic_inquiry,
            BTClassicState.PAGE: p.P_classic_page,
            BTClassicState.CONNECTED_IDLE: p.P_classic_idle,
            BTClassicState.CONNECTED_ACTIVE: p.P_classic_active,
            BTClassicState.CONNECTED_AUDIO: p.P_classic_audio
        }
        
        power = base_power.get(state, p.P_classic_standby)
        
        # Add codec power for audio state
        if state == BTClassicState.CONNECTED_AUDIO:
            power += self.codec_power(codec)
        
        return power
    
    def audio_streaming_power(self, bitrate: float, codec: str = 'sbc',
                              n_devices: int = 1) -> float:
        """
        Calculate power for A2DP audio streaming.
        
        Args:
            bitrate: Audio bitrate (kbps)
            codec: Audio codec
            n_devices: Number of connected audio devices
        """
        p = self.params
        
        # Base audio power
        base_power = p.P_classic_audio
        
        # Codec power
        codec_pwr = self.codec_power(codec)
        
        # Bitrate scaling (higher bitrate = slightly more RF power)
        bitrate_factor = 1 + 0.1 * (bitrate - 328) / 328  # SBC default is 328 kbps
        
        # Multi-device overhead (TWS, multipoint)
        device_factor = 1 + (n_devices - 1) * 0.4  # 40% per additional device
        
        return (base_power * bitrate_factor + codec_pwr) * device_factor


class BLEModule:
    """
    Bluetooth Low Energy power model.
    
    Models advertising, scanning, and connection power based on
    connection interval and PHY selection.
    """
    
    def __init__(self, params: Optional[BluetoothParameters] = None):
        self.params = params or BluetoothParameters()
        self.current_state = BLEState.OFF
        self.connections = []  # List of connection parameters
        
    def advertising_power(self, interval_ms: float = 100,
                          tx_power_dbm: int = 0) -> float:
        """
        Calculate advertising power consumption.
        
        Power scales inversely with advertising interval.
        
        Args:
            interval_ms: Advertising interval in ms
            tx_power_dbm: Transmit power in dBm
        """
        p = self.params
        
        # Base advertising power (at 100ms interval)
        base_power = p.P_ble_advertising
        
        # Scale by interval ratio
        interval_factor = 100.0 / max(interval_ms, 20)
        
        # TX power component
        tx_powers = p.tx_power_levels
        closest_level = min(tx_powers.keys(), key=lambda x: abs(x - tx_power_dbm))
        tx_power_w = tx_powers[closest_level]
        
        # Advertising duty cycle (~3ms per interval)
        duty_cycle = 3.0 / interval_ms
        
        return base_power * interval_factor + tx_power_w * duty_cycle
    
    def scanning_power(self, window_ms: float = 30,
                       interval_ms: float = 100,
                       active: bool = True) -> float:
        """
        Calculate scanning power consumption.
        
        Args:
            window_ms: Scan window duration
            interval_ms: Scan interval
            active: Active (with scan requests) or passive scanning
        """
        p = self.params
        
        # Duty cycle
        duty_cycle = window_ms / interval_ms
        
        # Active scanning adds scan request overhead
        mode_factor = 1.3 if active else 1.0
        
        return p.P_ble_scanning * duty_cycle * mode_factor
    
    def connection_power(self, interval_ms: float = 100,
                         slave_latency: int = 0,
                         payload_bytes: int = 20,
                         phy: str = '1M') -> float:
        """
        Calculate per-connection power consumption.
        
        Args:
            interval_ms: Connection interval
            slave_latency: Allowed skipped events
            payload_bytes: Typical payload size
            phy: PHY mode ('1M', '2M', 'Coded')
        """
        p = self.params
        
        # Effective interval considering latency
        effective_interval = interval_ms * (1 + slave_latency)
        
        # Base power scales with interval
        base_power = p.P_ble_connected * (100.0 / effective_interval)
        
        # PHY mode efficiency
        phy_factors = {
            '1M': 1.0,
            '2M': 0.7,   # Faster = less radio on time
            'Coded': 2.5  # Long range = more power
        }
        phy_factor = phy_factors.get(phy, 1.0)
        
        # Payload size impact (larger = longer TX)
        payload_factor = 1 + 0.005 * (payload_bytes - 20)
        
        return base_power * phy_factor * payload_factor
    
    def multi_connection_power(self, connections: List[Dict]) -> float:
        """
        Calculate total power for multiple BLE connections.
        
        Args:
            connections: List of connection parameter dicts
        """
        if not connections:
            return self.params.P_ble_standby
        
        total = 0
        for i, conn in enumerate(connections):
            conn_power = self.connection_power(
                interval_ms=conn.get('interval_ms', 100),
                slave_latency=conn.get('slave_latency', 0),
                payload_bytes=conn.get('payload_bytes', 20),
                phy=conn.get('phy', '1M')
            )
            
            # Add scheduling overhead for multiple connections
            if i > 0:
                conn_power *= self.params.multi_conn_factor
            
            total += conn_power
        
        return total


class BluetoothModule:
    """
    Complete Bluetooth module combining Classic and BLE.
    
    Implements continuous-time power model for realistic scenarios.
    """
    
    def __init__(self, params: Optional[BluetoothParameters] = None):
        self.params = params or BluetoothParameters()
        self.classic = BluetoothClassicModule(self.params)
        self.ble = BLEModule(self.params)
        
    def sigmoid(self, x: float, k: float = 1.0, x0: float = 0.0) -> float:
        """Smooth transition function."""
        return 1.0 / (1.0 + np.exp(-k * (x - x0)))
    
    def total_power(self, 
                    bt_classic_active: bool = False,
                    audio_streaming: bool = False,
                    audio_codec: str = 'aac',
                    audio_bitrate: float = 256,
                    n_audio_devices: int = 1,
                    ble_advertising: bool = False,
                    ble_adv_interval_ms: float = 100,
                    ble_scanning: bool = False,
                    ble_connections: List[Dict] = None) -> float:
        """
        Calculate total Bluetooth module power.
        
        Combines Classic and BLE contributions.
        """
        p = self.params
        total = 0
        
        # === Bluetooth Classic ===
        if audio_streaming:
            total += self.classic.audio_streaming_power(
                bitrate=audio_bitrate,
                codec=audio_codec,
                n_devices=n_audio_devices
            )
        elif bt_classic_active:
            total += p.P_classic_idle
        else:
            total += p.P_classic_standby
        
        # === BLE ===
        if ble_advertising:
            total += self.ble.advertising_power(interval_ms=ble_adv_interval_ms)
        
        if ble_scanning:
            total += self.ble.scanning_power()
        
        if ble_connections:
            total += self.ble.multi_connection_power(ble_connections)
        elif not ble_advertising and not ble_scanning:
            total += p.P_ble_standby
        
        return total
    
    def continuous_power_model(self, t: float,
                               audio_schedule: Callable[[float], bool] = None,
                               codec: str = 'aac',
                               ble_conn_schedule: Callable[[float], List[Dict]] = None
                               ) -> float:
        """
        Continuous-time power model for ODE integration.
        
        Args:
            t: Time (hours)
            audio_schedule: Function returning whether audio is playing
            codec: Audio codec
            ble_conn_schedule: Function returning BLE connections
        """
        audio_active = audio_schedule(t) if audio_schedule else False
        ble_conns = ble_conn_schedule(t) if ble_conn_schedule else []
        
        return self.total_power(
            audio_streaming=audio_active,
            audio_codec=codec,
            ble_connections=ble_conns
        )
    
    def current_draw(self, V_bat: float = 3.85, **kwargs) -> float:
        """Convert power to current."""
        return self.total_power(**kwargs) / V_bat


class BluetoothScenarios:
    """
    Predefined Bluetooth usage scenarios.
    """
    
    @staticmethod
    def music_streaming_tws() -> Dict:
        """
        True Wireless Stereo earbuds music streaming.
        Typical commute/workout scenario.
        """
        return {
            'bt_classic_active': True,
            'audio_streaming': True,
            'audio_codec': 'aac',
            'audio_bitrate': 256,
            'n_audio_devices': 2,  # Left + Right earbud
            'ble_advertising': False,
            'ble_scanning': False,
            'ble_connections': [
                {'interval_ms': 30, 'slave_latency': 0, 'phy': '2M'}
            ]  # LE Audio control channel
        }
    
    @staticmethod
    def smartwatch_connected() -> Dict:
        """
        Smartwatch continuously connected via BLE.
        """
        return {
            'bt_classic_active': False,
            'audio_streaming': False,
            'ble_advertising': False,
            'ble_scanning': False,
            'ble_connections': [
                {'interval_ms': 500, 'slave_latency': 4, 'payload_bytes': 50, 'phy': '1M'}
            ]
        }
    
    @staticmethod
    def fitness_tracker() -> Dict:
        """
        Fitness tracker with periodic sync.
        """
        return {
            'bt_classic_active': False,
            'audio_streaming': False,
            'ble_advertising': False,
            'ble_scanning': False,
            'ble_connections': [
                {'interval_ms': 1000, 'slave_latency': 10, 'payload_bytes': 20, 'phy': '1M'}
            ]
        }
    
    @staticmethod
    def car_bluetooth() -> Dict:
        """
        Connected to car for calls/audio.
        """
        return {
            'bt_classic_active': True,
            'audio_streaming': True,
            'audio_codec': 'sbc',  # Car systems often use SBC
            'audio_bitrate': 328,
            'n_audio_devices': 1,
            'ble_advertising': False,
            'ble_scanning': False,
            'ble_connections': []
        }
    
    @staticmethod
    def idle_discoverable() -> Dict:
        """
        Bluetooth on but idle, discoverable.
        """
        return {
            'bt_classic_active': False,
            'audio_streaming': False,
            'ble_advertising': True,
            'ble_adv_interval_ms': 1000,  # Slow advertising
            'ble_scanning': False,
            'ble_connections': []
        }


def analyze_bluetooth_power_breakdown():
    """
    Analyze Bluetooth power consumption across scenarios.
    Returns data for visualization.
    """
    module = BluetoothModule()
    scenarios = BluetoothScenarios()
    
    results = {}
    
    # Test each scenario
    scenario_configs = {
        'TWS Music': scenarios.music_streaming_tws(),
        'Smartwatch': scenarios.smartwatch_connected(),
        'Fitness Tracker': scenarios.fitness_tracker(),
        'Car Audio': scenarios.car_bluetooth(),
        'Idle Discoverable': scenarios.idle_discoverable()
    }
    
    for name, config in scenario_configs.items():
        power = module.total_power(**config)
        current = module.current_draw(**config)
        results[name] = {
            'power_mW': power * 1000,
            'current_mA': current * 1000,
            'config': config
        }
    
    # Connection interval sweep for BLE
    intervals = np.array([7.5, 15, 30, 50, 100, 200, 500, 1000, 2000, 4000])
    ble_power = []
    for interval in intervals:
        p = module.ble.connection_power(interval_ms=interval)
        ble_power.append(p * 1000)  # mW
    
    results['interval_sweep'] = {
        'intervals': intervals,
        'power_mW': np.array(ble_power)
    }
    
    # Codec comparison
    codecs = ['sbc', 'aac', 'aptx', 'aptx_hd', 'ldac', 'lc3']
    codec_power = []
    for codec in codecs:
        p = module.classic.audio_streaming_power(bitrate=256, codec=codec)
        codec_power.append(p * 1000)
    
    results['codec_comparison'] = {
        'codecs': codecs,
        'power_mW': codec_power
    }
    
    return results


if __name__ == "__main__":
    # Test Bluetooth module
    module = BluetoothModule()
    
    print("Bluetooth Module Power Analysis")
    print("=" * 50)
    
    # Analyze scenarios
    results = analyze_bluetooth_power_breakdown()
    
    print("\nScenario Power Consumption:")
    for name, data in results.items():
        if isinstance(data, dict) and 'power_mW' in data and not isinstance(data['power_mW'], np.ndarray):
            print(f"  {name}: {data['power_mW']:.1f} mW ({data['current_mA']:.1f} mA)")
    
    print("\nBLE Connection Interval Impact:")
    for interval, power in zip(results['interval_sweep']['intervals'],
                               results['interval_sweep']['power_mW']):
        print(f"  {interval:6.1f} ms: {power:.2f} mW")
    
    print("\nAudio Codec Comparison:")
    for codec, power in zip(results['codec_comparison']['codecs'],
                           results['codec_comparison']['power_mW']):
        print(f"  {codec:10s}: {power:.1f} mW")
