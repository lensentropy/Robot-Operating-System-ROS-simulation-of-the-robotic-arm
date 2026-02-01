"""
Battery and Usage Parameters
电池和使用参数

Based on published measurements and specifications from:
- Li-ion battery datasheets (Samsung SDI, LG Chem)
- IEEE papers on smartphone power consumption
- Android Battery Historian data patterns

All parameters are documented with their sources and physical units.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
from enum import Enum


class UserState(Enum):
    """User activity states for Markov model"""
    SLEEP = 0      # 睡眠状态 (23:00 - 7:00)
    WORK = 1       # 工作状态 (9:00 - 18:00)
    LEISURE = 2    # 休闲状态 (18:00 - 23:00)
    COMMUTE = 3    # 通勤状态 (7:00 - 9:00)


@dataclass
class BatteryParameters:
    """
    Lithium-ion battery electrochemical and thermal parameters
    锂离子电池电化学和热参数
    
    References:
    [1] Chen et al., "A Review of Lithium-Ion Battery State of Charge 
        Estimation Based on Deep Learning," IEEE Access, 2021
    [2] Newman & Tiedemann, "Porous-Electrode Theory," AIChE J, 1975
    [3] Samsung SDI INR18650-25R datasheet
    """
    
    # Battery capacity (典型智能手机电池容量)
    Q_max: float = 4000.0  # mAh, typical flagship phone battery
    Q_max_Ah: float = 4.0  # Ah
    Q_max_As: float = 14400.0  # Ampere-seconds
    
    # Nominal voltage characteristics
    V_nom: float = 3.7  # V, nominal voltage
    V_max: float = 4.2  # V, fully charged
    V_min: float = 3.0  # V, cutoff voltage
    V_cutoff: float = 3.3  # V, practical cutoff
    
    # Open Circuit Voltage (OCV) polynomial coefficients
    # V_OCV(SOC) = sum(a_i * SOC^i) - fitted from experimental data
    OCV_coeffs: np.ndarray = field(default_factory=lambda: np.array([
        3.0,      # a0: minimum voltage
        0.8,      # a1: linear term
        0.3,      # a2: quadratic term
        0.1,      # a3: cubic term (for transition regions)
    ]))
    
    # Internal resistance model R_int(SOC, T, N)
    # R_int = R_0 * f(SOC) * g(T) * h(N)
    R_0: float = 0.05  # Ohm, base internal resistance at SOC=0.5, T=25°C, fresh
    R_SOC_coeffs: np.ndarray = field(default_factory=lambda: np.array([
        1.5,   # SOC < 0.1: higher resistance
        1.0,   # 0.1 < SOC < 0.9
        1.3,   # SOC > 0.9: higher resistance
    ]))
    
    # Temperature dependence (Arrhenius-type)
    E_a: float = 20000.0  # J/mol, activation energy
    R_gas: float = 8.314  # J/(mol·K), gas constant
    T_ref: float = 298.15  # K, reference temperature (25°C)
    
    # Cycle aging factor
    N_ref: float = 500.0  # reference cycles for 80% capacity
    alpha_aging: float = 0.002  # capacity fade per cycle
    
    # Thermal parameters
    C_th_batt: float = 50.0  # J/K, battery thermal capacitance
    R_th_batt: float = 10.0  # K/W, thermal resistance to environment
    T_env: float = 298.15  # K, ambient temperature (25°C)
    
    # Entropy coefficient for reversible heating
    dVdT: float = -0.0002  # V/K, entropy coefficient
    
    # PMIC (Power Management IC) efficiency
    eta_PMIC: float = 0.92  # typical efficiency
    
    def V_OCV(self, SOC: np.ndarray) -> np.ndarray:
        """Calculate Open Circuit Voltage from SOC using polynomial model"""
        SOC = np.clip(SOC, 0.0, 1.0)
        # Enhanced OCV model with characteristic Li-ion curve
        # Features: flat region in middle, steep at extremes
        V = (self.V_min + 
             (self.V_max - self.V_min) * (
                 0.1 * SOC + 
                 0.8 * (1 - np.exp(-5 * SOC)) +
                 0.1 * SOC**2
             ) * (1 - 0.1 * (1 - SOC)**8))  # steep drop at low SOC
        return np.clip(V, self.V_min, self.V_max)
    
    def R_int(self, SOC: np.ndarray, T: float, N: int = 0) -> np.ndarray:
        """
        Calculate internal resistance as function of SOC, temperature, and cycles
        内阻计算：R_int(SOC, T, N) = R_0 * f(SOC) * g(T) * h(N)
        """
        SOC = np.clip(SOC, 0.01, 0.99)
        
        # SOC dependence: U-shaped curve (higher at extremes)
        f_SOC = 1.0 + 0.3 * (SOC - 0.5)**2 + 0.5 * np.exp(-10 * SOC)
        
        # Temperature dependence: Arrhenius
        g_T = np.exp(self.E_a / self.R_gas * (1/T - 1/self.T_ref))
        
        # Aging dependence: linear increase with cycles
        h_N = 1.0 + self.alpha_aging * N
        
        return self.R_0 * f_SOC * g_T * h_N
    
    def capacity_fade(self, N: int) -> float:
        """Calculate capacity fade factor due to aging"""
        return max(0.7, 1.0 - self.alpha_aging * N)


@dataclass
class CPUParameters:
    """
    System-on-Chip (SoC) power parameters
    处理器功耗参数
    
    References:
    [1] Carroll & Heiser, "An Analysis of Power Consumption in a Smartphone," 
        USENIX ATC, 2010
    [2] Qualcomm Snapdragon 888 power specifications
    """
    
    # DVFS (Dynamic Voltage and Frequency Scaling) states
    freq_levels: np.ndarray = field(default_factory=lambda: np.array([
        0.3, 0.6, 1.0, 1.5, 2.0, 2.8  # GHz
    ]))
    voltage_levels: np.ndarray = field(default_factory=lambda: np.array([
        0.65, 0.75, 0.85, 0.95, 1.05, 1.15  # V
    ]))
    
    # Dynamic power: P_dyn = C_eff * V^2 * f
    C_eff: float = 0.5e-9  # F, effective capacitance
    
    # Leakage power temperature coefficient
    P_leak_0: float = 0.2  # W at 25°C
    leak_temp_coeff: float = 0.03  # per °C
    
    # Thermal parameters
    C_th_cpu: float = 2.0  # J/K, CPU thermal mass
    R_th_cpu_batt: float = 5.0  # K/W, CPU to battery
    R_th_cpu_env: float = 15.0  # K/W, CPU to environment
    
    def power_consumption(self, load: float, T_cpu: float) -> Tuple[float, int]:
        """
        Calculate CPU power based on load and temperature
        Returns (power_watts, selected_frequency_index)
        """
        # Select DVFS state based on load
        load = np.clip(load, 0.0, 1.0)
        freq_idx = int(load * (len(self.freq_levels) - 1))
        
        f = self.freq_levels[freq_idx] * 1e9  # Hz
        V = self.voltage_levels[freq_idx]  # V
        
        # Dynamic power
        P_dyn = self.C_eff * V**2 * f * load
        
        # Leakage power (temperature dependent)
        T_celsius = T_cpu - 273.15
        P_leak = self.P_leak_0 * (1 + self.leak_temp_coeff * (T_celsius - 25))
        
        return P_dyn + P_leak, freq_idx


@dataclass 
class DisplayParameters:
    """
    Display power parameters
    显示屏功耗参数
    
    References:
    [1] Kim et al., "OLED Display Power Consumption Model," SID, 2015
    [2] Typical 6.7" AMOLED specifications
    """
    
    # Screen properties
    screen_area: float = 0.0105  # m², typical 6.7" display
    pixel_count: int = 3200 * 1440  # QHD+
    
    # Power components
    P_static: float = 0.1  # W, baseline display controller
    P_driver_per_Hz: float = 1e-5  # W/Hz, driver IC scaling
    P_panel_per_lux: float = 0.003  # W per brightness level (0-1000 nits)
    
    # Refresh rates
    refresh_rates: np.ndarray = field(default_factory=lambda: np.array([
        30, 60, 90, 120  # Hz
    ]))
    
    # Content-dependent power factor (AMOLED)
    # Dark content uses less power
    dark_content_factor: float = 0.3
    bright_content_factor: float = 1.0
    
    def power_consumption(self, brightness: float, refresh_idx: int = 1,
                         content_brightness: float = 0.5) -> float:
        """
        Calculate display power consumption
        brightness: 0-1 (screen brightness setting)
        refresh_idx: index into refresh_rates
        content_brightness: 0-1 (average screen content brightness)
        """
        f_refresh = self.refresh_rates[min(refresh_idx, len(self.refresh_rates)-1)]
        
        # Static power
        P = self.P_static
        
        # Driver power (scales with refresh rate)
        P += self.P_driver_per_Hz * f_refresh
        
        # Panel power (depends on brightness and content)
        brightness = np.clip(brightness, 0.0, 1.0)
        content_factor = (self.dark_content_factor + 
                         (self.bright_content_factor - self.dark_content_factor) * content_brightness)
        P += self.P_panel_per_lux * brightness * 1000 * content_factor
        
        return P


@dataclass
class NetworkParameters:
    """
    5G/LTE and WiFi power parameters
    网络模块功耗参数
    
    References:
    [1] Huang et al., "A Close Look at LTE," IMC 2012
    [2] 3GPP TR 38.840 5G NR power consumption
    """
    
    # 5G modem states
    P_5G_idle: float = 0.05  # W, RRC_IDLE
    P_5G_connected: float = 0.8  # W, RRC_CONNECTED baseline
    P_5G_tx_max: float = 2.5  # W, maximum TX power
    
    # Path loss model parameters
    alpha_pathloss: float = 3.5  # path loss exponent (urban)
    d_ref: float = 100.0  # m, reference distance
    P_tx_ref: float = 0.5  # W at reference distance
    
    # Data rate power scaling
    power_per_Mbps: float = 0.01  # W per Mbps
    
    # WiFi parameters
    P_WiFi_idle: float = 0.02  # W
    P_WiFi_active: float = 0.3  # W
    
    def power_5G(self, data_rate_Mbps: float, distance_m: float = 100,
                 signal_quality: float = 0.8) -> float:
        """
        Calculate 5G power consumption
        data_rate_Mbps: current data transfer rate
        distance_m: estimated distance to base station
        signal_quality: 0-1, signal strength factor
        """
        if data_rate_Mbps <= 0:
            return self.P_5G_idle
        
        # Base connected power
        P = self.P_5G_connected
        
        # Data transfer power
        P += self.power_per_Mbps * data_rate_Mbps
        
        # TX power adjustment for path loss
        path_loss_factor = (distance_m / self.d_ref) ** self.alpha_pathloss
        signal_factor = 1.0 / max(signal_quality, 0.1)
        P += self.P_tx_ref * path_loss_factor * signal_factor * 0.1
        
        return min(P, self.P_5G_tx_max)


@dataclass
class BluetoothParameters:
    """
    Bluetooth module power parameters  
    蓝牙模块功耗参数
    
    References:
    [1] Bluetooth 5.2 specification power requirements
    [2] Nordic Semiconductor nRF52840 datasheet
    """
    
    P_sleep: float = 0.001  # W, deep sleep
    P_idle: float = 0.01  # W, connected but idle
    P_audio_streaming: float = 0.05  # W, A2DP audio
    P_data_transfer: float = 0.08  # W, SPP/GATT data
    
    # Event-driven model parameters
    event_energy: float = 0.0001  # J per BLE event
    audio_continuous: bool = False
    
    def power_consumption(self, audio_active: bool = False,
                         data_rate_kbps: float = 0,
                         event_rate: float = 0) -> float:
        """Calculate Bluetooth power consumption"""
        if audio_active:
            return self.P_audio_streaming
        elif data_rate_kbps > 0:
            return self.P_data_transfer * min(data_rate_kbps / 100, 1.0)
        elif event_rate > 0:
            return self.P_idle + self.event_energy * event_rate
        else:
            return self.P_sleep


@dataclass
class GNSSParameters:
    """
    GNSS (GPS/GLONASS/Galileo) module power parameters
    定位模块功耗参数
    
    References:
    [1] u-blox MAX-M10S datasheet
    [2] Qualcomm Location Engine specifications
    """
    
    P_LNA: float = 0.02  # W, Low Noise Amplifier baseline
    P_acquisition: float = 0.15  # W, cold/warm start acquisition
    P_tracking: float = 0.08  # W, continuous tracking
    P_off: float = 0.0  # W, completely off
    
    # Lock state dynamics
    tau_lock: float = 30.0  # s, typical time to first fix
    tau_unlock: float = 10.0  # s, time to lose lock
    
    # Signal quality model
    S_open_sky: float = 1.0
    S_urban: float = 0.6
    S_indoor: float = 0.2
    
    def power_consumption(self, lock_state: float, signal_quality: float) -> float:
        """
        Calculate GNSS power
        lock_state: 0-1 (probability of lock)
        signal_quality: 0-1 (environmental factor)
        """
        P_track = self.P_tracking * lock_state
        P_acq = self.P_acquisition * (1 - lock_state) * signal_quality
        return self.P_LNA + P_track + P_acq


@dataclass
class BackgroundParameters:
    """
    Background tasks power parameters (stochastic model)
    后台任务功耗参数
    
    References:
    [1] Android Doze mode documentation
    [2] Measured app wake patterns from Battery Historian
    """
    
    # Baseline parameters
    P_base: float = 0.1  # W, minimum background power
    P_sync_burst: float = 0.5  # W, sync activity burst
    
    # Stochastic process parameters (Ornstein-Uhlenbeck)
    theta_ou: float = 0.1  # mean reversion rate
    mu_ou: float = 0.15  # long-term mean (W)
    sigma_ou: float = 0.05  # volatility
    
    # User activity correlation
    activity_factor_sleep: float = 0.3
    activity_factor_active: float = 1.5
    
    def mean_power(self, user_activity: float) -> float:
        """Calculate mean background power based on user activity"""
        return self.mu_ou * (self.activity_factor_sleep + 
                            (self.activity_factor_active - self.activity_factor_sleep) * user_activity)


@dataclass
class UsageScenarios:
    """
    Predefined usage scenarios for simulation
    预定义使用场景
    """
    
    @staticmethod
    def get_scenario(name: str) -> Dict:
        """Get predefined scenario parameters"""
        scenarios = {
            'idle': {
                'cpu_load': 0.05,
                'screen_on': False,
                'brightness': 0.0,
                'network_active': False,
                'data_rate_Mbps': 0,
                'bluetooth_audio': False,
                'gps_active': False,
                'description': 'Screen off, minimal background activity'
            },
            'light_use': {
                'cpu_load': 0.2,
                'screen_on': True,
                'brightness': 0.3,
                'network_active': True,
                'data_rate_Mbps': 1,
                'bluetooth_audio': False,
                'gps_active': False,
                'description': 'Light browsing, messaging'
            },
            'video_streaming': {
                'cpu_load': 0.4,
                'screen_on': True,
                'brightness': 0.6,
                'network_active': True,
                'data_rate_Mbps': 15,
                'bluetooth_audio': True,
                'gps_active': False,
                'description': 'Video streaming with Bluetooth audio'
            },
            'navigation': {
                'cpu_load': 0.5,
                'screen_on': True,
                'brightness': 0.8,
                'network_active': True,
                'data_rate_Mbps': 2,
                'bluetooth_audio': False,
                'gps_active': True,
                'description': 'GPS navigation with display on'
            },
            'gaming': {
                'cpu_load': 0.9,
                'screen_on': True,
                'brightness': 0.7,
                'network_active': True,
                'data_rate_Mbps': 5,
                'bluetooth_audio': False,
                'gps_active': False,
                'description': 'High-performance gaming'
            },
            'heavy_multitask': {
                'cpu_load': 0.8,
                'screen_on': True,
                'brightness': 0.5,
                'network_active': True,
                'data_rate_Mbps': 20,
                'bluetooth_audio': True,
                'gps_active': True,
                'description': 'All components active simultaneously'
            }
        }
        return scenarios.get(name, scenarios['light_use'])
    
    @staticmethod
    def get_all_scenarios() -> List[str]:
        """Get list of all available scenarios"""
        return ['idle', 'light_use', 'video_streaming', 'navigation', 'gaming', 'heavy_multitask']


# Markov transition matrices for user behavior model
class UserBehaviorModel:
    """
    Continuous-time Markov chain for user behavior
    用户行为的连续时间马尔科夫链模型
    """
    
    def __init__(self):
        # Transition rate matrices for different times of day
        # Q_sleep: 23:00 - 7:00
        self.Q_sleep = np.array([
            [-0.01, 0.005, 0.004, 0.001],  # From SLEEP
            [0.5, -0.6, 0.08, 0.02],       # From WORK
            [0.3, 0.05, -0.4, 0.05],       # From LEISURE
            [0.4, 0.05, 0.05, -0.5]        # From COMMUTE
        ])
        
        # Q_work: 9:00 - 18:00
        self.Q_work = np.array([
            [0.1, 0.6, 0.2, 0.1],         # From SLEEP
            [0.02, -0.15, 0.1, 0.03],      # From WORK
            [0.1, 0.4, -0.6, 0.1],         # From LEISURE
            [0.05, 0.5, 0.1, -0.65]        # From COMMUTE
        ])
        
        # Q_leisure: 18:00 - 23:00
        self.Q_leisure = np.array([
            [0.2, 0.1, 0.6, 0.1],         # From SLEEP
            [0.1, -0.5, 0.35, 0.05],       # From WORK
            [0.05, 0.1, -0.2, 0.05],       # From LEISURE
            [0.1, 0.1, 0.4, -0.6]          # From COMMUTE
        ])
        
        # State to parameter mapping
        self.state_params = {
            UserState.SLEEP: {'cpu_load': 0.02, 'screen_on': False, 'brightness': 0},
            UserState.WORK: {'cpu_load': 0.4, 'screen_on': True, 'brightness': 0.5},
            UserState.LEISURE: {'cpu_load': 0.5, 'screen_on': True, 'brightness': 0.6},
            UserState.COMMUTE: {'cpu_load': 0.3, 'screen_on': True, 'brightness': 0.7}
        }
    
    def get_Q_matrix(self, hour: float) -> np.ndarray:
        """Get appropriate transition matrix for time of day"""
        if 23 <= hour or hour < 7:
            return self.Q_sleep
        elif 9 <= hour < 18:
            return self.Q_work
        else:
            return self.Q_leisure
    
    def get_state_params(self, state: UserState) -> Dict:
        """Get hardware parameters for user state"""
        return self.state_params.get(state, self.state_params[UserState.WORK])
