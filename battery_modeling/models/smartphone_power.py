"""
Multi-Physics Power Consumption Models for Smartphone Subsystems

This module implements physics-based models for:
- 5G Communication (Link budget with Shannon capacity)
- Bluetooth/BLE (Discrete event-driven model)
- Background Tasks (Poisson process with tail energy)
- GNSS (State machine with hysteresis)
- OLED Display (Content-aware APL model)
- SoC/CPU (DVFS with thermal coupling)
"""

import numpy as np
from scipy.special import erfc
from dataclasses import dataclass
from typing import Optional, List, Tuple
import warnings


# =============================================================================
# 5G Communication Subsystem
# =============================================================================

@dataclass
class FiveGParameters:
    """5G NR physical layer parameters"""
    bandwidth: float = 100e6        # Channel bandwidth (Hz)
    path_loss_exp: float = 3.8      # Path loss exponent (urban)
    noise_figure: float = 5.0       # Receiver noise figure (dB)
    system_temp: float = 290        # System noise temperature (K)
    tx_antenna_gain: float = 0.0    # Transmitter antenna gain (dBi)
    rx_antenna_gain: float = 2.0    # Receiver antenna gain (dBi)
    carrier_freq: float = 3.5e9    # Carrier frequency (Hz)
    shadow_factor: float = 0.8      # Shadow fading factor
    
    # Power amplifier characteristics
    pa_max_efficiency: float = 0.35  # Maximum PA efficiency
    pa_min_efficiency: float = 0.05  # Minimum PA efficiency (backoff)
    pa_transition_power: float = 0.1 # PA transition point (W)
    
    # Base power consumption
    baseband_static: float = 0.8     # Static baseband power (W)
    baseband_rate_coeff: float = 1e-9  # Power per bps
    rf_static: float = 0.4           # Static RF power (W)


class FiveGModel:
    """5G communication power consumption model based on link budget"""
    
    def __init__(self, params: Optional[FiveGParameters] = None):
        self.params = params or FiveGParameters()
        self.k_B = 1.38e-23  # Boltzmann constant
        
    def required_snr(self, rate: float) -> float:
        """
        Calculate required SNR for given data rate (Shannon)
        SNR_req = 2^(R/B) - 1
        
        Args:
            rate: Data rate (bps)
        Returns:
            Required SNR (linear)
        """
        B = self.params.bandwidth
        return 2**(rate / B) - 1
    
    def noise_power(self) -> float:
        """
        Calculate receiver thermal noise floor
        P_noise = k_B * T_sys * B * F
        """
        p = self.params
        F_linear = 10**(p.noise_figure / 10)
        return self.k_B * p.system_temp * p.bandwidth * F_linear
    
    def required_rx_power(self, rate: float) -> float:
        """
        Calculate required received power
        P_rx = P_noise * SNR_req
        """
        snr_req = self.required_snr(rate)
        return self.noise_power() * snr_req
    
    def path_loss(self, distance: float) -> float:
        """
        Calculate path loss using modified Friis equation
        PL = (4*pi*d/lambda)^n * shadow_factor
        
        Args:
            distance: Distance to base station (m)
        Returns:
            Path loss (linear)
        """
        p = self.params
        c = 3e8
        wavelength = c / p.carrier_freq
        
        # Free space path loss with shadowing
        if distance < 1:
            distance = 1  # Minimum distance
            
        fspl = (4 * np.pi * distance / wavelength)**p.path_loss_exp
        return fspl / p.shadow_factor
    
    def required_tx_power(self, rate: float, distance: float) -> float:
        """
        Calculate required transmit power from simplified link budget
        Uses empirical smartphone TX power model
        
        Args:
            rate: Data rate (bps)
            distance: Distance to base station (m)
        Returns:
            Required TX power (W)
        """
        p = self.params
        
        # Simplified model: TX power scales with distance and rate
        # Base TX power at reference point (200m, 100Mbps)
        P_tx_ref = 0.1  # 100mW reference
        d_ref = 200
        R_ref = 100e6
        
        # Distance scaling (simplified path loss model)
        distance_factor = (distance / d_ref) ** (p.path_loss_exp / 2)
        
        # Rate scaling (approximated from Shannon)
        rate_factor = np.log2(1 + rate / R_ref)
        
        P_tx = P_tx_ref * distance_factor * rate_factor
        
        # Clamp to realistic smartphone TX power range (0.01W to 1W)
        return np.clip(P_tx, 0.01, 1.0)
    
    def pa_efficiency(self, P_tx: float) -> float:
        """
        Non-linear PA efficiency model
        Efficiency drops significantly at low power (backoff)
        """
        p = self.params
        # Sigmoid-like transition
        x = np.log10(P_tx / p.pa_transition_power + 1e-10)
        eta = p.pa_min_efficiency + (p.pa_max_efficiency - p.pa_min_efficiency) * \
              (1 / (1 + np.exp(-2 * x)))
        return np.clip(eta, p.pa_min_efficiency, p.pa_max_efficiency)
    
    def total_power(self, rate: float, distance: float) -> float:
        """
        Total 5G subsystem power consumption
        P_5G = P_static + alpha * R + P_tx / eta_PA
        
        Args:
            rate: Data rate (bps)
            distance: Distance to base station (m)
        Returns:
            Total power consumption (W)
        """
        p = self.params
        
        # Baseband power
        P_baseband = p.baseband_static + p.baseband_rate_coeff * rate
        
        # RF power
        P_tx = self.required_tx_power(rate, distance)
        eta_PA = self.pa_efficiency(P_tx)
        P_rf = p.rf_static + P_tx / eta_PA
        
        return P_baseband + P_rf
    
    def power_surface(self, rates: np.ndarray, distances: np.ndarray) -> np.ndarray:
        """Generate power consumption surface for visualization"""
        R, D = np.meshgrid(rates, distances)
        P = np.zeros_like(R)
        
        for i in range(R.shape[0]):
            for j in range(R.shape[1]):
                P[i, j] = self.total_power(R[i, j], D[i, j])
                
        return R, D, P


# =============================================================================
# Bluetooth/BLE Subsystem
# =============================================================================

@dataclass
class BluetoothParameters:
    """Bluetooth Low Energy parameters"""
    # Current levels (mA)
    I_rx: float = 7.8          # Receive current
    I_tx: float = 8.2          # Transmit current (0 dBm)
    I_sleep: float = 0.0025    # Deep sleep current
    I_cpu: float = 3.0         # Protocol processing current
    
    # Timing (ms)
    t_rx: float = 2.0          # Receive window
    t_pre: float = 0.5         # Pre-processing time
    t_post: float = 0.5        # Post-processing time
    t_tx_base: float = 0.5     # Base transmit time
    t_tx_per_byte: float = 0.008  # Additional time per byte
    
    # Battery voltage
    V_bat: float = 3.7         # Nominal battery voltage (V)
    
    # Audio streaming (A2DP)
    P_rf_base: float = 25      # RF base power (mW)
    codec_factor: float = 0.5  # Codec complexity factor


class BluetoothModel:
    """Bluetooth/BLE discrete event-driven power model"""
    
    def __init__(self, params: Optional[BluetoothParameters] = None):
        self.params = params or BluetoothParameters()
        
    def event_charge(self, payload_bytes: int) -> float:
        """
        Calculate charge consumed by single connection event
        Q_event = I_pre*t_pre + I_rx*t_rx + I_tx*t_tx(L) + I_post*t_post
        
        Args:
            payload_bytes: Payload size in bytes
        Returns:
            Charge in Coulombs
        """
        p = self.params
        
        # Calculate transmit time
        t_tx = p.t_tx_base + p.t_tx_per_byte * payload_bytes
        
        # Total charge (convert mA*ms to Coulombs)
        Q = (p.I_cpu * p.t_pre + 
             p.I_rx * p.t_rx + 
             p.I_tx * t_tx + 
             p.I_cpu * p.t_post) * 1e-6  # mA * ms = μC
        
        return Q
    
    def average_current(self, conn_interval: float, payload_bytes: int = 20) -> float:
        """
        Calculate average current for BLE connection
        I_BLE = (Q_event + I_sleep * (tau - T_active)) / tau
        
        Args:
            conn_interval: Connection interval (ms)
            payload_bytes: Payload size
        Returns:
            Average current (mA)
        """
        p = self.params
        
        Q_event = self.event_charge(payload_bytes)
        T_active = p.t_pre + p.t_rx + p.t_tx_base + p.t_tx_per_byte * payload_bytes + p.t_post
        
        # Hyperbolic relationship
        I_avg = p.I_sleep + Q_event * 1e6 / conn_interval  # Convert to mA
        
        return I_avg
    
    def ble_power(self, conn_interval: float, payload_bytes: int = 20) -> float:
        """
        BLE mode power consumption
        
        Args:
            conn_interval: Connection interval (ms)
            payload_bytes: Payload size
        Returns:
            Average power (mW)
        """
        p = self.params
        I_avg = self.average_current(conn_interval, payload_bytes)
        return I_avg * p.V_bat
    
    def audio_power(self, sample_rate: float = 44100, bit_depth: int = 16) -> float:
        """
        A2DP audio streaming power
        P_audio = P_rf_base + kappa * Fs * D_depth
        
        Args:
            sample_rate: Audio sample rate (Hz)
            bit_depth: Bit depth
        Returns:
            Power consumption (mW)
        """
        p = self.params
        return p.P_rf_base + p.codec_factor * sample_rate * bit_depth * 1e-6
    
    def total_power(self, mode: str, **kwargs) -> float:
        """
        Total Bluetooth power based on mode
        
        Args:
            mode: 'ble', 'audio', or 'idle'
            **kwargs: Mode-specific parameters
        Returns:
            Power (mW)
        """
        if mode == 'ble':
            return self.ble_power(
                kwargs.get('conn_interval', 100),
                kwargs.get('payload_bytes', 20)
            )
        elif mode == 'audio':
            return self.audio_power(
                kwargs.get('sample_rate', 44100),
                kwargs.get('bit_depth', 16)
            )
        else:
            return self.params.I_sleep * self.params.V_bat


# =============================================================================
# Background Tasks Subsystem
# =============================================================================

@dataclass
class BackgroundParameters:
    """Background task parameters"""
    # Power levels (mW)
    P_leak: float = 2.0            # Deep sleep power
    P_idle: float = 150.0          # Idle-high power
    P_proc: float = 500.0          # Processing power
    
    # Tail times (s)
    tau_tail_cell: float = 12.0    # Cellular tail time
    tau_tail_wifi: float = 0.25    # WiFi tail time
    
    # Task parameters
    t_proc_mean: float = 0.1       # Mean processing time (s)
    t_proc_std: float = 0.05       # Std of processing time


class BackgroundModel:
    """Background task power model with Poisson arrivals"""
    
    def __init__(self, params: Optional[BackgroundParameters] = None):
        self.params = params or BackgroundParameters()
        
    def sleep_probability(self, wake_rate: float, tail_time: float) -> float:
        """
        Probability of successfully entering sleep state
        P_sleep = exp(-lambda * tau_tail)
        
        Args:
            wake_rate: Wake-up rate (events/minute)
            tail_time: Tail time (s)
        Returns:
            Sleep probability
        """
        lambda_per_sec = wake_rate / 60
        return np.exp(-lambda_per_sec * tail_time)
    
    def average_power(self, wake_rate: float, network: str = 'cell') -> float:
        """
        Average background power with tail energy
        P_bg = P_leak + (P_idle - P_leak) * (1 - exp(-lambda * tau_tail))
        
        Args:
            wake_rate: Wake-up rate (events/minute)
            network: 'cell' or 'wifi'
        Returns:
            Average power (mW)
        """
        p = self.params
        tau_tail = p.tau_tail_cell if network == 'cell' else p.tau_tail_wifi
        
        P_sleep = self.sleep_probability(wake_rate, tau_tail)
        return p.P_leak + (p.P_idle - p.P_leak) * (1 - P_sleep)
    
    def simulate_random_current(self, duration: float, wake_rate: float, 
                                 dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulate random wake-up events with tail energy
        
        Args:
            duration: Simulation duration (s)
            wake_rate: Wake-up rate (events/minute)
            dt: Time step (s)
        Returns:
            time, current arrays
        """
        p = self.params
        lambda_per_sec = wake_rate / 60
        
        time = np.arange(0, duration, dt)
        current = np.ones_like(time) * p.P_leak / 3.7  # Base sleep current (mA)
        
        # Generate Poisson events
        np.random.seed(42)
        n_events = np.random.poisson(lambda_per_sec * duration)
        event_times = np.sort(np.random.uniform(0, duration, n_events))
        
        # Apply tail energy for each event
        for t_event in event_times:
            # Processing spike
            proc_duration = max(0.01, np.random.normal(p.t_proc_mean, p.t_proc_std))
            mask_proc = (time >= t_event) & (time < t_event + proc_duration)
            current[mask_proc] = p.P_proc / 3.7
            
            # Tail period
            mask_tail = (time >= t_event + proc_duration) & \
                       (time < t_event + proc_duration + p.tau_tail_cell)
            current[mask_tail] = np.maximum(current[mask_tail], p.P_idle / 3.7)
        
        return time, current


# =============================================================================
# GNSS Subsystem
# =============================================================================

@dataclass
class GNSSParameters:
    """GNSS receiver parameters"""
    P_acq: float = 115.0       # Acquisition mode power (mW)
    P_track: float = 45.0      # Tracking mode power (mW)
    P_lna: float = 10.0        # LNA static power (mW)
    
    S_threshold: float = 25.0  # Lock threshold (dB-Hz)
    alpha_transition: float = 0.5  # Transition steepness
    tau_react: float = 2.5     # Reaction time constant (s)


class GNSSModel:
    """GNSS power model with environment-aware state machine"""
    
    def __init__(self, params: Optional[GNSSParameters] = None):
        self.params = params or GNSSParameters()
        self.x_lock = 1.0  # Initial lock state
        
    def lock_probability(self, S_env: float) -> float:
        """
        Sigmoid lock probability based on signal quality
        Psi = 1 / (1 + exp(-alpha * (S_env - S_th)))
        
        Args:
            S_env: Environment signal strength (dB-Hz)
        Returns:
            Lock probability
        """
        p = self.params
        return 1 / (1 + np.exp(-p.alpha_transition * (S_env - p.S_threshold)))
    
    def update_lock_state(self, S_env: float, dt: float) -> float:
        """
        Update lock state with hysteresis dynamics
        dx/dt = (1/tau) * (Psi - x)
        
        Args:
            S_env: Environment signal strength
            dt: Time step (s)
        Returns:
            Updated lock state
        """
        p = self.params
        psi = self.lock_probability(S_env)
        dx = (psi - self.x_lock) / p.tau_react * dt
        self.x_lock = np.clip(self.x_lock + dx, 0, 1)
        return self.x_lock
    
    def instantaneous_power(self, S_env: float = None) -> float:
        """
        GNSS power based on current lock state
        P_GNSS = P_LNA + x*P_track + (1-x)*P_acq
        
        Args:
            S_env: If provided, update lock state first
        Returns:
            Power (mW)
        """
        p = self.params
        return p.P_lna + self.x_lock * p.P_track + (1 - self.x_lock) * p.P_acq
    
    def simulate_tunnel_passage(self, duration: float, dt: float = 0.1) -> dict:
        """
        Simulate passage through tunnel/urban canyon
        
        Returns:
            Dictionary with time, signal, power, lock_state
        """
        time = np.arange(0, duration, dt)
        n = len(time)
        
        # Create signal profile (open -> tunnel -> canyon -> open)
        S_env = np.ones(n) * 40  # Strong signal
        
        # Tunnel (t=10-30s)
        tunnel_mask = (time >= 10) & (time < 30)
        S_env[tunnel_mask] = 5  # Very weak signal
        
        # Urban canyon (t=40-55s)
        canyon_mask = (time >= 40) & (time < 55)
        S_env[canyon_mask] = 20 + 10 * np.sin(np.linspace(0, 4*np.pi, np.sum(canyon_mask)))
        
        # Reset state and simulate
        self.x_lock = 1.0
        power = np.zeros(n)
        lock_state = np.zeros(n)
        
        for i, t in enumerate(time):
            self.update_lock_state(S_env[i], dt)
            power[i] = self.instantaneous_power()
            lock_state[i] = self.x_lock
            
        return {
            'time': time,
            'signal': S_env,
            'power': power,
            'lock_state': lock_state
        }


# =============================================================================
# OLED Display Subsystem
# =============================================================================

@dataclass
class OLEDParameters:
    """OLED display parameters"""
    # Panel parameters
    width: int = 1440           # Resolution width
    height: int = 3200          # Resolution height
    gamma: float = 2.2          # Gamma correction
    
    # Power weights for RGB (blue is least efficient)
    w_R: float = 0.25
    w_G: float = 0.35
    w_B: float = 0.40
    
    # Power model coefficients
    P_static: float = 65.0      # Static base power (mW)
    C_drv: float = 1.25         # Driver coefficient (mW/Hz)
    beta_panel: float = 3.8     # Panel emission coefficient (mW/nit)
    
    # Brightness
    L_max: float = 1000         # Maximum brightness (nits)
    
    # LTPO refresh rates
    f_min: float = 1.0          # Minimum refresh (Hz)
    f_max: float = 120.0        # Maximum refresh (Hz)


class OLEDModel:
    """OLED display power model with APL and LTPO"""
    
    def __init__(self, params: Optional[OLEDParameters] = None):
        self.params = params or OLEDParameters()
        
    def calculate_apl(self, R: np.ndarray, G: np.ndarray, B: np.ndarray) -> float:
        """
        Calculate weighted Average Pixel Level
        APL = (1/N) * sum(w_R*(R/255)^gamma + w_G*(G/255)^gamma + w_B*(B/255)^gamma)
        
        Args:
            R, G, B: Arrays of pixel values (0-255)
        Returns:
            APL value (0-1)
        """
        p = self.params
        N = len(R)
        
        apl = (p.w_R * np.power(R / 255, p.gamma) +
               p.w_G * np.power(G / 255, p.gamma) +
               p.w_B * np.power(B / 255, p.gamma))
        
        return np.mean(apl)
    
    def emission_power(self, brightness: float, apl: float) -> float:
        """
        Calculate pixel emission power
        P_emit = beta * (L/L_max) * APL
        
        Args:
            brightness: Set brightness (nits)
            apl: Average pixel level
        Returns:
            Emission power (mW)
        """
        p = self.params
        return p.beta_panel * (brightness / p.L_max) * apl * brightness
    
    def driver_power(self, refresh_rate: float) -> float:
        """
        Calculate driver circuit power
        P_driver = C_eff * Vdd^2 * f
        """
        p = self.params
        return p.C_drv * refresh_rate
    
    def total_power(self, brightness: float, apl: float, refresh_rate: float) -> float:
        """
        Total OLED display power
        P_disp = P_static + C*f + beta*(L/L_max)*APL*L
        
        Args:
            brightness: Screen brightness (nits)
            apl: Average pixel level
            refresh_rate: Display refresh rate (Hz)
        Returns:
            Total power (mW)
        """
        p = self.params
        
        P_static = p.P_static
        P_driver = self.driver_power(refresh_rate)
        P_emit = self.emission_power(brightness, apl)
        
        return P_static + P_driver + P_emit
    
    def compare_themes(self, brightness: float = 500, refresh_rate: float = 60) -> dict:
        """
        Compare light vs dark theme power consumption
        """
        # Light theme: bright background
        apl_light = 0.85
        P_light = self.total_power(brightness, apl_light, refresh_rate)
        
        # Dark theme: dark background
        apl_dark = 0.15
        P_dark = self.total_power(brightness, apl_dark, refresh_rate)
        
        savings = (P_light - P_dark) / P_light * 100
        
        return {
            'light_power_mW': P_light,
            'dark_power_mW': P_dark,
            'savings_percent': savings
        }


# =============================================================================
# SoC/CPU Subsystem
# =============================================================================

@dataclass
class SoCParameters:
    """SoC (System-on-Chip) parameters - calibrated for realistic smartphone power"""
    # DVFS parameters
    V_th: float = 0.35          # Threshold voltage (V)
    kappa_dvfs: float = 5e-28   # DVFS scaling coefficient (calibrated)
    alpha_act: float = 0.25     # Activity factor
    C_eff: float = 100e-12      # Effective capacitance (F)
    
    # Leakage parameters (BSIM4) - calibrated for 7nm
    I_ref: float = 50e-3        # Reference leakage current (A)
    T_ref: float = 298          # Reference temperature (K)
    lambda_DIBL: float = 0.08   # DIBL coefficient
    zeta: float = 0.025         # Temperature sensitivity
    n_ideality: float = 1.3     # Ideality factor
    
    # Thermal parameters
    C_th: float = 8.0           # Thermal capacitance (J/K)
    R_th: float = 5.0           # Thermal resistance (K/W)
    
    # Operating limits
    f_min: float = 300e6        # Minimum frequency (Hz)
    f_max: float = 3.0e9        # Maximum frequency (Hz)
    V_min: float = 0.55         # Minimum voltage (V)
    V_max: float = 1.05         # Maximum voltage (V)
    T_max: float = 85           # Maximum junction temperature (°C)
    
    # Power scaling parameters (empirical)
    P_base: float = 0.15        # Base power at idle (W)
    P_scale: float = 2.5        # Power scaling factor


class SoCModel:
    """SoC power model with electro-thermal coupling"""
    
    def __init__(self, params: Optional[SoCParameters] = None):
        self.params = params or SoCParameters()
        self.k_B = 1.38e-23  # Boltzmann constant
        self.q = 1.6e-19     # Electron charge
        
    def frequency_to_voltage(self, freq: float) -> float:
        """
        Calculate required voltage for given frequency (Alpha-Power Law)
        V_dd = V_th + k * f^(1/alpha)
        """
        p = self.params
        # Simplified relationship
        f_norm = (freq - p.f_min) / (p.f_max - p.f_min)
        V_dd = p.V_min + (p.V_max - p.V_min) * np.power(f_norm, 0.6)
        return np.clip(V_dd, p.V_min, p.V_max)
    
    def dynamic_power(self, freq: float) -> float:
        """
        Dynamic power consumption using empirical smartphone model
        P_dyn ~ P_base + P_scale * (f/f_max)^2.5
        
        Args:
            freq: Operating frequency (Hz)
        Returns:
            Dynamic power (W)
        """
        p = self.params
        V_dd = self.frequency_to_voltage(freq)
        
        # Normalized frequency
        f_norm = (freq - p.f_min) / (p.f_max - p.f_min)
        f_norm = np.clip(f_norm, 0, 1)
        
        # Empirical power model (calibrated for smartphone SoC)
        # Power scales approximately as f^2.5 due to V-f coupling
        P_dyn = p.P_base + p.P_scale * np.power(f_norm, 2.5)
        
        return P_dyn
    
    def leakage_power(self, V_dd: float, T: float) -> float:
        """
        Static leakage power using simplified empirical model
        Calibrated for 7nm smartphone SoC
        
        Args:
            V_dd: Supply voltage (V)
            T: Junction temperature (°C)
        Returns:
            Leakage power (W)
        """
        # Base leakage at 25°C, nominal voltage
        P_leak_base = 0.05  # 50mW base leakage
        
        # Temperature scaling: approximately doubles every 10°C
        T_ref = 25.0
        temp_factor = np.power(2.0, (T - T_ref) / 10.0)
        
        # Voltage scaling: approximately quadratic with voltage
        V_ref = 0.8
        voltage_factor = (V_dd / V_ref) ** 2
        
        P_leak = P_leak_base * temp_factor * voltage_factor
        
        # Clamp to realistic range (0.02W to 1.5W)
        return np.clip(P_leak, 0.02, 1.5)
    
    def total_power(self, freq: float, T_junction: float) -> float:
        """
        Total SoC power
        P_SoC = P_dyn + P_leak
        
        Args:
            freq: Operating frequency (Hz)
            T_junction: Junction temperature (°C)
        Returns:
            Total power (W)
        """
        V_dd = self.frequency_to_voltage(freq)
        P_dyn = self.dynamic_power(freq)
        P_leak = self.leakage_power(V_dd, T_junction)
        
        return P_dyn + P_leak
    
    def thermal_dynamics(self, P_total: float, T_current: float, 
                         T_ambient: float, dt: float) -> float:
        """
        Update junction temperature
        C_th * dT/dt = P_total - (T - T_amb) / R_th
        
        Args:
            P_total: Total power dissipation (W)
            T_current: Current junction temperature (°C)
            T_ambient: Ambient temperature (°C)
            dt: Time step (s)
        Returns:
            New junction temperature (°C)
        """
        p = self.params
        dT = (P_total - (T_current - T_ambient) / p.R_th) / p.C_th * dt
        return T_current + dT
    
    def simulate_workload(self, freq_profile: np.ndarray, T_ambient: float = 25,
                          dt: float = 1.0) -> dict:
        """
        Simulate SoC under varying workload
        
        Args:
            freq_profile: Array of frequencies (Hz)
            T_ambient: Ambient temperature (°C)
            dt: Time step (s)
        Returns:
            Dictionary of results
        """
        n = len(freq_profile)
        
        T_junction = np.zeros(n)
        P_total = np.zeros(n)
        P_dynamic = np.zeros(n)
        P_leakage = np.zeros(n)
        V_dd = np.zeros(n)
        
        T_junction[0] = T_ambient
        
        for i in range(n):
            freq = freq_profile[i]
            V_dd[i] = self.frequency_to_voltage(freq)
            P_dynamic[i] = self.dynamic_power(freq)
            P_leakage[i] = self.leakage_power(V_dd[i], T_junction[i])
            P_total[i] = P_dynamic[i] + P_leakage[i]
            
            if i < n - 1:
                T_junction[i+1] = self.thermal_dynamics(
                    P_total[i], T_junction[i], T_ambient, dt)
                # Thermal throttling
                if T_junction[i+1] > self.params.T_max:
                    T_junction[i+1] = self.params.T_max
                    
        return {
            'T_junction': T_junction,
            'P_total': P_total,
            'P_dynamic': P_dynamic,
            'P_leakage': P_leakage,
            'V_dd': V_dd
        }


# =============================================================================
# System Integration
# =============================================================================

class SmartphonePowerModel:
    """Integrated smartphone power consumption model"""
    
    def __init__(self):
        self.fiveg = FiveGModel()
        self.bluetooth = BluetoothModel()
        self.background = BackgroundModel()
        self.gnss = GNSSModel()
        self.oled = OLEDModel()
        self.soc = SoCModel()
        
    def total_power(self, 
                    # 5G parameters
                    data_rate: float = 0,
                    cell_distance: float = 500,
                    # Bluetooth parameters
                    bt_mode: str = 'idle',
                    # Background parameters
                    wake_rate: float = 1.0,
                    # GNSS parameters
                    gps_active: bool = False,
                    signal_quality: float = 35,
                    # Display parameters
                    screen_on: bool = True,
                    brightness: float = 300,
                    apl: float = 0.5,
                    refresh_rate: float = 60,
                    # SoC parameters
                    cpu_freq: float = 1e9,
                    T_junction: float = 40) -> dict:
        """
        Calculate total system power consumption
        
        Returns:
            Dictionary with power breakdown (all values in mW)
        """
        power = {}
        
        # 5G (already returns reasonable W values, convert to mW)
        if data_rate > 0:
            p_5g = self.fiveg.total_power(data_rate, cell_distance)
            power['5G'] = min(p_5g * 1000, 5000)  # Cap at 5W
        else:
            power['5G'] = 50  # Idle power (mW)
            
        # Bluetooth (returns mW)
        power['Bluetooth'] = self.bluetooth.total_power(bt_mode)
        
        # Background (returns mW)
        power['Background'] = self.background.average_power(wake_rate)
        
        # GNSS (returns mW)
        if gps_active:
            self.gnss.update_lock_state(signal_quality, 0.1)
            power['GNSS'] = self.gnss.instantaneous_power()
        else:
            power['GNSS'] = 5  # Minimal standby
            
        # Display (returns mW)
        if screen_on:
            power['Display'] = self.oled.total_power(brightness, apl, refresh_rate)
        else:
            power['Display'] = 5  # Minimal standby
            
        # SoC (returns W, convert to mW and cap)
        p_soc = self.soc.total_power(cpu_freq, T_junction)
        power['SoC'] = min(p_soc * 1000, 8000)  # Cap at 8W
        
        # Total
        power['Total'] = sum(power.values())
        
        return power


if __name__ == "__main__":
    # Quick test
    model = SmartphonePowerModel()
    
    power = model.total_power(
        data_rate=100e6,
        cell_distance=300,
        bt_mode='ble',
        wake_rate=2.0,
        gps_active=True,
        signal_quality=35,
        screen_on=True,
        brightness=400,
        apl=0.4,
        refresh_rate=90,
        cpu_freq=2e9,
        T_junction=45
    )
    
    print("Power Consumption Breakdown:")
    for component, p in power.items():
        print(f"  {component}: {p:.1f} mW")
