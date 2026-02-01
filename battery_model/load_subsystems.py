"""
Smart Terminal Multi-Physics Load Subsystem Models
===================================================

This module implements physics-based power consumption models for:
1. 5G Communication (Link budget + Shannon capacity)
2. Bluetooth/BLE (Discrete event + duty cycle)
3. Background Tasks (Poisson process + tail energy)
4. GNSS Navigation (State machine + hysteresis)
5. OLED Display (APL content-aware + LTPO)
6. SoC Processor (DVFS + thermal coupling)

Each model captures the underlying physical mechanisms that determine
power consumption in modern smartphones.

Author: Load Modeling Framework
Date: 2026-02-01
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional, Callable
from enum import Enum


# =============================================================================
# 5G Communication Subsystem Model
# =============================================================================

@dataclass
class Params5G:
    """Parameters for 5G communication power model."""
    
    # Channel parameters
    bandwidth: float = 100e6      # Channel bandwidth (Hz) - 100 MHz for 5G NR
    path_loss_exp: float = 3.8    # Path loss exponent (urban dense)
    noise_figure: float = 5.0     # Receiver noise figure (dB)
    system_temp: float = 290.0    # System noise temperature (K)
    shadow_fading: float = 0.5    # Shadow fading factor
    
    # Antenna gains
    G_tx: float = 3.0             # Transmitter antenna gain (linear)
    G_rx: float = 3.0             # Receiver antenna gain (linear)
    
    # Carrier frequency
    f_carrier: float = 3.5e9     # Carrier frequency (Hz) - mid-band 5G
    
    # Power amplifier efficiency (will be interpolated)
    eta_PA_max: float = 0.35      # Max PA efficiency at saturation
    eta_PA_min: float = 0.05      # Min PA efficiency at backoff
    
    # Baseband parameters
    P_static: float = 0.5         # Static power consumption (W)
    alpha_bb: float = 1e-9        # Baseband power per bit/s (W·s/bit)
    
    # Max transmit power
    P_tx_max: float = 0.2         # Max transmit power (W)
    
    # Physical constants
    k_B: float = 1.38e-23         # Boltzmann constant (J/K)
    c: float = 3e8                # Speed of light (m/s)


class Model5G:
    """
    5G Communication Power Model based on Link Budget and Shannon Capacity.
    
    Key equations:
    - Shannon: R = B * log2(1 + SNR_req)
    - Required SNR: SNR_req = 2^(R/B) - 1
    - Link budget: P_tx = Lambda_env * d^n * SNR_req
    - Total power: P_5G = P_static + alpha_bb*R + P_tx/eta_PA
    """
    
    def __init__(self, params: Optional[Params5G] = None):
        self.params = params if params else Params5G()
        self._compute_env_coefficient()
        
    def _compute_env_coefficient(self):
        """Compute integrated environmental channel coefficient."""
        p = self.params
        
        # Wavelength
        lambda_c = p.c / p.f_carrier
        
        # Noise power
        noise_factor = 10 ** (p.noise_figure / 10)
        P_noise = p.k_B * p.system_temp * p.bandwidth * noise_factor
        
        # Path loss factor
        path_factor = (4 * np.pi / lambda_c) ** p.path_loss_exp
        
        # Combined environmental coefficient
        self.Lambda_env = (path_factor * P_noise) / (p.G_tx * p.G_rx * p.shadow_fading)
        
    def get_PA_efficiency(self, P_tx: float) -> float:
        """
        Get power amplifier efficiency based on output power level.
        
        PA efficiency follows a nonlinear characteristic:
        - High efficiency near saturation
        - Low efficiency at backoff
        """
        p = self.params
        
        # Normalized power level
        P_norm = np.clip(P_tx / p.P_tx_max, 0.01, 1.0)
        
        # Efficiency model (cubic interpolation)
        eta = p.eta_PA_min + (p.eta_PA_max - p.eta_PA_min) * (P_norm ** 0.5)
        
        return eta
    
    def calculate_required_tx_power(self, data_rate: float, distance: float) -> float:
        """
        Calculate required transmit power for given data rate and distance.
        
        P_tx = Lambda_env * d^n * (2^(R/B) - 1)
        
        Args:
            data_rate: Target data rate (bits/s)
            distance: Distance to base station (m)
            
        Returns:
            Required transmit power (W)
        """
        p = self.params
        
        # Required SNR from Shannon capacity inversion
        snr_req = 2 ** (data_rate / p.bandwidth) - 1
        
        # Link budget equation
        P_tx = self.Lambda_env * (distance ** p.path_loss_exp) * snr_req
        
        # Clip to max power
        return np.clip(P_tx, 0, p.P_tx_max * 2)  # Allow some headroom
    
    def get_power(self, data_rate: float, distance: float) -> dict:
        """
        Calculate total 5G subsystem power consumption.
        
        P_5G = P_static + alpha_bb * R + P_tx / eta_PA
        
        Returns:
            Dictionary with power breakdown
        """
        p = self.params
        
        # Required transmit power
        P_tx = self.calculate_required_tx_power(data_rate, distance)
        
        # PA efficiency
        eta_PA = self.get_PA_efficiency(P_tx)
        
        # Power components
        P_baseband = p.alpha_bb * data_rate
        P_rf = P_tx / eta_PA if eta_PA > 0 else 0
        P_total = p.P_static + P_baseband + P_rf
        
        return {
            'P_total': P_total,
            'P_static': p.P_static,
            'P_baseband': P_baseband,
            'P_rf': P_rf,
            'P_tx': P_tx,
            'eta_PA': eta_PA,
            'SNR_req': 2 ** (data_rate / p.bandwidth) - 1
        }


# =============================================================================
# Bluetooth/BLE Subsystem Model
# =============================================================================

@dataclass
class ParamsBluetooth:
    """Parameters for Bluetooth power model."""
    
    # Current levels (mA)
    I_tx: float = 8.2             # TX current at 0dBm
    I_rx: float = 7.8             # RX current
    I_sleep: float = 0.0025       # Deep sleep current (2.5 μA)
    I_cpu: float = 5.0            # Protocol processing current
    
    # Timing parameters (ms)
    t_rx: float = 0.328           # RX window duration
    t_pre: float = 0.1            # Pre-processing time
    t_post: float = 0.1           # Post-processing time
    
    # TX time per byte
    t_tx_per_byte: float = 0.008  # 8 μs per byte at 1 Mbps
    
    # Audio mode parameters
    P_RF_base: float = 25.0       # RF base power for A2DP (mW)
    kappa_codec: float = 0.0003   # Codec complexity coefficient


class ModelBluetooth:
    """
    Bluetooth/BLE Power Model with Discrete Event Integration.
    
    BLE average current:
    I_BLE(τ) ≈ I_sleep + Q_event / τ
    
    Where Q_event = I_rx*t_rx + I_tx*t_tx(L) + I_cpu*(t_pre + t_post)
    """
    
    def __init__(self, params: Optional[ParamsBluetooth] = None):
        self.params = params if params else ParamsBluetooth()
        
    def calculate_event_charge(self, payload_bytes: int, tx_power_level: float = 0) -> float:
        """
        Calculate charge consumed by single connection event.
        
        Q_event = I_rx*t_rx + I_tx(P_out)*t_tx(L) + I_cpu*t_proc
        
        Args:
            payload_bytes: Payload size in bytes
            tx_power_level: TX power level in dBm
            
        Returns:
            Charge in μC (microCoulombs)
        """
        p = self.params
        
        # TX current adjustment for power level
        I_tx_adj = p.I_tx * (10 ** (tx_power_level / 20))
        
        # TX duration based on payload
        t_tx = p.t_tx_per_byte * payload_bytes
        
        # Total charge (convert mA*ms to μC)
        Q_rx = p.I_rx * p.t_rx
        Q_tx = I_tx_adj * t_tx
        Q_proc = p.I_cpu * (p.t_pre + p.t_post)
        
        Q_event = Q_rx + Q_tx + Q_proc  # in mA*ms = μC
        
        return Q_event
    
    def get_average_current(self, interval_ms: float, payload_bytes: int = 20) -> float:
        """
        Calculate average BLE current.
        
        I_BLE(τ) = I_sleep + Q_event / τ
        
        Args:
            interval_ms: Connection interval (ms)
            payload_bytes: Payload size
            
        Returns:
            Average current in mA
        """
        p = self.params
        
        Q_event = self.calculate_event_charge(payload_bytes)
        
        # Average current follows hyperbolic law
        I_avg = (p.I_sleep * 1000) + (Q_event / interval_ms)  # Convert sleep to μA then back
        
        return I_avg
    
    def get_audio_power(self, sample_rate: float = 44100, bit_depth: int = 16) -> float:
        """
        Calculate A2DP audio streaming power.
        
        P_audio = P_RF_base + κ_codec * F_s * D_depth
        
        Returns:
            Power in mW
        """
        p = self.params
        
        P_audio = p.P_RF_base + p.kappa_codec * sample_rate * bit_depth
        return P_audio
    
    def get_power(self, mode: str, V_bat: float = 3.7, **kwargs) -> dict:
        """
        Get Bluetooth power consumption for specified mode.
        
        Modes: 'ble', 'audio', 'idle'
        
        Returns:
            Dictionary with power breakdown
        """
        p = self.params
        
        if mode == 'ble':
            interval = kwargs.get('interval_ms', 100)
            payload = kwargs.get('payload_bytes', 20)
            I_avg = self.get_average_current(interval, payload)
            P_total = V_bat * I_avg / 1000  # Convert to W
            
            return {
                'P_total': P_total * 1000,  # mW
                'I_avg': I_avg,
                'Q_event': self.calculate_event_charge(payload),
                'mode': 'BLE'
            }
            
        elif mode == 'audio':
            sample_rate = kwargs.get('sample_rate', 44100)
            bit_depth = kwargs.get('bit_depth', 16)
            P_audio = self.get_audio_power(sample_rate, bit_depth)
            
            return {
                'P_total': P_audio,
                'mode': 'A2DP Audio'
            }
            
        else:  # idle
            P_idle = V_bat * p.I_sleep  # mW
            return {
                'P_total': P_idle,
                'I_avg': p.I_sleep * 1000,  # μA
                'mode': 'Idle'
            }


# =============================================================================
# Background Tasks Subsystem Model
# =============================================================================

@dataclass
class ParamsBackground:
    """Parameters for background task power model."""
    
    # Current levels (mA)
    I_leak: float = 0.5           # Deep sleep leakage current
    I_idle: float = 15.0          # Idle-high state current
    I_active: float = 150.0       # Active processing current
    
    # Tail time parameters (seconds)
    tau_tail_cell: float = 12.0   # Cellular tail time
    tau_tail_wifi: float = 0.25   # WiFi tail time
    
    # Process parameters
    t_proc_mean: float = 0.1      # Mean processing time (s)
    t_proc_std: float = 0.05      # Processing time std dev


class ModelBackground:
    """
    Background Task Power Model with Poisson Arrivals and Tail Energy.
    
    Average power with overlapping tail windows:
    P_bg(λ) ≈ P_leak + (P_idle - P_leak) * (1 - exp(-λ * τ_tail))
    """
    
    def __init__(self, params: Optional[ParamsBackground] = None):
        self.params = params if params else ParamsBackground()
        
    def get_sleep_probability(self, wakeup_rate: float, interface: str = 'cell') -> float:
        """
        Calculate probability of successfully entering sleep state.
        
        P_sleep = exp(-λ * τ_tail)
        """
        p = self.params
        
        tau_tail = p.tau_tail_cell if interface == 'cell' else p.tau_tail_wifi
        
        return np.exp(-wakeup_rate * tau_tail)
    
    def get_average_power(self, wakeup_rate: float, V_bat: float = 3.7,
                          interface: str = 'cell') -> dict:
        """
        Calculate average background power consumption.
        
        P_bg(λ) = P_leak + (P_idle - P_leak) * (1 - exp(-λ * τ_tail))
        
        Args:
            wakeup_rate: Wakeups per minute
            V_bat: Battery voltage (V)
            interface: 'cell' or 'wifi'
            
        Returns:
            Dictionary with power breakdown
        """
        p = self.params
        
        # Convert rate to per-second
        lambda_per_sec = wakeup_rate / 60.0
        
        # Sleep probability
        P_sleep = self.get_sleep_probability(lambda_per_sec, interface)
        
        # Power levels in mW
        P_leak = V_bat * p.I_leak
        P_idle = V_bat * p.I_idle
        P_active = V_bat * p.I_active
        
        # Average power with tail energy effect
        P_avg = P_leak + (P_idle - P_leak) * (1 - P_sleep)
        
        # Add active power contribution
        tau_tail = p.tau_tail_cell if interface == 'cell' else p.tau_tail_wifi
        duty_active = lambda_per_sec * p.t_proc_mean
        P_avg += duty_active * (P_active - P_idle)
        
        return {
            'P_total': P_avg,
            'P_leak': P_leak,
            'P_idle': P_idle,
            'P_sleep_prob': P_sleep,
            'duty_cycle': 1 - P_sleep,
            'interface': interface
        }
    
    def simulate_random_wakeups(self, duration: float, wakeup_rate: float,
                                 dt: float = 0.001) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulate random background wakeups using Poisson process.
        
        Returns:
            (time_array, current_array)
        """
        p = self.params
        
        n_steps = int(duration / dt)
        time = np.arange(n_steps) * dt
        current = np.full(n_steps, p.I_leak)
        
        # Generate Poisson arrivals
        lambda_per_sec = wakeup_rate / 60.0
        n_events = np.random.poisson(lambda_per_sec * duration)
        event_times = np.random.uniform(0, duration, n_events)
        event_times.sort()
        
        # Add processing and tail states
        tau_tail = p.tau_tail_cell
        
        for t_event in event_times:
            # Processing phase
            idx_start = int(t_event / dt)
            idx_proc_end = min(int((t_event + p.t_proc_mean) / dt), n_steps)
            idx_tail_end = min(int((t_event + p.t_proc_mean + tau_tail) / dt), n_steps)
            
            # Set currents
            current[idx_start:idx_proc_end] = np.maximum(
                current[idx_start:idx_proc_end], p.I_active)
            current[idx_proc_end:idx_tail_end] = np.maximum(
                current[idx_proc_end:idx_tail_end], p.I_idle)
        
        return time, current


# =============================================================================
# GNSS Navigation Subsystem Model
# =============================================================================

class GNSSState(Enum):
    """GNSS receiver states."""
    OFF = 0
    ACQUISITION = 1
    TRACKING = 2


@dataclass
class ParamsGNSS:
    """Parameters for GNSS power model."""
    
    # Power levels (mW)
    P_LNA: float = 8.0            # Low noise amplifier base power
    P_acq: float = 115.0          # Acquisition mode power
    P_track: float = 45.0         # Tracking mode power
    
    # State transition parameters
    S_th: float = 28.0            # Signal threshold (dB-Hz)
    alpha: float = 0.5            # Sigmoid steepness
    tau_react: float = 2.5        # Reaction time constant (s)
    
    # Acquisition timeout
    acq_timeout: float = 30.0     # Acquisition timeout (s)


class ModelGNSS:
    """
    GNSS Power Model with Environment-Aware State Machine.
    
    Lock state evolution:
    dx_lock/dt = (Ψ(S_env) - x_lock) / τ_react
    
    Where Ψ(S) = 1 / (1 + exp(-α(S - S_th)))
    
    Power output:
    P_GNSS = P_LNA + x_lock * P_track + (1 - x_lock) * P_acq
    """
    
    def __init__(self, params: Optional[ParamsGNSS] = None):
        self.params = params if params else ParamsGNSS()
        self.x_lock = 0.0  # Lock state probability
        
    def reset(self, locked: bool = False):
        """Reset lock state."""
        self.x_lock = 1.0 if locked else 0.0
        
    def get_lock_probability(self, S_env: float) -> float:
        """
        Calculate lock probability using Sigmoid function.
        
        Ψ(S) = 1 / (1 + exp(-α(S - S_th)))
        """
        p = self.params
        return 1.0 / (1.0 + np.exp(-p.alpha * (S_env - p.S_th)))
    
    def step(self, S_env: float, dt: float = 1.0) -> float:
        """
        Update lock state and return instantaneous power.
        
        State equation:
        dx_lock/dt = (Ψ(S_env) - x_lock) / τ_react
        """
        p = self.params
        
        # Target lock probability
        Psi = self.get_lock_probability(S_env)
        
        # State evolution (Forward Euler)
        dx_dt = (Psi - self.x_lock) / p.tau_react
        self.x_lock = np.clip(self.x_lock + dx_dt * dt, 0.0, 1.0)
        
        # Power calculation
        P = p.P_LNA + self.x_lock * p.P_track + (1 - self.x_lock) * p.P_acq
        
        return P
    
    def get_power(self, S_env: float) -> dict:
        """
        Get GNSS power for given environmental signal level.
        
        Returns:
            Dictionary with power breakdown
        """
        p = self.params
        P = self.step(S_env)
        
        return {
            'P_total': P,
            'P_LNA': p.P_LNA,
            'x_lock': self.x_lock,
            'state': 'Tracking' if self.x_lock > 0.5 else 'Acquisition',
            'S_env': S_env
        }
    
    def simulate_trajectory(self, S_env_profile: np.ndarray, dt: float = 1.0,
                            locked_init: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulate GNSS power over trajectory with varying signal conditions.
        
        Returns:
            (time, power) arrays
        """
        self.reset(locked_init)
        
        n_steps = len(S_env_profile)
        time = np.arange(n_steps) * dt
        power = np.zeros(n_steps)
        
        for i, S_env in enumerate(S_env_profile):
            power[i] = self.step(S_env, dt)
            
        return time, power


# =============================================================================
# OLED Display Subsystem Model
# =============================================================================

@dataclass
class ParamsOLED:
    """Parameters for OLED display power model."""
    
    # Display dimensions
    width: int = 1440             # Screen width (pixels)
    height: int = 3200            # Screen height (pixels)
    
    # Power parameters
    P_base: float = 65.0          # Static base power (mW)
    k_drv: float = 1.25           # Refresh rate coefficient (mW/Hz)
    beta_panel: float = 3.8       # Panel emissive coefficient (mW/nit)
    
    # Gamma correction
    gamma: float = 2.2            # Display gamma
    
    # Subpixel weights (RGB efficiency differences)
    w_R: float = 0.30             # Red weight
    w_G: float = 0.59             # Green weight (most efficient)
    w_B: float = 0.11             # Blue weight (least efficient)
    
    # Maximum parameters
    L_max: float = 1200.0         # Maximum brightness (nits)
    f_max: float = 120.0          # Maximum refresh rate (Hz)
    
    # Touch layer
    P_touch: float = 10.0         # Touch scanning power (mW)


class ModelOLED:
    """
    OLED Display Power Model with Content-Aware APL and LTPO.
    
    Total power:
    P_disp = P_base + k_drv * f_refresh + β_panel * Θ(L_set) * APL
    
    Where APL = weighted average of normalized RGB values
    """
    
    def __init__(self, params: Optional[ParamsOLED] = None):
        self.params = params if params else ParamsOLED()
        
    def calculate_APL(self, R: float, G: float, B: float) -> float:
        """
        Calculate Average Pixel Level from mean RGB values.
        
        APL = w_R*(R/255)^γ + w_G*(G/255)^γ + w_B*(B/255)^γ
        
        Args:
            R, G, B: Mean RGB values (0-255)
            
        Returns:
            APL value (0-1)
        """
        p = self.params
        
        # Normalize and apply gamma
        r_norm = (R / 255.0) ** p.gamma
        g_norm = (G / 255.0) ** p.gamma
        b_norm = (B / 255.0) ** p.gamma
        
        # Weighted average
        APL = p.w_R * r_norm + p.w_G * g_norm + p.w_B * b_norm
        
        return APL
    
    def get_brightness_factor(self, L_set: float) -> float:
        """
        Get normalized brightness factor.
        
        Θ(L_set) = (L_set / L_max)^α
        """
        p = self.params
        return (L_set / p.L_max) ** 1.0
    
    def get_power(self, APL: float, L_set: float, f_refresh: float,
                  touch_active: bool = False) -> dict:
        """
        Calculate total OLED display power.
        
        P_disp = P_base + k_drv*f + β*Θ(L)*APL + P_touch
        
        Args:
            APL: Average Pixel Level (0-1)
            L_set: Set brightness (nits)
            f_refresh: Refresh rate (Hz)
            touch_active: Whether touch is being scanned
            
        Returns:
            Dictionary with power breakdown
        """
        p = self.params
        
        # Component powers
        P_static = p.P_base
        P_driver = p.k_drv * f_refresh
        
        # Emissive power
        Theta = self.get_brightness_factor(L_set)
        P_emissive = p.beta_panel * Theta * APL * L_set
        
        # Touch
        P_touch = p.P_touch if touch_active else 0
        
        P_total = P_static + P_driver + P_emissive + P_touch
        
        return {
            'P_total': P_total,
            'P_static': P_static,
            'P_driver': P_driver,
            'P_emissive': P_emissive,
            'P_touch': P_touch,
            'APL': APL,
            'brightness': L_set,
            'refresh_rate': f_refresh
        }
    
    def compare_themes(self, L_set: float, f_refresh: float = 60.0) -> dict:
        """
        Compare power consumption between light and dark themes.
        
        Returns:
            Dictionary with comparison results
        """
        # Light theme (mostly white background)
        APL_light = self.calculate_APL(245, 245, 245)
        P_light = self.get_power(APL_light, L_set, f_refresh)
        
        # Dark theme (mostly black background)
        APL_dark = self.calculate_APL(30, 30, 30)
        P_dark = self.get_power(APL_dark, L_set, f_refresh)
        
        savings = (P_light['P_total'] - P_dark['P_total']) / P_light['P_total'] * 100
        
        return {
            'light_theme': P_light,
            'dark_theme': P_dark,
            'power_savings_pct': savings,
            'absolute_savings_mW': P_light['P_total'] - P_dark['P_total']
        }


# =============================================================================
# SoC Processor Subsystem Model
# =============================================================================

@dataclass
class ParamsSoC:
    """Parameters for SoC power model."""
    
    # DVFS parameters
    V_th: float = 0.35            # Threshold voltage (V)
    V_nominal: float = 0.9        # Nominal voltage (V)
    f_max: float = 3.0e9          # Maximum frequency (Hz)
    
    # Dynamic power
    C_eff: float = 10e-9          # Effective capacitance (F)
    alpha_act: float = 0.3        # Activity factor
    
    # Leakage parameters (BSIM4 model)
    I_ref: float = 0.020          # Reference leakage current (A) at T_ref
    T_ref: float = 25.0           # Reference temperature (°C)
    lambda_DIBL: float = 0.08     # DIBL coefficient
    zeta: float = 0.08            # Temperature sensitivity
    
    # Thermal parameters
    C_th: float = 5.0             # Thermal capacitance (J/K)
    R_th: float = 8.0             # Thermal resistance (K/W)
    
    # Physical constants
    k_B: float = 8.617e-5         # Boltzmann constant (eV/K)


class ModelSoC:
    """
    SoC Power Model with Electro-Thermal Coupling.
    
    Dynamic power: P_dyn ∝ C*V²*f ≈ κ*f³
    Leakage power: I_leak = I_ref * (T/T_ref)² * exp(λV + ζ(T-T_ref))
    
    Thermal coupling:
    C_th * dT/dt = P_total - (T - T_amb) / R_th
    """
    
    def __init__(self, params: Optional[ParamsSoC] = None):
        self.params = params if params else ParamsSoC()
        self.T_chip = 25.0  # Current chip temperature
        
    def reset(self, T_init: float = 25.0):
        """Reset chip temperature."""
        self.T_chip = T_init
        
    def get_voltage_for_frequency(self, f: float) -> float:
        """
        Get required voltage for target frequency (DVFS relationship).
        
        f ∝ (V - V_th)^γ / V
        
        Simplified: V ≈ V_th + k * f^0.5
        """
        p = self.params
        
        # Normalized frequency
        f_norm = f / p.f_max
        
        # Voltage scaling (simplified cubic relationship inverse)
        V = p.V_th + (p.V_nominal - p.V_th) * np.sqrt(f_norm)
        
        return V
    
    def get_dynamic_power(self, f: float) -> float:
        """
        Calculate dynamic switching power.
        
        P_dyn = α * C_eff * V² * f
        """
        p = self.params
        
        V = self.get_voltage_for_frequency(f)
        P_dyn = p.alpha_act * p.C_eff * (V ** 2) * f
        
        return P_dyn
    
    def get_leakage_current(self, V: float, T: float) -> float:
        """
        Calculate temperature and voltage dependent leakage current.
        
        I_leak = I_ref * (T/T_ref)² * exp(λ*V + ζ*(T-T_ref) / (n*kT/q))
        """
        p = self.params
        
        T_kelvin = T + 273.15
        T_ref_kelvin = p.T_ref + 273.15
        
        # Temperature ratio squared (thermal velocity effect)
        temp_factor = (T_kelvin / T_ref_kelvin) ** 2
        
        # Exponential factors
        exp_factor = np.exp(p.lambda_DIBL * V + p.zeta * (T - p.T_ref))
        
        I_leak = p.I_ref * temp_factor * exp_factor
        
        return I_leak
    
    def step(self, f: float, T_amb: float, dt: float = 1.0) -> dict:
        """
        Update SoC state and calculate power with thermal coupling.
        
        Uses Newton-Raphson iteration to solve coupled equations.
        """
        p = self.params
        
        V = self.get_voltage_for_frequency(f)
        P_dyn = self.get_dynamic_power(f)
        
        # Iterative solution for thermal coupling
        for _ in range(5):  # Newton-Raphson iterations
            I_leak = self.get_leakage_current(V, self.T_chip)
            P_leak = V * I_leak
            P_total = P_dyn + P_leak
            
            # Thermal update
            dT_dt = (P_total - (self.T_chip - T_amb) / p.R_th) / p.C_th
            self.T_chip = self.T_chip + dT_dt * dt
            self.T_chip = np.clip(self.T_chip, T_amb, 105)  # Max junction temp
        
        return {
            'P_total': P_total * 1000,  # mW
            'P_dynamic': P_dyn * 1000,
            'P_leakage': P_leak * 1000,
            'I_leak': I_leak * 1000,  # mA
            'V_dd': V,
            'f': f / 1e9,  # GHz
            'T_chip': self.T_chip
        }
    
    def simulate_workload(self, f_profile: np.ndarray, T_amb: float,
                          dt: float = 1.0, T_init: float = 25.0) -> List[dict]:
        """
        Simulate SoC over time with varying workload.
        
        Returns:
            List of state dictionaries
        """
        self.reset(T_init)
        
        results = []
        for f in f_profile:
            result = self.step(f, T_amb, dt)
            results.append(result)
            
        return results


# =============================================================================
# Integrated System Power Model
# =============================================================================

class IntegratedPowerModel:
    """
    Integrated power model combining all subsystems.
    
    Total battery current:
    I_batt = P_total / (η_conv * V_batt)
    
    Where P_total = P_SoC + P_disp + P_5G + P_BT + P_GNSS + P_bg
    """
    
    def __init__(self):
        self.model_5g = Model5G()
        self.model_bt = ModelBluetooth()
        self.model_bg = ModelBackground()
        self.model_gnss = ModelGNSS()
        self.model_oled = ModelOLED()
        self.model_soc = ModelSoC()
        
        # PMIC efficiency
        self.eta_conv_base = 0.92
        
    def get_pmic_efficiency(self, I_load: float, V_batt: float) -> float:
        """
        Get DC-DC converter efficiency based on load.
        
        Efficiency drops at very light and very heavy loads.
        """
        # Optimal load around 1A
        I_optimal = 1.0
        eta = self.eta_conv_base - 0.05 * ((I_load - I_optimal) / I_optimal) ** 2
        return np.clip(eta, 0.80, 0.95)
    
    def calculate_total_power(self, scenario: dict) -> dict:
        """
        Calculate total system power for given usage scenario.
        
        Args:
            scenario: Dictionary with subsystem usage parameters
            
        Returns:
            Comprehensive power breakdown
        """
        results = {}
        
        # 5G
        if scenario.get('5g_active', False):
            r = self.model_5g.get_power(
                scenario.get('data_rate', 50e6),
                scenario.get('distance', 500)
            )
            results['5G'] = r['P_total'] * 1000  # mW
        else:
            results['5G'] = 0
            
        # Bluetooth
        if scenario.get('bt_mode') == 'audio':
            r = self.model_bt.get_power('audio')
            results['Bluetooth'] = r['P_total']
        elif scenario.get('bt_mode') == 'ble':
            r = self.model_bt.get_power('ble', 
                interval_ms=scenario.get('bt_interval', 100))
            results['Bluetooth'] = r['P_total']
        else:
            results['Bluetooth'] = 0
            
        # Background
        r = self.model_bg.get_average_power(
            scenario.get('wakeup_rate', 2.0)
        )
        results['Background'] = r['P_total']
        
        # GNSS
        if scenario.get('gnss_active', False):
            r = self.model_gnss.get_power(
                scenario.get('signal_quality', 35)
            )
            results['GNSS'] = r['P_total']
        else:
            results['GNSS'] = 0
            
        # Display
        if scenario.get('display_on', True):
            APL = self.model_oled.calculate_APL(
                *scenario.get('rgb_mean', (128, 128, 128))
            )
            r = self.model_oled.get_power(
                APL,
                scenario.get('brightness', 300),
                scenario.get('refresh_rate', 60)
            )
            results['Display'] = r['P_total']
        else:
            results['Display'] = 0
            
        # SoC
        f = scenario.get('cpu_freq', 1.5e9)
        r = self.model_soc.step(f, scenario.get('T_amb', 25))
        results['SoC'] = r['P_total']
        
        # Total
        results['Total'] = sum(results.values())
        
        return results


if __name__ == "__main__":
    # Test each model
    print("=" * 60)
    print("Load Subsystem Models Test")
    print("=" * 60)
    
    # 5G test
    model_5g = Model5G()
    r = model_5g.get_power(100e6, 500)
    print(f"\n5G (100 Mbps, 500m): {r['P_total']*1000:.1f} mW")
    
    # Bluetooth test
    model_bt = ModelBluetooth()
    r = model_bt.get_power('ble', interval_ms=100)
    print(f"BLE (100ms interval): {r['P_total']:.2f} mW")
    
    # GNSS test
    model_gnss = ModelGNSS()
    r = model_gnss.get_power(35)
    print(f"GNSS (35 dB-Hz): {r['P_total']:.1f} mW")
    
    # OLED test
    model_oled = ModelOLED()
    comparison = model_oled.compare_themes(500)
    print(f"OLED Dark Mode Savings: {comparison['power_savings_pct']:.1f}%")
    
    # SoC test
    model_soc = ModelSoC()
    r = model_soc.step(2e9, 25)
    print(f"SoC (2 GHz, 25°C): {r['P_total']:.1f} mW")
