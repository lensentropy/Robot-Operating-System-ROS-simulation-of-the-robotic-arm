"""
Smartphone Load Subsystem Models
=================================
Multi-Physics Load Models for Smartphone Components

This module implements continuous-time power consumption models for:
1. 5G Communication Module - Link budget & PA efficiency dynamics
2. Bluetooth/BLE - Discrete event-driven integral model
3. Background Tasks - Poisson process & tail energy dynamics
4. GNSS Navigation - Environment-aware dual-mode state machine
5. OLED Display - Content-aware & LTPO variable refresh
6. SoC/CPU - DVFS & thermal-coupled leakage dynamics

Mathematical Foundation:
------------------------
Total load current is the superposition of all subsystem currents:
    I_load(t) = Σ P_i(t) / V_bat(t)

Each subsystem follows physics-based continuous-time differential equations
or integral formulations that capture the true dynamic behavior.
"""

import numpy as np
from scipy.integrate import quad
from battery_params import (
    LOAD_5G_PARAMS, LOAD_BLE_PARAMS, LOAD_BACKGROUND_PARAMS,
    LOAD_GNSS_PARAMS, LOAD_DISPLAY_PARAMS, LOAD_SOC_PARAMS,
    PHYSICAL_CONSTANTS
)


# =============================================================================
# 5G Communication Module Model
# =============================================================================
class Load5GModel:
    """
    5G Communication subsystem power model.
    
    Based on Shannon-Hartley theorem and Friis transmission equation:
    
    P_5G(t) = P_static + α_bb * R(t) + P_tx(t) / η_PA
    
    where:
        P_tx(t) = Λ_env * d(t)^n * (2^(R(t)/B) - 1)
    
    This captures:
    - Exponential growth of TX power with data rate
    - Power-law scaling with distance
    - PA efficiency nonlinearity
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_5G_PARAMS
    
    def required_tx_power(self, data_rate, distance):
        """
        Calculate required transmit power from link budget.
        
        Parameters:
            data_rate: Instantaneous data rate [bit/s]
            distance: Distance to base station [m]
        
        Returns:
            P_tx: Required transmit power [W]
        """
        p = self.params
        
        # Shannon capacity constraint (with overflow protection)
        rate_ratio = np.clip(data_rate / p['B'], 0, 10)  # Limit exponent
        snr_required = 2**rate_ratio - 1
        
        # Link budget: P_tx required to maintain SNR
        P_tx = p['Lambda_env'] * (distance ** p['n']) * snr_required
        
        return np.clip(P_tx, 0, 2.0)  # Max 2W transmit power
    
    def power_consumption(self, t, data_rate, distance, is_active=True):
        """
        Calculate total 5G power consumption.
        
        Parameters:
            t: Time [s]
            data_rate: Data rate [bit/s]
            distance: Distance to base station [m]
            is_active: Whether 5G radio is active
        
        Returns:
            P_5G: Power consumption [W]
        """
        if not is_active:
            return 0.001  # Ultra-low standby power
        
        p = self.params
        
        # Static power
        P_static = p['P_static']
        
        # Baseband processing power
        P_baseband = p['alpha_bb'] * data_rate
        
        # RF transmit power (accounting for PA efficiency)
        P_tx = self.required_tx_power(data_rate, distance)
        P_rf = P_tx / p['eta_PA']
        
        return P_static + P_baseband + P_rf


# =============================================================================
# Bluetooth/BLE Model
# =============================================================================
class LoadBLEModel:
    """
    Bluetooth Low Energy power model.
    
    Discrete event-driven with hyperbolic average current law:
    
    I_BLE(τ) ≈ I_sleep + Q_event(L) / τ
    
    For audio streaming (A2DP), continuous model:
    
    P_audio(t) = P_RF_base + κ_codec * F_s(t) * D_depth
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_BLE_PARAMS
    
    def event_charge(self, payload_size=20):
        """
        Calculate charge consumed per connection event.
        
        Parameters:
            payload_size: Data payload size [bytes]
        
        Returns:
            Q_event: Charge per event [As]
        """
        p = self.params
        
        # Base event overhead
        Q_base = p['Q_event_base']
        
        # Additional charge for payload
        t_tx = payload_size * 8 / 1e6  # Time to transmit at 1 Mbps
        Q_tx = p['I_tx'] * t_tx
        Q_rx = p['I_rx'] * t_tx * 0.5  # RX for acknowledgment
        
        return Q_base + Q_tx + Q_rx
    
    def average_current_ble(self, connection_interval, payload_size=20):
        """
        Calculate average BLE current using hyperbolic model.
        
        Parameters:
            connection_interval: Connection interval τ [s]
            payload_size: Data payload [bytes]
        
        Returns:
            I_avg: Average current [A]
        """
        p = self.params
        
        Q_event = self.event_charge(payload_size)
        I_avg = p['I_sleep'] + Q_event / connection_interval
        
        return I_avg
    
    def power_consumption(self, t, V_bat, mode='ble', 
                          connection_interval=None, 
                          is_streaming=False):
        """
        Calculate Bluetooth power consumption.
        
        Parameters:
            t: Time [s]
            V_bat: Battery voltage [V]
            mode: 'ble' or 'classic'
            connection_interval: BLE connection interval [s]
            is_streaming: Whether audio streaming is active
        
        Returns:
            P_BT: Power consumption [W]
        """
        p = self.params
        connection_interval = connection_interval or p['tau_default']
        
        if is_streaming:
            # Audio streaming mode (A2DP)
            return p['P_audio_base']
        else:
            # BLE connection mode
            I_avg = self.average_current_ble(connection_interval)
            return V_bat * I_avg


# =============================================================================
# Background Tasks Model
# =============================================================================
class LoadBackgroundModel:
    """
    Background tasks power model with Poisson arrivals.
    
    Wake-up events follow non-homogeneous Poisson process:
        λ(t) = λ_base * activity_factor(t)
    
    Tail energy mechanism captured by O-U process:
        dI_bg(t) = θ * (μ_bg - I_bg(t)) * dt + σ_bg * dW(t)
    
    Average power:
        P_bg(t) = V_bat * (I_leak + I_active * duty_cycle(t))
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_BACKGROUND_PARAMS
        self._ou_state = 0.0  # O-U process state
    
    def duty_cycle(self, wake_rate):
        """
        Calculate duty cycle from wake-up rate.
        
        Parameters:
            wake_rate: Wake-ups per minute [1/min]
        
        Returns:
            dc: Duty cycle [-]
        """
        p = self.params
        
        # Average active time per minute
        t_active_per_wake = 0.5  # Average processing time [s]
        t_active_total = wake_rate * (t_active_per_wake + p['tau_tail'])
        
        dc = t_active_total / 60.0  # Per minute
        return np.clip(dc, 0, 1)
    
    def ou_process_step(self, dt, activity_level=1.0):
        """
        Update Ornstein-Uhlenbeck process state.
        
        Parameters:
            dt: Time step [s]
            activity_level: Activity multiplier [-]
        
        Returns:
            I_bg_noise: Background current fluctuation [A]
        """
        p = self.params
        
        mu_bg = activity_level * p['P_idle'] / 3.7  # Target current
        
        # O-U process update
        dW = np.random.normal(0, np.sqrt(dt))
        self._ou_state += p['theta_ou'] * (mu_bg - self._ou_state) * dt
        self._ou_state += p['sigma_bg'] * dW
        
        return np.maximum(self._ou_state, 0)
    
    def power_consumption(self, t, V_bat, wake_rate_per_min=None, 
                          activity_level=1.0, use_stochastic=True):
        """
        Calculate background tasks power consumption.
        
        Parameters:
            t: Time [s]
            V_bat: Battery voltage [V]
            wake_rate_per_min: Wake-up rate [1/min]
            activity_level: App activity level multiplier [-]
            use_stochastic: Whether to use O-U noise
        
        Returns:
            P_bg: Power consumption [W]
        """
        p = self.params
        wake_rate = wake_rate_per_min or (p['lambda_base'] * activity_level)
        
        # Base leakage power
        P_leak = p['P_leak']
        
        # Duty cycle based power
        dc = self.duty_cycle(wake_rate)
        P_duty = dc * p['P_proc'] + (1 - dc) * p['P_idle']
        
        # Add stochastic fluctuation
        if use_stochastic:
            I_noise = self.ou_process_step(1.0, activity_level)
            P_noise = V_bat * I_noise
        else:
            P_noise = 0
        
        return P_leak + P_duty + P_noise


# =============================================================================
# GNSS Navigation Model
# =============================================================================
class LoadGNSSModel:
    """
    GNSS navigation power model with environment-aware mode switching.
    
    Lock probability (Sigmoid):
        Ψ(S_env) = 1 / (1 + exp(-α * (S_env - S_th)))
    
    State dynamics (first-order relaxation):
        dx_lock/dt = (Ψ(S_env) - x_lock) / τ_react
    
    Power output:
        P_GNSS(t) = x_lock * P_track + (1 - x_lock) * P_acq + P_LNA
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_GNSS_PARAMS
        self.x_lock = 0.0  # Lock state variable
    
    def signal_quality(self, environment='urban', satellite_count=8):
        """
        Calculate signal quality factor.
        
        Parameters:
            environment: 'open', 'urban', 'canyon', 'indoor'
            satellite_count: Number of visible satellites
        
        Returns:
            S_env: Signal quality factor [0, 1]
        """
        # Environment attenuation factors
        env_factors = {
            'open': 1.0,
            'urban': 0.7,
            'canyon': 0.3,
            'indoor': 0.1,
            'tunnel': 0.0
        }
        
        env_factor = env_factors.get(environment, 0.5)
        sat_factor = np.clip(satellite_count / 12, 0, 1)
        
        S_env = env_factor * sat_factor
        return S_env
    
    def lock_probability(self, S_env):
        """
        Calculate lock probability using Sigmoid function.
        
        Parameters:
            S_env: Signal quality factor [-]
        
        Returns:
            Psi: Lock probability [-]
        """
        p = self.params
        Psi = 1 / (1 + np.exp(-p['alpha_gnss'] * (S_env - p['S_th'])))
        return Psi
    
    def update_lock_state(self, S_env, dt):
        """
        Update lock state using first-order dynamics.
        
        Parameters:
            S_env: Signal quality factor [-]
            dt: Time step [s]
        
        Returns:
            x_lock: Updated lock state [-]
        """
        p = self.params
        Psi = self.lock_probability(S_env)
        
        # First-order relaxation dynamics
        dx = (Psi - self.x_lock) / p['tau_react']
        self.x_lock += dx * dt
        self.x_lock = np.clip(self.x_lock, 0, 1)
        
        return self.x_lock
    
    def power_consumption(self, t, is_active=True, environment='urban', 
                          satellite_count=8, dt=1.0):
        """
        Calculate GNSS power consumption.
        
        Parameters:
            t: Time [s]
            is_active: Whether GNSS is enabled
            environment: Environment type
            satellite_count: Visible satellites
            dt: Time step for state update [s]
        
        Returns:
            P_GNSS: Power consumption [W]
        """
        if not is_active:
            return 0.0
        
        p = self.params
        
        # Get signal quality
        S_env = self.signal_quality(environment, satellite_count)
        
        # Update lock state
        x_lock = self.update_lock_state(S_env, dt)
        
        # Calculate power
        P_GNSS = (x_lock * p['P_track'] + 
                  (1 - x_lock) * p['P_acq'] + 
                  p['P_LNA'])
        
        return P_GNSS


# =============================================================================
# OLED Display Model
# =============================================================================
class LoadDisplayModel:
    """
    OLED display power model with content-aware emissive power.
    
    Average Pixel Level (APL):
        A(t) = (1/WH) * Σ [w_R*(R/255)^γ + w_G*(G/255)^γ + w_B*(B/255)^γ]
    
    Emissive power:
        P_emit(t) = β_panel * (L_set/L_max)^α * A(t)
    
    Driver power (LTPO):
        P_driver(t) = C_eff * V_dd² * f_refresh(t)
    
    Total:
        P_disp(t) = P_base + k_drv * f_refresh(t) + β_panel * Θ(L_set) * A(t)
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_DISPLAY_PARAMS
    
    def calculate_apl(self, content_type='mixed'):
        """
        Calculate Average Pixel Level for content type.
        
        Parameters:
            content_type: 'dark', 'mixed', 'bright', 'video', 'text'
        
        Returns:
            APL: Average pixel level [0, 1]
        """
        # Typical APL values for different content
        apl_values = {
            'dark': 0.1,
            'text': 0.25,
            'mixed': 0.4,
            'video': 0.5,
            'bright': 0.8,
            'white': 1.0
        }
        return apl_values.get(content_type, 0.4)
    
    def brightness_function(self, brightness_level):
        """
        Calculate normalized brightness function.
        
        Parameters:
            brightness_level: User brightness setting [0, 1]
        
        Returns:
            Theta: Normalized brightness factor [-]
        """
        p = self.params
        # Nonlinear brightness response
        Theta = brightness_level ** p['alpha_bright']
        return Theta
    
    def power_consumption(self, t, is_on=True, brightness=0.5, 
                          refresh_rate=60, content_type='mixed'):
        """
        Calculate display power consumption.
        
        Parameters:
            t: Time [s]
            is_on: Whether screen is on
            brightness: Brightness level [0, 1]
            refresh_rate: Screen refresh rate [Hz]
            content_type: Type of displayed content
        
        Returns:
            P_disp: Power consumption [W]
        """
        if not is_on:
            return 0.01  # Minimal standby power
        
        p = self.params
        
        # Base static power
        P_base = p['P_base']
        
        # Driver dynamic power (LTPO effect)
        P_driver = p['k_drv'] * refresh_rate
        
        # Emissive power
        APL = self.calculate_apl(content_type)
        Theta = self.brightness_function(brightness)
        P_emit = p['beta_panel'] * Theta * APL
        
        return P_base + P_driver + P_emit


# =============================================================================
# SoC/CPU Model
# =============================================================================
class LoadSoCModel:
    """
    SoC/CPU power model with DVFS and thermal coupling.
    
    Dynamic power (Alpha-Power Law):
        P_dyn(t) = κ_dvfs * f(t)³
    
    Static leakage (BSIM4-based):
        I_leak(t) = I_ref * (T/T_ref)² * exp(λ_DIBL*V_dd + ζ*(T-T_ref)/(n*k_B*T/q))
    
    Electro-thermal coupling:
        C_th * dT/dt = P_SoC(t) - (T - T_amb) / R_th
    """
    
    def __init__(self, params=None):
        self.params = params or LOAD_SOC_PARAMS
        self.T_soc = 40.0  # SoC temperature [°C]
    
    def frequency_from_load(self, cpu_load):
        """
        Calculate CPU frequency from load (DVFS policy).
        
        Parameters:
            cpu_load: CPU utilization [0, 1]
        
        Returns:
            f: CPU frequency [Hz]
        """
        p = self.params
        
        # Simplified DVFS: linear mapping with minimum frequency
        f = p['f_min'] + (p['f_max'] - p['f_min']) * cpu_load
        
        return f
    
    def dynamic_power(self, frequency):
        """
        Calculate dynamic switching power.
        
        Parameters:
            frequency: CPU frequency [Hz]
        
        Returns:
            P_dyn: Dynamic power [W]
        """
        p = self.params
        
        # Cubic relationship with frequency (Alpha-Power Law)
        P_dyn = p['kappa_dvfs'] * (frequency ** 3)
        
        return P_dyn
    
    def leakage_current(self, T_soc, V_dd=None):
        """
        Calculate temperature-dependent leakage current.
        
        Parameters:
            T_soc: SoC temperature [°C]
            V_dd: Supply voltage [V]
        
        Returns:
            I_leak: Leakage current [A]
        """
        p = self.params
        c = PHYSICAL_CONSTANTS
        
        V_dd = V_dd or p['V_dd_nom']
        T_kelvin = T_soc + c['T_kelvin_offset']
        T_ref_kelvin = p['T_leak_ref'] + c['T_kelvin_offset']
        
        # BSIM4-style leakage model
        temp_factor = (T_kelvin / T_ref_kelvin) ** 2
        dibl_factor = np.exp(p['lambda_DIBL'] * V_dd)
        thermal_factor = np.exp(p['zeta_temp'] * (T_soc - p['T_leak_ref']) / 
                                (p['n_ideal'] * c['k_B'] * T_kelvin / c['q']))
        
        I_leak = p['I_leak_ref'] * temp_factor * dibl_factor * thermal_factor
        
        return np.clip(I_leak, 0, 0.5)  # Max 500mA leakage
    
    def update_soc_temperature(self, P_soc, T_amb, dt):
        """
        Update SoC temperature using thermal dynamics.
        
        Parameters:
            P_soc: SoC power consumption [W]
            T_amb: Ambient temperature [°C]
            dt: Time step [s]
        
        Returns:
            T_soc: Updated SoC temperature [°C]
        """
        p = self.params
        
        # Thermal dynamics
        dT = (P_soc - (self.T_soc - T_amb) / p['R_th_soc']) / p['C_th_soc']
        self.T_soc += dT * dt
        
        return self.T_soc
    
    def power_consumption(self, t, cpu_load, T_amb=25.0, dt=1.0):
        """
        Calculate total SoC power consumption.
        
        Parameters:
            t: Time [s]
            cpu_load: CPU utilization [0, 1]
            T_amb: Ambient temperature [°C]
            dt: Time step [s]
        
        Returns:
            P_SoC: Total SoC power [W]
        """
        p = self.params
        
        # Get frequency from load
        f = self.frequency_from_load(cpu_load)
        
        # Dynamic power
        P_dyn = self.dynamic_power(f)
        
        # Static leakage power
        I_leak = self.leakage_current(self.T_soc)
        P_leak = p['V_dd_nom'] * I_leak
        
        # Total power
        P_soc = P_dyn + P_leak
        
        # Update SoC temperature (thermal feedback)
        self.update_soc_temperature(P_soc, T_amb, dt)
        
        return P_soc


# =============================================================================
# Unified Load Manager
# =============================================================================
class SmartphoneLoadManager:
    """
    Unified manager for all smartphone load subsystems.
    
    Aggregates power consumption from all components:
        P_total(t) = P_5G + P_BT + P_bg + P_GNSS + P_disp + P_SoC
    
    Provides continuous-time load current function for battery model.
    """
    
    def __init__(self):
        # Initialize all subsystem models
        self.load_5g = Load5GModel()
        self.load_ble = LoadBLEModel()
        self.load_background = LoadBackgroundModel()
        self.load_gnss = LoadGNSSModel()
        self.load_display = LoadDisplayModel()
        self.load_soc = LoadSoCModel()
        
        # Default usage profile
        self.profile = self._default_profile()
    
    def _default_profile(self):
        """Get default usage profile."""
        return {
            # 5G settings
            '5g_active': True,
            '5g_data_rate': 10e6,  # 10 Mbps
            '5g_distance': 500,    # 500m from base station
            
            # Bluetooth settings
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 0.1,
            
            # Background settings
            'bg_wake_rate': 2.0,
            'bg_activity': 1.0,
            
            # GNSS settings
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            
            # Display settings
            'display_on': True,
            'display_brightness': 0.5,
            'display_refresh': 60,
            'display_content': 'mixed',
            
            # CPU settings
            'cpu_load': 0.3,
        }
    
    def set_profile(self, **kwargs):
        """Update usage profile."""
        self.profile.update(kwargs)
    
    def total_power(self, t, V_bat, T_amb=25.0, dt=1.0, profile=None):
        """
        Calculate total power consumption from all subsystems.
        
        Parameters:
            t: Time [s]
            V_bat: Battery voltage [V]
            T_amb: Ambient temperature [°C]
            dt: Time step [s]
            profile: Usage profile dict (optional, uses stored profile if None)
        
        Returns:
            P_total: Total power consumption [W]
            breakdown: Dict of individual subsystem powers
        """
        p = profile or self.profile
        
        # 5G power
        P_5g = self.load_5g.power_consumption(
            t, 
            p['5g_data_rate'], 
            p['5g_distance'],
            p['5g_active']
        )
        
        # Bluetooth power
        P_bt = self.load_ble.power_consumption(
            t, V_bat,
            connection_interval=p['bt_interval'],
            is_streaming=p['bt_streaming']
        ) if p['bt_active'] else 0.0
        
        # Background power
        P_bg = self.load_background.power_consumption(
            t, V_bat,
            wake_rate_per_min=p['bg_wake_rate'],
            activity_level=p['bg_activity'],
            use_stochastic=False
        )
        
        # GNSS power
        P_gnss = self.load_gnss.power_consumption(
            t, 
            p['gnss_active'],
            p['gnss_environment'],
            p['gnss_satellites'],
            dt
        )
        
        # Display power
        P_disp = self.load_display.power_consumption(
            t,
            p['display_on'],
            p['display_brightness'],
            p['display_refresh'],
            p['display_content']
        )
        
        # SoC power
        P_soc = self.load_soc.power_consumption(
            t,
            p['cpu_load'],
            T_amb,
            dt
        )
        
        P_total = P_5g + P_bt + P_bg + P_gnss + P_disp + P_soc
        
        breakdown = {
            '5G': P_5g,
            'Bluetooth': P_bt,
            'Background': P_bg,
            'GNSS': P_gnss,
            'Display': P_disp,
            'SoC': P_soc,
            'Total': P_total
        }
        
        return P_total, breakdown
    
    def get_load_current_function(self, T_amb=25.0, profile=None):
        """
        Create a continuous load current function for battery simulation.
        
        Parameters:
            T_amb: Ambient temperature [°C]
            profile: Usage profile
        
        Returns:
            I_load_func: Function I_load(t, V_bat) -> current [A]
        """
        def I_load_func(t, V_bat=3.7):
            P_total, _ = self.total_power(t, V_bat, T_amb, 1.0, profile)
            return P_total / V_bat
        
        return I_load_func


# =============================================================================
# Usage Scenario Profiles
# =============================================================================
class UsageScenarios:
    """
    Predefined usage scenario profiles for common smartphone activities.
    """
    
    @staticmethod
    def idle_screen_off():
        """Screen off, minimal background activity."""
        return {
            '5g_active': True,
            '5g_data_rate': 100e3,
            '5g_distance': 500,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 1.0,
            'bg_wake_rate': 0.5,
            'bg_activity': 0.3,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': False,
            'display_brightness': 0.0,
            'display_refresh': 60,
            'display_content': 'dark',
            'cpu_load': 0.05,
        }
    
    @staticmethod
    def idle_screen_on():
        """Screen on, reading/browsing light content."""
        return {
            '5g_active': True,
            '5g_data_rate': 1e6,
            '5g_distance': 500,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 0.5,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.5,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': True,
            'display_brightness': 0.4,
            'display_refresh': 60,
            'display_content': 'text',
            'cpu_load': 0.15,
        }
    
    @staticmethod
    def video_streaming():
        """Video streaming scenario."""
        return {
            '5g_active': True,
            '5g_data_rate': 20e6,
            '5g_distance': 500,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 0.5,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.5,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': True,
            'display_brightness': 0.7,
            'display_refresh': 60,
            'display_content': 'video',
            'cpu_load': 0.4,
        }
    
    @staticmethod
    def gaming():
        """Intensive gaming scenario."""
        return {
            '5g_active': True,
            '5g_data_rate': 5e6,
            '5g_distance': 500,
            'bt_active': True,
            'bt_streaming': False,
            'bt_interval': 0.1,
            'bg_wake_rate': 0.5,
            'bg_activity': 0.3,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': True,
            'display_brightness': 0.9,
            'display_refresh': 120,
            'display_content': 'bright',
            'cpu_load': 0.9,
        }
    
    @staticmethod
    def navigation():
        """GPS navigation with audio."""
        return {
            '5g_active': True,
            '5g_data_rate': 5e6,
            '5g_distance': 700,
            'bt_active': True,
            'bt_streaming': True,
            'bt_interval': 0.1,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.6,
            'gnss_active': True,
            'gnss_environment': 'urban',
            'gnss_satellites': 6,
            'display_on': True,
            'display_brightness': 0.8,
            'display_refresh': 60,
            'display_content': 'mixed',
            'cpu_load': 0.5,
        }
    
    @staticmethod
    def voice_call():
        """Voice call scenario."""
        return {
            '5g_active': True,
            '5g_data_rate': 1e6,
            '5g_distance': 500,
            'bt_active': True,
            'bt_streaming': True,
            'bt_interval': 0.02,
            'bg_wake_rate': 0.5,
            'bg_activity': 0.3,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': False,
            'display_brightness': 0.0,
            'display_refresh': 60,
            'display_content': 'dark',
            'cpu_load': 0.2,
        }
    
    @staticmethod
    def weak_signal():
        """Weak cellular signal scenario (increased power)."""
        return {
            '5g_active': True,
            '5g_data_rate': 5e6,
            '5g_distance': 2000,  # Far from base station
            'bt_active': False,
            'bt_streaming': False,
            'bt_interval': 0.5,
            'bg_wake_rate': 1.0,
            'bg_activity': 0.5,
            'gnss_active': False,
            'gnss_environment': 'urban',
            'gnss_satellites': 8,
            'display_on': True,
            'display_brightness': 0.5,
            'display_refresh': 60,
            'display_content': 'mixed',
            'cpu_load': 0.3,
        }


if __name__ == "__main__":
    # Test load manager
    manager = SmartphoneLoadManager()
    
    # Test different scenarios
    scenarios = {
        'Idle (screen off)': UsageScenarios.idle_screen_off(),
        'Idle (screen on)': UsageScenarios.idle_screen_on(),
        'Video streaming': UsageScenarios.video_streaming(),
        'Gaming': UsageScenarios.gaming(),
        'Navigation': UsageScenarios.navigation(),
        'Voice call': UsageScenarios.voice_call(),
        'Weak signal': UsageScenarios.weak_signal(),
    }
    
    print("=" * 70)
    print("Smartphone Load Power Consumption Analysis")
    print("=" * 70)
    
    for name, profile in scenarios.items():
        P_total, breakdown = manager.total_power(0, 3.7, 25.0, 1.0, profile)
        
        print(f"\n{name}:")
        print("-" * 40)
        for component, power in breakdown.items():
            if component != 'Total':
                print(f"  {component:12s}: {power*1000:7.1f} mW ({power/P_total*100:5.1f}%)")
        print(f"  {'Total':12s}: {P_total*1000:7.1f} mW")
        print(f"  Estimated current @ 3.7V: {P_total/3.7*1000:.1f} mA")
