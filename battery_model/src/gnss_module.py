"""
GNSS (Global Navigation Satellite System) Module Power Consumption Model
========================================================================

This module implements a continuous-time state machine model for GPS/GNSS
receiver power consumption. The key insight is that GPS receivers have
distinct operational modes with dramatically different power profiles:

1. Cold Start Acquisition: High power, searching for satellites
2. Hot Start Acquisition: Medium power, reacquiring recent satellites
3. Tracking: Low power, maintaining lock on satellites
4. Sleep/Standby: Minimal power

The transitions between states are modeled using smooth sigmoid functions
to maintain differentiability for the continuous-time ODE system.

References:
- GPS receiver architecture (Kaplan & Hegarty, Understanding GPS)
- u-blox power consumption specifications
- Qualcomm GPS module datasheets
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, List
from enum import IntEnum


class GNSSState(IntEnum):
    """GNSS receiver operational states."""
    SLEEP = 0
    ACQUISITION_COLD = 1
    ACQUISITION_HOT = 2
    TRACKING = 3


@dataclass
class GNSSParameters:
    """
    GNSS module parameters based on typical smartphone implementations.
    
    Reference: Qualcomm Snapdragon GNSS, Broadcom BCM4775X
    """
    # Power consumption by state (W)
    P_sleep: float = 0.001  # Sleep mode: ~1 mW
    P_acq_cold: float = 0.180  # Cold acquisition: ~180 mW
    P_acq_hot: float = 0.120  # Hot acquisition: ~120 mW
    P_tracking: float = 0.030  # Tracking: ~30 mW
    
    # Acquisition times (s)
    t_cold_start: float = 35.0  # Cold start TTFF
    t_hot_start: float = 3.0   # Hot start TTFF
    
    # SNR thresholds (dB-Hz)
    SNR_acq_threshold: float = 30.0  # Minimum for acquisition
    SNR_track_threshold: float = 25.0  # Minimum for tracking
    
    # State transition sigmoid steepness
    k_transition: float = 2.0
    
    # Update rate (Hz)
    update_rate: float = 1.0
    
    # Multi-constellation (GPS+GLONASS+Galileo+BeiDou)
    n_constellations: int = 4
    
    # AGPS (Assisted GPS) available
    agps_enabled: bool = True
    
    # Signal attenuation factors (dB)
    atten_indoor: float = 15.0
    atten_urban_canyon: float = 10.0
    atten_open_sky: float = 0.0


class GNSSModule:
    """
    Continuous-time GNSS power consumption model with smooth state transitions.
    
    The power consumption is modeled as a weighted sum of state powers,
    where weights are determined by smooth (sigmoid) transition functions
    based on environmental SNR.
    
    P_GNSS(t) = Σ w_i(SNR(t)) * P_i
    
    Where w_i are the state occupation probabilities.
    """
    
    def __init__(self, params: Optional[GNSSParameters] = None):
        self.params = params or GNSSParameters()
        self._last_state = GNSSState.SLEEP
        self._lock_time = 0.0
        self._satellites_tracked = 0
        
    def sigmoid(self, x: float, k: float = 1.0, x0: float = 0.0) -> float:
        """
        Smooth sigmoid transition function.
        
        σ(x) = 1 / (1 + exp(-k*(x - x0)))
        """
        return 1.0 / (1.0 + np.exp(-k * (x - x0)))
    
    def nominal_snr(self, elevation: float = 45.0) -> float:
        """
        Calculate nominal satellite signal SNR based on elevation angle.
        
        Higher elevation = stronger signal (less atmospheric path).
        
        Args:
            elevation: Satellite elevation angle (degrees)
            
        Returns:
            Nominal SNR (dB-Hz)
        """
        # Typical GPS L1 C/A signal strength
        # Peak ~45 dB-Hz at high elevation, drops at low elevation
        base_snr = 45.0
        elevation_factor = np.sin(np.radians(elevation))
        
        return base_snr * elevation_factor + 10
    
    def effective_snr(self, snr_nominal: float, 
                      environment: str = 'outdoor',
                      attenuation_extra: float = 0.0) -> float:
        """
        Calculate effective SNR considering environmental attenuation.
        
        Args:
            snr_nominal: Nominal SNR (dB-Hz)
            environment: 'outdoor', 'urban', 'indoor', 'deep_indoor'
            attenuation_extra: Additional attenuation (dB)
        """
        p = self.params
        
        env_atten = {
            'outdoor': p.atten_open_sky,
            'urban': p.atten_urban_canyon,
            'indoor': p.atten_indoor,
            'deep_indoor': 25.0,
            'tunnel': 50.0  # Essentially blocked
        }
        
        atten = env_atten.get(environment, 0.0) + attenuation_extra
        
        return snr_nominal - atten
    
    def state_weights(self, snr: float, time_since_lock: float = 0.0,
                      requested: bool = True) -> np.ndarray:
        """
        Calculate continuous state occupation weights based on SNR.
        
        This is the key innovation: mapping discrete states to continuous
        weights using sigmoid functions, enabling smooth differentiation.
        
        Args:
            snr: Current effective SNR (dB-Hz)
            time_since_lock: Time since last satellite lock (hours)
            requested: Whether GPS is actively requested by app
            
        Returns:
            Array of weights [w_sleep, w_cold, w_hot, w_track]
        """
        p = self.params
        k = p.k_transition
        
        weights = np.zeros(4)
        
        if not requested:
            weights[GNSSState.SLEEP] = 1.0
            return weights
        
        # Probability of successful tracking based on SNR (higher k for sharper transition)
        # Track threshold at 25 dB-Hz
        p_track = self.sigmoid(snr, k * 1.5, p.SNR_track_threshold)
        
        # Probability of successful acquisition (threshold at 30 dB-Hz)
        # Need slightly lower SNR for acquisition than tracking (20 dB-Hz)
        p_acq = self.sigmoid(snr, k * 1.2, 20.0)
        
        # Hot start available if recent lock (within 2 hours)
        p_hot = self.sigmoid(2.0 - time_since_lock, k * 2, 0.5)
        
        # State weights with clearer separation
        # Tracking: strong signal
        weights[GNSSState.TRACKING] = p_track
        
        # Acquisition states: weak signal but still detectable
        acq_region = p_acq * (1 - p_track)
        weights[GNSSState.ACQUISITION_HOT] = acq_region * p_hot
        weights[GNSSState.ACQUISITION_COLD] = acq_region * (1 - p_hot)
        
        # Sleep when signal too weak (below 20 dB-Hz)
        weights[GNSSState.SLEEP] = 1 - p_acq
        
        # Normalize to ensure sum = 1
        total = np.sum(weights)
        if total > 0:
            weights /= total
        else:
            weights[GNSSState.SLEEP] = 1.0
        
        return weights
    
    def power_consumption(self, snr: float, time_since_lock: float = 0.0,
                          requested: bool = True) -> float:
        """
        Calculate instantaneous power consumption.
        
        P = Σ w_i * P_i
        
        Args:
            snr: Effective SNR (dB-Hz)
            time_since_lock: Time since last lock (hours)
            requested: GPS requested by app
            
        Returns:
            Power consumption (W)
        """
        p = self.params
        
        weights = self.state_weights(snr, time_since_lock, requested)
        
        powers = np.array([
            p.P_sleep,
            p.P_acq_cold,
            p.P_acq_hot,
            p.P_tracking
        ])
        
        return np.dot(weights, powers)
    
    def satellites_visible(self, snr: float) -> int:
        """
        Estimate number of visible satellites based on SNR.
        """
        p = self.params
        
        # Each constellation typically has 6-10 satellites visible
        max_satellites = 8 * p.n_constellations
        
        # Visibility probability
        p_vis = self.sigmoid(snr, 0.5, 20)
        
        return int(max_satellites * p_vis)
    
    def position_accuracy(self, snr: float, n_satellites: int) -> float:
        """
        Estimate horizontal position error (meters).
        
        Based on HDOP and SNR relationship.
        """
        if n_satellites < 4:
            return float('inf')  # Cannot get fix
        
        # Geometric dilution of precision (approximate)
        hdop = 2.0 + 8.0 / n_satellites
        
        # SNR-based range error
        range_error = 10.0 * np.exp(-snr / 20)
        
        return hdop * range_error
    
    def continuous_power_model(self, t: float,
                               snr_func: Callable[[float], float],
                               request_func: Callable[[float], bool],
                               lock_time_ref: float = 0.0) -> float:
        """
        Continuous-time power model for ODE integration.
        
        Args:
            t: Time (hours)
            snr_func: SNR as function of time
            request_func: GPS request state as function of time
            lock_time_ref: Reference time of last lock
            
        Returns:
            Power consumption (W)
        """
        snr = snr_func(t)
        requested = request_func(t)
        time_since_lock = max(t - lock_time_ref, 0)
        
        return self.power_consumption(snr, time_since_lock, requested)
    
    def current_draw(self, snr: float, V_bat: float = 3.85,
                     time_since_lock: float = 0.0,
                     requested: bool = True) -> float:
        """
        Calculate current draw from battery.
        """
        P = self.power_consumption(snr, time_since_lock, requested)
        return P / V_bat


class GNSSEnvironmentModel:
    """
    Environmental model for GPS signal quality.
    Generates realistic SNR profiles for different scenarios.
    """
    
    @staticmethod
    def outdoor_stationary() -> Callable[[float], float]:
        """Open sky, stationary - best case."""
        def snr_func(t):
            # Stable high SNR with minor variations
            base = 42.0
            variation = 2.0 * np.sin(2 * np.pi * t / 0.25)  # Slow variation
            return base + variation
        return snr_func
    
    @staticmethod
    def urban_walking() -> Callable[[float], float]:
        """Walking in urban area with buildings."""
        def snr_func(t):
            # Moderate SNR with significant variations
            base = 32.0
            # Building shadows every ~30 seconds
            shadow = 8.0 * np.abs(np.sin(2 * np.pi * t / 0.0083))
            # Random multipath
            noise = 3.0 * np.sin(50 * t)
            return base - shadow + noise
        return snr_func
    
    @staticmethod
    def driving_highway() -> Callable[[float], float]:
        """Highway driving - generally good signal."""
        def snr_func(t):
            base = 38.0
            # Occasional overpasses
            if (t * 60) % 5 < 0.2:  # Overpass every 5 minutes
                return 15.0
            return base + 2.0 * np.random.randn()
        return snr_func
    
    @staticmethod
    def indoor_office() -> Callable[[float], float]:
        """Indoor office environment."""
        def snr_func(t):
            # Weak signal with variations from window proximity
            base = 22.0
            variation = 5.0 * np.sin(2 * np.pi * t / 0.5)
            return base + variation
        return snr_func
    
    @staticmethod
    def subway_commute() -> Callable[[float], float]:
        """Subway/metro with stations."""
        def snr_func(t):
            # Cycle through tunnel (no signal) and stations (some signal)
            period = 0.05  # 3 minutes between stations
            phase = (t % period) / period
            
            if phase < 0.15:  # At station
                return 25.0 + 5.0 * (0.15 - abs(phase - 0.075)) / 0.075
            else:  # In tunnel
                return 5.0  # Essentially blocked
        return snr_func
    
    @staticmethod
    def mixed_environment(schedule: List[Tuple[float, str]]) -> Callable[[float], float]:
        """
        Mixed environment based on schedule.
        
        Args:
            schedule: List of (start_time, environment) tuples
        """
        env_funcs = {
            'outdoor': GNSSEnvironmentModel.outdoor_stationary(),
            'urban': GNSSEnvironmentModel.urban_walking(),
            'highway': GNSSEnvironmentModel.driving_highway(),
            'indoor': GNSSEnvironmentModel.indoor_office(),
            'subway': GNSSEnvironmentModel.subway_commute()
        }
        
        def snr_func(t):
            # Find current environment
            current_env = 'outdoor'
            for start_time, env in schedule:
                if t >= start_time:
                    current_env = env
            
            return env_funcs.get(current_env, env_funcs['outdoor'])(t)
        
        return snr_func


def analyze_gnss_state_dynamics():
    """
    Analyze GNSS state transition dynamics for visualization.
    """
    module = GNSSModule()
    
    # SNR range
    snr_range = np.linspace(10, 50, 100)
    
    # State weights at different SNR levels
    weights_matrix = np.zeros((100, 4))
    power_values = np.zeros(100)
    
    for i, snr in enumerate(snr_range):
        weights_matrix[i] = module.state_weights(snr, time_since_lock=0.5, requested=True)
        power_values[i] = module.power_consumption(snr, time_since_lock=0.5, requested=True)
    
    return {
        'snr_range': snr_range,
        'weights': weights_matrix,
        'power': power_values,
        'state_names': ['Sleep', 'Cold Acq', 'Hot Acq', 'Tracking']
    }


if __name__ == "__main__":
    # Test GNSS module
    module = GNSSModule()
    
    print("GNSS Module Power Analysis")
    print("=" * 50)
    
    # Test various SNR scenarios
    snr_levels = [45, 35, 28, 22, 15, 10]
    
    for snr in snr_levels:
        P = module.power_consumption(snr, time_since_lock=0.1, requested=True)
        I = module.current_draw(snr)
        weights = module.state_weights(snr, time_since_lock=0.1, requested=True)
        n_sats = module.satellites_visible(snr)
        
        print(f"\nSNR = {snr} dB-Hz:")
        print(f"  Power: {P*1000:.1f} mW, Current: {I*1000:.1f} mA")
        print(f"  Visible satellites: {n_sats}")
        print(f"  State weights: Sleep={weights[0]:.2f}, Cold={weights[1]:.2f}, "
              f"Hot={weights[2]:.2f}, Track={weights[3]:.2f}")
    
    # Test idle
    P_idle = module.power_consumption(40, requested=False)
    print(f"\nGPS Off (Sleep): {P_idle*1000:.2f} mW")
