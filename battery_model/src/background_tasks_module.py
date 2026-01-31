"""
Background Tasks Stochastic Power Consumption Model
===================================================

This module models the power consumption of background tasks using
stochastic differential equations. Background tasks exhibit:

1. Mean-reverting behavior (O-U process)
2. Periodic burst patterns (system wakeups)
3. Correlated multi-component behavior (CPU, network, disk)

The model captures the heavy-tailed distribution observed in real
smartphone power consumption data.

Key Equations:
    dI(t) = -θ(I(t) - μ)dt + σdW_t + burst(t)

References:
- Android doze mode and app standby documentation
- iOS background task scheduling
- Academic papers on smartphone energy profiling
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, List
from scipy.stats import levy_stable


@dataclass
class BackgroundTaskParameters:
    """
    Parameters for background task power model.
    
    Based on measurements from Android Battery Historian and
    iOS Energy Impact tools.
    """
    # Base leakage current (A) - always present
    I_leakage: float = 0.015  # ~15 mA baseline
    
    # O-U process parameters
    theta: float = 2.0  # Mean reversion rate (/hour)
    mu: float = 0.030   # Mean current (A) - ~30 mA average
    sigma: float = 0.008  # Volatility (A)
    
    # Burst (wake) parameters
    burst_interval: float = 0.05  # Average burst every 3 minutes (hours)
    burst_amplitude: float = 0.150  # Burst current (A) - ~150 mA
    burst_duration: float = 0.001  # Burst duration ~3.6s (hours)
    
    # Component correlation matrix parameters
    # [CPU, Network, Storage, Display controller]
    n_components: int = 4
    
    # CPU parameters
    cpu_idle: float = 0.020  # Idle current
    cpu_active: float = 0.800  # Active current
    
    # Network (WiFi/cellular standby)
    net_idle: float = 0.008
    net_sync: float = 0.100
    
    # Storage (flash controller)
    storage_idle: float = 0.002
    storage_active: float = 0.080
    
    # Display controller (screen off)
    display_standby: float = 0.005


class OrnsteinUhlenbeckProcess:
    """
    Ornstein-Uhlenbeck stochastic process for mean-reverting behavior.
    
    dX(t) = θ(μ - X(t))dt + σdW_t
    
    Exact solution at time t given X(0) = x0:
    X(t) = μ + (x0 - μ)e^(-θt) + σ∫₀ᵗ e^(-θ(t-s))dW_s
    """
    
    def __init__(self, theta: float, mu: float, sigma: float, x0: float = None):
        self.theta = theta
        self.mu = mu
        self.sigma = sigma
        self.x0 = x0 if x0 is not None else mu
        self._current = self.x0
        
    def mean(self, t: float) -> float:
        """Expected value at time t."""
        return self.mu + (self.x0 - self.mu) * np.exp(-self.theta * t)
    
    def variance(self, t: float) -> float:
        """Variance at time t."""
        return (self.sigma ** 2 / (2 * self.theta)) * (1 - np.exp(-2 * self.theta * t))
    
    def std(self, t: float) -> float:
        """Standard deviation at time t."""
        return np.sqrt(self.variance(t))
    
    def sample(self, t: float, dt: float = None) -> float:
        """
        Sample from the process at time t.
        
        Uses exact simulation for numerical stability.
        """
        if dt is None:
            dt = t
        
        mean = self.mean(dt)
        std = self.std(dt)
        
        return np.random.normal(mean, std)
    
    def path(self, t_array: np.ndarray, x0: float = None) -> np.ndarray:
        """
        Generate a sample path over time array.
        """
        if x0 is not None:
            self.x0 = x0
            self._current = x0
        
        n = len(t_array)
        path = np.zeros(n)
        path[0] = self._current
        
        for i in range(1, n):
            dt = t_array[i] - t_array[i-1]
            
            # Exact discrete sampling
            decay = np.exp(-self.theta * dt)
            mean = self.mu + (path[i-1] - self.mu) * decay
            var = (self.sigma ** 2 / (2 * self.theta)) * (1 - decay ** 2)
            
            path[i] = np.random.normal(mean, np.sqrt(var))
        
        return path


class BurstProcess:
    """
    Models periodic and random burst events (system wakeups).
    
    Combines:
    - Periodic wakeups (alarm manager, sync adapters)
    - Random push notifications
    - App-specific background activity
    """
    
    def __init__(self, interval: float = 0.05, amplitude: float = 0.15,
                 duration: float = 0.001):
        self.interval = interval
        self.amplitude = amplitude
        self.duration = duration
        self._next_burst = 0
        self._burst_times = []
        
    def generate_burst_times(self, t_end: float, seed: int = None) -> List[float]:
        """
        Generate burst event times using a Poisson process.
        """
        if seed is not None:
            np.random.seed(seed)
        
        times = []
        t = 0
        while t < t_end:
            # Exponential inter-arrival time
            dt = np.random.exponential(self.interval)
            t += dt
            if t < t_end:
                times.append(t)
        
        self._burst_times = times
        return times
    
    def burst_current(self, t: float, burst_times: List[float] = None) -> float:
        """
        Calculate burst contribution at time t.
        
        Uses smooth pulse function for differentiability.
        """
        if burst_times is None:
            burst_times = self._burst_times
        
        total_burst = 0
        for tb in burst_times:
            # Smooth pulse: Gaussian-like burst
            delta = (t - tb) / self.duration
            if abs(delta) < 5:  # Only compute nearby bursts
                pulse = np.exp(-delta ** 2 / 2)
                total_burst += self.amplitude * pulse
        
        return total_burst


class CorrelatedComponentModel:
    """
    Models correlated power consumption across hardware components.
    
    Different subsystems (CPU, network, storage) have correlated
    activity patterns - e.g., network sync triggers CPU and storage.
    
    Uses multivariate O-U process with correlation structure.
    """
    
    def __init__(self, params: BackgroundTaskParameters):
        self.params = params
        
        # Component parameters [idle, active, sigma]
        self.components = {
            'cpu': (params.cpu_idle, params.cpu_active, 0.05),
            'network': (params.net_idle, params.net_sync, 0.02),
            'storage': (params.storage_idle, params.storage_active, 0.01),
            'display': (params.display_standby, params.display_standby * 2, 0.002)
        }
        
        # Correlation matrix
        self.correlation = np.array([
            [1.0, 0.6, 0.4, 0.2],   # CPU correlates with network, storage
            [0.6, 1.0, 0.5, 0.1],   # Network triggers storage
            [0.4, 0.5, 1.0, 0.1],   # Storage
            [0.2, 0.1, 0.1, 1.0]    # Display controller mostly independent
        ])
        
        # Cholesky decomposition for correlated sampling
        self.chol = np.linalg.cholesky(self.correlation)
    
    def sample_correlated(self, activity_level: float = 0.1) -> np.ndarray:
        """
        Sample correlated component currents.
        
        Args:
            activity_level: 0-1 indicating background activity intensity
            
        Returns:
            Array of currents for each component
        """
        # Generate correlated standard normals
        z = np.random.randn(4)
        correlated_z = self.chol @ z
        
        currents = np.zeros(4)
        for i, (name, (idle, active, sigma)) in enumerate(self.components.items()):
            # Interpolate between idle and active based on activity
            mean = idle + activity_level * (active - idle)
            currents[i] = mean + sigma * correlated_z[i]
            currents[i] = max(currents[i], 0)  # Non-negative current
        
        return currents


class BackgroundTasksModule:
    """
    Complete background tasks power consumption model.
    
    Combines O-U process, burst events, and correlated components
    into a unified continuous-time model.
    
    Total current:
        I_bg(t) = I_leakage + I_ou(t) + I_burst(t) + Σ I_component(t)
    """
    
    def __init__(self, params: Optional[BackgroundTaskParameters] = None):
        self.params = params or BackgroundTaskParameters()
        
        # Initialize sub-models
        self.ou_process = OrnsteinUhlenbeckProcess(
            theta=self.params.theta,
            mu=self.params.mu,
            sigma=self.params.sigma
        )
        
        self.burst_process = BurstProcess(
            interval=self.params.burst_interval,
            amplitude=self.params.burst_amplitude,
            duration=self.params.burst_duration
        )
        
        self.component_model = CorrelatedComponentModel(self.params)
        
        self._burst_times = []
        self._ou_path = None
        self._time_array = None
        
    def initialize_simulation(self, t_end: float, dt: float = 0.0001,
                             seed: int = None) -> None:
        """
        Pre-generate stochastic paths for simulation.
        
        This ensures consistent results across multiple queries.
        """
        if seed is not None:
            np.random.seed(seed)
        
        # Generate time array
        n_points = int(t_end / dt) + 1
        self._time_array = np.linspace(0, t_end, n_points)
        
        # Generate O-U path
        self._ou_path = self.ou_process.path(self._time_array)
        
        # Generate burst times
        self._burst_times = self.burst_process.generate_burst_times(t_end)
        
    def current_at_time(self, t: float, interpolate: bool = True) -> float:
        """
        Get background current at specific time.
        
        If simulation not initialized, generates on-the-fly.
        """
        p = self.params
        
        # Base leakage
        I_total = p.I_leakage
        
        # O-U component
        if self._ou_path is not None and interpolate:
            # Interpolate from pre-generated path
            idx = np.searchsorted(self._time_array, t)
            idx = min(idx, len(self._ou_path) - 1)
            I_total += self._ou_path[idx]
        else:
            # Generate on-the-fly
            I_total += self.ou_process.sample(t)
        
        # Burst component
        I_total += self.burst_process.burst_current(t, self._burst_times)
        
        return max(I_total, 0)
    
    def power_consumption(self, t: float, V_bat: float = 3.85) -> float:
        """
        Calculate power consumption at time t.
        """
        I = self.current_at_time(t)
        return I * V_bat
    
    def generate_sample_path(self, t_array: np.ndarray, 
                            activity_schedule: Callable[[float], float] = None,
                            seed: int = None) -> dict:
        """
        Generate a complete sample path with all components.
        
        Args:
            t_array: Time points
            activity_schedule: Function returning activity level (0-1)
            seed: Random seed
            
        Returns:
            Dictionary with current components and total
        """
        if seed is not None:
            np.random.seed(seed)
        
        n = len(t_array)
        
        # Initialize arrays
        I_leakage = np.full(n, self.params.I_leakage)
        I_ou = self.ou_process.path(t_array)
        I_burst = np.zeros(n)
        I_components = np.zeros((n, 4))
        
        # Generate burst times
        burst_times = self.burst_process.generate_burst_times(t_array[-1])
        
        for i, t in enumerate(t_array):
            # Burst current
            I_burst[i] = self.burst_process.burst_current(t, burst_times)
            
            # Correlated components
            activity = activity_schedule(t) if activity_schedule else 0.1
            I_components[i] = self.component_model.sample_correlated(activity)
        
        # Total current
        I_total = I_leakage + I_ou + I_burst + np.sum(I_components, axis=1)
        I_total = np.maximum(I_total, 0)
        
        return {
            'time': t_array,
            'I_total': I_total,
            'I_leakage': I_leakage,
            'I_ou': I_ou,
            'I_burst': I_burst,
            'I_components': I_components,
            'burst_times': burst_times,
            'component_names': ['CPU', 'Network', 'Storage', 'Display']
        }


def analyze_background_statistics():
    """
    Analyze statistical properties of background current.
    Returns data for visualization.
    """
    module = BackgroundTasksModule()
    
    # Generate long sample path
    t_array = np.linspace(0, 2, 10000)  # 2 hours, high resolution
    data = module.generate_sample_path(t_array, seed=42)
    
    # Statistical analysis
    I = data['I_total']
    
    # Histogram bins
    bins = np.linspace(0, 0.3, 100)
    hist, bin_edges = np.histogram(I, bins=bins, density=True)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    
    # Fit parameters
    mean = np.mean(I)
    std = np.std(I)
    skew = np.mean((I - mean) ** 3) / std ** 3
    kurtosis = np.mean((I - mean) ** 4) / std ** 4 - 3
    
    # Percentiles (for heavy tail analysis)
    percentiles = [50, 90, 95, 99, 99.9]
    percentile_values = [np.percentile(I, p) for p in percentiles]
    
    return {
        'time': t_array,
        'current': I,
        'components': data['I_components'],
        'burst_times': data['burst_times'],
        'histogram': (bin_centers, hist),
        'statistics': {
            'mean': mean,
            'std': std,
            'skew': skew,
            'kurtosis': kurtosis,
            'percentiles': dict(zip(percentiles, percentile_values))
        }
    }


if __name__ == "__main__":
    # Test background tasks module
    module = BackgroundTasksModule()
    
    print("Background Tasks Module Analysis")
    print("=" * 50)
    
    # Generate sample path
    t = np.linspace(0, 1, 1000)  # 1 hour
    data = module.generate_sample_path(t, seed=42)
    
    I = data['I_total']
    print(f"\n1-hour sample statistics:")
    print(f"  Mean current: {np.mean(I)*1000:.1f} mA")
    print(f"  Std current: {np.std(I)*1000:.1f} mA")
    print(f"  Max current: {np.max(I)*1000:.1f} mA")
    print(f"  95th percentile: {np.percentile(I, 95)*1000:.1f} mA")
    print(f"  Number of bursts: {len(data['burst_times'])}")
    
    # Power consumption
    P = I * 3.85
    print(f"\nPower consumption:")
    print(f"  Mean: {np.mean(P)*1000:.1f} mW")
    print(f"  Peak: {np.max(P)*1000:.1f} mW")
