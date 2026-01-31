"""
5G Communication Module Power Consumption Model
===============================================

This module implements a physics-based continuous-time model for 5G 
transceiver power consumption, incorporating:
1. Shannon-Hartley theorem for required SNR
2. Friis transmission equation for path loss
3. Power amplifier (PA) nonlinear efficiency
4. Baseband processing power scaling

Key insight: Power consumption exhibits nonlinear coupling between 
data rate and distance to base station.

References:
- 3GPP TS 38.101-1: NR User Equipment (UE) radio transmission and reception
- Qualcomm Snapdragon X65 modem power specifications
- IEEE papers on 5G energy efficiency
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Tuple
from scipy.special import erfc


@dataclass
class Network5GParameters:
    """
    5G NR (New Radio) module parameters.
    
    Based on typical smartphone 5G implementations:
    - FR1 (Sub-6 GHz): 3.5 GHz band
    - FR2 (mmWave): 28 GHz band
    """
    # Operating frequency (Hz)
    f_carrier: float = 3.5e9  # 3.5 GHz n78 band
    
    # Channel bandwidth (Hz)
    B_channel: float = 100e6  # 100 MHz
    
    # Number of component carriers
    n_carriers: int = 1
    
    # Maximum transmit power (W)
    P_tx_max: float = 0.2  # 23 dBm UE power class 3
    
    # PA efficiency at max power
    eta_pa_max: float = 0.45
    
    # PA backoff factor
    pa_backoff: float = 3.0  # dB
    
    # Baseband processing power coefficients
    P_bb_idle: float = 0.1  # Idle baseband (W)
    k_bb: float = 2e-9  # W per bit/s
    
    # RF frontend power
    P_rf: float = 0.15  # LNA, mixer, etc. (W)
    
    # Path loss exponent (2-4, urban typically 3.5)
    n_path: float = 3.5
    
    # Reference distance (m)
    d_ref: float = 1.0
    
    # Reference path loss at d_ref (dB)
    L_ref: float = 40  # Free space at 1m, 3.5 GHz
    
    # Noise figure (dB)
    NF: float = 7.0
    
    # Noise spectral density (dBm/Hz)
    N_0: float = -174  # Thermal noise floor
    
    # Target BER
    BER_target: float = 1e-6
    
    # DRX (Discontinuous Reception) parameters
    drx_cycle: float = 320e-3  # DRX cycle (s)
    drx_on_duration: float = 10e-3  # On duration (s)


class Network5GModule:
    """
    Continuous-time 5G power consumption model.
    
    Total power consumption:
        P_5G(t) = P_tx(t) / η_PA + P_bb(R(t)) + P_rf + P_drx_overhead
    
    Where transmit power depends on required SNR for data rate R(t)
    and path loss to base station at distance d(t).
    """
    
    def __init__(self, params: Optional[Network5GParameters] = None):
        self.params = params or Network5GParameters()
        
    def path_loss(self, d: float) -> float:
        """
        Calculate path loss using log-distance model.
        
        L(d) = L_ref + 10 * n * log10(d / d_ref)
        
        Args:
            d: Distance to base station (m)
            
        Returns:
            Path loss (linear scale)
        """
        p = self.params
        
        d = max(d, p.d_ref)  # Prevent log(0)
        
        # Path loss in dB
        L_dB = p.L_ref + 10 * p.n_path * np.log10(d / p.d_ref)
        
        # Convert to linear
        L_linear = 10 ** (L_dB / 10)
        
        return L_linear
    
    def required_snr(self, R: float) -> float:
        """
        Calculate required SNR for data rate R using Shannon-Hartley theorem.
        
        C = B * log2(1 + SNR)
        => SNR = 2^(R/B) - 1
        
        With implementation margin for practical modulation schemes.
        
        Args:
            R: Required data rate (bits/s)
            
        Returns:
            Required SNR (linear)
        """
        p = self.params
        
        # Shannon limit
        spectral_efficiency = R / p.B_channel
        snr_shannon = 2 ** spectral_efficiency - 1
        
        # Implementation gap (typically 3-6 dB for practical systems)
        gap_dB = 4.5
        gap_linear = 10 ** (gap_dB / 10)
        
        return snr_shannon * gap_linear
    
    def noise_power(self) -> float:
        """
        Calculate total noise power at receiver.
        
        N = k * T * B * NF
        """
        p = self.params
        
        # Noise power in dBm
        N_dBm = p.N_0 + 10 * np.log10(p.B_channel) + p.NF
        
        # Convert to Watts
        N_W = 10 ** ((N_dBm - 30) / 10)
        
        return N_W
    
    def transmit_power(self, R: float, d: float) -> float:
        """
        Calculate required transmit power for rate R at distance d.
        
        P_tx = SNR_req * N * L(d)
        
        Args:
            R: Data rate (bits/s)
            d: Distance to base station (m)
            
        Returns:
            Required transmit power (W)
        """
        if R <= 0:
            return 0
        
        p = self.params
        
        snr_req = self.required_snr(R)
        N = self.noise_power()
        L = self.path_loss(d)
        
        P_tx = snr_req * N * L
        
        # Cap at maximum transmit power
        P_tx = min(P_tx, p.P_tx_max)
        
        return P_tx
    
    def pa_efficiency(self, P_tx: float) -> float:
        """
        Power amplifier efficiency as function of output power.
        
        Efficiency drops significantly at low power levels due to 
        PA operating away from saturation point.
        
        Model: η(P) = η_max * (P/P_max)^0.5 for Class A/AB amplifiers
        """
        p = self.params
        
        if P_tx <= 0:
            return 0.1  # Minimum efficiency
        
        # Normalized power level
        P_norm = P_tx / p.P_tx_max
        
        # Efficiency model (typical PA characteristic)
        eta = p.eta_pa_max * np.sqrt(P_norm)
        
        # Ensure minimum efficiency
        eta = max(eta, 0.1)
        
        return eta
    
    def baseband_power(self, R: float) -> float:
        """
        Baseband processing power consumption.
        
        Scales approximately linearly with data rate due to:
        - FFT/IFFT operations for OFDM
        - Channel coding/decoding
        - MIMO processing
        
        P_bb = P_idle + k_bb * R
        """
        p = self.params
        
        return p.P_bb_idle + p.k_bb * max(R, 0)
    
    def total_power(self, R: float, d: float, active: bool = True) -> float:
        """
        Calculate total 5G module power consumption.
        
        P_total = P_tx/η_PA + P_bb + P_rf (active)
        P_total = P_drx (idle with DRX)
        
        Args:
            R: Data rate (bits/s)
            d: Distance to base station (m)
            active: Whether actively transmitting/receiving
            
        Returns:
            Total power consumption (W)
        """
        p = self.params
        
        if not active:
            # DRX power consumption
            duty_cycle = p.drx_on_duration / p.drx_cycle
            return (p.P_bb_idle * 0.3 + p.P_rf * 0.5) * duty_cycle
        
        # Transmit power
        P_tx = self.transmit_power(R, d)
        
        # PA power consumption
        eta = self.pa_efficiency(P_tx)
        P_pa = P_tx / max(eta, 0.1)
        
        # Baseband power
        P_bb = self.baseband_power(R)
        
        # Total
        P_total = P_pa + P_bb + p.P_rf
        
        return P_total
    
    def power_current(self, R: float, d: float, V_bat: float = 3.85,
                      active: bool = True) -> float:
        """
        Convert power to battery current.
        
        Args:
            R: Data rate (bits/s)
            d: Distance to base station (m)
            V_bat: Battery voltage (V)
            active: Active state
            
        Returns:
            Current draw (A)
        """
        P = self.total_power(R, d, active)
        return P / V_bat
    
    def continuous_power_model(self, t: float, 
                               R_func: Callable[[float], float],
                               d_func: Callable[[float], float],
                               activity_func: Optional[Callable[[float], bool]] = None
                               ) -> float:
        """
        Continuous-time power consumption model.
        
        P_5G(t) = f(R(t), d(t), active(t))
        
        Args:
            t: Time (hours)
            R_func: Data rate as function of time
            d_func: Distance to BS as function of time
            activity_func: Activity state function
            
        Returns:
            Instantaneous power (W)
        """
        R = R_func(t)
        d = d_func(t)
        active = activity_func(t) if activity_func else (R > 0)
        
        return self.total_power(R, d, active)


class Network5GScenarios:
    """
    Predefined usage scenarios for 5G module.
    """
    
    @staticmethod
    def video_streaming_4k() -> Tuple[Callable, Callable]:
        """
        4K video streaming scenario.
        Data rate: ~25 Mbps with buffering variations
        """
        def R_func(t):
            # Base rate with buffering bursts
            base_rate = 25e6
            burst = 15e6 * np.sin(2 * np.pi * t / 0.1) ** 2  # Burst every 6 min
            return base_rate + burst
        
        def d_func(t):
            # Stationary indoor, 200m from BS
            return 200
        
        return R_func, d_func
    
    @staticmethod
    def web_browsing() -> Tuple[Callable, Callable]:
        """
        Web browsing with intermittent data bursts.
        """
        def R_func(t):
            # Bursty traffic pattern
            period = 0.05  # 3 minutes per page
            phase = (t % period) / period
            if phase < 0.1:  # Loading phase
                return 10e6 * (1 - phase / 0.1)
            else:
                return 0.1e6  # Background sync
        
        def d_func(t):
            return 150  # Indoor
        
        return R_func, d_func
    
    @staticmethod
    def mobile_gaming() -> Tuple[Callable, Callable]:
        """
        Online mobile gaming with real-time data.
        """
        def R_func(t):
            # Consistent low-latency traffic
            return 2e6 + 0.5e6 * np.random.randn()
        
        def d_func(t):
            return 100
        
        return R_func, d_func
    
    @staticmethod
    def commuting() -> Tuple[Callable, Callable]:
        """
        User on public transit with varying signal conditions.
        """
        def R_func(t):
            # Video call during commute
            return 3e6
        
        def d_func(t):
            # Distance varies as user moves
            base_d = 300
            variation = 400 * np.sin(2 * np.pi * t / 0.5)  # ~30 min cycle
            return max(base_d + variation, 50)
        
        return R_func, d_func
    
    @staticmethod
    def edge_signal() -> Tuple[Callable, Callable]:
        """
        User at cell edge with poor signal.
        Critical scenario for battery drain.
        """
        def R_func(t):
            # Moderate data usage
            return 5e6
        
        def d_func(t):
            # At cell edge, ~800m
            return 800
        
        return R_func, d_func


def analyze_5g_power_sensitivity():
    """
    Analyze 5G power consumption sensitivity to distance and rate.
    Returns data for visualization.
    """
    module = Network5GModule()
    
    # Parameter ranges
    distances = np.linspace(50, 1000, 50)
    rates = np.linspace(1e6, 100e6, 50)
    
    # Create meshgrid
    D, R = np.meshgrid(distances, rates)
    
    # Calculate power for each combination
    P = np.zeros_like(D)
    for i in range(len(rates)):
        for j in range(len(distances)):
            P[i, j] = module.total_power(rates[i], distances[j])
    
    return {
        'distances': distances,
        'rates': rates,
        'D_mesh': D,
        'R_mesh': R,
        'power': P,
        'current': P / 3.85  # Convert to current
    }


if __name__ == "__main__":
    # Test 5G module
    module = Network5GModule()
    
    print("5G Module Power Analysis")
    print("=" * 50)
    
    # Test various scenarios
    scenarios = [
        ("Close to BS (100m), low rate (1 Mbps)", 1e6, 100),
        ("Close to BS (100m), high rate (50 Mbps)", 50e6, 100),
        ("Far from BS (500m), low rate (1 Mbps)", 1e6, 500),
        ("Far from BS (500m), high rate (50 Mbps)", 50e6, 500),
        ("Cell edge (800m), moderate rate (10 Mbps)", 10e6, 800),
    ]
    
    for name, R, d in scenarios:
        P = module.total_power(R, d)
        I = module.power_current(R, d)
        print(f"{name}:")
        print(f"  Power: {P*1000:.1f} mW, Current: {I*1000:.1f} mA")
    
    # DRX idle
    P_idle = module.total_power(0, 200, active=False)
    print(f"\nDRX Idle: {P_idle*1000:.2f} mW")
