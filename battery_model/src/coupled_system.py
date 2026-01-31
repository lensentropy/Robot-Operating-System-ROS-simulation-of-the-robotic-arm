"""
Coupled System Integration for Smartphone Battery Model
======================================================

This module integrates all subsystem models into a unified continuous-time
differential equation system for complete smartphone battery simulation.

System Architecture:
    dS(t)/dt = -I_total(t) / (C_eff(I,T) * η)
    
    where I_total = I_screen + I_cpu + I_5g + I_gnss + I_bluetooth + I_background

The key innovation is modeling the coupling between subsystems:
1. Network-GPS coupling (e.g., AGPS data download)
2. CPU-Network coupling (data processing)
3. Thermal coupling (all components contribute to heating)

References:
- Smartphone energy profiling literature
- Android Battery Historian data
- iOS Energy Impact measurements
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field
from typing import Callable, Optional, Dict, List, Tuple
import warnings

# Import subsystem modules
from battery_core import BatteryCore, BatteryParameters, ThermalModel
from network_5g_module import Network5GModule, Network5GParameters
from gnss_module import GNSSModule, GNSSParameters
from background_tasks_module import BackgroundTasksModule, BackgroundTaskParameters
from bluetooth_module import BluetoothModule, BluetoothParameters


@dataclass
class ScreenParameters:
    """
    Screen (display) power consumption parameters.
    
    Based on typical OLED/AMOLED smartphone displays.
    Reference: DisplayMate measurements, Samsung/LG OLED data
    """
    # Display size
    diagonal_inches: float = 6.7
    resolution_x: int = 1440
    resolution_y: int = 3200
    
    # Power at different brightness levels (W)
    P_off: float = 0.001  # Display controller only
    P_min_brightness: float = 0.3  # Minimum visible brightness
    P_max_brightness: float = 2.5  # Maximum brightness (sunlight)
    
    # Refresh rate power scaling
    P_60Hz: float = 1.0  # Base power at 60Hz
    P_120Hz: float = 1.15  # 15% more at 120Hz
    P_144Hz: float = 1.22  # 22% more at 144Hz
    
    # Content-dependent power (OLED)
    # Dark content uses less power
    avg_pixel_luminance: float = 0.5  # 0-1 scale


@dataclass
class CPUParameters:
    """
    CPU/SoC power consumption parameters.
    
    Based on Snapdragon 8 Gen 2 / A16 Bionic class SoCs.
    """
    # Power states
    P_idle: float = 0.15  # Deep sleep
    P_light: float = 0.5  # Light tasks
    P_medium: float = 1.5  # Normal usage
    P_heavy: float = 4.0  # Gaming/computation
    P_peak: float = 8.0   # Maximum performance
    
    # DVFS scaling factor
    alpha_dvfs: float = 2.0  # P ∝ V² * f, approx P ∝ f^α


@dataclass
class CoupledSystemParameters:
    """
    Parameters for the complete coupled system.
    """
    battery: BatteryParameters = field(default_factory=BatteryParameters)
    screen: ScreenParameters = field(default_factory=ScreenParameters)
    cpu: CPUParameters = field(default_factory=CPUParameters)
    network_5g: Network5GParameters = field(default_factory=Network5GParameters)
    gnss: GNSSParameters = field(default_factory=GNSSParameters)
    background: BackgroundTaskParameters = field(default_factory=BackgroundTaskParameters)
    bluetooth: BluetoothParameters = field(default_factory=BluetoothParameters)
    
    # Coupling coefficients
    # CPU overhead for network data processing
    cpu_network_coupling: float = 0.05  # 50 mW per 10 Mbps
    
    # CPU overhead for GPS processing
    cpu_gnss_coupling: float = 0.02  # 20 mW when GPS active
    
    # Network for AGPS
    network_gnss_coupling: float = 0.01  # Brief data for AGPS


class ScreenModule:
    """
    Screen power consumption model.
    
    For OLED: P = P_base + P_pixel * brightness * luminance_factor
    """
    
    def __init__(self, params: Optional[ScreenParameters] = None):
        self.params = params or ScreenParameters()
        
    def power(self, brightness: float = 0.5, refresh_rate: int = 60,
              content_brightness: float = 0.5, on: bool = True) -> float:
        """
        Calculate display power consumption.
        
        Args:
            brightness: Display brightness (0-1)
            refresh_rate: Refresh rate (Hz)
            content_brightness: Average content luminance (0-1)
            on: Whether display is on
        """
        p = self.params
        
        if not on:
            return p.P_off
        
        # Base power interpolation by brightness
        base_power = p.P_min_brightness + brightness * (p.P_max_brightness - p.P_min_brightness)
        
        # OLED content-dependent power
        # Dark content uses less power (black pixels are off)
        content_factor = 0.3 + 0.7 * content_brightness
        
        # Refresh rate scaling
        if refresh_rate >= 144:
            rate_factor = p.P_144Hz
        elif refresh_rate >= 120:
            rate_factor = p.P_120Hz
        else:
            rate_factor = p.P_60Hz
        
        return base_power * content_factor * rate_factor


class CPUModule:
    """
    CPU/SoC power consumption model.
    
    Simple load-based model with DVFS scaling.
    """
    
    def __init__(self, params: Optional[CPUParameters] = None):
        self.params = params or CPUParameters()
    
    def power(self, load: float = 0.1) -> float:
        """
        Calculate CPU power based on load level.
        
        Args:
            load: CPU utilization (0-1)
        """
        p = self.params
        
        if load < 0.05:
            return p.P_idle
        elif load < 0.2:
            return p.P_idle + (p.P_light - p.P_idle) * (load / 0.2)
        elif load < 0.5:
            return p.P_light + (p.P_medium - p.P_light) * ((load - 0.2) / 0.3)
        elif load < 0.8:
            return p.P_medium + (p.P_heavy - p.P_medium) * ((load - 0.5) / 0.3)
        else:
            return p.P_heavy + (p.P_peak - p.P_heavy) * ((load - 0.8) / 0.2)


class CoupledBatterySystem:
    """
    Complete coupled smartphone battery system.
    
    Integrates all subsystems with coupling effects for accurate
    battery discharge simulation.
    """
    
    def __init__(self, params: Optional[CoupledSystemParameters] = None):
        self.params = params or CoupledSystemParameters()
        
        # Initialize subsystems
        self.battery = BatteryCore(self.params.battery)
        self.thermal = ThermalModel(self.battery)
        self.screen = ScreenModule(self.params.screen)
        self.cpu = CPUModule(self.params.cpu)
        self.network_5g = Network5GModule(self.params.network_5g)
        self.gnss = GNSSModule(self.params.gnss)
        self.background = BackgroundTasksModule(self.params.background)
        self.bluetooth = BluetoothModule(self.params.bluetooth)
        
    def compute_total_current(self, t: float, state: Dict,
                              scenario: 'UsageScenario') -> Tuple[float, Dict]:
        """
        Compute total current draw from all subsystems.
        
        Args:
            t: Time (hours)
            state: Current system state (SOC, temperature)
            scenario: Usage scenario with activity patterns
            
        Returns:
            (total_current, breakdown_dict)
        """
        V_bat = self.battery.terminal_voltage(
            state['SOC'], 
            state.get('I_prev', 0.5),
            state.get('T', 298.15)
        )
        
        breakdown = {}
        
        # === Screen ===
        screen_state = scenario.screen_state(t)
        P_screen = self.screen.power(**screen_state)
        breakdown['screen'] = P_screen / V_bat
        
        # === CPU ===
        cpu_load = scenario.cpu_load(t)
        P_cpu = self.cpu.power(cpu_load)
        breakdown['cpu'] = P_cpu / V_bat
        
        # === 5G Network ===
        data_rate = scenario.data_rate(t)
        bs_distance = scenario.bs_distance(t)
        network_active = data_rate > 0
        P_5g = self.network_5g.total_power(data_rate, bs_distance, network_active)
        breakdown['5g'] = P_5g / V_bat
        
        # === GPS ===
        gnss_snr = scenario.gnss_snr(t)
        gnss_requested = scenario.gnss_requested(t)
        P_gnss = self.gnss.power_consumption(gnss_snr, requested=gnss_requested)
        breakdown['gnss'] = P_gnss / V_bat
        
        # === Bluetooth ===
        bt_state = scenario.bluetooth_state(t)
        P_bt = self.bluetooth.total_power(**bt_state)
        breakdown['bluetooth'] = P_bt / V_bat
        
        # === Background tasks ===
        I_bg = self.background.current_at_time(t)
        breakdown['background'] = I_bg
        
        # === Coupling effects ===
        # CPU overhead for network processing
        if data_rate > 0:
            coupling_cpu_net = self.params.cpu_network_coupling * (data_rate / 10e6)
            breakdown['coupling_cpu_net'] = coupling_cpu_net / V_bat
        else:
            breakdown['coupling_cpu_net'] = 0
        
        # CPU overhead for GPS
        if gnss_requested:
            breakdown['coupling_cpu_gnss'] = self.params.cpu_gnss_coupling / V_bat
        else:
            breakdown['coupling_cpu_gnss'] = 0
        
        # Total current
        I_total = sum(breakdown.values())
        breakdown['total'] = I_total
        
        return I_total, breakdown
    
    def simulate(self, scenario: 'UsageScenario',
                 S0: float = 1.0,
                 T0: float = 298.15,
                 t_span: Tuple[float, float] = (0, 24),
                 dt: float = 0.001) -> Dict:
        """
        Run complete battery discharge simulation.
        
        Args:
            scenario: Usage scenario definition
            S0: Initial SOC
            T0: Initial temperature (K)
            t_span: Simulation time span (hours)
            dt: Time step for output
            
        Returns:
            Dictionary with simulation results
        """
        # Initialize background stochastic process
        self.background.initialize_simulation(t_span[1], dt=dt/10, seed=42)
        
        # Time array
        t_eval = np.arange(t_span[0], t_span[1], dt)
        
        def ode_func(t, y):
            S, T = y
            
            # Bound state variables
            S = np.clip(S, 0.01, 1.0)
            T = np.clip(T, 273.15, 333.15)  # 0°C to 60°C
            
            if S <= 0.02:
                return [0, 0]
            
            state = {'SOC': S, 'T': T, 'I_prev': 0.5}
            I_total, _ = self.compute_total_current(t, state, scenario)
            
            # Ensure positive current
            I_total = max(I_total, 0.001)
            
            # SOC derivative
            C_eff = self.battery.effective_capacity(I_total, T)
            dSdt = -I_total / (C_eff * self.params.battery.eta_coulomb)
            
            # Temperature derivative (simplified thermal model)
            # Heat generation proportional to power dissipation
            V_bat = self.battery.terminal_voltage(S, I_total, T)
            P_total = I_total * V_bat
            
            # Simple thermal model: heating from power, cooling to ambient
            T_ambient = 298.15
            thermal_mass = 50  # J/K (phone thermal mass)
            h_cooling = 2.0  # W/K (natural convection)
            
            Q_gen = 0.05 * P_total  # 5% of power becomes heat
            Q_cool = h_cooling * (T - T_ambient)
            dTdt = (Q_gen - Q_cool) / thermal_mass * 3600  # Convert to K/hour
            
            return [dSdt, dTdt]
        
        def event_empty(t, y):
            return y[0] - 0.02  # Stop at 2% SOC
        event_empty.terminal = True
        event_empty.direction = -1
        
        # Solve ODE
        solution = solve_ivp(
            ode_func,
            t_span,
            [S0, T0],
            t_eval=t_eval,
            events=event_empty,
            method='RK45',
            max_step=0.01
        )
        
        # Extract results and compute breakdowns
        t = solution.t
        S = solution.y[0]
        T = solution.y[1]
        
        # Compute power breakdown at each time point
        breakdown_keys = ['screen', 'cpu', '5g', 'gnss', 'bluetooth', 
                         'background', 'coupling_cpu_net', 'coupling_cpu_gnss', 'total']
        breakdowns = {k: [] for k in breakdown_keys}
        
        for i, ti in enumerate(t):
            state = {'SOC': S[i], 'T': T[i]}
            _, bd = self.compute_total_current(ti, state, scenario)
            for k in breakdown_keys:
                breakdowns[k].append(bd.get(k, 0))
        
        # Convert to arrays
        for k in breakdowns:
            breakdowns[k] = np.array(breakdowns[k])
        
        # Compute voltage
        V = np.array([self.battery.terminal_voltage(Si, Ii, Ti) 
                      for Si, Ii, Ti in zip(S, breakdowns['total'], T)])
        
        return {
            'time': t,
            'SOC': S,
            'temperature': T,
            'voltage': V,
            'current': breakdowns['total'],
            'power': breakdowns['total'] * V,
            'breakdown': breakdowns,
            'success': solution.success,
            'battery_life_hours': t[-1] if S[-1] < 0.03 else None
        }


class UsageScenario:
    """
    Base class for usage scenario definitions.
    
    Subclass this to define specific usage patterns.
    """
    
    def screen_state(self, t: float) -> Dict:
        """Return screen parameters at time t."""
        return {'brightness': 0.5, 'on': True}
    
    def cpu_load(self, t: float) -> float:
        """Return CPU load (0-1) at time t."""
        return 0.1
    
    def data_rate(self, t: float) -> float:
        """Return network data rate (bits/s) at time t."""
        return 0
    
    def bs_distance(self, t: float) -> float:
        """Return distance to base station (m) at time t."""
        return 200
    
    def gnss_snr(self, t: float) -> float:
        """Return GPS signal SNR (dB-Hz) at time t."""
        return 35
    
    def gnss_requested(self, t: float) -> bool:
        """Return whether GPS is requested at time t."""
        return False
    
    def bluetooth_state(self, t: float) -> Dict:
        """Return Bluetooth state at time t."""
        return {'bt_classic_active': False, 'audio_streaming': False}


class IdleScenario(UsageScenario):
    """Screen off, minimal background activity."""
    
    def screen_state(self, t: float) -> Dict:
        return {'brightness': 0, 'on': False}
    
    def cpu_load(self, t: float) -> float:
        return 0.02
    
    def data_rate(self, t: float) -> float:
        # Occasional background sync
        if (t * 60) % 15 < 0.5:  # Every 15 minutes
            return 0.5e6
        return 0


class VideoStreamingScenario(UsageScenario):
    """4K video streaming with moderate brightness."""
    
    def screen_state(self, t: float) -> Dict:
        return {
            'brightness': 0.6,
            'refresh_rate': 60,
            'content_brightness': 0.4,  # Typical video content
            'on': True
        }
    
    def cpu_load(self, t: float) -> float:
        return 0.35  # Video decode
    
    def data_rate(self, t: float) -> float:
        # Buffering pattern
        base = 25e6
        burst = 15e6 * (np.sin(2 * np.pi * t / 0.1) > 0.8)
        return base + burst
    
    def bs_distance(self, t: float) -> float:
        return 150  # Indoor
    
    def bluetooth_state(self, t: float) -> Dict:
        # TWS earbuds for audio
        return {
            'bt_classic_active': True,
            'audio_streaming': True,
            'audio_codec': 'aac',
            'n_audio_devices': 2
        }


class NavigationScenario(UsageScenario):
    """GPS navigation while driving."""
    
    def screen_state(self, t: float) -> Dict:
        return {
            'brightness': 0.9,  # High for car use
            'refresh_rate': 60,
            'content_brightness': 0.5,
            'on': True
        }
    
    def cpu_load(self, t: float) -> float:
        return 0.4  # Map rendering
    
    def data_rate(self, t: float) -> float:
        return 2e6  # Map data
    
    def bs_distance(self, t: float) -> float:
        # Varying distance while driving
        return 200 + 300 * np.sin(2 * np.pi * t / 0.5)
    
    def gnss_snr(self, t: float) -> float:
        # Generally good signal with occasional drops
        base = 38
        if (t * 60) % 3 < 0.2:  # Occasional tunnel/overpass
            return 15
        return base + 3 * np.random.randn()
    
    def gnss_requested(self, t: float) -> bool:
        return True
    
    def bluetooth_state(self, t: float) -> Dict:
        return {
            'bt_classic_active': True,
            'audio_streaming': True,
            'audio_codec': 'sbc',
            'n_audio_devices': 1
        }


class GamingScenario(UsageScenario):
    """Heavy mobile gaming."""
    
    def screen_state(self, t: float) -> Dict:
        return {
            'brightness': 0.7,
            'refresh_rate': 120,
            'content_brightness': 0.6,
            'on': True
        }
    
    def cpu_load(self, t: float) -> float:
        # Fluctuating high load
        return 0.7 + 0.2 * np.sin(10 * t)
    
    def data_rate(self, t: float) -> float:
        return 3e6  # Online multiplayer
    
    def gnss_requested(self, t: float) -> bool:
        return False
    
    def bluetooth_state(self, t: float) -> Dict:
        # Gaming earbuds
        return {
            'bt_classic_active': True,
            'audio_streaming': True,
            'audio_codec': 'aptx',
            'n_audio_devices': 1,
            'ble_connections': [{'interval_ms': 15}]  # Low latency
        }


class MixedUsageScenario(UsageScenario):
    """
    Realistic mixed daily usage pattern.
    
    Schedule:
    - 0-0.5h: Wake up, check notifications
    - 0.5-1.5h: Commute with music + navigation
    - 1.5-4h: Work, screen mostly off
    - 4-4.5h: Lunch break browsing
    - 4.5-8h: Work, occasional checks
    - 8-9h: Commute back
    - 9-10h: Video streaming
    - 10-12h: Idle overnight
    """
    
    def screen_state(self, t: float) -> Dict:
        hour = t
        
        if hour < 0.5:  # Morning check
            return {'brightness': 0.5, 'on': True, 'refresh_rate': 60}
        elif hour < 1.5:  # Commute navigation
            return {'brightness': 0.8, 'on': True, 'refresh_rate': 60}
        elif hour < 4:  # Work
            if (hour * 60) % 20 < 1:  # Quick checks
                return {'brightness': 0.4, 'on': True}
            return {'brightness': 0, 'on': False}
        elif hour < 4.5:  # Lunch
            return {'brightness': 0.5, 'on': True, 'refresh_rate': 120}
        elif hour < 8:  # Afternoon work
            if (hour * 60) % 30 < 2:
                return {'brightness': 0.4, 'on': True}
            return {'brightness': 0, 'on': False}
        elif hour < 9:  # Evening commute
            return {'brightness': 0.7, 'on': True}
        elif hour < 10:  # Video streaming
            return {'brightness': 0.5, 'on': True, 'content_brightness': 0.4}
        else:  # Night
            return {'brightness': 0, 'on': False}
    
    def cpu_load(self, t: float) -> float:
        hour = t
        
        if hour < 0.5:
            return 0.2
        elif hour < 1.5:
            return 0.4  # Navigation
        elif hour < 4:
            return 0.05  # Idle
        elif hour < 4.5:
            return 0.3  # Browsing
        elif hour < 8:
            return 0.05
        elif hour < 9:
            return 0.3
        elif hour < 10:
            return 0.35  # Video decode
        else:
            return 0.02
    
    def data_rate(self, t: float) -> float:
        hour = t
        
        if hour < 0.5:
            return 5e6
        elif hour < 1.5:
            return 3e6  # Navigation + music
        elif hour < 4:
            return 0.1e6  # Background
        elif hour < 4.5:
            return 15e6  # Web browsing
        elif hour < 8:
            return 0.1e6
        elif hour < 9:
            return 3e6
        elif hour < 10:
            return 25e6  # Video streaming
        else:
            return 0
    
    def bs_distance(self, t: float) -> float:
        hour = t
        
        if 0.5 <= hour < 1.5 or 8 <= hour < 9:
            # Commuting - varying distance
            return 200 + 400 * np.abs(np.sin(2 * np.pi * t / 0.25))
        elif 1.5 <= hour < 8:
            return 100  # Office
        else:
            return 150  # Home
    
    def gnss_snr(self, t: float) -> float:
        hour = t
        
        if 0.5 <= hour < 1.5 or 8 <= hour < 9:
            # Commuting - variable signal
            return 35 + 10 * np.sin(2 * np.pi * t / 0.05)
        elif 1.5 <= hour < 8:
            return 20  # Indoor office
        else:
            return 25  # Indoor home
    
    def gnss_requested(self, t: float) -> bool:
        hour = t
        return 0.5 <= hour < 1.5 or 8 <= hour < 9  # Navigation during commute
    
    def bluetooth_state(self, t: float) -> Dict:
        hour = t
        
        if 0.5 <= hour < 1.5 or 8 <= hour < 9:
            # Commuting with music
            return {
                'bt_classic_active': True,
                'audio_streaming': True,
                'audio_codec': 'aac',
                'n_audio_devices': 2
            }
        elif 9 <= hour < 10:
            # Video with TWS
            return {
                'bt_classic_active': True,
                'audio_streaming': True,
                'audio_codec': 'aac',
                'n_audio_devices': 2
            }
        else:
            # Smartwatch always connected
            return {
                'bt_classic_active': False,
                'ble_connections': [
                    {'interval_ms': 500, 'slave_latency': 4}
                ]
            }


def run_scenario_comparison():
    """
    Compare battery life across different usage scenarios.
    """
    system = CoupledBatterySystem()
    
    scenarios = {
        'Idle': IdleScenario(),
        'Video Streaming': VideoStreamingScenario(),
        'Navigation': NavigationScenario(),
        'Gaming': GamingScenario(),
        'Mixed Daily Use': MixedUsageScenario()
    }
    
    results = {}
    
    for name, scenario in scenarios.items():
        print(f"Simulating: {name}...")
        result = system.simulate(
            scenario,
            S0=1.0,
            t_span=(0, 24),
            dt=0.005
        )
        results[name] = result
        print(f"  Battery life: {result['time'][-1]:.2f} hours")
    
    return results


if __name__ == "__main__":
    print("Coupled System Battery Simulation")
    print("=" * 50)
    
    # Run comparison
    results = run_scenario_comparison()
    
    print("\n" + "=" * 50)
    print("Summary:")
    for name, result in results.items():
        life = result['time'][-1]
        final_soc = result['SOC'][-1]
        avg_power = np.mean(result['power']) * 1000
        print(f"{name:20s}: {life:5.2f}h, Final SOC: {final_soc*100:4.1f}%, "
              f"Avg Power: {avg_power:.0f} mW")
