"""
High-Fidelity Coupled Electro-Thermal-Aging Model for Lithium-Ion Batteries
============================================================================

This module implements a physics-based battery model that captures:
1. Capacity fade due to aging (Double-Exponential Model)
2. Impedance growth over lifecycle (Power-Law Model)
3. Open Circuit Voltage (Nernst-equation based Combined Model)
4. 2nd-Order Thevenin equivalent circuit dynamics
5. Two-State Thermal Model (Core-Surface)
6. Temperature-dependent parameter evolution

Based on NASA PCoE Dataset #5 and Panasonic NCR18650B industrial specifications.

Author: Battery Modeling Framework
Date: 2026-02-01
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, List, Optional
import warnings


@dataclass
class BatteryParameters:
    """Container for all battery model parameters."""
    
    # Capacity fade model coefficients (NASA data fitted)
    a_Q: float = -0.1137  # First exponential amplitude
    b_Q: float = 0.0243   # First exponential decay rate
    c_Q: float = 1.9305   # Second exponential amplitude
    d_Q: float = 0.0007   # Second exponential decay rate
    
    # Temperature correction coefficients (Sigmoid model)
    S_Q: float = 1.0391   # Sigmoid amplitude
    k_Q: float = 0.0895   # Sigmoid slope
    T0_Q: float = -15.1281  # Sigmoid inflection temperature (°C)
    
    # Impedance growth model coefficients (Power-law)
    a_R: float = 0.0110   # Power-law coefficient
    b_R: float = 0.2106   # Power-law exponent
    c_R: float = 0.0181   # Baseline resistance
    
    # Arrhenius temperature correction for resistance
    C_R: float = 0.7136   # Baseline coefficient
    A_R: float = 1.3533   # Exponential amplitude
    B_R: float = 0.0630   # Exponential decay rate
    
    # OCV model coefficients (Combined Nernst-based model)
    K0: float = 3.4704    # Constant term
    K1: float = 0.1670    # Linear SOC term
    K2: float = -0.0042   # Inverse SOC term
    K3: float = 0.0573    # ln(SOC) term
    K4: float = -0.0847   # ln(1-SOC) term
    
    # 2nd-Order RC circuit parameters (base values at 25°C)
    R0_base: float = 0.025    # Ohmic resistance (Ω)
    R1_base: float = 0.015    # Electrochemical polarization resistance (Ω)
    C1_base: float = 1000.0   # Electrochemical polarization capacitance (F)
    R2_base: float = 0.020    # Concentration polarization resistance (Ω)
    C2_base: float = 5000.0   # Concentration polarization capacitance (F)
    
    # Resistance distribution ratios
    lambda_R0: float = 0.35   # Ohmic ratio
    lambda_R1: float = 0.35   # Electrochemical polarization ratio
    lambda_R2: float = 0.30   # Concentration polarization ratio
    
    # Thermal model parameters
    Cc: float = 62.7      # Core heat capacity (J/K)
    Cs: float = 4.5       # Surface heat capacity (J/K)
    Rc_s: float = 1.94    # Core-to-surface thermal resistance (K/W)
    Rs_e: float = 3.08    # Surface-to-environment thermal resistance (K/W)
    
    # Entropy coefficient for reversible heat (dOCV/dT)
    dOCV_dT: float = -0.0002  # V/K (entropy coefficient)
    
    # Nominal capacity at fresh state
    Q_nominal: float = 2.0    # Nominal capacity (Ah)
    
    # Coulombic efficiency (temperature-dependent)
    eta_base: float = 0.998   # Base coulombic efficiency


@dataclass
class BatteryState:
    """Container for battery dynamic state variables."""
    
    SOC: float = 1.0          # State of Charge [0, 1]
    V1: float = 0.0           # RC1 voltage (V)
    V2: float = 0.0           # RC2 voltage (V)
    Tc: float = 25.0          # Core temperature (°C)
    Ts: float = 25.0          # Surface temperature (°C)
    V_term: float = 4.2       # Terminal voltage (V)
    Q_max: float = 2.0        # Current maximum capacity (Ah)
    R_total: float = 0.060    # Current total resistance (Ω)


class BatteryElectroThermalAgingModel:
    """
    High-fidelity coupled electro-thermal-aging battery model.
    
    This model integrates:
    - Electrochemical dynamics (2nd-order Thevenin circuit)
    - Thermal dynamics (Two-state lumped model)
    - Aging effects (Capacity fade + Impedance growth)
    """
    
    def __init__(self, params: Optional[BatteryParameters] = None):
        """Initialize the battery model with given parameters."""
        self.params = params if params else BatteryParameters()
        self.state = BatteryState()
        self._history = []
        
    def reset(self, SOC: float = 1.0, T_init: float = 25.0, N: int = 0):
        """Reset battery state to initial conditions."""
        self.state.SOC = SOC
        self.state.V1 = 0.0
        self.state.V2 = 0.0
        self.state.Tc = T_init
        self.state.Ts = T_init
        self._history = []
        
        # Update capacity and resistance for cycle count N
        self.state.Q_max = self.get_capacity(N, T_init)
        self.state.R_total = self.get_resistance(N, T_init)
        self.state.V_term = self.get_OCV(SOC)
        
    # ==========================================================================
    # Multi-Physics Parameter Evolution Models
    # ==========================================================================
    
    def get_capacity(self, N: int, Tc: float) -> float:
        """
        Calculate maximum available capacity considering aging and temperature.
        
        Capacity Fade Model (Double-Exponential):
        Q_max(N, Tc) = [a_Q * exp(-b_Q*N) + c_Q * exp(-d_Q*N)] * S_Q(Tc)
        
        Where S_Q(Tc) is the Sigmoid temperature correction factor.
        
        Args:
            N: Cycle number (aging state)
            Tc: Core temperature (°C)
            
        Returns:
            Maximum available capacity (Ah)
        """
        p = self.params
        
        # Aging-dependent capacity (double exponential)
        Q_aging = p.a_Q * np.exp(-p.b_Q * N) + p.c_Q * np.exp(-p.d_Q * N)
        
        # Temperature correction (Sigmoid function)
        S_Q = p.S_Q / (1.0 + np.exp(-p.k_Q * (Tc - p.T0_Q)))
        
        return max(Q_aging * S_Q, 0.1)  # Minimum 0.1 Ah
    
    def get_resistance(self, N: int, Tc: float) -> float:
        """
        Calculate total DC resistance considering aging and temperature.
        
        Impedance Growth Model (Power-Law + Arrhenius):
        R_total(N, Tc) = [a_R * N^b_R + c_R] * S_R(Tc)
        
        Where S_R(Tc) = C_R + A_R * exp(-B_R * Tc)
        
        Args:
            N: Cycle number (aging state)
            Tc: Core temperature (°C)
            
        Returns:
            Total DC resistance (Ω)
        """
        p = self.params
        
        # Aging-dependent resistance (power law)
        R_aging = p.a_R * np.power(max(N, 1), p.b_R) + p.c_R
        
        # Temperature correction (Arrhenius)
        S_R = p.C_R + p.A_R * np.exp(-p.B_R * Tc)
        
        return R_aging * S_R
    
    def get_OCV(self, SOC: float) -> float:
        """
        Calculate Open Circuit Voltage using Nernst-based Combined Model.
        
        V_OCV(z) = K0 + K1*z + K2/z + K3*ln(z) + K4*ln(1-z)
        
        Args:
            SOC: State of Charge [0, 1]
            
        Returns:
            Open Circuit Voltage (V)
        """
        p = self.params
        
        # Clamp SOC to avoid singularities
        z = np.clip(SOC, 0.001, 0.999)
        
        V_OCV = (p.K0 + 
                 p.K1 * z + 
                 p.K2 / z + 
                 p.K3 * np.log(z) + 
                 p.K4 * np.log(1.0 - z))
        
        return V_OCV
    
    def get_dOCV_dSOC(self, SOC: float) -> float:
        """
        Calculate derivative of OCV with respect to SOC.
        
        dV_OCV/dz = K1 - K2/z^2 + K3/z + K4/(z-1)
        """
        p = self.params
        z = np.clip(SOC, 0.001, 0.999)
        
        dV_dz = p.K1 - p.K2 / (z**2) + p.K3 / z + p.K4 / (z - 1.0)
        return dV_dz
    
    def get_coulombic_efficiency(self, Tc: float) -> float:
        """
        Calculate temperature-dependent Coulombic efficiency.
        
        Low temperature reduces charge transfer efficiency.
        """
        p = self.params
        # Efficiency decreases at low temperatures
        eta = p.eta_base * (1.0 - 0.002 * max(0, 25 - Tc))
        return np.clip(eta, 0.85, 1.0)
    
    # ==========================================================================
    # 2nd-Order Thevenin Equivalent Circuit Model
    # ==========================================================================
    
    def get_distributed_resistances(self, R_total: float, Tc: float) -> Tuple[float, float, float]:
        """
        Distribute total resistance among R0, R1, R2 based on temperature.
        
        At low temperatures, electrochemical polarization dominates.
        At high temperatures, ohmic resistance dominates.
        """
        p = self.params
        
        # Temperature-dependent distribution (shift towards R1/R2 at low T)
        T_ref = 25.0
        delta_T = Tc - T_ref
        
        # Adjust ratios
        shift = 0.05 * np.tanh(-delta_T / 20.0)
        
        lambda_0 = p.lambda_R0 - shift
        lambda_1 = p.lambda_R1 + shift * 0.6
        lambda_2 = p.lambda_R2 + shift * 0.4
        
        # Normalize
        total_lambda = lambda_0 + lambda_1 + lambda_2
        
        R0 = R_total * (lambda_0 / total_lambda)
        R1 = R_total * (lambda_1 / total_lambda)
        R2 = R_total * (lambda_2 / total_lambda)
        
        return R0, R1, R2
    
    def get_RC_time_constants(self, Tc: float) -> Tuple[float, float]:
        """
        Get temperature-adjusted RC time constants.
        
        Time constants increase at low temperatures due to slower diffusion.
        """
        p = self.params
        
        # Temperature factor (slower dynamics at low T)
        T_factor = np.exp(0.02 * (25.0 - Tc))
        
        tau1 = p.R1_base * p.C1_base * T_factor
        tau2 = p.R2_base * p.C2_base * T_factor
        
        return tau1, tau2
    
    # ==========================================================================
    # Two-State Thermal Model
    # ==========================================================================
    
    def calculate_heat_generation(self, I: float, R0: float, R1: float, R2: float, 
                                   Tc: float) -> float:
        """
        Calculate heat generation using Bernardi equation.
        
        Q_gen = I^2 * (R0 + R1 + R2) + I * Tc * (dOCV/dT)
        
        First term: Irreversible Joule heat
        Second term: Reversible entropic heat
        
        Args:
            I: Load current (A)
            R0, R1, R2: Resistance components (Ω)
            Tc: Core temperature (°C)
            
        Returns:
            Total heat generation rate (W)
        """
        p = self.params
        
        # Joule heat (irreversible)
        Q_joule = I**2 * (R0 + R1 + R2)
        
        # Entropic heat (reversible) - dOCV/dT is typically negative
        Tc_kelvin = Tc + 273.15
        Q_entropy = abs(I) * Tc_kelvin * p.dOCV_dT
        
        return Q_joule + Q_entropy
    
    # ==========================================================================
    # Numerical Solver (Forward Euler Discretization)
    # ==========================================================================
    
    def step(self, I: float, T_env: float, N: int, dt: float = 1.0) -> dict:
        """
        Advance battery state by one time step using Forward Euler method.
        
        State-space equations:
        - dV1/dt = -V1/(R1*C1) + I/C1
        - dV2/dt = -V2/(R2*C2) + I/C2
        - dTc/dt = (Q_gen - (Tc-Ts)/Rc_s) / Cc
        - dTs/dt = ((Tc-Ts)/Rc_s - (Ts-Tenv)/Rs_e) / Cs
        - dSOC/dt = -I * eta / (Q_max * 3600)
        
        Args:
            I: Load current (A), positive for discharge
            T_env: Environment temperature (°C)
            N: Cycle number (aging state)
            dt: Time step (seconds)
            
        Returns:
            Dictionary with current state and derived quantities
        """
        p = self.params
        s = self.state
        
        # Update aging-dependent parameters
        s.Q_max = self.get_capacity(N, s.Tc)
        s.R_total = self.get_resistance(N, s.Tc)
        
        # Distribute resistance
        R0, R1, R2 = self.get_distributed_resistances(s.R_total, s.Tc)
        
        # Get RC time constants
        tau1, tau2 = self.get_RC_time_constants(s.Tc)
        C1 = tau1 / R1 if R1 > 0 else p.C1_base
        C2 = tau2 / R2 if R2 > 0 else p.C2_base
        
        # Calculate heat generation
        Q_gen = self.calculate_heat_generation(I, R0, R1, R2, s.Tc)
        
        # ==== Forward Euler Integration ====
        
        # RC circuit dynamics (Eq. for V1 and V2)
        dV1_dt = -s.V1 / (R1 * C1) + I / C1 if R1 > 0 and C1 > 0 else 0
        dV2_dt = -s.V2 / (R2 * C2) + I / C2 if R2 > 0 and C2 > 0 else 0
        
        s.V1 = s.V1 + dV1_dt * dt
        s.V2 = s.V2 + dV2_dt * dt
        
        # Thermal dynamics (Two-state model)
        dTc_dt = (Q_gen - (s.Tc - s.Ts) / p.Rc_s) / p.Cc
        dTs_dt = ((s.Tc - s.Ts) / p.Rc_s - (s.Ts - T_env) / p.Rs_e) / p.Cs
        
        s.Tc = s.Tc + dTc_dt * dt
        s.Ts = s.Ts + dTs_dt * dt
        
        # SOC update (Coulomb counting with efficiency)
        eta = self.get_coulombic_efficiency(s.Tc)
        dSOC_dt = -I * eta / (s.Q_max * 3600.0)
        s.SOC = np.clip(s.SOC + dSOC_dt * dt, 0.0, 1.0)
        
        # Terminal voltage calculation
        V_OCV = self.get_OCV(s.SOC)
        s.V_term = V_OCV - s.V1 - s.V2 - I * R0
        
        # Store history
        result = {
            'time': len(self._history) * dt,
            'SOC': s.SOC,
            'V_term': s.V_term,
            'V_OCV': V_OCV,
            'V1': s.V1,
            'V2': s.V2,
            'Tc': s.Tc,
            'Ts': s.Ts,
            'I': I,
            'R_total': s.R_total,
            'R0': R0,
            'R1': R1,
            'R2': R2,
            'Q_max': s.Q_max,
            'Q_gen': Q_gen,
            'eta': eta
        }
        
        self._history.append(result)
        return result
    
    def simulate(self, I_profile: np.ndarray, T_env: float, N: int, 
                 dt: float = 1.0, SOC_init: float = 1.0, T_init: float = 25.0) -> List[dict]:
        """
        Run complete simulation with given current profile.
        
        Args:
            I_profile: Array of load currents (A) at each time step
            T_env: Environment temperature (°C)
            N: Cycle number (aging state)
            dt: Time step (seconds)
            SOC_init: Initial SOC
            T_init: Initial temperature (°C)
            
        Returns:
            List of state dictionaries for each time step
        """
        self.reset(SOC_init, T_init, N)
        
        results = []
        for I in I_profile:
            result = self.step(I, T_env, N, dt)
            results.append(result)
            
            # Stop if SOC depleted or voltage too low
            if self.state.SOC <= 0.001 or self.state.V_term < 2.5:
                break
                
        return results
    
    def get_history(self) -> List[dict]:
        """Return simulation history."""
        return self._history


# ==========================================================================
# Extended Theoretical Analysis Functions
# ==========================================================================

def analyze_capacity_degradation(N_range: np.ndarray, T_range: np.ndarray, 
                                   params: BatteryParameters) -> np.ndarray:
    """
    Generate capacity degradation surface over aging and temperature.
    
    Returns:
        2D array of capacity values [N x T]
    """
    model = BatteryElectroThermalAgingModel(params)
    
    Q_surface = np.zeros((len(N_range), len(T_range)))
    for i, N in enumerate(N_range):
        for j, T in enumerate(T_range):
            Q_surface[i, j] = model.get_capacity(int(N), T)
    
    return Q_surface


def analyze_resistance_evolution(N_range: np.ndarray, T_range: np.ndarray,
                                   params: BatteryParameters) -> np.ndarray:
    """
    Generate resistance evolution surface over aging and temperature.
    
    Returns:
        2D array of resistance values [N x T]
    """
    model = BatteryElectroThermalAgingModel(params)
    
    R_surface = np.zeros((len(N_range), len(T_range)))
    for i, N in enumerate(N_range):
        for j, T in enumerate(T_range):
            R_surface[i, j] = model.get_resistance(int(N), T)
    
    return R_surface


def calculate_remaining_runtime(model: BatteryElectroThermalAgingModel,
                                  I_load: float, T_env: float, N: int,
                                  SOC_init: float = 1.0) -> Tuple[float, List[dict]]:
    """
    Calculate estimated remaining runtime until cutoff voltage.
    
    Returns:
        (runtime in hours, simulation results)
    """
    dt = 1.0  # 1 second resolution
    max_steps = int(24 * 3600)  # Max 24 hours
    
    I_profile = np.full(max_steps, I_load)
    results = model.simulate(I_profile, T_env, N, dt, SOC_init)
    
    runtime_hours = len(results) * dt / 3600.0
    return runtime_hours, results


if __name__ == "__main__":
    # Basic test
    model = BatteryElectroThermalAgingModel()
    
    # Test OCV curve
    soc_range = np.linspace(0.01, 0.99, 100)
    ocv_values = [model.get_OCV(s) for s in soc_range]
    
    print("Battery Model Test")
    print(f"OCV at SOC=1.0: {model.get_OCV(0.99):.3f} V")
    print(f"OCV at SOC=0.5: {model.get_OCV(0.50):.3f} V")
    print(f"OCV at SOC=0.1: {model.get_OCV(0.10):.3f} V")
    
    # Test capacity at different conditions
    print(f"\nCapacity at N=0, T=25°C: {model.get_capacity(0, 25):.3f} Ah")
    print(f"Capacity at N=300, T=25°C: {model.get_capacity(300, 25):.3f} Ah")
    print(f"Capacity at N=0, T=-10°C: {model.get_capacity(0, -10):.3f} Ah")
    
    # Test resistance
    print(f"\nResistance at N=0, T=25°C: {model.get_resistance(0, 25):.4f} Ω")
    print(f"Resistance at N=300, T=25°C: {model.get_resistance(300, 25):.4f} Ω")
    print(f"Resistance at N=0, T=-10°C: {model.get_resistance(0, -10):.4f} Ω")
