"""
Smartphone Battery Discharge Core Model
========================================
2026 MCM Problem A: Continuous-Time Battery SOC Model

This module implements the core battery dynamics using continuous-time 
differential equations based on electrochemical principles.

References:
- Peukert's Law for capacity-rate relationship
- Arrhenius equation for temperature effects
- Equivalent circuit model (ECM) for lithium-ion batteries
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass
from typing import Callable, Optional, Tuple
import warnings

@dataclass
class BatteryParameters:
    """
    Lithium-ion battery parameters based on typical smartphone specifications.
    
    Reference values based on:
    - Samsung Galaxy S23 Ultra: 5000 mAh, 3.87V nominal
    - iPhone 15 Pro Max: 4422 mAh, 3.82V nominal
    - Typical Li-ion characteristics from IEEE papers
    """
    # Nominal capacity (Ah)
    C_nom: float = 4.5  # 4500 mAh typical flagship
    
    # Nominal voltage (V)
    V_nom: float = 3.85
    
    # Internal resistance (Ohm) - varies with SOC and temperature
    R_internal: float = 0.08
    
    # Peukert coefficient (dimensionless, typically 1.1-1.3 for Li-ion)
    k_peukert: float = 1.05
    
    # Reference discharge rate for Peukert (A)
    I_ref: float = 0.9  # C/5 rate
    
    # Temperature coefficients
    T_ref: float = 298.15  # Reference temperature (K) = 25°C
    E_a: float = 20000  # Activation energy (J/mol)
    R_gas: float = 8.314  # Gas constant (J/mol·K)
    
    # Coulombic efficiency
    eta_coulomb: float = 0.995
    
    # Self-discharge rate (/hour)
    k_self_discharge: float = 0.0001
    
    # Aging factor (0-1, 1 = new battery)
    SOH: float = 1.0  # State of Health


class BatteryCore:
    """
    Core battery model implementing continuous-time SOC dynamics.
    
    The fundamental equation:
        dS(t)/dt = -I_load(t) / (C_eff(I,T) * η_coulomb) - k_sd * S(t)
    
    Where:
        S(t): State of Charge (0-1)
        I_load(t): Total load current (A)
        C_eff: Effective capacity considering Peukert effect and temperature
        η_coulomb: Coulombic efficiency
        k_sd: Self-discharge rate
    """
    
    def __init__(self, params: Optional[BatteryParameters] = None):
        self.params = params or BatteryParameters()
        
    def effective_capacity(self, I: float, T: float) -> float:
        """
        Calculate effective capacity considering:
        1. Peukert effect (capacity reduction at high discharge rates)
        2. Temperature effect (Arrhenius relationship)
        3. Battery aging (SOH factor)
        
        Args:
            I: Discharge current (A)
            T: Temperature (K)
            
        Returns:
            Effective capacity (Ah)
        """
        p = self.params
        
        # Peukert capacity correction
        # C_peukert = C_nom * (I_ref / I)^(k-1) for I > I_ref
        if I > 0:
            peukert_factor = (p.I_ref / max(I, 0.001)) ** (p.k_peukert - 1)
            peukert_factor = min(peukert_factor, 1.2)  # Cap at 120%
        else:
            peukert_factor = 1.0
        
        # Temperature correction using Arrhenius equation
        # Capacity decreases at low temperatures, increases slightly at high temps
        # Bound temperature to prevent numerical issues
        T_bounded = np.clip(T, 253.15, 333.15)  # -20°C to 60°C
        
        # Calculate temperature factor with bounded exponent
        exp_arg = -(p.E_a / p.R_gas) * (1/T_bounded - 1/p.T_ref)
        exp_arg = np.clip(exp_arg, -5, 5)  # Prevent overflow
        temp_factor = np.exp(exp_arg)
        
        # Clip temperature factor to realistic range
        temp_factor = np.clip(temp_factor, 0.5, 1.1)
        
        # Apply SOH aging factor
        C_eff = p.C_nom * p.SOH * peukert_factor * temp_factor
        
        return C_eff
    
    def internal_resistance(self, S: float, T: float) -> float:
        """
        Calculate internal resistance as function of SOC and temperature.
        
        R_int increases at low SOC and low temperature.
        
        Model: R(S,T) = R_0 * (1 + α*(1-S)²) * exp(β*(T_ref-T)/T_ref)
        """
        p = self.params
        
        # Bound temperature to reasonable range
        T = np.clip(T, 253.15, 333.15)  # -20°C to 60°C
        
        # SOC dependence: resistance increases at low SOC
        alpha = 0.5
        S_clipped = np.clip(S, 0.01, 0.99)
        soc_factor = 1 + alpha * (1 - S_clipped) ** 2
        
        # Temperature dependence: resistance increases at low temp
        beta = 0.8
        temp_ratio = np.clip((p.T_ref - T) / p.T_ref, -0.5, 0.5)
        temp_factor = np.exp(beta * temp_ratio)
        
        return p.R_internal * soc_factor * temp_factor
    
    def open_circuit_voltage(self, S: float) -> float:
        """
        Open Circuit Voltage (OCV) as function of SOC.
        
        Polynomial fit based on typical Li-ion OCV-SOC curve.
        Data reference: Chen & Rincon-Mora, IEEE Trans. Energy Conversion, 2006
        Modified for typical smartphone Li-ion (3.0V - 4.2V range)
        """
        S_clipped = np.clip(S, 0.01, 0.99)
        
        # More realistic polynomial for Li-ion OCV
        # V_oc ranges from ~3.0V (empty) to ~4.2V (full)
        # Using improved coefficients for typical Li-ion cell
        V_oc = (3.0 + 
                1.2 * S_clipped - 
                0.3 * S_clipped**2 + 
                0.15 * S_clipped**3 + 
                0.15 * S_clipped**4)
        
        return V_oc
    
    def terminal_voltage(self, S: float, I: float, T: float) -> float:
        """
        Terminal voltage considering internal resistance drop.
        
        V_term = V_oc(S) - I * R_int(S, T)
        """
        V_oc = self.open_circuit_voltage(S)
        R_int = self.internal_resistance(S, T)
        
        return V_oc - I * R_int
    
    def soc_derivative(self, t: float, S: float, I_load_func: Callable,
                       T_func: Optional[Callable] = None) -> float:
        """
        Compute dS/dt for the ODE solver.
        
        dS/dt = -I_load(t) / (C_eff * η) - k_sd * S
        
        Args:
            t: Time (hours)
            S: Current SOC (0-1)
            I_load_func: Function returning total load current at time t
            T_func: Function returning temperature at time t (optional)
            
        Returns:
            Rate of change of SOC (per hour)
        """
        p = self.params
        
        # Get current load
        I_load = I_load_func(t)
        
        # Get temperature (default to 25°C if not provided)
        T = T_func(t) if T_func else p.T_ref
        
        # Effective capacity
        C_eff = self.effective_capacity(I_load, T)
        
        # SOC derivative
        dSdt = -I_load / (C_eff * p.eta_coulomb) - p.k_self_discharge * S
        
        return dSdt
    
    def simulate(self, S0: float, t_span: Tuple[float, float],
                 I_load_func: Callable, T_func: Optional[Callable] = None,
                 t_eval: Optional[np.ndarray] = None,
                 cutoff_voltage: float = 3.0) -> dict:
        """
        Simulate battery discharge over time.
        
        Args:
            S0: Initial SOC (0-1)
            t_span: (t_start, t_end) in hours
            I_load_func: Load current function I(t)
            T_func: Temperature function T(t) in Kelvin
            t_eval: Time points for output
            cutoff_voltage: Minimum voltage before shutdown (V)
            
        Returns:
            Dictionary with simulation results
        """
        
        def ode_func(t, y):
            S = y[0]
            if S <= 0.01:  # Prevent negative SOC
                return [0]
            return [self.soc_derivative(t, S, I_load_func, T_func)]
        
        def event_empty(t, y):
            return y[0] - 0.01  # Stop at 1% SOC
        event_empty.terminal = True
        event_empty.direction = -1
        
        # Solve ODE
        solution = solve_ivp(
            ode_func,
            t_span,
            [S0],
            t_eval=t_eval,
            events=event_empty,
            method='RK45',
            max_step=0.01  # 36 second max step for accuracy
        )
        
        # Extract results
        t = solution.t
        S = solution.y[0]
        
        # Calculate additional quantities
        T = T_func(t) if T_func else np.full_like(t, self.params.T_ref)
        I = np.array([I_load_func(ti) for ti in t])
        V = np.array([self.terminal_voltage(Si, Ii, Ti) 
                      for Si, Ii, Ti in zip(S, I, T)])
        P = I * V  # Power consumption (W)
        
        return {
            'time': t,
            'SOC': S,
            'current': I,
            'voltage': V,
            'power': P,
            'temperature': T,
            'success': solution.success,
            'message': solution.message
        }
    
    def estimate_remaining_time(self, S: float, I_avg: float, T: float = 298.15) -> float:
        """
        Estimate remaining discharge time.
        
        t_remain ≈ S * C_eff / I_avg
        
        Args:
            S: Current SOC
            I_avg: Expected average current (A)
            T: Temperature (K)
            
        Returns:
            Estimated time remaining (hours)
        """
        C_eff = self.effective_capacity(I_avg, T)
        
        if I_avg > 0:
            t_remain = S * C_eff * self.params.eta_coulomb / I_avg
        else:
            t_remain = float('inf')
        
        return t_remain


class ThermalModel:
    """
    Thermal dynamics model for battery temperature evolution.
    
    Heat generation sources:
    1. Joule heating (I²R)
    2. Entropic heating (reversible)
    3. Heat from power electronics
    
    dT/dt = (Q_gen - Q_dissipation) / (m * c_p)
    """
    
    def __init__(self, battery: BatteryCore):
        self.battery = battery
        
        # Thermal parameters
        self.m = 0.045  # Battery mass (kg) - typical smartphone battery
        self.c_p = 1000  # Specific heat (J/kg·K)
        self.h_conv = 5.0  # Convection coefficient (W/m²·K)
        self.A_surface = 0.005  # Surface area (m²)
        self.T_ambient = 298.15  # Ambient temperature (K)
        
    def heat_generation(self, S: float, I: float, T: float) -> float:
        """
        Calculate heat generation rate (W).
        
        Q_gen = I² * R_int + |I * T * dV_oc/dT|
        """
        R_int = self.battery.internal_resistance(S, T)
        
        # Joule heating
        Q_joule = I ** 2 * R_int
        
        # Entropic heating (approximately 0.1 mW/K for Li-ion at moderate rates)
        Q_entropic = abs(I) * 0.0001 * T
        
        return Q_joule + Q_entropic
    
    def temperature_derivative(self, S: float, I: float, T: float,
                               P_electronics: float = 0) -> float:
        """
        Compute dT/dt for thermal dynamics.
        
        Args:
            S: SOC
            I: Current (A)
            T: Temperature (K)
            P_electronics: Additional heat from electronics (W)
        """
        # Bound inputs
        T = np.clip(T, 253.15, 333.15)
        P_electronics = max(P_electronics, 0)
        
        Q_gen = self.heat_generation(S, I, T) + P_electronics
        Q_dissipation = self.h_conv * self.A_surface * (T - self.T_ambient)
        
        dTdt = (Q_gen - Q_dissipation) / (self.m * self.c_p)
        
        # Limit rate of temperature change
        dTdt = np.clip(dTdt * 3600, -50, 50)  # Max 50 K/hour change
        
        return dTdt


if __name__ == "__main__":
    # Test basic battery model
    battery = BatteryCore()
    
    # Constant 1A load
    def constant_load(t):
        return 1.0
    
    # Simulate 5 hours
    results = battery.simulate(
        S0=1.0,
        t_span=(0, 10),
        I_load_func=constant_load,
        t_eval=np.linspace(0, 10, 1000)
    )
    
    print(f"Simulation completed: {results['success']}")
    print(f"Final SOC: {results['SOC'][-1]:.2%}")
    print(f"Battery life: {results['time'][-1]:.2f} hours")
