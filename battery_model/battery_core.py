"""
Smartphone Battery Core Model
==============================
Coupled Electro-Thermal-Aging Model for Lithium-Ion Battery

This module implements the continuous-time state equations for:
1. Open Circuit Voltage (OCV) - Nernst-based combined model
2. 2nd-Order Thevenin Equivalent Circuit Model
3. Aging-dependent parameter evolution
4. Temperature-dependent corrections
5. Two-state thermal dynamics

Mathematical Foundation:
------------------------
The battery state is described by a system of coupled ODEs:

State Variables:
    z(t)     - State of Charge (SOC) ∈ [0, 1]
    V_1(t)   - Electrochemical polarization voltage [V]
    V_2(t)   - Concentration polarization voltage [V]
    T_c(t)   - Core temperature [°C]
    T_s(t)   - Surface temperature [°C]

Governing Equations:
    dz/dt = -I(t) * η(T_c) / (Q_max(N, T_c) * 3600)
    dV_1/dt = -V_1/(R_1*C_1) + I(t)/C_1
    dV_2/dt = -V_2/(R_2*C_2) + I(t)/C_2
    dT_c/dt = (Q_gen - (T_c - T_s)/R_cs) / C_c
    dT_s/dt = ((T_c - T_s)/R_cs - (T_s - T_env)/R_se) / C_s

Output Equation:
    V_term(t) = V_OCV(z) - V_1(t) - V_2(t) - I(t)*R_0
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
from battery_params import (
    BATTERY_PARAMS, AGING_CAPACITY_PARAMS, AGING_RESISTANCE_PARAMS,
    TEMP_CAPACITY_PARAMS, TEMP_RESISTANCE_PARAMS, OCV_PARAMS,
    RC_CIRCUIT_PARAMS, THERMAL_PARAMS, coulombic_efficiency
)


class BatteryAgingModel:
    """
    Battery aging model implementing capacity fade and impedance growth.
    
    Capacity Fade (Double-Exponential):
        Q_max(N) = a_Q * exp(-b_Q * N) + c_Q * exp(-d_Q * N)
    
    Impedance Growth (Power-Law):
        R_total(N) = a_R * N^b_R + c_R
    """
    
    def __init__(self, params_cap=None, params_res=None):
        self.params_cap = params_cap or AGING_CAPACITY_PARAMS
        self.params_res = params_res or AGING_RESISTANCE_PARAMS
    
    def capacity_fade(self, N):
        """
        Calculate maximum capacity as function of cycle number.
        
        Parameters:
            N: Cycle number [-]
        
        Returns:
            Q_max: Maximum capacity [Ah]
        """
        p = self.params_cap
        Q_max = p['a_Q'] * np.exp(-p['b_Q'] * N) + p['c_Q'] * np.exp(-p['d_Q'] * N)
        return np.maximum(Q_max, 0.1)  # Prevent negative capacity
    
    def impedance_growth(self, N):
        """
        Calculate total DC resistance as function of cycle number.
        
        Parameters:
            N: Cycle number [-]
        
        Returns:
            R_total: Total DC resistance [Ω]
        """
        p = self.params_res
        R_total = p['a_R'] * np.power(np.maximum(N, 1), p['b_R']) + p['c_R']
        return R_total


class TemperatureCorrectionModel:
    """
    Temperature-dependent correction factors for capacity and resistance.
    
    Capacity Correction (Sigmoid):
        S_Q(T) = S_Q_max / (1 + exp(-k_Q * (T - T_0)))
    
    Resistance Correction (Arrhenius):
        S_R(T) = C_R + A_R * exp(-B_R * T)
    """
    
    def __init__(self, params_cap=None, params_res=None):
        self.params_cap = params_cap or TEMP_CAPACITY_PARAMS
        self.params_res = params_res or TEMP_RESISTANCE_PARAMS
    
    def capacity_factor(self, T):
        """
        Calculate capacity correction factor for temperature.
        
        Parameters:
            T: Temperature [°C]
        
        Returns:
            S_Q: Capacity factor [-]
        """
        p = self.params_cap
        S_Q = p['S_Q_max'] / (1 + np.exp(-p['k_Q'] * (T - p['T_0'])))
        return S_Q
    
    def resistance_factor(self, T):
        """
        Calculate resistance correction factor for temperature.
        
        Parameters:
            T: Temperature [°C]
        
        Returns:
            S_R: Resistance factor [-]
        """
        p = self.params_res
        S_R = p['C_R'] + p['A_R'] * np.exp(-p['B_R'] * T)
        return S_R


class OCVModel:
    """
    Open Circuit Voltage model based on Nernst equation.
    
    V_OCV(z) = K_0 + K_1*z + K_2/z + K_3*ln(z) + K_4*ln(1-z)
    
    This model captures:
    - Linear SOC dependence (K_1)
    - Low SOC steep drop (K_2/z)
    - Logarithmic entropy effects (K_3, K_4)
    """
    
    def __init__(self, params=None):
        self.params = params or OCV_PARAMS
    
    def voltage(self, z):
        """
        Calculate open circuit voltage from SOC.
        
        Parameters:
            z: State of Charge ∈ (0, 1)
        
        Returns:
            V_OCV: Open circuit voltage [V]
        """
        # Clamp SOC to avoid numerical issues
        z = np.clip(z, 1e-6, 1 - 1e-6)
        
        p = self.params
        V_OCV = (p['K_0'] + p['K_1'] * z + p['K_2'] / z + 
                 p['K_3'] * np.log(z) + p['K_4'] * np.log(1 - z))
        return V_OCV
    
    def dVdz(self, z):
        """
        Calculate derivative of OCV with respect to SOC.
        
        Parameters:
            z: State of Charge ∈ (0, 1)
        
        Returns:
            dV/dz: OCV derivative [V]
        """
        z = np.clip(z, 1e-6, 1 - 1e-6)
        
        p = self.params
        dV = p['K_1'] - p['K_2'] / (z**2) + p['K_3'] / z - p['K_4'] / (1 - z)
        return dV


class ThermalModel:
    """
    Two-state lumped thermal model for battery.
    
    Core temperature dynamics:
        C_c * dT_c/dt = Q_gen - (T_c - T_s)/R_cs
    
    Surface temperature dynamics:
        C_s * dT_s/dt = (T_c - T_s)/R_cs - (T_s - T_env)/R_se
    
    Heat generation (Bernardi equation):
        Q_gen = I² * R_total + I * T_c * (dV_OCV/dT)
    """
    
    def __init__(self, params=None):
        self.params = params or THERMAL_PARAMS
    
    def heat_generation(self, I, R_total, T_c, dV_dT=None):
        """
        Calculate heat generation rate using Bernardi equation.
        
        Parameters:
            I: Load current [A]
            R_total: Total internal resistance [Ω]
            T_c: Core temperature [°C]
            dV_dT: Entropy coefficient [V/K]
        
        Returns:
            Q_gen: Heat generation rate [W]
        """
        dV_dT = dV_dT or self.params['dV_dT']
        
        # Joule heat (irreversible)
        Q_joule = I**2 * R_total
        
        # Entropic heat (reversible)
        T_kelvin = T_c + 273.15
        Q_entropy = I * T_kelvin * dV_dT
        
        Q_gen = Q_joule + np.abs(Q_entropy)
        return Q_gen
    
    def temperature_dynamics(self, T_c, T_s, T_env, Q_gen):
        """
        Calculate temperature derivatives.
        
        Parameters:
            T_c: Core temperature [°C]
            T_s: Surface temperature [°C]
            T_env: Environment temperature [°C]
            Q_gen: Heat generation rate [W]
        
        Returns:
            dT_c_dt, dT_s_dt: Temperature derivatives [°C/s]
        """
        p = self.params
        
        # Core temperature dynamics
        dT_c_dt = (Q_gen - (T_c - T_s) / p['R_cs']) / p['C_c']
        
        # Surface temperature dynamics
        dT_s_dt = ((T_c - T_s) / p['R_cs'] - (T_s - T_env) / p['R_se']) / p['C_s']
        
        return dT_c_dt, dT_s_dt


class BatteryCoreModel:
    """
    Complete coupled electro-thermal-aging battery model.
    
    This class integrates all submodels into a unified continuous-time
    state-space representation suitable for numerical simulation.
    
    State Vector:
        x = [z, V_1, V_2, T_c, T_s]
        
    where:
        z    - State of Charge [-]
        V_1  - Electrochemical polarization voltage [V]
        V_2  - Concentration polarization voltage [V]
        T_c  - Core temperature [°C]
        T_s  - Surface temperature [°C]
    """
    
    def __init__(self, cycle_number=0, battery_params=None, rc_params=None):
        """
        Initialize battery model.
        
        Parameters:
            cycle_number: Current battery cycle count [-]
            battery_params: Battery nominal parameters
            rc_params: RC circuit parameters
        """
        self.cycle_number = cycle_number
        self.battery_params = battery_params or BATTERY_PARAMS
        self.rc_params = rc_params or RC_CIRCUIT_PARAMS
        
        # Initialize submodels
        self.aging_model = BatteryAgingModel()
        self.temp_model_correction = TemperatureCorrectionModel()
        self.ocv_model = OCVModel()
        self.thermal_model = ThermalModel()
        
        # Cache for computed parameters
        self._update_aging_parameters()
    
    def _update_aging_parameters(self):
        """Update aging-dependent parameters."""
        N = self.cycle_number
        self.Q_max_base = self.aging_model.capacity_fade(N)
        self.R_total_base = self.aging_model.impedance_growth(N)
    
    def get_effective_capacity(self, T_c):
        """
        Get temperature-corrected maximum capacity.
        
        Parameters:
            T_c: Core temperature [°C]
        
        Returns:
            Q_max_eff: Effective maximum capacity [Ah]
        """
        S_Q = self.temp_model_correction.capacity_factor(T_c)
        return self.Q_max_base * S_Q
    
    def get_effective_resistance(self, T_c):
        """
        Get temperature-corrected total resistance.
        
        Parameters:
            T_c: Core temperature [°C]
        
        Returns:
            R_total_eff: Effective total resistance [Ω]
        """
        S_R = self.temp_model_correction.resistance_factor(T_c)
        return self.R_total_base * S_R
    
    def get_rc_parameters(self, T_c):
        """
        Get individual RC circuit parameters.
        
        Parameters:
            T_c: Core temperature [°C]
        
        Returns:
            R_0, R_1, C_1, R_2, C_2: RC circuit parameters
        """
        R_total = self.get_effective_resistance(T_c)
        
        p = self.rc_params
        R_0 = R_total * p['R0_ratio']
        R_1 = R_total * p['R1_ratio']
        R_2 = R_total * p['R2_ratio']
        
        # Calculate capacitances from time constants
        C_1 = p['tau_1'] / R_1 if R_1 > 0 else 1.0
        C_2 = p['tau_2'] / R_2 if R_2 > 0 else 1.0
        
        return R_0, R_1, C_1, R_2, C_2
    
    def state_equations(self, t, state, I_func, T_env_func):
        """
        Continuous-time state equations for the battery.
        
        Parameters:
            t: Time [s]
            state: State vector [z, V_1, V_2, T_c, T_s]
            I_func: Function returning load current I(t) [A]
            T_env_func: Function returning environment temperature T_env(t) [°C]
        
        Returns:
            dstate: State derivatives [dz/dt, dV_1/dt, dV_2/dt, dT_c/dt, dT_s/dt]
        """
        # Unpack state
        z, V_1, V_2, T_c, T_s = state
        
        # Get current input
        I = I_func(t)
        T_env = T_env_func(t)
        
        # Get temperature-corrected parameters
        Q_max = self.get_effective_capacity(T_c)
        R_0, R_1, C_1, R_2, C_2 = self.get_rc_parameters(T_c)
        R_total = R_0 + R_1 + R_2
        
        # Coulombic efficiency
        eta = coulombic_efficiency(T_c)
        
        # SOC dynamics (Coulomb counting)
        dz_dt = -I * eta / (Q_max * 3600)
        
        # RC circuit dynamics
        dV_1_dt = -V_1 / (R_1 * C_1) + I / C_1
        dV_2_dt = -V_2 / (R_2 * C_2) + I / C_2
        
        # Heat generation
        Q_gen = self.thermal_model.heat_generation(I, R_total, T_c)
        
        # Thermal dynamics
        dT_c_dt, dT_s_dt = self.thermal_model.temperature_dynamics(T_c, T_s, T_env, Q_gen)
        
        return [dz_dt, dV_1_dt, dV_2_dt, dT_c_dt, dT_s_dt]
    
    def terminal_voltage(self, state, I):
        """
        Calculate terminal voltage.
        
        Parameters:
            state: State vector [z, V_1, V_2, T_c, T_s]
            I: Load current [A]
        
        Returns:
            V_term: Terminal voltage [V]
        """
        z, V_1, V_2, T_c, T_s = state
        
        V_OCV = self.ocv_model.voltage(z)
        R_0, _, _, _, _ = self.get_rc_parameters(T_c)
        
        V_term = V_OCV - V_1 - V_2 - I * R_0
        return V_term
    
    def simulate(self, t_span, initial_state, I_func, T_env_func, 
                 method='RK45', max_step=1.0):
        """
        Simulate battery dynamics over time.
        
        Parameters:
            t_span: (t_start, t_end) time span [s]
            initial_state: Initial state [z_0, V_1_0, V_2_0, T_c_0, T_s_0]
            I_func: Load current function I(t) [A]
            T_env_func: Environment temperature function T_env(t) [°C]
            method: Integration method ('RK45', 'BDF', 'Radau')
            max_step: Maximum step size [s]
        
        Returns:
            t: Time array [s]
            states: State array (n_times, 5)
            V_term: Terminal voltage array [V]
        """
        # Define ODE function
        def ode_func(t, state):
            return self.state_equations(t, state, I_func, T_env_func)
        
        # Event function for SOC limits
        def soc_min_event(t, state):
            return state[0] - 0.01  # Trigger when SOC < 1%
        soc_min_event.terminal = True
        soc_min_event.direction = -1
        
        def soc_max_event(t, state):
            return 0.99 - state[0]  # Trigger when SOC > 99%
        soc_max_event.terminal = True
        soc_max_event.direction = -1
        
        # Solve ODE
        solution = solve_ivp(
            ode_func,
            t_span,
            initial_state,
            method=method,
            max_step=max_step,
            events=[soc_min_event, soc_max_event],
            dense_output=True
        )
        
        # Calculate terminal voltage at each time point
        V_term = np.array([
            self.terminal_voltage(solution.y[:, i], I_func(solution.t[i]))
            for i in range(len(solution.t))
        ])
        
        return solution.t, solution.y.T, V_term
    
    def remaining_discharge_time(self, current_state, I_avg, T_env=25.0, 
                                  V_cutoff=None, max_time=86400):
        """
        Predict remaining discharge time.
        
        Parameters:
            current_state: Current state [z, V_1, V_2, T_c, T_s]
            I_avg: Average discharge current [A]
            T_env: Environment temperature [°C]
            V_cutoff: Cutoff voltage [V]
            max_time: Maximum simulation time [s]
        
        Returns:
            t_remaining: Remaining time until cutoff [s]
            soc_final: Final SOC [-]
        """
        V_cutoff = V_cutoff or self.battery_params['V_min']
        
        # Define constant load
        I_func = lambda t: I_avg
        T_env_func = lambda t: T_env
        
        # Simulate
        t, states, V_term = self.simulate(
            (0, max_time), 
            current_state, 
            I_func, 
            T_env_func,
            max_step=10.0
        )
        
        # Find cutoff point
        cutoff_idx = np.where(V_term < V_cutoff)[0]
        if len(cutoff_idx) > 0:
            t_remaining = t[cutoff_idx[0]]
            soc_final = states[cutoff_idx[0], 0]
        else:
            t_remaining = t[-1]
            soc_final = states[-1, 0]
        
        return t_remaining, soc_final


# =============================================================================
# Factory function for easy model creation
# =============================================================================
def create_battery_model(cycle_number=0, **kwargs):
    """
    Factory function to create a battery model.
    
    Parameters:
        cycle_number: Battery cycle count [-]
        **kwargs: Additional parameters to override defaults
    
    Returns:
        BatteryCoreModel instance
    """
    return BatteryCoreModel(cycle_number=cycle_number, **kwargs)


if __name__ == "__main__":
    # Simple test
    battery = create_battery_model(cycle_number=100)
    
    # Initial state: 80% SOC, no polarization, 25°C
    initial_state = [0.8, 0.0, 0.0, 25.0, 25.0]
    
    # Constant 1A discharge at 25°C environment
    I_func = lambda t: 1.0
    T_env_func = lambda t: 25.0
    
    # Simulate 1 hour
    t, states, V_term = battery.simulate(
        (0, 3600), 
        initial_state, 
        I_func, 
        T_env_func
    )
    
    print(f"Simulation completed: {len(t)} time points")
    print(f"Initial SOC: {states[0, 0]:.3f}")
    print(f"Final SOC: {states[-1, 0]:.3f}")
    print(f"Initial Voltage: {V_term[0]:.3f} V")
    print(f"Final Voltage: {V_term[-1]:.3f} V")
    print(f"Temperature rise: {states[-1, 3] - states[0, 3]:.2f} °C")
