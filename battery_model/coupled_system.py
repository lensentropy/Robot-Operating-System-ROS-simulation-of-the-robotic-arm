"""
Coupled Electro-Thermal-Aging-Load System Model
=================================================
High-Fidelity Smartphone Battery Discharge Simulation

This module integrates:
1. Battery electrochemical model (OCV, 2nd-order RC)
2. Aging model (capacity fade, impedance growth)
3. Thermal model (two-state lumped thermal)
4. Multi-physics load models (5G, BT, GNSS, Display, SoC)

System State Vector:
    x = [z, V_1, V_2, T_c, T_s, T_soc, x_lock]
    
    z       - State of Charge [-]
    V_1     - Electrochemical polarization voltage [V]
    V_2     - Concentration polarization voltage [V]
    T_c     - Battery core temperature [°C]
    T_s     - Battery surface temperature [°C]
    T_soc   - SoC chip temperature [°C]
    x_lock  - GNSS lock state [-]

Continuous-Time Governing Equations:
------------------------------------
The system evolution is governed by a set of coupled ODEs:

Battery Dynamics:
    dz/dt = -I(t) * η(T_c) / (Q_max(N, T_c) * 3600)
    dV_1/dt = -V_1/(R_1*C_1) + I(t)/C_1
    dV_2/dt = -V_2/(R_2*C_2) + I(t)/C_2

Thermal Dynamics:
    dT_c/dt = (Q_gen - (T_c - T_s)/R_cs) / C_c
    dT_s/dt = ((T_c - T_s)/R_cs - (T_s - T_env)/R_se) / C_s

Load State Dynamics:
    dT_soc/dt = (P_soc - (T_soc - T_amb)/R_th_soc) / C_th_soc
    dx_lock/dt = (Ψ(S_env) - x_lock) / τ_react

Coupling Equations:
    I(t) = P_total(t, V_term) / V_term(t)
    V_term(t) = V_OCV(z) - V_1 - V_2 - I*R_0
    Q_gen = I² * R_total + I * T_c * (dV_OCV/dT)
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve
import warnings

from battery_core import BatteryCoreModel, create_battery_model
from load_models import SmartphoneLoadManager, UsageScenarios
from battery_params import (
    BATTERY_PARAMS, THERMAL_PARAMS, LOAD_SOC_PARAMS, LOAD_GNSS_PARAMS,
    coulombic_efficiency
)


class CoupledSmartphoneModel:
    """
    Complete coupled electro-thermal-aging-load smartphone model.
    
    This class provides:
    - Continuous-time state-space representation
    - Self-consistent current-voltage iteration
    - Multi-physics coupling between battery, thermal, and load subsystems
    - Adaptive time-stepping for stiff ODEs
    """
    
    def __init__(self, cycle_number=0, initial_soc=1.0, 
                 environment_temp=25.0, usage_profile=None):
        """
        Initialize coupled smartphone model.
        
        Parameters:
            cycle_number: Battery cycle count [-]
            initial_soc: Initial state of charge [-]
            environment_temp: Environment temperature [°C]
            usage_profile: Usage scenario profile dict
        """
        # Battery model
        self.battery = create_battery_model(cycle_number=cycle_number)
        
        # Load manager
        self.load_manager = SmartphoneLoadManager()
        
        # Environment
        self.T_env = environment_temp
        
        # Usage profile
        self.profile = usage_profile or UsageScenarios.idle_screen_on()
        self.load_manager.set_profile(**self.profile)
        
        # Initial state
        self.initial_state = self._create_initial_state(initial_soc)
        
        # Simulation parameters
        self.V_cutoff = BATTERY_PARAMS['V_min']
        
        # Storage for results
        self.results = None
    
    def _create_initial_state(self, initial_soc):
        """
        Create initial state vector.
        
        State: [z, V_1, V_2, T_c, T_s, T_soc, x_lock]
        """
        return np.array([
            initial_soc,        # z - SOC
            0.0,               # V_1 - Electrochemical polarization
            0.0,               # V_2 - Concentration polarization
            self.T_env,        # T_c - Battery core temperature
            self.T_env,        # T_s - Battery surface temperature
            self.T_env + 5,    # T_soc - SoC temperature (slightly higher)
            0.5,               # x_lock - GNSS lock state (partial lock)
        ])
    
    def _solve_current_voltage(self, state, profile=None):
        """
        Solve for self-consistent current and voltage.
        
        The load power P_load depends on voltage V:
            P_load = f(V)
            I = P_load / V
            V = V_OCV - I*R - V_1 - V_2
        
        Parameters:
            state: Current state vector
            profile: Usage profile
        
        Returns:
            I: Load current [A]
            V_term: Terminal voltage [V]
            P_total: Total power [W]
        """
        z, V_1, V_2, T_c, T_s, T_soc, x_lock = state
        
        # Get battery parameters
        R_0, R_1, C_1, R_2, C_2 = self.battery.get_rc_parameters(T_c)
        V_OCV = self.battery.ocv_model.voltage(z)
        
        # Initial guess for voltage
        V_guess = V_OCV - V_1 - V_2
        
        # Iterative solution for current-voltage consistency
        def residual(V_term):
            if V_term <= 0:
                return 1e6
            
            # Calculate power at this voltage
            P_total, _ = self.load_manager.total_power(
                0, V_term, T_c, 1.0, profile
            )
            
            # Calculate current
            I = P_total / V_term
            
            # Calculate what voltage should be
            V_calc = V_OCV - V_1 - V_2 - I * R_0
            
            return V_calc - V_term
        
        # Solve for self-consistent voltage
        try:
            V_term = fsolve(residual, V_guess, full_output=False)[0]
        except:
            V_term = V_guess
        
        # Ensure positive voltage
        V_term = max(V_term, self.V_cutoff)
        
        # Calculate final current and power
        P_total, breakdown = self.load_manager.total_power(
            0, V_term, T_c, 1.0, profile
        )
        I = P_total / V_term
        
        return I, V_term, P_total, breakdown
    
    def state_equations(self, t, state, profile=None, T_env=None):
        """
        Coupled system state equations.
        
        Parameters:
            t: Time [s]
            state: State vector [z, V_1, V_2, T_c, T_s, T_soc, x_lock]
            profile: Usage profile
            T_env: Environment temperature [°C]
        
        Returns:
            dstate: State derivatives
        """
        z, V_1, V_2, T_c, T_s, T_soc, x_lock = state
        T_env = T_env or self.T_env
        profile = profile or self.profile
        
        # Clamp SOC
        z = np.clip(z, 0.01, 0.99)
        
        # Solve for current
        I, V_term, P_total, _ = self._solve_current_voltage(state, profile)
        
        # Get battery parameters
        Q_max = self.battery.get_effective_capacity(T_c)
        R_0, R_1, C_1, R_2, C_2 = self.battery.get_rc_parameters(T_c)
        R_total = R_0 + R_1 + R_2
        
        # Coulombic efficiency
        eta = coulombic_efficiency(T_c)
        
        # ===== Battery Dynamics =====
        # SOC dynamics (Coulomb counting)
        dz_dt = -I * eta / (Q_max * 3600)
        
        # RC circuit dynamics
        tau_1 = R_1 * C_1
        tau_2 = R_2 * C_2
        dV_1_dt = -V_1 / tau_1 + I / C_1
        dV_2_dt = -V_2 / tau_2 + I / C_2
        
        # ===== Thermal Dynamics =====
        # Heat generation (Bernardi equation)
        Q_gen = self.battery.thermal_model.heat_generation(I, R_total, T_c)
        
        # Battery core and surface temperatures
        dT_c_dt, dT_s_dt = self.battery.thermal_model.temperature_dynamics(
            T_c, T_s, T_env, Q_gen
        )
        
        # ===== Load State Dynamics =====
        # SoC temperature dynamics
        p_soc = LOAD_SOC_PARAMS
        P_soc = self.load_manager.load_soc.power_consumption(
            t, profile.get('cpu_load', 0.3), T_env, 1.0
        )
        dT_soc_dt = (P_soc - (T_soc - T_env) / p_soc['R_th_soc']) / p_soc['C_th_soc']
        
        # GNSS lock state dynamics
        p_gnss = LOAD_GNSS_PARAMS
        if profile.get('gnss_active', False):
            S_env = self.load_manager.load_gnss.signal_quality(
                profile.get('gnss_environment', 'urban'),
                profile.get('gnss_satellites', 8)
            )
            Psi = self.load_manager.load_gnss.lock_probability(S_env)
            dx_lock_dt = (Psi - x_lock) / p_gnss['tau_react']
        else:
            dx_lock_dt = -x_lock / p_gnss['tau_react']  # Decay to 0 when off
        
        return [dz_dt, dV_1_dt, dV_2_dt, dT_c_dt, dT_s_dt, dT_soc_dt, dx_lock_dt]
    
    def simulate(self, duration, time_step=1.0, method='RK45',
                 profile=None, T_env_func=None, profile_schedule=None):
        """
        Run coupled system simulation.
        
        Parameters:
            duration: Simulation duration [s]
            time_step: Maximum time step [s]
            method: ODE solver method
            profile: Static usage profile
            T_env_func: Time-varying environment temperature function
            profile_schedule: List of (time, profile) tuples for dynamic scenarios
        
        Returns:
            SimulationResults object
        """
        profile = profile or self.profile
        T_env_func = T_env_func or (lambda t: self.T_env)
        
        # Build profile lookup if schedule provided
        if profile_schedule:
            schedule_times = [s[0] for s in profile_schedule]
            schedule_profiles = [s[1] for s in profile_schedule]
            
            def get_profile(t):
                idx = np.searchsorted(schedule_times, t, side='right') - 1
                idx = max(0, min(idx, len(schedule_profiles) - 1))
                return schedule_profiles[idx]
        else:
            get_profile = lambda t: profile
        
        # Define ODE function
        def ode_func(t, state):
            current_profile = get_profile(t)
            T_env = T_env_func(t)
            return self.state_equations(t, state, current_profile, T_env)
        
        # Event functions
        def soc_depleted(t, state):
            return state[0] - 0.01  # SOC < 1%
        soc_depleted.terminal = True
        soc_depleted.direction = -1
        
        def voltage_cutoff(t, state):
            I, V_term, _, _ = self._solve_current_voltage(state, get_profile(t))
            return V_term - self.V_cutoff
        voltage_cutoff.terminal = True
        voltage_cutoff.direction = -1
        
        # Solve ODE
        print(f"Starting simulation: {duration/3600:.2f} hours")
        
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            solution = solve_ivp(
                ode_func,
                (0, duration),
                self.initial_state,
                method=method,
                max_step=time_step,
                events=[soc_depleted],
                dense_output=True
            )
        
        print(f"Simulation completed: {solution.t[-1]/3600:.2f} hours, {len(solution.t)} time points")
        
        # Post-process results
        results = SimulationResults(self, solution, get_profile, T_env_func)
        self.results = results
        
        return results
    
    def predict_remaining_time(self, current_state=None, profile=None):
        """
        Predict remaining discharge time from current state.
        
        Parameters:
            current_state: Current state vector (uses initial if None)
            profile: Usage profile
        
        Returns:
            t_remaining: Remaining time [s]
            soc_final: Final SOC [-]
        """
        state = current_state if current_state is not None else self.initial_state
        profile = profile or self.profile
        
        # Quick estimation based on average current
        I, V_term, P_total, _ = self._solve_current_voltage(state, profile)
        z = state[0]
        T_c = state[3]
        Q_max = self.battery.get_effective_capacity(T_c)
        
        # Simple coulomb counting estimate
        t_estimate = z * Q_max * 3600 / I
        
        return t_estimate, 0.0


class SimulationResults:
    """
    Container for simulation results with post-processing methods.
    """
    
    def __init__(self, model, solution, profile_func, T_env_func):
        """
        Initialize results container.
        
        Parameters:
            model: CoupledSmartphoneModel instance
            solution: scipy.integrate solution object
            profile_func: Profile function of time
            T_env_func: Environment temperature function
        """
        self.model = model
        self.t = solution.t
        self.states = solution.y.T  # (n_times, 7)
        self.profile_func = profile_func
        self.T_env_func = T_env_func
        
        # Post-process
        self._compute_derived_quantities()
    
    def _compute_derived_quantities(self):
        """Compute derived quantities from states."""
        n = len(self.t)
        
        self.soc = self.states[:, 0]
        self.V_1 = self.states[:, 1]
        self.V_2 = self.states[:, 2]
        self.T_core = self.states[:, 3]
        self.T_surface = self.states[:, 4]
        self.T_soc = self.states[:, 5]
        self.x_lock = self.states[:, 6]
        
        # Compute voltages, currents, powers
        self.V_term = np.zeros(n)
        self.I_load = np.zeros(n)
        self.P_total = np.zeros(n)
        self.power_breakdown = []
        
        for i in range(n):
            state = self.states[i]
            profile = self.profile_func(self.t[i])
            
            I, V, P, breakdown = self.model._solve_current_voltage(state, profile)
            self.V_term[i] = V
            self.I_load[i] = I
            self.P_total[i] = P
            self.power_breakdown.append(breakdown)
        
        # Compute energy consumed
        self.energy_consumed = np.trapz(self.P_total, self.t) / 3600  # Wh
        
        # Compute remaining capacity
        Q_max = self.model.battery.get_effective_capacity(self.T_core[-1])
        self.remaining_capacity = self.soc[-1] * Q_max
    
    @property
    def discharge_time(self):
        """Total discharge time [s]."""
        return self.t[-1]
    
    @property
    def discharge_time_hours(self):
        """Total discharge time [hours]."""
        return self.t[-1] / 3600
    
    def get_power_breakdown_at(self, time_index):
        """Get power breakdown at specific time index."""
        return self.power_breakdown[time_index]
    
    def average_power(self):
        """Calculate average power consumption [W]."""
        return np.mean(self.P_total)
    
    def summary(self):
        """Print simulation summary."""
        print("\n" + "=" * 60)
        print("SIMULATION SUMMARY")
        print("=" * 60)
        print(f"Discharge time:     {self.discharge_time_hours:.2f} hours ({self.discharge_time:.0f} s)")
        print(f"Initial SOC:        {self.soc[0]*100:.1f}%")
        print(f"Final SOC:          {self.soc[-1]*100:.1f}%")
        print(f"Energy consumed:    {self.energy_consumed:.3f} Wh")
        print(f"Average power:      {self.average_power()*1000:.1f} mW")
        print(f"Average current:    {np.mean(self.I_load)*1000:.1f} mA")
        print(f"Initial voltage:    {self.V_term[0]:.3f} V")
        print(f"Final voltage:      {self.V_term[-1]:.3f} V")
        print(f"Peak temperature:   {np.max(self.T_core):.1f} °C")
        print(f"Temperature rise:   {self.T_core[-1] - self.T_core[0]:.1f} °C")
        
        # Power breakdown (average)
        avg_breakdown = {}
        for key in self.power_breakdown[0].keys():
            if key != 'Total':
                avg_breakdown[key] = np.mean([b[key] for b in self.power_breakdown])
        
        print("\nAverage Power Breakdown:")
        print("-" * 40)
        for component, power in sorted(avg_breakdown.items(), key=lambda x: -x[1]):
            pct = power / self.average_power() * 100
            print(f"  {component:12s}: {power*1000:7.1f} mW ({pct:5.1f}%)")
        
        return {
            'discharge_time_hours': self.discharge_time_hours,
            'energy_consumed_wh': self.energy_consumed,
            'average_power_mw': self.average_power() * 1000,
            'peak_temperature_c': np.max(self.T_core),
        }


# =============================================================================
# Factory Functions
# =============================================================================
def create_smartphone_model(cycle_number=0, initial_soc=1.0, 
                            environment_temp=25.0, scenario='idle_screen_on'):
    """
    Factory function to create a smartphone model with predefined scenario.
    
    Parameters:
        cycle_number: Battery cycle count [-]
        initial_soc: Initial SOC [-]
        environment_temp: Environment temperature [°C]
        scenario: Scenario name or dict
    
    Returns:
        CoupledSmartphoneModel instance
    """
    # Get scenario profile
    scenario_map = {
        'idle_screen_off': UsageScenarios.idle_screen_off,
        'idle_screen_on': UsageScenarios.idle_screen_on,
        'video_streaming': UsageScenarios.video_streaming,
        'gaming': UsageScenarios.gaming,
        'navigation': UsageScenarios.navigation,
        'voice_call': UsageScenarios.voice_call,
        'weak_signal': UsageScenarios.weak_signal,
    }
    
    if isinstance(scenario, str):
        profile = scenario_map.get(scenario, UsageScenarios.idle_screen_on)()
    else:
        profile = scenario
    
    return CoupledSmartphoneModel(
        cycle_number=cycle_number,
        initial_soc=initial_soc,
        environment_temp=environment_temp,
        usage_profile=profile
    )


def quick_discharge_simulation(scenario='idle_screen_on', cycle_number=0,
                               initial_soc=1.0, T_env=25.0, 
                               max_hours=24.0, time_step=10.0):
    """
    Quick simulation for battery discharge prediction.
    
    Parameters:
        scenario: Usage scenario name
        cycle_number: Battery cycle count
        initial_soc: Initial SOC
        T_env: Environment temperature [°C]
        max_hours: Maximum simulation hours
        time_step: Time step [s]
    
    Returns:
        SimulationResults object
    """
    model = create_smartphone_model(
        cycle_number=cycle_number,
        initial_soc=initial_soc,
        environment_temp=T_env,
        scenario=scenario
    )
    
    results = model.simulate(
        duration=max_hours * 3600,
        time_step=time_step
    )
    
    return results


if __name__ == "__main__":
    # Example simulation
    print("Creating smartphone model...")
    
    model = create_smartphone_model(
        cycle_number=100,
        initial_soc=0.8,
        environment_temp=25.0,
        scenario='idle_screen_on'
    )
    
    print("Running simulation...")
    results = model.simulate(duration=3600, time_step=10.0)
    
    results.summary()
