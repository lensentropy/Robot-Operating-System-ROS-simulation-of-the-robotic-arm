"""
High-Fidelity Coupled Electro-Thermal-Aging Model for Lithium-Ion Batteries
Based on NASA PCoE Dataset and Panasonic NCR18650B specifications

This module implements:
- Capacity fade model (double exponential)
- Impedance growth model (power law)
- OCV model based on Nernst equation
- 2nd-order Thevenin equivalent circuit
- Two-state thermal model with Bernardi heating
"""

import numpy as np
from scipy.optimize import curve_fit
from dataclasses import dataclass
from typing import Tuple, Optional
import warnings

@dataclass
class BatteryParameters:
    """Battery physical parameters"""
    # Aging model coefficients (NASA Dataset #5)
    a_Q: float = -0.1137  # Capacity fade coefficient 1
    b_Q: float = 0.0243   # Capacity fade rate 1
    c_Q: float = 1.9305   # Capacity fade coefficient 2
    d_Q: float = 0.0007   # Capacity fade rate 2
    
    # Temperature correction for capacity (Sigmoid)
    S_Q: float = 1.0391   # Capacity scaling factor
    k_Q: float = 0.0895   # Temperature sensitivity
    T0_Q: float = -15.1281  # Inflection temperature (°C)
    
    # Impedance growth coefficients
    a_R: float = 0.0110   # Power law coefficient
    b_R: float = 0.2106   # Power law exponent
    c_R: float = 0.0181   # Base resistance (Ohm)
    
    # Temperature correction for resistance (Arrhenius)
    C_R: float = 0.7136   # Base factor
    A_R: float = 1.3533   # Amplitude
    B_R: float = 0.0630   # Decay rate
    
    # OCV model coefficients (Nernst-based combined model)
    K0: float = 3.4704    # Constant term
    K1: float = 0.1670    # Linear term
    K2: float = -0.0042   # Inverse term
    K3: float = 0.0573    # Log(z) term
    K4: float = -0.0847   # Log(1-z) term
    
    # Equivalent circuit parameters
    R0_ratio: float = 0.5   # Ohmic resistance ratio
    R1_ratio: float = 0.3   # Electrochemical polarization ratio
    R2_ratio: float = 0.2   # Concentration polarization ratio
    tau1: float = 10.0      # Time constant 1 (s)
    tau2: float = 100.0     # Time constant 2 (s)
    
    # Thermal model parameters
    Cc: float = 62.7        # Core heat capacity (J/K)
    Cs: float = 4.5         # Surface heat capacity (J/K)
    Rc_s: float = 1.94      # Core-surface thermal resistance (K/W)
    Rs_e: float = 3.08      # Surface-environment thermal resistance (K/W)
    
    # Entropy coefficient
    dVdT: float = -0.0003   # Entropic heat coefficient (V/K)
    
    # Nominal parameters
    Q_nominal: float = 2.0  # Nominal capacity (Ah)
    V_max: float = 4.2      # Maximum voltage (V)
    V_min: float = 2.5      # Minimum voltage (V)


class BatteryModel:
    """
    High-fidelity coupled electro-thermal-aging battery model
    """
    
    def __init__(self, params: Optional[BatteryParameters] = None):
        self.params = params or BatteryParameters()
        
    def capacity_fade(self, N: np.ndarray) -> np.ndarray:
        """
        Double exponential capacity fade model
        Q_max(N) = a_Q * exp(-b_Q * N) + c_Q * exp(-d_Q * N)
        
        Args:
            N: Cycle number
        Returns:
            Maximum available capacity (Ah)
        """
        p = self.params
        return p.a_Q * np.exp(-p.b_Q * N) + p.c_Q * np.exp(-p.d_Q * N)
    
    def temperature_capacity_factor(self, T: np.ndarray) -> np.ndarray:
        """
        Sigmoid-based temperature correction for capacity
        S_Q(T) = S_Q / (1 + exp(-k_Q * (T - T0)))
        
        Args:
            T: Temperature (°C)
        Returns:
            Capacity correction factor
        """
        p = self.params
        return p.S_Q / (1 + np.exp(-p.k_Q * (T - p.T0_Q)))
    
    def capacity_with_aging_and_temp(self, N: float, T: float) -> float:
        """
        Combined capacity model with aging and temperature effects
        
        Args:
            N: Cycle number
            T: Temperature (°C)
        Returns:
            Effective capacity (Ah)
        """
        Q_aged = self.capacity_fade(np.array([N]))[0]
        S_T = self.temperature_capacity_factor(np.array([T]))[0]
        return Q_aged * S_T
    
    def impedance_growth(self, N: np.ndarray) -> np.ndarray:
        """
        Power law impedance growth model
        R_total(N) = a_R * N^b_R + c_R
        
        Args:
            N: Cycle number
        Returns:
            Total DC resistance (Ohm)
        """
        p = self.params
        return p.a_R * np.power(N, p.b_R) + p.c_R
    
    def temperature_resistance_factor(self, T: np.ndarray) -> np.ndarray:
        """
        Arrhenius-based temperature correction for resistance
        S_R(T) = C_R + A_R * exp(-B_R * T)
        
        Args:
            T: Temperature (°C)
        Returns:
            Resistance correction factor
        """
        p = self.params
        return p.C_R + p.A_R * np.exp(-p.B_R * T)
    
    def resistance_with_aging_and_temp(self, N: float, T: float) -> float:
        """
        Combined resistance model with aging and temperature effects
        
        Args:
            N: Cycle number
            T: Temperature (°C)
        Returns:
            Total internal resistance (Ohm)
        """
        R_aged = self.impedance_growth(np.array([N]))[0]
        S_T = self.temperature_resistance_factor(np.array([T]))[0]
        return R_aged * S_T
    
    def ocv(self, z: np.ndarray) -> np.ndarray:
        """
        Open Circuit Voltage based on Nernst equation
        V_OCV(z) = K0 + K1*z + K2/z + K3*ln(z) + K4*ln(1-z)
        
        Args:
            z: State of Charge (0-1)
        Returns:
            Open circuit voltage (V)
        """
        p = self.params
        # Clamp z to avoid numerical issues
        z = np.clip(z, 1e-6, 1-1e-6)
        return (p.K0 + p.K1 * z + p.K2 / z + 
                p.K3 * np.log(z) + p.K4 * np.log(1 - z))
    
    def get_circuit_parameters(self, R_total: float, T: float) -> Tuple[float, float, float, float, float]:
        """
        Distribute total resistance to circuit components
        
        Args:
            R_total: Total internal resistance
            T: Temperature (°C)
        Returns:
            R0, R1, R2, C1, C2
        """
        p = self.params
        # Temperature-dependent distribution
        temp_factor = 1.0 + 0.01 * (25 - T)  # Adjust ratios with temperature
        
        R0 = R_total * p.R0_ratio * temp_factor
        R1 = R_total * p.R1_ratio
        R2 = R_total * p.R2_ratio / temp_factor
        
        C1 = p.tau1 / R1 if R1 > 0 else 1e6
        C2 = p.tau2 / R2 if R2 > 0 else 1e6
        
        return R0, R1, R2, C1, C2
    
    def bernardi_heat_generation(self, I: float, R_total: float, T: float) -> float:
        """
        Bernardi equation for heat generation
        Q_gen = I^2 * R_total + I * T * (dV_OCV/dT)
        
        Args:
            I: Current (A)
            R_total: Total resistance (Ohm)
            T: Temperature (K or °C)
        Returns:
            Heat generation rate (W)
        """
        p = self.params
        # Joule heating (irreversible)
        Q_joule = I**2 * R_total
        # Entropic heating (reversible)
        T_kelvin = T + 273.15 if T < 200 else T  # Convert if in Celsius
        Q_entropy = abs(I) * T_kelvin * p.dVdT
        return Q_joule + Q_entropy


class BatterySimulator:
    """
    Dynamic simulation of battery behavior using Forward Euler method
    """
    
    def __init__(self, model: BatteryModel, dt: float = 1.0):
        """
        Initialize simulator
        
        Args:
            model: BatteryModel instance
            dt: Time step (seconds)
        """
        self.model = model
        self.dt = dt
        self.reset()
        
    def reset(self, SOC_init: float = 1.0, T_env: float = 25.0, N: int = 0):
        """Reset simulator state"""
        self.SOC = SOC_init
        self.V1 = 0.0  # Electrochemical polarization voltage
        self.V2 = 0.0  # Concentration polarization voltage
        self.Tc = T_env  # Core temperature
        self.Ts = T_env  # Surface temperature
        self.T_env = T_env
        self.N = N  # Cycle number
        self.time = 0.0
        
    def step(self, I: float) -> dict:
        """
        Advance simulation by one time step
        
        Args:
            I: Load current (A), positive for discharge
        Returns:
            Dictionary with state variables
        """
        p = self.model.params
        
        # Get current capacity and resistance
        Q_max = self.model.capacity_with_aging_and_temp(self.N, self.Tc)
        R_total = self.model.resistance_with_aging_and_temp(self.N, self.Tc)
        
        # Get circuit parameters
        R0, R1, R2, C1, C2 = self.model.get_circuit_parameters(R_total, self.Tc)
        
        # Update polarization voltages (Forward Euler)
        tau1 = R1 * C1
        tau2 = R2 * C2
        
        dV1_dt = -self.V1 / tau1 + I / C1
        dV2_dt = -self.V2 / tau2 + I / C2
        
        self.V1 += dV1_dt * self.dt
        self.V2 += dV2_dt * self.dt
        
        # Calculate terminal voltage
        V_ocv = self.model.ocv(np.array([self.SOC]))[0]
        V_term = V_ocv - self.V1 - self.V2 - I * R0
        
        # Heat generation (Bernardi)
        Q_gen = self.model.bernardi_heat_generation(I, R_total, self.Tc)
        
        # Update temperatures (Two-state thermal model)
        dTc_dt = (Q_gen - (self.Tc - self.Ts) / p.Rc_s) / p.Cc
        dTs_dt = ((self.Tc - self.Ts) / p.Rc_s - (self.Ts - self.T_env) / p.Rs_e) / p.Cs
        
        self.Tc += dTc_dt * self.dt
        self.Ts += dTs_dt * self.dt
        
        # Update SOC (Coulomb counting with efficiency)
        eta = self._coulombic_efficiency(self.Tc)
        dSOC = -I * eta * self.dt / (Q_max * 3600)
        self.SOC += dSOC
        self.SOC = np.clip(self.SOC, 0, 1)
        
        self.time += self.dt
        
        return {
            'time': self.time,
            'SOC': self.SOC,
            'V_term': V_term,
            'V_ocv': V_ocv,
            'V1': self.V1,
            'V2': self.V2,
            'Tc': self.Tc,
            'Ts': self.Ts,
            'R_total': R_total,
            'Q_max': Q_max,
            'Q_gen': Q_gen,
            'I': I
        }
    
    def _coulombic_efficiency(self, T: float) -> float:
        """Temperature-dependent coulombic efficiency"""
        # Efficiency decreases at low temperatures
        if T < -10:
            return 0.85
        elif T < 0:
            return 0.90 + 0.005 * T
        elif T < 45:
            return 0.995
        else:
            return 0.99
    
    def simulate(self, current_profile: np.ndarray, 
                 SOC_init: float = 1.0, 
                 T_env: float = 25.0,
                 N: int = 0) -> dict:
        """
        Run full simulation with given current profile
        
        Args:
            current_profile: Array of current values (A)
            SOC_init: Initial SOC
            T_env: Environment temperature (°C)
            N: Cycle number
        Returns:
            Dictionary of result arrays
        """
        self.reset(SOC_init, T_env, N)
        
        results = {key: [] for key in ['time', 'SOC', 'V_term', 'V_ocv', 
                                        'V1', 'V2', 'Tc', 'Ts', 'R_total', 
                                        'Q_max', 'Q_gen', 'I']}
        
        for I in current_profile:
            state = self.step(I)
            for key, value in state.items():
                results[key].append(value)
            
            # Check for cutoff
            if state['V_term'] < self.model.params.V_min:
                break
            if state['SOC'] < 0.01:
                break
                
        # Convert to numpy arrays
        for key in results:
            results[key] = np.array(results[key])
            
        return results


def generate_aging_data(cycles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate synthetic aging data based on NASA dataset patterns
    
    Args:
        cycles: Array of cycle numbers
    Returns:
        capacity, resistance arrays
    """
    model = BatteryModel()
    capacity = model.capacity_fade(cycles)
    resistance = model.impedance_growth(cycles)
    
    # Add realistic noise
    np.random.seed(42)
    capacity += np.random.normal(0, 0.01, len(cycles))
    resistance += np.random.normal(0, 0.002, len(cycles))
    
    return capacity, resistance


def fit_aging_parameters(cycles: np.ndarray, 
                         capacity: np.ndarray, 
                         resistance: np.ndarray) -> dict:
    """
    Fit aging model parameters to experimental data
    
    Args:
        cycles: Cycle numbers
        capacity: Measured capacity (Ah)
        resistance: Measured resistance (Ohm)
    Returns:
        Dictionary of fitted parameters with statistics
    """
    # Capacity fade model: double exponential
    def cap_model(N, a, b, c, d):
        return a * np.exp(-b * N) + c * np.exp(-d * N)
    
    # Resistance growth model: power law
    def res_model(N, a, b, c):
        return a * np.power(N, b) + c
    
    # Fit capacity
    try:
        cap_popt, cap_pcov = curve_fit(cap_model, cycles, capacity, 
                                        p0=[-0.1, 0.02, 1.9, 0.001],
                                        maxfev=5000)
        cap_pred = cap_model(cycles, *cap_popt)
        cap_r2 = 1 - np.sum((capacity - cap_pred)**2) / np.sum((capacity - np.mean(capacity))**2)
        cap_rmse = np.sqrt(np.mean((capacity - cap_pred)**2))
    except Exception as e:
        warnings.warn(f"Capacity fitting failed: {e}")
        cap_popt = [-0.1137, 0.0243, 1.9305, 0.0007]
        cap_r2 = 0
        cap_rmse = 0
    
    # Fit resistance
    try:
        res_popt, res_pcov = curve_fit(res_model, cycles, resistance,
                                        p0=[0.01, 0.2, 0.02],
                                        maxfev=5000)
        res_pred = res_model(cycles, *res_popt)
        res_r2 = 1 - np.sum((resistance - res_pred)**2) / np.sum((resistance - np.mean(resistance))**2)
        res_rmse = np.sqrt(np.mean((resistance - res_pred)**2))
    except Exception as e:
        warnings.warn(f"Resistance fitting failed: {e}")
        res_popt = [0.011, 0.2106, 0.0181]
        res_r2 = 0
        res_rmse = 0
    
    return {
        'capacity': {
            'a_Q': cap_popt[0],
            'b_Q': cap_popt[1],
            'c_Q': cap_popt[2],
            'd_Q': cap_popt[3],
            'R2': cap_r2,
            'RMSE': cap_rmse
        },
        'resistance': {
            'a_R': res_popt[0],
            'b_R': res_popt[1],
            'c_R': res_popt[2],
            'R2': res_r2,
            'RMSE': res_rmse
        }
    }


if __name__ == "__main__":
    # Quick test
    model = BatteryModel()
    simulator = BatterySimulator(model, dt=1.0)
    
    # Constant discharge at 1.5A
    current = np.ones(3600) * 1.5
    results = simulator.simulate(current, SOC_init=1.0, T_env=25.0, N=100)
    
    print(f"Simulation completed:")
    print(f"  Final SOC: {results['SOC'][-1]:.3f}")
    print(f"  Final Voltage: {results['V_term'][-1]:.3f} V")
    print(f"  Final Core Temp: {results['Tc'][-1]:.2f} °C")
    print(f"  Duration: {results['time'][-1]:.0f} s")
