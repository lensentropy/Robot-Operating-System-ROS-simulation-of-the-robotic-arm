"""
Kalman Filter Implementations for SOC Estimation
基于卡尔曼滤波的SOC估计

This module implements:
1. Extended Kalman Filter (EKF) for nonlinear state estimation
2. Unscented Kalman Filter (UKF) for highly nonlinear systems
3. Adaptive Kalman Filter with noise covariance estimation

References:
[1] Plett, "Extended Kalman Filtering for Battery Management Systems," 
    J. Power Sources, 2004
[2] He et al., "State of Charge Estimation Using Unscented Kalman Filter," 
    Energy, 2011
"""

import numpy as np
from typing import Tuple, Optional, Callable, List
from dataclasses import dataclass, field
from scipy.linalg import cholesky, sqrtm
import warnings


@dataclass
class KalmanState:
    """State estimate with covariance"""
    x: np.ndarray          # State mean
    P: np.ndarray          # State covariance
    innovation: float = 0  # Measurement innovation
    K: np.ndarray = None   # Kalman gain


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for Battery SOC Estimation
    扩展卡尔曼滤波器用于电池SOC估计
    
    State vector: x = [SOC, T_batt, R_int]
    Measurement: y = V_terminal
    
    The EKF linearizes the nonlinear battery model at each time step.
    """
    
    def __init__(self, 
                 Q_max: float = 14400.0,  # Capacity in As
                 dt: float = 1.0,          # Time step
                 V_OCV_func: Callable = None,
                 R_int_func: Callable = None):
        """
        Initialize EKF
        
        Parameters:
        -----------
        Q_max : float
            Battery capacity in Ampere-seconds
        dt : float
            Sampling time in seconds
        V_OCV_func : callable
            Function to compute OCV from SOC
        R_int_func : callable
            Function to compute internal resistance
        """
        self.Q_max = Q_max
        self.dt = dt
        
        # Default OCV function (simple polynomial)
        if V_OCV_func is None:
            self.V_OCV = lambda soc: 3.0 + 1.2 * soc - 0.3 * soc**2 + 0.2 * soc**3
        else:
            self.V_OCV = V_OCV_func
        
        # Default resistance function
        if R_int_func is None:
            self.R_int = lambda soc, T: 0.05 * (1 + 0.3*(soc-0.5)**2) * np.exp(2500*(1/T - 1/298.15))
        else:
            self.R_int = R_int_func
        
        # State dimension
        self.n_states = 3  # [SOC, T_batt, R_int]
        
        # Initialize state and covariance
        self.x = np.array([1.0, 298.15, 0.05])  # Initial: 100% SOC, 25°C, 50mΩ
        self.P = np.diag([0.01, 1.0, 0.001])    # Initial covariance
        
        # Process noise covariance
        self.Q = np.diag([1e-6, 0.01, 1e-8])
        
        # Measurement noise covariance
        self.R = np.array([[0.001]])  # Voltage measurement noise variance
        
        # History for analysis
        self.state_history = []
        self.covariance_history = []
        self.innovation_history = []
    
    def state_transition(self, x: np.ndarray, I: float, T_env: float = 298.15) -> np.ndarray:
        """
        State transition function f(x, u)
        状态转移函数
        
        x_k+1 = f(x_k, I_k)
        
        SOC: dSOC/dt = -I/Q_max
        T_batt: dT/dt = (I²R + I*T*dV/dT - (T-T_env)/R_th) / C_th
        R_int: slow drift (random walk)
        """
        SOC, T_batt, R_int = x
        
        # SOC dynamics (Coulomb counting)
        SOC_new = SOC - I * self.dt / self.Q_max
        
        # Thermal dynamics
        R_th = 10.0  # K/W, thermal resistance
        C_th = 50.0  # J/K, thermal capacitance
        dVdT = -0.0002  # V/K
        
        P_joule = I**2 * R_int
        P_entropy = abs(I) * T_batt * abs(dVdT)
        Q_dissipated = (T_batt - T_env) / R_th
        
        T_batt_new = T_batt + (P_joule + P_entropy - Q_dissipated) * self.dt / C_th
        
        # R_int: slow random walk (updated via process noise)
        R_int_new = R_int
        
        return np.array([SOC_new, T_batt_new, R_int_new])
    
    def state_jacobian(self, x: np.ndarray, I: float, T_env: float = 298.15) -> np.ndarray:
        """
        Compute Jacobian of state transition: F = df/dx
        状态转移雅可比矩阵
        """
        SOC, T_batt, R_int = x
        
        F = np.eye(self.n_states)
        
        # ∂SOC_new/∂SOC = 1 (already in identity)
        
        # Thermal dynamics Jacobian
        R_th = 10.0
        C_th = 50.0
        dVdT = -0.0002
        
        # ∂T_new/∂T = 1 - dt/(R_th*C_th) + dt*|I|*|dVdT|/C_th
        F[1, 1] = 1 - self.dt / (R_th * C_th) + self.dt * abs(I) * abs(dVdT) / C_th
        
        # ∂T_new/∂R_int = dt * I² / C_th
        F[1, 2] = self.dt * I**2 / C_th
        
        return F
    
    def measurement_function(self, x: np.ndarray, I: float) -> float:
        """
        Measurement function h(x)
        测量函数: V_terminal = V_OCV(SOC) - I * R_int
        """
        SOC, T_batt, R_int = x
        V_OCV = self.V_OCV(np.clip(SOC, 0.01, 0.99))
        V_terminal = V_OCV - I * R_int
        return V_terminal
    
    def measurement_jacobian(self, x: np.ndarray, I: float) -> np.ndarray:
        """
        Compute Jacobian of measurement: H = dh/dx
        测量雅可比矩阵
        """
        SOC, T_batt, R_int = x
        
        # Numerical derivative of V_OCV w.r.t. SOC
        eps = 1e-6
        dVOCV_dSOC = (self.V_OCV(SOC + eps) - self.V_OCV(SOC - eps)) / (2 * eps)
        
        H = np.array([[dVOCV_dSOC, 0, -I]])
        return H
    
    def predict(self, I: float, T_env: float = 298.15) -> KalmanState:
        """
        Prediction step (time update)
        预测步骤
        """
        # State prediction
        x_pred = self.state_transition(self.x, I, T_env)
        
        # Jacobian
        F = self.state_jacobian(self.x, I, T_env)
        
        # Covariance prediction
        P_pred = F @ self.P @ F.T + self.Q
        
        return KalmanState(x=x_pred, P=P_pred)
    
    def update(self, V_measured: float, I: float, 
               predicted: KalmanState) -> KalmanState:
        """
        Update step (measurement update)
        更新步骤
        """
        x_pred = predicted.x
        P_pred = predicted.P
        
        # Measurement Jacobian
        H = self.measurement_jacobian(x_pred, I)
        
        # Innovation (measurement residual)
        V_pred = self.measurement_function(x_pred, I)
        innovation = V_measured - V_pred
        
        # Innovation covariance
        S = H @ P_pred @ H.T + self.R
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # State update
        x_updated = x_pred + K.flatten() * innovation
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.n_states) - K @ H
        P_updated = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        
        # Clip SOC to valid range
        x_updated[0] = np.clip(x_updated[0], 0.0, 1.0)
        
        # Update internal state
        self.x = x_updated
        self.P = P_updated
        
        # Store history
        self.state_history.append(x_updated.copy())
        self.covariance_history.append(np.diag(P_updated).copy())
        self.innovation_history.append(innovation)
        
        return KalmanState(x=x_updated, P=P_updated, innovation=innovation, K=K)
    
    def step(self, V_measured: float, I: float, T_env: float = 298.15) -> KalmanState:
        """
        Complete EKF step: predict + update
        完整的EKF步骤
        """
        predicted = self.predict(I, T_env)
        updated = self.update(V_measured, I, predicted)
        return updated
    
    def get_SOC_estimate(self) -> Tuple[float, float]:
        """
        Get current SOC estimate with uncertainty
        返回SOC估计值及其不确定性
        """
        return self.x[0], np.sqrt(self.P[0, 0])
    
    def reset(self, SOC_init: float = 1.0, T_init: float = 298.15):
        """Reset filter to initial state"""
        self.x = np.array([SOC_init, T_init, 0.05])
        self.P = np.diag([0.01, 1.0, 0.001])
        self.state_history = []
        self.covariance_history = []
        self.innovation_history = []


class UnscentedKalmanFilter:
    """
    Unscented Kalman Filter for Battery SOC Estimation
    无迹卡尔曼滤波器用于电池SOC估计
    
    UKF handles nonlinearities better than EKF by using sigma points
    to propagate the state distribution through nonlinear functions.
    """
    
    def __init__(self,
                 Q_max: float = 14400.0,
                 dt: float = 1.0,
                 V_OCV_func: Callable = None,
                 R_int_func: Callable = None,
                 alpha: float = 1e-3,
                 beta: float = 2.0,
                 kappa: float = 0.0):
        """
        Initialize UKF with tuning parameters
        
        Parameters:
        -----------
        alpha : float
            Spread of sigma points (typically 1e-4 to 1)
        beta : float
            Prior knowledge about distribution (2 is optimal for Gaussian)
        kappa : float
            Secondary scaling parameter (usually 0 or 3-n)
        """
        self.Q_max = Q_max
        self.dt = dt
        
        # State dimension
        self.n = 3  # [SOC, T_batt, R_int]
        
        # UKF parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = alpha**2 * (self.n + kappa) - self.n
        
        # Sigma point weights
        self._compute_weights()
        
        # OCV and resistance functions
        if V_OCV_func is None:
            self.V_OCV = lambda soc: 3.0 + 1.2 * soc - 0.3 * soc**2 + 0.2 * soc**3
        else:
            self.V_OCV = V_OCV_func
        
        if R_int_func is None:
            self.R_int = lambda soc, T: 0.05 * (1 + 0.3*(soc-0.5)**2)
        else:
            self.R_int = R_int_func
        
        # Initialize state
        self.x = np.array([1.0, 298.15, 0.05])
        self.P = np.diag([0.01, 1.0, 0.001])
        
        # Noise covariances
        self.Q = np.diag([1e-6, 0.01, 1e-8])
        self.R = np.array([[0.001]])
        
        # History
        self.state_history = []
        self.sigma_points_history = []
    
    def _compute_weights(self):
        """Compute sigma point weights"""
        n = self.n
        lambda_ = self.lambda_
        
        # Mean weights
        self.W_m = np.zeros(2*n + 1)
        self.W_m[0] = lambda_ / (n + lambda_)
        self.W_m[1:] = 1 / (2 * (n + lambda_))
        
        # Covariance weights
        self.W_c = np.zeros(2*n + 1)
        self.W_c[0] = lambda_ / (n + lambda_) + (1 - self.alpha**2 + self.beta)
        self.W_c[1:] = 1 / (2 * (n + lambda_))
    
    def _compute_sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        Compute sigma points for the unscented transform
        计算Sigma点
        """
        n = self.n
        sigma_points = np.zeros((2*n + 1, n))
        
        # Matrix square root of P
        try:
            sqrt_P = cholesky((n + self.lambda_) * P, lower=True)
        except np.linalg.LinAlgError:
            # If Cholesky fails, use eigenvalue decomposition
            P_reg = P + 1e-8 * np.eye(n)
            sqrt_P = np.real(sqrtm((n + self.lambda_) * P_reg))
        
        sigma_points[0] = x
        for i in range(n):
            sigma_points[i + 1] = x + sqrt_P[:, i]
            sigma_points[n + i + 1] = x - sqrt_P[:, i]
        
        return sigma_points
    
    def _state_transition(self, x: np.ndarray, I: float, T_env: float = 298.15) -> np.ndarray:
        """State transition for a single sigma point"""
        SOC, T_batt, R_int = x
        
        # SOC dynamics
        SOC_new = SOC - I * self.dt / self.Q_max
        SOC_new = np.clip(SOC_new, 0.0, 1.0)
        
        # Thermal dynamics
        R_th = 10.0
        C_th = 50.0
        dVdT = -0.0002
        
        P_joule = I**2 * R_int
        P_entropy = abs(I) * T_batt * abs(dVdT)
        Q_diss = (T_batt - T_env) / R_th
        
        T_batt_new = T_batt + (P_joule + P_entropy - Q_diss) * self.dt / C_th
        
        R_int_new = R_int
        
        return np.array([SOC_new, T_batt_new, R_int_new])
    
    def _measurement_function(self, x: np.ndarray, I: float) -> float:
        """Measurement function for a single sigma point"""
        SOC, T_batt, R_int = x
        V_OCV = self.V_OCV(np.clip(SOC, 0.01, 0.99))
        return V_OCV - I * R_int
    
    def predict(self, I: float, T_env: float = 298.15) -> Tuple[np.ndarray, np.ndarray]:
        """
        UKF prediction step
        UKF预测步骤
        """
        # Generate sigma points
        sigma_points = self._compute_sigma_points(self.x, self.P)
        
        # Propagate sigma points through state transition
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2*self.n + 1):
            sigma_points_pred[i] = self._state_transition(sigma_points[i], I, T_env)
        
        # Compute predicted mean
        x_pred = np.sum(self.W_m[:, np.newaxis] * sigma_points_pred, axis=0)
        
        # Compute predicted covariance
        P_pred = self.Q.copy()
        for i in range(2*self.n + 1):
            diff = sigma_points_pred[i] - x_pred
            P_pred += self.W_c[i] * np.outer(diff, diff)
        
        return x_pred, P_pred, sigma_points_pred
    
    def update(self, V_measured: float, I: float,
               x_pred: np.ndarray, P_pred: np.ndarray,
               sigma_points_pred: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        UKF update step
        UKF更新步骤
        """
        # Regenerate sigma points from predicted state
        sigma_points = self._compute_sigma_points(x_pred, P_pred)
        
        # Transform sigma points through measurement function
        gamma = np.zeros(2*self.n + 1)
        for i in range(2*self.n + 1):
            gamma[i] = self._measurement_function(sigma_points[i], I)
        
        # Predicted measurement mean
        y_pred = np.sum(self.W_m * gamma)
        
        # Measurement covariance
        P_yy = self.R[0, 0]
        for i in range(2*self.n + 1):
            P_yy += self.W_c[i] * (gamma[i] - y_pred)**2
        
        # Cross-covariance
        P_xy = np.zeros(self.n)
        for i in range(2*self.n + 1):
            P_xy += self.W_c[i] * (sigma_points[i] - x_pred) * (gamma[i] - y_pred)
        
        # Kalman gain
        K = P_xy / P_yy
        
        # Innovation
        innovation = V_measured - y_pred
        
        # State update
        x_updated = x_pred + K * innovation
        x_updated[0] = np.clip(x_updated[0], 0.0, 1.0)
        
        # Covariance update
        P_updated = P_pred - np.outer(K, K) * P_yy
        
        return x_updated, P_updated, innovation
    
    def step(self, V_measured: float, I: float, T_env: float = 298.15) -> Tuple[np.ndarray, np.ndarray]:
        """
        Complete UKF step
        完整的UKF步骤
        """
        # Predict
        x_pred, P_pred, sigma_pred = self.predict(I, T_env)
        
        # Update
        x_updated, P_updated, innovation = self.update(V_measured, I, x_pred, P_pred, sigma_pred)
        
        # Store state
        self.x = x_updated
        self.P = P_updated
        
        # History
        self.state_history.append(x_updated.copy())
        
        return x_updated, P_updated
    
    def get_SOC_estimate(self) -> Tuple[float, float]:
        """Get SOC estimate with 1-sigma uncertainty"""
        return self.x[0], np.sqrt(self.P[0, 0])
    
    def reset(self, SOC_init: float = 1.0, T_init: float = 298.15):
        """Reset filter"""
        self.x = np.array([SOC_init, T_init, 0.05])
        self.P = np.diag([0.01, 1.0, 0.001])
        self.state_history = []


class AdaptiveKalmanFilter(ExtendedKalmanFilter):
    """
    Adaptive EKF with online noise covariance estimation
    自适应扩展卡尔曼滤波器
    
    Automatically adjusts Q and R based on innovation sequence.
    """
    
    def __init__(self, *args, window_size: int = 20, **kwargs):
        super().__init__(*args, **kwargs)
        self.window_size = window_size
        self.innovation_window = []
    
    def adapt_noise_covariance(self, innovation: float, H: np.ndarray, P_pred: np.ndarray):
        """
        Adapt measurement noise R based on innovation statistics
        基于新息统计自适应调整测量噪声
        """
        self.innovation_window.append(innovation)
        
        if len(self.innovation_window) > self.window_size:
            self.innovation_window.pop(0)
        
        if len(self.innovation_window) >= self.window_size // 2:
            # Estimate innovation variance
            innovations = np.array(self.innovation_window)
            S_empirical = np.var(innovations)
            
            # Expected innovation variance
            S_expected = (H @ P_pred @ H.T + self.R)[0, 0]
            
            # Adaptation factor
            if S_expected > 0:
                ratio = S_empirical / S_expected
                
                # Smooth adaptation
                adaptation_rate = 0.1
                if ratio > 1.5:  # Innovation too large
                    self.R *= (1 + adaptation_rate * (ratio - 1))
                elif ratio < 0.5:  # Innovation too small
                    self.R *= (1 - adaptation_rate * (1 - ratio))
    
    def step(self, V_measured: float, I: float, T_env: float = 298.15) -> KalmanState:
        """EKF step with adaptation"""
        # Predict
        predicted = self.predict(I, T_env)
        
        # Compute measurement Jacobian
        H = self.measurement_jacobian(predicted.x, I)
        
        # Adapt noise before update
        V_pred = self.measurement_function(predicted.x, I)
        innovation_estimate = V_measured - V_pred
        self.adapt_noise_covariance(innovation_estimate, H, predicted.P)
        
        # Update
        updated = self.update(V_measured, I, predicted)
        
        return updated


class DualKalmanFilter:
    """
    Dual Kalman Filter for joint state and parameter estimation
    双卡尔曼滤波器用于联合状态和参数估计
    
    Runs two coupled filters:
    1. State filter: estimates SOC, T_batt
    2. Parameter filter: estimates R_int, Q_max
    """
    
    def __init__(self, dt: float = 1.0):
        self.dt = dt
        
        # State filter (SOC, T_batt)
        self.x_state = np.array([1.0, 298.15])
        self.P_state = np.diag([0.01, 1.0])
        self.Q_state = np.diag([1e-6, 0.01])
        
        # Parameter filter (R_int, Q_max_factor)
        self.x_param = np.array([0.05, 1.0])  # R_int, capacity factor
        self.P_param = np.diag([0.001, 0.01])
        self.Q_param = np.diag([1e-8, 1e-6])
        
        # Measurement noise
        self.R = np.array([[0.001]])
        
        # Nominal capacity
        self.Q_max_nominal = 14400.0
        
        # OCV function
        self.V_OCV = lambda soc: 3.0 + 1.2 * soc - 0.3 * soc**2 + 0.2 * soc**3
        
        # History
        self.state_history = []
        self.param_history = []
    
    def step(self, V_measured: float, I: float, T_env: float = 298.15):
        """
        Dual filter step
        双滤波器步骤
        """
        # Extract current estimates
        SOC, T_batt = self.x_state
        R_int, Q_factor = self.x_param
        Q_max = self.Q_max_nominal * Q_factor
        
        # === STATE FILTER ===
        # Predict state
        SOC_pred = SOC - I * self.dt / Q_max
        SOC_pred = np.clip(SOC_pred, 0.0, 1.0)
        
        R_th, C_th = 10.0, 50.0
        dVdT = -0.0002
        P_heat = I**2 * R_int + abs(I) * T_batt * abs(dVdT)
        T_pred = T_batt + (P_heat - (T_batt - T_env)/R_th) * self.dt / C_th
        
        x_state_pred = np.array([SOC_pred, T_pred])
        
        # State Jacobian
        F_state = np.array([
            [1, 0],
            [0, 1 - self.dt/(R_th*C_th)]
        ])
        P_state_pred = F_state @ self.P_state @ F_state.T + self.Q_state
        
        # Measurement prediction
        V_pred = self.V_OCV(SOC_pred) - I * R_int
        
        # State measurement Jacobian
        eps = 1e-6
        dVOCV_dSOC = (self.V_OCV(SOC_pred + eps) - self.V_OCV(SOC_pred - eps)) / (2*eps)
        H_state = np.array([[dVOCV_dSOC, 0]])
        
        # State update
        S_state = H_state @ P_state_pred @ H_state.T + self.R
        K_state = P_state_pred @ H_state.T / S_state[0, 0]
        
        innovation = V_measured - V_pred
        self.x_state = x_state_pred + K_state.flatten() * innovation
        self.x_state[0] = np.clip(self.x_state[0], 0.0, 1.0)
        
        I_KH = np.eye(2) - np.outer(K_state.flatten(), H_state.flatten())
        self.P_state = I_KH @ P_state_pred
        
        # === PARAMETER FILTER ===
        # Parameters evolve as random walk
        x_param_pred = self.x_param.copy()
        P_param_pred = self.P_param + self.Q_param
        
        # Parameter measurement Jacobian
        H_param = np.array([[-I, I * self.dt * SOC_pred / (Q_factor**2 * self.Q_max_nominal)]])
        
        # Parameter update
        S_param = H_param @ P_param_pred @ H_param.T + self.R
        K_param = P_param_pred @ H_param.T / S_param[0, 0]
        
        self.x_param = x_param_pred + K_param.flatten() * innovation
        self.x_param = np.clip(self.x_param, [0.01, 0.7], [0.2, 1.3])
        
        I_KH_param = np.eye(2) - np.outer(K_param.flatten(), H_param.flatten())
        self.P_param = I_KH_param @ P_param_pred
        
        # Store history
        self.state_history.append(self.x_state.copy())
        self.param_history.append(self.x_param.copy())
        
        return self.x_state, self.x_param
    
    def get_estimates(self) -> dict:
        """Get all current estimates"""
        return {
            'SOC': self.x_state[0],
            'SOC_std': np.sqrt(self.P_state[0, 0]),
            'T_batt': self.x_state[1],
            'R_int': self.x_param[0],
            'Q_capacity_factor': self.x_param[1]
        }
