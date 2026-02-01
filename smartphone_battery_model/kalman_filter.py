"""
扩展卡尔曼滤波器 (EKF) 用于SOC估计
Extended Kalman Filter for State of Charge Estimation

卡尔曼滤波在电池SOC估计中的应用：
1. 融合库仑计数法和电压测量
2. 处理测量噪声和模型不确定性
3. 提供实时、鲁棒的SOC估计

理论基础：
────────────────────────────────────────────────────────────────────────
状态空间模型:

连续时间状态方程:
    dx/dt = f(x, u) + w(t)
    
    其中: x = [SOC, V_RC]^T  (状态向量)
          u = I              (输入电流)
          w(t) ~ N(0, Q)     (过程噪声)

观测方程:
    y = h(x) + v(t)
    
    其中: y = V_terminal     (端电压测量)
          v(t) ~ N(0, R)     (测量噪声)

离散化状态方程 (欧拉法):
    x_{k+1} = x_k + dt * f(x_k, u_k) + w_k
    
EKF更新步骤:
1. 预测步骤:
   x̂_{k|k-1} = x̂_{k-1|k-1} + dt * f(x̂_{k-1|k-1}, u_{k-1})
   P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q

2. 更新步骤:
   K_k = P_{k|k-1} * H_k^T * (H_k * P_{k|k-1} * H_k^T + R)^{-1}
   x̂_{k|k} = x̂_{k|k-1} + K_k * (y_k - h(x̂_{k|k-1}))
   P_{k|k} = (I - K_k * H_k) * P_{k|k-1}

其中:
   F_k = ∂f/∂x |_{x̂_{k-1|k-1}}  (状态转移雅可比矩阵)
   H_k = ∂h/∂x |_{x̂_{k|k-1}}     (观测雅可比矩阵)
────────────────────────────────────────────────────────────────────────

参考文献:
[1] Plett, G.L. "Extended Kalman filtering for battery management systems of LiPB-based HEV 
    battery packs Part 1-3" Journal of Power Sources, 2004
[2] He, H., et al. "State-of-Charge Estimation of the Lithium-Ion Battery Using an Adaptive 
    Extended Kalman Filter Based on an Improved Thevenin Model" IEEE TVT, 2011
"""

import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass
import warnings

from battery_model import SmartphoneBatteryModel, UsageProfile, BatteryParameters


@dataclass
class KalmanFilterConfig:
    """
    卡尔曼滤波器配置参数
    """
    # 采样时间 (秒)
    dt: float = 1.0
    
    # 过程噪声协方差
    Q_soc: float = 1e-6  # SOC过程噪声方差
    Q_vrc: float = 1e-4  # RC电压过程噪声方差
    
    # 测量噪声协方差  
    R_voltage: float = 0.01  # 电压测量噪声方差 (V^2)
    
    # 初始估计误差协方差
    P0_soc: float = 0.01  # 初始SOC估计误差方差
    P0_vrc: float = 0.001  # 初始RC电压估计误差方差
    
    # 自适应参数
    adaptive_R: bool = True  # 是否自适应调整测量噪声
    adaptive_Q: bool = True  # 是否自适应调整过程噪声
    
    # 遗忘因子 (用于协方差重置)
    forgetting_factor: float = 1.0


class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器实现
    
    用于智能手机电池SOC估计，融合：
    - 库仑计数（电流积分）
    - 开路电压测量
    - RC网络动态
    """
    
    def __init__(self, 
                 battery_model: SmartphoneBatteryModel,
                 config: KalmanFilterConfig = None):
        """
        初始化EKF
        
        参数:
            battery_model: 电池模型实例
            config: EKF配置
        """
        self.model = battery_model
        self.config = config or KalmanFilterConfig()
        
        # 状态维度: [SOC, V_RC]
        self.state_dim = 2
        
        # RC等效电路参数
        self.R0 = battery_model.battery_params.internal_resistance  # 欧姆内阻
        self.R1 = 0.02  # RC网络电阻
        self.C1 = 3000  # RC网络电容 (F)
        self.tau1 = self.R1 * self.C1  # RC时间常数
        
        # 初始化状态估计
        self.x_hat = np.array([1.0, 0.0])  # [SOC, V_RC]
        
        # 初始化协方差矩阵
        self.P = np.diag([self.config.P0_soc, self.config.P0_vrc])
        
        # 过程噪声协方差
        self.Q = np.diag([self.config.Q_soc, self.config.Q_vrc])
        
        # 测量噪声协方差
        self.R = np.array([[self.config.R_voltage]])
        
        # 历史记录
        self.history: Dict[str, List] = {
            'time': [],
            'soc_estimated': [],
            'soc_predicted': [],
            'voltage_measured': [],
            'voltage_predicted': [],
            'innovation': [],
            'kalman_gain': [],
            'P_trace': []
        }
        
        # 自适应参数
        self.innovation_window = []
        self.window_size = 10
    
    def initialize(self, initial_soc: float, initial_vrc: float = 0.0,
                   P0: np.ndarray = None):
        """
        初始化滤波器状态
        
        参数:
            initial_soc: 初始SOC估计
            initial_vrc: 初始RC电压
            P0: 初始协方差矩阵
        """
        self.x_hat = np.array([initial_soc, initial_vrc])
        
        if P0 is not None:
            self.P = P0
        else:
            self.P = np.diag([self.config.P0_soc, self.config.P0_vrc])
        
        # 清空历史
        for key in self.history:
            self.history[key] = []
        
        self.innovation_window = []
    
    def state_transition(self, x: np.ndarray, current: float, dt: float) -> np.ndarray:
        """
        状态转移函数 f(x, u)
        
        连续时间模型:
        dSOC/dt = -I / Q_eff
        dV_RC/dt = I/C1 - V_RC/(R1*C1)
        
        离散化 (欧拉法):
        SOC_{k+1} = SOC_k - dt * I / Q_eff
        V_RC_{k+1} = V_RC_k + dt * (I/C1 - V_RC_k/τ1)
        
        参数:
            x: 当前状态 [SOC, V_RC]
            current: 放电电流 (A), 正值表示放电
            dt: 时间步长 (s)
            
        返回:
            下一时刻状态
        """
        soc, v_rc = x
        
        # 有效容量 (Ah)
        Q_eff = self.model.effective_capacity(25.0) / 1000.0  # mAh -> Ah
        
        # SOC变化 (库仑计数)
        soc_new = soc - dt * current / (Q_eff * 3600)  # 转换为小时
        
        # RC电压变化
        v_rc_new = v_rc + dt * (current / self.C1 - v_rc / self.tau1)
        
        # 限制SOC范围
        soc_new = np.clip(soc_new, 0.0, 1.0)
        
        return np.array([soc_new, v_rc_new])
    
    def observation_model(self, x: np.ndarray, current: float) -> float:
        """
        观测模型 h(x)
        
        端电压 = OCV(SOC) - V_RC - I*R0
        
        参数:
            x: 状态 [SOC, V_RC]
            current: 放电电流 (A)
            
        返回:
            预测的端电压 (V)
        """
        soc, v_rc = x
        
        # 开路电压
        v_ocv = self.model.open_circuit_voltage(soc)
        
        # 端电压
        v_terminal = v_ocv - v_rc - current * self.R0
        
        return v_terminal
    
    def jacobian_F(self, x: np.ndarray, current: float, dt: float) -> np.ndarray:
        """
        状态转移雅可比矩阵 F = ∂f/∂x
        
        F = [[∂f1/∂SOC, ∂f1/∂V_RC],
             [∂f2/∂SOC, ∂f2/∂V_RC]]
        
        对于线性状态方程:
        F = [[1, 0],
             [0, 1 - dt/τ1]]
        """
        F = np.array([
            [1.0, 0.0],
            [0.0, 1.0 - dt / self.tau1]
        ])
        return F
    
    def jacobian_H(self, x: np.ndarray) -> np.ndarray:
        """
        观测雅可比矩阵 H = ∂h/∂x
        
        H = [∂V/∂SOC, ∂V/∂V_RC]
        
        ∂V/∂SOC = dOCV/dSOC
        ∂V/∂V_RC = -1
        """
        soc = x[0]
        
        # OCV对SOC的导数 (数值微分)
        delta = 1e-4
        soc_upper = min(soc + delta, 1.0)
        soc_lower = max(soc - delta, 0.01)
        
        ocv_upper = self.model.open_circuit_voltage(soc_upper)
        ocv_lower = self.model.open_circuit_voltage(soc_lower)
        
        dOCV_dSOC = (ocv_upper - ocv_lower) / (soc_upper - soc_lower)
        
        H = np.array([[dOCV_dSOC, -1.0]])
        return H
    
    def predict(self, current: float, dt: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        预测步骤
        
        x̂_{k|k-1} = f(x̂_{k-1|k-1}, u_{k-1})
        P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q
        
        参数:
            current: 放电电流 (A)
            dt: 时间步长 (s)
            
        返回:
            预测状态, 预测协方差
        """
        if dt is None:
            dt = self.config.dt
        
        # 状态预测
        x_pred = self.state_transition(self.x_hat, current, dt)
        
        # 雅可比矩阵
        F = self.jacobian_F(self.x_hat, current, dt)
        
        # 协方差预测
        P_pred = F @ self.P @ F.T + self.Q * self.config.forgetting_factor
        
        return x_pred, P_pred
    
    def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
               voltage_measured: float, current: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        更新步骤
        
        K_k = P_{k|k-1} * H_k^T * (H_k * P_{k|k-1} * H_k^T + R)^{-1}
        x̂_{k|k} = x̂_{k|k-1} + K_k * (y_k - h(x̂_{k|k-1}))
        P_{k|k} = (I - K_k * H_k) * P_{k|k-1}
        
        参数:
            x_pred: 预测状态
            P_pred: 预测协方差
            voltage_measured: 测量电压
            current: 放电电流
            
        返回:
            更新后状态, 更新后协方差
        """
        # 预测电压
        voltage_pred = self.observation_model(x_pred, current)
        
        # 创新（残差）
        innovation = voltage_measured - voltage_pred
        
        # 观测雅可比矩阵
        H = self.jacobian_H(x_pred)
        
        # 创新协方差
        S = H @ P_pred @ H.T + self.R
        
        # 卡尔曼增益
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # 状态更新
        x_update = x_pred + K.flatten() * innovation
        
        # 协方差更新 (Joseph形式，数值稳定)
        I_KH = np.eye(self.state_dim) - K @ H
        P_update = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        
        # 自适应噪声估计
        if self.config.adaptive_R:
            self._update_adaptive_R(innovation)
        
        return x_update, P_update, innovation, K
    
    def _update_adaptive_R(self, innovation: float):
        """
        自适应测量噪声估计
        
        基于创新序列的协方差估计测量噪声
        """
        self.innovation_window.append(innovation)
        
        if len(self.innovation_window) > self.window_size:
            self.innovation_window.pop(0)
        
        if len(self.innovation_window) >= 5:
            # 估计创新协方差
            innovations = np.array(self.innovation_window)
            estimated_R = np.var(innovations)
            
            # 平滑更新
            alpha = 0.1  # 平滑因子
            self.R[0, 0] = (1 - alpha) * self.R[0, 0] + alpha * estimated_R
    
    def step(self, voltage_measured: float, current: float, 
             dt: float = None, time: float = None) -> float:
        """
        单步滤波
        
        参数:
            voltage_measured: 测量电压 (V)
            current: 放电电流 (A)
            dt: 时间步长 (s)
            time: 当前时间 (s)
            
        返回:
            估计的SOC
        """
        if dt is None:
            dt = self.config.dt
        
        # 预测
        x_pred, P_pred = self.predict(current, dt)
        
        # 更新
        x_update, P_update, innovation, K = self.update(
            x_pred, P_pred, voltage_measured, current
        )
        
        # 更新状态
        self.x_hat = x_update
        self.x_hat[0] = np.clip(self.x_hat[0], 0.0, 1.0)  # 限制SOC范围
        self.P = P_update
        
        # 记录历史
        if time is not None:
            self.history['time'].append(time)
        self.history['soc_estimated'].append(self.x_hat[0])
        self.history['soc_predicted'].append(x_pred[0])
        self.history['voltage_measured'].append(voltage_measured)
        self.history['voltage_predicted'].append(self.observation_model(x_pred, current))
        self.history['innovation'].append(innovation)
        self.history['kalman_gain'].append(K[0, 0])
        self.history['P_trace'].append(np.trace(self.P))
        
        return self.x_hat[0]
    
    @property
    def soc(self) -> float:
        """当前SOC估计值"""
        return self.x_hat[0]
    
    @property
    def uncertainty(self) -> float:
        """当前SOC估计不确定性 (标准差)"""
        return np.sqrt(self.P[0, 0])


class UnscentedKalmanFilter:
    """
    无迹卡尔曼滤波器 (UKF)
    
    相比EKF，UKF不需要计算雅可比矩阵，对高度非线性系统有更好的性能
    
    算法步骤:
    1. 生成sigma点
    2. 通过非线性函数传播sigma点
    3. 计算加权均值和协方差
    """
    
    def __init__(self,
                 battery_model: SmartphoneBatteryModel,
                 config: KalmanFilterConfig = None):
        """初始化UKF"""
        self.model = battery_model
        self.config = config or KalmanFilterConfig()
        
        self.state_dim = 2
        
        # RC参数
        self.R0 = battery_model.battery_params.internal_resistance
        self.R1 = 0.02
        self.C1 = 3000
        self.tau1 = self.R1 * self.C1
        
        # UKF参数
        self.alpha = 1e-3  # sigma点扩散参数
        self.beta = 2  # 分布先验知识参数 (高斯分布为2)
        self.kappa = 0  # 次要缩放参数
        
        # 计算lambda
        n = self.state_dim
        self.lambda_ = self.alpha**2 * (n + self.kappa) - n
        
        # 计算权重
        self.Wm, self.Wc = self._compute_weights()
        
        # 初始化状态
        self.x_hat = np.array([1.0, 0.0])
        self.P = np.diag([self.config.P0_soc, self.config.P0_vrc])
        self.Q = np.diag([self.config.Q_soc, self.config.Q_vrc])
        self.R = np.array([[self.config.R_voltage]])
        
        # 历史记录
        self.history: Dict[str, List] = {
            'time': [],
            'soc_estimated': [],
            'voltage_measured': [],
            'P_trace': []
        }
    
    def _compute_weights(self) -> Tuple[np.ndarray, np.ndarray]:
        """计算sigma点权重"""
        n = self.state_dim
        
        # 均值权重
        Wm = np.zeros(2 * n + 1)
        Wm[0] = self.lambda_ / (n + self.lambda_)
        Wm[1:] = 1.0 / (2 * (n + self.lambda_))
        
        # 协方差权重
        Wc = Wm.copy()
        Wc[0] += (1 - self.alpha**2 + self.beta)
        
        return Wm, Wc
    
    def _sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        生成sigma点
        
        χ_0 = x̄
        χ_i = x̄ + √((n+λ)P)_i,  i = 1,...,n
        χ_i = x̄ - √((n+λ)P)_{i-n},  i = n+1,...,2n
        """
        n = self.state_dim
        
        # 计算矩阵平方根 (Cholesky分解)
        try:
            sqrt_P = np.linalg.cholesky((n + self.lambda_) * P)
        except np.linalg.LinAlgError:
            # 如果Cholesky失败，使用SVD
            sqrt_P = np.sqrt(n + self.lambda_) * np.real(
                np.linalg.sqrtm(P + 1e-10 * np.eye(n))
            )
        
        # 生成sigma点
        sigma_points = np.zeros((2 * n + 1, n))
        sigma_points[0] = x
        
        for i in range(n):
            sigma_points[i + 1] = x + sqrt_P[:, i]
            sigma_points[n + i + 1] = x - sqrt_P[:, i]
        
        return sigma_points
    
    def state_transition(self, x: np.ndarray, current: float, dt: float) -> np.ndarray:
        """状态转移函数"""
        soc, v_rc = x
        Q_eff = self.model.effective_capacity(25.0) / 1000.0
        
        soc_new = soc - dt * current / (Q_eff * 3600)
        v_rc_new = v_rc + dt * (current / self.C1 - v_rc / self.tau1)
        
        soc_new = np.clip(soc_new, 0.0, 1.0)
        
        return np.array([soc_new, v_rc_new])
    
    def observation_model(self, x: np.ndarray, current: float) -> float:
        """观测模型"""
        soc, v_rc = x
        v_ocv = self.model.open_circuit_voltage(np.clip(soc, 0.01, 1.0))
        return v_ocv - v_rc - current * self.R0
    
    def initialize(self, initial_soc: float, initial_vrc: float = 0.0):
        """初始化滤波器"""
        self.x_hat = np.array([initial_soc, initial_vrc])
        self.P = np.diag([self.config.P0_soc, self.config.P0_vrc])
        
        for key in self.history:
            self.history[key] = []
    
    def step(self, voltage_measured: float, current: float,
             dt: float = None, time: float = None) -> float:
        """
        单步UKF滤波
        """
        if dt is None:
            dt = self.config.dt
        
        n = self.state_dim
        
        # 1. 生成sigma点
        sigma_points = self._sigma_points(self.x_hat, self.P)
        
        # 2. 状态预测：传播sigma点
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2 * n + 1):
            sigma_points_pred[i] = self.state_transition(sigma_points[i], current, dt)
        
        # 3. 计算预测均值和协方差
        x_pred = np.sum(self.Wm[:, np.newaxis] * sigma_points_pred, axis=0)
        
        P_pred = self.Q.copy()
        for i in range(2 * n + 1):
            diff = sigma_points_pred[i] - x_pred
            P_pred += self.Wc[i] * np.outer(diff, diff)
        
        # 4. 观测预测：传播sigma点
        sigma_points_obs = self._sigma_points(x_pred, P_pred)
        y_sigma = np.zeros(2 * n + 1)
        for i in range(2 * n + 1):
            y_sigma[i] = self.observation_model(sigma_points_obs[i], current)
        
        # 5. 计算观测均值和协方差
        y_pred = np.sum(self.Wm * y_sigma)
        
        Pyy = float(self.R)
        for i in range(2 * n + 1):
            diff = y_sigma[i] - y_pred
            Pyy += self.Wc[i] * diff**2
        
        # 6. 计算互协方差
        Pxy = np.zeros(n)
        for i in range(2 * n + 1):
            x_diff = sigma_points_obs[i] - x_pred
            y_diff = y_sigma[i] - y_pred
            Pxy += self.Wc[i] * x_diff * y_diff
        
        # 7. 卡尔曼增益
        K = Pxy / Pyy
        
        # 8. 状态和协方差更新
        innovation = voltage_measured - y_pred
        self.x_hat = x_pred + K * innovation
        self.x_hat[0] = np.clip(self.x_hat[0], 0.0, 1.0)
        
        self.P = P_pred - np.outer(K, K) * Pyy
        
        # 记录历史
        if time is not None:
            self.history['time'].append(time)
        self.history['soc_estimated'].append(self.x_hat[0])
        self.history['voltage_measured'].append(voltage_measured)
        self.history['P_trace'].append(np.trace(self.P))
        
        return self.x_hat[0]
    
    @property
    def soc(self) -> float:
        return self.x_hat[0]
    
    @property
    def uncertainty(self) -> float:
        return np.sqrt(self.P[0, 0])


class AdaptiveKalmanFilter(ExtendedKalmanFilter):
    """
    自适应扩展卡尔曼滤波器
    
    自动调整Q和R矩阵以适应变化的系统特性
    """
    
    def __init__(self,
                 battery_model: SmartphoneBatteryModel,
                 config: KalmanFilterConfig = None):
        super().__init__(battery_model, config)
        
        # 自适应参数
        self.innovation_history = []
        self.residual_history = []
        self.adaptation_window = 20
        
        # Sage-Husa自适应参数
        self.b = 0.98  # 遗忘因子
    
    def step(self, voltage_measured: float, current: float,
             dt: float = None, time: float = None) -> float:
        """
        自适应滤波步骤
        """
        if dt is None:
            dt = self.config.dt
        
        # 预测
        x_pred, P_pred = self.predict(current, dt)
        
        # 预测电压
        voltage_pred = self.observation_model(x_pred, current)
        
        # 创新
        innovation = voltage_measured - voltage_pred
        
        # 观测雅可比矩阵
        H = self.jacobian_H(x_pred)
        
        # 自适应R估计 (Sage-Husa)
        self._adapt_R(innovation, H, P_pred)
        
        # 创新协方差
        S = H @ P_pred @ H.T + self.R
        
        # 卡尔曼增益
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # 状态更新
        x_update = x_pred + K.flatten() * innovation
        
        # 协方差更新
        I_KH = np.eye(self.state_dim) - K @ H
        P_update = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        
        # 自适应Q估计
        self._adapt_Q(x_update, x_pred, K, innovation)
        
        # 更新状态
        self.x_hat = x_update
        self.x_hat[0] = np.clip(self.x_hat[0], 0.0, 1.0)
        self.P = P_update
        
        # 记录
        self.innovation_history.append(innovation)
        if len(self.innovation_history) > self.adaptation_window:
            self.innovation_history.pop(0)
        
        if time is not None:
            self.history['time'].append(time)
        self.history['soc_estimated'].append(self.x_hat[0])
        self.history['voltage_measured'].append(voltage_measured)
        self.history['P_trace'].append(np.trace(self.P))
        
        return self.x_hat[0]
    
    def _adapt_R(self, innovation: float, H: np.ndarray, P_pred: np.ndarray):
        """
        自适应测量噪声估计 (Sage-Husa算法)
        """
        if len(self.innovation_history) < 5:
            return
        
        # 计算创新序列的协方差
        innovations = np.array(self.innovation_history[-self.adaptation_window:])
        innovation_cov = np.mean(innovations**2)
        
        # 估计R
        HPH = H @ P_pred @ H.T
        R_estimated = innovation_cov - HPH[0, 0]
        
        if R_estimated > 0:
            self.R[0, 0] = self.b * self.R[0, 0] + (1 - self.b) * R_estimated
    
    def _adapt_Q(self, x_update: np.ndarray, x_pred: np.ndarray,
                 K: np.ndarray, innovation: float):
        """
        自适应过程噪声估计
        """
        # 残差
        residual = x_update - x_pred
        self.residual_history.append(residual)
        
        if len(self.residual_history) > self.adaptation_window:
            self.residual_history.pop(0)
        
        if len(self.residual_history) < 5:
            return
        
        # 估计Q (简化版)
        residuals = np.array(self.residual_history[-self.adaptation_window:])
        Q_estimated = np.diag(np.var(residuals, axis=0))
        
        # 平滑更新
        self.Q = self.b * self.Q + (1 - self.b) * Q_estimated


class SOCEstimator:
    """
    SOC估计器 - 整合多种滤波方法
    
    提供统一接口，支持：
    - EKF (扩展卡尔曼滤波)
    - UKF (无迹卡尔曼滤波)
    - AEKF (自适应扩展卡尔曼滤波)
    - 组合滤波
    """
    
    def __init__(self, 
                 battery_model: SmartphoneBatteryModel,
                 method: str = 'ekf',
                 config: KalmanFilterConfig = None):
        """
        初始化SOC估计器
        
        参数:
            battery_model: 电池模型
            method: 滤波方法 ('ekf', 'ukf', 'aekf', 'combined')
            config: 滤波器配置
        """
        self.model = battery_model
        self.method = method
        self.config = config or KalmanFilterConfig()
        
        # 初始化滤波器
        if method == 'ekf':
            self.filter = ExtendedKalmanFilter(battery_model, config)
        elif method == 'ukf':
            self.filter = UnscentedKalmanFilter(battery_model, config)
        elif method == 'aekf':
            self.filter = AdaptiveKalmanFilter(battery_model, config)
        elif method == 'combined':
            self.filter_ekf = ExtendedKalmanFilter(battery_model, config)
            self.filter_ukf = UnscentedKalmanFilter(battery_model, config)
        else:
            raise ValueError(f"未知的滤波方法: {method}")
    
    def initialize(self, initial_soc: float):
        """初始化估计器"""
        if self.method == 'combined':
            self.filter_ekf.initialize(initial_soc)
            self.filter_ukf.initialize(initial_soc)
        else:
            self.filter.initialize(initial_soc)
    
    def estimate(self, voltage: float, current: float, 
                 dt: float = None, time: float = None) -> float:
        """
        估计SOC
        
        参数:
            voltage: 测量电压 (V)
            current: 放电电流 (A)
            dt: 时间步长 (s)
            time: 当前时间 (s)
            
        返回:
            估计的SOC
        """
        if self.method == 'combined':
            # 组合滤波：加权平均
            soc_ekf = self.filter_ekf.step(voltage, current, dt, time)
            soc_ukf = self.filter_ukf.step(voltage, current, dt, time)
            
            # 根据不确定性加权
            w_ekf = 1.0 / (self.filter_ekf.uncertainty + 1e-6)
            w_ukf = 1.0 / (self.filter_ukf.uncertainty + 1e-6)
            w_total = w_ekf + w_ukf
            
            return (w_ekf * soc_ekf + w_ukf * soc_ukf) / w_total
        else:
            return self.filter.step(voltage, current, dt, time)
    
    @property
    def soc(self) -> float:
        """当前SOC估计"""
        if self.method == 'combined':
            return (self.filter_ekf.soc + self.filter_ukf.soc) / 2
        return self.filter.soc
    
    @property
    def uncertainty(self) -> float:
        """估计不确定性"""
        if self.method == 'combined':
            return min(self.filter_ekf.uncertainty, self.filter_ukf.uncertainty)
        return self.filter.uncertainty
    
    @property
    def history(self) -> Dict:
        """历史记录"""
        if self.method == 'combined':
            return self.filter_ekf.history
        return self.filter.history


if __name__ == "__main__":
    # 示例：使用EKF估计SOC
    print("=" * 60)
    print("扩展卡尔曼滤波器 SOC 估计示例")
    print("=" * 60)
    
    from battery_model import SmartphoneBatteryModel, create_moderate_usage
    
    # 创建电池模型
    model = SmartphoneBatteryModel()
    usage = create_moderate_usage()
    
    # 创建EKF
    ekf = ExtendedKalmanFilter(model)
    ekf.initialize(initial_soc=0.95)
    
    # 模拟测量数据（添加噪声）
    np.random.seed(42)
    true_soc = 0.95
    dt = 1.0  # 1秒采样
    
    print("\n模拟SOC估计过程:")
    print("-" * 50)
    
    for i in range(100):
        # 计算真实电流
        power = model.total_power_consumption(usage)  # mW
        voltage = model.open_circuit_voltage(true_soc)
        current = power / (voltage * 1000)  # A
        
        # 更新真实SOC
        Q_eff = model.effective_capacity(25.0) / 1000  # Ah
        true_soc -= dt * current / (Q_eff * 3600)
        
        # 模拟电压测量（添加噪声）
        voltage_measured = model.open_circuit_voltage(true_soc) - current * 0.08
        voltage_measured += np.random.normal(0, 0.02)  # 添加测量噪声
        
        # EKF估计
        estimated_soc = ekf.step(voltage_measured, current, dt, time=i*dt)
        
        # 每20步打印一次
        if i % 20 == 0:
            print(f"时间={i:3d}s | 真实SOC={true_soc:.4f} | "
                  f"估计SOC={estimated_soc:.4f} | "
                  f"误差={abs(true_soc-estimated_soc)*100:.2f}%")
    
    print("\n最终估计不确定性 (标准差): {:.4f}".format(ekf.uncertainty))
