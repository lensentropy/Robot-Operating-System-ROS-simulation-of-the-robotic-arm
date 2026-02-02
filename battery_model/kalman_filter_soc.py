"""
扩展卡尔曼滤波器 (EKF) 用于SOC状态估计
Extended Kalman Filter for Battery SOC Estimation

结合电化学模型和电压测量进行SOC估计

Author: Battery Model Expert
Date: February 2026
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional, Callable
from scipy.linalg import cholesky, solve_triangular


@dataclass
class EKFParameters:
    """EKF参数配置"""
    # 过程噪声协方差
    Q_soc: float = 1e-8           # SOC过程噪声
    Q_temp: float = 1e-4          # 温度过程噪声
    Q_rint: float = 1e-10         # 内阻过程噪声
    
    # 测量噪声协方差
    R_voltage: float = 1e-4       # 电压测量噪声
    R_current: float = 1e-4       # 电流测量噪声
    
    # 初始估计误差协方差
    P0_soc: float = 0.01          # SOC初始不确定性
    P0_temp: float = 1.0          # 温度初始不确定性
    P0_rint: float = 1e-4         # 内阻初始不确定性


class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器用于电池状态估计
    
    状态向量: x = [SOC, T, R_int]^T
    测量向量: z = [V_terminal, I_load]^T
    
    状态方程:
    SOC(k+1) = SOC(k) - I(k)*dt / (Q_nom * eta)
    T(k+1) = T(k) + (P_loss - (T-T_amb)/R_th) * dt / C_th
    R_int(k+1) = R_int(k) (缓慢变化)
    
    测量方程:
    V = V_ocv(SOC) - I * R_int
    """
    
    def __init__(self, 
                 battery_capacity_mah: float = 4000.0,
                 params: EKFParameters = None):
        """
        初始化EKF
        
        Parameters:
        -----------
        battery_capacity_mah : float
            电池容量 (mAh)
        params : EKFParameters
            EKF参数
        """
        self.Q_nom = battery_capacity_mah / 1000.0  # 转换为Ah
        self.params = params or EKFParameters()
        
        # 状态维度
        self.n_states = 3    # [SOC, T, R_int]
        self.n_meas = 2      # [V, I]
        
        # 电池物理参数
        self.T_amb = 298.15  # 环境温度 (K)
        self.C_th = 15.0     # 热容 (J/K)
        self.R_th = 8.0      # 热阻 (K/W)
        self.eta_coulomb = 0.998  # 库仑效率
        
        # OCV-SOC查找表 (多项式系数)
        self.ocv_coeffs = np.array([3.0, 0.8, 0.3, -0.1])
        
        # 状态和协方差初始化
        self.x = None        # 状态估计
        self.P = None        # 误差协方差
        
        # 历史记录
        self.history = {
            'time': [],
            'soc_est': [],
            'soc_std': [],
            'temp_est': [],
            'rint_est': [],
            'innovation': [],
            'kalman_gain': []
        }
    
    def initialize(self, soc_init: float = 1.0, 
                   temp_init: float = 298.15,
                   rint_init: float = 0.08):
        """
        初始化滤波器状态
        
        Parameters:
        -----------
        soc_init : float
            初始SOC估计 (0-1)
        temp_init : float
            初始温度估计 (K)
        rint_init : float
            初始内阻估计 (Ohm)
        """
        # 状态初始化
        self.x = np.array([soc_init, temp_init, rint_init])
        
        # 协方差初始化
        self.P = np.diag([
            self.params.P0_soc,
            self.params.P0_temp,
            self.params.P0_rint
        ])
        
        # 清空历史
        self.history = {k: [] for k in self.history}
    
    def ocv_from_soc(self, soc: float) -> float:
        """计算开路电压"""
        soc = np.clip(soc, 0.0, 1.0)
        return np.polyval(self.ocv_coeffs[::-1], soc)
    
    def docv_dsoc(self, soc: float) -> float:
        """OCV对SOC的导数"""
        soc = np.clip(soc, 0.0, 1.0)
        # d/dsoc(a0 + a1*soc + a2*soc^2 + a3*soc^3) = a1 + 2*a2*soc + 3*a3*soc^2
        return self.ocv_coeffs[1] + 2*self.ocv_coeffs[2]*soc + 3*self.ocv_coeffs[3]*soc**2
    
    def state_transition(self, x: np.ndarray, I: float, dt: float) -> np.ndarray:
        """
        状态转移函数 f(x, u)
        
        Parameters:
        -----------
        x : np.ndarray
            当前状态 [SOC, T, R_int]
        I : float
            电流 (A)
        dt : float
            时间步长 (s)
        
        Returns:
        --------
        np.ndarray : 下一状态
        """
        SOC, T, R_int = x
        
        # 温度效率
        eta_temp = 1.0 - 0.01 * max(0, (T - 273.15 - 25))
        eta_temp = max(0.7, min(1.0, eta_temp))
        
        # SOC更新
        dSOC = -I * dt / (self.Q_nom * 3600 * eta_temp * self.eta_coulomb)
        SOC_new = np.clip(SOC + dSOC, 0.0, 1.0)
        
        # 温度更新 (热模型)
        P_loss = I**2 * R_int  # 焦耳热
        dT = (P_loss - (T - self.T_amb) / self.R_th) * dt / self.C_th
        T_new = np.clip(T + dT, 273.15, 373.15)
        
        # 内阻缓慢变化 (可以添加SOC/温度依赖)
        R_int_new = R_int * (1.0 + 0.001 * (1.0 - SOC))  # 随SOC降低略微增加
        
        return np.array([SOC_new, T_new, R_int_new])
    
    def measurement_function(self, x: np.ndarray, I: float) -> np.ndarray:
        """
        测量函数 h(x)
        
        Parameters:
        -----------
        x : np.ndarray
            状态 [SOC, T, R_int]
        I : float
            电流 (A)
        
        Returns:
        --------
        np.ndarray : 预测测量 [V, I]
        """
        SOC, T, R_int = x
        
        # 终端电压
        V_ocv = self.ocv_from_soc(SOC)
        V_terminal = V_ocv - I * R_int
        
        return np.array([V_terminal, I])
    
    def compute_jacobian_F(self, x: np.ndarray, I: float, dt: float) -> np.ndarray:
        """
        计算状态转移雅可比矩阵 F = df/dx
        """
        SOC, T, R_int = x
        
        F = np.eye(self.n_states)
        
        # dSOC'/dSOC
        F[0, 0] = 1.0
        
        # dSOC'/dT (温度效率影响)
        eta_temp = 1.0 - 0.01 * max(0, (T - 273.15 - 25))
        if T > 298.15:
            F[0, 1] = -I * dt / (self.Q_nom * 3600 * eta_temp**2 * self.eta_coulomb) * 0.01
        
        # dT'/dT
        F[1, 1] = 1.0 - dt / (self.C_th * self.R_th)
        
        # dT'/dR_int
        F[1, 2] = I**2 * dt / self.C_th
        
        # dR_int'/dSOC
        F[2, 0] = -0.001 * R_int
        
        # dR_int'/dR_int
        F[2, 2] = 1.0 + 0.001 * (1.0 - SOC)
        
        return F
    
    def compute_jacobian_H(self, x: np.ndarray, I: float) -> np.ndarray:
        """
        计算测量雅可比矩阵 H = dh/dx
        """
        SOC, T, R_int = x
        
        H = np.zeros((self.n_meas, self.n_states))
        
        # dV/dSOC
        H[0, 0] = self.docv_dsoc(SOC)
        
        # dV/dT (忽略,假设OCV不直接依赖T)
        H[0, 1] = 0.0
        
        # dV/dR_int
        H[0, 2] = -I
        
        # dI/d* (测量电流直接观测)
        H[1, :] = 0.0
        
        return H
    
    def predict(self, I: float, dt: float):
        """
        预测步骤
        
        Parameters:
        -----------
        I : float
            电流 (A)
        dt : float
            时间步长 (s)
        """
        # 状态预测
        self.x = self.state_transition(self.x, I, dt)
        
        # 雅可比矩阵
        F = self.compute_jacobian_F(self.x, I, dt)
        
        # 过程噪声协方差
        Q = np.diag([
            self.params.Q_soc,
            self.params.Q_temp,
            self.params.Q_rint
        ])
        
        # 协方差预测
        self.P = F @ self.P @ F.T + Q
    
    def update(self, z: np.ndarray, I: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        更新步骤
        
        Parameters:
        -----------
        z : np.ndarray
            测量值 [V_terminal, I_meas]
        I : float
            电流 (A)
        
        Returns:
        --------
        Tuple : (innovation, kalman_gain)
        """
        # 预测测量
        z_pred = self.measurement_function(self.x, I)
        
        # 创新 (测量残差)
        innovation = z - z_pred
        
        # 测量雅可比
        H = self.compute_jacobian_H(self.x, I)
        
        # 测量噪声协方差
        R = np.diag([
            self.params.R_voltage,
            self.params.R_current
        ])
        
        # 创新协方差
        S = H @ self.P @ H.T + R
        
        # 卡尔曼增益
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # 状态更新
        self.x = self.x + K @ innovation
        
        # 协方差更新 (Joseph形式,数值稳定)
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        # 确保状态在有效范围内
        self.x[0] = np.clip(self.x[0], 0.0, 1.0)  # SOC
        self.x[1] = np.clip(self.x[1], 273.15, 373.15)  # T
        self.x[2] = np.clip(self.x[2], 0.01, 0.5)  # R_int
        
        return innovation, K
    
    def step(self, V_meas: float, I_meas: float, dt: float):
        """
        执行一个滤波步骤 (预测+更新)
        
        Parameters:
        -----------
        V_meas : float
            测量电压 (V)
        I_meas : float
            测量电流 (A)
        dt : float
            时间步长 (s)
        """
        # 预测
        self.predict(I_meas, dt)
        
        # 更新
        z = np.array([V_meas, I_meas])
        innovation, K = self.update(z, I_meas)
        
        # 记录历史
        self.history['soc_est'].append(self.x[0])
        self.history['soc_std'].append(np.sqrt(self.P[0, 0]))
        self.history['temp_est'].append(self.x[1])
        self.history['rint_est'].append(self.x[2])
        self.history['innovation'].append(innovation)
        self.history['kalman_gain'].append(K)
    
    def get_soc_estimate(self) -> Tuple[float, float]:
        """
        获取当前SOC估计值和标准差
        
        Returns:
        --------
        Tuple[float, float] : (SOC估计值, 标准差)
        """
        return self.x[0], np.sqrt(self.P[0, 0])
    
    def get_state_estimate(self) -> Dict:
        """
        获取完整状态估计
        
        Returns:
        --------
        dict : 状态估计和不确定性
        """
        return {
            'soc': self.x[0],
            'soc_std': np.sqrt(self.P[0, 0]),
            'temperature': self.x[1],
            'temperature_std': np.sqrt(self.P[1, 1]),
            'internal_resistance': self.x[2],
            'rint_std': np.sqrt(self.P[2, 2])
        }


class UnscentedKalmanFilter:
    """
    无迹卡尔曼滤波器 (UKF) 用于电池SOC估计
    
    相比EKF,UKF对非线性系统有更好的估计精度
    """
    
    def __init__(self, 
                 battery_capacity_mah: float = 4000.0,
                 params: EKFParameters = None):
        """初始化UKF"""
        self.Q_nom = battery_capacity_mah / 1000.0
        self.params = params or EKFParameters()
        
        self.n_states = 3
        self.n_meas = 2
        
        # UKF参数
        self.alpha = 1e-3    # 分散参数
        self.beta = 2.0      # 先验分布参数 (高斯分布最优为2)
        self.kappa = 0.0     # 次要缩放参数
        
        # 计算lambda
        self.lambda_ = self.alpha**2 * (self.n_states + self.kappa) - self.n_states
        
        # 电池参数 (同EKF)
        self.T_amb = 298.15
        self.C_th = 15.0
        self.R_th = 8.0
        self.eta_coulomb = 0.998
        self.ocv_coeffs = np.array([3.0, 0.8, 0.3, -0.1])
        
        self.x = None
        self.P = None
        self.history = {'soc_est': [], 'soc_std': [], 'temp_est': [], 'rint_est': []}
    
    def initialize(self, soc_init: float = 1.0, 
                   temp_init: float = 298.15,
                   rint_init: float = 0.08):
        """初始化状态"""
        self.x = np.array([soc_init, temp_init, rint_init])
        self.P = np.diag([
            self.params.P0_soc,
            self.params.P0_temp,
            self.params.P0_rint
        ])
        self.history = {k: [] for k in self.history}
    
    def ocv_from_soc(self, soc: float) -> float:
        """计算OCV"""
        soc = np.clip(soc, 0.0, 1.0)
        return np.polyval(self.ocv_coeffs[::-1], soc)
    
    def generate_sigma_points(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成sigma点
        
        Returns:
        --------
        Tuple : (sigma_points, weights_mean, weights_cov)
        """
        n = self.n_states
        n_sigma = 2 * n + 1
        
        # Sigma点
        sigma = np.zeros((n_sigma, n))
        sigma[0] = self.x
        
        # 计算sqrt((n + lambda) * P)
        try:
            sqrt_matrix = cholesky((n + self.lambda_) * self.P, lower=True)
        except np.linalg.LinAlgError:
            sqrt_matrix = np.linalg.cholesky((n + self.lambda_) * (self.P + 1e-6 * np.eye(n)))
        
        for i in range(n):
            sigma[i + 1] = self.x + sqrt_matrix[:, i]
            sigma[i + 1 + n] = self.x - sqrt_matrix[:, i]
        
        # 权重
        Wm = np.zeros(n_sigma)
        Wc = np.zeros(n_sigma)
        
        Wm[0] = self.lambda_ / (n + self.lambda_)
        Wc[0] = Wm[0] + (1 - self.alpha**2 + self.beta)
        
        for i in range(1, n_sigma):
            Wm[i] = 1 / (2 * (n + self.lambda_))
            Wc[i] = Wm[i]
        
        return sigma, Wm, Wc
    
    def state_transition(self, x: np.ndarray, I: float, dt: float) -> np.ndarray:
        """状态转移函数"""
        SOC, T, R_int = x
        
        eta_temp = 1.0 - 0.01 * max(0, (T - 273.15 - 25))
        eta_temp = max(0.7, min(1.0, eta_temp))
        
        dSOC = -I * dt / (self.Q_nom * 3600 * eta_temp * self.eta_coulomb)
        SOC_new = np.clip(SOC + dSOC, 0.0, 1.0)
        
        P_loss = I**2 * R_int
        dT = (P_loss - (T - self.T_amb) / self.R_th) * dt / self.C_th
        T_new = np.clip(T + dT, 273.15, 373.15)
        
        R_int_new = R_int * (1.0 + 0.001 * (1.0 - SOC))
        
        return np.array([SOC_new, T_new, R_int_new])
    
    def measurement_function(self, x: np.ndarray, I: float) -> np.ndarray:
        """测量函数"""
        SOC, T, R_int = x
        V_ocv = self.ocv_from_soc(SOC)
        V_terminal = V_ocv - I * R_int
        return np.array([V_terminal, I])
    
    def step(self, V_meas: float, I_meas: float, dt: float):
        """执行UKF步骤"""
        n = self.n_states
        
        # 生成sigma点
        sigma, Wm, Wc = self.generate_sigma_points()
        n_sigma = sigma.shape[0]
        
        # 预测
        sigma_pred = np.zeros_like(sigma)
        for i in range(n_sigma):
            sigma_pred[i] = self.state_transition(sigma[i], I_meas, dt)
        
        # 预测均值和协方差
        x_pred = np.sum(Wm[:, np.newaxis] * sigma_pred, axis=0)
        
        Q = np.diag([self.params.Q_soc, self.params.Q_temp, self.params.Q_rint])
        P_pred = Q.copy()
        for i in range(n_sigma):
            diff = sigma_pred[i] - x_pred
            P_pred += Wc[i] * np.outer(diff, diff)
        
        # 更新状态和协方差
        self.x = x_pred
        self.P = P_pred
        
        # 重新生成sigma点
        sigma, Wm, Wc = self.generate_sigma_points()
        
        # 测量预测
        z_sigma = np.zeros((n_sigma, self.n_meas))
        for i in range(n_sigma):
            z_sigma[i] = self.measurement_function(sigma[i], I_meas)
        
        z_pred = np.sum(Wm[:, np.newaxis] * z_sigma, axis=0)
        
        # 测量协方差
        R = np.diag([self.params.R_voltage, self.params.R_current])
        Pzz = R.copy()
        for i in range(n_sigma):
            diff = z_sigma[i] - z_pred
            Pzz += Wc[i] * np.outer(diff, diff)
        
        # 交叉协方差
        Pxz = np.zeros((n, self.n_meas))
        for i in range(n_sigma):
            diff_x = sigma[i] - self.x
            diff_z = z_sigma[i] - z_pred
            Pxz += Wc[i] * np.outer(diff_x, diff_z)
        
        # 卡尔曼增益
        try:
            K = Pxz @ np.linalg.inv(Pzz)
        except np.linalg.LinAlgError:
            K = Pxz @ np.linalg.pinv(Pzz)
        
        # 更新
        z_meas = np.array([V_meas, I_meas])
        innovation = z_meas - z_pred
        self.x = self.x + K @ innovation
        self.P = self.P - K @ Pzz @ K.T
        
        # 约束
        self.x[0] = np.clip(self.x[0], 0.0, 1.0)
        self.x[1] = np.clip(self.x[1], 273.15, 373.15)
        self.x[2] = np.clip(self.x[2], 0.01, 0.5)
        
        # 记录
        self.history['soc_est'].append(self.x[0])
        self.history['soc_std'].append(np.sqrt(self.P[0, 0]))
        self.history['temp_est'].append(self.x[1])
        self.history['rint_est'].append(self.x[2])
    
    def get_soc_estimate(self) -> Tuple[float, float]:
        """获取SOC估计"""
        return self.x[0], np.sqrt(self.P[0, 0])


class AdaptiveEKF(ExtendedKalmanFilter):
    """
    自适应扩展卡尔曼滤波器
    
    动态调整过程噪声和测量噪声协方差
    """
    
    def __init__(self, 
                 battery_capacity_mah: float = 4000.0,
                 params: EKFParameters = None,
                 window_size: int = 20):
        """初始化自适应EKF"""
        super().__init__(battery_capacity_mah, params)
        
        self.window_size = window_size
        self.innovation_window = []
    
    def update_noise_covariance(self, innovation: np.ndarray):
        """
        基于创新序列自适应更新噪声协方差
        """
        self.innovation_window.append(innovation)
        
        if len(self.innovation_window) > self.window_size:
            self.innovation_window.pop(0)
        
        if len(self.innovation_window) >= self.window_size // 2:
            # 估计创新协方差
            innovations = np.array(self.innovation_window)
            S_est = np.cov(innovations.T)
            
            if S_est.ndim == 0:
                S_est = np.array([[S_est]])
            
            # 自适应调整测量噪声
            trace_S = np.trace(S_est)
            if trace_S > 1e-3:  # 如果创新过大,增加测量噪声
                self.params.R_voltage *= 1.05
                self.params.R_current *= 1.05
            elif trace_S < 1e-5:  # 如果创新很小,减小测量噪声
                self.params.R_voltage *= 0.95
                self.params.R_current *= 0.95
            
            # 限制范围
            self.params.R_voltage = np.clip(self.params.R_voltage, 1e-6, 1e-2)
            self.params.R_current = np.clip(self.params.R_current, 1e-6, 1e-2)


if __name__ == "__main__":
    # 测试卡尔曼滤波器
    print("测试扩展卡尔曼滤波器...")
    
    ekf = ExtendedKalmanFilter(battery_capacity_mah=4000)
    ekf.initialize(soc_init=0.95, temp_init=298.15, rint_init=0.08)
    
    # 模拟测量数据
    dt = 1.0  # 1秒
    I_load = 1.5  # 1.5A负载
    
    true_soc = 0.95
    
    for t in range(3600):  # 1小时
        # 真实SOC变化
        true_soc -= I_load / (4000 / 1000) / 3600
        true_soc = max(0, true_soc)
        
        # 模拟测量
        V_true = 3.0 + 0.8 * true_soc + 0.3 * true_soc**2 - 0.1 * true_soc**3
        V_meas = V_true - I_load * 0.08 + np.random.randn() * 0.01
        I_meas = I_load + np.random.randn() * 0.01
        
        ekf.step(V_meas, I_meas, dt)
    
    soc_est, soc_std = ekf.get_soc_estimate()
    print(f"真实SOC: {true_soc:.4f}")
    print(f"估计SOC: {soc_est:.4f} ± {soc_std:.4f}")
    print(f"误差: {abs(true_soc - soc_est):.4f}")
    
    # 测试UKF
    print("\n测试无迹卡尔曼滤波器...")
    ukf = UnscentedKalmanFilter(battery_capacity_mah=4000)
    ukf.initialize(soc_init=0.95, temp_init=298.15, rint_init=0.08)
    
    true_soc = 0.95
    for t in range(3600):
        true_soc -= I_load / 4 / 3600
        true_soc = max(0, true_soc)
        
        V_true = 3.0 + 0.8 * true_soc + 0.3 * true_soc**2 - 0.1 * true_soc**3
        V_meas = V_true - I_load * 0.08 + np.random.randn() * 0.01
        I_meas = I_load + np.random.randn() * 0.01
        
        ukf.step(V_meas, I_meas, dt)
    
    soc_est, soc_std = ukf.get_soc_estimate()
    print(f"真实SOC: {true_soc:.4f}")
    print(f"估计SOC: {soc_est:.4f} ± {soc_std:.4f}")
    print(f"误差: {abs(true_soc - soc_est):.4f}")
