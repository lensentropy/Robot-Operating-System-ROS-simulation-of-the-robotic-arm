"""
智能手机功耗模拟模型 - Python实现
====================================
基于MCM 2026 Problem A的现代智能手机功耗仿真模型
包含电池、显示屏、SoC、连接性(5G/BT/GPS)等子系统
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple, List, Optional

@dataclass
class SimulationParams:
    """仿真参数"""
    T_hours: float = 24.0
    dt: float = 1.0  # seconds
    
    @property
    def steps(self) -> int:
        return int(self.T_hours * 3600 / self.dt)
    
    @property
    def t_axis_hours(self) -> np.ndarray:
        return np.arange(self.steps) * self.dt / 3600

@dataclass
class BatteryParams:
    """电池参数"""
    Q_design_mAh: float = 5000.0  # mAh
    V_nom: float = 3.85  # V
    R_internal_base: float = 0.05  # Ohm
    Temp_coeff_R: float = 0.02
    PMIC_eff: float = 0.92
    Max_Power_Limit: float = 8.0  # W
    
    @property
    def Q_design_C(self) -> float:
        return (self.Q_design_mAh / 1000) * 3600

@dataclass
class SoCParams:
    """片上系统参数"""
    C_eff: float = 1.2e-9
    V_min: float = 0.65
    V_max: float = 1.05
    I_leak_ref: float = 0.005
    Temp_ref: float = 298.15
    Temp_leak_sens: float = 3500

@dataclass
class DisplayParams:
    """显示屏参数"""
    P_static: float = 0.050  # W
    P_dyn_slope: float = 0.0005
    Beta_panel: float = 2.5e-3
    L_max: float = 1200  # nits

@dataclass
class ConnectivityParams:
    """连接性参数 (5G/WiFi/BT/GPS)"""
    Baseband_Idle_5G: float = 0.080  # W
    Baseband_Active_Low: float = 0.300  # W
    Baseband_Active_High: float = 1.200  # W
    WiFi_Idle: float = 0.020  # W
    WiFi_Active: float = 0.400  # W
    BT_Idle: float = 0.005  # W
    BT_Active_Audio: float = 0.040  # W
    GPS_Tracking: float = 0.150  # W

@dataclass
class ThermalParams:
    """热参数"""
    C_th: float = 850.0  # J/K
    R_th: float = 18.0  # K/W
    T_amb: float = 298.15  # K
    T_throttle: float = 316.15  # K

@dataclass
class PhysicsParams:
    """动态物理参数"""
    Inertia_Util: float = 0.008
    Wander_Step: float = 0.2
    Noise_Correlation: float = 0.70

@dataclass
class SystemParams:
    """系统参数"""
    P_board_leak: float = 0.050  # W

class SmartphonePowerModel:
    """
    智能手机功耗仿真模型
    
    模拟24小时内智能手机在不同使用状态下的功耗和电池消耗
    """
    
    # 使用状态
    STATE_SLEEP = 0
    STATE_LIGHT = 1
    STATE_STREAM = 2
    STATE_GAME = 3
    STATE_NAMES = ['Sleep', 'Light', 'Stream', 'Game']
    
    # 目标利用率和频率中心点
    TARGET_UTIL_CENTER = [1.5, 25.0, 40.0, 85.0]  # %
    TARGET_FREQ_CENTER = [0.3, 1.2, 1.6, 2.6]  # GHz
    
    def __init__(self, 
                 sim_params: SimulationParams = None,
                 batt_params: BatteryParams = None,
                 soc_params: SoCParams = None,
                 disp_params: DisplayParams = None,
                 conn_params: ConnectivityParams = None,
                 therm_params: ThermalParams = None,
                 physics_params: PhysicsParams = None,
                 sys_params: SystemParams = None,
                 random_seed: int = 2026):
        
        self.sim = sim_params or SimulationParams()
        self.batt = batt_params or BatteryParams()
        self.soc = soc_params or SoCParams()
        self.disp = disp_params or DisplayParams()
        self.conn = conn_params or ConnectivityParams()
        self.therm = therm_params or ThermalParams()
        self.physics = physics_params or PhysicsParams()
        self.sys = sys_params or SystemParams()
        self.random_seed = random_seed
        
        # Markov状态转移矩阵
        self._init_markov_matrices()
    
    def _init_markov_matrices(self):
        """初始化Markov状态转移矩阵"""
        # 睡眠时段
        self.P_sleep = np.array([
            [0.999, 0.001, 0.000, 0.000],
            [0.050, 0.950, 0.000, 0.000],
            [0.020, 0.000, 0.980, 0.000],
            [0.020, 0.000, 0.000, 0.980]
        ])
        
        # 工作时段
        self.P_work = np.array([
            [0.950, 0.050, 0.000, 0.000],
            [0.010, 0.980, 0.005, 0.005],
            [0.010, 0.050, 0.940, 0.000],
            [0.050, 0.100, 0.000, 0.850]
        ])
        
        # 休闲时段
        self.P_leisure = np.array([
            [0.900, 0.100, 0.000, 0.000],
            [0.010, 0.900, 0.050, 0.040],
            [0.005, 0.010, 0.980, 0.005],
            [0.005, 0.010, 0.005, 0.980]
        ])
    
    def run_simulation(self, fast_mode: bool = False) -> Dict[str, np.ndarray]:
        """
        运行完整的功耗仿真
        
        Parameters:
        -----------
        fast_mode : bool
            如果为True，使用更大的时间步长加速仿真
        
        Returns:
        --------
        Dict[str, np.ndarray]
            包含所有历史记录的字典
        """
        np.random.seed(self.random_seed)
        
        # 快速模式：增大时间步长
        dt = self.sim.dt if not fast_mode else 60.0
        steps = int(self.sim.T_hours * 3600 / dt)
        t_axis = np.arange(steps) * dt / 3600
        
        # 初始化历史记录
        hist = {
            'SOC': np.zeros(steps),
            'Power_Total': np.zeros(steps),
            'Power_SoC': np.zeros(steps),
            'Power_Disp': np.zeros(steps),
            'Power_Conn': np.zeros(steps),
            'Power_Base': np.zeros(steps),
            'Temp': np.zeros(steps),
            'State': np.zeros(steps, dtype=int),
            'CPU_Freq': np.zeros(steps),
            'CPU_Util': np.zeros(steps),
            'Disp_Bri': np.zeros(steps),
            'Disp_APL': np.zeros(steps),
            't_hours': t_axis
        }
        
        # 初始状态
        current_soc_c = self.batt.Q_design_C
        current_temp = self.therm.T_amb
        current_state = 0
        state_dwell = 0
        min_dwell = int(15 * 60 / dt)
        
        # Walker变量
        walker_util = 1.0
        walker_freq = 0.3
        walker_bri = 0.0
        walker_apl = 5.0
        noise_mem_freq = 0.0
        
        for t in range(steps):
            curr_hr = t_axis[t]
            
            # 3.1 Markov状态转移
            if curr_hr >= 23 or curr_hr < 7:
                P_curr = self.P_sleep
                mode_min_dwell = int(60 * 60 / dt)
            elif (9 <= curr_hr < 12) or (14 <= curr_hr < 18):
                P_curr = self.P_work
                mode_min_dwell = int(10 * 60 / dt)
            else:
                P_curr = self.P_leisure
                mode_min_dwell = int(20 * 60 / dt)
            
            state_dwell += 1
            if t % max(1, int(60 / dt)) == 0 and state_dwell > mode_min_dwell:
                probs = P_curr[current_state, :]
                r = np.random.rand()
                next_s = np.searchsorted(np.cumsum(probs), r)
                if next_s != current_state:
                    current_state = next_s
                    state_dwell = 0
            
            hist['State'][t] = current_state
            s_idx = current_state
            
            # 3.2 有机生成
            # A. 利用率
            target_u = self.TARGET_UTIL_CENTER[s_idx]
            attraction = (target_u - walker_util) * self.physics.Inertia_Util
            wander = 0.1 * np.random.randn() if s_idx == 0 else self.physics.Wander_Step * np.random.randn()
            walker_util = np.clip(walker_util + attraction + wander, 0, 100)
            inst_util = np.clip(walker_util + 1.5 * np.random.randn(), 0.1, 100)
            
            # B. 频率
            f_min, f_max = 0.3, 3.2
            freq_demand = (walker_util / 100) ** 0.7
            target_f = f_min + (f_max - f_min) * freq_demand
            
            if s_idx == 0:
                inst_freq = 0.3
                noise_mem_freq = 0
            else:
                new_noise = 0.25 * np.random.randn()
                noise_mem_freq = self.physics.Noise_Correlation * noise_mem_freq + \
                                 (1 - self.physics.Noise_Correlation) * new_noise
                inst_freq = np.clip(np.round((target_f + noise_mem_freq) * 20) / 20, 0.2, 3.2)
            
            # C. 显示
            if s_idx == 0:
                walker_bri = 0
                walker_apl = 0
            else:
                sun = np.exp(-((curr_hr - 13) ** 2) / 18)
                tgt_bri = 100 + 800 * sun
                walker_bri = walker_bri + 0.005 * (tgt_bri - walker_bri) + 0.5 * np.random.randn()
                tgt_apl = 50
                if s_idx == 1:
                    tgt_apl = 85
                elif s_idx == 2:
                    tgt_apl = 40
                walker_apl = walker_apl + 0.05 * (tgt_apl - walker_apl) + 2.0 * np.random.randn()
            
            inst_bri = np.clip(walker_bri, 0, 1200)
            inst_apl = np.clip(walker_apl, 0, 100)
            
            if s_idx == 3:
                tgt_ref = 120
            elif s_idx == 0:
                tgt_ref = 1
            else:
                tgt_ref = 60
            
            # 3.3 功耗计算
            # 温度节流
            if current_temp > self.therm.T_throttle:
                throttle = max(0.5, 1.0 - (current_temp - self.therm.T_throttle) * 0.15)
                inst_freq *= throttle
                inst_bri *= throttle
            
            # 1. 显示功耗
            P_d_dyn = self.disp.P_dyn_slope * tgt_ref
            P_d_emit = self.disp.Beta_panel * inst_bri * (inst_apl / 100)
            P_disp = self.disp.P_static + P_d_dyn + P_d_emit
            
            # 2. SoC功耗
            V_dd = self.soc.V_min + (self.soc.V_max - self.soc.V_min) * (inst_freq / 3.0)
            P_s_dyn = self.soc.C_eff * (inst_freq * 1e9) * V_dd ** 2 * (inst_util / 100)
            I_leak = self.soc.I_leak_ref * (current_temp / self.soc.Temp_ref) ** 2 * \
                     np.exp(self.soc.Temp_leak_sens * (1 / self.soc.Temp_ref - 1 / current_temp))
            P_s_leak = V_dd * I_leak
            P_soc = P_s_dyn + P_s_leak
            
            # 3. 连接功耗
            P_conn = self._compute_connectivity_power(s_idx)
            
            # 4. 基础功耗
            P_base = self.sys.P_board_leak
            
            # 总负载
            P_load = min(P_disp + P_soc + P_conn + P_base, self.batt.Max_Power_Limit)
            
            # 3.4 电气计算
            curr_soc_p = max(0.001, current_soc_c / self.batt.Q_design_C)
            V_ocv = 3.0 + 1.0 * curr_soc_p - 0.4 * np.exp(-15 * curr_soc_p) + 0.3 * curr_soc_p ** 2
            R_int = self.batt.R_internal_base * (1 + 0.5 * np.exp(-10 * curr_soc_p)) * \
                    (1 + self.batt.Temp_coeff_R * (298.15 - current_temp))
            
            P_req = P_load / self.batt.PMIC_eff
            delta = V_ocv ** 2 - 4 * R_int * P_req
            if delta < 0:
                I_batt = V_ocv / (2 * R_int)
            else:
                I_batt = (V_ocv - np.sqrt(delta)) / (2 * R_int)
            
            # 充电（晚间）
            if curr_hr < 7:
                current_soc_c = self.batt.Q_design_C
                I_batt = 0
                P_req = 0
                P_disp = 0
                P_soc = 0.01
                P_conn = 0.05
                P_base = 0.01
            else:
                current_soc_c -= I_batt * dt
            
            if current_soc_c <= 0:
                current_soc_c = self.batt.Q_design_C
            
            # 热计算
            Heat = P_soc + I_batt ** 2 * R_int + 0.5 * P_disp + P_conn
            dT = (Heat - (current_temp - self.therm.T_amb) / self.therm.R_th) / self.therm.C_th * dt
            current_temp += dT
            
            # 记录
            hist['SOC'][t] = current_soc_c / self.batt.Q_design_C * 100
            hist['Power_Total'][t] = P_req
            hist['Power_SoC'][t] = P_soc
            hist['Power_Disp'][t] = P_disp
            hist['Power_Conn'][t] = P_conn
            hist['Power_Base'][t] = P_base
            hist['Temp'][t] = current_temp - 273.15
            hist['CPU_Freq'][t] = inst_freq
            hist['CPU_Util'][t] = inst_util
            hist['Disp_Bri'][t] = inst_bri
            hist['Disp_APL'][t] = inst_apl
        
        return hist
    
    def _compute_connectivity_power(self, state_idx: int) -> float:
        """计算连接功耗"""
        is_notification = np.random.rand() > 0.99
        
        if state_idx == 0:  # Sleep
            P_conn = self.conn.Baseband_Idle_5G + self.conn.WiFi_Idle
            if is_notification:
                P_conn += self.conn.Baseband_Active_Low
        elif state_idx == 1:  # Light
            P_conn = self.conn.WiFi_Active * 0.5 + self.conn.Baseband_Idle_5G
        elif state_idx == 2:  # Stream
            P_conn = self.conn.WiFi_Active + self.conn.BT_Active_Audio
        else:  # Game
            if np.random.rand() > 0.5:
                P_conn = self.conn.WiFi_Active + self.conn.Baseband_Idle_5G
            else:
                P_conn = self.conn.Baseband_Active_High + self.conn.WiFi_Idle
            P_conn += self.conn.BT_Active_Audio
            if np.random.rand() > 0.8:
                P_conn += self.conn.GPS_Tracking
        
        # 添加抖动
        P_conn *= (0.9 + 0.2 * np.random.rand())
        return P_conn
    
    def compute_metrics(self, hist: Dict[str, np.ndarray]) -> Dict[str, float]:
        """
        从仿真历史计算关键指标
        
        Returns:
        --------
        Dict[str, float]
            包含关键指标的字典
        """
        # 过滤非充电时段 (7:00之后)
        mask = hist['t_hours'] >= 7
        
        metrics = {
            # 电池指标
            'final_soc': hist['SOC'][-1],
            'min_soc': np.min(hist['SOC'][mask]) if np.any(mask) else hist['SOC'][-1],
            'soc_drop': hist['SOC'][mask][0] - hist['SOC'][-1] if np.any(mask) else 0,
            
            # 功耗指标
            'avg_power': np.mean(hist['Power_Total'][mask]) * 1000 if np.any(mask) else 0,  # mW
            'max_power': np.max(hist['Power_Total'][mask]) * 1000 if np.any(mask) else 0,  # mW
            'total_energy': np.sum(hist['Power_Total'][mask]) * (self.sim.dt / 3600) if np.any(mask) else 0,  # Wh
            
            # 各组件功耗
            'avg_power_soc': np.mean(hist['Power_SoC'][mask]) * 1000 if np.any(mask) else 0,
            'avg_power_disp': np.mean(hist['Power_Disp'][mask]) * 1000 if np.any(mask) else 0,
            'avg_power_conn': np.mean(hist['Power_Conn'][mask]) * 1000 if np.any(mask) else 0,
            
            # 热指标
            'avg_temp': np.mean(hist['Temp'][mask]) if np.any(mask) else 25,
            'max_temp': np.max(hist['Temp'][mask]) if np.any(mask) else 25,
            
            # 性能指标
            'avg_cpu_freq': np.mean(hist['CPU_Freq'][mask]) if np.any(mask) else 0.3,
            'avg_cpu_util': np.mean(hist['CPU_Util'][mask]) if np.any(mask) else 1.0,
        }
        
        return metrics


def create_model_with_params(params_dict: Dict[str, float]) -> SmartphonePowerModel:
    """
    使用参数字典创建模型
    
    Parameters:
    -----------
    params_dict : Dict[str, float]
        参数名到值的映射
    
    Returns:
    --------
    SmartphonePowerModel
        配置好的模型实例
    """
    batt = BatteryParams(
        Q_design_mAh=params_dict.get('batt_capacity', 5000),
        R_internal_base=params_dict.get('batt_R_internal', 0.05),
        PMIC_eff=params_dict.get('batt_PMIC_eff', 0.92),
    )
    
    soc = SoCParams(
        C_eff=params_dict.get('soc_C_eff', 1.2e-9),
        V_min=params_dict.get('soc_V_min', 0.65),
        V_max=params_dict.get('soc_V_max', 1.05),
        I_leak_ref=params_dict.get('soc_I_leak', 0.005),
    )
    
    disp = DisplayParams(
        P_static=params_dict.get('disp_P_static', 0.050),
        Beta_panel=params_dict.get('disp_Beta', 2.5e-3),
    )
    
    conn = ConnectivityParams(
        Baseband_Idle_5G=params_dict.get('conn_5G_idle', 0.080),
        Baseband_Active_High=params_dict.get('conn_5G_active', 1.200),
        WiFi_Active=params_dict.get('conn_WiFi_active', 0.400),
        BT_Active_Audio=params_dict.get('conn_BT_audio', 0.040),
        GPS_Tracking=params_dict.get('conn_GPS', 0.150),
    )
    
    therm = ThermalParams(
        C_th=params_dict.get('therm_C', 850),
        R_th=params_dict.get('therm_R', 18),
        T_amb=params_dict.get('therm_T_amb', 298.15),
    )
    
    return SmartphonePowerModel(
        batt_params=batt,
        soc_params=soc,
        disp_params=disp,
        conn_params=conn,
        therm_params=therm,
    )


# 默认参数及其范围定义
DEFAULT_PARAMS = {
    'batt_capacity': {'default': 5000, 'min': 3000, 'max': 7000, 'unit': 'mAh', 'desc': '电池容量'},
    'batt_R_internal': {'default': 0.05, 'min': 0.03, 'max': 0.10, 'unit': 'Ohm', 'desc': '内阻'},
    'batt_PMIC_eff': {'default': 0.92, 'min': 0.85, 'max': 0.98, 'unit': '', 'desc': 'PMIC效率'},
    'soc_C_eff': {'default': 1.2e-9, 'min': 0.8e-9, 'max': 2.0e-9, 'unit': 'F', 'desc': 'SoC等效电容'},
    'soc_V_min': {'default': 0.65, 'min': 0.5, 'max': 0.8, 'unit': 'V', 'desc': 'SoC最低电压'},
    'soc_V_max': {'default': 1.05, 'min': 0.9, 'max': 1.2, 'unit': 'V', 'desc': 'SoC最高电压'},
    'soc_I_leak': {'default': 0.005, 'min': 0.002, 'max': 0.01, 'unit': 'A', 'desc': '漏电流'},
    'disp_P_static': {'default': 0.050, 'min': 0.03, 'max': 0.10, 'unit': 'W', 'desc': '显示静态功耗'},
    'disp_Beta': {'default': 2.5e-3, 'min': 1.5e-3, 'max': 4.0e-3, 'unit': 'W/nit', 'desc': '面板系数'},
    'conn_5G_idle': {'default': 0.080, 'min': 0.05, 'max': 0.15, 'unit': 'W', 'desc': '5G待机功耗'},
    'conn_5G_active': {'default': 1.200, 'min': 0.8, 'max': 2.0, 'unit': 'W', 'desc': '5G活动功耗'},
    'conn_WiFi_active': {'default': 0.400, 'min': 0.2, 'max': 0.6, 'unit': 'W', 'desc': 'WiFi活动功耗'},
    'conn_BT_audio': {'default': 0.040, 'min': 0.02, 'max': 0.08, 'unit': 'W', 'desc': '蓝牙音频功耗'},
    'conn_GPS': {'default': 0.150, 'min': 0.08, 'max': 0.25, 'unit': 'W', 'desc': 'GPS功耗'},
    'therm_C': {'default': 850, 'min': 500, 'max': 1200, 'unit': 'J/K', 'desc': '热容'},
    'therm_R': {'default': 18, 'min': 10, 'max': 30, 'unit': 'K/W', 'desc': '热阻'},
    'therm_T_amb': {'default': 298.15, 'min': 288.15, 'max': 313.15, 'unit': 'K', 'desc': '环境温度'},
}


if __name__ == "__main__":
    # 测试模型
    print("创建模型...")
    model = SmartphonePowerModel()
    
    print("运行仿真...")
    hist = model.run_simulation(fast_mode=True)
    
    print("计算指标...")
    metrics = model.compute_metrics(hist)
    
    print("\n=== 仿真结果 ===")
    for key, value in metrics.items():
        print(f"{key}: {value:.4f}")
