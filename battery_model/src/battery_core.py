import numpy as np
from dataclasses import dataclass

@dataclass
class BatteryParameters:
    C_nom: float = 4.5       # 标称容量 (Ah)
    V_nom: float = 3.85      # 标称电压 (V)
    R_internal: float = 0.08 # 内阻 (Ohm)
    k_peukert: float = 1.05  # Peukert系数 (1.0-1.3)
    I_ref: float = 0.9       # Peukert参考电流 (A)
    T_ref: float = 298.15    # 参考温度 (K)
    E_a: float = 20000       # 活化能 (J/mol)
    eta: float = 0.995       # 库仑效率

class BatteryCore:
    def __init__(self, params: BatteryParameters = None):
        self.p = params or BatteryParameters()

    def effective_capacity(self, I: float, T: float) -> float:
        """Peukert效应 + Arrhenius温度修正"""
        # Peukert修正
        if I > self.p.I_ref:
            f_peukert = (self.p.I_ref / I) ** (self.p.k_peukert - 1)
        else:
            f_peukert = 1.0
            
        # 温度修正 (Arrhenius)
        R_gas = 8.314
        f_temp = np.exp(-(self.p.E_a / R_gas) * (1/T - 1/self.p.T_ref))
        f_temp = np.clip(f_temp, 0.6, 1.1) # 物理限制
        
        return self.p.C_nom * f_peukert * f_temp

    def open_circuit_voltage(self, S: float) -> float:
        """非线性 OCV-SOC 曲线 (5阶多项式拟合)"""
        s = np.clip(S, 0, 1)
        # 基于典型 LCO/石墨 电池数据
        return 3.0 + 1.2*s - 0.3*s**2 + 0.15*s**3 + 0.15*s**4

    def internal_resistance(self, S: float, T: float) -> float:
        """内阻随低SOC和低温增加"""
        s = np.clip(S, 0.01, 1)
        r_soc = 1 + 0.5 * (1-s)**2
        r_temp = np.exp(0.05 * (self.p.T_ref - T))
        return self.p.R_internal * r_soc * r_temp

    def terminal_voltage(self, S, I, T):
        return self.open_circuit_voltage(S) - I * self.internal_resistance(S, T)

class ThermalModel:
    def __init__(self):
        self.m = 0.045      # 质量 (kg)
        self.cp = 1000      # 比热容 (J/kgK)
        self.h = 8.0        # 对流换热系数 (W/m2K)
        self.A = 0.012      # 散热面积 (m2)
        
    def derivative(self, T, P_heat, T_amb=298.15):
        # dT/dt = (Q_gen - Q_diss) / (m*Cp)
        Q_diss = self.h * self.A * (T - T_amb)
        return (P_heat - Q_diss) / (self.m * self.cp)
