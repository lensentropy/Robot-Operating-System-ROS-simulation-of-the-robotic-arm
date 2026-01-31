import numpy as np
from battery_core import BatteryCore, ThermalModel
from network_5g_module import Network5GModule
from gnss_module import GNSSModule
from bluetooth_module import BluetoothModule
from background_tasks_module import BackgroundTasksModule

class CoupledBatterySystem:
    def __init__(self):
        self.batt = BatteryCore()
        self.therm = ThermalModel()
        self.net = Network5GModule()
        self.gnss = GNSSModule()
        self.bt = BluetoothModule()
        self.bg = BackgroundTasksModule()
        
    def simulate(self, scenario, S0, t_span, dt=0.01):
        t = np.arange(t_span[0], t_span[1], dt)
        n = len(t)
        
        # 初始化数组
        S = np.zeros(n); S[0] = S0
        T = np.zeros(n); T[0] = 298.15
        V = np.zeros(n)
        I_total = np.zeros(n)
        
        # 预生成后台电流
        I_bg_seq = self.bg.generate_path(t, seed=42)
        
        # 记录各部分功耗
        P_breakdown = {k: np.zeros(n) for k in ['Screen', 'CPU', 'Net', 'GPS', 'BT', 'BG']}
        
        end_idx = n  # 记录实际结束索引
        
        for i in range(n-1):
            # 1. 获取场景输入
            inp = scenario.get_input(t[i])
            
            # 2. 计算各模块功耗 (W)
            # 屏幕 (线性模型)
            p_scr = 0.0 + inp['bri'] * 2.0 
            
            # 5G
            p_net = self.net.total_power(inp['rate'], inp['dist'])
            
            # GPS
            p_gps = self.gnss.power_consumption(inp['snr'], inp['gps_on'])
            
            # 蓝牙
            p_bt = self.bt.total_power(**inp['bt_conf'])
            
            # 后台 (P = V * I)
            v_est = self.batt.open_circuit_voltage(S[i])
            p_bg = I_bg_seq[i] * v_est
            
            # CPU (耦合模型)
            # 基础 + 负载 + 网络开销 + GPS开销
            cpu_load = inp['cpu'] + (inp['rate']/100e6)*0.1 + (0.05 if inp['gps_on'] else 0)
            p_cpu = 0.1 + cpu_load * 3.5
            
            # 3. 总负载
            p_tot = p_scr + p_net + p_gps + p_bt + p_bg + p_cpu
            i_load = p_tot / v_est
            
            # 4. 状态更新 (Euler法，步长够小即可)
            # dS/dt
            eff_cap = self.batt.effective_capacity(i_load, T[i])
            dS = -i_load / eff_cap # /h
            S[i+1] = S[i] + dS * dt
            
            # dT/dt
            # 假设电子元件发热的80%传递给电池
            heat_electronics = p_tot * 0.8
            # 电池自身发热 I^2*R
            r_int = self.batt.internal_resistance(S[i], T[i])
            heat_joule = i_load**2 * r_int
            
            dT = self.therm.derivative(T[i], heat_electronics + heat_joule)
            T[i+1] = T[i] + dT * 3600 * dt # 转换为 /h
            
            # 记录数据
            I_total[i] = i_load
            V[i] = self.batt.terminal_voltage(S[i], i_load, T[i])
            
            P_breakdown['Screen'][i] = p_scr
            P_breakdown['CPU'][i] = p_cpu
            P_breakdown['Net'][i] = p_net
            P_breakdown['GPS'][i] = p_gps
            P_breakdown['BT'][i] = p_bt
            P_breakdown['BG'][i] = p_bg
            
            if S[i+1] <= 0.01:
                end_idx = i + 2  # 包含到电量耗尽的那个点
                break
                
        # 裁剪数组到有效数据长度
        t = t[:end_idx]
        S = S[:end_idx]
        T = T[:end_idx]
        V = V[:end_idx]
        I_total = I_total[:end_idx]
        for k in P_breakdown:
            P_breakdown[k] = P_breakdown[k][:end_idx]
        
        # 填充最后一个点
        if len(I_total) > 1:
            I_total[-1] = I_total[-2]
            V[-1] = V[-2]
        
        return {'time': t, 'SOC': S, 'voltage': V, 'current': I_total, 
                'temperature': T, 'breakdown': P_breakdown}

# --- 场景定义 ---
class Scenario:
    name = "Base"
    def get_input(self, t): return {}

class IdleScenario(Scenario):
    name = "Idle (Screen Off)"
    def get_input(self, t):
        return {'bri': 0, 'cpu': 0.01, 'rate': 0, 'dist': 100, 
                'snr': 0, 'gps_on': False, 'bt_conf': {'mode': 'idle'}}

class VideoStreamingScenario(Scenario):
    name = "4K Video (5G)"
    def get_input(self, t):
        return {'bri': 0.8, 'cpu': 0.4, 'rate': 25e6, 'dist': 300, 
                'snr': 0, 'gps_on': False, 'bt_conf': {'mode': 'audio'}}

class NavigationScenario(Scenario):
    name = "GPS Navigation"
    def get_input(self, t):
        # 模拟进入隧道 (SNR降低)
        in_tunnel = 2.0 < t < 2.5
        snr = 15 if in_tunnel else 40
        return {'bri': 0.7, 'cpu': 0.3, 'rate': 1e6, 'dist': 500, 
                'snr': snr, 'gps_on': True, 'bt_conf': {'mode': 'audio'}}

class GamingScenario(Scenario):
    name = "Heavy Gaming"
    def get_input(self, t):
        return {'bri': 0.9, 'cpu': 0.9, 'rate': 2e6, 'dist': 100, 
                'snr': 0, 'gps_on': False, 'bt_conf': {'mode': 'audio'}}

class MixedUsageScenario(Scenario):
    name = "Daily Mixed Use"
    def get_input(self, t):
        h = t % 24
        # 简单的时间表
        if 8 <= h < 9: return NavigationScenario().get_input(t) # 上班
        if 12 <= h < 13: return GamingScenario().get_input(t)   # 午休
        if 19 <= h < 21: return VideoStreamingScenario().get_input(t) # 晚上
        return IdleScenario().get_input(t) # 其他时间待机
