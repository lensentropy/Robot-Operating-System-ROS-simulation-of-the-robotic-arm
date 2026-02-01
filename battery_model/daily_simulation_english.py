#!/usr/bin/env python3
"""
24-Hour Smartphone Battery Discharge Simulation (English Version)
==================================================================
Coupled Electro-Thermal-Aging Model with Realistic Usage Scenarios

This simulation models battery behavior across different daily activities:
- Sleep (minimal background activity)
- Commute (navigation + audio streaming)
- Office (email, documents, browsing)
- Entertainment (video streaming, gaming)
- Charging (CC-CV charging protocol)

Output variables:
- SOC: State of Charge [%]
- I: Load current [A]
- T: Battery temperature [°C]
- L: Screen brightness [%]
- S: Signal strength [dBm]
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
import warnings
warnings.filterwarnings('ignore')

# Set professional plot style
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.dpi'] = 150

# =============================================================================
# Physical Parameters
# =============================================================================

class BatteryParameters:
    """Battery physical parameters based on NASA PCoE and Panasonic NCR18650B"""
    # Nominal specifications
    Q_nom = 4.0          # Nominal capacity [Ah]
    V_nom = 3.85         # Nominal voltage [V]
    V_max = 4.35         # Maximum voltage [V]
    V_min = 3.0          # Cutoff voltage [V]
    
    # Aging parameters (double-exponential model)
    a_Q, b_Q = -0.15, 0.025
    c_Q, d_Q = 4.15, 0.0008
    
    # Impedance parameters (power-law model)
    a_R, b_R, c_R = 0.008, 0.22, 0.025
    
    # Temperature correction parameters
    S_Q_max, k_Q, T_0 = 1.02, 0.1, -12.0
    C_R, A_R, B_R = 0.75, 1.2, 0.055
    
    # OCV parameters (Nernst-based)
    K = [3.45, 0.18, -0.005, 0.06, -0.09]
    
    # RC circuit parameters
    R0_ratio, R1_ratio, R2_ratio = 0.35, 0.40, 0.25
    tau_1, tau_2 = 25.0, 250.0
    
    # Thermal parameters
    C_c, C_s = 50.0, 12.0
    R_cs, R_se = 2.5, 20.0
    dV_dT = 0.0004
    
    # Charging parameters
    I_charge_max = 3.0
    V_charge = 4.35


# =============================================================================
# Activity Scenarios (English)
# =============================================================================

class ActivityScenarios:
    """Activity scenario definitions with power profiles"""
    
    @staticmethod
    def sleep():
        """Sleep mode - minimal power consumption"""
        return {
            'name': 'Sleep',
            'color': '#E3F2FD',
            '5g_active': True, '5g_data_rate': 50e3, '5g_distance': 400,
            'bt_active': True, 'bt_streaming': False, 'bt_interval': 2.0,
            'bg_wake_rate': 0.3, 'bg_activity': 0.2,
            'gnss_active': False, 'gnss_environment': 'indoor', 'gnss_satellites': 0,
            'display_on': False, 'display_brightness': 0.0, 
            'display_refresh': 1, 'display_content': 'dark',
            'cpu_load': 0.02,
            'T_env': 22.0, 'signal_strength': -75,
        }
    
    @staticmethod
    def commute():
        """Commute mode - navigation + audio"""
        return {
            'name': 'Commute',
            'color': '#FFF8E1',
            '5g_active': True, '5g_data_rate': 8e6, '5g_distance': 800,
            'bt_active': True, 'bt_streaming': True, 'bt_interval': 0.02,
            'bg_wake_rate': 1.5, 'bg_activity': 0.6,
            'gnss_active': True, 'gnss_environment': 'urban', 'gnss_satellites': 6,
            'display_on': True, 'display_brightness': 0.7, 
            'display_refresh': 60, 'display_content': 'mixed',
            'cpu_load': 0.45,
            'T_env': 28.0, 'signal_strength': -85,
        }
    
    @staticmethod
    def office():
        """Office mode - email, documents"""
        return {
            'name': 'Office',
            'color': '#E8F5E9',
            '5g_active': True, '5g_data_rate': 3e6, '5g_distance': 300,
            'bt_active': True, 'bt_streaming': False, 'bt_interval': 0.5,
            'bg_wake_rate': 2.0, 'bg_activity': 0.7,
            'gnss_active': False, 'gnss_environment': 'indoor', 'gnss_satellites': 0,
            'display_on': True, 'display_brightness': 0.5, 
            'display_refresh': 60, 'display_content': 'text',
            'cpu_load': 0.25,
            'T_env': 24.0, 'signal_strength': -65,
        }
    
    @staticmethod
    def entertainment():
        """Entertainment mode - video/gaming"""
        return {
            'name': 'Entertainment',
            'color': '#FCE4EC',
            '5g_active': True, '5g_data_rate': 25e6, '5g_distance': 400,
            'bt_active': True, 'bt_streaming': True, 'bt_interval': 0.05,
            'bg_wake_rate': 1.0, 'bg_activity': 0.4,
            'gnss_active': False, 'gnss_environment': 'indoor', 'gnss_satellites': 0,
            'display_on': True, 'display_brightness': 0.85, 
            'display_refresh': 90, 'display_content': 'video',
            'cpu_load': 0.65,
            'T_env': 25.0, 'signal_strength': -70,
        }
    
    @staticmethod
    def charging():
        """Charging state"""
        return {
            'name': 'Charging',
            'color': '#C8E6C9',
            'is_charging': True,
            '5g_active': True, '5g_data_rate': 1e6, '5g_distance': 300,
            'bt_active': True, 'bt_streaming': False, 'bt_interval': 1.0,
            'bg_wake_rate': 1.0, 'bg_activity': 0.5,
            'gnss_active': False, 'gnss_environment': 'indoor', 'gnss_satellites': 0,
            'display_on': False, 'display_brightness': 0.0, 
            'display_refresh': 60, 'display_content': 'dark',
            'cpu_load': 0.1,
            'T_env': 25.0, 'signal_strength': -65,
        }


def create_daily_schedule():
    """Create 24-hour activity schedule"""
    return [
        (0.0, 2.0, ActivityScenarios.sleep),
        (2.0, 2.5, ActivityScenarios.charging),
        (2.5, 7.0, ActivityScenarios.sleep),
        (7.0, 8.0, ActivityScenarios.commute),
        (8.0, 10.0, ActivityScenarios.office),
        (10.0, 11.0, ActivityScenarios.entertainment),
        (11.0, 12.5, ActivityScenarios.office),
        (12.5, 13.0, ActivityScenarios.charging),
        (13.0, 14.0, ActivityScenarios.entertainment),
        (14.0, 17.0, ActivityScenarios.office),
        (17.0, 18.0, ActivityScenarios.commute),
        (18.0, 20.0, ActivityScenarios.entertainment),
        (20.0, 21.0, ActivityScenarios.office),
        (21.0, 22.0, ActivityScenarios.entertainment),
        (22.0, 22.5, ActivityScenarios.charging),
        (22.5, 24.0, ActivityScenarios.sleep),
    ]


# =============================================================================
# Coupled Battery Model
# =============================================================================

class CoupledBatteryModel:
    """
    Coupled Electro-Thermal-Aging Battery Model
    
    State vector: x = [z, V1, V2, Tc, Ts]
    """
    
    def __init__(self, cycle_number=150):
        self.params = BatteryParameters()
        self.cycle_number = cycle_number
        self._update_aging_params()
    
    def _update_aging_params(self):
        N = self.cycle_number
        p = self.params
        self.Q_max_base = p.a_Q * np.exp(-p.b_Q * N) + p.c_Q * np.exp(-p.d_Q * N)
        self.R_total_base = p.a_R * np.power(max(N, 1), p.b_R) + p.c_R
    
    def capacity_temp_factor(self, T):
        p = self.params
        return p.S_Q_max / (1 + np.exp(-p.k_Q * (T - p.T_0)))
    
    def resistance_temp_factor(self, T):
        p = self.params
        return p.C_R + p.A_R * np.exp(-p.B_R * T)
    
    def get_capacity(self, Tc):
        return self.Q_max_base * self.capacity_temp_factor(Tc)
    
    def get_resistance(self, Tc):
        return self.R_total_base * self.resistance_temp_factor(Tc)
    
    def ocv(self, z):
        z = np.clip(z, 1e-6, 1 - 1e-6)
        K = self.params.K
        return K[0] + K[1]*z + K[2]/z + K[3]*np.log(z) + K[4]*np.log(1-z)
    
    def load_power(self, profile, V_bat, Tc):
        p = profile
        
        # 5G power
        rate_ratio = np.clip(p['5g_data_rate'] / 100e6, 0, 8)
        snr = 2**rate_ratio - 1
        P_tx = np.clip(1e-9 * (p['5g_distance'] ** 3.2) * snr, 0, 2.0)
        P_5g = 0.15 + P_tx / 0.35 if p['5g_active'] else 0.001
        
        # Bluetooth power
        if p['bt_streaming']:
            P_bt = 0.035
        elif p['bt_active']:
            P_bt = V_bat * (5e-6 + 50e-6 / p['bt_interval'])
        else:
            P_bt = 0
        
        # Background power
        duty = np.clip(p['bg_wake_rate'] * 10.5 / 60, 0, 1)
        P_bg = 0.005 + duty * 0.08 + (1 - duty) * 0.015
        
        # GNSS power
        if p['gnss_active']:
            lock_prob = 1 / (1 + np.exp(-10 * (p['gnss_satellites']/12 * 0.7 - 0.3)))
            P_gnss = lock_prob * 0.04 + (1 - lock_prob) * 0.15 + 0.01
        else:
            P_gnss = 0
        
        # Display power
        if p['display_on']:
            apl_map = {'dark': 0.1, 'text': 0.25, 'mixed': 0.4, 'video': 0.55}
            apl = apl_map.get(p['display_content'], 0.4)
            P_disp = 0.08 + 0.004 * p['display_refresh'] + 2.2 * (p['display_brightness'] ** 1.4) * apl
        else:
            P_disp = 0.008
        
        # SoC power
        f = 3e8 + (3e9 - 3e8) * p['cpu_load']
        P_dyn = 1.2e-28 * (f ** 3)
        T_k = Tc + 273.15
        I_leak = np.clip(0.004 * (T_k / 298) ** 2 * np.exp(0.015 * (Tc - 25)), 0, 0.3)
        P_soc = P_dyn + 1.0 * I_leak
        
        return P_5g + P_bt + P_bg + P_gnss + P_disp + P_soc, {
            '5G': P_5g, 'Bluetooth': P_bt, 'Background': P_bg,
            'GNSS': P_gnss, 'Display': P_disp, 'SoC': P_soc
        }
    
    def state_derivatives(self, t, state, profile):
        z, V1, V2, Tc, Ts = state
        z = np.clip(z, 0.01, 0.99)
        p = self.params
        
        Q_max = self.get_capacity(Tc)
        R_total = self.get_resistance(Tc)
        R0, R1, R2 = R_total * p.R0_ratio, R_total * p.R1_ratio, R_total * p.R2_ratio
        C1, C2 = p.tau_1 / R1 if R1 > 0 else 1, p.tau_2 / R2 if R2 > 0 else 1
        
        V_ocv = self.ocv(z)
        is_charging = profile.get('is_charging', False)
        
        if is_charging and z < 0.95:
            I_charge = max(min(p.I_charge_max, (p.V_charge - V_ocv) / R_total), 0.1)
            I = -I_charge
        else:
            V_term_est = V_ocv - V1 - V2
            P_load, _ = self.load_power(profile, V_term_est, Tc)
            I = P_load / max(V_term_est, 3.0)
        
        eta = 0.85 + 0.148 / (1 + np.exp(-0.1 * (Tc + 5)))
        
        dz_dt = -I * eta / (Q_max * 3600)
        dV1_dt = -V1 / (R1 * C1) + I / C1
        dV2_dt = -V2 / (R2 * C2) + I / C2
        
        Q_gen = I**2 * R_total + abs(I) * (Tc + 273.15) * p.dV_dT
        T_env = profile.get('T_env', 25.0)
        dTc_dt = (Q_gen - (Tc - Ts) / p.R_cs) / p.C_c
        dTs_dt = ((Tc - Ts) / p.R_cs - (Ts - T_env) / p.R_se) / p.C_s
        
        return [dz_dt, dV1_dt, dV2_dt, dTc_dt, dTs_dt]
    
    def terminal_voltage(self, state, profile):
        z, V1, V2, Tc, Ts = state
        z = np.clip(z, 0.01, 0.99)
        R_total = self.get_resistance(Tc)
        R0 = R_total * self.params.R0_ratio
        V_ocv = self.ocv(z)
        
        is_charging = profile.get('is_charging', False)
        if is_charging and z < 0.95:
            I = -min(self.params.I_charge_max, (self.params.V_charge - V_ocv) / R_total)
        else:
            V_est = V_ocv - V1 - V2
            P_load, _ = self.load_power(profile, V_est, Tc)
            I = P_load / max(V_est, 3.0)
        
        return V_ocv - V1 - V2 - I * R0, I


def run_daily_simulation(model, schedule, dt=30.0):
    """Run 24-hour simulation"""
    state = [0.85, 0.0, 0.0, 22.0, 22.0]
    
    results = {k: [] for k in ['t', 'soc', 'current', 'voltage', 'temperature', 
                                'brightness', 'signal', 'power', 'scenarios']}
    
    t = 0
    while t < 24 * 3600:
        t_hours = t / 3600
        current_scenario = None
        for start_h, end_h, scenario_func in schedule:
            if start_h <= t_hours < end_h:
                current_scenario = scenario_func()
                break
        if current_scenario is None:
            current_scenario = ActivityScenarios.sleep()
        
        dstate = model.state_derivatives(t, state, current_scenario)
        state = [np.clip(state[i] + dstate[i] * dt, 0.01, 0.99) if i == 0 
                 else state[i] + dstate[i] * dt for i in range(5)]
        
        V_term, I = model.terminal_voltage(state, current_scenario)
        P_load, _ = model.load_power(current_scenario, V_term, state[3])
        
        results['t'].append(t / 3600)
        results['soc'].append(state[0] * 100)
        results['current'].append(I)
        results['voltage'].append(V_term)
        results['temperature'].append(state[3])
        results['brightness'].append(current_scenario['display_brightness'] * 100)
        results['signal'].append(current_scenario['signal_strength'])
        results['power'].append(P_load)
        results['scenarios'].append(current_scenario)
        
        t += dt
    
    return {k: np.array(v) if k != 'scenarios' else v for k, v in results.items()}


def plot_daily_results_english(results, schedule, save_path=None):
    """Plot 24-hour simulation results - Professional English version"""
    fig, ax1 = plt.subplots(figsize=(14, 7))
    
    t = results['t']
    
    # Plot activity region backgrounds
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        ax1.axvspan(start_h, end_h, alpha=0.35, color=scenario['color'], zorder=0)
    
    # Add activity labels at top
    labeled = set()
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        mid = (start_h + end_h) / 2
        name = scenario['name']
        if name not in labeled or end_h - start_h > 2:
            ax1.text(mid, 103, name, ha='center', va='bottom', fontsize=9, fontweight='bold')
            labeled.add(name)
    
    # Primary Y-axis: SOC and Current
    line_soc, = ax1.plot(t, results['soc'], 'darkorange', linewidth=2.5, 
                         label='SOC [%]', zorder=5)
    
    current_scaled = np.abs(results['current']) * 30
    line_current, = ax1.plot(t, current_scaled, 'steelblue', linewidth=1.2, 
                             alpha=0.8, label='Current I [A×30]', zorder=4)
    
    ax1.set_xlabel('Time (hours)', fontsize=12)
    ax1.set_ylabel('SOC (%) / Current (A)', fontsize=12, color='darkorange')
    ax1.tick_params(axis='y', labelcolor='darkorange')
    ax1.set_xlim([0, 24])
    ax1.set_ylim([0, 108])
    ax1.set_xticks(np.arange(0, 25, 3))
    ax1.set_xticklabels([f'{int(h)}:00' for h in np.arange(0, 25, 3)])
    ax1.grid(True, alpha=0.2, linestyle='--')
    
    # Secondary Y-axis: Temperature, Brightness, Signal
    ax2 = ax1.twinx()
    
    line_temp, = ax2.plot(t, results['temperature'], 'purple', 
                         linewidth=1.5, alpha=0.9, label='Temperature T [°C]', zorder=3)
    line_bright, = ax2.plot(t, results['brightness'], 'goldenrod', 
                           linewidth=1.2, alpha=0.8, label='Brightness [%]', zorder=3)
    signal_norm = (results['signal'] + 100) * 1.2
    line_signal, = ax2.plot(t, signal_norm, 'forestgreen', 
                           linewidth=1.0, alpha=0.7, label='Signal [dBm]', zorder=2)
    
    ax2.set_ylabel('Temperature (°C) / Brightness (%) / Signal [dBm]', fontsize=11)
    ax2.set_ylim([0, 120])
    
    # Mark charging periods
    for start_h, end_h, scenario_func in schedule:
        scenario = scenario_func()
        if scenario.get('is_charging', False) or scenario['name'] == 'Charging':
            mid = (start_h + end_h) / 2
            ax1.annotate('Charging', xy=(mid, 18), fontsize=8, ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.8))
    
    # Legend
    lines = [line_soc, line_current, line_temp, line_bright, line_signal]
    labels = ['SOC [%]', 'Current I [A×30]', 'Temperature T [°C]', 
              'Brightness [%]', 'Signal Strength [dBm]']
    ax1.legend(lines, labels, loc='upper right', fontsize=9, framealpha=0.9, ncol=2)
    
    plt.title('24-Hour Smartphone Battery Coupled Model Simulation', 
              fontsize=14, fontweight='bold', pad=15)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Figure saved: {save_path}')
    
    return fig


def plot_comprehensive_analysis(results, model, save_path=None):
    """Generate comprehensive analysis figure with multiple panels"""
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    t = results['t']
    
    # Panel (a): SOC and Voltage
    ax1 = fig.add_subplot(gs[0, 0])
    ax1_twin = ax1.twinx()
    l1, = ax1.plot(t, results['soc'], 'b-', linewidth=2, label='SOC')
    l2, = ax1_twin.plot(t, results['voltage'], 'r-', linewidth=2, label='Voltage')
    ax1.set_xlabel('Time [hours]')
    ax1.set_ylabel('SOC [%]', color='blue')
    ax1_twin.set_ylabel('Voltage [V]', color='red')
    ax1.set_title('(a) State of Charge & Terminal Voltage')
    ax1.legend(handles=[l1, l2], loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 24])
    
    # Panel (b): Temperature dynamics
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, results['temperature'], 'r-', linewidth=2, label='Core Temp')
    ax2.fill_between(t, 20, results['temperature'], alpha=0.3, color='red')
    ax2.set_xlabel('Time [hours]')
    ax2.set_ylabel('Temperature [°C]')
    ax2.set_title('(b) Battery Thermal Dynamics')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 24])
    
    # Panel (c): Current profile
    ax3 = fig.add_subplot(gs[0, 2])
    colors = ['green' if i < 0 else 'red' for i in results['current']]
    ax3.fill_between(t, 0, results['current']*1000, 
                     where=results['current'] >= 0, color='coral', alpha=0.7, label='Discharge')
    ax3.fill_between(t, 0, results['current']*1000, 
                     where=results['current'] < 0, color='green', alpha=0.7, label='Charge')
    ax3.axhline(y=0, color='black', linewidth=0.5)
    ax3.set_xlabel('Time [hours]')
    ax3.set_ylabel('Current [mA]')
    ax3.set_title('(c) Load Current Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim([0, 24])
    
    # Panel (d): Power consumption
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(t, results['power']*1000, 'g-', linewidth=1.5)
    ax4.fill_between(t, 0, results['power']*1000, alpha=0.3, color='green')
    avg_power = np.mean(results['power']) * 1000
    ax4.axhline(y=avg_power, color='red', linestyle='--', 
                label=f'Average: {avg_power:.0f} mW')
    ax4.set_xlabel('Time [hours]')
    ax4.set_ylabel('Power [mW]')
    ax4.set_title('(d) Power Consumption')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim([0, 24])
    
    # Panel (e): Brightness profile
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.fill_between(t, 0, results['brightness'], alpha=0.7, color='gold')
    ax5.plot(t, results['brightness'], 'darkorange', linewidth=1)
    ax5.set_xlabel('Time [hours]')
    ax5.set_ylabel('Brightness [%]')
    ax5.set_title('(e) Display Brightness')
    ax5.grid(True, alpha=0.3)
    ax5.set_xlim([0, 24])
    ax5.set_ylim([0, 100])
    
    # Panel (f): Signal strength
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(t, results['signal'], 'forestgreen', linewidth=1.5)
    ax6.fill_between(t, -100, results['signal'], alpha=0.3, color='green')
    ax6.set_xlabel('Time [hours]')
    ax6.set_ylabel('Signal Strength [dBm]')
    ax6.set_title('(f) Cellular Signal Strength')
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim([0, 24])
    ax6.set_ylim([-100, -50])
    
    # Panel (g): Aging effect on capacity
    ax7 = fig.add_subplot(gs[2, 0])
    cycles = np.linspace(0, 500, 100)
    p = BatteryParameters()
    Q_max = [p.a_Q * np.exp(-p.b_Q * N) + p.c_Q * np.exp(-p.d_Q * N) for N in cycles]
    ax7.plot(cycles, Q_max, 'b-', linewidth=2)
    ax7.axvline(x=model.cycle_number, color='red', linestyle='--', 
                label=f'Current: N={model.cycle_number}')
    ax7.set_xlabel('Cycle Number N')
    ax7.set_ylabel('Capacity $Q_{max}$ [Ah]')
    ax7.set_title('(g) Capacity Fade Model')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # Panel (h): Temperature effect on resistance
    ax8 = fig.add_subplot(gs[2, 1])
    temps = np.linspace(-20, 60, 100)
    S_R = [p.C_R + p.A_R * np.exp(-p.B_R * T) for T in temps]
    ax8.plot(temps, S_R, 'r-', linewidth=2)
    ax8.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5)
    ax8.set_xlabel('Temperature [°C]')
    ax8.set_ylabel('Resistance Factor $S_R$ [-]')
    ax8.set_title('(h) Temperature-Resistance Correction')
    ax8.grid(True, alpha=0.3)
    
    # Panel (i): OCV curve
    ax9 = fig.add_subplot(gs[2, 2])
    soc_range = np.linspace(0.01, 0.99, 100)
    ocv_values = [model.ocv(z) for z in soc_range]
    ax9.plot(soc_range * 100, ocv_values, 'purple', linewidth=2)
    ax9.set_xlabel('SOC [%]')
    ax9.set_ylabel('$V_{OCV}$ [V]')
    ax9.set_title('(i) Open Circuit Voltage (Nernst Model)')
    ax9.grid(True, alpha=0.3)
    
    plt.suptitle('Comprehensive Battery Model Analysis', fontsize=14, fontweight='bold', y=0.98)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Figure saved: {save_path}')
    
    return fig


# =============================================================================
# Main Execution
# =============================================================================

if __name__ == "__main__":
    import os
    
    print("="*70)
    print("  24-Hour Smartphone Battery Coupled Model Simulation")
    print("  Electro-Thermal-Aging Model with Multi-Physics Load")
    print("="*70)
    
    # Initialize model
    print("\nInitializing coupled electro-thermal-aging model...")
    model = CoupledBatteryModel(cycle_number=150)
    
    # Create schedule
    schedule = create_daily_schedule()
    
    # Run simulation
    print("Running 24-hour simulation...")
    results = run_daily_simulation(model, schedule, dt=30.0)
    
    # Statistics
    print("\n" + "="*50)
    print("SIMULATION RESULTS")
    print("="*50)
    print(f"  Initial SOC:        {results['soc'][0]:.1f}%")
    print(f"  Final SOC:          {results['soc'][-1]:.1f}%")
    print(f"  Minimum SOC:        {min(results['soc']):.1f}%")
    print(f"  Maximum SOC:        {max(results['soc']):.1f}%")
    print(f"  Average Power:      {np.mean(results['power'])*1000:.1f} mW")
    print(f"  Peak Power:         {max(results['power'])*1000:.1f} mW")
    print(f"  Temperature Range:  {min(results['temperature']):.1f}°C - {max(results['temperature']):.1f}°C")
    print(f"  Energy Consumed:    {np.trapz(results['power'], results['t']*3600)/3600:.2f} Wh")
    
    # Generate figures
    print("\nGenerating visualization figures...")
    os.makedirs('output/results', exist_ok=True)
    
    # Main daily plot
    fig1 = plot_daily_results_english(results, schedule, 
                                       'output/results/daily_simulation_24h_english.png')
    plt.close(fig1)
    
    # Comprehensive analysis
    fig2 = plot_comprehensive_analysis(results, model,
                                        'output/results/comprehensive_analysis.png')
    plt.close(fig2)
    
    print("\nSimulation complete!")
