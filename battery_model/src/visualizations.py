import matplotlib.pyplot as plt
import numpy as np

# 设置样式
plt.style.use('bmh')
plt.rcParams['font.family'] = 'sans-serif'

def figure_5g_power_analysis(filename):
    from network_5g_module import Network5GModule
    net = Network5GModule()
    
    d = np.linspace(10, 1000, 100)
    r = np.linspace(1e6, 200e6, 100)
    D, R = np.meshgrid(d, r)
    Z = np.zeros_like(D)
    
    for i in range(D.shape[0]):
        for j in range(D.shape[1]):
            Z[i,j] = net.total_power(R[i,j], D[i,j])
            
    fig, ax = plt.subplots(figsize=(8, 6))
    cp = ax.contourf(D, R/1e6, Z, 20, cmap='viridis')
    cbar = fig.colorbar(cp)
    cbar.set_label('Power (W)')
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Throughput (Mbps)')
    ax.set_title('5G Power Consumption Model')
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def figure_gnss_state_dynamics(filename):
    from gnss_module import GNSSModule
    gnss = GNSSModule()
    
    snr = np.linspace(10, 50, 100)
    p = [gnss.power_consumption(s, True)*1000 for s in snr]
    
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(snr, p, 'r-', lw=2)
    ax.set_xlabel('SNR (dB-Hz)')
    ax.set_ylabel('Power (mW)')
    ax.set_title('GNSS State Machine Power Dynamics')
    ax.axvspan(10, 18, color='gray', alpha=0.2, label='Blind')
    ax.axvspan(18, 28, color='orange', alpha=0.2, label='Acquisition')
    ax.axvspan(28, 50, color='green', alpha=0.2, label='Tracking')
    ax.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def figure_background_stochastic(filename):
    from background_tasks_module import BackgroundTasksModule
    bg = BackgroundTasksModule()
    t = np.linspace(0, 1, 1000) # 1 hour
    i = bg.generate_path(t, seed=123) * 1000 # mA
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [3, 1]})
    
    ax1.plot(t*60, i, color='#2c3e50', lw=1)
    ax1.set_xlabel('Time (min)')
    ax1.set_ylabel('Current (mA)')
    ax1.set_title('Stochastic Background Activity (O-U + Poisson)')
    
    ax2.hist(i, bins=30, orientation='horizontal', color='#2c3e50', alpha=0.7)
    ax2.set_title('Distribution')
    ax2.axis('off')
    
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def figure_coupled_system_simulation(results, filename):
    if not results: return
    
    # 取混合场景展示
    res = results['Daily Mixed Use']
    t = res['time']
    
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(3, 1)
    
    # 1. SOC
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, res['SOC']*100, 'g-', lw=2)
    ax1.set_ylabel('SOC (%)')
    ax1.set_title('Mixed Usage: Battery SOC & Temperature')
    
    ax1b = ax1.twinx()
    ax1b.plot(t, res['temperature']-273.15, 'r--', alpha=0.6)
    ax1b.set_ylabel('Temp (°C)', color='r')
    
    # 2. Power Breakdown (Stacked)
    ax2 = fig.add_subplot(gs[1])
    bd = res['breakdown']
    labels = list(bd.keys())
    data = np.vstack([bd[k] for k in labels])
    ax2.stackplot(t, data, labels=labels, alpha=0.85)
    ax2.set_ylabel('Power (W)')
    ax2.legend(loc='upper right', ncol=6, fontsize='small')
    ax2.set_title('Power Consumption Components')
    
    # 3. Bar Chart Comparison
    ax3 = fig.add_subplot(gs[2])
    names = list(results.keys())
    lives = [results[n]['time'][-1] for n in names]
    
    bars = ax3.barh(names, lives, color='steelblue')
    ax3.set_xlabel('Battery Life (Hours)')
    for bar in bars:
        width = bar.get_width()
        ax3.text(width + 0.5, bar.get_y() + bar.get_height()/2, 
                 f'{width:.1f} h', va='center')
    
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()
