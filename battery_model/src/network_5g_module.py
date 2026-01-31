import numpy as np

class Network5GModule:
    def __init__(self):
        self.bandwidth = 100e6     # 100 MHz
        self.noise_fig = 7.0       # dB
        self.thermal_noise = -174  # dBm/Hz
        self.fc = 3.5e9            # 3.5 GHz
        self.c = 3e8
        
    def total_power(self, throughput_bps, distance_m):
        """
        计算5G模块功耗
        :param throughput_bps: 目标速率 (bps)
        :param distance_m: 基站距离 (m)
        """
        if throughput_bps <= 0:
            return 0.050 # 待机/DRX功耗 (W)

        # 1. 计算所需 SNR (香农定理)
        # R = B * log2(1 + SNR)  => SNR = 2^(R/B) - 1
        spectral_eff = throughput_bps / self.bandwidth
        snr_linear = 2**spectral_eff - 1
        snr_db = 10 * np.log10(max(snr_linear, 1e-9)) + 3 # +3dB margin
        
        # 2. 计算路径损耗 (Friis方程 + 城市阴影衰落)
        # PL(dB) = 20log(d) + 20log(f) - 147.55
        pl_db = 20*np.log10(distance_m) + 20*np.log10(self.fc) - 147.55
        pl_db += 10 # 增加一些城市损耗
        
        # 3. 计算所需发射功率
        # P_tx(dBm) = SNR(dB) + Noise(dBm) + PL(dB)
        noise_power_dbm = self.thermal_noise + 10*np.log10(self.bandwidth) + self.noise_fig
        tx_power_dbm = snr_db + noise_power_dbm + pl_db
        
        # 限制最大发射功率 (23 dBm = 0.2W)
        tx_power_dbm = min(tx_power_dbm, 23.0)
        tx_power_w = 10**(tx_power_dbm/10) / 1000
        
        # 4. 功耗模型 (PA效率随功率下降)
        # Eta = Eta_max * sqrt(P/P_max)
        eta_max = 0.45
        p_max = 0.2
        eta = eta_max * np.sqrt(max(tx_power_w, 0.001)/p_max)
        
        p_pa = tx_power_w / eta
        p_circuit = 0.2 + 1.5e-9 * throughput_bps # 基带处理功耗
        
        return p_pa + p_circuit
