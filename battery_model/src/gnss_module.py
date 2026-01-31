import numpy as np

class GNSSModule:
    def __init__(self):
        # 功耗参数 (W)
        self.P_sleep = 0.001
        self.P_track = 0.035
        self.P_acq = 0.180
        
        # 阈值
        self.snr_track = 28.0
        self.snr_acq = 18.0
        self.k = 0.5 # Sigmoid 斜率

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-self.k * x))

    def power_consumption(self, snr, active=False):
        if not active:
            return self.P_sleep
            
        # 连续状态机权重
        # w_track: 能够跟踪的概率
        w_track = self.sigmoid(snr - self.snr_track)
        
        # 如果不能跟踪，进入捕获模式
        # w_acq = 1 - w_track
        
        p_avg = w_track * self.P_track + (1 - w_track) * self.P_acq
        return p_avg
