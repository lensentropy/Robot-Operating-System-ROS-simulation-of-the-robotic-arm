import numpy as np

class BackgroundTasksModule:
    def __init__(self):
        self.mu = 0.020      # 均值 20mA
        self.theta = 5.0     # 回归速度
        self.sigma = 0.008   # 波动
        self.burst_rate = 10 # 突发/小时
        
    def generate_path(self, t_array, seed=None):
        if seed: np.random.seed(seed)
        n = len(t_array)
        dt_hours = t_array[1] - t_array[0]
        
        # 1. O-U 过程 (欧拉离散化)
        ou_path = np.zeros(n)
        ou_path[0] = self.mu
        
        noise = np.random.normal(0, np.sqrt(dt_hours), n)
        for i in range(1, n):
            dx = self.theta * (self.mu - ou_path[i-1]) * dt_hours + self.sigma * noise[i]
            ou_path[i] = ou_path[i-1] + dx
            
        # 2. 泊松突发 (Bursts)
        burst_path = np.zeros(n)
        # 每个步长的突发概率
        p_burst = self.burst_rate * dt_hours 
        bursts = np.random.random(n) < p_burst
        
        # 突发持续衰减模拟
        current_burst = 0
        decay = np.exp(-500 * dt_hours) # 快速衰减
        
        for i in range(n):
            if bursts[i]:
                current_burst = 0.150 # 150mA 峰值
            else:
                current_burst *= decay
            burst_path[i] = current_burst
            
        return np.maximum(0, ou_path + burst_path)
