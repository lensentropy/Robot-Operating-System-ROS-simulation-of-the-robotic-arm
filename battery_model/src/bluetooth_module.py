class BluetoothModule:
    def __init__(self):
        self.P_base = 0.010
        self.codecs = {'sbc': 0.030, 'aac': 0.033, 'ldac': 0.045}
        
    def total_power(self, mode='idle', **kwargs):
        """
        :param mode: 'idle', 'ble', 'audio'
        """
        p = 0.0
        if mode == 'idle':
            p = 0.001
        elif mode == 'ble':
            # BLE功耗与间隔成反比
            interval_ms = kwargs.get('interval_ms', 1000)
            p = 0.002 + (100 / interval_ms) * 0.005
        elif mode == 'audio':
            codec = kwargs.get('codec', 'aac')
            p = self.P_base + self.codecs.get(codec, 0.030)
            
        return p
