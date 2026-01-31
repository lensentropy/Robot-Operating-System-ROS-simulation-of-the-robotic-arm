# 智能手机电池放电连续时间建模 (MCM 2026)

## 项目概述

本项目实现了一个完整的智能手机电池放电仿真系统，包含以下物理模型：

- **核心电池模型**: Peukert效应 + Arrhenius温度修正
- **5G通信模型**: 基于香农定理(Shannon-Hartley)和Friis方程
- **GNSS定位模型**: 连续状态机 (Sigmoid状态转换)
- **蓝牙功耗模型**: 支持BLE和经典蓝牙音频
- **后台任务模型**: O-U随机过程 + 泊松突发

## 项目结构

```
battery_model/
├── run_simulation.py          # [主程序] 运行此文件即可
├── requirements.txt           # 依赖包
├── README.md                  # 本文档
├── results/                   # 输出目录 (运行后生成)
│   ├── fig1_5g_physics.png
│   ├── fig2_gnss_dynamics.png
│   ├── fig3_background_stochastic.png
│   └── fig4_system_results.png
└── src/
    ├── __init__.py            # 包初始化
    ├── battery_core.py        # 核心电池与热模型
    ├── network_5g_module.py   # 5G 通信物理模型
    ├── gnss_module.py         # GNSS/GPS 状态机模型
    ├── bluetooth_module.py    # 蓝牙功耗模型
    ├── background_tasks_module.py # 后台随机过程模型
    ├── coupled_system.py      # 耦合系统与场景定义
    └── visualizations.py      # 绘图模块
```

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 运行仿真

```bash
python3 run_simulation.py
```

### 3. 查看结果

运行完成后，在 `results/` 目录中查看生成的图表：

- `fig1_5g_physics.png`: 5G功耗热力图
- `fig2_gnss_dynamics.png`: GNSS状态机功耗曲线
- `fig3_background_stochastic.png`: 后台随机过程
- `fig4_system_results.png`: 综合仿真结果

## 仿真场景

系统支持以下预定义场景：

| 场景 | 描述 | 典型续航 |
|------|------|----------|
| Idle (Screen Off) | 待机模式 | ~48+ 小时 |
| 4K Video (5G) | 5G高清视频流 | ~4-5 小时 |
| GPS Navigation | 导航模式 | ~5-6 小时 |
| Heavy Gaming | 重度游戏 | ~3-4 小时 |
| Daily Mixed Use | 日常混合使用 | ~20 小时 |

## 模型参数

### 电池参数 (BatteryParameters)

- 标称容量: 4.5 Ah
- 标称电压: 3.85 V
- 内阻: 0.08 Ω
- Peukert系数: 1.05

### 热模型参数 (ThermalModel)

- 质量: 0.045 kg
- 比热容: 1000 J/kgK
- 对流换热系数: 8.0 W/m²K
- 散热面积: 0.012 m²

## 扩展

可以通过继承 `Scenario` 类来创建自定义使用场景：

```python
from coupled_system import Scenario

class CustomScenario(Scenario):
    name = "My Custom Scenario"
    
    def get_input(self, t):
        return {
            'bri': 0.5,      # 屏幕亮度 (0-1)
            'cpu': 0.3,      # CPU负载 (0-1)
            'rate': 10e6,    # 网络速率 (bps)
            'dist': 200,     # 基站距离 (m)
            'snr': 35,       # GPS信噪比 (dB-Hz)
            'gps_on': True,  # GPS开关
            'bt_conf': {'mode': 'audio', 'codec': 'aac'}
        }
```

## 许可证

MIT License
