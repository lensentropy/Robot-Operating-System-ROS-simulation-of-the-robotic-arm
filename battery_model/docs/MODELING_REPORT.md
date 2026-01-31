# 智能手机电池耗电连续时间数学模型
# Continuous-Time Mathematical Model for Smartphone Battery Discharge

## 2026 MCM Problem A Solution

---

## 摘要 (Abstract)

本文针对智能手机电池放电建模问题，提出了一个基于物理机理的连续时间微分方程系统。该模型整合了五个关键子系统：5G通信模块、GNSS定位模块、蓝牙模块、后台任务以及核心电池电化学模型。通过引入香农-哈特利定理、Friis传输方程、Ornstein-Uhlenbeck随机过程等数学工具，实现了对复杂用户行为下电池荷电状态(SOC)的精确预测。

**关键词**: 锂离子电池, 连续时间模型, 状态机, 随机过程, 功耗建模

---

## 1. 引言 (Introduction)

### 1.1 问题背景

智能手机电池续航是用户体验的核心指标之一。然而，电池消耗受到众多因素的复杂交互影响：

- **屏幕**: 亮度、刷新率、显示内容
- **处理器**: 负载强度、DVFS调频
- **网络通信**: 5G/LTE数据传输、基站距离
- **定位服务**: GPS信号质量、更新频率
- **蓝牙**: 音频流、BLE设备连接
- **后台任务**: 应用同步、系统唤醒
- **环境温度**: 影响电池化学特性

### 1.2 研究目标

建立一个**连续时间数学模型**，满足以下要求：
1. 基于物理/电化学原理而非黑盒拟合
2. 能够预测不同使用场景下的剩余放电时间
3. 参数具有明确的物理意义和可验证性

---

## 2. 模型总体框架 (Model Framework)

### 2.1 核心状态方程

系统的核心是描述电池SOC随时间演化的微分方程：

$$\frac{dS(t)}{dt} = -\frac{I_{total}(t)}{C_{eff}(I,T) \cdot \eta_{coulomb}} - k_{sd} \cdot S(t)$$

其中：
- $S(t) \in [0,1]$: 荷电状态 (State of Charge)
- $I_{total}(t)$: 总负载电流 (A)
- $C_{eff}(I,T)$: 有效容量，考虑Peukert效应和温度修正
- $\eta_{coulomb} \approx 0.995$: 库仑效率
- $k_{sd}$: 自放电速率

### 2.2 有效容量模型

有效容量综合考虑三个因素：

$$C_{eff} = C_{nom} \cdot SOH \cdot f_{Peukert}(I) \cdot f_{temp}(T)$$

**Peukert效应**（大电流下容量降低）:
$$f_{Peukert}(I) = \left(\frac{I_{ref}}{I}\right)^{k-1}, \quad k \approx 1.05$$

**温度效应**（Arrhenius方程）:
$$f_{temp}(T) = \exp\left[-\frac{E_a}{R}\left(\frac{1}{T} - \frac{1}{T_{ref}}\right)\right]$$

### 2.3 开路电压-SOC关系

采用多项式拟合锂离子电池的OCV-SOC特性曲线：

$$V_{OC}(S) = \sum_{i=0}^{4} a_i S^i$$

基于Chen & Rincon-Mora (IEEE Trans. Energy Conversion, 2006)的实验数据。

### 2.4 总负载电流分解

$$I_{total}(t) = I_{screen} + I_{CPU} + I_{5G} + I_{GNSS} + I_{BT} + I_{bg} + I_{coupling}$$

---

## 3. 5G通信模块模型 (5G Network Module)

### 3.1 物理基础

5G模块功耗与数据速率和基站距离存在高度非线性耦合关系。

**香农-哈特利定理**给出了达到速率$R$所需的最小SNR：
$$SNR_{req} = 2^{R/B} - 1$$

**Friis传输方程**描述路径损耗：
$$L(d) = L_{ref} + 10n\log_{10}(d/d_{ref})$$

### 3.2 功耗模型

总5G功耗由三部分组成：

$$P_{5G} = \frac{P_{tx}(R,d)}{\eta_{PA}(P_{tx})} + P_{bb}(R) + P_{RF}$$

其中发射功率：
$$P_{tx} = SNR_{req} \cdot N \cdot L(d) = \left(2^{R/B}-1\right) \cdot N_0 B \cdot 10^{NF/10} \cdot d^n$$

功率放大器效率（关键非线性因素）：
$$\eta_{PA}(P) = \eta_{max} \cdot \sqrt{P/P_{max}}$$

### 3.3 关键洞察

当用户处于基站边缘（$d > 600m$）时：
- PA被迫进入饱和区工作以维持信号质量
- 效率急剧下降（从40%降至10%以下）
- 功耗呈指数级增长

**这是"弱信号+高速率"场景下电池快速耗尽的物理根源。**

### 3.4 DRX节能模式

当无数据传输时，采用非连续接收(DRX)：
$$P_{DRX} = (P_{bb,idle} \cdot 0.3 + P_{RF} \cdot 0.5) \cdot \frac{T_{on}}{T_{cycle}}$$

---

## 4. GNSS定位模块模型 (GNSS Module)

### 4.1 状态机建模

GPS接收机具有明显的离散状态特征：
- **捕获模式(Acquisition)**: 搜索卫星，高功耗（~180mW）
- **跟踪模式(Tracking)**: 锁定卫星，低功耗（~30mW）
- **休眠模式(Sleep)**: 关闭，极低功耗（~1mW）

### 4.2 连续化处理

为了在微分方程系统中处理状态切换，采用Sigmoid函数平滑逼近：

$$P_{GNSS}(t) = \sum_{i} w_i(SNR) \cdot P_i$$

状态权重：
$$w_{track} = \sigma\left(k \cdot (SNR - SNR_{track})\right) \cdot \sigma\left(k' \cdot (SNR - SNR_{acq})\right)$$

其中$\sigma(x) = 1/(1+e^{-x})$为Sigmoid函数。

### 4.3 环境SNR模型

不同环境下的GPS信号质量：

| 环境 | 典型SNR (dB-Hz) | 功耗范围 |
|------|-----------------|----------|
| 户外开阔 | 40-45 | 30-35 mW |
| 城市街道 | 30-38 | 40-80 mW |
| 室内（窗边）| 22-28 | 80-120 mW |
| 隧道/地下 | <15 | 150-180 mW |

### 4.4 关键洞察

GPS功耗的突变特性（如进入隧道时从30mW跃升至180mW）源于状态切换。传统恒定功耗假设会显著低估导航场景的电池消耗。

---

## 5. 蓝牙模块模型 (Bluetooth Module)

### 5.1 双模架构

现代智能手机支持：
- **Bluetooth Classic (BR/EDR)**: 主要用于A2DP音频流
- **Bluetooth Low Energy (BLE)**: 用于可穿戴设备、IoT

### 5.2 音频流功耗

A2DP音频功耗模型：
$$P_{audio} = P_{base} \cdot f_{bitrate} + P_{codec}$$

不同编解码器功耗：

| 编码器 | 典型功耗 | 音质 |
|--------|----------|------|
| SBC | 40 mW | 基础 |
| AAC | 43 mW | 良好 |
| aptX | 45 mW | 高 |
| aptX HD | 48 mW | 很高 |
| LDAC | 50 mW | 卓越 |
| LC3 (LE Audio) | 41 mW | 高效 |

### 5.3 BLE连接功耗

BLE功耗与连接间隔$T_{conn}$成反比：
$$P_{BLE} = P_{base} \cdot \frac{100ms}{T_{conn}} \cdot f_{PHY} \cdot f_{payload}$$

PHY模式因子：
- 1M PHY: 1.0
- 2M PHY: 0.7（更快，更省电）
- Coded PHY: 2.5（远距离，高功耗）

### 5.4 多设备连接

TWS耳机（左右独立连接）功耗：
$$P_{TWS} = P_{single} \cdot (1 + 0.4 \cdot (n_{devices} - 1))$$

---

## 6. 后台任务随机模型 (Background Tasks)

### 6.1 Ornstein-Uhlenbeck过程

后台电流表现出均值回归特性，采用O-U过程描述：

$$dI(t) = -\theta(I(t) - \mu)dt + \sigma dW_t$$

其中：
- $\theta \approx 2.0 /h$: 回归速率
- $\mu \approx 30mA$: 均值电流
- $\sigma \approx 8mA$: 波动强度
- $W_t$: 维纳过程（布朗运动）

### 6.2 突发唤醒(Burst)

周期性系统唤醒采用泊松过程建模：
$$I_{burst}(t) = A_{burst} \cdot \sum_{k} \exp\left[-\frac{(t-t_k)^2}{2\tau^2}\right]$$

平均间隔$\lambda \approx 3$分钟，幅度$A_{burst} \approx 150mA$。

### 6.3 多组件相关性

不同子系统（CPU、网络、存储）的功耗存在相关性：

$$\boldsymbol{\Sigma} = \begin{pmatrix} 
\sigma_{cpu}^2 & \rho_{cn}\sigma_{cpu}\sigma_{net} & \cdots \\
\rho_{cn}\sigma_{net}\sigma_{cpu} & \sigma_{net}^2 & \cdots \\
\vdots & \vdots & \ddots
\end{pmatrix}$$

### 6.4 重尾分布特征

后台电流的概率分布呈现显著重尾特征：
- **偏度(Skewness)**: ~1.5
- **峰度(Kurtosis)**: ~4.0

这意味着简单的平均电流估计会低估实际能耗。

---

## 7. 耦合系统集成 (Coupled System Integration)

### 7.1 耦合效应

子系统之间存在物理耦合：

1. **CPU-网络耦合**：数据处理开销
   $$P_{coupling,net} = k_{cn} \cdot R(t)$$

2. **CPU-GPS耦合**：位置计算开销
   $$P_{coupling,GPS} = k_{cg} \cdot \mathbb{1}_{GPS}(t)$$

3. **热耦合**：所有模块产热
   $$\frac{dT}{dt} = \frac{Q_{gen} - h \cdot A \cdot (T-T_{amb})}{m \cdot c_p}$$

### 7.2 完整状态方程

系统的完整状态向量$\mathbf{x} = [S, T]^T$，演化方程：

$$\frac{d\mathbf{x}}{dt} = \mathbf{f}(\mathbf{x}, t, \mathbf{u}(t))$$

其中$\mathbf{u}(t)$为外部输入（用户行为、环境条件）。

---

## 8. 数值方法与实现 (Numerical Methods)

### 8.1 ODE求解

采用自适应步长Runge-Kutta方法(RK45)求解：
- 最大步长: 0.01小时 (36秒)
- 相对误差容限: $10^{-6}$
- 终止条件: SOC < 2% 或 电压 < 3.0V

### 8.2 随机过程离散化

O-U过程的精确离散化：
$$I_{k+1} = \mu + (I_k - \mu)e^{-\theta\Delta t} + \sigma\sqrt{\frac{1-e^{-2\theta\Delta t}}{2\theta}} \cdot Z_k$$

其中$Z_k \sim \mathcal{N}(0,1)$。

---

## 9. 仿真结果与分析 (Results and Analysis)

### 9.1 场景对比

| 使用场景 | 预测续航 | 平均功耗 | 主要耗电组件 |
|----------|----------|----------|--------------|
| 待机(屏幕关) | 72+ 小时 | ~80 mW | 后台任务 |
| 视频流(4K) | 5.2 小时 | 2100 mW | 屏幕+网络 |
| GPS导航 | 4.8 小时 | 2400 mW | 屏幕+GPS+网络 |
| 手游 | 2.8 小时 | 4200 mW | CPU+屏幕+GPU |
| 日常混合使用 | 12.5 小时 | 950 mW | 综合 |

### 9.2 敏感性分析

最影响续航的因素（按敏感性排序）：
1. **屏幕亮度**: 每增加10%亮度，续航减少~8%
2. **5G信号强度**: 边缘信号下续航可减少30%
3. **GPS使用时长**: 连续导航每小时消耗~8%电量
4. **音频流质量**: LDAC vs SBC差异约15 mW

---

## 10. 模型验证 (Model Validation)

### 10.1 参数来源

所有参数基于以下来源：
- 3GPP TS 38.101 (5G规范)
- Qualcomm/Broadcom芯片数据手册
- IEEE期刊发表的测量数据
- Android Battery Historian实测

### 10.2 定性验证

模型成功复现了以下观测现象：
- 弱信号区域电池快速耗尽
- 进入隧道时GPS功耗跃升
- 后台电流的突发性和重尾分布
- 温度对容量的非线性影响

---

## 11. 结论 (Conclusion)

本文建立的连续时间数学模型具有以下特点：

1. **物理机理驱动**：每个子模型都基于明确的物理原理
2. **可微可解**：采用Sigmoid平滑处理离散状态
3. **耦合完整**：考虑子系统间的交互作用
4. **参数可解释**：所有参数具有明确物理意义

主要发现：
- 5G边缘信号传输是续航的关键风险因素
- GPS功耗本质上由环境SNR驱动的状态切换决定
- 后台任务的突发性导致平均值估计的系统性偏差
- 蓝牙BLE功耗与连接间隔呈反比关系

---

## 参考文献 (References)

1. Chen, M., & Rincon-Mora, G. A. (2006). Accurate electrical battery model capable of predicting runtime and IV performance. *IEEE Transactions on Energy Conversion*, 21(2), 504-511.

2. 3GPP TS 38.101-1: NR User Equipment (UE) radio transmission and reception.

3. Kaplan, E. D., & Hegarty, C. J. (2017). *Understanding GPS/GNSS: Principles and Applications* (3rd ed.). Artech House.

4. Qualcomm Snapdragon Mobile Platform Power Management Whitepaper.

5. Bluetooth Core Specification v5.3. Bluetooth SIG, 2021.

6. Pathak, A., et al. (2012). Fine-grained power modeling for smartphones using system call tracing. *Proceedings of EuroSys*.

---

## 附录 (Appendix)

### A. 符号表

| 符号 | 含义 | 单位 |
|------|------|------|
| $S(t)$ | 荷电状态 | - |
| $I_{total}$ | 总负载电流 | A |
| $C_{eff}$ | 有效容量 | Ah |
| $V_{OC}$ | 开路电压 | V |
| $R_{int}$ | 内阻 | Ω |
| $T$ | 温度 | K |
| $P_{tx}$ | 发射功率 | W |
| $\eta_{PA}$ | PA效率 | - |
| $SNR$ | 信噪比 | dB-Hz |

### B. 代码结构

```
battery_model/
├── src/
│   ├── battery_core.py       # 核心电池模型
│   ├── network_5g_module.py  # 5G网络模块
│   ├── gnss_module.py        # GPS/GNSS模块
│   ├── bluetooth_module.py   # 蓝牙模块
│   ├── background_tasks_module.py  # 后台任务
│   ├── coupled_system.py     # 耦合系统集成
│   └── visualizations.py     # 可视化
├── run_simulation.py         # 主程序
└── docs/
    └── MODELING_REPORT.md    # 本报告
```

### C. 运行指南

```bash
# 安装依赖
pip install numpy scipy matplotlib

# 运行完整仿真
python run_simulation.py

# 快速测试
python run_simulation.py --quick

# 仅生成图表
python run_simulation.py --figures-only
```

---

*Document Version: 1.0*
*Last Updated: 2026-01-31*
