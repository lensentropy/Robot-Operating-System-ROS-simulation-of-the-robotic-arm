# Smartphone Battery Coupled Electro-Thermal-Aging Model

## MCM 2026 Problem A: Smart Phone Battery Depletion Modeling

This repository contains a high-fidelity continuous-time mathematical model for smartphone battery discharge prediction under real-world usage conditions.

## Overview

The model integrates multiple physics domains into a unified system of coupled ordinary differential equations (ODEs):

1. **Electrochemical Model**: 2nd-order Thevenin equivalent circuit with Nernst-based OCV
2. **Aging Model**: Double-exponential capacity fade and power-law impedance growth
3. **Thermal Model**: Two-state lumped thermal dynamics with Bernardi heat generation
4. **Load Models**: Physics-based power consumption for 5G, Bluetooth, GNSS, Display, and SoC

## Mathematical Foundation

### System State Vector

```
x = [z, V₁, V₂, Tc, Ts, Tsoc, xlock]
```

where:
- `z` - State of Charge (SOC) ∈ [0, 1]
- `V₁` - Electrochemical polarization voltage [V]
- `V₂` - Concentration polarization voltage [V]
- `Tc` - Battery core temperature [°C]
- `Ts` - Battery surface temperature [°C]
- `Tsoc` - SoC chip temperature [°C]
- `xlock` - GNSS lock state [-]

### Governing Equations

#### Battery Dynamics (Coulomb Counting)
```
dz/dt = -I(t) · η(Tc) / (Qmax(N, Tc) · 3600)
```

#### Capacity Fade (Double-Exponential)
```
Qmax(N) = aQ · exp(-bQ · N) + cQ · exp(-dQ · N)
```

#### Impedance Growth (Power-Law)
```
Rtotal(N) = aR · N^bR + cR
```

#### OCV Model (Nernst-based)
```
VOCV(z) = K₀ + K₁z + K₂/z + K₃ln(z) + K₄ln(1-z)
```

#### 2nd-Order RC Circuit
```
dV₁/dt = -V₁/(R₁C₁) + I(t)/C₁
dV₂/dt = -V₂/(R₂C₂) + I(t)/C₂
Vterm(t) = VOCV(z) - V₁(t) - V₂(t) - I(t)R₀
```

#### Two-State Thermal Model
```
Cc · dTc/dt = Qgen - (Tc - Ts)/Rcs
Cs · dTs/dt = (Tc - Ts)/Rcs - (Ts - Tenv)/Rse
```

where heat generation follows the Bernardi equation:
```
Qgen = I²Rtotal + I·Tc·(∂VOCV/∂T)
```

## Project Structure

```
battery_model/
├── battery_params.py      # Physical parameters and constants
├── battery_core.py        # Core battery electrochemical model
├── load_models.py         # Smartphone load subsystem models
├── coupled_system.py      # Integrated coupled system solver
├── visualization.py       # Plotting and visualization utilities
├── run_simulation.py      # Main simulation script
├── requirements.txt       # Python dependencies
└── README.md             # This file
```

## Installation

```bash
cd battery_model
pip install -r requirements.txt
```

## Usage

### Quick Test
```bash
python run_simulation.py --quick
```

### Full Simulation Suite
```bash
python run_simulation.py
```

### Specific Scenario
```bash
python run_simulation.py --scenario gaming
```

### Generate Model Figures Only
```bash
python run_simulation.py --figures-only
```

### Print Model Equations
```bash
python run_simulation.py --equations
```

## Available Usage Scenarios

| Scenario | Description |
|----------|-------------|
| `idle_screen_off` | Minimal background activity, screen off |
| `idle_screen_on` | Light browsing, screen on at moderate brightness |
| `video_streaming` | Video playback with 5G streaming |
| `gaming` | Intensive gaming with high CPU and display |
| `navigation` | GPS navigation with audio |
| `voice_call` | Voice call scenario |
| `weak_signal` | Weak cellular signal (high TX power) |

## Key Features

1. **Physics-Based**: All model components are derived from first principles
2. **Continuous-Time**: True continuous ODE formulation (not discrete time-stepping)
3. **Multi-Physics Coupling**: Battery-thermal-load feedback loops
4. **Wide Operating Range**: -20°C to 60°C temperature, 0-500 cycles aging
5. **Validated Parameters**: Based on NASA PCoE Dataset #5 and Panasonic NCR18650B specs

## Load Model Physics

### 5G Communication
- Shannon-Hartley capacity constraint
- Friis transmission equation for link budget
- Power amplifier efficiency model

### Bluetooth/BLE
- Discrete event charge integration
- Hyperbolic average current law
- A2DP streaming model

### GNSS
- Sigmoid lock probability
- First-order state relaxation dynamics
- Environment-aware mode switching

### OLED Display
- Content-aware APL (Average Pixel Level)
- LTPO variable refresh rate
- Nonlinear brightness response

### SoC/CPU
- Alpha-Power Law DVFS
- BSIM4 leakage current model
- Electro-thermal coupling

## Output

The simulation produces:
- SOC and voltage time series
- Temperature profiles (core, surface, SoC)
- Current and power consumption
- Power breakdown by subsystem
- Battery life predictions

## Data Sources

- **NASA PCoE Dataset #5**: Battery aging characterization
- **Panasonic NCR18650B Datasheet**: Temperature performance specifications

## References

1. Bernardi, D., Pawlikowski, E., & Newman, J. (1985). A general energy balance for battery systems.
2. He, H., Xiong, R., & Fan, J. (2011). Evaluation of lithium-ion battery equivalent circuit models.
3. Sakurai, T., & Newton, A. R. (1990). Alpha-power law MOSFET model.

## License

This model is developed for the 2026 Mathematical Contest in Modeling (MCM).

## Authors

MCM 2026 Team - Problem A
