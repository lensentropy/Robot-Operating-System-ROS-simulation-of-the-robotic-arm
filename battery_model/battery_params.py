"""
Smartphone Battery Model - Physical Parameters
================================================
Based on NASA PCoE Dataset #5 and Panasonic NCR18650B specifications.

This module contains all physical parameters for the coupled electro-thermal-aging model.
"""

import numpy as np

# =============================================================================
# Battery Nominal Parameters (NCR18650B baseline)
# =============================================================================
BATTERY_PARAMS = {
    # Nominal specifications
    'Q_nom': 2.0,           # Nominal capacity [Ah] (typical smartphone battery)
    'V_nom': 3.7,           # Nominal voltage [V]
    'V_max': 4.2,           # Maximum voltage [V]
    'V_min': 2.5,           # Cutoff voltage [V]
    
    # Temperature reference
    'T_ref': 25.0,          # Reference temperature [°C]
}

# =============================================================================
# Aging Model Parameters (from NASA PCoE Dataset #5)
# =============================================================================
# Capacity fade: Double-exponential model
# Q_max(N) = a_Q * exp(-b_Q * N) + c_Q * exp(-d_Q * N)
AGING_CAPACITY_PARAMS = {
    'a_Q': -0.1137,         # Fast decay coefficient [Ah]
    'b_Q': 0.0243,          # Fast decay rate [1/cycle]
    'c_Q': 1.9305,          # Slow decay coefficient [Ah]
    'd_Q': 0.0007,          # Slow decay rate [1/cycle]
}

# Impedance growth: Power-law model
# R_total(N) = a_R * N^b_R + c_R
AGING_RESISTANCE_PARAMS = {
    'a_R': 0.0110,          # Growth coefficient [Ω]
    'b_R': 0.2106,          # Growth exponent [-]
    'c_R': 0.0181,          # Initial resistance offset [Ω]
}

# =============================================================================
# Temperature Correction Parameters (from Panasonic Datasheet)
# =============================================================================
# Capacity temperature correction: Sigmoid model
# S_Q(T) = S_Q_max / (1 + exp(-k_Q * (T - T_0)))
TEMP_CAPACITY_PARAMS = {
    'S_Q_max': 1.0391,      # Maximum capacity factor [-]
    'k_Q': 0.0895,          # Temperature sensitivity [1/°C]
    'T_0': -15.1281,        # Inflection temperature [°C]
}

# Resistance temperature correction: Arrhenius model
# S_R(T) = C_R + A_R * exp(-B_R * T)
TEMP_RESISTANCE_PARAMS = {
    'C_R': 0.7136,          # Base resistance factor [-]
    'A_R': 1.3533,          # Pre-exponential factor [-]
    'B_R': 0.0630,          # Temperature coefficient [1/°C]
}

# =============================================================================
# OCV Model Parameters (Nernst-based Combined Model)
# =============================================================================
# V_OCV(z) = K_0 + K_1*z + K_2/z + K_3*ln(z) + K_4*ln(1-z)
OCV_PARAMS = {
    'K_0': 3.4704,          # Constant term [V]
    'K_1': 0.1670,          # Linear coefficient [V]
    'K_2': -0.0042,         # Inverse term coefficient [V]
    'K_3': 0.0573,          # log(z) coefficient [V]
    'K_4': -0.0847,         # log(1-z) coefficient [V]
}

# =============================================================================
# 2nd-Order RC Circuit Parameters
# =============================================================================
RC_CIRCUIT_PARAMS = {
    # Resistance distribution ratios (at reference temperature)
    'R0_ratio': 0.4,        # Ohmic resistance ratio [-]
    'R1_ratio': 0.35,       # Electrochemical polarization ratio [-]
    'R2_ratio': 0.25,       # Concentration polarization ratio [-]
    
    # Time constants
    'tau_1': 20.0,          # Fast RC time constant [s]
    'tau_2': 200.0,         # Slow RC time constant [s]
}

# =============================================================================
# Thermal Model Parameters (Two-State Lumped Model)
# =============================================================================
THERMAL_PARAMS = {
    # Thermal capacitances
    'C_c': 60.0,            # Core thermal capacity [J/K]
    'C_s': 15.0,            # Surface thermal capacity [J/K]
    
    # Thermal resistances
    'R_cs': 2.0,            # Core-to-surface thermal resistance [K/W]
    'R_se': 25.0,           # Surface-to-environment thermal resistance [K/W]
    
    # Entropy coefficient (for reversible heat)
    'dV_dT': 0.0003,        # Entropy heat coefficient [V/K]
}

# =============================================================================
# Coulombic Efficiency Parameters
# =============================================================================
def coulombic_efficiency(T_c):
    """
    Temperature-dependent Coulombic efficiency.
    
    Parameters:
        T_c: Core temperature [°C]
    
    Returns:
        eta: Coulombic efficiency factor [-]
    """
    # Sigmoid-based efficiency model
    eta_max = 0.998
    eta_min = 0.85
    k_eta = 0.1
    T_eta = -5.0
    
    eta = eta_min + (eta_max - eta_min) / (1 + np.exp(-k_eta * (T_c - T_eta)))
    return eta


# =============================================================================
# Load Subsystem Default Parameters
# =============================================================================

# 5G Communication Module Parameters
LOAD_5G_PARAMS = {
    'P_static': 0.15,       # Static power consumption [W]
    'alpha_bb': 1e-8,       # Baseband processing coefficient [W/(bit/s)]
    'B': 100e6,             # Channel bandwidth [Hz]
    'Lambda_env': 1e-9,     # Environmental channel coefficient
    'n': 3.5,               # Path loss exponent
    'eta_PA': 0.35,         # Power amplifier efficiency
}

# Bluetooth/BLE Parameters
LOAD_BLE_PARAMS = {
    'I_sleep': 5e-6,        # Sleep current [A]
    'I_rx': 8e-3,           # Receive current [A]
    'I_tx': 12e-3,          # Transmit current [A]
    'Q_event_base': 50e-6,  # Base event charge [As]
    'tau_default': 100e-3,  # Default connection interval [s]
    'P_audio_base': 0.03,   # Audio streaming base power [W]
}

# Background Tasks Parameters
LOAD_BACKGROUND_PARAMS = {
    'lambda_base': 0.5,     # Base wake-up rate [events/min]
    'P_proc': 0.1,          # Processing power [W]
    'P_idle': 0.02,         # Idle high-power state [W]
    'P_leak': 0.005,        # Deep sleep leakage [W]
    'tau_tail': 10.0,       # Tail time [s]
    'theta_ou': 0.1,        # O-U process mean reversion rate
    'sigma_bg': 0.01,       # Background noise volatility
}

# GNSS Parameters
LOAD_GNSS_PARAMS = {
    'P_acq': 0.15,          # Acquisition mode power [W]
    'P_track': 0.04,        # Tracking mode power [W]
    'P_LNA': 0.01,          # LNA static power [W]
    'S_th': 0.3,            # Signal quality threshold [-]
    'alpha_gnss': 10.0,     # Transition steepness
    'tau_react': 5.0,       # Reaction time constant [s]
}

# OLED Display Parameters
LOAD_DISPLAY_PARAMS = {
    'P_base': 0.1,          # Base static power [W]
    'k_drv': 0.005,         # Refresh rate coefficient [W/Hz]
    'beta_panel': 2.5,      # Panel emissive coefficient [W]
    'gamma': 2.2,           # Gamma correction factor
    'w_R': 0.30,            # Red channel weight
    'w_G': 0.59,            # Green channel weight
    'w_B': 0.11,            # Blue channel weight
    'alpha_bright': 1.5,    # Brightness nonlinearity
}

# SoC/CPU Parameters
LOAD_SOC_PARAMS = {
    'kappa_dvfs': 1.5e-28,  # DVFS cubic coefficient [W/Hz³] (gives ~3W at 3GHz)
    'f_max': 3e9,           # Maximum frequency [Hz]
    'f_min': 3e8,           # Minimum frequency [Hz]
    'I_leak_ref': 5e-3,     # Reference leakage current [A]
    'T_leak_ref': 25.0,     # Reference temperature for leakage [°C]
    'lambda_DIBL': 0.1,     # DIBL coefficient
    'zeta_temp': 0.02,      # Temperature leakage sensitivity
    'n_ideal': 1.5,         # Ideality factor
    'V_dd_nom': 1.0,        # Nominal supply voltage [V]
    'C_th_soc': 5.0,        # SoC thermal capacity [J/K]
    'R_th_soc': 10.0,       # SoC thermal resistance [K/W]
}

# Physical constants
PHYSICAL_CONSTANTS = {
    'k_B': 1.380649e-23,    # Boltzmann constant [J/K]
    'q': 1.602176634e-19,   # Elementary charge [C]
    'T_kelvin_offset': 273.15,  # Kelvin offset
}
