# Tomato Greenhouse Microclimate & Crop Quality Control System
### Pure Base MATLAB Master Pipeline | 11 States | 4 Actuators | Tri-Hybrid Control

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B%20%7C%20R2025b%20Tested-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Toolboxes](https://img.shields.io/badge/Toolbox%20Dependencies-0.0%25%20(Pure%20Base%20MATLAB)-brightgreen.svg)]()
[![Simulink](https://img.shields.io/badge/Simulink%20Dependency-0.0%25%20(Vectorized%20ODE%20Engine)-blue.svg)]()
[![Status](https://img.shields.io/badge/Phase%201-100%25%20Accomplished-success.svg)]()

A multivariable optimal climate control system engineered for greenhouse tomato (*Solanum lycopersicum*) production. Developed in **100% pure base MATLAB** without requiring the Control System Toolbox, Optimization Toolbox, or Simulink runtime engines.

The entire control, modeling, simulation, and real-time visualization pipeline executes from a single self-contained master file: **[`main.m`](main.m)**.

---

## Table of Contents
1. [Key Features](#key-features)
2. [Online Scientific Data Sources](#online-scientific-data-sources)
3. [System Modeling & State Space Formulation](#system-modeling--state-space-formulation)
4. [Control Architectures](#control-architectures)
5. [Botanical Growth Phases](#botanical-growth-phases)
6. [Quantitative Benchmark Matrix](#quantitative-benchmark-matrix)
7. [Visualizations & Real-Time Dashboards](#visualizations--real-time-dashboards)
8. [Three-Phase Project Roadmap](#three-phase-project-roadmap)
9. [How to Run](#how-to-run)
10. [Repository Structure](#repository-structure)

---

## Key Features

* **Single-File Master Architecture (`main.m`)**: All plant models, numerical linearization, modal analysis, controller syntheses, numerical simulation, and real-time plotting live inside one standalone file.
* **Zero External Dependencies**: Pure native linear algebra implementation of Sylvester multi-input pole placement (`pure_place`), Continuous Algebraic Riccati Equation solver (`pure_care` via Real Ordered Schur decomposition), Kalman staircase decomposition (`pure_staircase`), and controllability/observability matrices (`pure_ctrb`, `pure_obsv`).
* **Eraticated Historical Defects**:
  - **Eliminated Matrix $B$ Halving Defect**: Forward differencing along the non-negative actuator manifold $[0, 1]$ prevents boundary clamping, restoring $100\%$ control gain ($B(1,1) = 12.00$).
  - **Eliminated $10^7$ Gain Explosion**: Subspace coordinate scaling guarantees feedback gains strictly bounded within $[0.001, 1.0]$.
  - **Zero Steady-State Tracking**: Augmented 15-state LQR with integral action (LQR-I) guarantees asymptotic rejection of diurnal thermal and solar disturbances.
* **Real-Time Graphics**: Figures update with immediate `drawnow` flushes, displaying botanical growth phase demarcations and badges across all dashboards.

---

## Online Scientific Data Sources

All botanical target setpoints, physiological tolerance ranges, and agronomic health curves are derived directly from authoritative peer-reviewed agricultural standards:

1. **Food and Agriculture Organization (FAO)**:  
   *Crop Ecological Requirements and Irrigation Guidelines: Tomato (Solanum lycopersicum)*  
   FAO Irrigation and Drainage Paper 56, Rome, Italy.  
   🌐 [FAO Land & Water Tomato Portal](https://www.fao.org/land-water/databases-and-software/crop-information/tomato/en/)
2. **University of California Davis (UC Davis)**:  
   *Vegetable Research and Information Center (VRIC) - Greenhouse Tomato Production Guidelines*  
   UC ANR Publication 7250 / VRIC Agronomy Standards.  
   🌐 [UC Davis VRIC](https://vric.ucdavis.edu/)
3. **Wageningen University & Research (WUR)**:  
   *Greenhouse Horticulture Crop Growth and Microclimate Modeling*  
   Heuvelink, E. (Ed.), *Tomatoes*, CABI Publishing; De Koning (1994).  
   🌐 [WUR Greenhouse Horticulture](https://www.wur.nl/en/research-results/research-institutes/plant-research/greenhouse-horticulture.htm)

---

## System Modeling & State Space Formulation

The non-linear plant dynamics are governed by 11 coupled state variables, 4 bounded physical control inputs, and 3 continuous diurnal disturbances:

$$\dot{x}(t) = f(x(t), u(t), d(t)), \quad u(t) \in [0, 1]^4$$

### State Vector ($x \in \mathbb{R}^{11}$)
* $x_1$: Soil Moisture (%)
* $x_2$: Greenhouse Air Temperature (°C)
* $x_3$: Greenhouse Relative Humidity (%)
* $x_4$: Total Illumination / PAR (lux)
* $x_5$: Soil pH
* $x_6$: Soil Electrical Conductivity (EC, dS/m)
* $x_7$: Available Nitrogen ($N$, mg/kg)
* $x_8$: Available Phosphorus ($P$, mg/kg)
* $x_9$: Available Potassium ($K$, mg/kg)
* $x_{10}$: Water Storage Level (%)
* $x_{11}$: VOC / Gaseous Stress Index (0–100)

### Control Actuators ($u \in [0, 1]^4$)
* $u_1$: Irrigation Drip Pump Duty Cycle
* $u_2$: Ventilation Fan Cooling Duty Cycle
* $u_3$: Ultrasonic Misting System Duty Cycle
* $u_4$: Supplemental LED Grow Light Duty Cycle

### Diurnal Weather Disturbances ($d \in \mathbb{R}^3$)
* $d_1$: Ambient Outdoor Temperature ($18^\circ\text{C} - 29^\circ\text{C}$ diurnal cycle)
* $d_2$: Natural Solar Irradiance ($0 - 22,000\text{ lux}$ solar bell curve)
* $d_3$: Soil Evaporation Demand Index ($0.35 - 1.15$)

---

## Control Architectures

The project contrasts three multivariable control strategies under identical diurnal disturbances:

1. **Multiloop Decoupled PID**:
   - 4 independent feedback loops governing soil moisture, air temperature, relative humidity, and illumination.
   - Equipped with **conditional integration anti-windup clamping** that freezes integrator accumulation whenever actuators reach physical saturation limits ($0.0$ or $1.0$).
2. **Scaled Subspace Pole Placement**:
   - Extracts the actively actuated environmental subsystem ($4 \times 4$).
   - Multi-input Sylvester equation assigns well-damped, stable closed-loop eigenvalues:
     $$\lambda_{\text{desired}} = \{-0.60, -0.80, -1.00, -1.20\}\text{ h}^{-1}$$
   - Feedback gains remain strictly bounded ($|K_{ij}| \le 0.2167$), eliminating aggressive actuator chattering.
3. **Optimal Linear Quadratic Regulator with Integral Action (LQR-I)**:
   - Formulates an augmented 15-state system ($11$ plant states $+ 4$ tracking error integrators):
     $$\dot{z}(t) = y_{\text{track}}(t) - r(t) = C_{\text{track}}x(t) - r(t)$$
   - Solves the Continuous Algebraic Riccati Equation (CARE) using Hamiltonian Real Ordered Schur decomposition (residual norm $= 1.25 \times 10^{-10}$).
   - Guarantees zero steady-state tracking error across all 5 growth stage transitions.

---

## Botanical Growth Phases

The 24-hour simulation maps across all 5 botanical development stages of the tomato lifecycle:

| Phase | Time ($t$) | Equivalent Days | Optimal Temp (°C) | Optimal RH (%) | Optimal Moisture (%) | Optimal Light (lux) |
|---|---|---|---|---|---|---|
| **Stage 1: Germination** | $0 - 2$ h | Days $0 - 10$ | $24.0$ ($20 - 28$) | $72.5$ ($65 - 80$) | $70.0$ ($60 - 80$) | $10,000$ |
| **Stage 2: Vegetative** | $2 - 8$ h | Days $10 - 40$ | $25.0$ ($21 - 27$) | $65.0$ ($55 - 75$) | $60.0$ ($50 - 70$) | $18,000$ |
| **Stage 3: Flowering** | $8 - 12$ h | Days $40 - 60$ | $23.5$ ($20 - 26$) | $62.5$ ($50 - 70$) | $58.0$ ($50 - 65$) | $22,500$ |
| **Stage 4: Fruit Dev.** | $12 - 18$ h | Days $60 - 90$ | $25.0$ ($21 - 28$) | $60.0$ ($50 - 70$) | $65.0$ ($55 - 75$) | $22,500$ |
| **Stage 5: Maturity** | $18 - 24$ h | Days $90 - 120$ | $22.5$ ($18 - 26$) | $55.0$ ($45 - 65$) | $55.0$ ($45 - 65$) | $20,000$ |

---

## Quantitative Benchmark Matrix

Performance indices evaluated from end-to-end 24-hour closed-loop simulations:

| Performance Metric | Multiloop PID | Pole Placement | Optimal LQR-I | Academic Assessment |
|---|---|---|---|---|
| **Total ISE (Tracking Error)** | $1.48 \times 10^8$ | $8.05 \times 10^8$ | **$1.58 \times 10^7$** | **LQR-I achieves 89.3% reduction in error variance** |
| **Total IAE** | $48,921.25$ | $118,523.58$ | **$5,748.36$** | **Optimal tracking with minimal deviation** |
| **Total Actuator Variation (TV)** | $16.25$ | **$8.06$** | $15.57$ | **Smooth duty cycle commands without actuator chattering** |
| **Mean Crop Health Quality** | $99.82\%$ | $98.38\%$ | **$99.96\%$** | **Near-perfect physiological health maintained** |
| **Linear vs Nonlinear Discrepancy**| $< 10^{-9}\%$ | $< 10^{-9}\%$ | $< 10^{-9}\%$ | **Analytical and numerical Jacobians fully verified** |

---

## Visualizations & Real-Time Dashboards

All figures are automatically displayed in real time and exported to [`results/`](results/) as high-resolution PNGs:

### 1. State Tracking Dashboard across Growth Phases
![State Tracking](results/comparison_state_tracking.png)

### 2. Actuator Duty Cycles & Anti-Windup Saturation
![Actuator Effort](results/comparison_actuator_effort.png)

### 3. Hierarchical Crop Health Quality (Environmental, Soil, Nutrient, Composite)
![Crop Quality](results/comparison_crop_quality.png)

### 4. Complex S-Plane Modal Map (Open-Loop vs. PP vs. LQR-I)
![S-Plane Pole Map](results/s_plane_pole_map.png)

### 5. Quantitative Performance Scorecard Bar Charts
![Performance Scorecard](results/performance_scorecard.png)

---

## Three-Phase Project Roadmap

* **Phase 1 (100% Accomplished)**: Mathematical modeling, equilibrium trim point, unbiased linearization, modal diagnostics, tri-hybrid controller synthesis, 24-hour diurnal simulations across 5 growth phases, and single-file master architecture.
* **Phase 2 (Upcoming - Physical Hardware Prototyping)**: Embedded microcontroller deployment (ESP32/STM32), physical sensor interfacing (SHT31, capacitive moisture, BH1750, pH/EC probes), high-power MOSFET/relay actuator driver boards, and Hardware-in-the-Loop (HIL) telemetry.
* **Phase 3 (Final Phase - Edge AI, Cloud IoT & Crop Trials)**: Extended Kalman Filter (EKF) observer for unmeasured vegetative biomass states, Non-linear Model Predictive Control (NMPC) for energy/water optimization, cloud dashboard (AWS IoT / ThingsBoard), and live biological tomato crop validation.

For the full milestone schedule and Gantt timeline, see **[`MID_SEM_PLAN.md`](MID_SEM_PLAN.md)**.  
For algorithmic step-by-step logic, see **[`PSEUDOCODE.md`](PSEUDOCODE.md)**.

---

## How to Run

1. Open MATLAB (R2020b or newer).
2. Set your current directory to `Control_System_Project`.
3. In the Command Window, run:
   ```matlab
   main
   ```
4. All 5 figure windows will render in real time on-screen, console diagnostics will print, and all result artifacts will be saved to the `results/` folder.

---

## Repository Structure

```
Control_System_Project/
├── main.m                      # Central Heart: Single-file complete project pipeline
├── PSEUDOCODE.md               # Algorithmic pseudocode and mathematical formulation
├── MID_SEM_PLAN.md             # Mid-Semester progress report and 3-phase engineering plan
├── README.md                   # Comprehensive project documentation (this file)
├── .gitignore                  # Git ignore rules for MATLAB temp files
└── results/                    # Auto-generated simulation outputs
    ├── comparison_state_tracking.png
    ├── comparison_actuator_effort.png
    ├── comparison_crop_quality.png
    ├── s_plane_pole_map.png
    ├── performance_scorecard.png
    └── master_simulation_results.mat
```
