# PSEUDOCODE SPECIFICATION: 11-STATE TOMATO GREENHOUSE CLIMATE & CROP CONDITION SYSTEM

**Project Title:** Pure Base MATLAB Unified Microclimate & Crop Condition Monitoring System  
**Target Biological Plant:** Greenhouse Tomato (*Solanum lycopersicum*)  
**Execution Kernel:** Single-File Master Orchestration (`main.m`)  
**Hardware Node Target:** ESP32 Microcontroller with Multi-Sensor Suite  
**Authoritative Online Data Sources:** FAO Irrigation Paper 56, UC Davis VRIC Pub 7250, Wageningen UR  

---

## 1. High-Level Master Architecture

```text
========================================================================================
ALGORITHM 1: Master Orchestration Pipeline
========================================================================================
Input:  Initial State x0 in R^11, Diurnal Weather d(t) in R^3, Soil Type {'Sandy','Loamy','Clay'}
Output: Performance Scorecard, Real-Time Dashboards (5 Figures), Exported Artifacts (.mat, .png)

1:  INITIALIZE system parameters with Soil Physics (Sandy, Loamy, Clay)
2:  LOAD authoritative online botanical database (FAO-56, UC Davis, WUR)
3:  MAP physical hardware sensors to state vector (BME280, Capacitive v1.2, BH1750, pH, EC, NPK, MQ-135)
4:  COMPUTE complete 11-state equilibrium trim operating point (x0, u0*, d0) such that ||dx0||_inf ~ 0 across all states
5:  COMPUTE numerical Jacobians A = df/dx and B = df/du using forward differencing on u in [0, 1]
6:  PERFORM modal decomposition & dual observability analysis:
      - Open-loop eigenvalues: lambda = eig(A)
      - Controllability matrix Co = pureCtrb(A, B), rank(Co), Kalman staircase decomposition
      - Case A (4 Climate Sensors): C_primary in R^(4x11) -> rank(Ob) = 4/11
      - Case B (Full Dedicated Suite): C_full in R^(11x11) -> rank(Ob) = 11/11 (Fully Observable)
7:  SYNTHESIZE tri-hybrid control laws:
      - Decentralized Multi-Loop PID with anti-windup tracking clamping
      - Scaled Subspace Pole Placement via Sylvester solver on controllable modes
      - Optimal LQR with Integral Action (LQR-I) via Real Ordered Schur CARE solver
8:  FOR each controller in {Decentralized_PID, PolePlacement, LQRI} DO:
      - FOR step k = 1 to N DO:
          a. Enforce physical state bounds: 0 <= M, RH, Water <= 100%
          b. Sensor Layer: y_meas = clamp(x + noise, sensor_min, sensor_max)
          c. Compute tracking error using SENSED values: e = setpoint - y_meas
          d. Controller evaluates raw command: u_cmd = control_law(e)
          e. Actuator Saturation Operator: u_sat = clamp(u_cmd, 0.0, 1.0)
          f. Water-Level Safety Interlock: IF x_water < 10% THEN u_sat[1] = 0.0 (Pump Cutoff)
          g. Evaluate Crop Condition Index (CCI) from sensor measurements
          h. Forward integrate true plant physics: x = x + Tomato_Dynamics(x, u_actual, d) * dt
        END FOR
      - COMPUTE performance indices: Total ISE, Total IAE, Total Actuator Variation (TV)
9:  VALIDATE dynamic consistency via perturbation deviation comparison: delta_x_NL = x_NL - x_trim versus delta_x_L
10: RENDER 5 real-time figure windows with explicit growth phase demarcations (drawnow):
      - Fig 1: Multivariable State Tracking Dashboard
      - Fig 2: Actuator Duty Cycle Effort & Safety Interlock Saturation
      - Fig 3: Hierarchical Crop Condition Index (CCI)
      - Fig 4: Complex S-Plane Modal Map
      - Fig 5: Quantitative Performance Scorecard
11: EXPORT figures to results/*.png and save master simulation dataset to results/*.mat
12: TERMINATE execution with exit code 0
========================================================================================
```

---

## 2. Soil Physics & Nonlinear Plant Dynamics (`Tomato_Dynamics`)

The system models water retention, infiltration, and drainage based on substrate classification:
- **Sandy Soil**: Fast drainage ($k_{\text{drain}} = 0.045$), high infiltration ($\gamma = 14.0$), high evaporation ($k_{\text{evap}} = 2.60$).
- **Loamy Soil (Default)**: Balanced drainage ($k_{\text{drain}} = 0.020$), balanced infiltration ($\gamma = 12.0$), moderate evaporation ($k_{\text{evap}} = 2.00$).
- **Clay Soil**: Slow drainage ($k_{\text{drain}} = 0.008$), lower infiltration ($\gamma = 9.5$), high retention ($k_{\text{evap}} = 1.40$).

```text
========================================================================================
ALGORITHM 2: Nonlinear System Dynamic Derivative Function with Soil Physics
========================================================================================
Function dx = Tomato_Dynamics(x, u, d, params)
    1: Physical Actuator Saturation: u_sat = clamp(u, 0.0, 1.0)
    2: Water Safety Interlock: IF x[10] < params.safety.minWaterLevel THEN u_sat[1] = 0.0
    3: Extract soil parameters: soil = params.soil
    4: Unpack states: M = x[1], T = x[2], H = x[3], L = x[4], pH = x[5], EC = x[6],
                     N = x[7], P = x[8], K = x[9], Water = x[10], VOC = x[11]
    5: Unpack disturbances: extT = d[1], natL = d[2], evap = d[3]
    
    6: % 1. Soil Moisture Derivative (dM/dt) - Modulated by Soil Substrate Physics
       drainage = soil.drainageGain * max(M - soil.drainageThreshold, 0.0)
       moistureLoss = soil.evapFactor * evap * (1.0 + 0.01 * max(T - 25.0, 0.0))
       dM = soil.irrigationGain * u_sat[1] - moistureLoss - drainage

    7: % 2. Air Temperature Derivative (dT/dt)
       dT = 0.15 * (extT - T) + 0.00008 * L - 3.0 * u_sat[2]

    8: % 3. Relative Humidity Derivative (dH/dt)
       dH = 8.0 * u_sat[3] - 4.0 * u_sat[2] - 0.15 * max(T - 25.0, 0.0)

    9: % 4. Total Illumination Derivative (dL/dt)
       dL = 0.5 * (natL - L) + 30000.0 * u_sat[4]

   10: % 5. Soil pH Derivative (dpH/dt)
       dpH = 0.04 * (6.35 - pH) + 0.02 * (6.45 - pH) * u_sat[1] - 0.0002 * max(N - 140.0, 0.0)

   11: % 6. Soil EC Derivative (dEC/dt)
       dEC = 0.55 * u_sat[1] - 0.05 * (evap / 0.8) - 0.03 * leachFac * drainage * EC + 0.02 * (2.4 - EC)

   12: % 7-9. Soil Macronutrients (N, P, K) with Crop Transpiration Uptake & Leaching
       transpiration = (0.50 + 0.50 * (L / 25000.0)) * (1.0 + 0.02 * max(T - 22.0, 0.0))
       dN = 22.0 * u_sat[1] - 3.8 * transpiration - 0.05 * leachFac * drainage * (N / 100.0) + 0.03 * (130.0 - N)
       dP = 7.5  * u_sat[1] - 1.2 * transpiration - 0.02 * leachFac * drainage * (P / 50.0)  + 0.02 * (48.0  - P)
       dK = 28.0 * u_sat[1] - 4.8 * transpiration - 0.04 * leachFac * drainage * (K / 150.0) + 0.03 * (190.0 - K)

   13: % 10. Water Tank Level (dWater/dt) with Automated Float Replenishment Valve
       q_refill = 0.05 * (80.0 - Water) + refillNominal
       dWater = q_refill - (0.5 * evap + 0.2 * u_sat[1])

   14: % 11. VOC / Gas Index (dVOC/dt)
       dVOC = 0.05 * (100.0 - VOC) + 0.10 * max(T - 30.0, 0.0)

   15: Return dx = [dM; dT; dH; dL; dpH; dEC; dN; dP; dK; dWater; dVOC]
End Function
========================================================================================
```

---

## 3. Physical Sensor Layer & Measurement Transduction

```text
========================================================================================
ALGORITHM 3: Simulated ESP32 Hardware Sensor Layer
========================================================================================
Function y_meas = simulateSensorLayer(x_true, sensorConfig)
    Initialize y_meas in R^11
    FOR i = 1 to 11 DO
        % Generate Gaussian measurement noise according to hardware datasheet
        v_i = sensorConfig[i].noiseStd * randn()
        sensed = x_true[i] + v_i
        % Enforce physical transducer limits & ADC saturation
        y_meas[i] = clamp(sensed, sensorConfig[i].minVal, sensorConfig[i].maxVal)
    END FOR
    Return y_meas
End Function
========================================================================================
```

---

## 4. Physical State Bounds Enforcement

```text
========================================================================================
ALGORITHM 4: Physical State Bounding
========================================================================================
Function x_bounded = enforcePhysicalBounds(x)
    x[1]  = clamp(x[1],  0.0, 100.0)     % Soil Moisture cannot exceed 100% or drop < 0%
    x[2]  = clamp(x[2], -10.0, 60.0)     % Air Temperature (-10 to 60 deg C)
    x[3]  = clamp(x[3],  0.0, 100.0)     % Relative Humidity cannot exceed 100%
    x[4]  = max(x[4], 0.0)               % Illumination is strictly non-negative
    x[5]  = clamp(x[5],  0.0, 14.0)      % pH scale bounded to [0, 14]
    x[6]  = max(x[6], 0.0)               % Electrical Conductivity >= 0
    x[7]  = max(x[7], 0.0)               % Nitrogen >= 0
    x[8]  = max(x[8], 0.0)               % Phosphorus >= 0
    x[9]  = max(x[9], 0.0)               % Potassium >= 0
    x[10] = clamp(x[10], 0.0, 100.0)     % Water tank capacity bounded to [0, 100]%
    x[11] = max(x[11], 0.0)              % Gas/VOC index >= 0
    Return x_bounded
End Function
========================================================================================
```

---

## 5. Control Laws with Saturation & Anti-Windup

### 5.1 Decentralized Multi-Loop PID Control
```text
Loop 1 (Soil Moisture)    -> Capacitive Sensor (y1) -> Irrigation Pump (u1)
Loop 2 (Air Temperature)  -> BME280 Temp (y2)       -> Ventilation Fan (u2)
Loop 3 (Air Humidity)     -> BME280 RH (y3)         -> Misting System (u3)
Loop 4 (Total Light)      -> BH1750 Light (y4)      -> Supplemental Grow Light (u4)

Raw Command:
  u_cmd_j(k) = Kp_j * e_j(k) + Ki_j * int_j(k) + Kd_j * (e_j(k) - e_j(k-1)) / dt
Actuator Saturation Operator:
  u_sat_j(k) = clamp(u_cmd_j(k), 0.0, 1.0)
Water-Level Safety Interlock:
  IF j == 1 AND x_water < 10.0% THEN u_sat[1] = 0.0

Tracking Anti-Windup Logic:
  IF (u_cmd_j >= 1.0 AND e_j > 0) OR (u_cmd_j <= 0.0 AND e_j < 0) THEN
      int_j(k+1) = int_j(k)   % Freeze integrator during physical saturation
  ELSE
      int_j(k+1) = int_j(k) + e_j(k) * dt
  END IF
```

### 5.2 Scaled Subspace Pole Placement
```text
Feedback Law from Sensed Outputs:
  u_cmd = u0* - K * (y_meas - x_target)
Actuator Saturation Operator:
  u_sat = clamp(u_cmd, 0.0, 1.0)
Water-Level Safety Interlock:
  IF x_water < 10.0% THEN u_sat[1] = 0.0
```

### 5.3 Optimal LQR-I
```text
State Feedback + Error Integration:
  u_cmd = u0* - Kx * (y_meas - x_target) - Ki * z_int
Actuator Saturation Operator:
  u_sat = clamp(u_cmd, 0.0, 1.0)
Water-Level Safety Interlock:
  IF x_water < 10.0% THEN u_sat[1] = 0.0

Anti-Windup Integration on z_int:
  FOR j = 1 to 4 DO
      IF (u_cmd[j] >= 1.0 AND y_err[j] < 0) OR (u_cmd[j] <= 0.0 AND y_err[j] > 0) THEN
          z_int_j(k+1) = z_int_j(k)   % Freeze integrator channel during saturation
      ELSE
          z_int_j(k+1) = z_int_j(k) + y_err[j] * dt
      END IF
  END FOR
```

---

## 6. Crop Condition Index (CCI / CESI) Formulation

$$\text{CCI} = 0.50 \cdot \text{CCI}_{\text{environmental}} + 0.25 \cdot \text{CCI}_{\text{soil}} + 0.25 \cdot \text{CCI}_{\text{nutrient}}$$

Where each sub-index scores physiological suitability against FAO-56 and UC Davis ranges:
```text
Function score = rangeScore(val, low, high)
    IF low <= val <= high THEN
        Return 100.0
    ELSE IF val < low THEN
        Return max(0.0, 100.0 - ((low - val) / (low * 0.30)) * 100.0)
    ELSE
        Return max(0.0, 100.0 - ((val - high) / (high * 0.30)) * 100.0)
    END IF
End Function
```
