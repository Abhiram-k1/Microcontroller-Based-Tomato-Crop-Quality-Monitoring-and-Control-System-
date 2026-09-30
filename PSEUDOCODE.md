# PSEUDOCODE SPECIFICATION: 11-STATE TOMATO GREENHOUSE CLIMATE CONTROL SYSTEM

**Project Title:** Pure Base MATLAB Unified Climate and Crop Quality Control System  
**Target Crop:** Greenhouse Tomato (*Solanum lycopersicum*)  
**Architecture:** Single-File Master Orchestration (`main.m`)  
**Mathematical Kernel:** 100% Base MATLAB (Zero External Toolboxes, Zero Simulink Dependencies)  
**Authoritative Online Data Sources:** FAO Irrigation Paper 56, UC Davis VRIC Pub 7250, Wageningen UR Greenhouse Crop Models  

---

## 1. High-Level Pipeline Architecture

```text
========================================================================================
ALGORITHM 1: Master Orchestration Pipeline
========================================================================================
Input:  Initial State x0 in R^11, Diurnal Weather d(t) in R^3, Simulation Horizon T_end = 24 h
Output: Performance Scorecard, Real-Time Dashboards (5 Figures), Exported Artifacts (.mat, .png)

1:  INITIALIZE system parameters and load online botanical database (FAO-56, UC Davis, WUR)
2:  COMPUTE equilibrium trim operating point (x0, u0*, d0) such that ||dx0(1:4)||_inf ~ 0
3:  COMPUTE numerical Jacobians A = df/dx and B = df/du using forward differencing on u in [0, 1]
4:  PERFORM modal decomposition:
      - Open-loop eigenvalues: lambda = eig(A)
      - Controllability matrix Co = pureCtrb(A, B), rank(Co), Kalman staircase decomposition
      - Observability matrix Ob = pureObsv(A, C), rank(Ob)
5:  SYNTHESIZE tri-hybrid control laws:
      - Multiloop Decoupled PID with anti-windup tracking clamping
      - Scaled Subspace Pole Placement via Sylvester solver on controllable modes
      - Optimal LQR with Integral Action (LQR-I) via Real Ordered Schur CARE solver
6:  FOR each controller in {PID, PolePlacement, LQRI} DO:
      - EXECUTE unified 24-hour diurnal closed-loop simulation across 5 growth stages
      - EVALUATE hierarchical crop quality index at every time step
      - COMPUTE performance indices: Total ISE, Total IAE, Total Actuator Variation (TV)
7:  VALIDATE dynamic consistency via linear vs nonlinear step response comparison
8:  RENDER 5 real-time figure windows with explicit botanical growth phase demarcations:
      - Fig 1: Multivariable State Tracking Dashboard
      - Fig 2: Actuator Duty Cycle Effort & Saturation
      - Fig 3: Hierarchical Crop Health Quality
      - Fig 4: Complex S-Plane Modal Map
      - Fig 5: Quantitative Performance Scorecard
9:  EXPORT figures to results/*.png and save master simulation dataset to results/*.mat
10: TERMINATE execution with exit code 0
========================================================================================
```

---

## 2. Nonlinear Plant Dynamics (`Tomato_Dynamics`)

The system comprises 11 state variables, 4 control inputs, and 3 external disturbances:
- **States $x \in \mathbb{R}^{11}$:** 
  $x_1$: Soil Moisture (%), $x_2$: Air Temperature (°C), $x_3$: Relative Humidity (%), $x_4$: Total Illumination (lux),
  $x_5$: Soil pH, $x_6$: Soil EC (dS/m), $x_7$: Nitrogen (mg/kg), $x_8$: Phosphorus (mg/kg), $x_9$: Potassium (mg/kg),
  $x_{10}$: Water Tank Level (%), $x_{11}$: VOC / Gas Index.
- **Controls $u \in [0, 1]^4$:**
  $u_1$: Irrigation Pump, $u_2$: Ventilation Fan, $u_3$: Misting System, $u_4$: Supplemental Grow Light.
- **Disturbances $d \in \mathbb{R}^3$:**
  $d_1$: Ambient Temperature (°C), $d_2$: Natural Solar Irradiance (lux), $d_3$: Soil Evaporation Rate.

```text
========================================================================================
ALGORITHM 2: Nonlinear System Dynamic Derivative Function
========================================================================================
Function dx = Tomato_Dynamics(x, u, d, params)
    1: Enforce actuator limits: u_sat = clamp(u, 0.0, 1.0)
    2: Unpack states: M = x[1], T = x[2], H = x[3], L = x[4], pH = x[5], EC = x[6],
                     N = x[7], P = x[8], K = x[9], Water = x[10], VOC = x[11]
    3: Unpack disturbances: extT = d[1], natL = d[2], evap = d[3]
    
    4: % 1. Soil Moisture Derivative (dM/dt)
       drainage = 0.02 * max(M - 50.0, 0.0)
       loss = 2.0 * evap * (1.0 + 0.01 * max(T - 25.0, 0.0))
       dM = 12.0 * u_sat[1] - loss - drainage

    5: % 2. Air Temperature Derivative (dT/dt)
       dT = 0.15 * (extT - T) + 0.00008 * L - 3.0 * u_sat[2]

    6: % 3. Relative Humidity Derivative (dH/dt)
       dH = 8.0 * u_sat[3] - 4.0 * u_sat[2] - 0.15 * max(T - 25.0, 0.0)

    7: % 4. Total Illumination Derivative (dL/dt)
       dL = 0.5 * (natL - L) + 30000.0 * u_sat[4]

    8: % 5. Soil pH Derivative (dpH/dt)
       dpH = 0.01 * (6.4 - pH) + 0.002 * u_sat[1]

    9: % 6. Soil EC Derivative (dEC/dt)
       dEC = 0.02 * (2.8 - EC) * u_sat[1] - 0.005 * (EC - 2.0) * (1.0 - u_sat[1])

   10: % 7-9. Soil Macronutrients (N, P, K)
       dN = 0.015 * (120.0 - N) - 0.03 * u_sat[1]
       dP = 0.010 * (50.0  - P) - 0.01 * u_sat[1]
       dK = 0.015 * (170.0 - K) - 0.02 * u_sat[1]

   11: % 10. Water Tank Level (dWater/dt)
       dWater = -(0.5 * evap + 0.2 * u_sat[1])

   12: % 11. VOC / Gas Index (dVOC/dt)
       dVOC = 0.05 * (100.0 - VOC) + 0.10 * max(T - 30.0, 0.0)

   13: Return dx = [dM; dT; dH; dL; dpH; dEC; dN; dP; dK; dWater; dVOC]
End Function
========================================================================================
```

---

## 3. Stationary Operating Trim & Unbiased Linearization

```text
========================================================================================
ALGORITHM 3: Analytical Operating Trim Solver
========================================================================================
Function trim = findTrimLocal(params)
    1: x0 = params.initialState; d0 = [extT; natL; evap]
    2: M = x0[1]; T = x0[2]; L = x0[4]
    3: % Solve for u0* that exactly zeroes the environmental derivatives:
       u1* = (2.0 * evap * (1 + 0.01 * max(T - 25, 0)) + 0.02 * max(M - 50, 0)) / 12.0
       u2* = max(0, (0.15 * (extT - T) + 0.00008 * L) / 3.0)
       u3* = max(0, (4.0 * u2* + 0.15 * max(T - 25, 0)) / 8.0)
       u4* = (natL < L) ? min(1, max(0, (L - natL) / 30000.0)) : 0.0
    4: u0 = clamp([u1*; u2*; u3*; u4*], 0.0, 1.0)
    5: dx0 = Tomato_Dynamics(x0, u0, d0, params)
    6: trim.x0 = x0; trim.u0 = u0; trim.d0 = d0; trim.maxEnvDeriv = max(|dx0[1:4]|)
    7: Return trim
End Function

========================================================================================
ALGORITHM 4: Unbiased Numerical Linearization (Boundary Clamping Halving Fix)
========================================================================================
Function [A, B, C, D] = linearizeLocal(x0, u0, d0, params)
    1: h = 1e-6
    2: f0 = Tomato_Dynamics(x0, u0, d0, params)
    3: FOR i = 1 to 11 DO (Central Difference for States)
         xp = x0 + h * e_i;  xm = x0 - h * e_i
         A[:, i] = (Tomato_Dynamics(xp, u0, d0, params) - Tomato_Dynamics(xm, u0, d0, params)) / (2*h)
       END FOR
    4: FOR j = 1 to 4 DO (Unbiased Differencing for Actuators)
         IF u0[j] + h <= 1.0 THEN
             up = u0 + h * e_j
             B[:, j] = (Tomato_Dynamics(x0, up, d0, params) - f0) / h
         ELSE
             up = u0 - h * e_j
             B[:, j] = (f0 - Tomato_Dynamics(x0, up, d0, params)) / h
         END IF
       END FOR
    5: C = eye(11); D = zeros(11, 4)
    6: Return [A, B, C, D]
End Function
========================================================================================
```

---

## 4. Modal Diagnostics & Subspace Decomposition

```text
========================================================================================
ALGORITHM 5: Kalman Controllability & Staircase Decomposition
========================================================================================
Function [Co, rankCo, T, Acc, Bcc, Auu, r] = pureControllability(A, B)
    1: Co = [B, A*B, A^2*B, ..., A^(n-1)*B]
    2: Compute SVD of Co: [U, S, V] = svd(Co, 'econ')
    3: Determine rank r = count(diag(S) > tol)
    4: Form orthogonal similarity transformation T = [U[:, 1:r], NullComplement]
    5: Transformed matrices:
         A_bar = T' * A * T = [Acc, Acu; 0, Auu]
         B_bar = T' * B     = [Bcc; 0]
    6: Verify stabilizability: all eigenvalues of Auu must satisfy Re(lambda) <= 0
    7: Return [Co, rankCo, T, Acc, Bcc, Auu, r]
End Function
========================================================================================
```

---

## 5. Controller Syntheses

### 5.1 Decoupled Multiloop PID with Tracking Clamping
```text
Loop 1 (Soil Moisture)    -> Actuator: Irrigation Pump (u1)
Loop 2 (Air Temperature)  -> Actuator: Ventilation Fan (u2)
Loop 3 (Air Humidity)     -> Actuator: Misting System (u3)
Loop 4 (Total Light)      -> Actuator: Supplemental Grow Light (u4)

Control Law:
  u_raw_j(k) = Kp_j * e_j(k) + Ki_j * int_j(k) + Kd_j * (e_j(k) - e_j(k-1)) / dt
  u_sat_j(k) = clamp(u_raw_j(k), 0.0, 1.0)

Anti-Windup Clamping Logic:
  IF (u_raw_j >= 1.0 AND e_j > 0) OR (u_raw_j <= 0.0 AND e_j < 0) THEN
      int_j(k+1) = int_j(k)   % Freeze integrator during saturation
  ELSE
      int_j(k+1) = int_j(k) + e_j(k) * dt
  END IF
```

### 5.2 Scaled Subspace Pole Placement
```text
1: Partition active environmental subsystem: A_env = A[1:4, 1:4], B_env = B[1:4, :]
2: Assign desired stable poles: poles_des = [-0.60, -0.80, -1.00, -1.20]
3: Solve Sylvester Equation for Multi-Input Pole Placement:
     (A_env - lambda_j * I) * x_j = B_env * g_j
   Optimize G to minimize cond(X)
4: K_env = G * inv(X)
5: K = [K_env, zeros(4, 7)]
   Feedback law: u = clamp(u0* - K * (x - x_target), 0.0, 1.0)
```

### 5.3 Optimal LQR with Integral Action (LQR-I)
```text
1: Form Augmented State Space:
     x_a = [x; z] in R^15, where dz/dt = y_track - r = C_track * x - r
     A_a = [A, 0; C_track, 0], B_a = [B; 0]
2: Apply coordinate scaling transformation Sa to normalize engineering units
3: Extract controllable augmented subspace via Kalman staircase: (Aca, Bca)
4: Form continuous-time Hamiltonian matrix:
     Ham = [Aca, -Bca * inv(R) * Bca'; -Qca, -Aca']
5: Solve Continuous Algebraic Riccati Equation (CARE) via Real Ordered Schur Decomposition:
     [U_h, T_h] = schur(Ham, 'real'); [U_h, ~] = ordschur(U_h, T_h, 'lhp')
     P = U_h[n+1:2n, 1:n] * inv(U_h[1:n, 1:n])
6: Kca = inv(R) * Bca' * P
7: Transform back to physical units: Ka = [Kx, Ki]
8: Control law with anti-windup:
     u_raw = u0* - Kx * (x - x_target) - Ki * z_int
     u = clamp(u_raw, 0.0, 1.0)
     Update z_int only when actuator is not saturated in the direction of error
```

---

## 6. Real-Time Graphical Dashboards & Growth Phase Demarcations

```text
========================================================================================
ALGORITHM 6: Real-Time Plotting with Growth Phase Demarcation
========================================================================================
Function renderRealTimeDashboards(resultsPID, resultsPP, resultsLQRI, db)
    1: Define phase boundaries: transitions = [2.0, 8.0, 12.0, 18.0] hours
    2: For each Figure 1 to 5 DO:
         - Create figure with 'Visible', 'on'
         - Plot setpoints and controller trajectories
         - Draw vertical dashed demarcation lines at t = 2, 8, 12, 18 h
         - Render styled background text badges for:
           [Germination, Vegetative, Flowering, Fruit Dev., Maturity]
         - Call `drawnow` to immediately flush graphics pipeline to screen
       END FOR
    3: Export all figures to results/*.png via exportgraphics
End Function
========================================================================================
```
