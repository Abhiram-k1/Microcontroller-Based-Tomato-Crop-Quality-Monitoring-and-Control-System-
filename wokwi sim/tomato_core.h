// tomato_core.h
// Platform-independent core of the tomato crop controller, aligned with main.m
// (Microcontroller-Based-Tomato-Crop-Quality-Monitoring-and-Control-System) and
// with build_tomato_simulink_main.m:
//   - growth-stage setpoints and FAO / UC Davis ranges (db.phase in main.m)
//   - diurnal disturbance generator          (getDisturbancesLocal)
//   - 11-state nonlinear plant, Loamy soil   (Tomato_Dynamics)
//     forward-Euler step + physical bounds   (simulateClosedLoopLocal / enforcePhysicalBoundsLocal)
//   - 3 controllers with main.m gains: Decentralized PID, Pole Placement, LQR-I
//   - saturation + water-level pump interlock and Crop Condition Index (CCI)
// Time unit is HOURS everywhere (same as main.m).
#pragma once
#include <math.h>

namespace tomato {

// State order: [M T RH L pH EC N P K W VOC]
enum StateIndex { S_M = 0, S_T, S_RH, S_L, S_PH, S_EC, S_N, S_P, S_K, S_W, S_VOC, NX };
// Actuator order: [pump fan mist growLight]
enum ActIndex { A_PUMP = 0, A_FAN, A_MIST, A_LIGHT, NU };
// Controllers (main.m step 5)
enum CtrlType { CTRL_PID = 0, CTRL_PP, CTRL_LQRI, NCTRL };
static const char* const CTRL_NAME[NCTRL] = {"PID", "PP", "LQRI"};

static const float Ts   = 0.01f;            // main.m dt (h)
static const float PI_F = 3.14159265358979f;
static const float MIN_WATER = 10.0f;       // params.safety.minWaterLevel (%)

// main.m params.initialState
static const float X0[NX]      = {60, 25, 65, 15000, 6.35f, 2.4f, 130, 48, 180, 80, 100};
// main.m getSensorConfigLocal: noise sigma and sensor range
static const float SIGMA[NX]   = {0.50f, 0.20f, 0.80f, 50, 0.05f, 0.02f, 1.0f, 0.5f, 1.0f, 0.5f, 1.0f};
static const float SENS_LO[NX] = {0, -40, 0, 0,     0,  0,  0,    0,    0,    0, 0};
static const float SENS_HI[NX] = {100, 85, 100, 65535, 14, 10, 1999, 1999, 1999, 100, 500};
// main.m enforcePhysicalBoundsLocal
static const float LIM_LO[NX]  = {0, -10, 0, 0,      0,  0,  0,   0,   0,   0, 0};
static const float LIM_HI[NX]  = {100, 60, 100, 100000, 14, 15, 500, 500, 500, 100, 500};

// Loamy soil (main.m default selectedSoilType)
static const float SOIL_DRAIN_GAIN = 0.020f, SOIL_IRR_GAIN = 12.0f, SOIL_EVAP = 2.00f;
static const float SOIL_DRAIN_THR  = 50.0f,  SOIL_LEACH    = 1.00f;

// ---- main.m designPIDLocal: loops = [moisture; temperature; humidity; light]
static const float PID_KP[4] = {0.080f, 0.200f, 0.100f, 0.00005f};
static const float PID_KI[4] = {0.015f, 0.020f, 0.015f, 0.00001f};
static const float PID_KD[4] = {0.005f, 0.010f, 0.005f, 0.00001f};

// ---- main.m findTrimLocal (Loamy): u0 and x0 used by Pole Placement and LQR-I
static const float U0[NU]  = {0.15f, 0.4f, 0.2f, 0.0f};
static const float X0T[NX] = {60, 25, 65, 15000, 6.35697674f, 3.09615385f, 132.146179f,
                              53.1573705f, 193.975155f, 80, 100};

// ---- main.m designPolePlacementLocal (Loamy) : K (4x11)
static const float K_PP[NU][NX] = {
    {0.0483333333f, -0.000495726495f, 0, -1.28032681e-08f, 0, 0, 0, 0, 0, 0, 0},
    {0, -0.216666667f, 0, -1.65079379e-05f, 0, 0, 0, 0, 0, 0, 0},
    {0, -0.147135417f, 0.125f, -1.06150802e-05f, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 2.33333277e-05f, 0, 0, 0, 0, 0, 0, 0}};

// ---- main.m designLQRILocal (Loamy) : Kx (4x11) and Ki (4x4)
static const float K_LQ_X[NU][NX] = {
    {0.330459032f, -0.000411907084f, 1.70943597e-05f, -2.35615582e-09f, -0.00048730567f, 1.10423468e-05f,
     -4.82261765e-07f, 2.5341091e-06f, 2.14226478e-06f, -2.33895427e-06f, 0},
    {0.000120718129f, -0.985164275f, -0.0598830294f, -2.62215588e-06f, 8.28422466e-08f, -7.07571585e-06f,
     2.1496636e-07f, 5.96326007e-07f, 1.87591977e-07f, 2.86070424e-09f, 0},
    {1.12178067e-05f, -0.371269761f, 0.374265168f, -7.63411539e-07f, 4.73891231e-08f, -1.74940498e-06f,
     4.62547921e-08f, 1.28064853e-07f, 4.01913174e-08f, 8.31601969e-10f, 0},
    {-5.86313392e-06f, 0.0464893214f, -0.00536773738f, 0.000390023379f, -1.52581716e-07f, 4.40130187e-07f,
     -9.19215081e-07f, -2.6187991e-06f, -8.05514131e-07f, -8.74582938e-10f, 0}};
static const float K_LQ_Z[NU][4] = {
    {0.632455504f, 0.00042116229f, 4.07407507e-05f, -7.12435437e-09f},
    {0.000209844063f, -1.5711911f, -0.173577178f, 4.41550606e-05f},
    {6.03033087e-06f, -0.391637481f, 0.556055166f, 2.02211983e-05f},
    {-4.64245726e-06f, 0.0972116579f, -0.00783676747f, 0.00122323702f}};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }
inline float posf(float v)    { return v > 0.0f ? v : 0.0f; }

// ---------------------------------------------------------------- growth stage
struct Setpoints {
    float v[4];   // [moisture; temperature; humidity; light]
    int   stage;  // 1..5
};

inline Setpoints growthStage(float t) {   // main.m getPhaseIndexLocal + db.phase(*)._sp
    Setpoints s;
    if (t < 2)       { s.stage = 1; s.v[0] = 70; s.v[1] = 24;    s.v[2] = 72.5f; s.v[3] = 10000; }
    else if (t < 8)  { s.stage = 2; s.v[0] = 60; s.v[1] = 25;    s.v[2] = 65;    s.v[3] = 18000; }
    else if (t < 12) { s.stage = 3; s.v[0] = 58; s.v[1] = 23.5f; s.v[2] = 62.5f; s.v[3] = 22500; }
    else if (t < 18) { s.stage = 4; s.v[0] = 65; s.v[1] = 25;    s.v[2] = 60;    s.v[3] = 22500; }
    else             { s.stage = 5; s.v[0] = 55; s.v[1] = 22.5f; s.v[2] = 55;    s.v[3] = 20000; }
    return s;
}

// ---------------------------------------------------------------- disturbances
struct Dist { float extT, natL, evap; };

inline Dist disturbance(float t) {          // main.m getDisturbancesLocal
    float td   = fmodf(t, 24.0f);
    float extT = 23.5f + 5.5f * sinf(2.0f * PI_F * (td - 9.5f) / 24.0f);
    float natL = 0.0f;
    if (td >= 6.0f && td <= 19.5f) natL = fmaxf(0.0f, 22000.0f * sinf(PI_F * (td - 6.0f) / 13.5f));
    float evap = 0.35f + 0.60f * (natL / 22000.0f) + 0.03f * posf(extT - 22.0f);
    Dist d = {extT, natL, evap};
    return d;
}

// ---------------------------------------------------------------- plant
inline void plantDerivative(const float* x, const float* u, const Dist& d, float* dx) {  // main.m Tomato_Dynamics
    float pump  = clamp01(u[A_PUMP]);
    float fan   = clamp01(u[A_FAN]);
    float mist  = clamp01(u[A_MIST]);
    float light = clamp01(u[A_LIGHT]);
    if (x[S_W] < MIN_WATER) pump = 0.0f;

    const float M = x[S_M], T = x[S_T], L = x[S_L], pH = x[S_PH], EC = x[S_EC];
    const float N = x[S_N], P = x[S_P], K = x[S_K], W = x[S_W], VOC = x[S_VOC];
    const float evap = d.evap;

    float drainage     = SOIL_DRAIN_GAIN * posf(M - SOIL_DRAIN_THR);
    float moistureLoss = SOIL_EVAP * evap * (1.0f + 0.01f * posf(T - 25.0f));
    dx[S_M]  = SOIL_IRR_GAIN * pump - moistureLoss - drainage;
    dx[S_T]  = 0.15f * (d.extT - T) + 0.00008f * L - 3.0f * fan;
    dx[S_RH] = 8.0f * mist - 4.0f * fan - 0.15f * posf(T - 25.0f);
    dx[S_L]  = 0.5f * (d.natL - L) + 30000.0f * light;

    float transp = (0.50f + 0.50f * (L / 25000.0f)) * (1.0f + 0.02f * posf(T - 22.0f));
    dx[S_PH] = 0.04f * (6.35f - pH) + 0.02f * (6.45f - pH) * pump - 0.0002f * posf(N - 140.0f);
    dx[S_EC] = 0.55f * pump - 0.05f * (evap / 0.8f) - 0.03f * SOIL_LEACH * drainage * EC + 0.02f * (2.4f - EC);
    dx[S_N]  = 22.0f * pump - 3.8f * transp - 0.05f * SOIL_LEACH * drainage * (N / 100.0f) + 0.03f * (130.0f - N);
    dx[S_P]  = 7.5f * pump - 1.2f * transp - 0.02f * SOIL_LEACH * drainage * (P / 50.0f) + 0.02f * (48.0f - P);
    dx[S_K]  = 28.0f * pump - 4.8f * transp - 0.04f * SOIL_LEACH * drainage * (K / 150.0f) + 0.03f * (190.0f - K);

    float refillNominal = 0.5f * evap + 0.2f * 0.15f;            // main.m fallback (evaporation-dependent)
    float qRefill = 0.05f * (80.0f - W) + refillNominal;
    dx[S_W]   = qRefill - (0.5f * evap + 0.2f * pump);
    dx[S_VOC] = 0.05f * (100.0f - VOC) + 0.10f * posf(T - 30.0f);
}

// One forward-Euler step of length Ts followed by physical bounds (main.m loop).
inline void plantStep(float* x, const float* u, const Dist& d) {
    float dx[NX];
    plantDerivative(x, u, d, dx);
    for (int i = 0; i < NX; i++) x[i] = clampf(x[i] + dx[i] * Ts, LIM_LO[i], LIM_HI[i]);
}

// ---------------------------------------------------------------- controllers
struct Controller {
    CtrlType type;
    float integ[4];   // PID integrators
    float prevE[4];   // PID previous error (main.m starts at 0)
    float z[4];       // LQR-I integral states

    Controller() : type(CTRL_LQRI) { reset(); }
    void reset() {
        for (int i = 0; i < 4; i++) { integ[i] = 0.0f; prevE[i] = 0.0f; z[i] = 0.0f; }
    }

    // sp: 4 setpoints, y: 11 sensor values, u: 4 commands (main.m step 6 switch)
    void step(const float* sp, const float* y, float* u) {
        // errors from SENSED values; temperature reversed so the fan cools
        const float e[4] = {sp[0] - y[S_M], y[S_T] - sp[1], sp[2] - y[S_RH], sp[3] - y[S_L]};
        float xt[NX];
        for (int i = 0; i < NX; i++) xt[i] = X0T[i];
        for (int i = 0; i < 4; i++) xt[i] = sp[i];

        if (type == CTRL_PID) {                 // decentralized PID, clamping anti-windup
            for (int j = 0; j < 4; j++) {
                float uRaw = PID_KP[j] * e[j] + PID_KI[j] * integ[j] + PID_KD[j] * (e[j] - prevE[j]) / Ts;
                u[j] = clamp01(uRaw);
                if (!((uRaw >= 1.0f && e[j] > 0) || (uRaw <= 0.0f && e[j] < 0))) integ[j] += e[j] * Ts;
                prevE[j] = e[j];
            }
        } else if (type == CTRL_PP) {           // u = u0 - K (y - x*)
            for (int j = 0; j < NU; j++) {
                float acc = U0[j];
                for (int i = 0; i < NX; i++) acc -= K_PP[j][i] * (y[i] - xt[i]);
                u[j] = acc;                     // saturated by applySafety()
            }
        } else {                                // LQR-I: u = u0 - Kx (y - x*) - Ki z
            float uRaw[NU];
            for (int j = 0; j < NU; j++) {
                float acc = U0[j];
                for (int i = 0; i < NX; i++) acc -= K_LQ_X[j][i] * (y[i] - xt[i]);
                for (int i = 0; i < 4; i++)  acc -= K_LQ_Z[j][i] * z[i];
                uRaw[j] = acc;
                u[j] = clamp01(acc);
            }
            // anti-windup: freeze z[j] only if integrating would push u[j] further into
            // saturation (du/dz = -Ki(j,j); the temperature loop has the opposite sign)
            for (int j = 0; j < 4; j++) {
                float yErr = y[j] - sp[j];
                float push = -K_LQ_Z[j][j] * yErr;
                if (!((uRaw[j] >= 1.0f && push > 0) || (uRaw[j] <= 0.0f && push < 0))) z[j] += yErr * Ts;
            }
        }
    }
};

// ---------------------------------------------------------------- safety
// Saturation + pump cut-off when the tank level is below the minimum.
// main.m checks the TRUE tank level x(10); pass the plant state here.
inline bool applySafety(float* u, const float* x) {
    for (int i = 0; i < NU; i++) u[i] = clamp01(u[i]);
    bool lowWater = x[S_W] < MIN_WATER;
    if (lowWater) u[A_PUMP] = 0.0f;
    return lowWater;
}

// ---------------------------------------------------------------- crop condition index
struct Quality { float env, soil, nutr, total; bool vocWarn; };

// per stage: [T, RH, M, L, pH, EC, N, P, K] x [lo, hi]   (main.m db.phase(*)._range)
static const float RANGES[5][9][2] = {
    {{20,28},{65,80},{60,80},{5000,15000}, {6.0f,6.6f},{1.6f,2.5f},{90,140}, {30,55},{120,180}},
    {{21,27},{55,75},{50,70},{12000,25000},{6.0f,6.6f},{1.8f,2.7f},{115,165},{35,60},{140,200}},
    {{20,26},{50,70},{50,65},{15000,30000},{6.1f,6.7f},{2.0f,2.9f},{110,155},{40,68},{155,220}},
    {{21,28},{50,70},{55,75},{15000,30000},{6.1f,6.7f},{2.1f,3.1f},{105,150},{38,62},{165,230}},
    {{18,26},{45,65},{45,65},{12000,28000},{6.0f,6.6f},{2.2f,3.2f},{95,140}, {32,55},{150,210}}
};

inline float rangeScore(float v, const float* r) {   // main.m rSc
    float lo = r[0], hi = r[1];
    if (v >= lo && v <= hi) return 100.0f;
    if (v < lo) return fmaxf(0.0f, 100.0f - ((lo - v) / fmaxf(lo * 0.40f, 1.0f)) * 100.0f);
    return fmaxf(0.0f, 100.0f - ((v - hi) / fmaxf(hi * 0.40f, 1.0f)) * 100.0f);
}

inline Quality cropQuality(const float* y, int stage) {  // main.m calcQualityOnlineLocal
    if (stage < 1) stage = 1;
    if (stage > 5) stage = 5;
    const float (*R)[2] = RANGES[stage - 1];
    Quality q;
    q.env  = (rangeScore(y[S_T], R[0]) + rangeScore(y[S_RH], R[1]) +
              rangeScore(y[S_M], R[2]) + rangeScore(y[S_L], R[3])) / 4.0f;
    q.soil = (rangeScore(y[S_PH], R[4]) + rangeScore(y[S_EC], R[5])) / 2.0f;
    q.nutr = (rangeScore(y[S_N], R[6]) + rangeScore(y[S_P], R[7]) + rangeScore(y[S_K], R[8])) / 3.0f;
    q.total   = clampf(0.50f * q.env + 0.25f * q.soil + 0.25f * q.nutr, 0.0f, 100.0f);
    q.vocWarn = y[S_VOC] > 200.0f;   // extra alarm (not part of main.m CCI)
    return q;
}

}  // namespace tomato
