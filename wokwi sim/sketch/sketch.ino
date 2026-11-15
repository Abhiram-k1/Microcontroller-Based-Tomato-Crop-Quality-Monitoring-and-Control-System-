// =============================================================================
//  TOMATO CROP MONITORING & AUTOMATIC CONTROL  -  Wokwi / ESP32
//  Aligned with main.m and build_tomato_simulink_main.m
//  (11 states, 4 actuators, 5 growth stages, Loamy soil, 3 controllers)
//
//  MODE button (GPIO18):
//    SIM  closed loop: firmware runs the main.m nonlinear plant + controller
//    HW   all 11 channels come from Wokwi parts: DHT22 (T, RH), LDR (lux) and
//         8 potentiometers (moisture, pH, EC, N, P, K, tank level, VOC); the
//         controller reacts to them. Wokwi has no capacitive-moisture, pH, EC,
//         RS485-NPK, float or MQ-135 part, so potentiometers stand in for them.
//  CTRL button (GPIO19): cycles Decentralized PID -> Pole Placement -> LQR-I
//
//  Time is compressed: one control step (Ts = 0.01 h) every LOOP_MS.
//  With LOOP_MS = 50  ->  1 simulated hour = 5 s,  24 h = 120 s.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "tomato_core.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

using namespace tomato;

// ------------------------------ user options --------------------------------
#define LOOP_MS            50      // real ms per control step
#define NOISE_ON           1       // 1 = Gaussian sensor noise in SIM mode
#define PRINT_EVERY_TICKS  25      // serial log every 0.25 simulated hours
#define START_IN_SIM_MODE  true
#define START_CONTROLLER   CTRL_LQRI   // CTRL_PID, CTRL_PP or CTRL_LQRI

// ------------------------------ pins ----------------------------------------
const uint8_t PIN_DHT      = 16;
const uint8_t PIN_MOIST    = 34;   // pot  -> soil moisture 0..100 %
const uint8_t PIN_LDR      = 35;   // LDR module AO -> lux
const uint8_t PIN_PH       = 32;   // pot  -> pH 4..9
const uint8_t PIN_EC       = 33;   // pot  -> EC 0..5 mS/cm
const uint8_t PIN_TANK     = 36;   // pot  -> water tank level 0..100 %
const uint8_t PIN_VOC      = 39;   // pot  -> VOC 0..500
const uint8_t PIN_N        = 13;   // pot  -> nitrogen   0..260 mg/kg (stands in for RS485 NPK, N)
const uint8_t PIN_P        = 4;    // pot  -> phosphorus 0..100 mg/kg (NPK, P)
const uint8_t PIN_K        = 14;   // pot  -> potassium  0..380 mg/kg (NPK, K)
const uint8_t PIN_BTN      = 18;   // mode button (to GND)
const uint8_t PIN_BTN_CTRL = 19;   // controller-select button (to GND)
const uint8_t PIN_ACT[NU]  = {25, 26, 27, 17};   // pump, fan, mist, grow light (PWM LEDs)
const uint8_t PIN_LOWWATER = 23;   // red LED
const uint8_t PIN_HIGHVOC  = 5;    // orange LED

DHT dht(PIN_DHT, DHT22);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ------------------------------ state ---------------------------------------
float x[NX];            // true / model state
float y[NX];            // what the controller sees
float u[NU];            // safe actuator commands
Controller ctrl;
bool  simMode = START_IN_SIM_MODE;
float simTime = 0.0f;   // hours
uint32_t nextTick = 0;
uint32_t tickCount = 0;
uint32_t lastBtnMs = 0;
uint32_t lastCtrlBtnMs = 0;
uint32_t lastDisplayMs = 0;

// ------------------------------ helpers -------------------------------------
float gauss() {
    float u1 = random(1, 10000) / 10000.0f;
    float u2 = random(0, 10000) / 10000.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

float adcFrac(uint8_t pin) { return analogRead(pin) / 4095.0f; }

// Wokwi photoresistor module (3.3 V, 12-bit ADC) -> lux
float readLux() {
    int a = analogRead(PIN_LDR);
    if (a <= 1)    return 65535.0f;
    if (a >= 4094) return 0.1f;
    const float GAMMA = 0.7f, RL10 = 50.0f;
    float v = a / 4095.0f * 3.3f;
    float resistance = 2000.0f * v / (1.0f - v / 3.3f);
    float lux = powf(RL10 * 1e3f * powf(10.0f, GAMMA) / resistance, 1.0f / GAMMA);
    if (!isfinite(lux)) lux = 0.0f;
    return clampf(lux, 0.0f, 65535.0f);
}

// Read the real (Wokwi) sensors into the matching state slots.
void readHardwareInto(float* s) {
    static float lastT = 25.0f, lastH = 65.0f;
    static uint32_t lastDht = 0;
    if (millis() - lastDht >= 2000) {          // DHT22 needs >= 2 s between reads
        lastDht = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) lastT = t;
        if (!isnan(h)) lastH = h;
    }
    s[S_T]   = lastT;
    s[S_RH]  = lastH;
    s[S_M]   = adcFrac(PIN_MOIST) * 100.0f;
    s[S_L]   = readLux();
    s[S_PH]  = 4.0f + adcFrac(PIN_PH) * 5.0f;
    s[S_EC]  = adcFrac(PIN_EC) * 5.0f;
    s[S_W]   = adcFrac(PIN_TANK) * 100.0f;
    s[S_VOC] = adcFrac(PIN_VOC) * 500.0f;
    s[S_N]   = adcFrac(PIN_N) * 260.0f;    // mid position = 130 mg/kg
    s[S_P]   = adcFrac(PIN_P) * 100.0f;    // mid position = 50 mg/kg
    s[S_K]   = adcFrac(PIN_K) * 380.0f;    // mid position = 190 mg/kg
}

void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    for (int i = 0; i < NU; i++) ledcAttach(PIN_ACT[i], 5000, 8);
#else
    for (int i = 0; i < NU; i++) { ledcSetup(i, 5000, 8); ledcAttachPin(PIN_ACT[i], i); }
#endif
}

void pwmWriteAct(int i, float v) {
    int duty = (int)(clamp01(v) * 255.0f + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_ACT[i], duty);
#else
    ledcWrite(i, duty);
#endif
}

void resetSystem() {
    for (int i = 0; i < NX; i++) x[i] = X0[i];
    for (int i = 0; i < NX; i++) y[i] = X0[i];
    for (int i = 0; i < NU; i++) u[i] = 0.0f;
    ctrl.reset();
    simTime = 0.0f;
    tickCount = 0;
}

void drawDisplay(const Setpoints& sp, const Quality& q, bool lowWater) {
    char b[24];
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    snprintf(b, sizeof(b), "%s %-4s %5.2fh S%d", simMode ? "SIM" : "HW ", CTRL_NAME[ctrl.type], simTime, sp.stage);
    display.setCursor(0, 0);  display.print(b);
    snprintf(b, sizeof(b), "M%4.1f>%2.0f T%4.1f>%4.1f", y[S_M], sp.v[0], y[S_T], sp.v[1]);
    display.setCursor(0, 8);  display.print(b);
    snprintf(b, sizeof(b), "H%4.1f>%4.1f L%5.0f", y[S_RH], sp.v[2], y[S_L]);
    display.setCursor(0, 16); display.print(b);
    snprintf(b, sizeof(b), "pH%4.2f EC%3.1f W%3.0f", y[S_PH], y[S_EC], y[S_W]);
    display.setCursor(0, 24); display.print(b);
    snprintf(b, sizeof(b), "N%3.0f P%2.0f K%3.0f V%3.0f", y[S_N], y[S_P], y[S_K], y[S_VOC]);
    display.setCursor(0, 32); display.print(b);
    snprintf(b, sizeof(b), "P%3.0f F%3.0f M%3.0f L%3.0f", u[0] * 100, u[1] * 100, u[2] * 100, u[3] * 100);
    display.setCursor(0, 40); display.print(b);
    snprintf(b, sizeof(b), "Q E%3.0f S%3.0f N%3.0f", q.env, q.soil, q.nutr);
    display.setCursor(0, 48); display.print(b);
    snprintf(b, sizeof(b), "TOTAL%5.1f %s%s", q.total, lowWater ? "LOWH2O " : "", q.vocWarn ? "VOC!" : "");
    display.setCursor(0, 56); display.print(b);
    display.display();
}

void logSerial(const Setpoints& sp, const Quality& q, bool lowWater) {
    char b[320];
    snprintf(b, sizeof(b),
             "%.2f,%s,%s,%d,%.2f,%.2f,%.2f,%.0f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%d,%d",
             simTime, simMode ? "SIM" : "HW", CTRL_NAME[ctrl.type], sp.stage,
             y[S_M], y[S_T], y[S_RH], y[S_L], y[S_PH], y[S_EC], y[S_N], y[S_P], y[S_K], y[S_W], y[S_VOC],
             u[0], u[1], u[2], u[3], q.env, q.soil, q.nutr, q.total, lowWater ? 1 : 0, q.vocWarn ? 1 : 0);
    Serial.println(b);
}

// ------------------------------ setup / loop --------------------------------
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    randomSeed(42);

    pinMode(PIN_BTN, INPUT_PULLUP);
    pinMode(PIN_BTN_CTRL, INPUT_PULLUP);
    ctrl.type = START_CONTROLLER;
    pinMode(PIN_LOWWATER, OUTPUT);
    pinMode(PIN_HIGHVOC, OUTPUT);
    pwmSetup();
    dht.begin();

    Wire.begin(21, 22);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 not found (continuing without display)");
    }

    resetSystem();
    Serial.println("t_h,mode,ctrl,stage,M,T,RH,L,pH,EC,N,P,K,W,VOC,pump,fan,mist,light,qEnv,qSoil,qNutr,qTotal,lowWater,highVOC");
    nextTick = millis();
}

void loop() {
    uint32_t now = millis();

    // mode button (debounced)
    if (digitalRead(PIN_BTN) == LOW && now - lastBtnMs > 300) {
        lastBtnMs = now;
        simMode = !simMode;
        resetSystem();
        Serial.println(simMode ? "# Mode: SIM (closed-loop model)" : "# Mode: HW (Wokwi sensors)");
    }
    // controller button (debounced): PID -> PP -> LQR-I, restarts the 24 h run
    if (digitalRead(PIN_BTN_CTRL) == LOW && now - lastCtrlBtnMs > 300) {
        lastCtrlBtnMs = now;
        ctrl.type = (CtrlType)((ctrl.type + 1) % NCTRL);
        resetSystem();
        Serial.print("# Controller: "); Serial.println(CTRL_NAME[ctrl.type]);
    }

    if ((int32_t)(now - nextTick) < 0) return;
    nextTick += LOOP_MS;
    if ((int32_t)(now - nextTick) > 5 * LOOP_MS) nextTick = now + LOOP_MS;   // resync if we fell behind

    // 1. setpoints for the current growth stage
    Setpoints sp = growthStage(simTime);

    // 2. sensors
    if (simMode) {
        for (int i = 0; i < NX; i++) {
            float n = NOISE_ON ? SIGMA[i] * gauss() : 0.0f;
            y[i] = clampf(x[i] + n, SENS_LO[i], SENS_HI[i]);
        }
    } else {
        readHardwareInto(x);                         // all 11 sensors overwrite the states
        for (int i = 0; i < NX; i++) y[i] = x[i];
    }

    // 3. controller + safety (main.m: interlock on the true tank level)
    ctrl.step(sp.v, y, u);
    bool lowWater = applySafety(u, x);

    // 4. quality
    Quality q = cropQuality(y, sp.stage);

    // 5. outputs
    for (int i = 0; i < NU; i++) pwmWriteAct(i, u[i]);
    digitalWrite(PIN_LOWWATER, lowWater ? HIGH : LOW);
    digitalWrite(PIN_HIGHVOC,  q.vocWarn ? HIGH : LOW);

    // 6. plant advances: forward Euler as in main.m (in HW mode it is overwritten next tick)
    plantStep(x, u, disturbance(simTime));
    simTime += Ts;
    tickCount++;

    // 7. reporting
    if (tickCount % PRINT_EVERY_TICKS == 0) logSerial(sp, q, lowWater);
    if (now - lastDisplayMs >= 250) { lastDisplayMs = now; drawDisplay(sp, q, lowWater); }

    // repeat the 24 h demo cycle
    if (simTime >= 24.0f) {
        Serial.println("# 24 h cycle complete - restarting");
        resetSystem();
    }
}
