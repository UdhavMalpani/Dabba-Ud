/*
  Project 8 - Stage 0 thrust stand logger (BOM item #38)
  Board  : Arduino Uno / Nano (ATmega328P, 5 V)
  Library: "HX711" by Bogdan Necula (bogde/HX711) - install via Library Manager. Servo is built in.

  Measures one Emax ECO II 2207 1700KV + 5x4.3x3 prop (open, or inside a printed duct + grilles) on 6S.
  Automated ramp 1000 -> 2000 us in 50 us (5 %) steps, 3.0 s settle, 20 HX711 samples averaged per step,
  then a gentle ramp-down. One CSV row per step, EXACT columns (parsed by src/analysis/thrust_stand_analyzer.py):

      config,tip_gap_mm,pwm_us,throttle_pct,thrust_g,voltage_v,current_a,power_w,rpm

  Every non-data line printed by this sketch starts with '#', so the analyzer can skip it.

  WIRING
    D9  -> ESC signal (single ESC or one channel of the BLS 60A 4-in-1; ESC GND to Arduino GND)
    D4  -> HX711 DOUT,  D5 -> HX711 SCK,  HX711 VCC 5 V, GND
    D2  -> optical RPM sensor output (open collector / TTL, active LOW pulse per blade pass), INPUT_PULLUP
    D3  -> ABORT push button to GND (normally open), INPUT_PULLUP, interrupt on FALLING edge
    A0  -> battery voltage divider midpoint (e.g. 100k top / 10k bottom -> ratio 11.0, 6S 25.2 V -> 2.29 V)
    A2  -> current sensor output (ACS758 hall sensor, or the ESC current pad - set scale with 'k')
    Keep the divider/current sensor grounds at the Arduino GND (single-point ground at the ESC negative).

  SERIAL COMMANDS (115200 baud, newline terminated; 'x' at the start of a line acts IMMEDIATELY)
    x               ABORT now (throttle -> 1000 us, latched until 'r')
    r               reset abort latch
    s               start automated sweep (requires: not aborted, ESC armed >= 3 s, V >= undervoltage limit)
    t               tare the load cell (no thrust, prop stopped)
    c <grams>       calibrate scale with a known mass on the cell (after 't')
    f <factor>      set HX711 calibration factor directly (negative flips thrust sign)
    l <label>       config label, e.g. "l duct_gap1.5"  (no commas, max 23 chars)
    g <mm>          tip gap in mm for the CSV (use 0 or NaN for open prop -> 'g -1' prints NaN)
    b <n>           RPM pulses per revolution (blade count for a blade-pass optical sensor; 1 for one tape mark)
    v <volts>       calibrate voltage divider: enter the true pack voltage measured with a multimeter
    k <mV/A> <mV0>  current sensor sensitivity and zero offset (ACS758-100U: 40 600, ACS758-200B: 10 2500,
                    SpeedyBee BLS ESC current pad "scale 400": 40 0)
    z               zero the current offset at 0 A (ESC disarmed / prop stopped)
    u <volts>       undervoltage cutoff (default 19.8 V for 6S; 13.2 for 4S)
    h               print CSV header
    ?               status / help

  SAFETY CUTOFFS (any -> immediate 1000 us + abort latch): button on D3, serial 'x',
    current > 45 A, |thrust| > 4500 g (5 kg cell), pack voltage < 19.8 V (3 consecutive readings).
  ALWAYS: prop guard / duct ring bolted down, stand clamped, nobody in the prop plane, LiPo in a bag nearby.
*/

#include <Servo.h>
#include <HX711.h>

// ---------------- Pins ----------------
const uint8_t PIN_ESC      = 9;
const uint8_t PIN_RPM      = 2;   // INT0
const uint8_t PIN_ABORT    = 3;   // INT1
const uint8_t PIN_HX_DOUT  = 4;
const uint8_t PIN_HX_SCK   = 5;
const uint8_t PIN_VBAT     = A0;
const uint8_t PIN_CURR     = A2;

// ---------------- Sweep settings ----------------
const int      PWM_MIN_US        = 1000;
const int      PWM_MAX_US        = 2000;
const int      PWM_STEP_US       = 50;      // 5 % of the 1000 us range
const uint32_t SETTLE_MS         = 3000UL;
const uint8_t  SAMPLES_PER_STEP  = 20;
const uint32_t ARM_TIME_MS       = 3000UL;  // ESC sees 1000 us for >= 3 s before a sweep
const uint32_t RAMPDOWN_DT_MS    = 150UL;   // 50 us every 150 ms on the way down
const uint32_t SAMPLE_TIMEOUT_MS = 6000UL;  // HX711 must deliver 20 samples within this time
const uint32_t SAFETY_DT_MS      = 20UL;    // 50 Hz safety check of V/I

// ---------------- Safety limits ----------------
const float CURRENT_LIMIT_A  = 45.0;
const float THRUST_LIMIT_G   = 4500.0;
float       undervoltageV    = 19.8;        // 6S, 3.30 V/cell; change with 'u'
const uint8_t UV_CONFIRM     = 3;           // consecutive low readings before cutoff (noise rejection)

// ---------------- Calibration (adjustable over serial; not stored in EEPROM) ----------------
float hxFactor      = 420.0;    // raw counts per gram - CALIBRATE with 'c'
float vDividerRatio = 11.0;     // (Rtop + Rbot) / Rbot
float adcRefV       = 5.0;      // Uno/Nano default AREF (measure the 5 V rail for best accuracy)
float currMvPerA    = 40.0;     // ACS758-100U
float currOffsetMv  = 600.0;    // ACS758-100U zero-current output
uint8_t pulsesPerRev = 3;       // tri-blade blade-pass sensor

char  configLabel[24] = "open_5x4.3x3";
float tipGapMm        = NAN;    // NaN = open prop

// ---------------- Objects / state ----------------
Servo esc;
HX711 scale;

enum State { ST_IDLE, ST_SETTLE, ST_SAMPLE, ST_RAMPDOWN, ST_ABORTED };
State state = ST_IDLE;
// Explicit prototype so the Arduino builder does not auto-generate one above the enum declaration
void enterState(State s);

volatile bool     abortFlag  = false;
volatile uint32_t rpmPulses  = 0;
volatile uint32_t lastPulseUs = 0;

int      pwmCmd        = PWM_MIN_US;
uint32_t stateStartMs  = 0;
uint32_t idleSinceMs   = 0;
uint32_t lastSafetyMs  = 0;
uint32_t lastRampMs    = 0;
uint8_t  uvCount       = 0;

// per-step accumulators
uint8_t  nSamples   = 0;
float    sumThrust  = 0, sumV = 0, sumI = 0;
uint32_t pulseStart = 0;
uint32_t sampleStartMs = 0;

float lastThrustG = 0;

char  rxBuf[40];
uint8_t rxLen = 0;

// ---------------- ISRs ----------------
void isrAbort() {
  abortFlag = true;
  esc.writeMicroseconds(PWM_MIN_US);   // Servo lib guards with cli/SREG - safe from ISR
}

void isrRpm() {
  uint32_t now = micros();
  // Reject glitches: 35k RPM x 3 blades = 1750 Hz -> 571 us period; ignore edges closer than 150 us
  if ((uint32_t)(now - lastPulseUs) > 150UL) {
    rpmPulses++;
    lastPulseUs = now;
  }
}

// ---------------- Helpers ----------------
float readVoltage() {
  long acc = 0;
  for (uint8_t i = 0; i < 4; i++) acc += analogRead(PIN_VBAT);
  float vPin = (acc / 4.0) * adcRefV / 1023.0;
  return vPin * vDividerRatio;
}

float readCurrentMv() {
  long acc = 0;
  for (uint8_t i = 0; i < 4; i++) acc += analogRead(PIN_CURR);
  return (acc / 4.0) * adcRefV * 1000.0 / 1023.0;
}

float readCurrent() {
  float a = (readCurrentMv() - currOffsetMv) / currMvPerA;
  return a;
}

uint32_t pulsesSnapshot() {
  noInterrupts();
  uint32_t p = rpmPulses;
  interrupts();
  return p;
}

void setPwm(int us) {
  if (abortFlag) us = PWM_MIN_US;
  pwmCmd = constrain(us, PWM_MIN_US, PWM_MAX_US);
  esc.writeMicroseconds(pwmCmd);
}

void printHeader() {
  Serial.println(F("config,tip_gap_mm,pwm_us,throttle_pct,thrust_g,voltage_v,current_a,power_w,rpm"));
}

void triggerAbort(const __FlashStringHelper *why) {
  abortFlag = true;
  esc.writeMicroseconds(PWM_MIN_US);
  pwmCmd = PWM_MIN_US;
  if (state != ST_ABORTED) {
    Serial.print(F("# ABORT: "));
    Serial.println(why);
  }
  state = ST_ABORTED;
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
}

void beginStepSampling() {
  nSamples = 0;
  sumThrust = 0; sumV = 0; sumI = 0;
  pulseStart = pulsesSnapshot();
  sampleStartMs = millis();
  enterState(ST_SAMPLE);
}

void printStatus() {
  Serial.print(F("# state=")); Serial.print((int)state);
  Serial.print(F(" pwm=")); Serial.print(pwmCmd);
  Serial.print(F(" V=")); Serial.print(readVoltage(), 2);
  Serial.print(F(" I=")); Serial.print(readCurrent(), 2);
  Serial.print(F(" thrust_g=")); Serial.print(lastThrustG, 1);
  Serial.print(F(" factor=")); Serial.print(hxFactor, 3);
  Serial.print(F(" label=")); Serial.print(configLabel);
  Serial.print(F(" gap=")); Serial.print(tipGapMm, 2);
  Serial.print(F(" ppr=")); Serial.print(pulsesPerRev);
  Serial.print(F(" uv=")); Serial.print(undervoltageV, 2);
  Serial.print(F(" abort=")); Serial.println(abortFlag ? 1 : 0);
  Serial.println(F("# cmds: x r s t c<g> f<k> l<label> g<mm> b<n> v<V> k<mV/A> <mV0> z u<V> h ?"));
}

// ---------------- Safety (called every loop) ----------------
void safetyCheck() {
  if (abortFlag && state != ST_ABORTED) {
    triggerAbort(F("button/serial"));
    return;
  }
  if (state == ST_IDLE || state == ST_ABORTED) return;

  uint32_t now = millis();
  if (now - lastSafetyMs < SAFETY_DT_MS) return;
  lastSafetyMs = now;

  float i = readCurrent();
  if (i > CURRENT_LIMIT_A) { triggerAbort(F("overcurrent > 45 A")); return; }

  float v = readVoltage();
  if (v < undervoltageV) {
    if (++uvCount >= UV_CONFIRM) { triggerAbort(F("undervoltage")); return; }
  } else {
    uvCount = 0;
  }
  if (fabs(lastThrustG) > THRUST_LIMIT_G) { triggerAbort(F("thrust overload > 4500 g")); return; }
}

// ---------------- Serial command handling ----------------
void handleCommand(char *line) {
  while (*line == ' ') line++;
  char cmd = line[0];
  char *arg = line + 1;
  while (*arg == ' ') arg++;

  switch (cmd) {
    case 'r':
      if (digitalRead(PIN_ABORT) == LOW) { Serial.println(F("# abort button still pressed")); break; }
      if (state == ST_ABORTED || abortFlag) {
        abortFlag = false;
        setPwm(PWM_MIN_US);
        idleSinceMs = millis();
        enterState(ST_IDLE);
        Serial.println(F("# abort latch cleared; ESC at 1000 us, re-arming 3 s"));
      }
      break;
    case 's':
      if (state != ST_IDLE) { Serial.println(F("# busy or aborted ('r' to reset)")); break; }
      if (millis() - idleSinceMs < ARM_TIME_MS) { Serial.println(F("# wait: ESC arming (1000 us for 3 s)")); break; }
      if (readVoltage() < undervoltageV) { Serial.println(F("# refused: pack voltage below cutoff / not connected")); break; }
      Serial.print(F("# sweep start: ")); Serial.println(configLabel);
      printHeader();
      uvCount = 0;
      setPwm(PWM_MIN_US);
      enterState(ST_SETTLE);
      break;
    case 't':
      if (state != ST_IDLE) { Serial.println(F("# tare only when idle")); break; }
      Serial.println(F("# taring..."));
      scale.tare(20);
      Serial.println(F("# tare done"));
      break;
    case 'c': {
      if (state != ST_IDLE) { Serial.println(F("# calibrate only when idle")); break; }
      float grams = atof(arg);
      if (grams <= 0) { Serial.println(F("# usage: c <known mass g>")); break; }
      scale.set_scale(1.0);
      float raw = scale.get_units(20);   // tare-corrected raw counts
      if (fabs(raw) < 1.0) { Serial.println(F("# no load detected")); scale.set_scale(hxFactor); break; }
      hxFactor = raw / grams;
      scale.set_scale(hxFactor);
      Serial.print(F("# new factor = ")); Serial.println(hxFactor, 4);
      break;
    }
    case 'f':
      if (atof(arg) != 0) { hxFactor = atof(arg); scale.set_scale(hxFactor); }
      Serial.print(F("# factor = ")); Serial.println(hxFactor, 4);
      break;
    case 'l': {
      uint8_t j = 0;
      for (uint8_t k = 0; arg[k] != '\0' && j < sizeof(configLabel) - 1; k++) {
        char ch = arg[k];
        if (ch == ',' || ch == '\r' || ch == '\n') continue;
        configLabel[j++] = (ch == ' ') ? '_' : ch;
      }
      configLabel[j] = '\0';
      Serial.print(F("# label = ")); Serial.println(configLabel);
      break;
    }
    case 'g':
      tipGapMm = atof(arg);
      if (tipGapMm < 0) tipGapMm = NAN;
      Serial.print(F("# tip gap mm = ")); Serial.println(tipGapMm, 2);
      break;
    case 'b': {
      int n = atoi(arg);
      if (n >= 1 && n <= 8) pulsesPerRev = (uint8_t)n;
      Serial.print(F("# pulses/rev = ")); Serial.println(pulsesPerRev);
      break;
    }
    case 'v': {
      float trueV = atof(arg);
      float vPin = readVoltage() / vDividerRatio;
      if (trueV > 1.0 && vPin > 0.05) vDividerRatio = trueV / vPin;
      Serial.print(F("# divider ratio = ")); Serial.println(vDividerRatio, 4);
      break;
    }
    case 'k': {
      char *endp;
      float s = strtod(arg, &endp);
      float o = strtod(endp, NULL);
      if (s > 0) { currMvPerA = s; currOffsetMv = o; }
      Serial.print(F("# current mV/A = ")); Serial.print(currMvPerA, 2);
      Serial.print(F(" offset mV = ")); Serial.println(currOffsetMv, 1);
      break;
    }
    case 'z':
      if (state != ST_IDLE) { Serial.println(F("# zero only when idle")); break; }
      currOffsetMv = readCurrentMv();
      Serial.print(F("# current offset mV = ")); Serial.println(currOffsetMv, 1);
      break;
    case 'u':
      if (atof(arg) > 0) undervoltageV = atof(arg);
      Serial.print(F("# undervoltage cutoff V = ")); Serial.println(undervoltageV, 2);
      break;
    case 'h':
      printHeader();
      break;
    case '?':
      printStatus();
      break;
    default:
      if (cmd != '\0') Serial.println(F("# unknown command ('?' for help)"));
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    // 'x' as the FIRST character of a line aborts immediately (no newline needed).
    // Mid-line 'x' is kept so labels like "open_5x4.3x3" do not abort.
    if ((ch == 'x' || ch == 'X') && rxLen == 0) {
      triggerAbort(F("serial x"));
      continue;
    }
    if (ch == '\n' || ch == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        handleCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    }
  }
}

// ---------------- Load cell polling (non-blocking: read only when a conversion is ready) ----------------
bool pollLoadCell(float &grams) {
  if (!scale.is_ready()) return false;
  grams = scale.get_units(1);   // data is ready -> returns in < 1 ms
  lastThrustG = grams;
  return true;
}

// ---------------- Sweep state machine ----------------
void emitRow() {
  float n = (float)nSamples;
  float thrust = sumThrust / n;
  float v = sumV / n;
  float i = sumI / n;
  if (i < 0) i = 0;
  uint32_t dtMs = millis() - sampleStartMs;
  uint32_t pulses = pulsesSnapshot() - pulseStart;
  float rpm = (dtMs > 0) ? (pulses * 60000.0 / ((float)dtMs * pulsesPerRev)) : 0.0;
  float throttlePct = (pwmCmd - PWM_MIN_US) * 100.0 / (PWM_MAX_US - PWM_MIN_US);

  Serial.print(configLabel);      Serial.print(',');
  if (isnan(tipGapMm)) Serial.print(F("NaN")); else Serial.print(tipGapMm, 2);
  Serial.print(',');
  Serial.print(pwmCmd);           Serial.print(',');
  Serial.print(throttlePct, 1);   Serial.print(',');
  Serial.print(thrust, 1);        Serial.print(',');
  Serial.print(v, 3);             Serial.print(',');
  Serial.print(i, 3);             Serial.print(',');
  Serial.print(v * i, 2);         Serial.print(',');
  Serial.println(rpm, 0);
}

void runStateMachine() {
  uint32_t now = millis();
  float g;

  switch (state) {
    case ST_IDLE:
    case ST_ABORTED:
      // keep ESC disarmed-safe at 1000 us; keep thrust reading fresh for '?'
      esc.writeMicroseconds(PWM_MIN_US);
      pwmCmd = PWM_MIN_US;
      pollLoadCell(g);
      break;

    case ST_SETTLE:
      pollLoadCell(g);                       // keeps overload check live while settling
      if (now - stateStartMs >= SETTLE_MS) beginStepSampling();
      break;

    case ST_SAMPLE:
      if (pollLoadCell(g)) {
        sumThrust += g;
        sumV += readVoltage();
        sumI += readCurrent();
        nSamples++;
      }
      if (nSamples >= SAMPLES_PER_STEP) {
        emitRow();
        if (pwmCmd + PWM_STEP_US > PWM_MAX_US) {
          Serial.println(F("# sweep complete - ramping down"));
          lastRampMs = now;
          enterState(ST_RAMPDOWN);
        } else {
          setPwm(pwmCmd + PWM_STEP_US);
          enterState(ST_SETTLE);
        }
      } else if (now - sampleStartMs > SAMPLE_TIMEOUT_MS) {
        triggerAbort(F("HX711 timeout (check wiring)"));
      }
      break;

    case ST_RAMPDOWN:
      pollLoadCell(g);
      if (now - lastRampMs >= RAMPDOWN_DT_MS) {
        lastRampMs = now;
        if (pwmCmd - PWM_STEP_US <= PWM_MIN_US) {
          setPwm(PWM_MIN_US);
          Serial.println(F("# done - ESC at 1000 us"));
          idleSinceMs = now;
          enterState(ST_IDLE);
        } else {
          setPwm(pwmCmd - PWM_STEP_US);
        }
      }
      break;
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);

  esc.attach(PIN_ESC, PWM_MIN_US, PWM_MAX_US);
  esc.writeMicroseconds(PWM_MIN_US);         // arm ESC at minimum throttle
  idleSinceMs = millis();

  pinMode(PIN_ABORT, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ABORT), isrAbort, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRpm, FALLING);

  scale.begin(PIN_HX_DOUT, PIN_HX_SCK);
  scale.set_scale(hxFactor);
  if (scale.wait_ready_timeout(1000)) {
    scale.tare(20);
    Serial.println(F("# HX711 ready, tared"));
  } else {
    Serial.println(F("# WARNING: HX711 not found - check DOUT D4 / SCK D5"));
  }

  Serial.println(F("# Project 8 Stage 0 thrust stand logger"));
  Serial.println(F("# Set label (l), tip gap (g), tare (t), calibrate (c), then 's'. 'x' or D3 button aborts."));
  printStatus();
}

void loop() {
  pollSerial();
  safetyCheck();
  runStateMachine();
  safetyCheck();
}
