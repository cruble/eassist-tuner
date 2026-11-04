/*
  assist_bench.ino  —  first closed loop: hand on the bar, torque at the wheel
  ESP32-WROOM-32  +  HX711 + bar load cell  <->  Flipsky Mini V6 MK5 (sensored)

  WHAT THIS IS
  The simplest thing that can possibly work. Force in, current out, one
  direction, hard limits everywhere. It is NOT the four-quadrant law --
  there is no speed sign logic, no launch/cruise gain switch, no taper.
  Those come after we know a human in the loop feels sane.

  WHAT IT DOES
      newtons  ->  deadband  ->  gain  ->  rate limit  ->  current
  and a hard ERPM ceiling that zeroes the command if the wheel runs away.

  --------------------------------------------------------------------------
  BEFORE YOU UPLOAD -- ALL FIVE

    [ ] Wheel OFF the bench, secured, free to spin, nothing in its path
    [ ] ESC power button within reach. That is the abort.
    [ ] App to Use = UART
    [ ] Load cell calibrated and SAVED to flash (handle_cal, 's')
    [ ] You have read the runaway note below

  THE RUNAWAY NOTE
  Current control on an unloaded wheel accelerates until it runs out of
  voltage. On the chair, a person and the ground are the load. On a bench
  there is nothing, so even a small current winds the wheel up fast. That
  is why ERPM_CEILING exists, and why it starts low.

  --------------------------------------------------------------------------
  WIRING   (all of it already on the bench)

    HX711   VCC->3V3   GND->GND   DT->D18   SCK->D19
    LEDs    D25 green (push)   D26 yellow (over ceiling)
    ESC     COMM pin 3 -> D16 (RX2)   pin 4 -> D17 (TX2)   pin 6 -> GND

  SERIAL  115200

  COMMANDS
    a           arm
    x           STOP -- zero current, disarm
    k <A/N>     gain, amps per newton      (default 0.10)
    m <A>       max current                (default 1.5, hard cap 3.0)
    e <erpm>    speed ceiling              (default 2000 = 4.1 km/h)
    b <N>       deadband in newtons        (default 4.0)
    t           tare the load cell
    d           toggle the live stream
    h           help

  START ARMED WITH THE DEFAULTS AND PUSH GENTLY. Watch the numbers before
  you trust your hand.
  --------------------------------------------------------------------------
*/

#include "HX711.h"
#include <Preferences.h>

// ---------------- pins ----------------
const int PIN_DT      = 18;
const int PIN_SCK     = 19;
const int PIN_LED_FWD = 25;
const int PIN_LED_CAP = 26;
const int PIN_RX      = 16;
const int PIN_TX      = 17;

#define VESC Serial2
const long VESC_BAUD = 115200;

const uint8_t COMM_GET_VALUES  = 4;
const uint8_t COMM_SET_CURRENT = 6;

// ---------------- limits ----------------
const float I_HARD_CAP   = 3.0f;      // nothing may exceed this, ever
const float RAMP_A_PER_S = 4.0f;      // how fast current may change
const unsigned long SEND_MS = 20;     // 50 Hz, well inside the ESC's 1 s timeout
const unsigned long POLL_MS = 200;    // how often we ask for speed

// ---------------- tunables ----------------
float gainAperN   = 0.10f;
float iMax        = 1.5f;
float erpmCeiling = 2000.0f;          // 4.1 km/h on this wheel
float deadbandN   = 4.0f;

// ---------------- state ----------------
HX711 scale;
Preferences prefs;

float calSlope = 0.0f, calIntercept = 0.0f;
bool  calValid = false;
long  zeroOffset = 0;
float smoothed = 0.0f;
const float ALPHA = 0.30f;

bool  armed      = false;
bool  streaming  = true;
float forceN     = 0.0f;
float cmdAmps    = 0.0f;
int32_t lastErpm = 0;
bool  ceilingHit = false;

unsigned long lastSend = 0, lastPoll = 0, lastPrint = 0;
uint8_t rxBuf[512];

// ---------------------------------------------------------------- protocol

uint16_t crc16(const uint8_t *b, unsigned n) {
  uint16_t c = 0;
  for (unsigned i = 0; i < n; i++) {
    c ^= (uint16_t)b[i] << 8;
    for (int k = 0; k < 8; k++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

void sendPayload(const uint8_t *p, uint8_t n) {
  uint16_t c = crc16(p, n);
  VESC.write((uint8_t)0x02); VESC.write(n); VESC.write(p, n);
  VESC.write((uint8_t)(c >> 8)); VESC.write((uint8_t)(c & 0xFF));
  VESC.write((uint8_t)0x03);
}

// EVERY motion command goes through here. One clamp, one path to the motor.
void sendCurrent(float amps) {
  if (!armed)          amps = 0.0f;
  if (amps > iMax)     amps = iMax;
  if (amps > I_HARD_CAP) amps = I_HARD_CAP;
  if (amps < 0.0f)     amps = 0.0f;        // forward only in this sketch
  int32_t ma = (int32_t)(amps * 1000.0f);
  uint8_t p[5];
  p[0] = COMM_SET_CURRENT;
  p[1] = (ma >> 24) & 0xFF; p[2] = (ma >> 16) & 0xFF;
  p[3] = (ma >>  8) & 0xFF; p[4] =  ma        & 0xFF;
  sendPayload(p, 5);
}

int readPacket(unsigned long ms) {
  unsigned long t0 = millis();
  int st = 0, len = 0, got = 0, need = 1, lgot = 0;
  uint16_t crcRx = 0;
  while (millis() - t0 < ms) {
    if (!VESC.available()) { delay(1); continue; }
    uint8_t c = VESC.read();
    switch (st) {
      case 0: if (c == 0x02) { need = 1; lgot = 0; len = 0; st = 1; }
              else if (c == 0x03) { need = 2; lgot = 0; len = 0; st = 1; }
              break;
      case 1: len = (len << 8) | c;
              if (++lgot >= need) {
                if (len <= 0 || len > (int)sizeof(rxBuf)) { st = 0; break; }
                got = 0; st = 2;
              }
              break;
      case 2: rxBuf[got++] = c; if (got >= len) { crcRx = 0; got = 0; st = 3; } break;
      case 3: crcRx = (crcRx << 8) | c; if (++got >= 2) st = 4; break;
      case 4: if (c == 0x03 && crc16(rxBuf, len) == crcRx) return len; st = 0; break;
    }
  }
  return -1;
}

// Ask the ESC how fast the wheel is turning. Short timeout so the control
// loop never stalls waiting.
void pollSpeed() {
  while (VESC.available()) VESC.read();
  uint8_t id = COMM_GET_VALUES;
  sendPayload(&id, 1);
  int len = readPacket(40);
  if (len < 29) return;
  // skip to rpm: temps(4) currents(16) duty(2) = byte 23
  int i = 23;
  lastErpm = ((int32_t)rxBuf[i] << 24) | ((int32_t)rxBuf[i+1] << 16) |
             ((int32_t)rxBuf[i+2] << 8) | (int32_t)rxBuf[i+3];
}

// ---------------------------------------------------------------- force

void doTare() {
  Serial.println("# taring, hands off the bar...");
  double sum = 0; long n = 0;
  unsigned long t0 = millis();
  zeroOffset = 0;
  while (millis() - t0 < 2000) if (scale.is_ready()) { sum += scale.read(); n++; }
  if (n) zeroOffset = (long)(sum / n);
  smoothed = 0.0f;
  Serial.println("# tared.");
}

float countsToNewtons(float counts) {
  if (!calValid || calSlope == 0.0f) return 0.0f;
  return (counts - calIntercept) / calSlope;
}

// ---------------------------------------------------------------- commands

void allStop(const char *why) {
  armed = false; cmdAmps = 0.0f;
  uint8_t p[5] = { COMM_SET_CURRENT, 0, 0, 0, 0 };
  sendPayload(p, 5);
  Serial.print("# STOP -- "); Serial.println(why);
}

void help() {
  Serial.println("# a arm | x STOP | t tare | d stream | h help");
  Serial.println("# c <slope> <intercept>  set calibration directly");
  Serial.println("# k <A/N> gain | m <A> max | e <erpm> ceiling | b <N> deadband");
  Serial.print("# gain "); Serial.print(gainAperN, 3);
  Serial.print("  max "); Serial.print(iMax, 2);
  Serial.print(" A  ceiling "); Serial.print(erpmCeiling, 0);
  Serial.print(" erpm ("); Serial.print(erpmCeiling / 482.0f, 1);
  Serial.print(" km/h)  deadband "); Serial.print(deadbandN, 1);
  Serial.println(" N");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_CAP, OUTPUT);
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println();
  Serial.println("=== assist_bench ===");
  Serial.println("# force in, current out. one direction. disarmed at boot.");

  scale.begin(PIN_DT, PIN_SCK);
  unsigned long t0 = millis();
  while (!scale.is_ready()) {
    if (millis() - t0 > 3000) {
      Serial.println("# HX711 NOT RESPONDING -- check 3V3, GND, D18, D19");
      while (1) delay(1000);
    }
    delay(50);
  }

  prefs.begin("handle", true);
  calSlope     = prefs.getFloat("slope", 0.0f);
  calIntercept = prefs.getFloat("inter", 0.0f);
  calValid     = prefs.getBool("valid", false);
  prefs.end();

  if (!calValid) {
    Serial.println("# NO CALIBRATION IN FLASH. run handle_cal and save it first.");
    Serial.println("# refusing to arm without one -- force would be meaningless.");
    Serial.println("# enter yours directly:   c 10980.85 15543.0");
  } else {
    Serial.print("# calibration: "); Serial.print(calSlope, 1);
    Serial.println(" counts/N");
  }

  doTare();
  help();
  Serial.println("# 'a' to arm. push gently.");
}

void loop() {
  // ---- commands ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      char c = line.charAt(0);
      float v = line.substring(1).toFloat();
      if      (c == 'x') allStop("commanded");
      else if (c == 'a') {
        if (!calValid) Serial.println("# not calibrated. refusing.");
        else { armed = true; Serial.println("# ARMED. push the bar. 'x' stops."); }
      }
      else if (c == 't') doTare();
      else if (c == 'c') {
        String arg = line.substring(1); arg.trim();
        int sp = arg.indexOf(' ');
        float sl = (sp > 0) ? arg.substring(0, sp).toFloat() : 0.0f;
        float ic = (sp > 0) ? arg.substring(sp + 1).toFloat() : 0.0f;
        if (sl != 0.0f) {
          calSlope = sl; calIntercept = ic; calValid = true;
          prefs.begin("handle", false);
          prefs.putFloat("slope", calSlope);
          prefs.putFloat("inter", calIntercept);
          prefs.putBool("valid", true);
          prefs.end();
          Serial.print("# calibration set and saved: ");
          Serial.print(calSlope, 2); Serial.print(" counts/N, intercept ");
          Serial.println(calIntercept, 1);
        } else {
          Serial.println("# usage:  c <slope> <intercept>");
          Serial.println("# yours from 10-11:  c 10980.85 15543.0");
        }
      }
      else if (c == 'd') { streaming = !streaming;
                           Serial.println(streaming ? "# stream on" : "# stream off"); }
      else if (c == 'k' && v > 0) { gainAperN = v; help(); }
      else if (c == 'm' && v > 0) { iMax = (v > I_HARD_CAP) ? I_HARD_CAP : v; help(); }
      else if (c == 'e' && v > 0) { erpmCeiling = v; help(); }
      else if (c == 'b' && v >= 0) { deadbandN = v; help(); }
      else if (c == 'h') help();
      else Serial.println("# ? try h");
    }
  }

  // ---- force ----
  if (scale.is_ready()) {
    long raw = scale.read() - zeroOffset;
    smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;
    forceN = countsToNewtons(smoothed);
  }

  // ---- speed ----
  if (millis() - lastPoll >= POLL_MS) { lastPoll = millis(); pollSpeed(); }

  // ---- the law, such as it is ----
  float want = 0.0f;
  if (armed) {
    float f = forceN - deadbandN;
    if (f > 0.0f) want = f * gainAperN;
    if (want > iMax) want = iMax;
  }

  ceilingHit = (fabs((float)lastErpm) > erpmCeiling);
  if (ceilingHit) want = 0.0f;         // hard ceiling, no blending yet

  // rate limit, so the wheel is never shoved
  float maxStep = RAMP_A_PER_S * (SEND_MS / 1000.0f);
  if (want > cmdAmps + maxStep) cmdAmps += maxStep;
  else if (want < cmdAmps - maxStep) cmdAmps -= maxStep;
  else cmdAmps = want;

  // ---- send ----
  if (millis() - lastSend >= SEND_MS) {
    lastSend = millis();
    sendCurrent(cmdAmps);
  }

  // ---- lights ----
  int duty = (int)(255.0f * (cmdAmps / (iMax > 0 ? iMax : 1.0f)));
  if (duty > 255) duty = 255;
  if (duty < 0) duty = 0;
  analogWrite(PIN_LED_FWD, armed ? duty : 0);
  digitalWrite(PIN_LED_CAP, ceilingHit ? HIGH : LOW);

  // ---- stream ----
  if (streaming && millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(armed ? "ARM " : "--- ");
    Serial.print("cnt "); Serial.print((long)smoothed); Serial.print("   ");
    Serial.print("F "); Serial.print(forceN, 1); Serial.print(" N   ");
    Serial.print("I "); Serial.print(cmdAmps, 2); Serial.print(" A   ");
    Serial.print("erpm "); Serial.print(lastErpm);
    Serial.print("  "); Serial.print(lastErpm / 482.0f, 2); Serial.print(" km/h");
    if (ceilingHit) Serial.print("   [CEILING]");
    Serial.println();
  }

  delay(2);
}
