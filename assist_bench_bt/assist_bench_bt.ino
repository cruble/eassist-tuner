/*
  assist_bench_bt.ino  —  assist_bench, plus a phone that can tune it
  ESP32-WROOM-32  +  HX711 + bar load cell  <->  Flipsky Mini V6 MK5 (sensored)
                  +  Bluetooth Low Energy to a phone (field_tuner/index.html)

  WHAT CHANGED FROM assist_bench
  The control law is untouched. Same deadband, gain, rate limit, ceiling,
  same single sendCurrent() path to the motor. What is new:

    1. A BLE serial link (Nordic UART Service) alongside USB. Both feed the
       SAME command parser. USB behaves exactly as before.
    2. The phone has fewer rights than the USB cable:
         phone CAN     k m e b   tunables
                       x         STOP (it can always take power away)
                       t c       tare, calibration -- refused while armed
                       h p       help, report parameters
         phone CANNOT  a         arm. Arming stays at the chair.
    3. Hard bounds on every tunable, enforced in firmware for BOTH links, so
       no slider -- and no typo -- can set something unsafe:
         gain      0.01 .. GAIN_HARD_MAX   A/N
         max amps  0.10 .. I_HARD_CAP      A
         ceiling   200  .. ERPM_HARD_MAX   erpm
         deadband  DEADBAND_MIN .. 20      N
       (assist_bench had no upper bound on the speed ceiling. This does.)
    4. Machine-readable lines to the phone (see PHONE PROTOCOL below).

  A PHONE DISCONNECT CHANGES NOTHING. The chair keeps the last parameters
  and keeps running the law. The phone is a tuning knob, not a safety device.
  The abort is still the ESC power button / E-stop.

  --------------------------------------------------------------------------
  BEFORE YOU UPLOAD -- ALL FIVE  (same as assist_bench)

    [ ] Wheel OFF the bench, secured, free to spin, nothing in its path
    [ ] ESC power button within reach. That is the abort.
    [ ] App to Use = UART
    [ ] Load cell calibrated and SAVED to flash (handle_cal, 's')
    [ ] You have read the runaway note in assist_bench

  Board: "ESP32 Dev Module". If the upload says the sketch is too big, set
  Tools > Partition Scheme > "Huge APP (3MB No OTA)". BLE is large.

  --------------------------------------------------------------------------
  WIRING   unchanged from assist_bench

    HX711   VCC->3V3   GND->GND   DT->D18   SCK->D19
    LEDs    D25 green (push)   D26 yellow (over ceiling)   [optional now]
    ESC     COMM pin 3 -> D16 (RX2)   pin 4 -> D17 (TX2)   pin 6 -> GND

  RADIO NOTE: with BLE on, ADC2 (GPIO 0,2,4,12-15,25-27) cannot be read.
  Nothing here reads an analogue pin, but the lever must go on 32-39.

  SERIAL  115200     BLE name "eAssist"

  COMMANDS (USB, and phone where allowed)
    a           arm                         USB only
    x           STOP -- zero current, disarm
    k <A/N>     gain, amps per newton      (default 0.10)
    m <A>       max current                (default 1.5, hard cap 3.0)
    e <erpm>    speed ceiling              (default 2000 = 4.1 km/h)
    b <N>       deadband in newtons        (default 4.0)
    c <s> <i>   calibration slope/intercept, saved to flash
    t           tare the load cell
    d           toggle the USB live stream  USB only
    p           report parameters
    h           help

  PHONE PROTOCOL  (text lines, '\n' terminated, sent in 20-byte notifications)
    L <iCap> <kMax> <eMax> <bMin>                    hard limits, on connect
    P <k> <m> <e> <b> <calValid 0/1> <slope> <inter> parameters, on connect
                                                     and after every change
    T <armed 0/1> <F N> <I A> <erpm> <ceiling 0/1>   telemetry, 5 Hz
    # <text>                                         human message
  --------------------------------------------------------------------------
*/

#include "HX711.h"
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
// Nothing -- USB or phone -- may set a value outside these.
const float I_HARD_CAP    = 3.0f;     // A. nothing may exceed this, ever
const float GAIN_HARD_MAX = 0.50f;    // A/N. 6 N over deadband already hits 3 A
const float ERPM_HARD_MAX = 2900.0f;  // ~6.0 km/h, top of the planned taper
const float DEADBAND_MIN  = 2.0f;     // N. a resting hand must not drive
const float RAMP_A_PER_S  = 4.0f;     // how fast current may change
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

unsigned long lastSend = 0, lastPoll = 0, lastPrint = 0, lastTele = 0;
uint8_t rxBuf[512];

// ---------------------------------------------------------------- BLE link
// Nordic UART Service: the de-facto "serial over BLE". The phone writes
// command lines to RX and listens for notifications on TX.

#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone -> chair
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // chair -> phone

enum Source { SRC_USB, SRC_PHONE };

BLEServer         *bleServer = nullptr;
BLECharacteristic *bleTx     = nullptr;
volatile bool bleConnected   = false;
volatile bool bleJustJoined  = false;
volatile bool bleJustLeft    = false;

// Commands arrive on the BLE task. They are queued here and run in loop(),
// so the parser and the motor path only ever run on one thread.
const int CMD_LEN = 64;
struct CmdLine { char s[CMD_LEN]; };
QueueHandle_t bleRxQueue;

// Outgoing bytes wait here and leave in 20-byte notifications, a few ms
// apart, so a burst of text never floods the radio or stalls the loop.
const int TXQ_SIZE = 1024;
char txq[TXQ_SIZE];
int  txHead = 0, txTail = 0;
unsigned long lastNotify = 0;
const unsigned long NOTIFY_GAP_MS = 8;
const int CHUNK = 20;

void bleQueue(const char *s, size_t n) {
  if (!bleConnected) return;
  for (size_t i = 0; i < n; i++) {
    int next = (txHead + 1) % TXQ_SIZE;
    if (next == txTail) return;               // full: drop, never block
    txq[txHead] = s[i]; txHead = next;
  }
}
void bleQueue(const String &s) { bleQueue(s.c_str(), s.length()); }

void blePump() {
  if (!bleConnected || txHead == txTail) return;
  if (millis() - lastNotify < NOTIFY_GAP_MS) return;
  uint8_t chunk[CHUNK]; int n = 0;
  while (n < CHUNK && txTail != txHead) { chunk[n++] = txq[txTail]; txTail = (txTail + 1) % TXQ_SIZE; }
  bleTx->setValue(chunk, n);
  bleTx->notify();
  lastNotify = millis();
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s)    override { bleConnected = true;  bleJustJoined = true; }
  void onDisconnect(BLEServer *s) override { bleConnected = false; bleJustLeft = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = String(c->getValue().c_str());   // works on core 2.x and 3.x
    int start = 0;
    while (start < (int)v.length()) {
      int nl = v.indexOf('\n', start);
      if (nl < 0) nl = v.length();
      String piece = v.substring(start, nl); piece.trim();
      if (piece.length()) {
        CmdLine L; strncpy(L.s, piece.c_str(), CMD_LEN - 1); L.s[CMD_LEN - 1] = 0;
        xQueueSend(bleRxQueue, &L, 0);         // full queue: drop, never block
      }
      start = nl + 1;
    }
  }
};

void bleBegin() {
  bleRxQueue = xQueueCreate(8, sizeof(CmdLine));
  BLEDevice::init("eAssist");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());
  BLEService *svc = bleServer->createService(NUS_SERVICE);
  bleTx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ---------------------------------------------------------------- output
// say() goes to USB and, if a phone is connected, to the phone as a '#' line.

void say(const String &msg) {
  Serial.print("# "); Serial.println(msg);
  bleQueue("# " + msg + "\n");
}

void sendLimits() {
  char b[64];
  snprintf(b, sizeof b, "L %.2f %.2f %.0f %.1f\n", I_HARD_CAP, GAIN_HARD_MAX, ERPM_HARD_MAX, DEADBAND_MIN);
  bleQueue(b, strlen(b));
}

void sendParams() {
  char b[96];
  snprintf(b, sizeof b, "P %.3f %.2f %.0f %.1f %d %.2f %.1f\n",
           gainAperN, iMax, erpmCeiling, deadbandN, calValid ? 1 : 0, calSlope, calIntercept);
  bleQueue(b, strlen(b));
}

void sendTelemetry() {
  char b[64];
  snprintf(b, sizeof b, "T %d %.1f %.2f %ld %d\n",
           armed ? 1 : 0, forceN, cmdAmps, (long)lastErpm, ceilingHit ? 1 : 0);
  bleQueue(b, strlen(b));
}

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
  say("taring, hands off the bar...");
  double sum = 0; long n = 0;
  unsigned long t0 = millis();
  zeroOffset = 0;
  while (millis() - t0 < 2000) if (scale.is_ready()) { sum += scale.read(); n++; }
  if (n) zeroOffset = (long)(sum / n);
  smoothed = 0.0f;
  say("tared.");
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
  say(String("STOP -- ") + why);
}

void help() {
  Serial.println("# a arm | x STOP | t tare | d stream | p params | h help");
  Serial.println("# c <slope> <intercept>  set calibration directly");
  Serial.println("# k <A/N> gain | m <A> max | e <erpm> ceiling | b <N> deadband");
  Serial.print("# gain "); Serial.print(gainAperN, 3);
  Serial.print("  max "); Serial.print(iMax, 2);
  Serial.print(" A  ceiling "); Serial.print(erpmCeiling, 0);
  Serial.print(" erpm ("); Serial.print(erpmCeiling / 482.0f, 1);
  Serial.print(" km/h)  deadband "); Serial.print(deadbandN, 1);
  Serial.println(" N");
  Serial.print("# phone: "); Serial.println(bleConnected ? "connected" : "not connected");
  sendParams();
}

// Clamp into [lo, hi] and say so if the request was outside it.
float bounded(const char *name, float v, float lo, float hi) {
  if (v < lo) { say(String(name) + " below floor, set to " + String(lo, 2)); return lo; }
  if (v > hi) { say(String(name) + " above cap, set to "   + String(hi, 2)); return hi; }
  return v;
}

// One parser for both links. 'src' decides what is allowed.
void handleLine(String line, Source src) {
  line.trim();
  if (!line.length()) return;
  bool phone = (src == SRC_PHONE);
  char c = line.charAt(0);
  String arg = line.substring(1); arg.trim();
  float v = arg.toFloat();
  bool hasNum = arg.length() > 0 && (isdigit(arg.charAt(0)) || arg.charAt(0) == '.' || arg.charAt(0) == '-');

  if (c == 'x') { allStop(phone ? "from phone" : "commanded"); sendParams(); return; }

  if (c == 'a') {
    if (phone)          say("arming is physical -- not from the phone.");
    else if (!calValid) say("not calibrated. refusing.");
    else { armed = true; say("ARMED. push the bar. 'x' stops."); }
    return;
  }

  if (c == 't') {
    if (phone && armed) { say("disarm before taring -- a hand on the bar would become zero."); return; }
    doTare(); return;
  }

  if (c == 'c') {
    if (phone && armed) { say("disarm before changing calibration."); return; }
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
      say("calibration set and saved: " + String(calSlope, 2) + " counts/N, intercept " + String(calIntercept, 1));
      sendParams();
    } else {
      say("usage:  c <slope> <intercept>   e.g. c 10980.85 0");
    }
    return;
  }

  if (c == 'd') {
    if (phone) { say("stream toggle is USB only."); return; }
    streaming = !streaming;
    Serial.println(streaming ? "# stream on" : "# stream off");
    return;
  }

  if (c == 'k' || c == 'm' || c == 'e' || c == 'b') {
    if (!hasNum) { say("? needs a number, e.g. k 0.10"); return; }
    if      (c == 'k') gainAperN   = bounded("gain",     v, 0.01f, GAIN_HARD_MAX);
    else if (c == 'm') iMax        = bounded("max amps", v, 0.10f, I_HARD_CAP);
    else if (c == 'e') erpmCeiling = bounded("ceiling",  v, 200.0f, ERPM_HARD_MAX);
    else if (c == 'b') deadbandN   = bounded("deadband", v, DEADBAND_MIN, 20.0f);
    if (phone) {
      say(String("set ") + c + " " + arg);
      sendParams();
    } else help();
    return;
  }

  if (c == 'p') { sendParams(); if (!phone) help(); return; }
  if (c == 'h') { help(); if (phone) say("phone: k m e b x t c p. arming is at the chair."); return; }

  say("? try h");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_CAP, OUTPUT);
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println();
  Serial.println("=== assist_bench_bt ===");
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
    Serial.println("# enter yours directly:   c 10980.85 0");
  } else {
    Serial.print("# calibration: "); Serial.print(calSlope, 1);
    Serial.println(" counts/N");
  }

  doTare();
  bleBegin();                     // after tare, so the radio is not part of zero
  Serial.println("# BLE advertising as \"eAssist\"");
  help();
  Serial.println("# 'a' to arm (USB). push gently.");
}

void loop() {
  // ---- commands: USB ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line, SRC_USB);
  }

  // ---- commands: phone ----
  CmdLine L;
  while (xQueueReceive(bleRxQueue, &L, 0) == pdTRUE) handleLine(String(L.s), SRC_PHONE);

  // ---- phone comes and goes ----
  if (bleJustJoined) {
    bleJustJoined = false;
    txHead = txTail = 0;
    lastNotify = millis() + 300;  // give the phone a moment to subscribe
    Serial.println("# phone connected");
    bleQueue("# connected to eAssist\n");
    sendLimits(); sendParams();
  }
  if (bleJustLeft) {
    bleJustLeft = false;
    Serial.println("# phone disconnected -- parameters kept, law unchanged");
    delay(50);
    BLEDevice::startAdvertising();
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

  // ---- stream: USB (human) ----
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

  // ---- stream: phone (machine) ----
  if (bleConnected && millis() - lastTele >= 200) { lastTele = millis(); sendTelemetry(); }
  blePump();

  delay(2);
}
