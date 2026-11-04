/*
  assist_chair.ino  —  standalone: switch on, push, tune from the phone if needed
  ESP32-WROOM-32  +  HX711 + bar load cell  <->  Flipsky Mini V6 MK5 (sensored)
                  +  Bluetooth Low Energy to a phone (field_tuner.html)

  Same control law as assist_bench / assist_bench_bt. What is new: the chair
  arms itself. No laptop anywhere in the sequence.

  --------------------------------------------------------------------------
  HOW THE CHAIR TURNS ON  (the ESC power switch is the arm switch)

    1. Hands OFF the bar. Flip the ESC power switch.
       The ESC powers the ESP32 too, so this one switch starts everything.
    2. ESP32 waits until the ESC answers on UART.          LED: slow blink
    3. Two-second tare, hands off.                          LED: slow blink
         - The bar must be still, and the new zero must be close to the
           last good one. A hand on the bar fails one or both.
         - If it fails: LED double-blinks. Let go; it retries every 2 s.
    4. ARMED. Push.                                         LED: steady on

    STOP from the phone (or 'x' on USB) disarms and LATCHES.  LED: slow blink
    The only ways back are the switch (power-cycle the ESC) or 'a' on USB.
    The phone can never arm.

    If the ESC stops answering for 1 s, the chair disarms and goes back to
    step 2. (Only matters on the bench, with the ESP32 on USB power.)

    LED fast blink = FAULT: no calibration in flash, or HX711 not answering.
    It will not arm. Fix and power-cycle.

  The abort is still the ESC switch / XT90. Turning it off kills both the
  motor and the ESP32 -- that is the point of powering one from the other.
  --------------------------------------------------------------------------
  POWER  (changed from the harness sheet)
    ESC COMM 5 V  -> Schottky diode (1N5817, stripe toward ESP32) -> ESP32 VIN/5V
    ESC COMM GND  -> ESP32 GND   (already there: pin 6, white)
    Meter COMM 7 and 8 before connecting -- one is 3.3 V, one is 5 V. Use 5 V.
    The diode stops the ESC 5 V from backfeeding a laptop when USB is in, so
    USB and ESC power can be connected at the same time.

  WIRING
    HX711   VCC->3V3   GND->GND   DT->D18   SCK->D19
    LED     D25 -> 330 ohm -> LED -> GND        (status, see above)
    ESC     COMM pin 3 -> D16 (RX2)   pin 4 -> D17 (TX2)   pin 6 -> GND
    D26 is no longer used.

  RADIO NOTE: with BLE on, ADC2 (GPIO 0,2,4,12-15,25-27) cannot be READ as
  analogue. D25 as a digital LED output is fine. The lever must go on 32-39.

  SERIAL 115200     BLE name "eAssist"

  COMMANDS (USB, and phone where allowed)
    a           arm                         USB only (bench override)
    x           STOP -- zero current, disarm, latch
    k <A/N>     gain, amps per newton      (default 0.10)
    m <A>       max current                (default 1.5, hard cap 3.0)
    e <erpm>    speed ceiling              (default 2000 = 4.1 km/h)
    b <N>       deadband in newtons        (default 4.0)
    c <s> <i>   calibration slope/intercept, saved to flash   (disarmed only)
    t           tare now and accept it as the new zero        (disarmed only)
                -- the way out when the mount changed and auto-tare keeps refusing
    d           toggle the USB live stream  USB only
    p           report parameters
    h           help

  PHONE PROTOCOL  (text lines, '\n' terminated, sent in 20-byte notifications)
    L <iCap> <kMax> <eMax> <bMin>                        hard limits
    P <k> <m> <e> <b> <calValid 0/1> <slope> <inter>     parameters
    T <armed> <F N> <I A> <erpm> <ceiling> <state>       telemetry, 5 Hz
        state: F fault  W waiting for ESC  Z taring  H hand on bar
               A armed  S stopped (cycle power)
    # <text>                                             human message
  --------------------------------------------------------------------------
  Board: "ESP32 Dev Module", default partition scheme.
*/

#include "HX711.h"
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- pins ----------------
const int PIN_DT  = 18;
const int PIN_SCK = 19;
const int PIN_LED = 25;
const int PIN_RX  = 16;
const int PIN_TX  = 17;

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

// ---------------- auto-arm ----------------
const int   ESC_OK_REPLIES   = 3;     // consecutive answers before we trust the ESC
const unsigned long ESC_LOST_MS = 1000;
const float TARE_STILL_N     = 1.0f;  // max spread during the 2 s tare
const float TARE_MATCH_N     = 5.0f;  // max distance from the last good zero
const unsigned long TARE_RETRY_MS = 2000;

// ---------------- tunables ----------------
float gainAperN   = 0.10f;
float iMax        = 1.5f;
float erpmCeiling = 2000.0f;          // 4.1 km/h on this wheel
float deadbandN   = 4.0f;

// ---------------- state ----------------
enum ChairState { ST_FAULT, ST_WAIT_ESC, ST_TARING, ST_HAND_ON_BAR, ST_ARMED, ST_STOPPED };
ChairState state = ST_WAIT_ESC;
const char STATE_CODE[] = { 'F', 'W', 'Z', 'H', 'A', 'S' };

HX711 scale;
Preferences prefs;

float calSlope = 0.0f, calIntercept = 0.0f;
bool  calValid = false;
bool  hxOk = false;
long  zeroOffset = 0;
long  savedZero = 0; bool savedZeroValid = false;
float smoothed = 0.0f;
const float ALPHA = 0.30f;

bool  armed      = false;
bool  streaming  = true;
float forceN     = 0.0f;
float cmdAmps    = 0.0f;
int32_t lastErpm = 0;
bool  ceilingHit = false;
int   escReplies = 0;
unsigned long lastEscReply = 0, lastTareTry = 0;

unsigned long lastSend = 0, lastPoll = 0, lastPrint = 0, lastTele = 0;
uint8_t rxBuf[512];

// ---------------------------------------------------------------- BLE link
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone -> chair
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // chair -> phone

enum Source { SRC_USB, SRC_PHONE };

BLEServer         *bleServer = nullptr;
BLECharacteristic *bleTx     = nullptr;
volatile bool bleConnected   = false;
volatile bool bleJustJoined  = false;
volatile bool bleJustLeft    = false;

const int CMD_LEN = 64;
struct CmdLine { char s[CMD_LEN]; };
QueueHandle_t bleRxQueue;

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
  if ((long)(millis() - lastNotify) < (long)NOTIFY_GAP_MS) return;
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
        xQueueSend(bleRxQueue, &L, 0);
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
  snprintf(b, sizeof b, "T %d %.1f %.2f %ld %d %c\n",
           armed ? 1 : 0, forceN, cmdAmps, (long)lastErpm, ceilingHit ? 1 : 0, STATE_CODE[state]);
  bleQueue(b, strlen(b));
}

// ---------------------------------------------------------------- state

void setState(ChairState s, const char *why) {
  if (s == state) return;
  state = s;
  armed = (s == ST_ARMED);
  if (!armed) cmdAmps = 0.0f;
  say(why);
}

// One LED. steady = armed. slow blink = waiting / stopped.
// double blink = hand on bar. fast blink = fault.
void updateLed() {
  unsigned long t = millis();
  bool on = false;
  switch (state) {
    case ST_ARMED:       on = true; break;
    case ST_FAULT:       on = (t / 100) % 2; break;                       // 5 Hz
    case ST_HAND_ON_BAR: { unsigned long p = t % 1200;                     // blink-blink, pause
                           on = (p < 120) || (p > 240 && p < 360); } break;
    default:             on = (t / 500) % 2; break;                       // 1 Hz
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
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

// Ask the ESC how fast the wheel is turning. Returns true if it answered --
// which is also how we know the ESC is powered.
bool pollSpeed() {
  while (VESC.available()) VESC.read();
  uint8_t id = COMM_GET_VALUES;
  sendPayload(&id, 1);
  int len = readPacket(40);
  if (len < 29 || rxBuf[0] != COMM_GET_VALUES) return false;
  int i = 23;   // skip to rpm: id(1) temps(4) currents(16) duty(2)
  lastErpm = ((int32_t)rxBuf[i] << 24) | ((int32_t)rxBuf[i+1] << 16) |
             ((int32_t)rxBuf[i+2] << 8) | (int32_t)rxBuf[i+3];
  return true;
}

// ---------------------------------------------------------------- force

// Two seconds of samples. Returns the mean; spread is max-min in counts.
bool sampleZero(long &mean, long &spread) {
  double sum = 0; long n = 0, lo = LONG_MAX, hi = LONG_MIN;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (scale.is_ready()) {
      long r = scale.read(); sum += r; n++;
      if (r < lo) lo = r;
      if (r > hi) hi = r;
    }
    // keep listening to the ESC, or two silent seconds look like it went away
    if (millis() - lastPoll >= POLL_MS) {
      lastPoll = millis();
      if (pollSpeed()) { lastEscReply = millis(); if (escReplies < 100) escReplies++; }
    }
    if (bleConnected) blePump();
    updateLed();
    delay(1);
  }
  if (n < 5) return false;
  mean = (long)(sum / n); spread = hi - lo;
  return true;
}

void acceptZero(long z) {
  zeroOffset = z; smoothed = 0.0f; forceN = 0.0f;
  savedZero = z; savedZeroValid = true;
  prefs.begin("handle", false); prefs.putLong("zero", z); prefs.putBool("zvalid", true); prefs.end();
}

// Auto-tare: only accepted if the bar was still AND the zero is near the
// last good one. Returns true if accepted.
bool autoTare() {
  long mean, spread;
  if (!sampleZero(mean, spread)) { say("tare: HX711 gave no samples"); return false; }
  float spreadN = spread / calSlope;
  float driftN  = savedZeroValid ? fabs((float)(mean - savedZero)) / calSlope : 0.0f;
  if (spreadN > TARE_STILL_N) {
    say("tare refused: bar moving (" + String(spreadN, 1) + " N). hands off.");
    return false;
  }
  if (savedZeroValid && driftN > TARE_MATCH_N) {
    say("tare refused: " + String(driftN, 1) + " N from last zero. hand on bar? "
        "if the mount changed, send t to accept.");
    return false;
  }
  acceptZero(mean);
  say("tared (" + String(driftN, 1) + " N from last zero).");
  return true;
}

// Manual tare: the person says the bar is free. Accept whatever we see.
void manualTare() {
  say("taring, hands off the bar...");
  long mean, spread;
  if (!sampleZero(mean, spread)) { say("tare: HX711 gave no samples"); return; }
  acceptZero(mean);
  say("tared and saved as the new zero.");
}

float countsToNewtons(float counts) {
  if (!calValid || calSlope == 0.0f) return 0.0f;
  return (counts - calIntercept) / calSlope;
}

// ---------------------------------------------------------------- commands

void allStop(const char *why) {
  uint8_t p[5] = { COMM_SET_CURRENT, 0, 0, 0, 0 };
  sendPayload(p, 5);
  if (state == ST_FAULT) { say(String("STOP -- ") + why); return; }
  setState(ST_STOPPED, (String("STOP -- ") + why + ". cycle ESC power to re-arm.").c_str());
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
  Serial.print("# state "); Serial.print(STATE_CODE[state]);
  Serial.print("   phone "); Serial.println(bleConnected ? "connected" : "not connected");
  sendParams();
}

float bounded(const char *name, float v, float lo, float hi) {
  if (v < lo) { say(String(name) + " below floor, set to " + String(lo, 2)); return lo; }
  if (v > hi) { say(String(name) + " above cap, set to "   + String(hi, 2)); return hi; }
  return v;
}

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
    if (phone)                   say("arming is the ESC switch -- not the phone.");
    else if (state == ST_FAULT)  say("fault -- refusing to arm.");
    else if (escReplies < ESC_OK_REPLIES) say("ESC not answering -- refusing to arm.");
    else setState(ST_ARMED, "ARMED from USB. push the bar. 'x' stops.");
    return;
  }

  if (c == 't') {
    if (armed) { say("disarm before taring -- a hand on the bar would become zero."); return; }
    if (state == ST_FAULT) { say("fault -- no tare."); return; }
    manualTare();
    // A tare never arms by itself. It only resets the zero; the normal
    // hands-off check then runs again and arms if the bar is still free.
    if (state == ST_HAND_ON_BAR) { setState(ST_TARING, "new zero accepted. re-checking hands off..."); lastTareTry = millis(); }
    return;
  }

  if (c == 'c') {
    if (armed) { say("disarm before changing calibration."); return; }
    int sp = arg.indexOf(' ');
    float sl = (sp > 0) ? arg.substring(0, sp).toFloat() : 0.0f;
    float ic = (sp > 0) ? arg.substring(sp + 1).toFloat() : 0.0f;
    if (sl != 0.0f) {
      calSlope = sl; calIntercept = ic; calValid = true;
      prefs.begin("handle", false);
      prefs.putFloat("slope", calSlope);
      prefs.putFloat("inter", calIntercept);
      prefs.putBool("valid", true);
      prefs.putBool("zvalid", false);      // new mount, new zero: forget the old one
      prefs.end();
      savedZeroValid = false;
      say("calibration saved: " + String(calSlope, 2) + " counts/N, intercept " + String(calIntercept, 1)
          + ". old zero forgotten -- tare next.");
      if (state == ST_FAULT && hxOk) setState(ST_WAIT_ESC, "calibration present -- waiting for ESC.");
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
    if (phone) { say(String("set ") + c + " " + arg); sendParams(); }
    else help();
    return;
  }

  if (c == 'p') { sendParams(); if (!phone) help(); return; }
  if (c == 'h') { help(); if (phone) say("phone: k m e b x t c p. arming is the ESC switch."); return; }

  say("? try h");
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_LED, OUTPUT);
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println();
  Serial.println("=== assist_chair ===");
  Serial.println("# standalone. the ESC switch arms it. phone tunes, never arms.");

  bleBegin();
  Serial.println("# BLE advertising as \"eAssist\"");

  scale.begin(PIN_DT, PIN_SCK);
  unsigned long t0 = millis();
  while (!scale.is_ready() && millis() - t0 < 3000) delay(50);
  hxOk = scale.is_ready();

  prefs.begin("handle", true);
  calSlope       = prefs.getFloat("slope", 0.0f);
  calIntercept   = prefs.getFloat("inter", 0.0f);
  calValid       = prefs.getBool("valid", false);
  savedZero      = prefs.getLong("zero", 0);
  savedZeroValid = prefs.getBool("zvalid", false);
  prefs.end();

  state = ST_WAIT_ESC;
  if (!hxOk)          setState(ST_FAULT, "FAULT: HX711 not responding -- check 3V3, GND, D18, D19.");
  else if (!calValid) setState(ST_FAULT, "FAULT: no calibration in flash. send c <slope> <intercept>.");
  else { say("calibration " + String(calSlope, 1) + " counts/N. waiting for ESC..."); }
  help();
}

// ---------------------------------------------------------------- loop

void loop() {
  // ---- commands ----
  if (Serial.available()) handleLine(Serial.readStringUntil('\n'), SRC_USB);
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

  // ---- speed, and is the ESC there at all ----
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();
    if (pollSpeed()) { lastEscReply = millis(); if (escReplies < 100) escReplies++; }
  }
  bool escLost = (escReplies > 0) && (millis() - lastEscReply > ESC_LOST_MS);
  if (escLost) {
    escReplies = 0; lastErpm = 0;
    if (state == ST_ARMED || state == ST_TARING || state == ST_HAND_ON_BAR)
      setState(ST_WAIT_ESC, "ESC stopped answering -- disarmed. waiting for ESC.");
    else if (state == ST_STOPPED)
      setState(ST_WAIT_ESC, "ESC off. latch cleared -- will re-arm when it returns.");
  }

  // ---- the arming sequence ----
  if (state == ST_WAIT_ESC && escReplies >= ESC_OK_REPLIES) {
    setState(ST_TARING, "ESC answering. taring -- hands off the bar.");
    lastTareTry = 0;
  }
  if ((state == ST_TARING || state == ST_HAND_ON_BAR) && millis() - lastTareTry >= TARE_RETRY_MS) {
    if (autoTare()) setState(ST_ARMED, "ARMED. push the bar.");
    else            setState(ST_HAND_ON_BAR, "not armed -- let go of the bar. retrying.");
    lastTareTry = millis();
  }

  // ---- force ----
  if (hxOk && scale.is_ready()) {
    long raw = scale.read() - zeroOffset;
    smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;
    forceN = countsToNewtons(smoothed);
  }

  // ---- the law, such as it is ----
  float want = 0.0f;
  if (armed) {
    float f = forceN - deadbandN;
    if (f > 0.0f) want = f * gainAperN;
    if (want > iMax) want = iMax;
  }

  ceilingHit = (fabs((float)lastErpm) > erpmCeiling);
  if (ceilingHit) want = 0.0f;         // hard ceiling, no blending yet

  float maxStep = RAMP_A_PER_S * (SEND_MS / 1000.0f);
  if (want > cmdAmps + maxStep) cmdAmps += maxStep;
  else if (want < cmdAmps - maxStep) cmdAmps -= maxStep;
  else cmdAmps = want;

  // ---- send ----
  if (millis() - lastSend >= SEND_MS) {
    lastSend = millis();
    sendCurrent(cmdAmps);
  }

  updateLed();

  // ---- stream: USB (human) ----
  if (streaming && millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(STATE_CODE[state]); Serial.print(armed ? " ARM " : " --- ");
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
