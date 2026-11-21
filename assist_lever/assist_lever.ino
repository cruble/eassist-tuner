/*
  assist_lever.ino  —  assist_chair (11/14) with the lever's range kept in flash
  standalone: switch on, squeeze, tune from the phone if needed

  LEVER RANGE (11/20): nothing about the lever is learned at switch-on any
  more. Its range is recorded once, on the chair, and kept in flash:
      z   lever RELEASED  -> this reading is rest
      l   lever SQUEEZED  -> this reading is full
  Both only while disarmed (ESC switch off), both saved the moment they are
  sent. 'z' alone re-zeros and keeps the span. Until both exist the lever is
  ignored. The chair will not arm while the lever is squeezed -- LED double
  blink, same as a hand on the bar -- and arms as soon as it is let go.
  Response is linear: 8% dead zone, then 0..100% of squeeze = 0..max current.
  NO PUSH BAR IN THIS BUILD (HAS_CELL = false): the HX711 is never set up,
  never read and never checked, whatever was saved before, and 'u 1' is
  refused. The load cell code is still in the file, switched off; set
  HAS_CELL true to get assist_chair's behaviour back.
  The P line carries two extra fields at the end for the phone page
  (field_tuner_lever): lever rest, and 1/0 for "range recorded".
  ESP32-WROOM-32  +  HX711 + bar load cell  <->  Flipsky Mini V6 MK5 (sensored)
                  +  Bluetooth Low Energy to a phone (field_tuner/index.html)

  Same push law as assist_bench / assist_bench_bt. The chair arms itself --
  no laptop anywhere in the sequence -- and (11/05) it can REVERSE:

                       push                pull
    rolling forward    assist              nothing (brake: later)
    at rest            assist              reverse, capped
    rolling backward   nothing (later)     reverse, capped

  LEVER (11/07): a potentiometer on GPIO 34, driven by a brake-lever cable.
  Squeeze -> help rises to max current. Let go -> help stops. Forward only.
  Rest and span come from flash (see LEVER RANGE above). Load cell and lever
  both run; the larger request wins. Pulling MUST raise the reading -- a
  broken wire then reads 0 = below rest = no help. If yours falls when
  pulled, swap the pot's outer wires ('l' refuses a falling reading).

  BOOST (11/14): a second hold button on the phone. Slow drive only creeps --
  it fades out at 0.6 km/h -- so it cannot show what assist feels like at a
  walk. Boost gives a constant current while held, all the way up to the
  normal speed ceiling, fading out over the last fifth of it. Same rules as
  slow drive: armed only, forward only, two signals to start, stops 0.8 s
  after the last signal, 60 s at most. Its current ('i') never exceeds max.

  PUSH BAR ON/OFF (11/13): 'u 0' turns the load cell off. The chair then
  needs no zero: it arms as soon as the ESC answers, ignores the bar
  entirely, and a missing amplifier or calibration is not a fault. Slow
  drive and the lever still work. 'u 1' brings the bar and its hands-off
  tare back. THIS BUILD STARTS WITH THE BAR OFF until a saved setting says
  otherwise -- on the clamped handle its readings were too noisy to arm.

  SLOW DRIVE (11/13): the one way the phone can move the chair. Hold the
  button on the phone -> the chair creeps forward; let go -> it stops.
    - Armed only. The ESC switch is still the arm.
    - Starting needs two 'f' in a row, so one stray late signal starts nothing.
    - While held the phone sends 'f' about 4 times a second. If none arrives
      for 0.8 s (thumb off, phone out of range, page closed) the chair stops.
    - Its own low speed limit ('y') and its own current ('n', never above
      max current). Current fades out as the speed limit approaches.
    - Hard stop after 30 s whatever happens. 'q', x, phone STOP, ESC off end it.

  'At rest' is |v| <= 0.15 m/s, the band from the control grid. Pull uses the
  same gain and deadband as push, mirrored. Reverse has its own current cap
  and speed ceiling, both lower. Reverse max = 0 turns reverse off.

  --------------------------------------------------------------------------
  HOW THE CHAIR TURNS ON  (the ESC power switch is the arm switch)

    0. Power bank on. The ESP32 boots and waits for the ESC.  LED: slow blink
    1. Hands OFF the bar. Flip the ESC power switch.
    2. ESP32 waits until the ESC answers on UART.          LED: slow blink
    3. Two-second tare, hands off.                          LED: slow blink
         - The bar must be still, and the new zero must be close to the
           last good one. A hand on the bar fails one or both.
         - If it fails: LED double-blinks. Let go; it retries every 2 s.
    4. ARMED. Push.                                         LED: steady on

    STOP from the phone (or 'x' on USB) disarms and LATCHES.  LED: slow blink
    The only ways back are the switch (power-cycle the ESC) or 'a' on USB.
    The phone can never arm.

    ESC switch OFF = disarmed within 1 s (the ESP32 notices the ESC went
    quiet). ON again = back to step 2: tare, then arm. So with the ESP32 on
    its own power bank, the ESC switch is still the one arm/disarm switch.

    LED fast blink = FAULT: no calibration in flash, or HX711 not answering.
    It will not arm. Fix and power-cycle.

  The abort is still the ESC switch / XT90.
  --------------------------------------------------------------------------
  POWER  (as on the harness sheet -- unchanged)
    ESP32 on its own USB power bank. COMM 7 and 8 stay empty.
    Optional later: feed the ESP32 from the ESC's 5 V through a Schottky
    diode so one switch powers everything. Not needed for this firmware.

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
    m <A>       max current                (default 1.5, hard cap 10.0)
    e <erpm>    speed ceiling              (default 2000 = 4.1 km/h)
    b <N>       deadband in newtons        (default 4.0)
    r <A>       reverse max current        (default 0.8, hard cap 1.5, 0 = off)
    v <erpm>    reverse speed ceiling      (default 600 = 1.2 km/h, max 1000)
    g <counts>  lever span: reading above rest for a full squeeze (default 1200)
    f           slow drive: keep going (send repeatedly; stops 0.8 s after the last)
    q           slow drive: stop now (stays armed)
    n <A>       slow drive current         (default 1.0, never above max current)
    y <erpm>    slow drive speed limit     (default 300 = 0.6 km/h, max 1000)
    u <0|1>     push bar (load cell) off / on   (default 0 = off, no zeroing)
    o           boost: keep going (send repeatedly, like f)
    i <A>       boost current              (default 3.0, never above max current)
    c <s> <i>   calibration slope/intercept, saved to flash   (disarmed only)
    t           tare now and accept it as the new zero        (disarmed only)
                -- the way out when the mount changed and auto-tare keeps refusing
    d           toggle the USB live stream  USB only
    j <A> <s>   DEMO: drive forward at <A> amps for <s> seconds (max 5 s).
                USB only, armed only. Capped by max current, speed ceiling
                and ramp like everything else. x / phone STOP / ESC off end it.
    w           save k m e b r v g n y u i to flash -- the values it starts with next time
    p           report parameters
    h           help

  PHONE PROTOCOL  (text lines, '\n' terminated, sent in 20-byte notifications)
    L <iCap> <kMax> <eMax> <bMin> <rCap> <vMax>          hard limits
    P <k> <m> <e> <b> <calValid 0/1> <slope> <inter> <unsaved 0/1> <r> <v> <g> <n> <y> <u> <i>   parameters
    T <armed> <F N> <I A> <erpm> <ceiling> <state> <lever %> <lever raw> <phone drive: 0 none, 1 slow drive, 2 boost>   telemetry, 5 Hz
                                         (lever % is -1 when no lever was found)
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
const int PIN_LEVER = 34;           // ADC1 -- still readable with Bluetooth on
const int PIN_RX  = 16;
const int PIN_TX  = 17;

#define VESC Serial2
const long VESC_BAUD = 115200;

const uint8_t COMM_GET_VALUES  = 4;
const uint8_t COMM_SET_CURRENT = 6;

// ---------------- limits ----------------
// Nothing -- USB or phone -- may set a value outside these.
// 11/14: raised from 3 A to 10 A -- 3 A would not start the chair with 100+ lb
// of ballast. A ceiling for stepping up from the phone, not a setting to run at.
// Still under the ESC's own motor limit (12 A in mk5-config-10-21-26-2b).
// Nothing here watches motor temperature, and heat goes with current squared:
// 10 A is ~11x the heat of 3 A. Above ~6 A: short holds, hand on the motor.
const float I_HARD_CAP    = 10.0f;    // A. nothing may exceed this, ever
const float GAIN_HARD_MAX = 0.50f;    // A/N. 6 N over deadband already hits 3 A
const float ERPM_HARD_MAX = 2900.0f;  // ~6.0 km/h, top of the planned taper
const float DEADBAND_MIN  = 2.0f;     // N. a resting hand must not drive
const float I_REV_HARD_CAP    = 1.5f;    // A. reverse is always gentler than forward
const float ERPM_REV_HARD_MAX = 1000.0f; // ~2.1 km/h backwards, at most
const float REST_ERPM         = 260.0f;  // 0.15 m/s = 0.54 km/h x 482 erpm per km/h
// If the speed reads NEGATIVE while the wheel turns forward, set this to -1.
// Everything about direction depends on it. Check it before trusting reverse.
const int   ERPM_SIGN         = 1;
const float LEVER_SPAN_MIN    = 200.0f;  // counts. less travel than this is not a usable range
const float LEVER_DEADZONE    = 0.08f;   // first 8% of the squeeze does nothing
const float LEVER_ALPHA       = 0.2f;    // smoothing
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
// Defaults, used until something is saved with 'w'. Saved values are loaded
// at boot (and re-clamped, in case the hard limits above have since changed).
const float DEF_GAIN = 0.10f, DEF_IMAX = 1.5f, DEF_ERPM = 2000.0f, DEF_DEADBAND = 4.0f;
const float DEF_IREV = 0.8f, DEF_EREV = 600.0f, DEF_SPAN = 1200.0f;
const float DEF_CREEP_A = 1.0f, DEF_CREEP_ERPM = 300.0f;
float gainAperN   = DEF_GAIN;
float iMax        = DEF_IMAX;
float erpmCeiling = DEF_ERPM;         // 4.1 km/h on this wheel
float deadbandN   = DEF_DEADBAND;
float iRevMax     = DEF_IREV;         // 0 = reverse off
float erpmRevCeil = DEF_EREV;         // 1.2 km/h backwards
float leverSpan   = DEF_SPAN;         // ADC counts above rest = full squeeze
float creepAmps   = DEF_CREEP_A;      // slow drive current
float creepErpm   = DEF_CREEP_ERPM;   // slow drive speed limit, 0.6 km/h
float savedK = DEF_GAIN, savedM = DEF_IMAX, savedE = DEF_ERPM, savedB = DEF_DEADBAND;
float savedR = DEF_IREV, savedV = DEF_EREV, savedG = DEF_SPAN;
float savedN = DEF_CREEP_A, savedY = DEF_CREEP_ERPM;
const bool HAS_CELL     = false;     // false = no load cell fitted: never set up, read or checked
const bool DEF_USE_CELL = false;     // push bar off by default in this build
bool  useCell = DEF_USE_CELL, savedU = DEF_USE_CELL;

// lever state
float leverRaw = 0.0f, leverRest = 0.0f, leverFrac = 0.0f;
bool  leverPresent = false;          // true = rest AND span are recorded in flash
bool  leverRestSet = false;          // true = 'z' has been recorded at some point


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
unsigned long jogUntil = 0;  float jogAmps = 0.0f;   // USB demo command
const float JOG_MAX_S = 5.0f;

// slow drive from the phone (hold to go)
const unsigned long CREEP_HEARTBEAT_MS = 800;    // no 'f' for this long -> stop
const unsigned long CREEP_MAX_MS       = 30000;  // one hold never runs longer than this
const float CREEP_ERPM_HARD_MAX        = 1000.0f; // ~2.1 km/h
bool creepOn = false;
int  driveMode = 0, primeMode = 0;               // 1 slow drive, 2 boost
const unsigned long BOOST_MAX_MS       = 60000;  // one boost hold never runs longer than this
const float DEF_BOOST_A = 3.0f;
float boostAmps = DEF_BOOST_A, savedI = DEF_BOOST_A;
unsigned long creepStart = 0, creepLast = 0, creepPrime = 0;
const unsigned long CREEP_PRIME_MS = 600;        // the second 'f' must follow the first within this
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
  snprintf(b, sizeof b, "L %.2f %.2f %.0f %.1f %.2f %.0f\n", I_HARD_CAP, GAIN_HARD_MAX, ERPM_HARD_MAX, DEADBAND_MIN,
           I_REV_HARD_CAP, ERPM_REV_HARD_MAX);
  bleQueue(b, strlen(b));
}

bool unsavedTune() {
  return fabs(gainAperN - savedK) > 1e-4f || fabs(iMax - savedM) > 1e-4f ||
         fabs(erpmCeiling - savedE) > 0.5f || fabs(deadbandN - savedB) > 1e-3f ||
         fabs(iRevMax - savedR) > 1e-4f || fabs(erpmRevCeil - savedV) > 0.5f ||
         fabs(leverSpan - savedG) > 0.5f ||
         fabs(creepAmps - savedN) > 1e-4f || fabs(creepErpm - savedY) > 0.5f ||
         useCell != savedU || fabs(boostAmps - savedI) > 1e-4f;
}

void sendParams() {
  char b[176];
  snprintf(b, sizeof b, "P %.3f %.2f %.0f %.1f %d %.2f %.1f %d %.2f %.0f %.0f %.2f %.0f %d %.2f %.0f %d\n",
           gainAperN, iMax, erpmCeiling, deadbandN, calValid ? 1 : 0, calSlope, calIntercept,
           unsavedTune() ? 1 : 0, iRevMax, erpmRevCeil, leverSpan, creepAmps, creepErpm, useCell ? 1 : 0, boostAmps,
           leverRest, leverPresent ? 1 : 0);
  bleQueue(b, strlen(b));
}

void sendTelemetry() {
  char b[80];
  snprintf(b, sizeof b, "T %d %.1f %.2f %ld %d %c %d %d %d\n",
           armed ? 1 : 0, forceN, cmdAmps, (long)lastErpm, ceilingHit ? 1 : 0, STATE_CODE[state],
           leverPresent ? (int)(leverFrac * 100.0f + 0.5f) : -1, (int)leverRaw, creepOn ? driveMode : 0);
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

// The load cell can only cause a fault while it is in use.
void recheckFaults() {
  if (useCell && !hxOk)          setState(ST_FAULT, "FAULT: HX711 not responding -- check 3V3, GND, D18, D19. (or 'u 0': push bar off)");
  else if (useCell && !calValid) setState(ST_FAULT, "FAULT: no calibration in flash. send c <slope> <intercept>. (or 'u 0')");
  else if (state == ST_FAULT)    setState(ST_WAIT_ESC, "fault cleared -- waiting for ESC.");
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
  if (!armed)                  amps = 0.0f;
  if (amps > iMax)             amps = iMax;
  if (amps > I_HARD_CAP)       amps = I_HARD_CAP;
  if (amps < -iRevMax)         amps = -iRevMax;        // 0 here = no reverse at all
  if (amps < -I_REV_HARD_CAP)  amps = -I_REV_HARD_CAP;
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
  lastErpm *= ERPM_SIGN;
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

// The lever's range lives in flash ('z' released, 'l' squeezed). Nothing is
// learned at switch-on, so a squeezed lever can never become its own zero.
float readLeverNow() {
  float sum = 0; for (int i = 0; i < 16; i++) { sum += analogRead(PIN_LEVER); delay(1); }
  return sum / 16.0f;
}

// Past the dead zone right now? Used to refuse arming with the lever held.
bool leverSqueezed() { return leverPresent && leverFrac > 0.0f; }

// Look for the amplifier. Only called while the push bar is in use.
void probeHx() {
  unsigned long t0 = millis();
  while (!scale.is_ready() && millis() - t0 < 3000) delay(50);
  hxOk = scale.is_ready();
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
  Serial.println("# LEVER RANGE (disarmed):  z = lever released   l = lever fully squeezed");
  if (leverPresent) { Serial.print("# lever range "); Serial.print(leverRest, 0); Serial.print(" -> "); Serial.println(leverRest + leverSpan, 0); }
  else              Serial.println("# lever NOT calibrated -- ignored until z and l are sent");
  Serial.println("# k <A/N> gain | m <A> max | e <erpm> ceiling | b <N> deadband | r <A> v <erpm> reverse | w save");
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
    else if (leverSqueezed())    say("lever is squeezed -- let go, then arm.");
    else setState(ST_ARMED, "ARMED from USB. push the bar. 'x' stops.");
    return;
  }

  if (c == 't') {
    if (armed) { say("disarm before taring -- a hand on the bar would become zero."); return; }
    if (!useCell) { say("push bar is off -- nothing to tare. 'u 1' turns it on."); return; }
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
      recheckFaults();
      sendParams();
    } else {
      say("usage:  c <slope> <intercept>   e.g. c 10980.85 0");
    }
    return;
  }

  // push bar (load cell) on / off
  if (c == 'u') {
    if (!HAS_CELL) { say("this build has no push bar -- lever and phone drive only."); sendParams(); return; }
    if (arg != "0" && arg != "1") { say("usage:  u 0 (push bar off)   u 1 (push bar on)"); return; }
    bool on = (arg == "1");
    if (on != useCell) {
      useCell = on;
      // The amplifier is only looked for when wanted. The look can take 3 s,
      // so the motor is told to stop first.
      if (on && !hxOk) { cmdAmps = 0.0f; sendCurrent(0.0f); probeHx(); }
      say(on ? "push bar ON -- it will tare before arming. hands off."
             : "push bar OFF -- no zeroing; the chair arms when the ESC answers.");
      // Re-run the arming sequence under the new rule. A STOP stays latched.
      if (state != ST_STOPPED) { if (state != ST_FAULT) setState(ST_WAIT_ESC, "re-arming..."); recheckFaults(); }
    }
    sendParams();
    return;
  }

  // phone drive. 'f' = slow drive, 'o' = boost. Each both starts its mode and
  // keeps it alive; silence stops it.
  if (c == 'f' || c == 'o') {
    int mode = (c == 'f') ? 1 : 2;
    const char *name = (mode == 1) ? "slow drive" : "boost";
    if (!armed) { if (!creepOn && millis() - creepLast > 2000) { creepLast = millis(); say(String(name) + ": not armed."); } return; }
    if (creepOn && mode != driveMode) return;      // the other button is held: ignore
    if (!creepOn) {
      // Starting takes two signals in a row, of the same kind. One stray, late
      // one (a stalled link delivering it after the thumb is off) starts nothing.
      if (creepPrime && primeMode == mode && millis() - creepPrime < CREEP_PRIME_MS) {
        creepOn = true; driveMode = mode; creepStart = millis(); creepPrime = 0; say(String(name) + " ON.");
      } else { creepPrime = millis(); primeMode = mode; return; }
    }
    creepLast = millis();
    return;
  }
  if (c == 'q') { creepPrime = 0; if (creepOn) { creepOn = false; say("phone drive off (released)."); } return; }

  if (c == 'j') {
    if (phone)  { say("demo drive is USB only."); return; }
    if (!armed) { say("demo drive: arm first (ESC switch on, hands off)."); return; }
    int sp = arg.indexOf(' ');
    float amps = (sp > 0) ? arg.substring(0, sp).toFloat() : arg.toFloat();
    float secs = (sp > 0) ? arg.substring(sp + 1).toFloat() : 2.0f;
    if (amps <= 0.0f || secs <= 0.0f) { say("usage:  j <amps> <seconds>   e.g. j 1.0 3"); return; }
    if (amps > iMax) amps = iMax;
    if (secs > JOG_MAX_S) secs = JOG_MAX_S;
    jogAmps = amps; jogUntil = millis() + (unsigned long)(secs * 1000.0f);
    say("DEMO: " + String(amps, 2) + " A for " + String(secs, 1) + " s. x stops.");
    return;
  }

  // lever range: z = released, l = fully squeezed. Saved the moment it is sent.
  if (c == 'z' || c == 'l') {
    if (armed) { say("disarm first (ESC switch off) -- the lever is live while armed."); return; }
    float now = readLeverNow();
    if (c == 'z') {
      leverRest = now; leverRestSet = true;
      prefs.begin("lever", false); prefs.putFloat("rest", leverRest); prefs.putBool("rset", true); prefs.end();
      say("lever RELEASED = " + String(now, 0) + ", saved."
          + (leverPresent ? " span kept at " + String(leverSpan, 0) + "."
                          : String(" now squeeze it fully, hold, and send l.")));
    } else {
      if (!leverRestSet) { say("send z first, with the lever released."); return; }
      float span = now - leverRest;
      if (span < 0.0f) {
        say("reading FELL when squeezed (" + String(leverRest, 0) + " -> " + String(now, 0)
            + "). swap the pot's two outer wires, then z and l again.");
        return;
      }
      if (span < LEVER_SPAN_MIN) {
        say("only " + String(span, 0) + " counts between released and squeezed -- need "
            + String(LEVER_SPAN_MIN, 0) + ". is the lever moving the slider?");
        return;
      }
      if (span > 4000.0f) span = 4000.0f;
      leverSpan = span; savedG = span; leverPresent = true;
      prefs.begin("lever", false); prefs.putFloat("span", leverSpan); prefs.putBool("valid", true); prefs.end();
      say("lever SQUEEZED = " + String(now, 0) + ". range " + String(leverRest, 0) + " -> "
          + String(leverRest + leverSpan, 0) + " saved. let go; the chair arms with the ESC switch.");
    }
    sendParams();
    return;
  }

  if (c == 'd') {
    if (phone) { say("stream toggle is USB only."); return; }
    streaming = !streaming;
    Serial.println(streaming ? "# stream on" : "# stream off");
    return;
  }

  if (c == 'k' || c == 'm' || c == 'e' || c == 'b' || c == 'r' || c == 'v' || c == 'g' || c == 'n' || c == 'y' || c == 'i') {
    if (!hasNum) { say("? needs a number, e.g. k 0.10"); return; }
    if      (c == 'k') gainAperN   = bounded("gain",     v, 0.01f, GAIN_HARD_MAX);
    else if (c == 'm') iMax        = bounded("max amps", v, 0.10f, I_HARD_CAP);
    else if (c == 'e') erpmCeiling = bounded("ceiling",  v, 200.0f, ERPM_HARD_MAX);
    else if (c == 'b') deadbandN   = bounded("deadband", v, DEADBAND_MIN, 20.0f);
    else if (c == 'r') iRevMax     = bounded("reverse max", v, 0.0f, I_REV_HARD_CAP);
    else if (c == 'v') erpmRevCeil = bounded("reverse ceiling", v, 100.0f, ERPM_REV_HARD_MAX);
    else if (c == 'g') leverSpan   = bounded("lever span", v, 200.0f, 4000.0f);
    else if (c == 'n') creepAmps   = bounded("slow drive current", v, 0.2f, I_HARD_CAP);
    else if (c == 'y') creepErpm   = bounded("slow drive speed", v, 100.0f, CREEP_ERPM_HARD_MAX);
    else if (c == 'i') boostAmps   = bounded("boost current", v, 0.2f, I_HARD_CAP);
    if (phone) { say(String("set ") + c + " " + arg); sendParams(); }
    else help();
    return;
  }

  if (c == 'w') {
    prefs.begin("tune", false);
    prefs.putFloat("k", gainAperN); prefs.putFloat("m", iMax);
    prefs.putFloat("e", erpmCeiling); prefs.putFloat("b", deadbandN);
    prefs.putFloat("r", iRevMax); prefs.putFloat("v", erpmRevCeil); prefs.putFloat("g", leverSpan);
    prefs.putFloat("n", creepAmps); prefs.putFloat("y", creepErpm);
    prefs.putBool("u", useCell); prefs.putFloat("i", boostAmps);
    prefs.putBool("valid", true);
    prefs.end();
    if (leverPresent) { prefs.begin("lever", false); prefs.putFloat("span", leverSpan); prefs.end(); }
    savedK = gainAperN; savedM = iMax; savedE = erpmCeiling; savedB = deadbandN;
    savedR = iRevMax; savedV = erpmRevCeil; savedG = leverSpan; savedN = creepAmps; savedY = creepErpm; savedU = useCell; savedI = boostAmps;
    say("saved: gain " + String(gainAperN, 3) + "  max " + String(iMax, 2) + " A  ceiling "
        + String(erpmCeiling, 0) + " erpm  deadband " + String(deadbandN, 1) + " N  reverse "
        + String(iRevMax, 2) + " A / " + String(erpmRevCeil, 0) + " erpm. these load at switch-on.");
    sendParams();
    return;
  }

  // limits go with every report: the copy sent at connect can be lost if
  // the phone has not finished subscribing yet.
  if (c == 'p') { sendLimits(); sendParams(); if (!phone) help(); return; }
  if (c == 'h') { help(); if (phone) say("phone: k m e b r v g n y u i f o q w x t c p z l. arming is the ESC switch."); return; }

  say("? try h");
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LEVER, INPUT);            // 100k pull-down on the board, no internal pulls on 34
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println();
  Serial.println("=== assist_lever ===");
  Serial.println("# standalone. the ESC switch arms it. phone tunes, never arms.");

  bleBegin();
  Serial.println("# BLE advertising as \"eAssist\"");

  if (HAS_CELL) scale.begin(PIN_DT, PIN_SCK);   // probed further down, and only if the push bar is on
  leverRaw = readLeverNow();             // start the filter where the lever is, not at 0

  prefs.begin("handle", true);
  calSlope       = prefs.getFloat("slope", 0.0f);
  calIntercept   = prefs.getFloat("inter", 0.0f);
  calValid       = prefs.getBool("valid", false);
  savedZero      = prefs.getLong("zero", 0);
  savedZeroValid = prefs.getBool("zvalid", false);
  prefs.end();

  prefs.begin("tune", true);
  if (prefs.getBool("valid", false)) {
    gainAperN   = bounded("saved gain",     prefs.getFloat("k", DEF_GAIN),     0.01f, GAIN_HARD_MAX);
    iMax        = bounded("saved max amps", prefs.getFloat("m", DEF_IMAX),     0.10f, I_HARD_CAP);
    erpmCeiling = bounded("saved ceiling",  prefs.getFloat("e", DEF_ERPM),     200.0f, ERPM_HARD_MAX);
    deadbandN   = bounded("saved deadband", prefs.getFloat("b", DEF_DEADBAND), DEADBAND_MIN, 20.0f);
    iRevMax     = bounded("saved reverse max", prefs.getFloat("r", DEF_IREV), 0.0f, I_REV_HARD_CAP);
    erpmRevCeil = bounded("saved reverse ceiling", prefs.getFloat("v", DEF_EREV), 100.0f, ERPM_REV_HARD_MAX);
    leverSpan   = bounded("saved lever span", prefs.getFloat("g", DEF_SPAN), 200.0f, 4000.0f);
    creepAmps   = bounded("saved slow drive current", prefs.getFloat("n", DEF_CREEP_A), 0.2f, I_HARD_CAP);
    creepErpm   = bounded("saved slow drive speed", prefs.getFloat("y", DEF_CREEP_ERPM), 100.0f, CREEP_ERPM_HARD_MAX);
    savedK = gainAperN; savedM = iMax; savedE = erpmCeiling; savedB = deadbandN;
    useCell     = prefs.getBool("u", DEF_USE_CELL);
    boostAmps   = bounded("saved boost current", prefs.getFloat("i", DEF_BOOST_A), 0.2f, I_HARD_CAP);
    savedR = iRevMax; savedV = erpmRevCeil; savedG = leverSpan; savedN = creepAmps; savedY = creepErpm;
    savedU = useCell; savedI = boostAmps;
    Serial.println("# loaded saved tuning from flash.");
  } else {
    Serial.println("# no saved tuning -- using defaults.");
  }
  prefs.end();

  // lever range, recorded on the chair with z and l. It outranks the span in "tune".
  prefs.begin("lever", true);
  leverRestSet = prefs.getBool("rset", false);
  leverRest    = prefs.getFloat("rest", 0.0f);
  if (prefs.getBool("valid", false) && leverRestSet) {
    leverSpan = bounded("saved lever span", prefs.getFloat("span", DEF_SPAN), LEVER_SPAN_MIN, 4000.0f);
    savedG = leverSpan; leverPresent = true;
  }
  prefs.end();

  if (!HAS_CELL) { useCell = false; savedU = false; }   // a saved "push bar on" means nothing here
  if (useCell) probeHx();

  state = ST_WAIT_ESC;
  recheckFaults();
  if (!useCell)              say("push bar OFF -- no zeroing. waiting for ESC...");
  else if (state != ST_FAULT) say("calibration " + String(calSlope, 1) + " counts/N. waiting for ESC...");
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
    if (useCell) {
      setState(ST_TARING, "ESC answering. taring -- hands off the bar.");
      lastTareTry = 0;
    } else if (leverSqueezed()) {
      setState(ST_HAND_ON_BAR, "not armed -- let go of the lever.");
      lastTareTry = millis();
    } else {
      setState(ST_ARMED, leverPresent ? "ARMED -- push bar off. lever and phone drive."
                                      : "ARMED -- push bar off. lever NOT calibrated (z, l): phone drive only.");
    }
  }
  // Waiting for hands off. A squeezed lever blocks arming on its own; with the
  // push bar off that is the only check, so it is retried quickly.
  if ((state == ST_TARING || state == ST_HAND_ON_BAR) && millis() - lastTareTry >= (useCell ? TARE_RETRY_MS : 300UL)) {
    if (leverSqueezed())  setState(ST_HAND_ON_BAR, "not armed -- let go of the lever.");
    else if (!useCell)    setState(ST_ARMED, "ARMED -- push bar off. lever and phone drive.");
    else if (autoTare())  setState(ST_ARMED, "ARMED. push the bar.");
    else                  setState(ST_HAND_ON_BAR, "not armed -- let go of the bar. retrying.");
    lastTareTry = millis();
  }

  // ---- force ----
  if (useCell && hxOk && scale.is_ready()) {
    long raw = scale.read() - zeroOffset;
    smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;
    forceN = countsToNewtons(smoothed);
  }

  // ---- the law, such as it is ----
  // ---- lever ----
  leverRaw = (1.0f - LEVER_ALPHA) * leverRaw + LEVER_ALPHA * (float)analogRead(PIN_LEVER);
  leverFrac = 0.0f;
  if (leverPresent && leverSpan > 0.0f) {
    float f = (leverRaw - leverRest) / leverSpan;        // below rest (or a broken wire) -> negative -> 0
    f = (f - LEVER_DEADZONE) / (1.0f - LEVER_DEADZONE);
    leverFrac = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
  }

  // Which way is the wheel going? Inside the rest band counts as stopped.
  bool rollingFwd  = lastErpm >  REST_ERPM;
  bool rollingBack = lastErpm < -REST_ERPM;

  float want = 0.0f;
  if (armed) {
    float barN = useCell ? forceN : 0.0f;              // bar off = it asks for nothing
    if (barN > deadbandN) {                            // PUSH
      if (!rollingBack) want = (barN - deadbandN) * gainAperN;
      if (want > iMax) want = iMax;                    // rolling back + push = nothing yet
    } else if (barN < -deadbandN && iRevMax > 0.0f) {  // PULL
      if (!rollingFwd) want = (barN + deadbandN) * gainAperN;     // negative
      if (want < -iRevMax) want = -iRevMax;            // rolling fwd + pull = nothing yet
    }
  }

  // lever: squeeze -> forward help up to max current. larger request wins.
  if (armed && !rollingBack && leverFrac > 0.0f) {
    float lv = leverFrac * iMax;
    if (lv > want) want = lv;
  }

  // slow drive from the phone: replaces the force law while the button is held.
  if (creepOn) {
    unsigned long now = millis();
    if (!armed)                               { creepOn = false; }
    else if (now - creepLast > CREEP_HEARTBEAT_MS) { creepOn = false; say("phone drive off -- lost the phone's signal."); }
    else if (now - creepStart > (driveMode == 2 ? BOOST_MAX_MS : CREEP_MAX_MS)) { creepOn = false; say("phone drive off -- time limit."); }
    else {
      float amps, fade;
      if (driveMode == 2) {
        // BOOST: constant push at walking pace. Full current up to 80% of the
        // normal speed ceiling, fading to nothing at the ceiling.
        amps = boostAmps < iMax ? boostAmps : iMax;
        fade = (erpmCeiling - (float)lastErpm) / (0.2f * erpmCeiling);
      } else {
        // SLOW DRIVE: full current below half its speed limit, fading to nothing at the limit
        amps = creepAmps < iMax ? creepAmps : iMax;
        fade = (creepErpm - (float)lastErpm) / (0.5f * creepErpm);
      }
      if (fade > 1.0f) fade = 1.0f;
      if (fade < 0.0f) fade = 0.0f;
      want = amps * fade;
    }
  }

  // USB demo drive: overrides the force law while it runs, armed only
  if (jogUntil) {
    if (!armed || (long)(millis() - jogUntil) >= 0) { jogUntil = 0; if (armed) say("demo done."); }
    else want = jogAmps;
  }

  // hard ceilings, each direction its own, no blending yet
  ceilingHit = (lastErpm > erpmCeiling) || (lastErpm < -erpmRevCeil);
  if (ceilingHit) want = 0.0f;

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
    Serial.print("   lever "); Serial.print((int)leverRaw);
    if (leverPresent) { Serial.print(" "); Serial.print((int)(leverFrac * 100)); Serial.print("%"); }
    if (ceilingHit) Serial.print("   [CEILING]");
    Serial.println();
  }

  // ---- stream: phone (machine) ----
  if (bleConnected && millis() - lastTele >= 200) { lastTele = millis(); sendTelemetry(); }
  blePump();

  delay(2);
}
