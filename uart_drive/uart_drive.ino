/*
  uart_drive.ino  —  first command over UART
  ESP32-WROOM-32  <->  Flipsky Mini V6 MK5

  WHAT CHANGED FROM uart_hello
  That sketch could only listen. This one can also talk. It is the first
  code in this project that makes the motor move, so everything in it is
  built around bounding what can go wrong rather than around convenience.

  THE SAFETY MODEL, STATED PLAINLY
    1. Nothing moves until you arm it. Boot state is disarmed, zero current.
    2. A hard ceiling of 3 A is enforced inside the send function itself,
       so no command path anywhere in this file can exceed it. Your ESC is
       configured for 12 A; this is deliberately a quarter of that.
    3. The default move is a PULSE -- a fixed current for one second, then
       zero. Bounded in strength and in time. It cannot run away.
    4. 'x' zeroes current and disarms, from any state.
    5. The ESC's own 1-second timeout is the backstop: if this sketch
       crashes or the cable falls out, the ESC stops hearing commands and
       releases the motor by itself. We are relying on that on purpose.

  --------------------------------------------------------------------------
  BEFORE YOU UPLOAD THIS -- ALL FOUR

    [ ] Phase bullets connected, same letters as always
    [ ] Wheel OFF the bench, secured, free to spin, nothing in its path
    [ ] The ESC's power button within reach. That is the abort.
    [ ] App Settings -> General -> App to Use = UART   (and written)

  THE WHEEL WILL TURN. That is the point of this sketch.

  --------------------------------------------------------------------------
  WIRING   (unchanged from uart_hello)

    ESC COMM pin 3  TX/SCL   GREEN    ->   ESP32 D16   (RX2)
    ESC COMM pin 4  RX/SDA   ORANGE   ->   ESP32 D17   (TX2)
    ESC COMM pin 6  GND      WHITE    ->   ESP32 GND

  SERIAL  115200    screen /dev/cu.usbserial-XXXX 115200   (quit: Ctrl-A k y)

  COMMANDS
    a           arm
    x           STOP -- zero current and disarm
    p           pulse: the set current for 1 second, then zero
    c <amps>    set the pulse current, e.g.  c 1.5   (capped at 3.0)
    b           brake pulse: regenerative braking for 1 second
    g           read values once
    v           firmware version
    h           help

  START WITH  a  THEN  p  AT THE DEFAULT 1.0 A. Watch what the wheel does
  before you raise anything.
  --------------------------------------------------------------------------
*/

const int  PIN_RX    = 16;
const int  PIN_TX    = 17;
const long VESC_BAUD = 115200;

#define VESC Serial2

const uint8_t COMM_FW_VERSION       = 0;
const uint8_t COMM_GET_VALUES       = 4;
const uint8_t COMM_SET_CURRENT      = 6;
const uint8_t COMM_SET_CURRENT_BRAKE= 7;

// ---- the ceiling. nothing in this file can exceed it. ----
const float I_CAP      = 3.0f;
const float I_DEFAULT  = 1.0f;
const unsigned long PULSE_MS   = 1000;
const unsigned long REFRESH_MS = 20;     // re-send well inside the ESC's 1 s timeout

bool  armed    = false;
bool  pulsing  = false;
bool  braking  = false;
float setAmps  = I_DEFAULT;
unsigned long pulseStart = 0;
unsigned long lastSend   = 0;

uint8_t rxBuf[512];

// ---------------------------------------------------------------- protocol

uint16_t crc16(const uint8_t *buf, unsigned len) {
  uint16_t crc = 0;
  for (unsigned i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

void sendPayload(const uint8_t *payload, uint8_t len) {
  uint16_t crc = crc16(payload, len);
  VESC.write((uint8_t)0x02);
  VESC.write(len);
  VESC.write(payload, len);
  VESC.write((uint8_t)(crc >> 8));
  VESC.write((uint8_t)(crc & 0xFF));
  VESC.write((uint8_t)0x03);
}

void appendInt32(uint8_t *b, int &i, int32_t v) {
  b[i++] = (v >> 24) & 0xFF;  b[i++] = (v >> 16) & 0xFF;
  b[i++] = (v >>  8) & 0xFF;  b[i++] =  v        & 0xFF;
}

// Every motion command in this sketch goes through here. The clamp lives
// in one place on purpose -- there is no second path to the motor.
void sendMotorCurrent(uint8_t id, float amps) {
  if (!armed) amps = 0.0f;
  if (amps >  I_CAP) amps =  I_CAP;
  if (amps < -I_CAP) amps = -I_CAP;
  uint8_t p[5]; int i = 0;
  p[i++] = id;
  appendInt32(p, i, (int32_t)(amps * 1000.0f));
  sendPayload(p, i);
}

void allStop(const char *why) {
  pulsing = braking = false;
  armed = false;                       // clamp applies after this, so send 0 directly
  uint8_t p[5]; int i = 0;
  p[i++] = COMM_SET_CURRENT;
  appendInt32(p, i, 0);
  sendPayload(p, i);
  Serial.print("# STOP -- "); Serial.println(why);
  Serial.println("# disarmed. 'a' to arm again.");
}

int readPacket(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  int state = 0, payLen = 0, got = 0, lenNeed = 1, lenGot = 0;
  uint16_t crcRx = 0;
  while (millis() - t0 < timeoutMs) {
    if (!VESC.available()) { delay(1); continue; }
    uint8_t c = VESC.read();
    switch (state) {
      case 0:
        if (c == 0x02)      { lenNeed = 1; lenGot = 0; payLen = 0; state = 1; }
        else if (c == 0x03) { lenNeed = 2; lenGot = 0; payLen = 0; state = 1; }
        break;
      case 1:
        payLen = (payLen << 8) | c;
        if (++lenGot >= lenNeed) {
          if (payLen <= 0 || payLen > (int)sizeof(rxBuf)) { state = 0; break; }
          got = 0; state = 2;
        }
        break;
      case 2:
        rxBuf[got++] = c;
        if (got >= payLen) { crcRx = 0; got = 0; state = 3; }
        break;
      case 3:
        crcRx = (crcRx << 8) | c;
        if (++got >= 2) state = 4;
        break;
      case 4:
        if (c == 0x03 && crc16(rxBuf, payLen) == crcRx) return payLen;
        state = 0;
        break;
    }
  }
  return -1;
}

int16_t rd16(const uint8_t *b, int &i) {
  int16_t v = (int16_t)(((uint16_t)b[i] << 8) | b[i+1]); i += 2; return v;
}
int32_t rd32(const uint8_t *b, int &i) {
  int32_t v = ((int32_t)b[i] << 24) | ((int32_t)b[i+1] << 16) |
              ((int32_t)b[i+2] << 8) | (int32_t)b[i+3];
  i += 4; return v;
}

// ---------------------------------------------------------------- telemetry

void showValues() {
  while (VESC.available()) VESC.read();
  uint8_t id = COMM_GET_VALUES;
  sendPayload(&id, 1);
  int len = readPacket(300);
  if (len < 29) { Serial.println("# no values"); return; }

  int i = 1;
  rd16(rxBuf, i);                              // temp fet
  rd16(rxBuf, i);                              // temp motor
  float iMotor = rd32(rxBuf, i) / 100.0f;
  float iInput = rd32(rxBuf, i) / 100.0f;
  rd32(rxBuf, i); rd32(rxBuf, i);              // id, iq
  float duty  = rd16(rxBuf, i) / 1000.0f;
  int32_t rpm = rd32(rxBuf, i);
  float volts = rd16(rxBuf, i) / 10.0f;

  Serial.print("#  erpm "); Serial.print(rpm);
  Serial.print("   km/h "); Serial.print(rpm / 482.0f, 2);   // 15 pole pairs, 82.5 mm
  Serial.print("   Imot "); Serial.print(iMotor, 2);
  Serial.print("   Ibat "); Serial.print(iInput, 2);
  Serial.print("   duty "); Serial.print(duty, 3);
  Serial.print("   V "); Serial.println(volts, 1);
  if (len > 53 && rxBuf[53] != 0) {
    Serial.print("# FAULT CODE "); Serial.println(rxBuf[53]);
  }
}

void showFw() {
  while (VESC.available()) VESC.read();
  uint8_t id = COMM_FW_VERSION;
  sendPayload(&id, 1);
  int len = readPacket(300);
  if (len < 3) { Serial.println("# no reply"); return; }
  Serial.print("# fw "); Serial.print(rxBuf[1]); Serial.print('.');
  if (rxBuf[2] < 10) Serial.print('0');
  Serial.print(rxBuf[2]); Serial.print("  hw ");
  for (int i = 3; i < len && rxBuf[i]; i++) Serial.print((char)rxBuf[i]);
  Serial.println();
}

// ---------------------------------------------------------------- commands

void help() {
  Serial.println("# a arm | x STOP | p pulse | c <amps> set | b brake");
  Serial.println("# g values | v firmware | h help");
  Serial.print("# pulse current "); Serial.print(setAmps, 2);
  Serial.print(" A, cap "); Serial.print(I_CAP, 1); Serial.println(" A");
}

void startPulse(bool brake) {
  if (!armed) { Serial.println("# not armed. press 'a' first."); return; }
  pulsing = true; braking = brake;
  pulseStart = millis(); lastSend = 0;
  Serial.print("# "); Serial.print(brake ? "BRAKE" : "PULSE");
  Serial.print(" "); Serial.print(setAmps, 2);
  Serial.print(" A for "); Serial.print(PULSE_MS); Serial.println(" ms");
  showValues();
}

void setup() {
  Serial.begin(115200);
  delay(400);
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // say nothing to the motor until asked
  Serial.println();
  Serial.println("=== uart_drive ===");
  Serial.println("# THIS SKETCH MOVES THE WHEEL.");
  Serial.println("# disarmed at boot. 'x' stops everything, any time.");
  help();
  Serial.println("# sanity check first:");
  showFw();
  showValues();
  Serial.println("# ready. 'a' to arm.");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      char cmd = line.charAt(0);
      if (cmd == 'x') allStop("commanded");
      else if (cmd == 'a') {
        armed = true;
        Serial.println("# ARMED. 'p' to pulse, 'x' to stop.");
      }
      else if (cmd == 'p') startPulse(false);
      else if (cmd == 'b') startPulse(true);
      else if (cmd == 'c') {
        float v = line.substring(1).toFloat();
        if (v > 0.0f && v <= I_CAP) {
          setAmps = v;
          Serial.print("# pulse current now "); Serial.print(setAmps, 2); Serial.println(" A");
        } else {
          Serial.print("# must be above 0 and at most "); Serial.println(I_CAP, 1);
        }
      }
      else if (cmd == 'g') showValues();
      else if (cmd == 'v') showFw();
      else if (cmd == 'h') help();
      else Serial.println("# ? try h");
    }
  }

  // ---- the pulse ----
  if (pulsing) {
    if (millis() - pulseStart >= PULSE_MS) {
      pulsing = false;
      sendMotorCurrent(COMM_SET_CURRENT, 0.0f);
      Serial.println("# pulse ended, current zero");
      delay(60);
      showValues();
      Serial.println("# (motor is coasting -- timeout brake current is 0)");
    } else if (millis() - lastSend >= REFRESH_MS) {
      lastSend = millis();
      sendMotorCurrent(braking ? COMM_SET_CURRENT_BRAKE : COMM_SET_CURRENT, setAmps);
    }
  }

  delay(4);     // keep the loop from starving the watchdog
}
