/*
  uart_hello.ino  —  first contact with the ESC over UART
  ESP32-WROOM-32  <->  Flipsky Mini V6 MK5  (FW 6.06)

  WHAT THIS IS
  A read-only handshake. It asks the ESC two questions and prints the
  answers. It sends no motion commands of any kind -- there is no
  setCurrent in this file, deliberately. We prove the conversation works
  before we give it anything to do.

  WHY WE ARE NOT USING A LIBRARY
  VescUart targets "FW5+" and makes no claim about 6.x. If the reply
  layout shifted, a library parses the right bytes into the wrong
  variables and hands you plausible nonsense. So this talks the packet
  protocol directly and can dump the raw reply in hex. If a number looks
  wrong, we can see the actual bytes and realign.

  --------------------------------------------------------------------------
  WIRING   (ESC powered OFF while you wire this)

    ESC COMM pin 3  TX/SCL   GREEN    ->   ESP32 D16   (RX2)
    ESC COMM pin 4  RX/SDA   ORANGE   ->   ESP32 D17   (TX2)
    ESC COMM pin 6  GND      WHITE    ->   ESP32 GND

  Wire colours are as built on 10-16-26. They are the cable's own colours,
  not a standard -- if this cable is ever replaced, recount from the dot.

  Positions numbered the way the Flipsky manual lists them, SWITCH end = 1.
  VERIFIED ON THE BENCH 10-12-26: continuity to battery negative beeps at
  position 6, which is where the manual puts GND. Order confirmed.
      1 = SWITCH
      2 = ADC2
      3 = TX/SCL   <- connect (ESP32 listens to this)
      4 = RX/SDA   <- connect (ESP32 transmits into this)
      5 = ADC1
      6 = GND      <- connect
      7 = 3.3V     <- do NOT connect
      8 = 5V       <- do NOT connect

  TX goes to RX and RX goes to TX. Crossed, not straight. If the board
  stays silent, swapping these two is the first thing to try -- it is
  harmless, both are 3.3 V logic.

  Leave COMM pins 7 and 8 alone. The ESP32 is on USB and does not want
  a second supply.

  CHECK YOUR BOARD'S SILKSCREEN for D16 / D17. If your clone labels them
  differently, change PIN_RX / PIN_TX below -- ESP32 UARTs are remappable,
  any free pins will do.

  --------------------------------------------------------------------------
  ESC CONFIG -- THIS IS NOT OPTIONAL

    App Settings -> General -> App to Use  =  UART        (then WRITE it)

  Learned the hard way on 10-16-26: permanent_uart_enabled = 1 is NOT
  enough on this firmware. With App to Use set to No App, the COMM port's
  TX pin sits dead at 0 V and nothing answers. Selecting the UART app is
  what brings the peripheral up.

  Quick check without any code: ESC powered, meter on DC volts, black on
  battery negative, red on the TX wire. An idle UART transmit line sits
  HIGH -- 3.3 V means the port is alive, 0 V means it is not.

  --------------------------------------------------------------------------
  SERIAL  115200     screen /dev/cu.usbserial-XXXX 115200   (quit: Ctrl-A k y)

  COMMANDS
    v    ask for firmware version   <- START HERE
    g    ask for values once
    d    toggle 2 Hz polling of values
    r    toggle raw hex dump of every reply
    h    help

  THE CANARY
  You already know the pack sits around 41 V. If 'g' reports an input
  voltage near 41, the protocol works AND the field offsets are right.
  If it reports 3.2 or 180000, we have found a layout mismatch
  immediately -- and the raw dump will show us where.
  --------------------------------------------------------------------------
*/

// ---------------------------------------------------------------- config

const int  PIN_RX    = 16;        // ESP32 RX2  <- ESC TX
const int  PIN_TX    = 17;        // ESP32 TX2  -> ESC RX
const long VESC_BAUD = 115200;    // VESC default

#define VESC Serial2

const uint8_t COMM_FW_VERSION = 0;
const uint8_t COMM_GET_VALUES = 4;

bool polling = false;
bool rawDump = true;

uint8_t rxBuf[512];

// ---------------------------------------------------------------- protocol

// VESC uses CRC16-CCITT, polynomial 0x1021, initial value 0x0000,
// computed over the payload only.
uint16_t crc16(const uint8_t *buf, unsigned len) {
  uint16_t crc = 0;
  for (unsigned i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
  }
  return crc;
}

// Frame: 0x02 | len | payload | crc_hi | crc_lo | 0x03
void sendPayload(const uint8_t *payload, uint8_t len) {
  uint16_t crc = crc16(payload, len);
  VESC.write((uint8_t)0x02);
  VESC.write(len);
  VESC.write(payload, len);
  VESC.write((uint8_t)(crc >> 8));
  VESC.write((uint8_t)(crc & 0xFF));
  VESC.write((uint8_t)0x03);
  VESC.flush();
}

void request(uint8_t id) { sendPayload(&id, 1); }

// Returns payload length, or -1 on timeout / bad packet.
int readPacket(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  int state = 0, payLen = 0, got = 0, lenNeed = 1, lenGot = 0;
  uint16_t crcRx = 0;

  while (millis() - t0 < timeoutMs) {
    if (!VESC.available()) { delay(1); continue; }
    uint8_t c = VESC.read();

    switch (state) {
      case 0:                                  // waiting for a start byte
        if (c == 0x02)      { lenNeed = 1; lenGot = 0; payLen = 0; state = 1; }
        else if (c == 0x03) { lenNeed = 2; lenGot = 0; payLen = 0; state = 1; }
        break;

      case 1:                                  // length
        payLen = (payLen << 8) | c;
        if (++lenGot >= lenNeed) {
          if (payLen <= 0 || payLen > (int)sizeof(rxBuf)) { state = 0; break; }
          got = 0; state = 2;
        }
        break;

      case 2:                                  // payload
        rxBuf[got++] = c;
        if (got >= payLen) { crcRx = 0; got = 0; state = 3; }
        break;

      case 3:                                  // crc
        crcRx = (crcRx << 8) | c;
        if (++got >= 2) state = 4;
        break;

      case 4:                                  // stop byte + verify
        if (c == 0x03 && crc16(rxBuf, payLen) == crcRx) return payLen;
        state = 0;                             // bad frame, resync
        break;
    }
  }
  return -1;
}

// ---------------------------------------------------------------- decoding

int16_t rd16(const uint8_t *b, int &i) {
  int16_t v = (int16_t)(((uint16_t)b[i] << 8) | b[i + 1]);
  i += 2; return v;
}

int32_t rd32(const uint8_t *b, int &i) {
  int32_t v = ((int32_t)b[i] << 24) | ((int32_t)b[i + 1] << 16) |
              ((int32_t)b[i + 2] <<  8) |  (int32_t)b[i + 3];
  i += 4; return v;
}

void dumpHex(const uint8_t *b, int len) {
  Serial.print("# raw ["); Serial.print(len); Serial.println(" bytes]");
  for (int i = 0; i < len; i++) {
    if (i % 16 == 0) { Serial.println(); Serial.print("#  "); 
                       if (i < 100) Serial.print(' ');
                       if (i < 10)  Serial.print(' ');
                       Serial.print(i); Serial.print(": "); }
    if (b[i] < 0x10) Serial.print('0');
    Serial.print(b[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

void showFwVersion(const uint8_t *b, int len) {
  if (len < 3) { Serial.println("# fw reply too short"); return; }
  Serial.println("# --- firmware ---");
  Serial.print("# version : "); Serial.print(b[1]);
  Serial.print('.');            Serial.println(b[2]);

  // A null-terminated hardware name string follows.
  Serial.print("# hardware: ");
  int i = 3;
  while (i < len && b[i] != 0) { Serial.print((char)b[i]); i++; }
  Serial.println();
  Serial.println("# if that reads 6.6 and 60_MK5, the link is good.");
}

void showValues(const uint8_t *b, int len) {
  // Expected FW5/6 layout, after the packet id at b[0]:
  //   temp_fet i16/10, temp_motor i16/10, motor_current i32/100,
  //   input_current i32/100, id i32/100, iq i32/100, duty i16/1000,
  //   rpm i32, input_voltage i16/10, ...
  if (len < 29) { Serial.println("# values reply shorter than expected"); return; }

  int i = 1;
  float tFet   = rd16(b, i) / 10.0f;
  float tMot   = rd16(b, i) / 10.0f;
  float iMotor = rd32(b, i) / 100.0f;
  float iInput = rd32(b, i) / 100.0f;
  rd32(b, i);                                   // id, not needed yet
  rd32(b, i);                                   // iq, not needed yet
  float duty   = rd16(b, i) / 1000.0f;
  int32_t rpm  = rd32(b, i);                    // ERPM, electrical
  float volts  = rd16(b, i) / 10.0f;

  Serial.println("# --- values ---");
  Serial.print("# input voltage : "); Serial.print(volts, 1);  Serial.println(" V   <-- CANARY, expect ~41");
  Serial.print("# erpm          : "); Serial.println(rpm);
  Serial.print("# motor current : "); Serial.print(iMotor, 2); Serial.println(" A");
  Serial.print("# input current : "); Serial.print(iInput, 2); Serial.println(" A");
  Serial.print("# duty          : "); Serial.println(duty, 3);
  Serial.print("# temp fet      : "); Serial.print(tFet, 1);   Serial.println(" C");
  Serial.print("# temp motor    : "); Serial.print(tMot, 1);   Serial.println(" C");

  // Fault code sits well past the fields we parse; find it defensively.
  if (len > 53) {
    Serial.print("# fault code    : "); Serial.println(b[53]);
  }
}

void handleReply(int len) {
  if (len < 1) { Serial.println("# no reply (timeout)"); return; }
  if (rawDump) dumpHex(rxBuf, len);

  switch (rxBuf[0]) {
    case COMM_FW_VERSION: showFwVersion(rxBuf, len); break;
    case COMM_GET_VALUES: showValues(rxBuf, len);    break;
    default:
      Serial.print("# unexpected packet id "); Serial.println(rxBuf[0]);
      break;
  }
}

void ask(uint8_t id) {
  while (VESC.available()) VESC.read();     // clear stale bytes
  request(id);
  handleReply(readPacket(300));
}

// ---------------------------------------------------------------- sketch

void help() {
  Serial.println("# v = firmware version | g = values once | d = poll 2Hz");
  Serial.println("# r = toggle raw hex   | h = help");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  VESC.begin(VESC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println();
  Serial.println("=== uart_hello ===");
  Serial.println("# read-only. no motion commands in this sketch.");
  Serial.print("# RX2 on D"); Serial.print(PIN_RX);
  Serial.print(", TX2 on D"); Serial.print(PIN_TX);
  Serial.print(", "); Serial.print(VESC_BAUD); Serial.println(" baud");
  help();
  Serial.println("# press 'v' first.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'v') ask(COMM_FW_VERSION);
    else if (c == 'g') ask(COMM_GET_VALUES);
    else if (c == 'd') { polling = !polling;
                         Serial.println(polling ? "# polling ON" : "# polling OFF"); }
    else if (c == 'r') { rawDump = !rawDump;
                         Serial.println(rawDump ? "# raw ON" : "# raw OFF"); }
    else if (c == 'h') help();
  }

  static unsigned long last = 0;
  if (polling && millis() - last >= 500) {
    last = millis();
    ask(COMM_GET_VALUES);
  }
}
