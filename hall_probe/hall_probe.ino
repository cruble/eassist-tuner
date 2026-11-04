/*
  hall_probe.ino  —  read the motor's hall sensors directly, before the ESC sees them
  ESP32-WROOM-32

  WHY THIS EXISTS
  The three hall signal wires can be in any order -- VESC builds its own
  table during detection and sorts the sequence out. But 5V and GROUND have
  to be right, and getting those backwards kills hall sensors.

  So we identify the pinout here, on a board that costs twelve dollars,
  before the motor controller is anywhere near it.

  It also confirms three other things in the same two minutes:
    - that the sensors actually work
    - whether your motor uses 120-degree spacing (six codes) or 60
    - the pole count, independently, by counting state changes per turn

  --------------------------------------------------------------------------
  IDENTIFY THE POWER PAIR FIRST

  On hoverboard motors the convention is near-universal:
      RED    = sensor supply
      BLACK  = sensor ground
      three other colours = the three signals
      a sixth wire, if present = motor temperature (ignore it for now)

  Before connecting anything, measure resistance between your suspected red
  and black. You want a finite reading -- some hundreds of ohms to a few k.
  A dead short or a full open means you have the wrong pair, and you should
  stop and tell me what you measured.

  --------------------------------------------------------------------------
  WIRING

      motor RED    ->  ESP32 3V3        <- NOT 5V. see note below.
      motor BLACK  ->  ESP32 GND
      signal 1     ->  D27
      signal 2     ->  D33
      signal 3     ->  D32

  The signal order genuinely does not matter here. Whichever three wires you
  pick, this sketch shows you the pattern; the controller works out the rest
  later. Pick an order, write it on tape, keep it consistent.

  WHY 3.3 V AND NOT 5 V
  Hall ICs generally run happily from 3.3 V, and at 3.3 V their outputs
  cannot exceed what the ESP32's pins tolerate. Powering them at 5 V puts
  5 V logic into 3.3 V inputs, which the ESP32 will survive briefly and
  resent permanently.

  If the codes never change at 3.3 V, the sensors may genuinely need 5 V.
  Tell me before you try it -- the fix is three resistor dividers, not
  moving the wire.

  --------------------------------------------------------------------------
  SERIAL  115200     screen /dev/cu.usbserial-XXXX 115200   (quit: Ctrl-A k y)

  TURN THE WHEEL SLOWLY BY HAND. Each line is one state change.

  COMMANDS
    r    reset the transition counter
    s    summary -- transitions counted, and what they imply
    h    help
  --------------------------------------------------------------------------
*/

const int PIN_H1 = 27;
const int PIN_H2 = 33;
const int PIN_H3 = 32;

// The valid 120-degree sequence, in forward order. Index = position in the
// cycle; value = the three-bit code at that position.
// H1 is the top bit, H3 the bottom.
const uint8_t SEQ[6] = { 0b101, 0b100, 0b110, 0b010, 0b011, 0b001 };

int sectorOf(uint8_t code) {
  for (int i = 0; i < 6; i++) if (SEQ[i] == code) return i;
  return -1;                         // 000 or 111 -- impossible on a healthy sensor
}

uint8_t  lastCode   = 0xFF;
int      lastSector = -1;
long     transitions = 0;
long     fwd = 0, rev = 0, bad = 0;

uint8_t readCode() {
  return (digitalRead(PIN_H1) ? 4 : 0) |
         (digitalRead(PIN_H2) ? 2 : 0) |
         (digitalRead(PIN_H3) ? 1 : 0);
}

void printBits(uint8_t c) {
  Serial.print(c & 4 ? '1' : '0');
  Serial.print(c & 2 ? '1' : '0');
  Serial.print(c & 1 ? '1' : '0');
}

void summary() {
  Serial.println("# --- summary ---");
  Serial.print("# transitions : "); Serial.println(transitions);
  Serial.print("# forward     : "); Serial.println(fwd);
  Serial.print("# reverse     : "); Serial.println(rev);
  Serial.print("# impossible  : "); Serial.println(bad);
  if (bad > 0) {
    Serial.println("# 000 or 111 seen. A sensor is not reporting, or a wire is loose.");
  }
  Serial.println("#");
  Serial.println("# If you turned the wheel exactly ONE full revolution:");
  Serial.print("#   transitions / 6 = pole pairs = ");
  Serial.println(transitions / 6.0f, 1);
  Serial.println("#   we measured 15 pole pairs by stopwatch, so expect about 90.");
}

void help() {
  Serial.println("# r reset counter | s summary | h help");
  Serial.println("# turn the wheel SLOWLY. one line per state change.");
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // Pull-ups on, in case these sensors have open-drain outputs.
  pinMode(PIN_H1, INPUT_PULLUP);
  pinMode(PIN_H2, INPUT_PULLUP);
  pinMode(PIN_H3, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== hall_probe ===");
  Serial.println("# reads the motor's sensors directly. the ESC is not involved.");
  Serial.print("# H1 on D"); Serial.print(PIN_H1);
  Serial.print(", H2 on D");  Serial.print(PIN_H2);
  Serial.print(", H3 on D");  Serial.println(PIN_H3);
  help();

  uint8_t c = readCode();
  Serial.print("# resting code: "); printBits(c);
  if (sectorOf(c) < 0) {
    Serial.println("   <-- IMPOSSIBLE. check power and ground before turning anything.");
  } else {
    Serial.print("   sector "); Serial.println(sectorOf(c));
  }
  Serial.println("# ready. turn the wheel.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      transitions = fwd = rev = bad = 0;
      Serial.println("# counter reset. turn exactly one full revolution, then press 's'.");
    }
    else if (c == 's') summary();
    else if (c == 'h') help();
  }

  uint8_t code = readCode();
  if (code != lastCode) {
    // crude debounce -- hall edges are clean but wire ends on a breadboard are not
    delay(2);
    if (readCode() != code) return;

    int sec = sectorOf(code);
    transitions++;

    Serial.print("  ");
    printBits(code);

    if (sec < 0) {
      bad++;
      Serial.println("   IMPOSSIBLE -- 000 or 111");
    } else {
      Serial.print("   sector ");
      Serial.print(sec);
      if (lastSector >= 0) {
        int d = (sec - lastSector + 6) % 6;
        if      (d == 1) { fwd++; Serial.print("   -->"); }
        else if (d == 5) { rev++; Serial.print("   <--"); }
        else             { Serial.print("   ?? skipped "); Serial.print(d); }
      }
      Serial.println();
      lastSector = sec;
    }
    lastCode = code;
  }

  delay(1);
}
