/*
  ppm_probe.ino  —  find the ESC's receiver port, then test throttle mapping
  ESP32-WROOM-32

  WHAT THIS DOES
  Outputs a standard servo pulse. Nothing else. Use it to work out which
  3-pin connector on the ESC is the receiver input, and then to exercise
  the ESC's PPM mapping before any force sensing is involved.

  WIRING
    ESP32 D27  ->  ESC signal pin (white wire on a servo lead)
    ESP32 GND  ->  ESC ground     (black wire)
    Leave the ESC's 5V disconnected. The ESP32 stays on USB.

  HOW TO USE IT
    1. Battery OFF, motor phases DISCONNECTED. We are only looking for
       a number on a screen, not turning anything.
    2. Upload, open serial at 115200.
    3. Connect the ESC (battery on, USB to VESC Tool).
    4. VESC Tool -> App Settings -> PPM. There is a live pulse display.
    5. Type pulse widths here and watch whether that display follows.

  COMMANDS   (type a number, press enter)
    1000      full one way
    1500      centre / neutral
    2000      full the other way
    s         sweep slowly 1100 -> 1900 -> 1100, repeating
    x         stop sweeping, hold centre
    o         output off entirely (no pulses at all)

  If the VESC Tool display tracks what you type, you have found the port.
  If it does not, move the lead to the next 3-pin connector and try again.
  A 3.3 V logic pulse on the wrong port will not damage anything.
  --------------------------------------------------------------------------
*/

const int PIN_PPM = 27;

int  pulseUs   = 1500;      // what we are currently sending
bool sweeping  = false;
bool outputOn  = true;
int  sweepDir  = 1;

const int FRAME_MS = 20;    // 50 Hz, the servo standard

void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(PIN_PPM, OUTPUT);
  digitalWrite(PIN_PPM, LOW);

  Serial.println();
  Serial.println("=== ppm_probe ===");
  Serial.println("D27 = signal, GND = ground. ESC 5V not connected.");
  Serial.println("Type 1000-2000 to set pulse width.");
  Serial.println("s = sweep, x = hold centre, o = output off");
  Serial.print("sending "); Serial.print(pulseUs); Serial.println(" us");
}

void loop() {
  // ---- commands ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      if (line == "s") { sweeping = true;  outputOn = true;
                         Serial.println("# sweeping"); }
      else if (line == "x") { sweeping = false; pulseUs = 1500; outputOn = true;
                         Serial.println("# centre, 1500 us"); }
      else if (line == "o") { outputOn = !outputOn;
                         Serial.println(outputOn ? "# output ON" : "# output OFF"); }
      else {
        int v = line.toInt();
        if (v >= 800 && v <= 2200) {
          pulseUs = v; sweeping = false; outputOn = true;
          Serial.print("# sending "); Serial.print(pulseUs); Serial.println(" us");
        } else {
          Serial.println("# 1000-2000 please, or s / x / o");
        }
      }
    }
  }

  // ---- sweep ----
  static unsigned long lastStep = 0;
  if (sweeping && millis() - lastStep >= 20) {
    lastStep = millis();
    pulseUs += sweepDir * 4;
    if (pulseUs >= 1900) { pulseUs = 1900; sweepDir = -1; }
    if (pulseUs <= 1100) { pulseUs = 1100; sweepDir =  1; }
  }

  // ---- emit one frame ----
  if (outputOn) {
    digitalWrite(PIN_PPM, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(PIN_PPM, LOW);
    delay(FRAME_MS - (pulseUs / 1000) - 1);
  } else {
    digitalWrite(PIN_PPM, LOW);
    delay(FRAME_MS);
  }

  // ---- occasional status ----
  static unsigned long lastSay = 0;
  if (millis() - lastSay >= 1000) {
    lastSay = millis();
    Serial.print("us="); Serial.print(pulseUs);
    Serial.println(outputOn ? "" : "  (output off)");
  }
}
