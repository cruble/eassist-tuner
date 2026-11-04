/*
  handle_hx711.ino  —  first light for the handle sensor
  ESP32-WROOM-32  +  HX711  +  bar load cell

  GOAL OF THIS SKETCH
  Prove the sensor is alive and responds to force. No calibration, no
  newtons, no control law. Just: does pushing on the bar move the number,
  does letting go bring it back, and is it stable when untouched.

  Calibration against the fish scale comes next, once we trust the reading.

  --------------------------------------------------------------------------
  LIBRARY
  Arduino IDE -> Library Manager -> search "HX711 Arduino Library"
  by Bogdan Necula. Install that one.

  WIRING
    HX711 VCC -> 3V3      (not 5V -- keeps its output inside the
                           ESP32's 3.3 V logic range)
    HX711 GND -> GND
    HX711 DT  -> D18
    HX711 SCK -> D19

    Load cell -> HX711 screw terminals, by label:
      red   -> E+        black -> E-
      green -> A+        white -> A-
    If push reads negative, swap green and white. That's the only effect.

    LEDs, carried over from the last sketch:
      D25 -> 220R -> green LED  -> GND     push
      D26 -> 220R -> yellow LED -> GND     pull

  SERIAL
    screen /dev/cu.usbserial-XXXX 115200     (quit: Ctrl-A k y)
    t = tare (re-zero)      r = reset the LED auto-scale
  --------------------------------------------------------------------------
*/

#include "HX711.h"

const int PIN_DT      = 18;
const int PIN_SCK     = 19;
const int PIN_LED_FWD = 25;
const int PIN_LED_REV = 26;

HX711 scale;

long  zeroOffset = 0;
float smoothed   = 0.0f;

// The LEDs auto-scale to the biggest push seen since boot, so they do
// something useful before we know the counts-per-newton. Decays slowly
// so one accidental whack doesn't deaden them forever.
float peakSeen   = 20000.0f;     // starting guess, in raw counts

const float ALPHA = 0.25f;       // low-pass, gentle at 10 samples/sec

void doTare() {
  Serial.println("taring -- hands off...");
  long sum = 0;
  int  n   = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (scale.is_ready()) { sum += scale.read(); n++; }
  }
  if (n > 0) zeroOffset = sum / n;
  smoothed = 0.0f;
  Serial.print("zero offset = "); Serial.print(zeroOffset);
  Serial.print("   (from "); Serial.print(n); Serial.println(" samples)");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== handle_hx711 ===");

  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_REV, OUTPUT);

  scale.begin(PIN_DT, PIN_SCK);

  // Wait for the chip, but don't hang silently if it isn't there.
  unsigned long t0 = millis();
  while (!scale.is_ready()) {
    if (millis() - t0 > 3000) {
      Serial.println("HX711 NOT RESPONDING.");
      Serial.println("Check: VCC on 3V3, GND shared, DT on D18, SCK on D19,");
      Serial.println("and that the load cell's four wires are in E+/E-/A+/A-.");
      Serial.println("(A missing load cell alone can cause this -- the HX711");
      Serial.println(" needs the bridge connected to produce a reading.)");
      while (1) delay(1000);
    }
    delay(50);
  }
  Serial.println("HX711 responding.");

  delay(500);
  doTare();

  Serial.println();
  Serial.println("Press on the free end of the bar.");
  Serial.println("t = tare,  r = reset LED scale");
  Serial.println("raw\tsmoothed\tbar");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') doTare();
    if (c == 'r') { peakSeen = 20000.0f; Serial.println("LED scale reset"); }
  }

  if (!scale.is_ready()) return;

  long raw = scale.read() - zeroOffset;
  smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;

  // ---- LEDs ----
  float mag = fabs(smoothed);
  if (mag > peakSeen) peakSeen = mag;        // grow to fit
  peakSeen *= 0.9995f;                       // and decay very slowly
  if (peakSeen < 5000.0f) peakSeen = 5000.0f;

  int duty = (int)(255.0f * mag / peakSeen);
  if (duty > 255) duty = 255;
  if (duty < 4)   duty = 0;                  // a crude deadband, visual only

  analogWrite(PIN_LED_FWD, smoothed > 0 ? duty : 0);
  analogWrite(PIN_LED_REV, smoothed < 0 ? duty : 0);

  // ---- print ----
  Serial.print(raw);
  Serial.print('\t');
  Serial.print((long)smoothed);
  Serial.print('\t');

  // A little ASCII bar so you can see it move without reading digits.
  int bars = (int)(20.0f * mag / peakSeen);
  if (bars > 20) bars = 20;
  Serial.print(smoothed < 0 ? '-' : '+');
  for (int i = 0; i < bars; i++) Serial.print('#');
  Serial.println();
}
