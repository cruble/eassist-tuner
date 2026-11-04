/*
  handle_read.ino  —  Step 1 of the e-assist bench test
  ESP32-WROOM-32 (30-pin)  +  SparkFun Qwiic Scale NAU7802  +  TAL220 10 kg load cell

  What this does:
    - finds the NAU7802 on I2C and says so
    - sets 80 samples/sec, gain 128
    - tares at boot (hands OFF the load cell while it boots)
    - prints raw counts and a smoothed value

  Serial Monitor, 115200 baud. Commands:
    t  = tare (re-zero)
    r  = print raw counts only (good for Serial Plotter)

  Wiring:
    Qwiic breakout cable -> ESP32
      black   GND   -> GND
      red     3.3V  -> 3V3    (NOT 5V)
      blue    SDA   -> GPIO21
      yellow  SCL   -> GPIO22

    TAL220 load cell -> NAU7802 screw terminals (match the labels)
      red    -> E+  (excitation +)
      black  -> E-  (excitation -)
      green  -> A+  (signal +)
      white  -> A-  (signal -)
    If push reads negative, swap green and white. That is the only consequence.
*/

#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

NAU7802 scale;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

long  zeroOffset = 0;
float smoothed   = 0;
bool  rawOnly    = false;

// first-order low pass, about 5 Hz at 80 samples/sec
const float ALPHA = 0.3f;

void doTare() {
  long sum = 0;
  int  n   = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {         // average one second
    if (scale.available()) { sum += scale.getReading(); n++; }
  }
  if (n > 0) zeroOffset = sum / n;
  smoothed = 0;
  Serial.print("tared. zero offset = ");
  Serial.print(zeroOffset);
  Serial.print("  (");
  Serial.print(n);
  Serial.println(" samples)");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== handle_read ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!scale.begin()) {
    Serial.println("NAU7802 NOT FOUND.");
    Serial.println("Check: 3V3 not 5V, SDA=21, SCL=22, GND shared, Qwiic cable seated.");
    while (1) delay(1000);               // stop here on purpose
  }
  Serial.println("NAU7802 found.");

  scale.setLDO(NAU7802_LDO_3V3);
  scale.setGain(NAU7802_GAIN_128);
  scale.setSampleRate(NAU7802_SPS_80);
  scale.calibrateAFE();                  // internal offset calibration
  delay(100);

  Serial.println("Hands off the load cell. Taring...");
  doTare();
  Serial.println("Ready. Press on the free end of the load cell.");
  Serial.println("counts\tsmoothed");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') doTare();
    if (c == 'r') { rawOnly = !rawOnly; }
  }

  static unsigned long lastPrint = 0;

  if (scale.available()) {
    long raw = scale.getReading() - zeroOffset;
    smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;

    if (millis() - lastPrint >= 100) {   // print 10x/sec
      lastPrint = millis();
      if (rawOnly) {
        Serial.println(raw);
      } else {
        Serial.print(raw);
        Serial.print('\t');
        Serial.println((long)smoothed);
      }
    }
  }
}
