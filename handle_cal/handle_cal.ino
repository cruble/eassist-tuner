/*
  handle_cal.ino  —  handle force sensor with built-in calibration
  ESP32-WROOM-32  +  HX711  +  bar load cell

  WHY THIS EXISTS
  The counts-per-newton figure belongs to the MOUNTING, not just the cell.
  Change the bracket, the lever arm, or the angle the handle pulls at, and
  the number changes. Since this rig will move from bench to handle to
  chair, calibration has to be something you re-run in two minutes, not a
  one-time shop procedure. So it lives in here.

  Calibration survives a reboot (stored in flash).

  --------------------------------------------------------------------------
  WIRING   (unchanged)
    HX711 VCC -> 3V3   GND -> GND   DT -> D18   SCK -> D19
    Load cell -> HX711:  red E+   black E-   green A+   white A-
    D25 -> 220R -> green LED -> GND      push
    D26 -> 220R -> yellow LED -> GND     pull

  SERIAL  115200
    screen /dev/cu.usbserial-XXXX 115200        quit: Ctrl-A k y
    To capture a log to a file:  screen -L /dev/cu.usbserial-XXXX 115200
    (writes screenlog.0 in the current directory)

  COMMANDS
    t            tare -- re-zero with no load
    c <v> <unit> record a calibration point at the force applied RIGHT NOW
                 units: n (newtons), kg, lb.  default kg
                 e.g.   c 5 kg      c 11 lb      c 49 n
    l            list calibration points as CSV
    f            fit the points and show counts-per-newton
    s            save calibration to flash
    p            print current calibration
    z            clear all calibration points
    d            toggle CSV data streaming (ms,raw,smoothed,newtons)
    h            help
  --------------------------------------------------------------------------
*/

#include "HX711.h"
#include <Preferences.h>

const int PIN_DT      = 18;
const int PIN_SCK     = 19;
const int PIN_LED_FWD = 25;
const int PIN_LED_REV = 26;

HX711 scale;
Preferences prefs;

// ---------- calibration ----------
// Model: counts = slope * newtons + intercept
// We fit both so a stubborn mount offset shows up rather than hiding.
float calSlope     = 0.0f;      // counts per newton. 0 = not calibrated
float calIntercept = 0.0f;
bool  calValid     = false;

const int MAX_PTS = 12;
float ptN[MAX_PTS];             // applied force, newtons
float ptC[MAX_PTS];             // measured counts
int   nPts = 0;

// ---------- state ----------
long  zeroOffset = 0;
float smoothed   = 0.0f;
float peakSeen   = 20000.0f;
bool  streaming  = false;

const float ALPHA = 0.25f;

// ---------------------------------------------------------------- helpers

// Average readings over a window. Used for taring and for cal points,
// where a steady number matters more than a fast one.
float averageCounts(unsigned long ms) {
  double sum = 0; long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (scale.is_ready()) { sum += (double)(scale.read() - zeroOffset); n++; }
  }
  return n ? (float)(sum / n) : 0.0f;
}

void doTare() {
  Serial.println("# taring, hands off...");
  zeroOffset = 0;                       // average raw, not offset-corrected
  double sum = 0; long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (scale.is_ready()) { sum += (double)scale.read(); n++; }
  }
  if (n) zeroOffset = (long)(sum / n);
  smoothed = 0.0f;
  Serial.print("# zero offset = "); Serial.println(zeroOffset);
}

float countsToNewtons(float counts) {
  if (!calValid || calSlope == 0.0f) return NAN;
  return (counts - calIntercept) / calSlope;
}

// Least squares fit of counts = slope*N + intercept
void fitCalibration() {
  if (nPts < 2) {
    Serial.println("# need at least 2 points. 3 or 4 is better.");
    return;
  }
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < nPts; i++) {
    sx += ptN[i]; sy += ptC[i];
    sxx += (double)ptN[i] * ptN[i];
    sxy += (double)ptN[i] * ptC[i];
  }
  double d = nPts * sxx - sx * sx;
  if (fabs(d) < 1e-9) { Serial.println("# points are degenerate"); return; }

  calSlope     = (float)((nPts * sxy - sx * sy) / d);
  calIntercept = (float)((sy * sxx - sx * sxy) / d);
  calValid     = true;

  // R^2, so you can see whether it is actually linear
  double meanY = sy / nPts, ssTot = 0, ssRes = 0;
  for (int i = 0; i < nPts; i++) {
    double pred = calSlope * ptN[i] + calIntercept;
    ssRes += (ptC[i] - pred) * (ptC[i] - pred);
    ssTot += (ptC[i] - meanY) * (ptC[i] - meanY);
  }
  double r2 = ssTot > 0 ? 1.0 - ssRes / ssTot : 0.0;

  Serial.println("# --- fit ---");
  Serial.print("# counts per newton : "); Serial.println(calSlope, 2);
  Serial.print("# intercept         : "); Serial.println(calIntercept, 1);
  Serial.print("# R^2               : "); Serial.println(r2, 5);
  Serial.print("# full scale at 200 N would be ");
  Serial.print(calSlope * 200.0f, 0); Serial.println(" counts");
  if (r2 < 0.995) Serial.println("# NOTE: R^2 below 0.995 -- suspect the mount, not the cell.");
  Serial.println("# 's' to save.");
}

void printCal() {
  if (!calValid) { Serial.println("# not calibrated"); return; }
  Serial.print("# slope "); Serial.print(calSlope, 2);
  Serial.print(" counts/N   intercept "); Serial.println(calIntercept, 1);
}

void listPoints() {
  Serial.println("# CSV follows");
  Serial.println("point,newtons,counts");
  for (int i = 0; i < nPts; i++) {
    Serial.print(i); Serial.print(',');
    Serial.print(ptN[i], 3); Serial.print(',');
    Serial.println(ptC[i], 1);
  }
}

void saveCal() {
  prefs.begin("handle", false);
  prefs.putFloat("slope", calSlope);
  prefs.putFloat("inter", calIntercept);
  prefs.putBool("valid", calValid);
  prefs.end();
  Serial.println("# saved to flash");
}

void loadCal() {
  prefs.begin("handle", true);
  calSlope     = prefs.getFloat("slope", 0.0f);
  calIntercept = prefs.getFloat("inter", 0.0f);
  calValid     = prefs.getBool("valid", false);
  prefs.end();
}

// ---------------------------------------------------------------- commands

void addPoint(String arg) {
  arg.trim();
  float v = arg.toFloat();
  if (v == 0.0f) { Serial.println("# usage: c <value> <n|kg|lb>"); return; }

  float newtons;
  String a = arg; a.toLowerCase();
  if (a.indexOf("lb") >= 0)      newtons = v * 4.4482f;
  else if (a.indexOf('n') >= 0 && a.indexOf("kg") < 0) newtons = v;
  else                           newtons = v * 9.80665f;   // kg, the default

  Serial.print("# hold "); Serial.print(newtons, 2);
  Serial.println(" N steady -- sampling 3 s");
  float c = averageCounts(3000);

  if (nPts >= MAX_PTS) { Serial.println("# point list full"); return; }
  ptN[nPts] = newtons; ptC[nPts] = c; nPts++;

  Serial.print("# point "); Serial.print(nPts - 1);
  Serial.print(": "); Serial.print(newtons, 2); Serial.print(" N -> ");
  Serial.print(c, 1); Serial.println(" counts");
  if (nPts >= 2) Serial.println("# 'f' to fit");
}

void help() {
  Serial.println("# t tare | c <v> <n|kg|lb> point | l list | f fit");
  Serial.println("# s save | p print cal | z clear | d stream | h help");
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== handle_cal ===");

  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_REV, OUTPUT);

  scale.begin(PIN_DT, PIN_SCK);
  unsigned long t0 = millis();
  while (!scale.is_ready()) {
    if (millis() - t0 > 3000) {
      Serial.println("HX711 NOT RESPONDING. Check 3V3, GND, D18, D19,");
      Serial.println("and the load cell's four wires in E+/E-/A+/A-.");
      while (1) delay(1000);
    }
    delay(50);
  }
  Serial.println("# HX711 ok");

  loadCal();
  printCal();

  delay(400);
  doTare();
  help();
  Serial.println("# ready");
}

// ---------------------------------------------------------------- loop

void loop() {
  // ---- serial commands ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      char cmd = line.charAt(0);
      String arg = line.substring(1);
      switch (cmd) {
        case 't': doTare();                 break;
        case 'c': addPoint(arg);            break;
        case 'l': listPoints();             break;
        case 'f': fitCalibration();         break;
        case 's': saveCal();                break;
        case 'p': printCal();               break;
        case 'z': nPts = 0; Serial.println("# points cleared"); break;
        case 'd': streaming = !streaming;
                  if (streaming) Serial.println("ms,raw,smoothed,newtons");
                  else Serial.println("# stream off");
                  break;
        case 'h': help();                   break;
        default:  Serial.println("# ? try h"); break;
      }
    }
  }

  if (!scale.is_ready()) return;

  long raw = scale.read() - zeroOffset;
  smoothed = (1.0f - ALPHA) * smoothed + ALPHA * (float)raw;

  // ---- LEDs ----
  float mag = fabs(smoothed);
  if (mag > peakSeen) peakSeen = mag;
  peakSeen *= 0.9995f;
  if (peakSeen < 5000.0f) peakSeen = 5000.0f;
  int duty = (int)(255.0f * mag / peakSeen);
  if (duty > 255) duty = 255;
  if (duty < 4) duty = 0;
  analogWrite(PIN_LED_FWD, smoothed > 0 ? duty : 0);
  analogWrite(PIN_LED_REV, smoothed < 0 ? duty : 0);

  // ---- output ----
  static unsigned long lastOut = 0;
  unsigned long now = millis();

  if (streaming) {
    if (now - lastOut >= 100) {           // 10 Hz CSV
      lastOut = now;
      float n = countsToNewtons(smoothed);
      Serial.print(now);          Serial.print(',');
      Serial.print(raw);          Serial.print(',');
      Serial.print((long)smoothed); Serial.print(',');
      if (isnan(n)) Serial.println("");
      else          Serial.println(n, 2);
    }
  } else {
    if (now - lastOut >= 500) {           // 2 Hz human-readable
      lastOut = now;
      float n = countsToNewtons(smoothed);
      Serial.print("counts ");
      Serial.print((long)smoothed);
      if (!isnan(n)) { Serial.print("   "); Serial.print(n, 1); Serial.print(" N"); }
      Serial.println();
    }
  }
}
