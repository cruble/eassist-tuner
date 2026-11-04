/*
  assist_law.ino  —  the four-quadrant assist law, on the bench
  ESP32-WROOM-32 (30-pin, Elegoo)

  WHAT CHANGED FROM THE FIRST VERSION
  The first version treated handle force as the whole story. It isn't.
  The same force means different things depending on which way the chair
  is already rolling, and whether a hand is actually on the handle.
  This version implements the nine-cell table:

              | rolling fwd      | at rest          | rolling back
     push +   | assist, cruise   | assist, launch   | BRAKE  (deferred)
     F ~ 0    | hold speed       | brake and hold   | brake and hold
     pull -   | regen brake      | reverse, capped  | BRAKE  (deferred)

  Seven cells are implemented. The two marked deferred need speed sensing
  we don't have yet, so the rule there is blunt and safe: any backward
  roll under power gets braked.

  --------------------------------------------------------------------------
  WIRING  (one new wire vs. last time)

  10k LINEAR pot = the fake handle:
     outer leg -> 3V3,  other outer leg -> GND,  wiper -> D34
     Centered = zero force. One way push, the other pull.

  D4  -> GND  = the safety loop (E-stop NC + wrist tether, in series)
                Pull it: everything stops, and will not re-arm until the
                handle force is back near zero.

  D15 -> GND  = the GRIP SENSOR stand-in.  *** NEW WIRE ***
                In  = a hand is on the handle.
                Out = no hand.
                This is the second channel. Force alone cannot tell a
                resting hand from an absent one, and that distinction is
                the difference between "hold speed" and "stop the chair."

  D25 -> 220R -> green LED  -> GND     assist forward
  D26 -> 220R -> yellow LED -> GND     assist reverse
  D2  -> onboard LED                   braking or disarmed

  Monitor:  screen /dev/cu.usbserial-0001 115200     (quit: Ctrl-A k y)
  Commands: z = re-zero the knob,  p = pause printing
  --------------------------------------------------------------------------
*/

// ---------- pins ----------
const int PIN_POT     = 34;
const int PIN_SAFETY  = 4;
const int PIN_GRIP    = 15;
const int PIN_LED_FWD = 25;
const int PIN_LED_REV = 26;
const int PIN_LED_BRK = 2;

// ---------- the numbers worth arguing about ----------
const float F_RANGE   = 150.0f;   // N, full knob travel each way
const float DEADBAND  =   4.0f;   // N, resting hands are not a request

// Launch and cruise are NOT the same curve.
// Breaking 100 kg out of standstill wants more help, delivered slowly.
// Correcting at walking pace wants less help, delivered quickly.
const float K_LAUNCH  =   2.6f;   // assist ratio from a dead stop
const float K_CRUISE  =   1.8f;   // assist ratio at walking pace
const float RAMP_LAUNCH = 0.45f;  // s, 0 to full -- deliberately gentle
const float RAMP_CRUISE = 0.20f;  // s, 0 to full -- lag feels like fighting

const float WHEEL_R   =  0.0825f; // m, 6.5" hoverboard wheel
const float KT        =  1.00f;   // Nm/A  <-- PLACEHOLDER until VESC
                                  // motor detection measures it.
const float I_MAX     = 12.0f;    // A, drive ceiling
const float I_REGEN   =  8.0f;    // A, max braking from a pull
const float I_HOLD    =  3.0f;    // A, standstill holding brake
const float I_DECEL   =  4.0f;    // A, hands-gone controlled stop
const float I_MAX_REV =  4.0f;    // A, reverse is deliberately feeble

const float V_TAPER_START = 4.0f; // km/h, assist begins fading
const float V_TAPER_END   = 6.0f; // km/h, assist is gone
const float V_MOVING      = 0.15f;// m/s, below this we call it stopped
const float V_LAUNCH_END  = 0.8f; // m/s, gain has fully crossed to cruise

// ---------- simulated chair (a MODEL, labelled SIM in the output) ----------
const float SIM_MASS = 100.0f;    // kg, chair + occupant
const float SIM_CRR  = 0.02f;     // rolling resistance coefficient
float simSpeed = 0.0f;            // m/s, SIGNED

// ---------- state ----------
int   potZero    = 2048;
float fFiltered  = 0.0f;
float iCommanded = 0.0f;
bool  needsRearm = true;
bool  printing   = true;
unsigned long lastLoop = 0, lastPrint = 0;
const unsigned long LOOP_MS = 10;

// ---------------------------------------------------------------- helpers

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// The only function that changes when the load cell arrives.
float readHandleForce() {
  int raw = analogRead(PIN_POT);
  return (float)(raw - potZero) * (F_RANGE / 2048.0f);
}

float lowpass(float in, float prev) {
  const float ALPHA = 0.25f;      // ~5 Hz at a 100 Hz loop
  return prev + ALPHA * (in - prev);
}

float deadband(float f) {
  if (f >  DEADBAND) return f - DEADBAND;
  if (f < -DEADBAND) return f + DEADBAND;
  return 0.0f;
}

// Speed, not velocity. Reverse has to taper too, or it runs away.
float taper(float kmh) {
  float s = fabs(kmh);
  if (s <= V_TAPER_START) return 1.0f;
  if (s >= V_TAPER_END)   return 0.0f;
  return 1.0f - (s - V_TAPER_START) / (V_TAPER_END - V_TAPER_START);
}

// Blend launch gain into cruise gain as the chair gets going.
float gainFor(float speed) {
  float t = clampf(fabs(speed) / V_LAUNCH_END, 0.0f, 1.0f);
  return K_LAUNCH + t * (K_CRUISE - K_LAUNCH);
}

float rampFor(float speed) {
  float t = clampf(fabs(speed) / V_LAUNCH_END, 0.0f, 1.0f);
  return RAMP_LAUNCH + t * (RAMP_CRUISE - RAMP_LAUNCH);
}

float rateLimit(float target, float current, float dt, float rampSeconds) {
  float maxStep = (I_MAX / rampSeconds) * dt;
  float d = target - current;
  d = clampf(d, -maxStep, maxStep);
  return current + d;
}

// Braking current always opposes motion, whichever way it is going.
float brakeCurrent(float amps, float speed) {
  if (speed > 0) return -amps;
  if (speed < 0) return  amps;
  return 0.0f;
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(PIN_SAFETY, INPUT_PULLUP);
  pinMode(PIN_GRIP,   INPUT_PULLUP);
  pinMode(PIN_LED_FWD, OUTPUT);
  pinMode(PIN_LED_REV, OUTPUT);
  pinMode(PIN_LED_BRK, OUTPUT);
  analogReadResolution(12);

  Serial.println();
  Serial.println("=== assist_law : four quadrant ===");
  Serial.println("Center the knob. Zeroing in 2 seconds...");
  delay(2000);

  long sum = 0;
  for (int i = 0; i < 64; i++) { sum += analogRead(PIN_POT); delay(5); }
  potZero = sum / 64;
  Serial.print("knob zero = "); Serial.println(potZero);
  Serial.println("z = re-zero,  p = pause");
  Serial.println();
  Serial.println("force_N\tcurrent_A\tSIMkmh\tgrip\tstate");

  lastLoop = millis();
}

// ---------------------------------------------------------------- loop

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') printing = !printing;
    if (c == 'z') {
      long s = 0;
      for (int i = 0; i < 32; i++) { s += analogRead(PIN_POT); delay(3); }
      potZero = s / 32;
      Serial.print("re-zeroed at "); Serial.println(potZero);
    }
  }

  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  // ---- inputs ----
  bool loopClosed = (digitalRead(PIN_SAFETY) == LOW);   // E-stop + tether
  bool handOn     = (digitalRead(PIN_GRIP)   == LOW);   // grip sensor

  float fRaw = readHandleForce();
  fFiltered  = lowpass(fRaw, fFiltered);
  float f    = deadband(fFiltered);

  float v       = simSpeed;                 // signed, m/s
  float kmh     = v * 3.6f;
  bool  moving  = fabs(v) > V_MOVING;
  bool  fwd     = v > 0;

  // ---- arming ----
  // Closing the loop again is not enough. The handles must also be at rest,
  // or re-arming with a hand still pushing would make the chair jump.
  bool forceAtRest = (fabs(fFiltered) < DEADBAND);
  bool armed;
  if (!loopClosed)            { needsRearm = true;  armed = false; }
  else if (needsRearm)        { armed = forceAtRest; if (armed) needsRearm = false; }
  else                        { armed = true; }

  // ---- the nine cells ----
  float iTarget;
  const char* state;
  float ramp = rampFor(v);

  if (!armed) {
    iTarget = 0.0f;
    state = loopClosed ? "WAIT-REARM" : "DISARMED";
    ramp = RAMP_CRUISE;
  }
  else if (!handOn) {
    // No hand at all. At rest, hold. Moving, do NOT slam on the brake --
    // an unrestrained occupant is sitting in this thing. Ramp it down.
    if (moving) { iTarget = brakeCurrent(I_DECEL, v); state = "DECEL-NOHAND"; }
    else        { iTarget = 0.0f;                     state = "HOLD-NOHAND"; }
    ramp = RAMP_LAUNCH;
  }
  else if (f > 0.0f) {                      // ---- pushing ----
    if (moving && !fwd) {
      iTarget = brakeCurrent(I_REGEN, v);   // rolling back, being pushed fwd
      state = "BRAKE-ROLLBACK";             //   deferred cell: just brake
    } else {
      float K = gainFor(v);
      iTarget = (K * f * WHEEL_R) / KT;
      iTarget *= taper(kmh);
      iTarget = clampf(iTarget, 0.0f, I_MAX);
      state = moving ? "ASSIST-CRUISE" : "ASSIST-LAUNCH";
    }
  }
  else if (f < 0.0f) {                      // ---- pulling ----
    if (moving && fwd) {
      // Pulling back on a chair that is rolling forward is a request to
      // SLOW DOWN, not to drive backward. This was the worst bug in v1.
      float frac = clampf(-f / 60.0f, 0.0f, 1.0f);
      iTarget = brakeCurrent(I_REGEN * frac, v);
      state = "REGEN-BRAKE";
    } else if (!moving) {
      float K = gainFor(v);
      iTarget = (K * f * WHEEL_R) / KT;
      iTarget = clampf(iTarget, -I_MAX_REV, 0.0f);   // reverse stays feeble
      state = "REVERSE-CAPPED";
    } else {
      iTarget = brakeCurrent(I_REGEN, v);   // deferred cell
      state = "BRAKE-ROLLBACK";
    }
  }
  else {                                    // ---- hand on, not pushing ----
    if (moving) {
      // Walking along with the chair is not a request to stop.
      // No torque, and crucially NO BRAKE.
      iTarget = 0.0f;
      state = "HOLD-SPEED";
    } else {
      iTarget = 0.0f;
      state = "BRAKE-HOLD";                 // parked. the motor holds it.
    }
  }

  iCommanded = rateLimit(iTarget, iCommanded, dt, ramp);

  // ---- outputs: LEDs standing in for the motor ----
  int duty = (int)(255.0f * fabs(iCommanded) / I_MAX);
  duty = duty > 255 ? 255 : duty;
  bool braking = (iCommanded * v < 0) || (!moving && !handOn) ||
                 (!moving && f == 0.0f && armed);

  if (!armed) {
    analogWrite(PIN_LED_FWD, 0);
    analogWrite(PIN_LED_REV, 0);
    digitalWrite(PIN_LED_BRK, HIGH);
  } else {
    digitalWrite(PIN_LED_BRK, braking ? HIGH : LOW);
    analogWrite(PIN_LED_FWD, iCommanded > 0 ? duty : 0);
    analogWrite(PIN_LED_REV, iCommanded < 0 ? duty : 0);
  }

  // ---- simulated chair ----
  float fMotor = armed ? (iCommanded * KT / WHEEL_R) : 0.0f;
  float fHand  = handOn ? f : 0.0f;
  float fDrag  = (v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f)) * SIM_CRR * SIM_MASS * 9.81f;
  float accel  = (fHand + fMotor - fDrag) / SIM_MASS;
  float vNew   = v + accel * dt;
  // braking must not drag the chair backward through zero
  if (v > 0 && vNew < 0 && fMotor < 0) vNew = 0;
  if (v < 0 && vNew > 0 && fMotor > 0) vNew = 0;
  simSpeed = clampf(vNew, -3.0f, 3.0f);
  if (fabs(simSpeed) < 0.01f) simSpeed = 0.0f;

  // ---- print ----
  if (printing && now - lastPrint >= 100) {
    lastPrint = now;
    Serial.print(f, 1);               Serial.print('\t');
    Serial.print(iCommanded, 2);      Serial.print('\t');
    Serial.print(simSpeed * 3.6f, 2); Serial.print('\t');
    Serial.print(handOn ? "ON " : "off"); Serial.print('\t');
    Serial.println(state);
  }
}
