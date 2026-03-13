#include <Wire.h>
#include <Zumo32U4.h>

// HARDWARE OBJECTS
Zumo32U4LCD display;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

// MOTOR ORIENTATION
// Set to true if the robot's motors are mounted upside-down
// (drives backward when it should go forward)
const bool INVERT_MOTORS = false;

//  DYNAMIC FLOOR CALIBRATION
//  CALIBRATION_SAMPLES  — readings averaged per sensor when A/C starts a match
//  CALIBRATION_DELAY_MS — pause between samples during floor calibration
//  BOUNDARY_TOLERANCE   — allowed deviation from baseline before boundary hit

#define NUM_LINE_SENSORS 5

const uint8_t CALIBRATION_SAMPLES = 30;
const uint8_t CALIBRATION_DELAY_MS = 5;
const uint16_t BOUNDARY_TOLERANCE = 400;

uint16_t lineSensorValues[NUM_LINE_SENSORS];
uint16_t floorBaseline[NUM_LINE_SENSORS];
bool floorCalibrated = false;

// PROXIMITY THRESHOLDS
// Sensor returns 0–6.  Lower = more sensitive (detects farther away).
const uint8_t PROX_SEARCH_THRESHOLD = 3; // used during scan / search / cross-ring
const uint8_t PROX_ATTACK_THRESHOLD = 4; // used during attack (closer = real target)
const uint8_t PROX_LOCK_THRESHOLD = 2;   // lower threshold used to keep lock once attacking

// SPEED SETTINGS
const int16_t SEARCH_SPEED = 350; // CHANGED: fast forward during search — cover ground aggressively
const int16_t ATTACK_SPEED = 370; // max — full ram
const int16_t SCAN_SPEED = 350;   // 360° scan
const int16_t EVADE_SPEED = 370;  // boundary escape burst
const int16_t TURN_SPEED = 280;   // general turning

// --- SEARCH PATTERN CONSTANTS ---
// After bouncing off a boundary the robot turns a "bounce angle" before
// charging across the ring again.  The angle rotates through a set of
// offsets so successive passes cut across different chords of the circle,
// covering more arena area than repeating the same path.
const int16_t SEARCH_BOUNCE_TURN_BASE_MS = 220;  // base turn time (shortest chord)
const int16_t SEARCH_BOUNCE_TURN_STEP_MS = 70;   // added per index in the angle table
const uint8_t SEARCH_ANGLE_TABLE_SIZE    = 5;     // number of distinct bounce angles
// Slight left-right bias keeps the robot from hugging one wall.
// The table stores signed offsets: + = extra CW ms, - = extra CCW ms.
const int16_t SEARCH_ANGLE_BIAS[] = { 0, 40, -30, 60, -50 };

// BUZZER VOLUME
const uint8_t BUZZER_VOLUME = 9; // 0–15 — applies to every sound

// SCAN / SEARCH TIMING
const unsigned long SCAN_TIMEOUT = 2500;     // ms for 360° spin
const unsigned long CROSS_RING_TIME = 1200;  // ms driving to far side
const unsigned long REACQUIRE_TIMEOUT = 1100; // ms to reacquire target before giving up (longer for weak IR)

// --- IMPROVED TRACKING CONSTANTS ---
// Sensor peak-hold: remembers the best reading for this many ms,
// bridging brief dropouts caused by the black opponent's weak IR reflection.
const unsigned long SENSOR_HOLD_MS = 120;

// Confidence tracking: a score (0–255) that rises when the target is seen
// and falls when it is not.  Prevents a single noisy cycle from breaking lock.
const uint8_t CONFIDENCE_GAIN_STRONG = 55;  // added per loop when signal >= PROX_ATTACK_THRESHOLD
const uint8_t CONFIDENCE_GAIN_WEAK   = 18;  // added per loop when signal >= PROX_LOCK_THRESHOLD
const uint8_t CONFIDENCE_DECAY       = 5;   // subtracted per loop with no signal
const uint8_t CONFIDENCE_LOCK_MIN    = 30;  // stay in ATK_LOCK above this
const uint8_t CONFIDENCE_COAST_MIN   = 10;  // keep coasting above this

// Coast phase: after losing signal, drive forward briefly
// (opponent may be too close for IR, or momentary noise).
const unsigned long COAST_DURATION_MS = 200;

// Reacquire sweep: oscillate left/right with forward bias.
const unsigned long REACQUIRE_SWEEP_HALF_MS = 280; // ms per half-oscillation
const int16_t SWEEP_BASE_SPEED = 200;              // forward component during sweep
const int16_t SWEEP_INITIAL_WIDTH = 180;            // initial turn differential
const int16_t SWEEP_WIDTH_GROW   = 40;              // grows each half-period
const int16_t SWEEP_WIDTH_MAX    = 380;             // capped here

// Proportional steering clamps during ATK_LOCK.
const int16_t STEER_ADJ_MIN = 30;   // never adjust less than this (avoids dead-band)
const int16_t STEER_ADJ_MAX = 300;  // never adjust more than this

// STATE MACHINE
enum RobotState
{
  STATE_IDLE,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// Search pattern
bool searchTurnDir = false;       // false = right, true = left
unsigned long searchLegStart = 0;
uint8_t searchBounceIndex = 0;    // cycles through SEARCH_ANGLE_TABLE_SIZE angles
uint8_t searchBounceCount = 0;    // total bounces since entering SEARCH

// Attack / tracking
bool lastSenseRight = true;       // last-known opponent side
unsigned long objectLastSeen = 0; // millis() timestamp

// --- IMPROVED TRACKING STATE ---
// Peak-hold buffers: remember strongest recent reading per side
uint8_t heldLeft = 0, heldRight = 0;
unsigned long heldLeftTime = 0, heldRightTime = 0;

// Confidence score (0–255)
uint8_t trackConfidence = 0;

// Sub-phases within ATTACK
enum AttackPhase { ATK_LOCK, ATK_COAST, ATK_REACQUIRE };
AttackPhase attackPhase = ATK_LOCK;
unsigned long coastStartTime = 0;
unsigned long sweepStartTime = 0;
bool sweepDir = false;
uint8_t sweepCycles = 0; // counts half-periods for widening sweep

// Wraps motors.setSpeeds() — flips both channels if INVERT_MOTORS is true
void drive(int16_t left, int16_t right)
{
  if (INVERT_MOTORS)
    motors.setSpeeds(-left, -right);
  else
    motors.setSpeeds(left, right);
}

//                         SETUP
void setup()
{
  Serial.begin(9600);

  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  display.clear();
  display.print(F("CECE up!"));
  display.gotoXY(0, 1);
  display.print(F(" A|B|C"));

  currentState = STATE_IDLE;
}

//                        MAIN LOOP
void loop()
{
  switch (currentState)
  {
  case STATE_IDLE:
    handleIdle();
    break;
  case STATE_CROSS_RING:
    handleCrossRing();
    break;
  case STATE_SEARCH:
    handleSearch();
    break;
  case STATE_ATTACK:
    handleAttack();
    break;
  }
}

//  EMERGENCY STOP — returns to IDLE
void emergencyStop()
{
  drive(0, 0);
  buzzer.playNote(NOTE_C(3), 300, BUZZER_VOLUME);
  ledYellow(0);
  ledRed(1);
  display.clear();
  display.print(F("STOPPED"));
  display.gotoXY(0, 1);
  display.print(F(" A|B|C"));
  currentState = STATE_IDLE;
}

//  FLOOR CALIBRATION
//  Called when A/C starts a match. Robot must be sitting still on the arena
//  floor (not on the tawara/boundary) when this runs.
void calibrateFloor()
{
  display.clear();
  display.print(F("CAL..."));

  Serial.println(F("\n===== FLOOR CALIBRATION ====="));
  Serial.print(F("Sampling "));
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(F(" times..."));

  uint32_t sums[NUM_LINE_SENSORS] = {0, 0, 0, 0, 0};

  for (uint8_t sample = 0; sample < CALIBRATION_SAMPLES; sample++)
  {
    lineSensors.read(lineSensorValues);
    for (uint8_t index = 0; index < NUM_LINE_SENSORS; index++)
    {
      sums[index] += lineSensorValues[index];
    }
    delay(CALIBRATION_DELAY_MS);
  }

  Serial.print(F("Baseline -> "));
  for (uint8_t index = 0; index < NUM_LINE_SENSORS; index++)
  {
    floorBaseline[index] = (uint16_t)(sums[index] / CALIBRATION_SAMPLES);
    Serial.print(F("S"));
    Serial.print(index);
    Serial.print(F("="));
    Serial.print(floorBaseline[index]);
    if (index < NUM_LINE_SENSORS - 1)
      Serial.print(F("  "));
  }
  Serial.println();
  Serial.print(F("Tolerance: +/-"));
  Serial.println(BOUNDARY_TOLERANCE);

  floorCalibrated = true;

  // Show centre-sensor baseline on LCD as a quick sanity check
  display.clear();
  display.print(F("CAL OK"));
  display.gotoXY(0, 1);
  display.print(F("C="));
  display.print(floorBaseline[2]);
}

//  STATE: IDLE — waiting for button press
//    A = 5-second countdown + CCW scan, then fight
//    B = emergency stop
//    C = 5-second countdown + CW scan, then fight
void handleIdle()
{
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    doCountdownAndScan(false); // scan left (CCW)
    if (currentState != STATE_ATTACK)
    {
      currentState = STATE_CROSS_RING;
      stateStartTime = millis();
    }
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    ledRed(0);
    doCountdownAndScan(true); // scan right (CW)
    if (currentState != STATE_ATTACK)
    {
      currentState = STATE_CROSS_RING;
      stateStartTime = millis();
    }
  }
}

//  HELPER: blocking 5-second countdown + 360° scan
//  Called from handleIdle() AFTER calibrateFloor() completes.
//  scanCW: true = spin right (CW), false = spin left (CCW)
//  If opponent is detected during the scan, transitions directly
//  to ATTACK and returns; otherwise returns to let caller set
//  STATE_CROSS_RING.
void doCountdownAndScan(bool scanCW)
{
  unsigned long countdownStart = millis();
  int lastAnnouncedSecond = 5;

  // Immediate visual/audio feedback on button press.
  display.clear();
  display.gotoXY(3, 0);
  display.print(5);
  display.gotoXY(0, 1);
  display.print(scanCW ? F("CW") : F("CCW"));
  buzzer.playNote(NOTE_C(4), 150, BUZZER_VOLUME);

  // Calibrate after countdown start so timing starts immediately on button press.
  calibrateFloor();

  while (true)
  {
    unsigned long elapsed = millis() - countdownStart;
    if (elapsed >= 5000)
    {
      break;
    }

    int remaining = 5 - (int)(elapsed / 1000);
    if (remaining != lastAnnouncedSecond)
    {
      lastAnnouncedSecond = remaining;

      display.clear();
      display.gotoXY(3, 0);
      display.print(remaining);
      display.gotoXY(0, 1);
      display.print(scanCW ? F("CW") : F("CCW"));

      switch (remaining)
      {
      case 5:
        buzzer.playNote(NOTE_C(4), 150, BUZZER_VOLUME);
        break;
      case 4:
        buzzer.playNote(NOTE_D(4), 150, BUZZER_VOLUME);
        break;
      case 3:
        buzzer.playNote(NOTE_E(4), 150, BUZZER_VOLUME);
        break;
      case 2:
        buzzer.playNote(NOTE_G(4), 150, BUZZER_VOLUME);
        break;
      case 1:
        buzzer.playNote(NOTE_A(4), 150, BUZZER_VOLUME);
        break;
      }
    }
  }

  // GO!
  display.clear();
  display.gotoXY(2, 0);
  display.print(F("GO!"));
  buzzer.playNote(NOTE_C(6), 300, BUZZER_VOLUME);

  // 360° scan (blocking spin)
  display.clear();
  display.print(F("SCANNING"));

  unsigned long scanStart = millis();

  while (true)
  {
    if (scanCW)
      drive(SCAN_SPEED, -SCAN_SPEED);
    else
      drive(-SCAN_SPEED, SCAN_SPEED);

    // Check for opponent while spinning
    proxSensors.read();
    uint8_t lv = proxSensors.countsFrontWithLeftLeds();
    uint8_t rv = proxSensors.countsFrontWithRightLeds();

    if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
    {
      drive(0, 0);
      transitionToAttack(rv >= lv);
      return; // go straight to ATTACK — caller's CROSS_RING set is skipped
    }

    if (millis() - scanStart >= SCAN_TIMEOUT)
    {
      drive(0, 0);
      display.clear();
      display.print(F("CROSS"));
      return;
    }
  }
}

//  STATE: CROSS_RING — drive to the far side of the 30″ dohyo
void handleCrossRing()
{
  // B button = emergency stop
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  // Always scan for opponent while driving
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    drive(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  if (checkBoundary())
  {
    bounceOffBoundary();
    transitionToSearch();
    return;
  }

  // Drive forward at search speed
  display.gotoXY(0, 0);
  display.print(F("CROSSING"));
  drive(SEARCH_SPEED, SEARCH_SPEED);

  //  Timeout → should have crossed by now, begin search
  if (millis() - stateStartTime > CROSS_RING_TIME)
  {
    transitionToSearch();
  }
}

// ================================================================
//  STATE: SEARCH — aggressive drive-to-boundary sweep pattern
//
//  Strategy: drive FORWARD at high speed until a boundary is hit,
//  then bounce off at a varied angle and charge across again.
//  Each bounce picks a different angle from a small table so
//  successive passes cut different chords across the circular arena,
//  maximising ground coverage over time.  The robot continuously
//  checks proximity sensors while driving so it can snap to ATTACK
//  the instant anything is detected.
//
//  Why this beats spinning-in-place:
//  - Constant forward motion covers the full ring diameter per pass.
//  - Varied bounce angles prevent repetitive back-and-forth ruts.
//  - High speed means more passes per minute → more chance of
//    physical contact even when IR can't see a black opponent.
// ================================================================
void handleSearch()
{
  // B button = emergency stop
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  // Continuously scan for opponent while moving
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    drive(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  // --- Boundary hit → bounce at a varied angle and keep going ---
  if (checkBoundary())
  {
    searchBounceCount++;

    // Pick which side to turn away from
    int8_t side = getBoundarySide();
    bool turnRight = (side <= 0); // left/centre hit → turn right

    // Calculate the turn duration from the angle table.
    // The table cycles so the robot never repeats the same
    // angle more than once every SEARCH_ANGLE_TABLE_SIZE bounces.
    int16_t turnMs = SEARCH_BOUNCE_TURN_BASE_MS
                     + (int16_t)(searchBounceIndex % SEARCH_ANGLE_TABLE_SIZE)
                       * SEARCH_BOUNCE_TURN_STEP_MS;
    int16_t bias = SEARCH_ANGLE_BIAS[searchBounceIndex % SEARCH_ANGLE_TABLE_SIZE];
    // Apply bias: positive adds time if turning right, subtracts if left
    if (turnRight)
      turnMs += bias;
    else
      turnMs -= bias;
    // Clamp to a sane range
    if (turnMs < 150) turnMs = 150;
    if (turnMs > 600) turnMs = 600;

    // Advance the angle index for the next bounce
    searchBounceIndex++;
    if (searchBounceIndex >= SEARCH_ANGLE_TABLE_SIZE)
      searchBounceIndex = 0;

    // --- Bounce manoeuvre (blocking, like original) ---
    ledRed(1);
    buzzer.playNote(NOTE_A(5), 150, BUZZER_VOLUME);

    // Reverse briefly
    drive(-EVADE_SPEED, -EVADE_SPEED);
    delay(350);

    // Turn away at the computed angle
    if (turnRight)
      drive(TURN_SPEED, -TURN_SPEED);
    else
      drive(-TURN_SPEED, TURN_SPEED);
    delay(turnMs);

    drive(0, 0);
    ledRed(0);

    // Alternate the default turn direction every few bounces
    // to avoid a long-term rotational bias
    if (searchBounceCount % 3 == 0)
      searchTurnDir = !searchTurnDir;

    display.clear();
    display.print(F("SEARCH"));
    display.gotoXY(0, 1);
    display.print(F("B"));
    display.print(searchBounceCount);
    return;
  }

  // --- Default: charge straight ahead at full search speed ---
  drive(SEARCH_SPEED, SEARCH_SPEED);

  display.gotoXY(0, 1);
  display.print(F("DRIVE   "));
}

// ================================================================
//  STATE: ATTACK — 3-phase tracking optimised for weak IR targets
//
//  Phase 1  ATK_LOCK:  Target visible → proportional steering at
//           full attack speed.  Uses peak-held sensor values for
//           lock decisions and raw values for responsive steering.
//
//  Phase 2  ATK_COAST: Signal just dropped out → keep driving
//           forward with a slight bias toward the last-known side.
//           Bridges 1–2 cycle sensor gaps without losing ground.
//
//  Phase 3  ATK_REACQUIRE: Still no signal → oscillating sweep
//           with forward motion.  Sweep widens progressively.
//           Falls back to SEARCH after REACQUIRE_TIMEOUT.
//
//  Any detection in any phase snaps back to ATK_LOCK immediately.
// ================================================================
void handleAttack()
{
  // B button = emergency stop
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  // ── Read sensors ──
  proxSensors.read();
  uint8_t rawL = proxSensors.countsFrontWithLeftLeds();
  uint8_t rawR = proxSensors.countsFrontWithRightLeds();

  // ── Peak-hold: remember strongest reading per side for SENSOR_HOLD_MS ──
  // This bridges brief 1–2 cycle dropouts that are common with
  // black opponents reflecting very little IR light.
  unsigned long now = millis();
  if (rawL > 0) { heldLeft  = rawL;  heldLeftTime  = now; }
  else if (now - heldLeftTime  > SENSOR_HOLD_MS) { heldLeft  = 0; }
  if (rawR > 0) { heldRight = rawR;  heldRightTime = now; }
  else if (now - heldRightTime > SENSOR_HOLD_MS) { heldRight = 0; }

  // ── Classify signal strength ──
  bool strongNow = (rawL >= PROX_ATTACK_THRESHOLD) || (rawR >= PROX_ATTACK_THRESHOLD);
  bool weakNow   = (rawL >= PROX_LOCK_THRESHOLD)   || (rawR >= PROX_LOCK_THRESHOLD);
  bool heldSeen  = (heldLeft >= PROX_LOCK_THRESHOLD) || (heldRight >= PROX_LOCK_THRESHOLD);

  // ── Update tracking confidence ──
  if (strongNow)
    trackConfidence = (trackConfidence <= 255 - CONFIDENCE_GAIN_STRONG)
                        ? trackConfidence + CONFIDENCE_GAIN_STRONG : 255;
  else if (weakNow || heldSeen)
    trackConfidence = (trackConfidence <= 255 - CONFIDENCE_GAIN_WEAK)
                        ? trackConfidence + CONFIDENCE_GAIN_WEAK : 255;
  else
    trackConfidence = (trackConfidence >= CONFIDENCE_DECAY)
                        ? trackConfidence - CONFIDENCE_DECAY : 0;

  // ── Decide if we have a usable lock ──
  // "locked" uses raw OR held values, gated by confidence, so a single
  // noisy zero-reading cycle does not instantly break pursuit.
  bool locked = (strongNow || weakNow || heldSeen)
                && (trackConfidence >= CONFIDENCE_LOCK_MIN);

  // ── Boundary check ──
  if (checkBoundary())
  {
    bounceOffBoundary();

    // Re-read after the bounce to see if target is still ahead
    proxSensors.read();
    rawL = proxSensors.countsFrontWithLeftLeds();
    rawR = proxSensors.countsFrontWithRightLeds();
    if (rawL >= PROX_LOCK_THRESHOLD || rawR >= PROX_LOCK_THRESHOLD)
    {
      // Still see something — stay aggressive, enter reacquire sweep
      lastSenseRight = (rawR >= rawL);
      objectLastSeen = now;
      trackConfidence = max(trackConfidence, (uint8_t)80);
      attackPhase = ATK_REACQUIRE;
      sweepStartTime = now;
      sweepDir = lastSenseRight;
      sweepCycles = 0;
      display.clear();
      display.print(F("ATTACK!"));
    }
    else
    {
      transitionToSearch();
    }
    return;
  }

  // ================================================================
  //  Phase routing — any detection snaps to ATK_LOCK
  // ================================================================

  if (locked)
  {
    // ============= ATK_LOCK: active proportional steering =============
    attackPhase = ATK_LOCK;
    objectLastSeen = now;
    ledYellow(1);

    // Use RAW values for steering direction (responsive), but use held
    // values as a fallback when raw drops to zero momentarily.
    uint8_t steerL = (rawL > 0) ? rawL : heldLeft;
    uint8_t steerR = (rawR > 0) ? rawR : heldRight;
    int16_t err = (int16_t)steerR - (int16_t)steerL;

    if (err > 0)
    {
      // Target biased right — curve right
      int16_t adj = map(err, 1, 6, STEER_ADJ_MIN, STEER_ADJ_MAX);
      drive(ATTACK_SPEED, ATTACK_SPEED - adj);
      lastSenseRight = true;
    }
    else if (err < 0)
    {
      // Target biased left — curve left
      int16_t adj = map(-err, 1, 6, STEER_ADJ_MIN, STEER_ADJ_MAX);
      drive(ATTACK_SPEED - adj, ATTACK_SPEED);
      lastSenseRight = false;
    }
    else
    {
      // Dead-centre — full charge
      drive(ATTACK_SPEED, ATTACK_SPEED);
    }

    display.gotoXY(0, 1);
    display.print(F("LCK "));
    display.print(rawL);
    display.print(' ');
    display.print(rawR);
    display.print(' ');
    display.print(trackConfidence);
  }
  else if (trackConfidence >= CONFIDENCE_COAST_MIN
           && attackPhase != ATK_REACQUIRE)
  {
    // ========= ATK_COAST: brief forward drive after dropout =========
    // The opponent may be very close (IR saturated / out of range) or
    // the sensor had a momentary glitch.  Keep charging straight with
    // a slight bias toward the last-known side.
    if (attackPhase != ATK_COAST)
    {
      attackPhase = ATK_COAST;
      coastStartTime = now;
    }

    ledYellow(1); // LED stays on — we haven't given up yet

    if (lastSenseRight)
      drive(ATTACK_SPEED, ATTACK_SPEED - 50);
    else
      drive(ATTACK_SPEED - 50, ATTACK_SPEED);

    // Transition to reacquire sweep after coast window
    if (now - coastStartTime > COAST_DURATION_MS)
    {
      attackPhase = ATK_REACQUIRE;
      sweepStartTime = now;
      sweepDir = lastSenseRight;
      sweepCycles = 0;
    }

    display.gotoXY(0, 1);
    display.print(F("COAST   "));
  }
  else
  {
    // ====== ATK_REACQUIRE: oscillating sweep with forward motion ======
    // Sweeps left-right around the last known direction,
    // progressively widening each half-period to cover more area.
    // Forward component keeps the robot advancing, not just spinning.
    if (attackPhase != ATK_REACQUIRE)
    {
      attackPhase = ATK_REACQUIRE;
      sweepStartTime = now;
      sweepDir = lastSenseRight;
      sweepCycles = 0;
    }

    ledYellow(0);

    unsigned long sweepElapsed = now - sweepStartTime;
    uint8_t halfPeriods = (uint8_t)(sweepElapsed / REACQUIRE_SWEEP_HALF_MS);

    // Flip direction each half-period
    bool curDir = sweepDir;
    if (halfPeriods % 2 == 1) curDir = !curDir;

    // Widen the sweep progressively (capped)
    int16_t width = SWEEP_INITIAL_WIDTH + ((int16_t)halfPeriods * SWEEP_WIDTH_GROW);
    if (width > SWEEP_WIDTH_MAX) width = SWEEP_WIDTH_MAX;

    // Forward-biased sweep: not a pure spin, robot covers ground
    if (curDir) // sweep right
      drive(SWEEP_BASE_SPEED + width, SWEEP_BASE_SPEED - width);
    else        // sweep left
      drive(SWEEP_BASE_SPEED - width, SWEEP_BASE_SPEED + width);

    // Give up after REACQUIRE_TIMEOUT → transition to SEARCH
    if (now - objectLastSeen > REACQUIRE_TIMEOUT)
    {
      transitionToSearch();
    }

    display.gotoXY(0, 1);
    display.print(F("REACQ   "));
  }
}

//  HELPER: transition into ATTACK state
//  Initialises all tracking state so ATTACK begins with a warm lock.
void transitionToAttack(bool opponentOnRight)
{
  lastSenseRight = opponentOnRight;
  objectLastSeen = millis();

  // Pre-load confidence so the first few noisy cycles don't
  // immediately drop us back to SEARCH.
  trackConfidence = 90;
  attackPhase = ATK_LOCK;

  // Seed peak-hold buffers with the triggering reading
  proxSensors.read();
  heldLeft  = proxSensors.countsFrontWithLeftLeds();
  heldRight = proxSensors.countsFrontWithRightLeds();
  heldLeftTime  = millis();
  heldRightTime = millis();

  currentState = STATE_ATTACK;
  stateStartTime = millis();

  ledYellow(1);
  buzzer.playNote(NOTE_C(6), 300, BUZZER_VOLUME);
  display.clear();
  display.print(F("ATTACK!"));
}

//  HELPER: transition into SEARCH state
void transitionToSearch()
{
  drive(0, 0);

  searchTurnDir = !searchTurnDir; // vary direction each time
  searchLegStart = millis();
  // Don't reset searchBounceIndex here — let it continue cycling
  // through the angle table across multiple ATTACK→SEARCH→ATTACK
  // transitions so the coverage pattern stays varied.
  searchBounceCount = 0;

  currentState = STATE_SEARCH;
  stateStartTime = millis();

  ledYellow(0);
  buzzer.playNote(NOTE_C(3), 400, BUZZER_VOLUME);
  display.clear();
  display.print(F("SEARCH"));
}

//  BOUNDARY DETECTION

// Debug counter to throttle serial output (prints every N calls)
unsigned long lastBoundaryDebug = 0;
const unsigned long BOUNDARY_DEBUG_INTERVAL = 200; // ms between debug prints

bool checkBoundary()
{
  if (!floorCalibrated)
    return false;

  lineSensors.read(lineSensorValues);

  bool shouldPrint = (millis() - lastBoundaryDebug >= BOUNDARY_DEBUG_INTERVAL);
  bool boundaryHit = false;
  uint8_t triggerSensor = 255;
  uint16_t triggerDev = 0;

  // Check all sensors first
  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    uint16_t baseline = floorBaseline[i];
    uint16_t reading = lineSensorValues[i];
    uint16_t deviation = (reading > baseline)
                             ? (reading - baseline)
                             : (baseline - reading);
    if (deviation > BOUNDARY_TOLERANCE && !boundaryHit)
    {
      boundaryHit = true;
      triggerSensor = i;
      triggerDev = deviation;
    }
  }

  // Print debug info periodically OR when boundary hit
  if (shouldPrint || boundaryHit)
  {
    lastBoundaryDebug = millis();

    Serial.print(F("LINE: "));
    for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
      uint16_t baseline = floorBaseline[i];
      uint16_t reading = lineSensorValues[i];
      int16_t diff = (int16_t)reading - (int16_t)baseline;

      Serial.print(F("S"));
      Serial.print(i);
      Serial.print(F(":"));
      Serial.print(reading);
      Serial.print(F("("));
      if (diff >= 0)
        Serial.print(F("+"));
      Serial.print(diff);
      Serial.print(F(") "));
    }

    if (boundaryHit)
    {
      Serial.print(F(" >>> BOUNDARY! S"));
      Serial.print(triggerSensor);
      Serial.print(F(" dev="));
      Serial.print(triggerDev);
      Serial.print(F(" > tol="));
      Serial.print(BOUNDARY_TOLERANCE);
    }
    Serial.println();
  }

  return boundaryHit;
}

// Returns which side of the robot hit the boundary.
//   -1 = left,  0 = centre / both,  1 = right
int8_t getBoundarySide()
{
  bool leftHit = false;
  bool rightHit = false;

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    uint16_t baseline = floorBaseline[i];
    uint16_t reading = lineSensorValues[i];
    uint16_t deviation = (reading > baseline)
                             ? (reading - baseline)
                             : (baseline - reading);
    if (deviation > BOUNDARY_TOLERANCE)
    {
      if (i <= 1)
        leftHit = true; // sensors 0,1
      if (i >= 3)
        rightHit = true; // sensors 3,4
      if (i == 2)
      {
        leftHit = true;
        rightHit = true;
      }
    }
  }

  if (leftHit && rightHit)
    return 0;
  if (leftHit)
    return -1;
  if (rightHit)
    return 1;
  return 0;
}

// Back up and turn away from the boundary.
void bounceOffBoundary()
{
  int8_t side = getBoundarySide();

  // Visual + audible alert
  ledRed(1);
  buzzer.playNote(NOTE_A(5), 200, BUZZER_VOLUME);
  display.clear();
  display.print(F("BOUNDARY"));

  // Reverse briefly
  drive(-EVADE_SPEED, -EVADE_SPEED);
  delay(400);

  // Turn away from the boundary edge
  if (side <= 0) // left or centre → turn right
    drive(TURN_SPEED, -TURN_SPEED);
  else // right → turn left
    drive(-TURN_SPEED, TURN_SPEED);

  delay(250 + random(200)); // randomised to vary the search path

  drive(0, 0);
  ledRed(0);
}