/*  ============================================================
 *  IEEE @ UMBC — Zumo 32U4 Sumo Robot  (30-inch Dohyo)
 *  ============================================================
 *
 *  CONTROLS
 *    Button A — Start 5-second countdown, then fight
 *    Button B — Emergency stop (kills motors, returns to idle)
 *    Button C — Enter TEST MODE from idle/stopped; cycles tests
 *
 *  STATE MACHINE
 *    IDLE ──▶ COUNTDOWN ──▶ SCAN_360 ──▶ ATTACK ◀──▶ EVADE
 *                                   └──▶ SEARCH ──▶ ATTACK
 *    Any state ──(B)──▶ STOPPED ──(A)──▶ COUNTDOWN
 *    IDLE/STOPPED ──(C)──▶ TEST (C cycles sub-modes, B exits)
 *
 *  OFFENSIVE
 *    • Fast 360° spin to locate the opponent via proximity sensors
 *    • If found, charge at full ATTACK_SPEED
 *    • If not found, drive across the ring and begin a "fuzzy
 *      search" — drive forward, bounce off boundary lines in
 *      randomised directions, continuously scanning for opponent
 *
 *  DEFENSIVE
 *    • Encoder-based push detection: if commanded-forward but
 *      encoder counts are too low, the robot is being stalled
 *    • On push detect → burst reverse, quick turn, counter-charge
 *    • Search phase uses slow speed; attack uses full speed
 *  ============================================================ */

#include <Wire.h>
#include <Zumo32U4.h>

// ======================== HARDWARE OBJECTS ========================
Zumo32U4LCD       display;
Zumo32U4Motors    motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors      lineSensors;
Zumo32U4Buzzer    buzzer;
Zumo32U4Encoders  encoders;
Zumo32U4IMU       imu;
Zumo32U4ButtonA   buttonA;
Zumo32U4ButtonB   buttonB;
Zumo32U4ButtonC   buttonC;

// TurnSensor.h must be included AFTER declaring display, imu, buttonA
#include "TurnSensor.h"

// =================================================================
//  ██  LINE-SENSOR BOUNDARY CONFIGURATION  ██
// =================================================================
//  Adjust these values to match YOUR arena + boundary colours.
//
//  The five line sensors return 0 – 2000:
//      LOW  values (  0 – ~500 )  = high reflectance (white/bright)
//      HIGH values (~1500 – 2000)  = low  reflectance (dark)
//
//  LINE_THRESHOLD — the cut-off reading that separates
//                   "arena floor" from "boundary".
//
//  BOUNDARY_IS_WHITE
//      true  → white/bright tawara on a dark arena floor
//              (boundary detected when reading < LINE_THRESHOLD)
//      false → dark tawara on a light arena floor
//              (boundary detected when reading > LINE_THRESHOLD)
//
//  *** Change these two values after testing on your surface ***
// =================================================================
const uint16_t LINE_THRESHOLD    = 500;
const bool     BOUNDARY_IS_WHITE = true;
// =================================================================

#define NUM_LINE_SENSORS 5
uint16_t lineSensorValues[NUM_LINE_SENSORS];

// ===================== PROXIMITY THRESHOLDS ======================
// Sensor returns 0–6.  Lower = more sensitive (detects farther away).
const uint8_t PROX_SEARCH_THRESHOLD = 3;   // used during scan / search / cross-ring
const uint8_t PROX_ATTACK_THRESHOLD = 4;   // used during attack (closer = real target)

// ===================== SPEED SETTINGS ============================
const int16_t SEARCH_SPEED   = 150;  // slow — conserve position
const int16_t ATTACK_SPEED   = 400;  // max — full ram
const int16_t SCAN_SPEED     = 250;  // 360° scan
const int16_t EVADE_SPEED    = 400;  // escape burst
const int16_t TURN_SPEED     = 400;  // general turning

// ============= ENCODER / PUSH-DETECTION SETTINGS =================
//  While commanding forward at ATTACK_SPEED, the encoders
//  normally return a large positive count every PUSH_CHECK_INTERVAL.
//  If the counts fall below PUSH_DETECT_THRESHOLD the robot
//  is stalled or being pushed backward.
const int16_t       PUSH_DETECT_THRESHOLD = 10;   // encoder ticks
const unsigned long  PUSH_CHECK_INTERVAL  = 100;  // ms
const unsigned long  EVADE_DURATION       = 600;   // ms total evade

// ============= SCAN / SEARCH TIMING ==============================
const unsigned long  SCAN_TIMEOUT         = 2500;  // ms for 360° spin
const unsigned long  CROSS_RING_TIME      = 1200;  // ms driving to far side
const unsigned long  SEARCH_DRIVE_TIME    = 800;   // ms forward per leg
const unsigned long  SEARCH_TURN_TIME     = 400;   // ms turning per leg
const unsigned long  REACQUIRE_TIMEOUT    = 400;   // ms to reacquire target

// ========================= STATE MACHINE =========================
enum RobotState {
  STATE_IDLE,
  STATE_COUNTDOWN,
  STATE_SCAN_360,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_EVADE,
  STATE_STOPPED,
  STATE_TEST
};

// ===================== TEST MODE SUB-MODES =======================
enum TestMode {
  TEST_LINE_SENSORS,   // raw line-sensor values → LCD + Serial
  TEST_PROXIMITY,      // proximity sensor readings → LCD + Serial
  TEST_ENCODERS,       // encoder counts (push-detection check)
  TEST_MOTORS,         // spin motors at low speed
  TEST_GYRO,           // live gyro angle
  TEST_PUSH_DETECT,    // drive forward slowly, beep + evade on push
  NUM_TEST_MODES       // sentinel — always last
};

TestMode       currentTestMode  = TEST_LINE_SENSORS;
unsigned long  testPrintTimer   = 0;
const unsigned long TEST_PRINT_INTERVAL = 150;  // ms between serial prints

// Push-detect test mode helpers
bool           testPushEvading   = false;
unsigned long  testEvadeStart    = 0;

RobotState    currentState    = STATE_IDLE;
unsigned long stateStartTime  = 0;

// Countdown
int lastPlayedSecond = -1;

// Push detection
unsigned long lastPushCheck = 0;

// Search pattern
bool          searchTurnDir   = false;  // false = right, true = left
unsigned long searchLegStart  = 0;
bool          searchDriving   = true;

// Attack / tracking
bool          lastSenseRight  = true;   // last-known opponent side
unsigned long objectLastSeen  = 0;      // millis() timestamp

// =================================================================
//                         SETUP
// =================================================================
void setup()
{
  Serial.begin(9600);          // ← for test-mode calibration output

  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  // Calibrate the gyro — robot must be still on the table
  turnSensorSetup();

  display.clear();
  display.print(F("SumoBot"));
  display.gotoXY(0, 1);
  display.print(F("A=Go C=Test"));

  currentState = STATE_IDLE;
}

// =================================================================
//                        MAIN LOOP
// =================================================================
void loop()
{
  // ── Button B = emergency stop from ANY state ──
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  switch (currentState)
  {
    case STATE_IDLE:       handleIdle();       break;
    case STATE_COUNTDOWN:  handleCountdown();  break;
    case STATE_SCAN_360:   handleScan360();    break;
    case STATE_CROSS_RING: handleCrossRing();  break;
    case STATE_SEARCH:     handleSearch();     break;
    case STATE_ATTACK:     handleAttack();     break;
    case STATE_EVADE:      handleEvade();      break;
    case STATE_STOPPED:    handleStopped();    break;
    case STATE_TEST:       handleTestMode();   break;
  }
}

// =================================================================
//  EMERGENCY STOP
// =================================================================
void emergencyStop()
{
  motors.setSpeeds(0, 0);
  buzzer.playNote(NOTE_C(3), 300, 15);
  ledYellow(0);
  ledRed(1);
  display.clear();
  display.print(F("STOPPED"));
  display.gotoXY(0, 1);
  display.print(F("A=Go C=Tst"));
  currentState = STATE_STOPPED;
}

// =================================================================
//  STATE: IDLE — waiting for the user to press A
// =================================================================
void handleIdle()
{
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    lastPlayedSecond = -1;
    currentState  = STATE_COUNTDOWN;
    stateStartTime = millis();
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    enterTestMode();
  }
}

// =================================================================
//  STATE: STOPPED — motors off, press A to restart
// =================================================================
void handleStopped()
{
  motors.setSpeeds(0, 0);
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    lastPlayedSecond = -1;
    currentState  = STATE_COUNTDOWN;
    stateStartTime = millis();
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    enterTestMode();
  }
}

// =================================================================
//  STATE: COUNTDOWN — 5-4-3-2-1-GO! with ascending buzzer tones
// =================================================================
void handleCountdown()
{
  unsigned long elapsed = millis() - stateStartTime;
  int remaining = 5 - (int)(elapsed / 1000);

  if (remaining > 0 && remaining <= 5)
  {
    // ── Only update LCD + buzzer when the number changes ──
    if (remaining != lastPlayedSecond)
    {
      lastPlayedSecond = remaining;

      display.clear();
      display.gotoXY(3, 0);
      display.print(remaining);

      // Ascending tone: 5 = lowest, 1 = highest before "GO"
      switch (remaining)
      {
        case 5: buzzer.playNote(NOTE_C(4), 150, 12); break;  // 262 Hz
        case 4: buzzer.playNote(NOTE_D(4), 150, 12); break;  // 294 Hz
        case 3: buzzer.playNote(NOTE_E(4), 150, 12); break;  // 330 Hz
        case 2: buzzer.playNote(NOTE_G(4), 150, 12); break;  // 392 Hz
        case 1: buzzer.playNote(NOTE_A(4), 150, 12); break;  // 440 Hz
      }
    }
  }
  else if (remaining <= 0)
  {
    // ── GO! ──
    display.clear();
    display.gotoXY(2, 0);
    display.print(F("GO!"));
    buzzer.playNote(NOTE_C(6), 300, 15);   // high-pitched GO tone
    delay(300);

    // Prepare for 360° scan
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    turnSensorReset();

    currentState  = STATE_SCAN_360;
    stateStartTime = millis();

    display.clear();
    display.print(F("SCANNING"));
  }
}

// =================================================================
//  STATE: SCAN_360 — fast clockwise spin looking for opponent
// =================================================================
void handleScan360()
{
  turnSensorUpdate();

  // ── Spin clockwise ──
  motors.setSpeeds(SCAN_SPEED, -SCAN_SPEED);

  // ── Check proximity sensors while spinning ──
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    // Opponent found — immediately attack
    motors.setSpeeds(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  // ── Compute clockwise degrees rotated (0→360) ──
  uint32_t rotated = -turnAngle;   // CW rotation = positive
  int32_t cwDegrees = ((int32_t)(rotated >> 16) * 360L) >> 16;

  display.gotoXY(0, 1);
  display.print(cwDegrees);
  display.print(F(" deg  "));

  // ── Full 360° clockwise rotation complete ──
  // Guard: wait at least 1s before checking angle to prevent
  // gyro noise from triggering an instant false-positive.
  if (millis() - stateStartTime > 1000 && cwDegrees >= 350)
  {
    motors.setSpeeds(0, 0);
    currentState  = STATE_CROSS_RING;
    stateStartTime = millis();

    display.clear();
    display.print(F("CROSS"));
  }
}

// =================================================================
//  STATE: CROSS_RING — drive to the far side of the 30″ dohyo
// =================================================================
void handleCrossRing()
{
  // ── Always scan for opponent while driving ──
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  // ── Hit boundary → stop crossing, begin fuzzy search ──
  // COMMENTED OUT FOR TESTING — boundary detection disabled
  // if (checkBoundary())
  // {
  //   bounceOffBoundary();
  //   transitionToSearch();
  //   return;
  // }

  // ── Drive forward at search speed ──
  motors.setSpeeds(SEARCH_SPEED, SEARCH_SPEED);

  // ── Timeout → should have crossed by now, begin search ──
  if (millis() - stateStartTime > CROSS_RING_TIME)
  {
    transitionToSearch();
  }
}

// =================================================================
//  STATE: SEARCH — fuzzy bouncing search pattern
// =================================================================
void handleSearch()
{
  // ── Continuously scan for opponent ──
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  // ── Bounce off boundary ──
  // COMMENTED OUT FOR TESTING — boundary detection disabled
  // if (checkBoundary())
  // {
  //   bounceOffBoundary();
  //   searchLegStart = millis();
  //   searchDriving  = true;
  //   return;
  // }

  // ── Alternating drive / turn legs ──
  unsigned long legElapsed = millis() - searchLegStart;

  if (searchDriving)
  {
    motors.setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
    if (legElapsed > SEARCH_DRIVE_TIME)
    {
      searchDriving = false;
      searchLegStart = millis();
    }
  }
  else
  {
    // Turn in the current search direction
    if (searchTurnDir)
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);   // left
    else
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);   // right

    if (legElapsed > SEARCH_TURN_TIME)
    {
      searchDriving  = true;
      searchTurnDir  = !searchTurnDir;              // alternate next time
      searchLegStart = millis();
    }
  }

  // ── LCD ──
  display.gotoXY(0, 1);
  display.print(searchDriving ? F(">>FWD ") : F(">>TRN "));
}

// =================================================================
//  STATE: ATTACK — charge the opponent!
// =================================================================
void handleAttack()
{
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();
  bool seen = (lv >= PROX_ATTACK_THRESHOLD) || (rv >= PROX_ATTACK_THRESHOLD);

  // ── Boundary check — do not drive off the ring ──
  // COMMENTED OUT FOR TESTING — boundary detection disabled
  // if (checkBoundary())
  // {
  //   bounceOffBoundary();
  //
  //   // If we still see the opponent, keep attacking; otherwise search
  //   proxSensors.read();
  //   lv = proxSensors.countsFrontWithLeftLeds();
  //   rv = proxSensors.countsFrontWithRightLeds();
  //   if (lv >= PROX_ATTACK_THRESHOLD || rv >= PROX_ATTACK_THRESHOLD)
  //   {
  //     lastSenseRight = (rv >= lv);
  //     objectLastSeen = millis();
  //   }
  //   else
  //   {
  //     transitionToSearch();
  //   }
  //   return;
  // }

  // ── Push detection (always active — testing) ──
  if (checkBeingPushed())
  {
    transitionToEvade();
    return;
  }

  if (seen)
  {
    ledYellow(1);
    objectLastSeen = millis();

    // Proportional steering toward the stronger signal
    // Aggressive adj (up to 300) so the robot curves hard toward closer targets
    if (rv > lv)
    {
      // Object biased right — curve right
      int16_t adj = map(rv - lv, 0, 6, 0, 300);
      motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED - adj);
      lastSenseRight = true;
    }
    else if (lv > rv)
    {
      // Object biased left — curve left
      int16_t adj = map(lv - rv, 0, 6, 0, 300);
      motors.setSpeeds(ATTACK_SPEED - adj, ATTACK_SPEED);
      lastSenseRight = false;
    }
    else
    {
      // Dead-centre — full charge!
      motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED);
    }

    display.gotoXY(0, 1);
    display.print(F("ATK "));
    display.print(lv);
    display.print(' ');
    display.print(rv);
  }
  else
  {
    // ── Lost sight — turn toward last-known direction to reacquire ──
    ledYellow(0);

    if (lastSenseRight)
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);   // turn right
    else
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);   // turn left

    // Give up after REACQUIRE_TIMEOUT and switch to search
    if (millis() - objectLastSeen > REACQUIRE_TIMEOUT)
    {
      transitionToSearch();
    }
  }
}

// =================================================================
//  STATE: EVADE — we are being pushed; escape and counter-attack
// =================================================================
void handleEvade()
{
  unsigned long elapsed = millis() - stateStartTime;

  // ── While evading, respect the boundary ──
  // COMMENTED OUT FOR TESTING — boundary detection disabled
  // if (checkBoundary())
  // {
  //   bounceOffBoundary();
  //   // After bouncing, try to reacquire
  //   afterEvade();
  //   return;
  // }

  if (elapsed < EVADE_DURATION / 3)
  {
    // Phase 1: BURST REVERSE to break contact
    motors.setSpeeds(-EVADE_SPEED, -EVADE_SPEED);
    display.gotoXY(0, 1);
    display.print(F("<<REV "));
  }
  else if (elapsed < EVADE_DURATION * 2 / 3)
  {
    // Phase 2: Quick spin to get a new attack angle
    motors.setSpeeds(EVADE_SPEED, -EVADE_SPEED);
    display.gotoXY(0, 1);
    display.print(F(">>SPIN"));
  }
  else
  {
    // Phase 3: Evade complete — counter-charge or search
    afterEvade();
  }
}

// =================================================================
//  HELPER: transition into ATTACK state
// =================================================================
void transitionToAttack(bool opponentOnRight)
{
  lastSenseRight = opponentOnRight;
  objectLastSeen = millis();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  currentState  = STATE_ATTACK;
  stateStartTime = millis();

  ledYellow(1);
  buzzer.playNote(NOTE_C(6), 300, 15);   // high-pitch beep — target found
  display.clear();
  display.print(F("ATTACK!"));
}

// =================================================================
//  HELPER: transition into SEARCH state
// =================================================================
void transitionToSearch()
{
  motors.setSpeeds(0, 0);

  searchDriving  = true;
  searchTurnDir  = !searchTurnDir;  // vary direction each time
  searchLegStart = millis();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  currentState  = STATE_SEARCH;
  stateStartTime = millis();

  ledYellow(0);
  buzzer.playNote(NOTE_C(3), 400, 15);   // low-pitch beep — target lost
  display.clear();
  display.print(F("SEARCH"));
}

// =================================================================
//  HELPER: transition into EVADE state
// =================================================================
void transitionToEvade()
{
  motors.setSpeeds(0, 0);

  currentState  = STATE_EVADE;
  stateStartTime = millis();

  ledRed(1);
  display.clear();
  display.print(F("EVADE!"));
}

// =================================================================
//  HELPER: after evade — decide next state
// =================================================================
void afterEvade()
{
  ledRed(0);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
    transitionToAttack(rv >= lv);
  else
    transitionToSearch();
}

// =================================================================
//  BOUNDARY DETECTION
// =================================================================

// Returns true if ANY line sensor detects the boundary.
bool checkBoundary()
{
  lineSensors.read(lineSensorValues);

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    bool triggered = BOUNDARY_IS_WHITE
                       ? (lineSensorValues[i] < LINE_THRESHOLD)
                       : (lineSensorValues[i] > LINE_THRESHOLD);
    if (triggered) return true;
  }
  return false;
}

// Returns which side of the robot hit the boundary.
//   -1 = left,  0 = centre / both,  1 = right
int8_t getBoundarySide()
{
  // lineSensorValues[] already populated by checkBoundary()
  bool leftHit  = false;
  bool rightHit = false;

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    bool triggered = BOUNDARY_IS_WHITE
                       ? (lineSensorValues[i] < LINE_THRESHOLD)
                       : (lineSensorValues[i] > LINE_THRESHOLD);
    if (triggered)
    {
      if (i <= 1) leftHit  = true;   // sensors 0,1
      if (i >= 3) rightHit = true;   // sensors 3,4
      if (i == 2) { leftHit = true; rightHit = true; }
    }
  }

  if (leftHit && rightHit) return 0;
  if (leftHit)             return -1;
  if (rightHit)            return  1;
  return 0;
}

// Back up and turn away from the boundary.
void bounceOffBoundary()
{
  int8_t side = getBoundarySide();

  // Reverse briefly
  motors.setSpeeds(-SEARCH_SPEED, -SEARCH_SPEED);
  delay(200);

  // Turn away from the boundary edge
  if (side <= 0)   // left or centre → turn right
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  else             // right → turn left
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);

  delay(250 + random(200));   // randomised to vary the search path

  motors.setSpeeds(0, 0);
}

// =================================================================
//  PUSH DETECTION  (encoder-based)
// =================================================================
//  Called during ATTACK state.  If the robot is commanding full
//  forward speed but the wheels barely move (stalled / pushed),
//  returns true.
bool checkBeingPushed()
{
  if (millis() - lastPushCheck < PUSH_CHECK_INTERVAL) return false;

  int16_t leftCounts  = encoders.getCountsAndResetLeft();
  int16_t rightCounts = encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  // Both wheels barely turning despite full-speed command → pushed
  return (leftCounts < PUSH_DETECT_THRESHOLD &&
          rightCounts < PUSH_DETECT_THRESHOLD);
}

// =================================================================
//  TEST MODE  —  C button cycles sub-modes, B exits
// =================================================================
//  Sub-modes:
//    1. Line Sensors  — raw readings on LCD + Serial (for colour calibration)
//    2. Proximity     — front proximity L/R values
//    3. Encoders      — live encoder counts + push-detect flag
//    4. Motors        — spins wheels slowly so you can verify direction
//    5. Gyro          — live angle in degrees
// =================================================================

void enterTestMode()
{
  motors.setSpeeds(0, 0);
  currentTestMode = TEST_LINE_SENSORS;
  currentState    = STATE_TEST;
  testPrintTimer  = 0;

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();

  ledRed(0);
  ledYellow(0);

  showTestModeHeader();

  Serial.println(F("\n===== ENTERING TEST MODE ====="));
  Serial.println(F("C = next test | B = exit\n"));
}

// Print the name of the current test on the LCD.
void showTestModeHeader()
{
  display.clear();
  display.gotoXY(0, 0);
  switch (currentTestMode)
  {
    case TEST_LINE_SENSORS: display.print(F("T:LineSns")); break;
    case TEST_PROXIMITY:    display.print(F("T:Prox"   )); break;
    case TEST_ENCODERS:     display.print(F("T:Encoder")); break;
    case TEST_MOTORS:       display.print(F("T:Motors" )); break;
    case TEST_GYRO:         display.print(F("T:Gyro"   )); break;
    case TEST_PUSH_DETECT:  display.print(F("T:Push"   )); break;
    default: break;
  }
}

void handleTestMode()
{
  // ── C button → cycle to next test sub-mode ──
  if (buttonC.getSingleDebouncedPress())
  {
    motors.setSpeeds(0, 0);   // stop motors when switching
    currentTestMode = (TestMode)((int)currentTestMode + 1);
    if (currentTestMode >= NUM_TEST_MODES)
      currentTestMode = TEST_LINE_SENSORS;

    // reset helpers for the new mode
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    turnSensorReset();
    testPrintTimer  = 0;
    testPushEvading = false;
    lastPushCheck   = millis();

    showTestModeHeader();
    Serial.print(F("\n--- Switched to test: "));
    Serial.println((int)currentTestMode);
    return;
  }

  // ── Throttled output (LCD + Serial) ──
  bool shouldPrint = (millis() - testPrintTimer >= TEST_PRINT_INTERVAL);

  switch (currentTestMode)
  {
    // ─────────────────────────────────────────────────────────────
    //  LINE SENSORS — raw values for colour / threshold calibration
    //  Serial format:  LINE  s0  s1  s2  s3  s4  (tab-separated)
    //  Open Serial Plotter to see live graphs.
    // ─────────────────────────────────────────────────────────────
    case TEST_LINE_SENSORS:
    {
      lineSensors.read(lineSensorValues);
      // LCD: show first and last sensor + boundary flag
      display.gotoXY(0, 1);
      display.print(lineSensorValues[0]);
      display.print(' ');
      display.print(lineSensorValues[4]);
      display.print(checkBoundary() ? F(" B") : F("  "));

      if (shouldPrint)
      {
        testPrintTimer = millis();
        Serial.print(F("LINE\t"));
        for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
        {
          Serial.print(lineSensorValues[i]);
          if (i < NUM_LINE_SENSORS - 1) Serial.print('\t');
        }
        Serial.print(F("\tBoundary="));
        Serial.println(checkBoundary() ? F("YES") : F("no"));
      }
      break;
    }

    // ─────────────────────────────────────────────────────────────
    //  PROXIMITY SENSORS
    // ─────────────────────────────────────────────────────────────
    case TEST_PROXIMITY:
    {
      proxSensors.read();
      uint8_t lv = proxSensors.countsFrontWithLeftLeds();
      uint8_t rv = proxSensors.countsFrontWithRightLeds();
      bool seen = (lv >= PROX_SEARCH_THRESHOLD) || (rv >= PROX_SEARCH_THRESHOLD);

      display.gotoXY(0, 1);
      display.print(F("L"));
      display.print(lv);
      display.print(F(" R"));
      display.print(rv);
      display.print(seen ? F(" !") : F("  "));

      if (shouldPrint)
      {
        testPrintTimer = millis();
        Serial.print(F("PROX\tL="));
        Serial.print(lv);
        Serial.print(F("\tR="));
        Serial.print(rv);
        Serial.print(F("\tSeen="));
        Serial.println(seen ? F("YES") : F("no"));
      }
      break;
    }

    // ─────────────────────────────────────────────────────────────
    //  ENCODERS + PUSH DETECTION
    //  Does NOT reset counts so you see cumulative ticks.
    //  Push detect runs against a short sample window.
    // ─────────────────────────────────────────────────────────────
    case TEST_ENCODERS:
    {
      int16_t lc = encoders.getCountsLeft();
      int16_t rc = encoders.getCountsRight();

      display.gotoXY(0, 1);
      display.print(lc);
      display.print(' ');
      display.print(rc);
      display.print(F("   "));

      if (shouldPrint)
      {
        testPrintTimer = millis();
        Serial.print(F("ENC\tL="));
        Serial.print(lc);
        Serial.print(F("\tR="));
        Serial.println(rc);
      }
      break;
    }

    // ─────────────────────────────────────────────────────────────
    //  MOTORS — slow spin so you can verify wiring + direction
    //  Left turns forward, Right turns forward, both at 100.
    // ─────────────────────────────────────────────────────────────
    case TEST_MOTORS:
    {
      motors.setSpeeds(100, 100);
      int16_t lc = encoders.getCountsLeft();
      int16_t rc = encoders.getCountsRight();

      display.gotoXY(0, 1);
      display.print(F("FWD 100 "));

      if (shouldPrint)
      {
        testPrintTimer = millis();
        Serial.print(F("MOTOR\tspd=100\tencL="));
        Serial.print(lc);
        Serial.print(F("\tencR="));
        Serial.println(rc);
      }
      break;
    }

    // ─────────────────────────────────────────────────────────────
    //  GYRO — live angle (degrees)
    // ─────────────────────────────────────────────────────────────
    case TEST_GYRO:
    {
      turnSensorUpdate();
      int32_t degrees = (((int32_t)turnAngle >> 16) * 360) >> 16;

      display.gotoXY(0, 1);
      display.print(degrees);
      display.print(F(" deg  "));

      if (shouldPrint)
      {
        testPrintTimer = millis();
        Serial.print(F("GYRO\tdeg="));
        Serial.println(degrees);
      }
      break;
    }

    // ─────────────────────────────────────────────────────────────
    //  PUSH DETECTION — drive forward slowly, beep + evade on push
    // ─────────────────────────────────────────────────────────────
    case TEST_PUSH_DETECT:
    {
      if (testPushEvading)
      {
        // ── Mini evade sequence inside test mode ──
        unsigned long evadeElapsed = millis() - testEvadeStart;

        if (evadeElapsed < EVADE_DURATION / 3)
        {
          motors.setSpeeds(-EVADE_SPEED, -EVADE_SPEED);  // reverse
          display.gotoXY(0, 1);
          display.print(F("<<REV "));
        }
        else if (evadeElapsed < EVADE_DURATION * 2 / 3)
        {
          motors.setSpeeds(EVADE_SPEED, -EVADE_SPEED);   // spin CW
          display.gotoXY(0, 1);
          display.print(F(">>SPIN"));
        }
        else
        {
          // Evade done — resume crawling forward
          motors.setSpeeds(0, 0);
          testPushEvading = false;
          encoders.getCountsAndResetLeft();
          encoders.getCountsAndResetRight();
          lastPushCheck = millis();
          display.gotoXY(0, 1);
          display.print(F("ready "));
          Serial.println(F("PUSH\tEvade complete — resuming"));
        }
      }
      else
      {
        // ── Crawl forward and monitor for push ──
        motors.setSpeeds(100, 100);

        if (checkBeingPushed())
        {
          // Push detected! Beep and evade
          buzzer.playNote(NOTE_A(5), 200, 15);
          testPushEvading = true;
          testEvadeStart  = millis();
          Serial.println(F("PUSH\t*** PUSH DETECTED — evading ***"));

          display.gotoXY(0, 1);
          display.print(F("PUSH! "));
        }
        else
        {
          int16_t lc = encoders.getCountsLeft();
          int16_t rc = encoders.getCountsRight();

          display.gotoXY(0, 1);
          display.print(F("OK "));
          display.print(lc);
          display.print(' ');
          display.print(rc);
          display.print(F("  "));

          if (shouldPrint)
          {
            testPrintTimer = millis();
            Serial.print(F("PUSH\tL="));
            Serial.print(lc);
            Serial.print(F("\tR="));
            Serial.println(rc);
          }
        }
      }
      break;
    }

    default: break;
  }
}
