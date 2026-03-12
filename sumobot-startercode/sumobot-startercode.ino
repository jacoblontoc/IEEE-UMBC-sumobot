/*  ============================================================
 *  IEEE @ UMBC — Zumo 32U4 Sumo Robot  (30-inch Dohyo)
 *  ============================================================
 *
 *  CONTROLS
 *    Button A — Start 5-second countdown + CCW scan, then fight
 *    Button B — Emergency stop (kills motors, returns to idle)
 *    Button C — Start 5-second countdown + CW scan, then fight
 *
 *  STATE MACHINE
 *    IDLE ──(A)──▶ [countdown + CCW scan] ──▶ CROSS_RING ──▶ SEARCH
 *    IDLE ──(C)──▶ [countdown + CW scan]  ──▶ CROSS_RING ──▶ SEARCH
 *    SEARCH ──▶ ATTACK ──▶ SEARCH
 *    Any state ──(B)──▶ IDLE
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
Zumo32U4LCD display;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

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
const uint16_t LINE_THRESHOLD = 500;
const bool BOUNDARY_IS_WHITE = true;
// =================================================================

#define NUM_LINE_SENSORS 5
uint16_t lineSensorValues[NUM_LINE_SENSORS];

// ===================== PROXIMITY THRESHOLDS ======================
// Sensor returns 0–6.  Lower = more sensitive (detects farther away).
const uint8_t PROX_SEARCH_THRESHOLD = 3; // used during scan / search / cross-ring
const uint8_t PROX_ATTACK_THRESHOLD = 4; // used during attack (closer = real target)

// ===================== SPEED SETTINGS ============================
const int16_t SEARCH_SPEED = 150; // slow — conserve position
const int16_t ATTACK_SPEED = 400; // max — full ram
const int16_t SCAN_SPEED = 250;   // 360° scan
const int16_t EVADE_SPEED = 400;  // escape burst
const int16_t TURN_SPEED = 400;   // general turning

// ===================== BUZZER VOLUME =============================
const uint8_t BUZZER_VOLUME = 9; // 0–15 — applies to every sound

// ============= ENCODER / PUSH-DETECTION SETTINGS =================
//  While commanding forward at ATTACK_SPEED, the encoders
//  normally return a large positive count every PUSH_CHECK_INTERVAL.
//  If the counts fall below PUSH_DETECT_THRESHOLD the robot
//  is stalled or being pushed backward.
const int16_t PUSH_DETECT_THRESHOLD = 10;      // encoder ticks
const unsigned long PUSH_CHECK_INTERVAL = 100; // ms
const unsigned long EVADE_DURATION = 600;      // ms total evade

// ============= SCAN / SEARCH TIMING ==============================
const unsigned long SCAN_TIMEOUT = 2500;     // ms for 360° spin
const unsigned long CROSS_RING_TIME = 1200;  // ms driving to far side
const unsigned long SEARCH_DRIVE_TIME = 800; // ms forward per leg
const unsigned long SEARCH_TURN_TIME = 400;  // ms turning per leg
const unsigned long REACQUIRE_TIMEOUT = 400; // ms to reacquire target

// ========================= STATE MACHINE =========================
enum RobotState
{
  STATE_IDLE,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// Push detection
unsigned long lastPushCheck = 0;

// Search pattern
bool searchTurnDir = false; // false = right, true = left
unsigned long searchLegStart = 0;
bool searchDriving = true;

// Attack / tracking
bool lastSenseRight = true;       // last-known opponent side
unsigned long objectLastSeen = 0; // millis() timestamp

// =================================================================
//                         SETUP
// =================================================================
void setup()
{
  Serial.begin(9600);

  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  display.clear();
  display.print(F("A=CCW B=Stp"));
  display.gotoXY(0, 1);
  display.print(F("C=CW"));

  currentState = STATE_IDLE;
}

// =================================================================
//                        MAIN LOOP
// =================================================================
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

// =================================================================
//  EMERGENCY STOP — returns to IDLE
// =================================================================
void emergencyStop()
{
  motors.setSpeeds(0, 0);
  buzzer.playNote(NOTE_C(3), 300, BUZZER_VOLUME);
  ledYellow(0);
  ledRed(1);
  display.clear();
  display.print(F("STOPPED"));
  display.gotoXY(0, 1);
  display.print(F("A=CCW C=CW"));
  currentState = STATE_IDLE;
}

// =================================================================
//  STATE: IDLE — waiting for button press
//    A = countdown + CCW scan, then fight
//    B = emergency stop
//    C = countdown + CW scan, then fight
// =================================================================
void handleIdle()
{
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    doCountdownAndScan(false); // scan left (CCW)
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    ledRed(0);
    doCountdownAndScan(true); // scan right (CW)
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
  }
}

// =================================================================
//  HELPER: blocking countdown + 360° scan
//  Called from handleIdle() when A or B is pressed.
//  scanCW: true = spin right (CW), false = spin left (CCW)
//  If opponent is detected during the scan, transitions directly
//  to ATTACK and returns; otherwise returns to let caller set
//  STATE_CROSS_RING.
// =================================================================
void doCountdownAndScan(bool scanCW)
{
  // ── 5-4-3-2-1 countdown (blocking) ──
  for (int i = 5; i >= 1; i--)
  {
    display.clear();
    display.gotoXY(3, 0);
    display.print(i);

    switch (i)
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
    delay(1000);
  }

  // ── GO! ──
  display.clear();
  display.gotoXY(2, 0);
  display.print(F("GO!"));
  buzzer.playNote(NOTE_C(6), 300, BUZZER_VOLUME);
  delay(300);

  // ── 360° scan (blocking spin) ──
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();

  display.clear();
  display.print(F("SCANNING"));

  unsigned long scanStart = millis();

  while (true)
  {
    turnSensorUpdate();

    if (scanCW)
      motors.setSpeeds(SCAN_SPEED, -SCAN_SPEED);
    else
      motors.setSpeeds(-SCAN_SPEED, SCAN_SPEED);

    // Check for opponent while spinning
    proxSensors.read();
    uint8_t lv = proxSensors.countsFrontWithLeftLeds();
    uint8_t rv = proxSensors.countsFrontWithRightLeds();

    if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
    {
      motors.setSpeeds(0, 0);
      transitionToAttack(rv >= lv);
      return; // go straight to ATTACK — caller's CROSS_RING set is skipped
    }

    // Compute degrees rotated
    uint32_t rotated = scanCW ? -turnAngle : turnAngle;
    int32_t degrees = ((int32_t)(rotated >> 16) * 360L) >> 16;

    // Full rotation complete (with 1s guard against gyro noise)
    if (millis() - scanStart > 1000 && degrees >= 350)
    {
      motors.setSpeeds(0, 0);
      display.clear();
      display.print(F("CROSS"));
      return;
    }
  }
}

// =================================================================
//  STATE: CROSS_RING — drive to the far side of the 30″ dohyo
// =================================================================
void handleCrossRing()
{
  // ── B button = emergency stop ──
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

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
  // ── B button = emergency stop ──
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

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
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // left
    else
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // right

    if (legElapsed > SEARCH_TURN_TIME)
    {
      searchDriving = true;
      searchTurnDir = !searchTurnDir; // alternate next time
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
  // ── B button = emergency stop ──
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();
  bool seen = (lv >= PROX_ATTACK_THRESHOLD) || (rv >= PROX_ATTACK_THRESHOLD);

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
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // turn right
    else
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // turn left

    // Give up after REACQUIRE_TIMEOUT and switch to search
    if (millis() - objectLastSeen > REACQUIRE_TIMEOUT)
    {
      transitionToSearch();
    }
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

  currentState = STATE_ATTACK;
  stateStartTime = millis();

  ledYellow(1);
  buzzer.playNote(NOTE_C(6), 300, BUZZER_VOLUME); // high-pitch beep — target found
  display.clear();
  display.print(F("ATTACK!"));
}

// =================================================================
//  HELPER: transition into SEARCH state
// =================================================================
void transitionToSearch()
{
  motors.setSpeeds(0, 0);

  searchDriving = true;
  searchTurnDir = !searchTurnDir; // vary direction each time
  searchLegStart = millis();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  currentState = STATE_SEARCH;
  stateStartTime = millis();

  ledYellow(0);
  buzzer.playNote(NOTE_C(3), 400, BUZZER_VOLUME); // low-pitch beep — target lost
  display.clear();
  display.print(F("SEARCH"));
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
    if (triggered)
      return true;
  }
  return false;
}

// Returns which side of the robot hit the boundary.
//   -1 = left,  0 = centre / both,  1 = right
int8_t getBoundarySide()
{
  // lineSensorValues[] already populated by checkBoundary()
  bool leftHit = false;
  bool rightHit = false;

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    bool triggered = BOUNDARY_IS_WHITE
                         ? (lineSensorValues[i] < LINE_THRESHOLD)
                         : (lineSensorValues[i] > LINE_THRESHOLD);
    if (triggered)
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

  // Reverse briefly
  motors.setSpeeds(-SEARCH_SPEED, -SEARCH_SPEED);
  delay(200);

  // Turn away from the boundary edge
  if (side <= 0) // left or centre → turn right
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  else // right → turn left
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);

  delay(250 + random(200)); // randomised to vary the search path

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
  if (millis() - lastPushCheck < PUSH_CHECK_INTERVAL)
    return false;

  int16_t leftCounts = encoders.getCountsAndResetLeft();
  int16_t rightCounts = encoders.getCountsAndResetRight();
  lastPushCheck = millis();

  // Both wheels barely turning despite full-speed command → pushed
  return (leftCounts < PUSH_DETECT_THRESHOLD &&
          rightCounts < PUSH_DETECT_THRESHOLD);
}
