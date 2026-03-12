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
const uint16_t BOUNDARY_TOLERANCE = 350;

uint16_t lineSensorValues[NUM_LINE_SENSORS];
uint16_t floorBaseline[NUM_LINE_SENSORS];
bool floorCalibrated = false;

// PROXIMITY THRESHOLDS
// Sensor returns 0–6.  Lower = more sensitive (detects farther away).
const uint8_t PROX_SEARCH_THRESHOLD = 3; // used during scan / search / cross-ring
const uint8_t PROX_ATTACK_THRESHOLD = 4; // used during attack (closer = real target)

// SPEED SETTINGS
const int16_t SEARCH_SPEED = 300; // slow — conserve position
const int16_t ATTACK_SPEED = 400; // max — full ram
const int16_t SCAN_SPEED = 400;   // 360° scan
const int16_t EVADE_SPEED = 400;  // 360° scan
const int16_t TURN_SPEED = 400;   // general turning

// BUZZER VOLUME
const uint8_t BUZZER_VOLUME = 9; // 0–15 — applies to every sound

// SCAN / SEARCH TIMING
const unsigned long SCAN_TIMEOUT = 2500;     // ms for 360° spin
const unsigned long CROSS_RING_TIME = 1200;  // ms driving to far side
const unsigned long SEARCH_DRIVE_TIME = 800; // ms forward per leg
const unsigned long SEARCH_TURN_TIME = 400;  // ms turning per leg
const unsigned long REACQUIRE_TIMEOUT = 400; // ms to reacquire target

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
bool searchTurnDir = false; // false = right, true = left
unsigned long searchLegStart = 0;
bool searchDriving = true;

// Attack / tracking
bool lastSenseRight = true;       // last-known opponent side
unsigned long objectLastSeen = 0; // millis() timestamp

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
//  Called BEFORE countdown starts. Robot must be sitting still on the arena
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
  delay(500);
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
    calibrateFloor();          // calibrate BEFORE countdown
    doCountdownAndScan(false); // scan left (CCW)
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    ledRed(0);
    calibrateFloor();         // calibrate BEFORE countdown
    doCountdownAndScan(true); // scan right (CW)
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
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
  int lastAnnouncedSecond = -1;

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

//  STATE: SEARCH — fuzzy bouncing search pattern
void handleSearch()
{
  // B button = emergency stop
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  // Continuously scan for opponent
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
    display.clear();
    display.print(F("SEARCH"));
    searchLegStart = millis();
    searchDriving = true;
    return;
  }

  // Alternating drive / turn legs
  unsigned long legElapsed = millis() - searchLegStart;

  if (searchDriving)
  {
    drive(SEARCH_SPEED, SEARCH_SPEED);
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
      drive(-TURN_SPEED, TURN_SPEED); // left
    else
      drive(TURN_SPEED, -TURN_SPEED); // right

    if (legElapsed > SEARCH_TURN_TIME)
    {
      searchDriving = true;
      searchTurnDir = !searchTurnDir; // alternate next time
      searchLegStart = millis();
    }
  }

  // LCD
  display.gotoXY(0, 1);
  display.print(searchDriving ? F(">>FWD ") : F(">>TRN "));
}

//  STATE: ATTACK — charge the opponent!
void handleAttack()
{
  // B button = emergency stop
  if (buttonB.getSingleDebouncedPress())
  {
    emergencyStop();
    return;
  }

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();
  bool seen = (lv >= PROX_ATTACK_THRESHOLD) || (rv >= PROX_ATTACK_THRESHOLD);

  if (checkBoundary())
  {
    bounceOffBoundary();

    proxSensors.read();
    lv = proxSensors.countsFrontWithLeftLeds();
    rv = proxSensors.countsFrontWithRightLeds();
    if (lv >= PROX_ATTACK_THRESHOLD || rv >= PROX_ATTACK_THRESHOLD)
    {
      lastSenseRight = (rv >= lv);
      objectLastSeen = millis();
      display.clear();
      display.print(F("ATTACK!"));
    }
    else
    {
      transitionToSearch();
    }
    return;
  }

  if (seen)
  {
    ledYellow(1);
    objectLastSeen = millis();

    if (rv > lv)
    {
      // Object biased right — curve right
      int16_t adj = map(rv - lv, 0, 6, 0, 300);
      drive(ATTACK_SPEED, ATTACK_SPEED - adj);
      lastSenseRight = true;
    }
    else if (lv > rv)
    {
      // Object biased left — curve left
      int16_t adj = map(lv - rv, 0, 6, 0, 300);
      drive(ATTACK_SPEED - adj, ATTACK_SPEED);
      lastSenseRight = false;
    }
    else
    {
      // Dead-centre — full charge!
      drive(ATTACK_SPEED, ATTACK_SPEED);
    }

    display.gotoXY(0, 1);
    display.print(F("ATK "));
    display.print(lv);
    display.print(' ');
    display.print(rv);
  }
  else
  {
    // Lost sight
    ledYellow(0);

    display.gotoXY(0, 1);
    display.print(F("LOST    "));

    if (lastSenseRight)
      drive(TURN_SPEED, -TURN_SPEED); // turn right
    else
      drive(-TURN_SPEED, TURN_SPEED); // turn left

    // Give up after REACQUIRE_TIMEOUT and switch to search
    if (millis() - objectLastSeen > REACQUIRE_TIMEOUT)
    {
      transitionToSearch();
    }
  }
}

//  HELPER: transition into ATTACK state
void transitionToAttack(bool opponentOnRight)
{
  lastSenseRight = opponentOnRight;
  objectLastSeen = millis();

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

  searchDriving = true;
  searchTurnDir = !searchTurnDir; // vary direction each time
  searchLegStart = millis();

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