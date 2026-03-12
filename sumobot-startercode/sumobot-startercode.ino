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

// DYNAMIC FLOOR CALIBRATION
#define NUM_LINE_SENSORS 5

const uint8_t  CALIBRATION_SAMPLES   = 30;
const uint8_t  CALIBRATION_DELAY_MS  = 5;
const uint16_t BOUNDARY_TOLERANCE    = 350;

uint16_t lineSensorValues[NUM_LINE_SENSORS];
uint16_t floorBaseline[NUM_LINE_SENSORS];
bool floorCalibrated = false;

// PROXIMITY THRESHOLDS
// OPT: unified to 3 — engage and hold target sooner in all modes
const uint8_t PROX_SEARCH_THRESHOLD = 3;
const uint8_t PROX_ATTACK_THRESHOLD = 3;

// SPEED SETTINGS
// OPT: SEARCH_SPEED raised to 400 — cover ring faster, engage sooner
const int16_t SEARCH_SPEED = 400; 
const int16_t ATTACK_SPEED = 400;
const int16_t SCAN_SPEED   = 400;
const int16_t EVADE_SPEED  = 400;
const int16_t TURN_SPEED   = 400;

// BUZZER VOLUME
const uint8_t BUZZER_VOLUME = 9;

// SCAN / SEARCH TIMING
// OPT: SCAN_TIMEOUT reduced — full spin takes <2s at this speed, 2500 wastes time
// OPT: search legs tightened — denser sweep pattern
const unsigned long SCAN_TIMEOUT       = 2000;  
const unsigned long CROSS_RING_TIME    = 1200;
const unsigned long SEARCH_DRIVE_TIME  = 500;   
const unsigned long SEARCH_TURN_TIME   = 300;
const unsigned long REACQUIRE_TIMEOUT  = 400;

// STATE MACHINE
// OPT: added STATE_EVADE to replace blocking delay() calls in bounceOffBoundary()
enum RobotState
{
  STATE_IDLE,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_EVADE   // NEW — non-blocking boundary evasion
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// Search pattern
bool searchTurnDir = false;
unsigned long searchLegStart = 0;
bool searchDriving = true;

// Attack / tracking
bool lastSenseRight = true;
unsigned long objectLastSeen = 0;

// OPT: Evade state — replaces blocking delay() in bounceOffBoundary()
enum EvadePhase { EVADE_REVERSE, EVADE_TURN };
EvadePhase evadePhase;
int8_t evadeTurnDir;
unsigned long evadePhaseStart;
RobotState evadeReturnState; // which state to go to after evade completes

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
  case STATE_IDLE:       handleIdle();      break;
  case STATE_CROSS_RING: handleCrossRing(); break;
  case STATE_SEARCH:     handleSearch();    break;
  case STATE_ATTACK:     handleAttack();    break;
  case STATE_EVADE:      handleEvade();     break;
  }
}

//  EMERGENCY STOP
void emergencyStop()
{
  motors.setSpeeds(0, 0);
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
      sums[index] += lineSensorValues[index];
    delay(CALIBRATION_DELAY_MS);
  }

  Serial.print(F("Baseline -> "));
  for (uint8_t index = 0; index < NUM_LINE_SENSORS; index++)
  {
    floorBaseline[index] = (uint16_t)(sums[index] / CALIBRATION_SAMPLES);
    Serial.print(F("S")); Serial.print(index);
    Serial.print(F("=")); Serial.print(floorBaseline[index]);
    if (index < NUM_LINE_SENSORS - 1) Serial.print(F("  "));
  }
  Serial.println();
  Serial.print(F("Tolerance: +/-"));
  Serial.println(BOUNDARY_TOLERANCE);

  floorCalibrated = true;

  display.clear();
  display.print(F("CAL OK"));
  display.gotoXY(0, 1);
  display.print(F("C="));
  display.print(floorBaseline[2]);
  delay(500);
}

//  STATE: IDLE
void handleIdle()
{
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    calibrateFloor();
    doCountdownAndScan(false); // CCW
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    ledRed(0);
    calibrateFloor();
    doCountdownAndScan(true);  // CW
    currentState = STATE_CROSS_RING;
    stateStartTime = millis();
  }
}

//  HELPER: 5-second countdown + 360° scan
void doCountdownAndScan(bool scanCW)
{
  unsigned long countdownStart = millis();
  int lastAnnouncedSecond = -1;

  while (true)
  {
    unsigned long elapsed = millis() - countdownStart;
    if (elapsed >= 5000) break;

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
      case 5: buzzer.playNote(NOTE_C(4), 150, BUZZER_VOLUME); break;
      case 4: buzzer.playNote(NOTE_D(4), 150, BUZZER_VOLUME); break;
      case 3: buzzer.playNote(NOTE_E(4), 150, BUZZER_VOLUME); break;
      case 2: buzzer.playNote(NOTE_G(4), 150, BUZZER_VOLUME); break;
      case 1: buzzer.playNote(NOTE_A(4), 150, BUZZER_VOLUME); break;
      }
    }
  }

  display.clear();
  display.gotoXY(2, 0);
  display.print(F("GO!"));
  buzzer.playNote(NOTE_C(6), 300, BUZZER_VOLUME);

  display.clear();
  display.print(F("SCANNING"));

  unsigned long scanStart = millis();

  while (true)
  {
    if (scanCW)
      motors.setSpeeds(SCAN_SPEED, -SCAN_SPEED);
    else
      motors.setSpeeds(-SCAN_SPEED, SCAN_SPEED);

    proxSensors.read();
    uint8_t lv = proxSensors.countsFrontWithLeftLeds();
    uint8_t rv = proxSensors.countsFrontWithRightLeds();

    if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
    {
      motors.setSpeeds(0, 0);
      transitionToAttack(rv >= lv);
      return;
    }

    // OPT: SCAN_TIMEOUT reduced to 2000ms — full spin done well before this
    if (millis() - scanStart >= SCAN_TIMEOUT)
    {
      motors.setSpeeds(0, 0);
      display.clear();
      display.print(F("CROSS"));
      return;
    }
  }
}

//  STATE: CROSS_RING
void handleCrossRing()
{
  if (buttonB.getSingleDebouncedPress()) { emergencyStop(); return; }

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  if (checkBoundary())
  {
    transitionToEvade(STATE_SEARCH);
    return;
  }

  display.gotoXY(0, 0);
  display.print(F("CROSSING"));
  motors.setSpeeds(SEARCH_SPEED, SEARCH_SPEED);

  if (millis() - stateStartTime > CROSS_RING_TIME)
    transitionToSearch();
}

//  STATE: SEARCH
void handleSearch()
{
  if (buttonB.getSingleDebouncedPress()) { emergencyStop(); return; }

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_SEARCH_THRESHOLD || rv >= PROX_SEARCH_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    transitionToAttack(rv >= lv);
    return;
  }

  if (checkBoundary())
  {
    transitionToEvade(STATE_SEARCH);
    return;
  }

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
    if (searchTurnDir)
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    else
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);

    if (legElapsed > SEARCH_TURN_TIME)
    {
      searchDriving = true;
      searchTurnDir = !searchTurnDir;
      searchLegStart = millis();
    }
  }

  display.gotoXY(0, 1);
  display.print(searchDriving ? F(">>FWD ") : F(">>TRN "));
}

//  STATE: ATTACK
//
//  OPT 1: Opponent check comes BEFORE boundary check.
//         If we can see the opponent we keep pushing even if
//         our own sensors are over the tawara — we may be
//         about to shove them out. Only bounce when we've lost
//         sight near the edge.
//
//  OPT 2: Reacquire by curving forward rather than spinning
//         in place — covers more ground and tracks a moving
//         opponent better.
void handleAttack()
{
  if (buttonB.getSingleDebouncedPress()) { emergencyStop(); return; }

  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();
  bool seen = (lv >= PROX_ATTACK_THRESHOLD) || (rv >= PROX_ATTACK_THRESHOLD);

  // OPT: only evade boundary when we've LOST the opponent.
  // If we can still see them, keep pushing — this is the win moment.
  if (checkBoundary() && !seen)
  {
    transitionToEvade(STATE_SEARCH);
    return;
  }

  if (seen)
  {
    ledYellow(1);
    objectLastSeen = millis();

    if (rv > lv)
    {
      int16_t adj = map(rv - lv, 0, 6, 0, 300);
      motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED - adj);
      lastSenseRight = true;
    }
    else if (lv > rv)
    {
      int16_t adj = map(lv - rv, 0, 6, 0, 300);
      motors.setSpeeds(ATTACK_SPEED - adj, ATTACK_SPEED);
      lastSenseRight = false;
    }
    else
    {
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
    // OPT: curve forward toward last known side instead of spinning in place.
    // Covers more ring area and tracks a moving opponent far better.
    ledYellow(0);
    display.gotoXY(0, 1);
    display.print(F("LOST    "));

    if (lastSenseRight)
      motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED / 3); // curve right, still moving
    else
      motors.setSpeeds(ATTACK_SPEED / 3, ATTACK_SPEED); // curve left, still moving

    if (millis() - objectLastSeen > REACQUIRE_TIMEOUT)
      transitionToSearch();
  }
}

//  STATE: EVADE  (replaces blocking bounceOffBoundary())
//
//  OPT: All delays replaced with millis()-based timing so the
//       robot stays sensor-aware throughout the entire evade.
//       Reverse trimmed 400ms → 250ms. Turn window tightened.
//       After evade completes, returns to evadeReturnState.
void handleEvade()
{
  if (buttonB.getSingleDebouncedPress()) { emergencyStop(); return; }

  // If the opponent appears during evasion, abort and attack immediately
  proxSensors.read();
  uint8_t lv = proxSensors.countsFrontWithLeftLeds();
  uint8_t rv = proxSensors.countsFrontWithRightLeds();

  if (lv >= PROX_ATTACK_THRESHOLD || rv >= PROX_ATTACK_THRESHOLD)
  {
    motors.setSpeeds(0, 0);
    ledRed(0);
    transitionToAttack(rv >= lv);
    return;
  }

  unsigned long elapsed = millis() - evadePhaseStart;

  if (evadePhase == EVADE_REVERSE)
  {
    motors.setSpeeds(-EVADE_SPEED, -EVADE_SPEED);
    if (elapsed > 250) // was 400ms
    {
      evadePhase = EVADE_TURN;
      evadePhaseStart = millis();
      if (evadeTurnDir <= 0) // left hit or centre → turn right
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      else                   // right hit → turn left
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    }
  }
  else // EVADE_TURN
  {
    // OPT: shorter, randomised turn — was 250+random(200), now tighter
    unsigned long turnTime = 200 + random(150);
    if (elapsed > turnTime)
    {
      motors.setSpeeds(0, 0);
      ledRed(0);
      if (evadeReturnState == STATE_SEARCH)
        transitionToSearch();
      else
      {
        currentState = evadeReturnState;
        stateStartTime = millis();
      }
    }
  }
}

//  TRANSITION HELPERS
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

void transitionToSearch()
{
  motors.setSpeeds(0, 0);
  searchDriving = true;
  searchTurnDir = !searchTurnDir;
  searchLegStart = millis();
  currentState = STATE_SEARCH;
  stateStartTime = millis();
  ledYellow(0);
  buzzer.playNote(NOTE_C(3), 400, BUZZER_VOLUME);
  display.clear();
  display.print(F("SEARCH"));
}

// OPT: new helper — begins non-blocking evade, remembers where to return
void transitionToEvade(RobotState returnTo)
{
  evadeTurnDir = getBoundarySide();
  evadePhase = EVADE_REVERSE;
  evadePhaseStart = millis();
  evadeReturnState = returnTo;
  currentState = STATE_EVADE;
  motors.setSpeeds(-EVADE_SPEED, -EVADE_SPEED);
  ledRed(1);
  buzzer.playNote(NOTE_A(5), 80, BUZZER_VOLUME); // shorter beep than before
  display.clear();
  display.print(F("EVADE"));
}

//  BOUNDARY DETECTION
unsigned long lastBoundaryDebug = 0;
const unsigned long BOUNDARY_DEBUG_INTERVAL = 200;

bool checkBoundary()
{
  if (!floorCalibrated) return false;

  lineSensors.read(lineSensorValues);

  bool shouldPrint = (millis() - lastBoundaryDebug >= BOUNDARY_DEBUG_INTERVAL);
  bool boundaryHit = false;
  uint8_t triggerSensor = 255;
  uint16_t triggerDev = 0;

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    uint16_t baseline  = floorBaseline[i];
    uint16_t reading   = lineSensorValues[i];
    uint16_t deviation = (reading > baseline) ? (reading - baseline) : (baseline - reading);
    if (deviation > BOUNDARY_TOLERANCE && !boundaryHit)
    {
      boundaryHit    = true;
      triggerSensor  = i;
      triggerDev     = deviation;
    }
  }

  if (shouldPrint || boundaryHit)
  {
    lastBoundaryDebug = millis();
    Serial.print(F("LINE: "));
    for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
      uint16_t baseline = floorBaseline[i];
      uint16_t reading  = lineSensorValues[i];
      int16_t  diff     = (int16_t)reading - (int16_t)baseline;
      Serial.print(F("S")); Serial.print(i);
      Serial.print(F(":")); Serial.print(reading);
      Serial.print(F("("));
      if (diff >= 0) Serial.print(F("+"));
      Serial.print(diff);
      Serial.print(F(") "));
    }
    if (boundaryHit)
    {
      Serial.print(F(" >>> BOUNDARY! S"));
      Serial.print(triggerSensor);
      Serial.print(F(" dev="));  Serial.print(triggerDev);
      Serial.print(F(" > tol=")); Serial.print(BOUNDARY_TOLERANCE);
    }
    Serial.println();
  }

  return boundaryHit;
}

int8_t getBoundarySide()
{
  bool leftHit  = false;
  bool rightHit = false;

  for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++)
  {
    uint16_t baseline  = floorBaseline[i];
    uint16_t reading   = lineSensorValues[i];
    uint16_t deviation = (reading > baseline) ? (reading - baseline) : (baseline - reading);
    if (deviation > BOUNDARY_TOLERANCE)
    {
      if (i <= 1) leftHit  = true;
      if (i >= 3) rightHit = true;
      if (i == 2) { leftHit = true; rightHit = true; }
    }
  }

  if (leftHit && rightHit) return  0;
  if (leftHit)             return -1;
  if (rightHit)            return  1;
  return 0;
}