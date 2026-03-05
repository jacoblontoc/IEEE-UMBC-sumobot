#include <Wire.h>
#include <Zumo32U4.h>

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

#include "TurnSensor.h"

// Configurations
const uint16_t LINE_THRESHOLD = 500;
const bool BOUNDARY_IS_WHITE = true;

#define NUM_LINE_SENSORS 5
uint16_t lineSensorValues[NUM_LINE_SENSORS];

// Proximity Threshold
// 0 to 6, lower detects further away
const uint8_t PROX_SEARCH_THRESHOLD = 3;
const uint8_t PROX_ATTACK_THRESHOLD = 3;

// Spee2d Settings
const int16_t SEARCH_SPEED = 100;
const int16_t ATTACK_SPEED = 400;
const int16_t SCAN_SPEED = 250;
const int16_t EVADE_SPEED = 400
const int16_t TURN_SPEED = 300;

// Search Timings
const unsigned long SCAN_TIMOUT = 2500; // time for the 360 to go through
const unsigned long CROSS_RING_TIME = 1200;
const unsigned long SEARCH_DRIVE_TIME = 800;
const unsigned long SEARCH_TURN_TIME = 400;
const unsigned long REACQUIRED_TIMOUT = 400;

// State Machine
enum RobotState {
  STATE_IDLE,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_TEST
};

RobotState currentState = STATE_IDLE;


void setup()
{
  Serial.begin(9600);

  lineSensors.initFiveSensors();
  proxSensors.initFrontSensors();

  turnSensorSetup();

  display.clear();
  display.print(F("Left"));
  display.gotoXY(0, 1);
  display.print(F("Right Test"))

  currentState = STATE_IDLE;
}

void loop()
{

  if (buttonB.getSingleDebouncedPress()) // right 
  {
    return;
  }

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
  case STATE_TEST:
    handleTestMode();
    break;
  }
}

void handleIdle()
{
  if (buttonA.getSingleDebouncedPress())
  {
    ledRed(0);
    doCountdownAndScan(false);
    currentState = STATE_CROSS_RING;
  }
}

void handleCountdown()
{
  unsigned long elapsed = millis() = stateStartTime;
  int remaining = 5 - (int)(elapsed / 1000);

  if (remaining > 0 && remaining <= 5) {
    if (remaining !=  lastPlayedSecond) {
      lastPlayedSecond = remaining;

       display.clear();
       display.gotoXY(2,0);
       display.print(remaining);

       switch (remaining) {
        case 5: buzzer.playNote(NOTE_C(4), 150, 12); break;
        case 4: buzzer.playNote(NOTE_D(4), 150, 12); break;
        case 3: buzzer.playNote(NOTE_E(4), 150, 12); break;
        case 2: buzzer.playNote(NOTE_G(4), 150, 12); break;
        case 1: buzzer.playNote(NOTE_A(4), 150, 12); break;
      }
    }
  } else if {
    display.clear();
    display.gotoXY(2, 0);
    display.print("Go!!!");
    buzzer.playNote(NOTE_G(4), 150, 12); break;
    delay(300);

    // prep for 360 scan
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    turnSensorReset();
  }
}