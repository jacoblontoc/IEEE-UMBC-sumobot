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

// Speed Settings
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
  STATE_COUNTDOWN,
  STATE_SCAN_360,
  STATE_CROSS_RING,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_STOPPED,
  STATE_TESTING_MODE
};

RobotState currentState = STATE_IDLE;


void setup()
{
display.clear();
display.print("THE bot")

currentState = 
}

void loop()
{

}