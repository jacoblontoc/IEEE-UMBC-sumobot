/* This example uses the front proximity sensor on the Zumo 32U4
Front Sensor Array to locate an opponent robot or any other
reflective object. Using the motors to turn, it scans its
surroundings. If it senses an object, it turns on its yellow LED
and drives towards that object, stopping at a set distance. */

#include <Wire.h>
#include <Zumo32U4.h>

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 1;

// ===== ADJUST THIS VALUE TO SET STOPPING DISTANCE =====
// How close the robot gets before stopping. Proximity sensor
// readings range from 0 (nothing) to 6 (very close).
//   Lower value = robot stops FARTHER from opponent
//   Higher value = robot stops CLOSER to opponent
// Recommended range: 3 (far) to 6 (very close)
const uint8_t stopDistanceThreshold = 6;
// =======================================================

// The maximum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMax = 300;

// The minimum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMin = 100;

// The speed at which the robot drives forward towards the opponent.
// 400 is full speed. Adjust for more or less aggressive driving.
const uint16_t driveSpeed = 400;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  When no object is seen,
// this variable helps us make a good guess about which direction
// to turn.
bool senseDir = RIGHT;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;

void setup()
{
  proxSensors.initFrontSensor();

  // Wait for the user to press A before driving the motors.
  display.clear();
  display.print(F("Press A"));
  buttonA.waitForButton();
  display.clear();
}

void turnRight()
{
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;
}

void turnLeft()
{
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
}

void stop()
{
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
}

void loop()
{
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

  if (objectSeen)
  {
    // An object is visible, so we will start decelerating in
    // order to help the robot find the object without
    // overshooting or oscillating.
    turnSpeed -= deceleration;
  }
  else
  {
    // An object is not visible, so we will accelerate in order
    // to help find the object sooner.
    turnSpeed += acceleration;
  }

  // Constrain the turn speed so it is between turnSpeedMin and
  // turnSpeedMax.
  turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

  if (objectSeen)
  {
    // An object seen.
    ledYellow(1);

    lastTimeObjectSeen = millis();

    // Use the higher of the two sensor readings as our
    // proximity estimate.
    uint8_t closestReading = max(leftValue, rightValue);

    if (leftValue < rightValue)
    {
      // The right value is greater, so the object is probably
      // closer to the robot's right LEDs, which means the robot
      // is not facing it directly.  Turn to the right to try to
      // make it more even.
      turnRight();
      senseDir = RIGHT;
    }
    else if (leftValue > rightValue)
    {
      // The left value is greater, so turn to the left.
      turnLeft();
      senseDir = LEFT;
    }
    else
    {
      // The left and right values are roughly equal, meaning the
      // object is straight ahead. Drive towards it or stop.

      // ===== STOPPING DISTANCE CHECK =====
      // If the sensor reading meets or exceeds stopDistanceThreshold,
      // the opponent is close enough — stop the motors.
      // Adjust stopDistanceThreshold (defined above) to change
      // how far or close the robot stops from the opponent.
      if (closestReading >= stopDistanceThreshold)
      {
        // Opponent is within desired stopping distance — halt.
        stop();
      }
      else
      {
        // Opponent is ahead but still far away — drive forward.
        motors.setSpeeds(driveSpeed, driveSpeed);
        turningLeft = false;
        turningRight = false;
      }
      // ====================================
    }
  }
  else
  {
    // No object is seen, so just keep turning in the direction
    // that we last sensed the object.
    ledYellow(0);

    if (senseDir == RIGHT)
    {
      turnRight();
    }
    else
    {
      turnLeft();
    }
  }

  display.gotoXY(0, 0);
  display.print(leftValue);
  display.print(' ');
  display.print(rightValue);
  display.gotoXY(0, 1);
  display.print(turningRight ? 'R' : (turningLeft ? 'L' : ' '));
  display.print(' ');
  display.print(turnSpeed);
  display.print(' ');
  display.print(' ');
}