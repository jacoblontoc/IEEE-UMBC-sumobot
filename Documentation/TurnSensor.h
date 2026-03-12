// TurnSensor.h — Gyro-based turn tracking for Zumo 32U4
//
// Provides turnSensorSetup(), turnSensorReset(), turnSensorUpdate(),
// and the global `turnAngle` variable.
//
// This file must be included AFTER declaring objects named
// `display` and `imu` in your sketch.

#pragma once

#include <Wire.h>

// ----- Angle constants -----
const int32_t turnAngle45 = 0x20000000;            // 45 degrees
const int32_t turnAngle90 = turnAngle45 * 2;       // 90 degrees
const int32_t turnAngle1  = (turnAngle45 + 22) / 45; // ~1 degree

// turnAngle: 32-bit unsigned value where 0x20000000 = 45° CCW.
// Cast to int32_t to get a signed range of -180° to +180°.
uint32_t turnAngle = 0;

// Current angular rate (raw gyro units, 0.07 °/s per digit).
int16_t turnRate;

// Calibrated zero-rate offset.
int16_t gyroOffset;

// Timestamp of the last gyro update (microseconds).
uint16_t gyroLastUpdate = 0;

// ---------------------------------------------------------------
// Reset the angle counter to zero.
// ---------------------------------------------------------------
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// ---------------------------------------------------------------
// Call as often as possible to keep `turnAngle` up-to-date.
// ---------------------------------------------------------------
void turnSensorUpdate()
{
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  uint16_t m  = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  int32_t d = (int32_t)turnRate * dt;

  // Convert gyro-digits × µs  →  turnAngle units.
  // (0.07 dps/digit) × (1 / 1 000 000 s/µs) × (2^29 / 45 unit/°)
  //   = 14 680 064 / 17 578 125
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

// ---------------------------------------------------------------
// Calibrate the gyro.  Robot must be STATIONARY during this call.
// Shows "Gyro cal" on the display, takes ~1-2 seconds, then
// returns automatically (no button press required).
// ---------------------------------------------------------------
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));
  ledYellow(1);

  delay(500);  // Let the user take their hands off

  // Average 1024 gyro samples to find the zero-rate offset.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }

  ledYellow(0);
  gyroOffset = total / 1024;

  display.clear();
  display.print(F("Gyro OK"));
  delay(500);
  display.clear();
}
