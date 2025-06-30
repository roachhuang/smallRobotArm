#ifndef DEFINE_H_
#define DEFINE_H_

bool waitingAck;
bool allDone;
const uint8_t END_EFFECTOR_PIN = 16;
const uint8_t ledPin = 13;  // the LED is connected to digital pin 13
#define PI 3.14159          //26535897932384626433832795
// Pin definitions
const uint8_t PUL_PINS[6] = { 39, 43, 47, 46, A6, A0 };
const uint8_t DIR_PINS[6] = { 37, 41, 45, 48, A7, A1 };
// enable pin for the axis 3, 2 and 1
const uint8_t EN_PINS[4] = { 32, A8, A2, 38 };  // EN321, EN4, EN5, EN6
const float GEAR_RATIOS[6] = { 4.8, 4.0, 5.0, 2.8, 2.1, 1.0 };
const float MICROSTEPS = 32;
// Must match hardware config, Conversion factors(degrees per step)
const double DEG_PER_STEP[6] = {
  360.0 / 200.0 / 16.0 / 4.8,
  360.0 / 200.0 / 16.0 / 4.0,
  360.0 / 200.0 / 8.0 / 5.0,  // lower microstep produces more torque
  360.0 / 200.0 / 32.0 / 2.8,
  360.0 / 200.0 / 32.0 / 2.1,
  360.0 / 200.0 / 32.0 / 1.0
};

// Serial communication variables
const int MAX_INPUT_SIZE = 100;

#define X_MIN_PIN 3
#define X_MAX_PIN 2
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15
#define Z_MIN_PIN 18
#define Z_MAX_PIN 19  // J2 limit switch

const int lower_limit[6] = { -114, -81, -180, -180, -139, -180 };
const int upper_limit[6] = { 114, 77, 70, 180, 139, 180 };
const float MAX_SPEED = 1000.0;
#endif  // DEFINE_H_
