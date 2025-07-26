#ifndef DEFINE_H_
#define DEFINE_H_

// bool waitingAck;
bool buf_full=false;

#define BAUD_RATE 115200
// Command buffer definitions
#define BUFFER_SIZE 200
float targetBuffer[BUFFER_SIZE][6];
volatile int writeIndex = 0;
volatile int readIndex = 0;
// === Macros ===
#define BUFFER_EMPTY()      (readIndex == writeIndex)
#define BUFFER_FULL()       (((writeIndex + 1) % BUFFER_SIZE) == readIndex)
#define BUFFER_INCREMENT(idx)  ((idx + 1) % BUFFER_SIZE)

const uint8_t END_EFFECTOR_PIN = 16;
const uint8_t ledPin = 13;  // the LED is connected to digital pin 13
#define PI 3.14159          //26535897932384626433832795
// Pin definitions
const uint8_t PUL_PINS[6] = { 39, 43, 47, 46, A6, A0 };
const uint8_t DIR_PINS[6] = { 37, 41, 45, 48, A7, A1 };
// enable pin for the axis 3, 2 and 1
const uint8_t EN_PINS[4] = { 32, A8, A2, 38 };  // EN321, EN4, EN5, EN6
const float GEAR_RATIOS[6] = { 4.8, 4.0, 5.0, 2.8, 2.1, 1.0 };
// const float MICROSTEPS = 32;
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
// steps/sec. For different joints (if you want joint-specific limits)
const float MAX_JOINT_SPEEDS[6] = {
  1500,  // Joint 1 (base) - can handle higher speeds. ≈27°/sec for joint 1
  1200,  // Joint 2 (shoulder) - heavier load
  1200,  // Joint 3 (elbow) - heavier load  
  1800,  // Joint 4 (wrist roll) - lighter
  1800,  // Joint 5 (wrist pitch) - lighter
  2000   // Joint 6 (wrist yaw) - lightest
};
// steps/sec²
const float MAX_JOINT_ACCELS[6] = {
  800,   // Joint 1
  600,   // Joint 2 - slower accel for heavy loads
  600,   // Joint 3 - slower accel for heavy loads
  1000,  // Joint 4
  1000,  // Joint 5  
  1200   // Joint 6
};

// Stepper motor objects
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[3], DIR_PINS[3]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[4], DIR_PINS[4]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[5], DIR_PINS[5])
};

// Current positions in degrees
float currentPositions[6] = { 0.0, -78.51, 73.90, 0.0, -90.0, 0.0 };
const float homePositions[6] = { 0.0, -78.51, 73.90, 0.0, -90.0, 0.0 };
// float currentPositions[6] = { 0.0, -1.37, 1.2898, 0.0, -1.5708, 0.0 }; in radians
// float jointSpaceCmd[6];  // in angle 
char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;
bool newCommandReady = false;
const uint8_t step_threshold = 32; // Threshold for considering movement complete

// Function declarations
void processCommand(const char *cmd);
long degreesToSteps(float degrees, int joint);
float stepsToRadians(long steps, int joint);
void enableMotors(bool enable);
void serialEvent();
void sendCurrentPositions();
void parseFloats(const char *str, float *out, int maxCount);
int getBackoffDirection(uint8_t joint_num);
void move_j2_up();
void homeJoint(uint8_t limitPin, uint8_t joint_num);
bool isWithinLimits(float angle, int joint);
void moveToPositionSync(float targetDegrees[6]);

#endif  // DEFINE_H_
