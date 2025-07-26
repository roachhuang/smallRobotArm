#include <AccelStepper.h>
#include "define.h"

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ;  // Wait for serial port to connect, needed for native USB
  }

  buf_full = false;

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  // Enable pins for stepper drivers

  pinMode(EN_PINS[0], OUTPUT);  // EN321
  pinMode(EN_PINS[1], OUTPUT);  // EN4
  pinMode(EN_PINS[2], OUTPUT);  // EN5
  pinMode(EN_PINS[3], OUTPUT);  // EN6

  // Set pin modes for limit switches (INPUT_PULLUP for active-low switches)
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);

  steppers[2].setPinsInverted(true, false, false);  // Invert direction pin for joint3
  enableMotors(true);

  delay(1000);

  // Configure steppers
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(MAX_JOINT_SPEEDS[i]);      // steps/sec
    steppers[i].setAcceleration(MAX_JOINT_ACCELS[i]);  // steps/sec²
  }

  // Set initial positions
  for (int i = 0; i < 6; i++) {
    steppers[i].setCurrentPosition(degreesToSteps(homePositions[i], i));
  }
}

void loop() {
  // Handle serial communication
  serialEvent();
  if (newCommandReady) {
    processCommand(inputBuffer);
    newCommandReady = false;
  }

  // Run all steppers FIRST
  for (int i = 0; i < 6; i++) {
    steppers[i].run();
  }
  // Then Check if all steppers have reached their targets
  bool allDone = true;
  for (int i = 0; i < 6; i++) {
    if (steppers[i].distanceToGo() > step_threshold) {
      allDone = false;
      break;
    }
  }

  // If current movement is done and there are commands in the buffer, set next target
  if (allDone && !BUFFER_EMPTY()) {
    float jointSpaceCmd[6];  // in angle
    memcpy(jointSpaceCmd, targetBuffer[readIndex], sizeof(float) * 6);
    readIndex = BUFFER_INCREMENT(readIndex);
    moveToPositionSync(jointSpaceCmd);
    if (buf_full == true) {
      Serial.println("ack");
      buf_full = false;
    }
  }

  // Periodically send current positions (5Hz)
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 200) {  // 5Hz update
    // sendCurrentPositions();
    lastSend = millis();
  }
}

void processCommand(const char *cmd) {
  if (cmd[0] == 'g') {
    float jointSpaceCmd[6];
    // Check if buffer has space
    if (BUFFER_FULL()) {
      Serial.println("buffer_full");
      buf_full = true;
      return;
    }
    parseFloats(cmd + 1, jointSpaceCmd, 6);

    memcpy(targetBuffer[writeIndex], jointSpaceCmd, sizeof(float) * 6);
    writeIndex = BUFFER_INCREMENT(writeIndex);
    buf_full = false;
    Serial.println("ack");

  } else if (strcmp(cmd, "g28") == 0) {
    move_j2_up();
    homeJoint(Y_MAX_PIN, 2);  // Home joint 3
    homeJoint(Z_MAX_PIN, 1);  // Home joint 2
    Serial.println("ack");
  } else if (strcmp(cmd, "eOn") == 0) {
    digitalWrite(END_EFFECTOR_PIN, HIGH);
    Serial.println("ack");
  } else if (strcmp(cmd, "eOff") == 0) {
    digitalWrite(END_EFFECTOR_PIN, LOW);
    Serial.println("ack");
  } else if (strcmp(cmd, "en") == 0) {
    enableMotors(true);
    Serial.println("ack");
  } else if (strcmp(cmd, "dis") == 0) {
    enableMotors(false);
    Serial.println("ack");
  }
}

/*
// Improved command parsing using strtok
void processCommand(const char *cmd) {
    char commandType = cmd[0];
    char* rest = (char*)(cmd + 1);
    
    if (commandType == 'G') {
        if (strcmp(rest, "28") == 0) {
            // Homing sequence
        }
        else {
            // Parse G-code with coordinates
            char* token = strtok(rest, " ");
            float targets[6] = {0};
            int idx = 0;
            
            while (token != NULL && idx < 6) {
                if (token[0] == 'X') targets[0] = atof(token+1);
                if (token[0] == 'Y') targets[1] = atof(token+1);
                if (token[0] == 'Z') targets[2] = atof(token+1);
                if (token[0] == 'A') targets[3] = atof(token+1);
                if (token[0] == 'B') targets[4] = atof(token+1);
                if (token[0] == 'C') targets[5] = atof(token+1);
                token = strtok(NULL, " ");
                idx++;
            }
            // Add to buffer
        }
    }
    // Other commands...
}
*/

// Convert degrees to steps for a specific joint
long degreesToSteps(float degrees, int joint) {
  return round(degrees / DEG_PER_STEP[joint]);
}
// Convert steps to radians for a specific joint
float stepsToRadians(long steps, int joint) {
  return steps * DEG_PER_STEP[joint] * PI / 180.0;
}

void enableMotors(bool enable) {
  uint8_t val = enable ? LOW : HIGH;
  digitalWrite(EN_PINS[0], val);  // EN321
  digitalWrite(EN_PINS[1], val);  // EN4
  digitalWrite(EN_PINS[2], val);  // EN5
  digitalWrite(EN_PINS[3], val);  // EN6
}

void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';
      inputIndex = 0;
      newCommandReady = true;
      return;
    } else if (inputIndex < MAX_INPUT_SIZE - 1) {
      inputBuffer[inputIndex++] = c;
    }
  }
}

void sendCurrentPositions() {
  String data = "f";
  for (int i = 0; i < 6; i++) {
    currentPositions[i] = stepsToRadians(steppers[i].currentPosition(), i);
    data += String(currentPositions[i], 2);
    if (i < 5) data += ",";
  }

  uint8_t checksum = 0;
  for (size_t i = 0; i < data.length(); ++i) {
    checksum += data[i];
  }
  checksum %= 256;
  Serial.print(data);
  Serial.print("*");
  Serial.println(checksum);
}

void parseFloats(const char *str, float *out, int maxCount) {
  int idx = 0;
  bool negative = false;
  float value = 0.0f;
  float scale = 1.0f;
  bool decimal = false;
  while (*str && idx < maxCount) {
    char c = *str++;
    if (c == '-') {
      negative = true;
    } else if (c >= '0' && c <= '9') {
      value = value * 10.0f + (c - '0');
      if (decimal) scale *= 10.0f;
    } else if (c == '.') {
      decimal = true;
    } else if (c == ',') {
      out[idx++] = (negative ? -1 : 1) * (value / scale);
      value = 0.0f;
      scale = 1.0f;
      negative = false;
      decimal = false;
    }
  }

  if (idx < maxCount) {
    out[idx++] = (negative ? -1 : 1) * (value / scale);
  }

  while (idx < maxCount) {
    out[idx++] = 0.0f;
  }
}

int getBackoffDirection(uint8_t joint_num) {
  if (joint_num == 1) return 1;   // Joint 2: +ve direction (ccw)
  if (joint_num == 2) return -1;  // Joint 3
  return 1;                       // Default
}

void move_j2_up() {
  AccelStepper &stepper = steppers[1];  // Joint 2
  stepper.setAcceleration(500);
  stepper.setMaxSpeed(800);
  long step_offset = 400;  // ~tune based on gear ratio
  stepper.moveTo(stepper.currentPosition() + step_offset);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void homeJoint(uint8_t limitPin, uint8_t joint_num) {
  if (joint_num >= 6) return;
  int dir = getBackoffDirection(joint_num);
  int homeSpeed = 800;
  AccelStepper &stepper = steppers[joint_num];
  stepper.setAcceleration(500);
  bool triggered = joint_num == 2 ? LOW : HIGH;
  // Back off if switch is triggered
  if (digitalRead(limitPin) == triggered) {
    stepper.setSpeed(homeSpeed * dir);
    unsigned long start = millis();
    while (millis() - start < 600) {
      stepper.runSpeed();
    }
    delay(100);
  }

  // Move toward switch
  stepper.setSpeed(-homeSpeed / 4 * dir);
  unsigned long start = millis();
  while (digitalRead(limitPin) == !triggered && millis() - start < 10000) {
    stepper.runSpeed();
  }
  stepper.stop();
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void moveToPositionSync(float targetDegrees[6]) {
  long targetSteps[6], deltaSteps[6];
  long maxDelta = 0;
  // Compute deltas
  for (int i = 0; i < 6; i++) {
    targetSteps[i] = degreesToSteps(targetDegrees[i], i);
    deltaSteps[i] = abs(targetSteps[i] - steppers[i].currentPosition());
    if (deltaSteps[i] > maxDelta) maxDelta = deltaSteps[i];
  }

  // Scale speeds based on longest distance
  for (int i = 0; i < 6; i++) {
    float speedRatio = maxDelta == 0 ? 0 : (float)deltaSteps[i] / maxDelta;
    float scaledSpeed = MAX_JOINT_SPEEDS[i] * speedRatio;
    steppers[i].setMaxSpeed(scaledSpeed);
    steppers[i].setAcceleration(MAX_JOINT_ACCELS[i] * speedRatio);
    steppers[i].moveTo(targetSteps[i]);
  }
}
