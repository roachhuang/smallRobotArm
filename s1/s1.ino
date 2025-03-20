// 0.00  0.01  0.46  3.14  2.04 -1.57

//#include <SoftwareSerial.h>
#define END_EFFECTOR_PIN 16
//driver for the axis 1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1

//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

// rest pose (power off) of the robot arm.
float curPos1 = 0.0;
float curPos2 = -78.51;
float curPos3 = 73.90;
float curPos4 = 0.0;
float curPos5 = -90.0;
float curPos6 = 0.0;
// float Ji[6] = { curPos1, curPos2, curPos3, curPos4, curPos5, curPos6 };  //{x, y, z, ZYZ Euler angles} home position

// Motor step direction flags
volatile bool dir1 = false;
volatile bool dir2 = false;
volatile bool dir3 = false;
volatile bool dir4 = false;
volatile bool dir5 = false;
volatile bool dir6 = false;

// Motor pulsing flags
volatile bool PULstat1 = false;
volatile bool PULstat2 = false;
volatile bool PULstat3 = false;
volatile bool PULstat4 = false;
volatile bool PULstat5 = false;
volatile bool PULstat6 = false;

//robot geometry
/*
360.0 / 200.0 / 32.0 calculates the degrees per microstep of the motor itself.
4.8: The division by 4.8 indicates that the motor's output is being further reduced by a gear system.
This means that for every 4.8 rotations of the motor, the output shaft of the gear system rotates once.
So, the 4.8 is a ratio. in essence, it calculates the degrees of rotation of the final output shaft per microstep, considering the gear reduction.
*/
// the motors are 200 steps /rev (50 teeth*4phase), microstepping=32
const double dl1 = 360.0 / 200.0 / 32.0 / 4.8;  // 4.8 is gear ratio
const double dl2 = 360.0 / 200.0 / 32.0 / 4.0;
const double dl3 = 360.0 / 200.0 / 32.0 / 5.0;
const double dl4 = 360.0 / 200.0 / 32.0 / 2.8;
const double dl5 = 360.0 / 200.0 / 32.0 / 2.1;
const double dl6 = 360.0 / 200.0 / 32.0 / 1.0;
// Target joint positions
volatile float targetPos1 = 0.0;
volatile float targetPos2 = 0.0;
volatile float targetPos3 = 0.0;
volatile float targetPos4 = 0.0;
volatile float targetPos5 = 0.0;
volatile float targetPos6 = 0.0;

// Stepping enable flag
volatile bool stepping = false;
volatile bool lastSteppingState = false;
// String dataIn = ""; //variable to store the bluetooth command
// int index = 0; //index corresonding to the robot position
// float Joint1[50], Joint2[50], Joint3[50], Joint4[50], Joint5[50], Joint6[50], MaxSpeed[50], InSpeed[50], FinSpeed[50];
const uint8_t ledPin = 13;  // the LED is connected to digital pin 13

float X[6] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
float velG = 0;

// float* splitStringToFloatArray(char* str);
const int MAX_INPUT_SIZE = 256;  // Adjust as needed
char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;

float* splitStringToFloatArray(char* str) {
  static float arr[6];
  int count = 0;
  char* token = strtok(str, ",");

  while (token != NULL && count < 6) {
    arr[count++] = atof(token);
    token = strtok(NULL, ",");
  }

  // If there were less than 6 values, fill the rest with 0.0
  while (count < 6) {
    arr[count++] = 0.0;
  }
  return arr;
}

#include <TimerOne.h>
void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(END_EFFECTOR_PIN, OUTPUT);
  digitalWrite(END_EFFECTOR_PIN, LOW);

  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);

  digitalWrite(PUL1_PIN, LOW);  // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW);  //LOW = negative direction

  digitalWrite(PUL2_PIN, LOW);  // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL3_PIN, LOW);  // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW);  //LOW = negative direction

  digitalWrite(PUL4_PIN, LOW);  // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL5_PIN, LOW);  // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL6_PIN, LOW);  // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW);  //LOW = positive direction

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH);

  Serial.begin(115200);
  //The Arduino Mega 2560 typically uses a 16 MHz clock.
  Timer1.initialize(150);  // 3 times higher than the pulse rate.
  Timer1.start();
  Timer1.attachInterrupt(ISR_routine);  
}

void loop() {
  // float velG = 0.25e-4;
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';
      inputIndex = 0;

      char val[MAX_INPUT_SIZE];
      strncpy(val, inputBuffer + 1, MAX_INPUT_SIZE - 1);  // skip header
      val[MAX_INPUT_SIZE - 1] = '\0';

      if (strncmp(inputBuffer, "m", 1) == 0) {
        float* j_over_time = splitStringToFloatArray(val);
        goTrajectory(j_over_time);
      } else if (strncmp(inputBuffer, "g", 1) == 0) {
        // The 'g' command is designed for incremental movements. Instead of specifying absolute joint angles, you specify how much each joint should move relative to its current position.
        float* targetAngles = splitStringToFloatArray(val);
        float from[6] = { curPos1, curPos2, curPos3, curPos4, curPos5, curPos6 };
        goStrightLine(from, targetAngles, 0.25e-4, 0.75e-10, 0.0, 0.0);
        // Serial.println("ack");

        velG = 0.25e-4;
      } else if (strcmp(inputBuffer, "eOn") == 0) {
        digitalWrite(END_EFFECTOR_PIN, HIGH);
      } else if (strcmp(inputBuffer, "eOff") == 0) {
        digitalWrite(END_EFFECTOR_PIN, LOW);

      } else if (strcmp(inputBuffer, "en") == 0) {
        digitalWrite(EN321_PIN, LOW);
        digitalWrite(EN4_PIN, LOW);
        digitalWrite(EN5_PIN, LOW);
        digitalWrite(EN6_PIN, LOW);
      } else if (strcmp(inputBuffer, "dis") == 0) {
        digitalWrite(EN321_PIN, HIGH);
        digitalWrite(EN4_PIN, HIGH);
        digitalWrite(EN5_PIN, HIGH);
        digitalWrite(EN6_PIN, HIGH);
      }
      //add else if statements for other commands here.
    } else if (inputIndex < MAX_INPUT_SIZE - 1) {
      inputBuffer[inputIndex++] = c;
    }
  }

  cli();
  bool isSteppingDone = (!stepping && lastSteppingState);
  lastSteppingState = stepping; 
  sei();
  if (isSteppingDone) {
    // curPos1 = targetAngles[0];
    // curPos2 = targetAngles[1];
    // curPos3 = targetAngles[2];
    // curPos4 = targetAngles[3];
    // curPos5 = targetAngles[4];
    // curPos6 = targetAngles[5];
    Serial.println("ack");
  }

  // Small delay to prevent serial communication interference
  delayMicroseconds(50);  // Adjust to a suitable value (e.g., 50 or 100)
}

void goStrightLine(float* xfi, float* xff, float vel0, float acc0, float velini, float velfin) {
  //
  float lmax = max(abs(xff[0] - xfi[0]), abs(xff[1] - xfi[1]));
  lmax = max(lmax, abs(xff[2] - xfi[2]));
  lmax = max(lmax, abs(xff[3] - xfi[3]));
  lmax = max(lmax, abs(xff[4] - xfi[4]));
  lmax = max(lmax, abs(xff[5] - xfi[5]));
  unsigned long preMil = micros();
  double l = 0.0;
  vel0 = min(vel0, sqrt(lmax * acc0 + 0.5 * velini * velini + 0.5 * velfin * velfin));
  unsigned long curMil = micros();
  unsigned long t = 0;
  double tap = vel0 / acc0 - velini / acc0;
  double lap = velini * tap + acc0 * tap * tap / 2.0;
  double lcsp = lmax - (vel0 * vel0 / 2.0 / acc0 - velfin * velfin / 2.0 / acc0);
  double tcsp = (lcsp - lap) / vel0 + tap;
  double tfin = vel0 / acc0 - velfin / acc0 + tcsp;
  while (curMil - preMil <= tfin) {
    t = curMil - preMil;
    //acceleration phase
    if (t <= tap) {
      l = velini * t + acc0 * t * t / 2.0;
    }
    //contant maximum speed phase
    if (t > tap && t <= tcsp) {
      l = lap + vel0 * (t - tap);
    }
    //deceleration phase
    if (t > tcsp) {
      l = lcsp + vel0 * (t - tcsp) - acc0 * (t - tcsp) * (t - tcsp) / 2.0;
    }

    //trajectory x and y as a function of l
    float Xx[6];
    Xx[0] = xfi[0] + (xff[0] - xfi[0]) / lmax * l;
    Xx[1] = xfi[1] + (xff[1] - xfi[1]) / lmax * l;
    Xx[2] = xfi[2] + (xff[2] - xfi[2]) / lmax * l;
    Xx[3] = xfi[3] + (xff[3] - xfi[3]) / lmax * l;
    Xx[4] = xfi[4] + (xff[4] - xfi[4]) / lmax * l;
    Xx[5] = xfi[5] + (xff[5] - xfi[5]) / lmax * l;

    goTrajectory(Xx);
    curMil = micros();
  }
}

void goTrajectory(float Jf[]) {
  targetPos1 = Jf[0];
  targetPos2 = Jf[1];
  targetPos3 = Jf[2];
  targetPos4 = Jf[3];
  targetPos5 = Jf[4];
  targetPos6 = Jf[5];

  dir1 = (targetPos1 > curPos1);
  dir2 = (targetPos2 > curPos2);
  dir3 = (targetPos3 > curPos3);
  dir4 = (targetPos4 > curPos4);
  dir5 = (targetPos5 > curPos5);
  dir6 = (targetPos6 > curPos6);
  // noInterrupts();  // Disable interrupts; no shared var are being modified inside the ISR during this fn.
  stepping = true;
  // interrupts();  // Re-enable interrupts
}

void ISR_routine() {
  if (stepping) {
    stepMotors();
  }
}

void stepMotors() {
  bool allMotorsStopped = true;

  // Joint 1
  if (abs(targetPos1 - curPos1) > dl1 / 2.0) {
    digitalWrite(DIR1_PIN, dir1 ? HIGH : LOW);
    digitalWrite(PUL1_PIN, PULstat1 ? LOW : HIGH);
    PULstat1 = !PULstat1;
    curPos1 += dir1 ? dl1 / 2.0 : -dl1 / 2.0;
    allMotorsStopped = false;
  }

  // Joint 2
  if (abs(targetPos2 - curPos2) > dl2 / 2.0) {
    digitalWrite(DIR2_PIN, dir2 ? HIGH : LOW);
    digitalWrite(PUL2_PIN, PULstat2 ? LOW : HIGH);
    PULstat2 = !PULstat2;
    curPos2 += dir2 ? dl2 / 2.0 : -dl2 / 2.0;
    allMotorsStopped = false;
  }

  // Joint 3
  if (abs(targetPos3 - curPos3) > dl3 / 2.0) {
    digitalWrite(DIR3_PIN, dir3 ? LOW : HIGH);  // Reversed direction
    digitalWrite(PUL3_PIN, PULstat3 ? LOW : HIGH);
    PULstat3 = !PULstat3;
    curPos3 += dir3 ? dl3 / 2.0 : -dl3 / 2.0;
    allMotorsStopped = false;
  }

  // Joint 4
  if (abs(targetPos4 - curPos4) > dl4 / 2.0) {
    digitalWrite(DIR4_PIN, dir4 ? HIGH : LOW);
    digitalWrite(PUL4_PIN, PULstat4 ? LOW : HIGH);
    PULstat4 = !PULstat4;
    curPos4 += dir4 ? dl4 / 2.0 : -dl4 / 2.0;
    allMotorsStopped = false;
  }

  // Joint 5
  if (abs(targetPos5 - curPos5) > dl5 / 2.0) {
    digitalWrite(DIR5_PIN, dir5 ? HIGH : LOW);
    digitalWrite(PUL5_PIN, PULstat5 ? LOW : HIGH);
    PULstat5 = !PULstat5;
    curPos5 += dir5 ? dl5 / 2.0 : -dl5 / 2.0;
    allMotorsStopped = false;
  }

  // Joint 6
  if (abs(targetPos6 - curPos6) > dl6 / 2.0) {
    digitalWrite(DIR6_PIN, dir6 ? HIGH : LOW);
    digitalWrite(PUL6_PIN, PULstat6 ? LOW : HIGH);
    PULstat6 = !PULstat6;
    curPos6 += dir6 ? dl6 / 2.0 : -dl6 / 2.0;
    allMotorsStopped = false;
  }

  if (allMotorsStopped) {
    stepping = false;
  }

  // OCR1A = 300; // micro second
}

/*
void stepMotors() {
  unsigned long currentTime = micros();
  if (currentTime >= nextStepTime) {
    bool allMotorsStopped = true;
    for (int i = 0; i < 6; i++) {
      if (motors[i].stepsRemaining > 0) {
        digitalWrite(motors[i].pulPin, HIGH);
        digitalWrite(motors[i].pulPin, LOW);
        motors[i].stepsRemaining--;
        motors[i].curPos += (digitalRead(motors[i].dirPin) == HIGH) ? motors[i].dl / 2.0 : -motors[i].dl / 2.0;
        allMotorsStopped = false;
      }
    }
    if (allMotorsStopped) {
      stepping = false;
    }
    nextStepTime = currentTime + 1000;  // Adjust for desired step frequency
  }
}
*/
