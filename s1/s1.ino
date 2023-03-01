// 0.00  0.01  0.46  3.14  2.04 -1.57

// #include <math.h>
//#include <SoftwareSerial.h>

//#define PI 3.1415926535897932384626433832795

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
/*
  double curPos1 = 0.0;
  double curPos2 = 0.0;
  double curPos3 = 0.0;
  double curPos4 = 0.0;
  double curPos5 = 0.0;
  double curPos6 = 0.0;
*/
// rest pose (power off) of the robot arm.
float curPos1 = 0.0;
float curPos2 = -78.51;
float curPos3 = 73.90;
float curPos4 = 0.0;
float curPos5 = -90.0;
float curPos6 = 0.0;
float Ji[6] = {curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles} home position

boolean PULstat1 = 0;
boolean PULstat2 = 0;
boolean PULstat3 = 0;
boolean PULstat4 = 0;
boolean PULstat5 = 0;
boolean PULstat6 = 0;

//robot geometry
// the motors are 200 steps /rev (50 teeth*4phase), microstepping=32
const double dl1 = 360.0 / 200.0 / 32.0 / 4.8;  // 4.8 is gear ratio
const double dl2 = 360.0 / 200.0 / 32.0 / 4.0;
const double dl3 = 360.0 / 200.0 / 32.0 / 5.0;
const double dl4 = 360.0 / 200.0 / 32.0 / 2.8;
const double dl5 = 360.0 / 200.0 / 32.0 / 2.1;
const double dl6 = 360.0 / 200.0 / 32.0 / 1.0;

// String dataIn = ""; //variable to store the bluetooth command
// int index = 0; //index corresonding to the robot position
// float Joint1[50], Joint2[50], Joint3[50], Joint4[50], Joint5[50], Joint6[50], MaxSpeed[50], InSpeed[50], FinSpeed[50];
const uint8_t ledPin = 13; // the LED is connected to digital pin 13

float X[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float velG = 0;
/*
  int i = 0;
  int j = 0;
  int k = 0;
  float J[4][7];  // col 0: timing
  float V[5][6];  // plus head and tail
  float A[5][6];  // plus head and tail
*/

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin , HIGH );

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

  digitalWrite(PUL1_PIN, LOW); // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW); //LOW = negative direction

  digitalWrite(PUL2_PIN, LOW); // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW); //LOW = positive direction

  digitalWrite(PUL3_PIN, LOW); // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW); //LOW = negative direction

  digitalWrite(PUL4_PIN, LOW); // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW); //LOW = positive direction

  digitalWrite(PUL5_PIN, LOW); // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW); //LOW = positive direction

  digitalWrite(PUL6_PIN, LOW); // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW); //LOW = positive direction

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH);

  Serial.begin(115200);
  /*
    goHome();
    curPos1 = 0.0;  //
    curPos2 = 0.0;
    curPos3 = 0.0;
    curPos4 = 0.0;
    curPos5 = 90.0;
    curPos6 = 0.0;
  */
}

void loop() {
  String val;
  // float velG = 0.25e-4;

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("j")) {
      digitalWrite(ledPin, LOW);
      // Serial.println("data == \"jn\"");
      val = data.substring(1, data.length());
      splitString(val, X);      // update array X
      // goStrightLine(Ji, X, 0.25e-4, 0.75e-10, 0.5 * velG, 0.5 * velG);
      goStrightLine(Ji, X, 0.15e-4, 0.35e-10, 0, 0 );
      Serial.println("ack");
      // des, src, size
      memcpy(Ji, X, sizeof(X));
      velG = 0.25e-4;
    }
    else if (data.startsWith("m")) {
      digitalWrite(ledPin, LOW);
      val = data.substring(1, data.length());
      splitString(val, X);
      goTrajectory(X);
      // Serial.println("ack m");
    }
    else if (data.startsWith("g")) {
      val = data.substring(1, data.length());
      splitString(val, X);
      float Jinitial[6] = {curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6] = {curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      for (int i = 0; i < sizeof(X); i++) {
        Jfinal[i] = (X[i] != 0) ? X[i] : Jfinal[i];
      }
      goStrightLine(Jinitial, Jfinal, 0.25e-4, 0.75e-10, 0.0, 0.0);
    }
    /*
      else if (data.startsWith("J")) {
      val = data.substring(1, data.length());
      splitString(val, X);      // update array X
      Serial.println("ack");
      // des, src, size
      memcpy(&J[i][0], X, sizeof(X));
      i++;
      }
      else if (data.startsWith("V")) {
      val = data.substring(1, data.length());
      splitString(val, X);      // update array X
      Serial.println("ack");
      // des, src, size
      memcpy(&V[j][0], X, sizeof(X));
      j++;
      //Serial.println(j);
      }
      else if (data.startsWith("A")) {
      val = data.substring(1, data.length());
      splitString(val, X);      // update array X
      Serial.println("ack");
      // des, src, size
      memcpy(&A[k][0], X, sizeof(X));
      k++;
      //Serial.println(k);
      }
      else if (data.startsWith("T")) {
      i = j = k = 0;
      // traj();
      }
    */
    else if (data == "en") {
      digitalWrite(EN321_PIN, LOW);
      digitalWrite(EN4_PIN, LOW);
      digitalWrite(EN5_PIN, LOW);
      digitalWrite(EN6_PIN, LOW);
    }
    else if (data == "dis") {
      digitalWrite(EN321_PIN, HIGH);
      digitalWrite(EN4_PIN, HIGH);
      digitalWrite(EN5_PIN, HIGH);
      digitalWrite(EN6_PIN, HIGH);
    }
    else if (data == "rst") {      
      curPos1 = 0.0;
      curPos2 = -78.51;
      curPos3 = 73.90;
      curPos4 = 0.0;
      curPos5 = -90.0;
      curPos6 = 0.0;      
      float curPos[6] = {curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      memcpy(Ji, curPos, sizeof(Ji));
    }
  }
  // delay(10);
}

void splitString(String val, float arr[]) {
  char *p = strtok((char*)val.c_str(), ","); // Use strtok to split the String into individual values
  //while(p!=null && i < 6)
  for (int i = 0; i < 6; i++) {
    X[i] = atof(p);
    // Serial.println(X[i]);
    // with a NULL pointer as the first argument will return the next token, and so on until there are no more tokens
    p = strtok(NULL, ",");
  }
  /*
    int StringCount = 0;
    // Split the string into substrings
    while (str.length() > 0)
    {
    int index = str.indexOf('\t');
    if (index == -1) // No space found
    {
      arr[StringCount++] = str.toFloat();
      break;
    }
    else
    {
      arr[StringCount++] = str.substring(0, index).toFloat();
      str = str.substring(index + 1);
    }
    }
    // return X;
  */
}

void goStrightLine(float * xfi, float * xff, float vel0, float acc0, float velini, float velfin) {
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

// this function takes care of gear ratios, we just give degrees.
void goTrajectory(float Jf[]) {
  const int delF = 2;

  //execution
  // joint #1
  if (Jf[0] - curPos1 > 0.0) {
    // positive direction of rotation - CCW
    digitalWrite(DIR1_PIN, HIGH);
    while (Jf[0] - curPos1 > dl1 / 2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 + dl1 / 2.0;
      if (Jf[0] - curPos1 > dl1 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    // neg direction roatiaon - CW rotation.
    digitalWrite(DIR1_PIN, LOW);
    while (-Jf[0] + curPos1 > dl1 / 2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 - dl1 / 2.0;
      if (-Jf[0] + curPos1 > dl1 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #2
  if (Jf[1] - curPos2 > 0.0) { // positive direction of rotation
    digitalWrite(DIR2_PIN, HIGH);
    while (Jf[1] - curPos2 > dl2 / 2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 + dl2 / 2.0;
      if (Jf[1] - curPos2 > dl2 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR2_PIN, LOW);
    while (-Jf[1] + curPos2 > dl2 / 2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 - dl2 / 2.0;
      if (-Jf[1] + curPos2 > dl2 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #3
  if (Jf[2] - curPos3 > 0.0) { // positive direction of rotation
    digitalWrite(DIR3_PIN, LOW);
    while (Jf[2] - curPos3 > dl3 / 2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 + dl3 / 2.0;
      if (Jf[2] - curPos3 > dl3 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR3_PIN, HIGH);
    while (-Jf[2] + curPos3 > dl3 / 2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 - dl3 / 2.0;
      if (-Jf[2] + curPos3 > dl3 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #4
  if (Jf[3] - curPos4 > 0.0) { // positive direction of rotation
    digitalWrite(DIR4_PIN, HIGH);
    while (Jf[3] - curPos4 > dl4 / 2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 + dl4 / 2.0;
      if (Jf[3] - curPos4 > dl4 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR4_PIN, LOW);
    while (-Jf[3] + curPos4 > dl4 / 2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 - dl4 / 2.0;
      if (-Jf[3] + curPos4 > dl4 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #5
  if (Jf[4] - curPos5 > 0.0) { // positive direction of rotation
    digitalWrite(DIR5_PIN, HIGH);
    while (Jf[4] - curPos5 > dl5 / 2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 + dl5 / 2.0;
      if (Jf[4] - curPos5 > dl5 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR5_PIN, LOW);
    while (-Jf[4] + curPos5 > dl5 / 2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 - dl5 / 2.0;
      if (-Jf[4] + curPos5 > dl5 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #6
  if (Jf[5] - curPos6 > 0.0) { // positive direction of rotation
    digitalWrite(DIR6_PIN, HIGH);
    while (Jf[5] - curPos6 > dl6 / 2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 + dl6 / 2.0;
      if (Jf[5] - curPos6 > dl6 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR6_PIN, LOW);
    while (-Jf[5] + curPos6 > dl6 / 2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 - dl6 / 2.0;
      if (-Jf[5] + curPos6 > dl6 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
}

/*
  const int totalPoints = 4;
  const int ts[4] = {0, 8, 16, 20};
  // t = elapsed time; i = joint_num
  float eq1(double t, int i) {
  double dt = t - 0;
  return V[0][i] * dt + 1 / 2 * A[0][i] * dt * dt;
  }
  float eq2(double t, int i) {
  return V[1][i] * (t - 0.25);
  }

  float eq3(float t, int i) {
  float dt1, dt2;
  dt1 = t - 0.25;
  dt2 = t - (ts[1] - 0.25);
  return V[1][i] * dt1 + 1 / 2 * A[1][i] * dt2 * dt2;
  }
  float eq4(double t, int i) {
  float dt = t - ts[1];
  return V[2][i] * dt;
  }
  float eq5(float t, int i) {
  float dt1, dt2;
  dt1 = t - ts[1];
  dt2 = t - (ts[2] - 0.25);
  return V[2][i] * dt1 + 1 / 2 * A[2][i] * dt2 * dt2;
  }
  float eq6(float t, int i) {
  float dt = t - ts[2];
  return V[3][i] * dt;
  }
  float eq7(float t, int i) {
  float dt1, dt2;
  dt1 = t - ts[2];
  dt2 = t - (ts[totalPoints - 1] - 0.5);
  return V[3][i] * dt1 + 1 / 2 * A[3][i] * dt2 * dt2;
  }

  void traj() {
  float Xx[6];
  double t;
  uint32_t start_time = millis();
  uint32_t curr_time = millis();
  while (millis() - start_time < 12 * 1000) {
    t = (double)(curr_time - start_time) / 1000.0;
    // elasped_time = millis() - start_time;
    Serial.println(t);
    /*
      for (int i = 0; i < 6; i++) {
      if (t >= ts[0] && t <= (ts[0] + 0.5))
        Xx[i] = J[0][i] + eq1(t, i);
      else if (t > (ts[0] + 0.5) && t <= (ts[1] - 0.25))
        Xx[i] = J[0][i] + eq2(t, i);
      else if (t > (ts[1] - 0.25) && t <= (ts[1] + 0.25))
        Xx[i] = J[0][i] + eq3(t, i);
      else if (t > (ts[1] + 0.25) && t <= (ts[2] - 0.25))
        Xx[i] = J[1][i] + eq4(t, i);
      else if ( t > (ts[2] - 0.25) && t <= (ts[2] + 0.25))
        Xx[i] = J[1][i] + eq5(t, i);
      else if (t > (ts[2] + 0.25) && t <= (ts[totalPoints - 1] - 0.5))
        Xx[i] = J[2][i] + eq6(t, i);
      else if ( t > (ts[totalPoints - 1] - 0.5) && t <= ts[totalPoints - 1])
        Xx[i] = J[2][i] + eq7(t, i);
      }


    //char buf[40];
    //sprintf(buf,"Xx=%.2f,%.2f,%.2f", Xx[0],Xx[1],Xx[2]);
    //Serial.print(Xx[0]);
    //Serial.print(Xx[1]);
    //Serial.println(Xx[2]);
    //goTrajectory(Xx);
    curr_time = millis();
  }
  }

  void goHome() {
  // go to the home position (all joints equal to 0)
  // joint #2
  digitalWrite(DIR2_PIN, HIGH);
  int delValue = 4000; // delay value after pulse
  int incValue = 7; //
  int accRate = 530; // accel in 1st 530 steps and deccel in last 530 steps, and constant vel in btw.
  // 1.8 deg / 32 microstep = 0.05625deg/step, 2791*2 * 0.05625 = 314.8 deg, 314.8 /dl2 -> 314.8/4=78.7 degrees (actual degrees moved)
  int totSteps = 2791 * 2;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) { // does the totSteps big enough for doing half accel and half deccel
      if (i < accRate) {
        //acceleration in step 0 and 530
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration in last 530 (accRate) steps
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      // do half acc and half deccel, no constant vel.
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL2_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL2_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #3 (note that join3 and joint2's motor are in opposite direction.
  digitalWrite(DIR3_PIN, HIGH);
  delValue = 4000;
  incValue = 7;
  accRate = 530;
  totSteps = 6569;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL3_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL3_PIN, LOW);
    delayMicroseconds(delValue);
  }
  // joint #5
  digitalWrite(DIR5_PIN, HIGH);
  delValue = 4000;
  incValue = 7;
  accRate = 530;
  totSteps = 90 / dl5;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL5_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL5_PIN, LOW);
    delayMicroseconds(delValue);
  }
  }
*/
