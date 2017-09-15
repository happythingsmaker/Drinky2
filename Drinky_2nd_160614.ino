#include <Servo.h>

// SERVO
#define SERVO_ARM     0  // D3
#define SERVO_Y       1  // D9      //P angle limit : 45~135
#define SERVO_P       2  // D10     //P angle limit : 90~120
#define SERVO_R       3  // D11     //R angle limit : 80~110

// HEAD LED
#define PWM_CHEEK_L   5   // D5
#define PWM_CHEEK_R   6   // D6
#define BRIGHT_L  0
#define BRIGHT_R  1

// SENSOR
#define ADC_HAND      0   // A0
#define ADC_PSD       7   // A7

#define THRESHOLD_HAND   700  // threshold value for glass. the more water is the less value is
#define THRESHOLD_PSD    500  // threshold value for glass. the shorter distance is the more value is

// ANGLE
#define ANGLE_SHOT_ARM   35   //      an angle value for ARM when it shots
#define ANGLE_SHOT_P    120   //110   an angle value for neck(pitch) when it shots

#define ANGLE_INIT_ARM  135   //      the angle value for arm  when it starts
#define ANGLE_INIT_Y  90      //     the angle value for neck Y when it starts 
#define ANGLE_INIT_P  100      //     the angle value for neck P when it starts 
#define ANGLE_INIT_R  90      //     the angle value for neck R when it starts 


#define ANGLE_MIN_Y  45
#define ANGLE_MIN_P  100
#define ANGLE_MIN_R  80
#define ANGLE_MAX_Y  135
#define ANGLE_MAX_P  130
#define ANGLE_MAX_R  120

// ETC
#define MAX_DRUNK_COUNT 5


//   initializing function ========================//
void initSensors();           // init sensors  (pull up)
void attachServos();            // attaching Servos to PWM pins
void detachServos(int servoNum);            // detaching Servos to PWM pins

//   testing function      ========================//
void initialTest();           //
void sensorToCheek();         // testing code

//  macro function

// opreating function      ========================//
void drinkAndRelease();
void intro();
void waitGlass();


// checking function       ========================//
bool checkGlass();
//  global variations             ========================//
Servo servo[4];  // create servo objects to control a servo    // twelve servo objects can be created on most boards
int currentBrightness = 0;
int currentAngle[4] = {ANGLE_INIT_ARM, ANGLE_INIT_Y, ANGLE_INIT_P, ANGLE_INIT_R};
//int pos = 0;      // variable to store the servo position

// ================================================//
//  SETUP                                          //
// ================================================//
void setup() {
  Serial.begin(9600);
  initSensors();
  attachServos();
  //initialTest();
}

// ================================================//
// LOOP                                            //
// ================================================//
int drunkCount = 0;
void loop() {
  intro();    //say hello to your friends
  while (1) {
    if (drunkCount < MAX_DRUNK_COUNT) {
      while (!checkGlass()) {
        waitGlass(drunkCount);    // waiting motion for pouring alchol into glass
      }
      reactGlass();    // reaction for thanks to pouring
      while (!checkCheers()) {
        waitCheers();    // waiting motion for cheers
      }
      DrinkAndRelease();         // drinking motion
      drunkCount ++;
      reactDrink();   // recation
    } else {
      actDrunk();          // drunk, dosen't drink  anymore
    }
  }

  /*
    FaceHeartbeat(7);
    FaceHeartbeat(6);
    FaceHeartbeat(5);
    FaceHeartbeat(4);
    FaceHeartbeat(3);
    FaceHeartbeat(2);
    FaceHeartbeat(1);
  */

  /*
    servo[SERVO_ARM].write(0);
    servo[SERVO_NECK_YAW].write(90);
    delay(10);
    //INITIAL degree test
  */
  /*
    servo[SERVO_NECK_YAW].write(90);
    delay(3000);
  */



  /*
    for (pos = 1000; pos <= 2000; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //myservo.write(pos);              // tell servo to go to position in variable 'pos'
    myservo.writeMicroseconds(pos);
    //delay(delayTime);                       // waits 15ms for the servo to reach the position
    delayMicroseconds(delayTime);


    }
    delay(500);

    for (pos = 2000; pos >= 1000; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.writeMicroseconds(pos);
    //Serial.println(myservo.read());
    //delay(delayTime);                       // waits 15ms for the servo to reach the position
    delayMicroseconds(delayTime);
    }
    delay(500);
  */
  //myservo.write(94);

  //delay(1000);
  /*
    myservo.detach();
    pinMode(9,OUTPUT);
    digitalWrite(9,LOW);
    delay(5000);
    myservo.attach(9);
  */
}

void initialTest() {
  // SERVO TEST
  const int delayTime = 500;

  servo[SERVO_ARM].write(90); delay(delayTime * 2); servo[SERVO_ARM].write(ANGLE_INIT_ARM); delay(delayTime);
  servo[SERVO_Y].write(45); delay(delayTime);   servo[SERVO_Y].write(ANGLE_INIT_Y); delay(delayTime);
  servo[SERVO_P].write(120); delay(delayTime);  servo[SERVO_P].write(ANGLE_INIT_P); delay(delayTime);
  servo[SERVO_R].write(120); delay(delayTime);  servo[SERVO_R].write(ANGLE_INIT_R); delay(delayTime);

  // LED TEST
  analogWrite(PWM_CHEEK_L, 255);
  analogWrite(PWM_CHEEK_R, 255);
  delay(delayTime);
  analogWrite(PWM_CHEEK_L, 0);
  analogWrite(PWM_CHEEK_R, 0);
  delay(delayTime);

  // SENSOR TEST
  int testTimes = 500;
  while (testTimes--) {
    int analogValueHand = analogRead(ADC_HAND);
    int analogValuePSD = analogRead(ADC_PSD);

    if (analogValueHand < THRESHOLD_HAND) {
      analogWrite(PWM_CHEEK_L, 255);
    } else {
      analogWrite(PWM_CHEEK_L, 0);
    }
    if (analogValuePSD  > THRESHOLD_PSD) {
      analogWrite(PWM_CHEEK_R, 255);
    } else {
      analogWrite(PWM_CHEEK_R, 0);
    }
    Serial.print("HAND = "); Serial.print(analogValueHand);
    Serial.print("\t PSD = "); Serial.println(analogValuePSD);
    delay(10);
  }
}

void initSensors() {
  digitalWrite(ADC_HAND, HIGH);  // Pull Up
}

void attachServos() {
  servo[SERVO_ARM].attach(3);   // D3 : ARM motor
  servo[SERVO_Y].attach(9);     // D9 : Yaw motor
  servo[SERVO_P].attach(10);    // D10 : Pitch motor
  servo[SERVO_R].attach(11);    // D11 : Roll motor
}

void detachServos(int servoNum) {
  servo[servoNum].detach();
}

void DrinkAndRelease() {
  changeFaceBright(255, 400);
  changeFaceBright(0, 400);
  changeFaceBright(255, 200);
  changeFaceBright(0, 200);
  changeFaceBright(255, 100);
  changeFaceBright(0, 100);

  servo[SERVO_ARM].attach(3);   // D3 : ARM motor
  servo[SERVO_ARM].write(ANGLE_SHOT_ARM);
  delay(600);
  servo[SERVO_P].write(ANGLE_SHOT_P);

  changeFaceBright(255, 3000);

  servo[SERVO_ARM].write(ANGLE_INIT_ARM);
  delay(300);
  servo[SERVO_P].write(ANGLE_INIT_P);
  delay(1000);

  servo[SERVO_ARM].detach();   // D3 : ARM motor
}

void FaceHeartbeat(int delayMS) {
  for (int i = 0 ; i < 255; i += 2) {
    analogWrite(PWM_CHEEK_L, i);
    analogWrite(PWM_CHEEK_R, i);
    delay(1);
  }
  delay(delayMS * 50);
  for (int i = 255 ; i > 0; i --) {
    analogWrite(PWM_CHEEK_L, i);
    analogWrite(PWM_CHEEK_R, i);
    delay(1);
  }
  delay(delayMS * 200);
}

void intro() {
  const int delayTime = 250;
  brinkFace(1, 0);
  servo[SERVO_R].write(80);
  delay(delayTime);

  brinkFace(0, 1);
  servo[SERVO_R].write(110);
  delay(delayTime);

  brinkFace(0, 0);
  servo[SERVO_R].write(ANGLE_INIT_R);
  servo[SERVO_P].write(120);
  delay(delayTime);

  brinkFace(1, 1);
  servo[SERVO_P].write(ANGLE_INIT_P);
  delay(delayTime);

  brinkFace(1, 0);
  servo[SERVO_Y].write(45);
  delay(delayTime);

  brinkFace(0, 1);
  servo[SERVO_Y].write(135);
  delay(delayTime);

  brinkFace(0, 0);
  servo[SERVO_Y].write(ANGLE_INIT_Y);
  servo[SERVO_ARM].write(90);
  delay(delayTime);

  brinkFace(1, 1);
  servo[SERVO_ARM].write(ANGLE_INIT_ARM);
  delay(delayTime);

  brinkFace(0, 0);
  servo[SERVO_ARM].write(90);
  delay(delayTime);

  brinkFace(1, 1);
  servo[SERVO_ARM].write(ANGLE_INIT_ARM);
  delay(delayTime);

}

void waitGlass(int drunkCount) {
  const int movingTime = (MAX_DRUNK_COUNT - drunkCount) * 400;
  detachServos(SERVO_ARM);

  cosMotionWithCheek(SERVO_P, 120, 200, movingTime);
  delay(movingTime / 2);
  cosMotionWithCheek(SERVO_P, 95, 0, movingTime);
  delay(movingTime / 2);
}

void cosMotionWithCheek(int servoNum, int goalAngle, int goalBrightness, int delayTime) {
  double outputPosition = map(currentAngle[servoNum], 0, 180, 650, 2350);
  double goalPosition = map(goalAngle, 0, 180, 650, 2350);

  double outputBrightness = currentBrightness;
  if (delayTime != 0) {
    for (int i = 1; i <= delayTime; i ++) {
      servo[servoNum].writeMicroseconds(outputPosition + ((goalPosition - outputPosition) * (cos(PI + PI * i / delayTime) + 1) / 2));
      analogWrite(PWM_CHEEK_L, (outputBrightness + (goalBrightness - currentBrightness) * (cos(PI + PI * i / delayTime) + 1) / 2));
      analogWrite(PWM_CHEEK_R, (outputBrightness + (goalBrightness - currentBrightness) * (cos(PI + PI * i / delayTime) + 1) / 2));
      //Serial.println(outputPosition + ((goalPosition - outputPosition) * (cos(PI + PI * i / delayTime) + 1) / 2));
      delay(1);
    }
  }
  //servo[servoNum].write(goalAngle);
  currentBrightness = goalBrightness;
  currentAngle[servoNum] = goalAngle;
}

void cosMotion(int servoNum, int goalAngle, int delayTime) {
  double outputPosition = map(currentAngle[servoNum], 0, 180, 650, 2350);
  double goalPosition = map(goalAngle, 0, 180, 650, 2350);
  if (delayTime != 0) {
    for (int i = 1; i <= delayTime; i ++) {
      servo[servoNum].writeMicroseconds(outputPosition + ((goalPosition - outputPosition) * (cos(PI + PI * i / delayTime) + 1) / 2));
      //Serial.println(outputPosition + ((goalPosition - outputPosition) * (cos(PI + PI * i / delayTime) + 1) / 2));
      delay(1);
    }
  }
  //servo[servoNum].write(goalAngle);
  currentAngle[servoNum] = goalAngle;
}

void smoothMotion(int servoNum, int goalAngle, int delayTime) {
  double outputPosition = map(currentAngle[servoNum], 0, 180, 650, 2350);
  double goalPosition = map(goalAngle, 0, 180, 650, 2350);

  if (delayTime != 0) {
    for (int i = 1; i <= delayTime; i ++) {

      servo[servoNum].writeMicroseconds(outputPosition + ((goalPosition - outputPosition) * i) / (double)delayTime);
      Serial.println(outputPosition + ((goalPosition - outputPosition) * i) / (double)delayTime);
      delay(1);
    }
  }
  servo[servoNum].write(goalAngle);
  currentAngle[servoNum] = goalAngle;
}

// it can be used for smooth
void changeFaceBright(int brightness, int delayTime) {
  double unitBrightness = 0;
  double outputBrightness = currentBrightness;
  unitBrightness  = ((double)brightness - currentBrightness) / (double)delayTime;

  if (delayTime != 0) {
    for ( int i = 1 ; i < delayTime ; i ++ ) {
      outputBrightness += unitBrightness ;
      analogWrite(PWM_CHEEK_L, outputBrightness);
      analogWrite(PWM_CHEEK_R, outputBrightness);
      delay(1);
    }
  }
  analogWrite(PWM_CHEEK_L, brightness);
  analogWrite(PWM_CHEEK_R, brightness);
  currentBrightness = brightness;           //eventually

}

void brinkFace(bool left, bool right) {
  if (left)
    analogWrite(PWM_CHEEK_L, 255);
  else
    analogWrite(PWM_CHEEK_L, 0);
  if (right)
    analogWrite(PWM_CHEEK_R, 255);
  else
    analogWrite(PWM_CHEEK_R, 0);
}

bool checkGlass() {
  int analogValueHand = 0;
  for (int i = 0; i < 20; i ++) {
    analogValueHand += analogRead(ADC_HAND);  // for average
  }
  analogValueHand /= 20;

  Serial.print("average ADC : ");
  Serial.println(analogValueHand);

  if (analogValueHand < THRESHOLD_HAND) {
    Serial.print("GLASS OK! THRESHOLD : ");
    Serial.print(THRESHOLD_HAND);
    Serial.print("\t SENSING VALUE : ");
    Serial.println(analogValueHand);
    return true;
  } else {
    return false;
  }
}

void reactGlass() {
  const int delayTime = 300;
  cosMotionWithCheek(SERVO_R, 80, 0, delayTime);
  cosMotionWithCheek(SERVO_R, 100, 64, delayTime);
  cosMotionWithCheek(SERVO_R, 80, 128, delayTime);
  cosMotionWithCheek(SERVO_R, 100, 196, delayTime);
  cosMotionWithCheek(SERVO_R, 80, 255, delayTime);
  cosMotionWithCheek(SERVO_R, 100, 196, delayTime);
  cosMotionWithCheek(SERVO_R, 80, 128, delayTime);
  cosMotionWithCheek(SERVO_R, 100, 64, delayTime);
  cosMotionWithCheek(SERVO_R, 90, 0, delayTime);
}

bool checkCheers() {
  int analogValuePSD = 0;
  static int countLimit = 10;

  if (--countLimit == 0) {
    countLimit = 10;
    return true;
  }

  for (int i = 0; i < 20; i ++) {
    analogValuePSD += analogRead(ADC_PSD);  // for average
  }
  analogValuePSD /= 20;

  Serial.print("average PSD : ");
  Serial.println(analogValuePSD);

  if (analogValuePSD > THRESHOLD_PSD) {
    Serial.print("CHEERS OK! THRESHOLD : ");
    Serial.print(THRESHOLD_PSD);
    Serial.print("\t SENSING VALUE : ");
    Serial.println(analogValuePSD);
    return true;
  } else {
    return false;
  }
}

void waitCheers() {
  cosMotionWithCheek(SERVO_P, 120, 0, 300);
  cosMotionWithCheek(SERVO_P, 95, 128, 100);
  delay(500);
}

void actDrunk() {
  const int minTime = 100;
  const int maxTime = 1000;

  detachServos(SERVO_ARM);

  cosMotionWithCheek(SERVO_P, random(ANGLE_MIN_P, ANGLE_MAX_P), random(0, 255), random(minTime, maxTime));
  delay(random(minTime, maxTime));
  cosMotionWithCheek(SERVO_Y, random(ANGLE_MIN_Y, ANGLE_MAX_Y), random(0, 255), random(minTime, maxTime));
  delay(random(minTime, maxTime));
  cosMotionWithCheek(SERVO_R, random(ANGLE_MIN_R, ANGLE_MAX_R), random(0, 255), random(minTime, maxTime));
  delay(random(minTime, maxTime));

}

void reactDrink() {
  const int delayTime = 150;
  cosMotionWithCheek(SERVO_Y, 45, 255,  delayTime);
  cosMotionWithCheek(SERVO_Y, 135, 125, delayTime);
  cosMotionWithCheek(SERVO_Y, ANGLE_INIT_Y, 0, delayTime);

  cosMotionWithCheek(SERVO_R, 80, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_R, 110, 0, delayTime);   //right
  cosMotionWithCheek(SERVO_R, 80, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_R, 110, 0, delayTime);   //right
  cosMotionWithCheek(SERVO_R, 80, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_R, 110, 0, delayTime);   //right
  cosMotionWithCheek(SERVO_R, ANGLE_INIT_R, 0, delayTime);   //right

  cosMotionWithCheek(SERVO_P, 120, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_P, ANGLE_INIT_P, 0, delayTime); // left


  cosMotionWithCheek(SERVO_Y, 45, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_Y, 135, 125, delayTime);   //right
  cosMotionWithCheek(SERVO_Y, ANGLE_INIT_Y, 0, delayTime);   //right

  cosMotionWithCheek(SERVO_R, 80, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_R, 110, 0, delayTime);   //right
  cosMotionWithCheek(SERVO_R, ANGLE_INIT_R, 0, delayTime);   //right

  cosMotionWithCheek(SERVO_P, 120, 255,  delayTime); // left
  cosMotionWithCheek(SERVO_P, ANGLE_INIT_P, 0, delayTime); // left

  servo[SERVO_Y].write(ANGLE_INIT_Y);
  servo[SERVO_P].write(ANGLE_INIT_P);
  servo[SERVO_R].write(ANGLE_INIT_R);

}
