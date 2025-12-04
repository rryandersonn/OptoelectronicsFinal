//Ryan Anderson - Optoelectronics Final Project
//This code finds the brightest light using 6 photoresistors and rotates a stepper motor about a center point, pointing towards the most intense light source
#include <Arduino.h>     //include Arduino (platform.io integration)
#include <WiFi.h>        //include WiFi to disable for ADC2 Pins
#include <Preferences.h> //include preferences for remembering intial position (flash memory)

Preferences prefs;       //initialize preferences

int IN1 = 14; //Define stepper motor pins (coil 1)
int IN2 = 27; //(coil 2)
int IN3 = 26; //(coil 3)
int IN4 = 25; //(coil 4)

int stepIndex = 0;
const int stepsPerRev = 4096; //define total ticks
const float degPerStep = 360.0 / 4096.0; //define degrees per step = 0.08789
const int stepDelay = 1; //set delay (no delay will cause instability)

int stepsArr[8][4] = {   //8-step half-step sequence
  {1, 0, 0, 0},          //step 1
  {1, 1, 0, 0},          //step 2
  {0, 1, 0, 0},          //step 3
  {0, 1, 1, 0},          //step 4
  {0, 0, 1, 0},          //step 5
  {0, 0, 1, 1},          //step 6
  {0, 0, 0, 1},          //step 7
  {1, 0, 0, 1}           //step 8
};

//PHOTORESISTOR SENSOR PINS (6 sensors)
int LDRpins[6] = {36, 39, 34, 35, 32, 33};              //VP, VN, 34, 35, 32, 33 
float sensorAnglesDeg[6] = {0, 60, 120, 180, 240, 300}; //Degrees per sensor - each element matches the elements in LDRpins

int brightThreshold = 150; //edit this to find idea Threshold

//STEPPER FUNCTIONS
void setStep(int a, int b, int c, int d) {
  digitalWrite(IN1, a);    //step 1
  digitalWrite(IN2, b);    //step 2
  digitalWrite(IN3, c);    //step 3
  digitalWrite(IN4, d);    //step 4
}

void stepMotor(bool dir) {
  if (dir) stepIndex++;    //step counterclockwise
  else     stepIndex--;    //step clockwise

  if (stepIndex > 7) stepIndex = 0; //if greater than 7 restart step index to repeat
  if (stepIndex < 0) stepIndex = 7; //if lesser than 0 restart step index to repeat

  setStep(
    stepsArr[stepIndex][0],         
    stepsArr[stepIndex][1],
    stepsArr[stepIndex][2],
    stepsArr[stepIndex][3]
  );
  delay(stepDelay);
}

void rotateSteps(int steps) {
  bool dir = (steps > 0);
  steps = abs(steps);
  for (int i = 0; i < steps; i++) stepMotor(dir);
}

// TRACKED MOTOR ANGLE
float currentAngle = 0;   // will be overwritten by NVS on startup to maintain position

// SETUP
void setup() {
  Serial.begin(115200);   //baud set to 115200

  WiFi.mode(WIFI_OFF);    //disable WiFi to allow ADC2 pins to work
  WiFi.disconnect(true);  //disable WiFi

  pinMode(IN1, OUTPUT);   // Stepper outputs (coil 1)
  pinMode(IN2, OUTPUT);   //(coil 2)
  pinMode(IN3, OUTPUT);   //(coil 3)
  pinMode(IN4, OUTPUT);   //(coil 4)

  for (int i = 0; i < 6; i++) {
    pinMode(LDRpins[i], INPUT); //set inputs for photoresistors
  }

  Serial.println("6-Sensor Light Tracker Starting..."); //print to serial monitor once setup is completed

  prefs.begin("tracker", false);
  float savedAngle = prefs.getFloat("angle", 0.0);      //Load last saved angle and return to 0 degrees
  prefs.end();                                          //close flash memory

  Serial.print("Saved angle = "); 
  Serial.println(savedAngle);                           //print the saved angle

  int stepsToZero = -(savedAngle / degPerStep);         // convert angle to steps back to zero
  rotateSteps(stepsToZero);

  currentAngle = 0;                                     // now at zero
  Serial.println("Returned to zero position.");
}

//ENTER LOOP
void loop() {

  int L[6]; //define array of all photresistors

  for (int i = 0; i < 6; i++) {
    L[i] = analogRead(LDRpins[i]);                  //Read all 6 sensors
  }

  bool allDim = true;                               //Check if light is too dim
  for (int i = 0; i < 6; i++) {
    if (L[i] >= brightThreshold) {
      allDim = false;
      break;
    }
  }

  //enter if allDim is true
  if (allDim) {
    Serial.print("Low light - holding   ");
    for (int i = 0; i < 6; i++) {
      Serial.print("L"); Serial.print(i);
      Serial.print("="); Serial.print(L[i]); Serial.print(" ");
    }
    Serial.println();
    delay(10);
    return;
  }

  float x = 0, y = 0; //Weighted vector sum
  for (int i = 0; i < 6; i++) {
    float angleRad = sensorAnglesDeg[i] * DEG_TO_RAD;
    x += L[i] * cos(angleRad);
    y += L[i] * sin(angleRad);
  }

  float targetAngle = atan2(y, x) * RAD_TO_DEG;   //translate to degrees from magnitude 
  if (targetAngle < 0) targetAngle += 360;        //(360 for full hat)

  float error = targetAngle - currentAngle;       //Motor remember error

  if (error > 180) error -= 360;                  //shortest wrap-around
  if (error < -180) error += 360;

  int stepsToMove = error / degPerStep;           //control system to fix error

  if (abs(stepsToMove) > 2) {                     //move to desired location if error is greater then 2/360%
    rotateSteps(stepsToMove);
    currentAngle = targetAngle;                  

    prefs.begin("tracker", false);
    prefs.putFloat("angle", currentAngle);        //Save current angle to flash
    prefs.end();
  }

  //DEBUG OUTPUT
  for (int i = 0; i < 6; i++) {
    Serial.print("L"); Serial.print(i);
    Serial.print("="); Serial.print(L[i]);
    Serial.print("  ");
  }

  Serial.print(" TargetAngle=");
  Serial.println(targetAngle);                    //print target angle -> moves to this
  delay(10);                                      //delay for stability
}
