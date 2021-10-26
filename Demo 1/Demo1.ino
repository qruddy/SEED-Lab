#include <Encoder.h>

//declaration of wheels
Encoder left(2,13);
Encoder right(3,14);

//pin declarations
const int enablePin = 4;
const int directionPinL = 7;
const int directionPinR = 8;
const int motorPinL = 9;
const int motorPinR = 10;

const double wheel_gap = 1; //distance between two wheels
//d = 5.875 in
//const double r = 2.9375; //radius of each wheel in inches
const double r = 0.24479; //radius of each wheel in feet

double final_position = 1;
double final_angle = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Encoder start: ");
  pinMode(enablePin, HIGH);
  pinMode(motorPinL, INPUT);
  pinMode(motorPinR, INPUT);
}

long oldCountL = -999;
long oldCountR = -999;
double countL = 0;
double countR = 0;
double thetaL = 0;
double thetaL_old = 0;
double thetaR_old = 0;
double thetaR = 0;
double deltaThetaL = 0;
double deltaThetaR = 0;
double prevPosition = 0;
double currentPosition = 0;
int wheelPWM = 100;

void loop() {

  analogWrite(motorPinL, wheelPWM);
  analogWrite(motorPinR, wheelPWM);

  //0 --> CW, 1--> CCW
  //0 = reverse, 1 = forward
  digitalWrite(directionPinL, 1);
  digitalWrite(directionPinR, 1);
  
  countL = left.read(); //read current number of counts for left wheel
  countR = right.read(); //read current number of counts for right wheel
  
  if (countL != oldCountL) {
    oldCountL = countL;
    thetaL = 9 * (double) countL / 80; //current left wheel position in degrees
    deltaThetaL = thetaL - thetaL_old;
  }

  if (countR != oldCountR) {
    oldCountR = countR;
    thetaR = 9 * (double) countR / 80; //current left wheel position in degrees
    deltaThetaR = thetaL - thetaL_old;
  }

  currentPosition = prevPosition + deltaThetaL * r;

  if (currentPosition >= final_position) { //if final position is reached, turn the motors off
    analogWrite(motorPinL, 0);
    analogWrite(motorPinR, 0);
  }

  prevPosition = currentPosition;
  thetaL_old = thetaL;
  thetaR_old = thetaR;
}
