#include <Encoder.h>

//declaration of wheels
Encoder left(2,11);
Encoder right(3,13);

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
  Serial.begin(115200);
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
//velocity controller variables
double rho = 0.0; //current velocity stored
double integral = 0.0; //Variable storing current integral value utilized in the integral portion of the PI controller, in units of [feet/second^2]
double error = 0.0; //Variable storing error used in proportional component of the PI controller, in units of [feet/second]
double rho_d = 1.0; //desired linear velocity for control scheme, in units of [feet/second]
double rho_Kp = 5; //Proportional Gain of the PI controller, in units of [volts/radian]
double rho_Ki = 0.1; //Integral Gain of the PI controller, in units of [volts*seconds/radian]
int sample_rate = 5 //num milliseconds to wait before continuing

void loop() {
  double timeStart = millis();

  error = (rho_d - rho); //calculates error based on difference between current and desired position
  integral = error*(.005)+integral; //integral calculation uses 5 ms as time between each iteration
  int voltage_send = 255*(Kp*error + Ki*integral)/7.2; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
  if (abs(voltage_send) > 255){ //rails max PWM value to 255
    voltage_send = 255;
  }

  analogWrite(motorPinL, voltage_send);
  analogWrite(motorPinR, voltage_send);

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

  //rho calc
  rho = (r/2) * (deltaThetaR + deltaThetaL) * sample_rate;
  
  prevPosition = currentPosition;
  thetaL_old = thetaL;
  thetaR_old = thetaR;
  
  while (millis() < sample_rate + timeStart){
        //do nothing
     }
}
