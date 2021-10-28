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

//velocity controller variables
double rho = 0;
double rho_integral = 0.0; //Variable storing current integral value utilized in the integral portion of the PI controller, in units of [feet/second^2]
double rho_error = 0.0; //Variable storing error used in proportional component of the PI controller, in units of [feet/second]
double rho_Kp = 0.0; //Proportional Gain of the PI controller, in units of [volts/radian] (value = 0.0)
double rho_Ki = 0.0; //Integral Gain of the PI controller, in units of [volts*seconds/radian] (value = 33.5)

//angle controller variables
double phi = 0;
double phi_dot = 0;
double phi_error = 0.0;
double phi_dot_error = 0.0;
double phi_dot_integral = 0.0;
double phi_derivative = 0.0;
double phi_last_error = 0.0;

double phi_dot_Kp = 0.362; //matlab simulation gives Kp = 0.362
double phi_dot_Ki = 0.4; //matlab simulation gives Ki = 11.489 ~ 11.5
double phi_Kp = 1;
double phi_Kd = 0.25;

double v_bar = 0;
double v_delta = 0;
double x = 0; //initial position is 0
double timeStart = 0;
double sample_rate = 46.0; //num milliseconds to wait before continuing
int v_left = 0;
int v_right = 0;
double d_t_2 = 0;
double rho_d = 3.0; //desired linear velocity for control scheme, in units of [feet/second]

//DESIRED VALUES:
double x_d = 0.0; //sample of value = 5.0
double phi_dot_d =  -1 * PI / 2; //sample value of -pi/2
double phi_d = 0.0;

void loop() {
  
  double d_t = millis() - timeStart;
  //x = x + rho * (d_t) / 1000.0;

  timeStart = millis();

  //rho_d = rho_d * (x_d - x) / x_d;

  if (x_d - x < 0.1) {
    rho_d = 0.75;
    rho_integral = 0.00;
  } else {
    rho_d = 3;
  }
  
  //Serial.println("X " + (String)x);
  
//---------------------------------velocity ctrl------------------------------------------
  //Serial.println("phi_dot_d: " + (String)phi_dot_d);
  rho_error = (rho_d - rho); //calculates error based on difference between current and desired position
  rho_integral = rho_error * (.070) + rho_integral; //integral calculation uses 5 ms as time between each iteration

  v_bar = (255 * (rho_Kp * rho_error + rho_Ki * rho_integral) / 7.5); //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
  Serial.println("phi_dot_error = " + (String)phi_dot_error);
  Serial.println("phi_dot_integral = " + (String)phi_dot_integral);
  if (abs(v_bar) > 255){ //rails max PWM value to 255
    v_bar = 255;
  } else if (abs(v_bar) < 12){
    v_bar = 0;
  }
//------------------------------end of velocity ctrl------------------------------------------

//------------------------------angle ctrl------------------------------------------
  phi_dot_error = (phi_dot_d - phi_dot); //calculates error based on difference between current and desired position
  phi_dot_integral = phi_dot_error*(0.07)+phi_dot_integral; //integral calculation uses 5 ms as time between each iteration

  if (abs(phi_dot_error) < 0.05){
    phi_dot_error = 0.00;
    phi_dot_integral = 0.00;
  }
  
  /*make distinction between phi dot and phi
  for regular phi, matlab gives these gain values:
  Kp = 3.5
  Kd = 0.25 (fudged both numbers, matlab's tuner was giving 0 Kd and a tiny Kp, so I adjusted to have simulated rise time of 1.3 seconds and steady state error of ~ 3 degrees)
  After a quick discussion w/ Dr. Coulston, he explained that derivative control can be implemented as Kd * (current error - previous error)/sample rate (the discrete calculation of an instantaneous rate of change)
  */

  /*prototype phi control
   
   */
  
  v_delta = (255 * (phi_dot_Kp * phi_dot_error + phi_dot_Ki * phi_dot_integral) / 7.5) + v_delta; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
  if (abs(v_delta) > 255){ //rails max PWM value to 255
    v_delta = 255;
  } else if (abs(v_delta) < 12){
    v_delta = 0;
  }
  Serial.println("v_delta " + (String)v_delta);
//------------------------------end of angle ctrl------------------------------------------

  digitalWrite(directionPinL, 1);
  digitalWrite(directionPinR, 1);


  //0 --> CW, 1--> CCW
  //0 = reverse, 1 = forward
  //to go left of the zero axis, set L = 0 & R = 1
  //to go right of the zero axis, set L = 1 & R = 0

  if (phi_dot_d > 0) {
    digitalWrite(directionPinL, 1);
    digitalWrite(directionPinR, 0);
  } else if (phi_dot_d < 0) {
    digitalWrite(directionPinL, 0);
    digitalWrite(directionPinR, 1);
  } else if (abs(phi_dot_error) < 0.05) {
    digitalWrite(directionPinL, 1);
    digitalWrite(directionPinR, 1);
  }


  //v_left is the voltage written to the motor controlling the left wheel
  //v_right is the voltage written to the motor controlling the right wheel
  
  v_left = (v_bar + v_delta) / 2.0;
  if (v_left > 255) {
    v_left = 255;
  }
  v_right = 1.1 * (v_bar - v_delta) / 2.0;
  if (v_right > 255) {
    v_right = 255;
  }

  analogWrite(motorPinL, abs(v_left));
  analogWrite(motorPinR, abs(v_right));

  countL = left.read(); //read current number of counts for left wheel
  countR = right.read(); //read current number of counts for right wheel
  
  if (countL != oldCountL) {
    oldCountL = countL;
    thetaL = PI * (double) countL / 1600; //current left wheel position in radians
  }
  deltaThetaL = thetaL - thetaL_old;
  thetaL_old = thetaL;

  if (countR != oldCountR) {
    oldCountR = countR;
    thetaR = -1.0 * PI * (double) countR / 1600; //current right wheel position in radians
  }
  deltaThetaR = thetaR - thetaR_old;
  thetaR_old = thetaR;

  Serial.println("DT_L: " + (String)deltaThetaL);
  Serial.println("DT_R: " + (String)deltaThetaR);

  //deltaThetaR = abs(deltaThetaR);
  //deltaThetaL = abs(deltaThetaL);

  //rho calc
  double d_t_2 = millis() - timeStart; //time since iteration of loop began
  rho = 1000.0 * (r/2.0) * ((deltaThetaR) + (deltaThetaL)) / d_t_2;
  phi_dot = 1000.0 * r * (deltaThetaL - deltaThetaR) / (wheel_gap * d_t_2);  
  Serial.println("phi_dot " + (String)phi_dot);

}
