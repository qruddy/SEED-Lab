#include <Encoder.h>
#include <Wire.h>

//wheel encoder declarations
Encoder left(2,11);
Encoder right(3,13);

//pin declarations
const int enablePin = 4;
const int directionPinL = 7;
const int directionPinR = 8;
const int motorPinL = 9;
const int motorPinR = 10;
const int startPin = 5; //this pin is used to indicate when to start tracking variables (when the switch is flipped on)

#define SLAVE_ADDRESS 0x04
double receive[32] = {};
void receiveData(int);

const double wheel_gap = 1; //distance between two wheels
//d = 5.875 in //gap in inches
const double r = 0.24479; //radius of each wheel in feet
//r = 2.9375; //radius in inches
bool ctrlFlag = false; //when tape is initially detected, this flag will be set true
bool stopFlag = false; //this flag triggers when the cross is detected
bool endFlag = false; //this flag triggers when the robot has reached its final position, prevents further movement

void setup() {
  //changing the baud rate affects the I2C communication severely, we found 19200 baud to be the most consistent
  Serial.begin(19200);
  Serial.println("Encoder start: ");
  pinMode(enablePin, HIGH);
  pinMode(motorPinL, INPUT);
  pinMode(motorPinR, INPUT);
  pinMode(startPin, INPUT);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Serial.println("Ready!");
}

//initial declarations for encoders
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
double rho_Kp = 0.25; 
double rho_Ki = 33.5; 
//angle controller variables
double phi = 0.0;
double phi_dot = 0;
double phi_error = 0.0;
double phi_dot_error = 0.0;
double phi_dot_integral = 0.0;
double phi_derivative = 0.0;
double phi_last_error = 0.0;

double phi_dot_Kp = 0.25;
double phi_dot_Ki = 0.75; 
double phi_Kp = 0.8; 
double phi_Kd = 0.25;

double phi_dot_d = 0.0;

double v_bar = 0;
double v_delta = 0;
double timeStart = 0;
int v_left = 0;
int v_right = 0;
double d_t = 0;
double rho_d = 0.35; //desired linear velocity for control scheme, in units of [feet/second]
double current_receive = 0;
double set_angle = 0.0;

void loop() {
  
  if (digitalRead(startPin) == HIGH) {
  
    int i = 0;
    digitalWrite(directionPinL, 1);
    digitalWrite(directionPinR, 1);
  
    timeStart = millis(); //used to track time spent in each iteration of loop
    
    //read encoder counts
    countL = left.read(); //read current number of counts for left wheel
    countR = right.read(); //read current number of counts for right wheel
    
    //convert counts to radians
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
    
    //---------------------------------velocity ctrl------------------------------------------
    rho_error = (rho_d - rho); //calculates error based on difference between current and desired position
    rho_integral = rho_error * (.070) + rho_integral; //integral calculation uses 5 ms as time between each iteration
    v_bar = (255 * (rho_Kp * rho_error + rho_Ki * rho_integral) / 7.0); //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
    if (abs(v_bar) > 255){ //rails max PWM value to 255
      v_bar = 255;
    } else if (abs(v_bar) < 12){ //rails min PWM value to 0
      v_bar = 0;
    }
    //------------------------------end of velocity ctrl---------------------------------------
    
    //------------------------------angle ctrl------------------------------------------
    phi = (r/wheel_gap) * (thetaL - thetaR);
    //phi error is no longer calculated, but is set based on the angle found by the pi camera
    phi_dot_d =  phi_Kp * phi_error + phi_Kd * (phi_error - phi_last_error) / (0.07); //calculate desired phi dot based on desired phi (angular position)
    phi_dot_error = (phi_dot_d - phi_dot); //calculates error based on difference between current and desired position
    phi_dot_integral = phi_dot_error*(0.07) + phi_dot_integral; //integral calculation uses 5 ms as time between each iteration

    //upper limit on phi integral just in case
    if (phi_dot_integral > 0.3) {
      phi_dot_integral = 0.3;
    } else if (phi_dot_integral < -0.3) {
      phi_dot_integral = -0.3;
    }
    
    //Serial.println("phi dot integral" + (String)phi_dot_integral);
    v_delta = (255 * (phi_dot_Kp * phi_dot_error + phi_dot_Ki * phi_dot_integral) / 7.0) + v_delta; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
    if (abs(v_delta) > 255){ //rails max PWM value to 255
      v_delta = 255;
    }

    //limit on rotational voltage value to keep robot within angles the camera can see
    if (v_delta > 75) {
      v_delta = 75;
    } if (v_delta < -75) {
      v_delta = -75;
    }
    //------------------------------end of angle ctrl------------------------------------------
    
    //calculate left wheel voltage
    v_left = (v_bar + v_delta) / 2.0;
    if (v_left > 255) {
      v_left = 255;
    }
    //calculate right wheel voltage
    v_right = 1.06 * (v_bar - v_delta) / 2.0; //the 1.125 multiplier exists because the right motor is slightly weaker than the left one
    if (v_right > 255) {
      v_right = 255;
    }
    
    if (stopFlag && !endFlag) { //triggers when cross is detected
      endTime();
    }
    if (endFlag) { //triggers when robot is over cross (stops all movement)
      v_left = 0;
      v_right = 0;
    }

    v_left = abs(v_left);
    v_right = abs(v_right);
    //write to the two motors using calculated voltages
    analogWrite(motorPinL, v_left);
    analogWrite(motorPinR, v_right);
    
    double d_t = millis() - timeStart; //time since iteration of loop began
    rho = 1000.0 * (r / 2.0) * ((deltaThetaR) + (deltaThetaL)) / d_t; //calculate rho, the current velocity
    phi_dot = 1000.0 * r * (deltaThetaL - deltaThetaR) / (wheel_gap * d_t); //calculate phi_dot, the rotational velocity
    phi_last_error = phi_error; //track previous error value
  }
}

void receiveData(int byteCount){
  int i = 0;
  receive[i] = {};
  while (Wire.available()) {
    receive[i] = Wire.read();
    if (receive[i] == 100) { //if tape is near bottom of fov, send stop flag
      stopFlag = true; //sets this flag if the camera detects the cross
      rho_d = 0;
      rho_error = 0;
      rho_integral = 0;
      phi_dot_d = 0;
      phi_dot_error = 0;
      phi_error = 0;
      phi_dot_integral = 0.0;
    } else {
      if (receive[i] > 200) { //if a negative angle is sent, it underflows from 256 and must be adjusted accordingly
        set_angle = receive[i] - 256;
      } else {
        set_angle = receive[i];
      }
      set_angle = set_angle * PI / 180.0; //set phi value
      phi_error = -1.0 * set_angle;
      if (stopFlag) { //overwrites previous calculation if cross is detected
        phi_error = 0;
        rho_d = 0;
      }
    }         
  } 
  ++i;  
}

//this function triggers when the cross is detected to give the robot the final push to reach the cross
//since the robot detects the cross ~2 ft away, this code makes the robot move forward another 2 ft
void endTime() { 
  analogWrite(motorPinL, 120);
  analogWrite(motorPinR, 130);
  delay(2000);
  analogWrite(motorPinL, 0);
  analogWrite(motorPinR, 0);
  endFlag = true;
}
