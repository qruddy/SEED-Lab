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
double phi_d = 0;

#define SLAVE_ADDRESS 0x04
double receive[32] = {};
void receiveData(int);


const double wheel_gap = 1; //distance between two wheels
//d = 5.875 in //gap in inches
const double r = 0.24479; //radius of each wheel in feet
//r = 2.9375; //radius in inches
bool ctrlFlag = false; //when tape is initially detected, this flag will be set true

void setup() {
  Serial.begin(9600);
  Serial.println("Encoder start: ");
  pinMode(enablePin, HIGH);
  pinMode(motorPinL, INPUT);
  pinMode(motorPinR, INPUT);
  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(receiveData);

  Serial.println("Ready!");
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
double rho_Kp = 0.34; //Proportional Gain of the PI controller, in units of [volts/radian] (value = 0.0)
double rho_Ki = 33.5; //Integral Gain of the PI controller, in units of [volts*seconds/radian] (value = 33.5)

//angle controller variables
double phi = 0.0;
double phi_dot = 0;
double phi_error = 0.0;
double phi_dot_error = 0.0;
double phi_dot_integral = 0.0;
double phi_derivative = 0.0;
double phi_last_error = 0.0;

double phi_dot_Kp = 0.26; //matlab simulation gives Kp = 0.362
double phi_dot_Ki = 0.34; //matlab simulation gives Ki = 11.489 ~ 11.5
double phi_Kp = 0.35; //matlab give Kp = 3.5
double phi_Kd = 0.25; //matlab give Kd = 0.25

double phi_dot_d = 0.0;
double x_d = 0.0;

double v_bar = 0;
double v_delta = 0;
double x = 0; //initial position is 0
double timeStart = 0;
int v_left = 0;
int v_right = 0;
double d_t_2 = 0;
double rho_d = 2.0; //desired linear velocity for control scheme, in units of [feet/second]


void loop() {
  Serial.println("ctrlflag " + (String)ctrlFlag);
  int i = 0;
  //spin in a circle until tape detected
  if (!ctrlFlag) { //rotate in a circle
    digitalWrite(directionPinL, 1);
    digitalWrite(directionPinR, 0);

    analogWrite(motorPinL, 100);
    analogWrite(motorPinR, 100);
  } else { //once blue tape is detected, transition to control system

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
    
    
    //controls which direction the wheels spin
    Serial.println("phi_error " + (String)phi_error);
    Serial.println("phi d " + (String)phi_d);
    
    digitalWrite(directionPinL, 1);
    digitalWrite(directionPinR, 1);
  
  
  //---------------------------------velocity ctrl------------------------------------------
    rho_error = (rho_d - rho); //calculates error based on difference between current and desired position
    rho_integral = rho_error * (.070) + rho_integral; //integral calculation uses 5 ms as time between each iteration
    v_bar = (255 * (rho_Kp * rho_error + rho_Ki * rho_integral) / 7.5); //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
    if (abs(v_bar) > 255){ //rails max PWM value to 255
      v_bar = 255;
    } else if (abs(v_bar) < 12){ //rails min PWM value to 0
      v_bar = 0;
    }
  //------------------------------end of velocity ctrl------------------------------------------
  
  //------------------------------angle ctrl------------------------------------------
    phi = (r/wheel_gap) * (thetaL - thetaR);
    //Serial.println(phi);
    phi_error = phi_d - phi; //phi d is determined by the intial angle given by raspberry pi camera, and should not change
    phi_dot_d =  phi_Kp * phi_error + phi_Kd * (phi_error - phi_last_error) / (0.07); //calculate desired phi dot based on desired phi (angular position)
    phi_dot_error = (phi_dot_d - phi_dot); //calculates error based on difference between current and desired position
    phi_dot_integral = phi_dot_error*(0.07)+phi_dot_integral; //integral calculation uses 5 ms as time between each iteration
    v_delta = (255 * (phi_dot_Kp * phi_dot_error + phi_dot_Ki * phi_dot_integral) / 7.5) + v_delta; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
    if (abs(v_delta) > 255){ //rails max PWM value to 255
      v_delta = 255;
    } else if (abs(v_delta) < 12){
      v_delta = 0;
    }

  //------------------------------end of angle ctrl------------------------------------------
  
  
    //calculate left wheel voltage
    v_left = (v_bar + v_delta) / 2.0;
    if (v_left > 255) {
      v_left = 255;
    }
    //calculate right wheel voltage
    v_right = 1.125 * (v_bar - v_delta) / 2.0; //the 1.125 multiplier exists because the right motor is slightly weaker than the left one
    if (v_right > 255) {
      v_right = 255;
    }
  
    //write to the two motors using calculated voltages
      if((v_left > 0) && ( v_left < 40) && ((x_d - x) > 0.05)) {
        v_left = v_left + 40;
      }
      if((v_right > 0) && (v_right < 40) && ((x_d - x) > 0.05)) {
        v_right = v_right + 40;
      }

    double d_t_2 = millis() - timeStart; //time since iteration of loop began
    rho = 1000.0 * (r/2.0) * ((deltaThetaR) + (deltaThetaL)) / d_t_2; //calculate rho, the current position
    phi_dot = 1000.0 * r * (deltaThetaL - deltaThetaR) / (wheel_gap * d_t_2); //calculate phi_dot, the rotational velocity
    phi_last_error = phi_error; //track previous error value
  
  } //end of else statement
  //phi_d = angle between robot's current position and tape
  //reset robot's angle to zero
  //for trial 1, move forward 4 ft, which should place the robot near or above the tape
  //for trial 2, move until the camera no longer detects blue tape
}

void receiveData(int byteCount){
  int i = 0;
  receive[i] = {};
  while (Wire.available()){
    receive[i] = Wire.read();
    if (receive[i] == 50){
      ctrlFlag = true;
    } else{
    
        if (receive[i] > 200) {
          phi_d = receive[i] - 256;
        } else {
          phi_d = receive[i];
        }
    }
    ++i;
  }
}
