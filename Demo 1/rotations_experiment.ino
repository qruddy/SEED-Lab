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
const double d = 5.875/12.000;
//const double r = 2.9375; //radius of each wheel in inches
const double r = 0.24479; //radius of each wheel in feet

const int sample_rate = 5; //fixed sample rate in ms

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
double ang_velocL = 0;
double ang_velocR = 0;
double phi = 0;
int test_start = 100; //how many ms to wait until start experimentation
int test_end = 1100; //time in ms to stop the experiment at
const int test_motor_voltage = 0.5; //voltage that needs to be applied to each motor in order to drive step response of Vbar = 1 (Vbar = V1 + V2)
int test_pwm = test_motor_voltage * (255/7.2); //voltages for each motor converted to pwm in order to achieve step response of Vbar = 1

void loop() {
  
  double timeStart = millis();

  if (timeStart >= test_start && timeStart <= test_end){ //if we are within experimental time range
    //drive motors
     digitalWrite(directionPinL, 0); //drive left motor backwards
     digitalWrite(directionPinR, 0); //drive firght motor forwards
     analogWrite(motorPinL, test_pwm); //drive both motors at test voltages
     analogWrite(motorPinR, test_pwm); //~

     //read from encoders
     countL = left.read(); //read current number of counts for left wheel
     countR = right.read(); //read current number of counts for right wheel
     if (countL != oldCountL) {
        oldCountL = countL;
        thetaL = PI * (double) countL / 1600; //current left wheel position in degrees
        deltaThetaL = thetaL - thetaL_old;
     }
     if (countR != oldCountR) {
        oldCountR = countR;
        thetaR = PI * (double) countR / 1600; //current left wheel position in degrees
        deltaThetaR = thetaR - thetaR_old;
     }

     //calculate desired values
     ang_velocL = (deltaThetaL)/sample_rate;
     ang_velocR = (deltaThetaR)/sample_rate;
     phi = r*(ang_velocL - ang_velocR)/d; //calculate current forward velocity to capture and sure things seem right


     //update measurement variables
     thetaL_old = thetaL;
     thetaR_old = thetaR;

     //print out values to be analyzed
     Serial.println("thetadot 1 (R motor):" + (String)ang_velocR); //print right wheel angular velocity
     Serial.println("thetadot 2 (L motor):" + (String)ang_velocL); //print left wheel angular velocity
     Serial.println("phi dot:" + (String)phi); //print overall rotational velocity

     while (millis() < sample_rate + timeStart){
        //do nothing
     }
     
  } else { //if we are outside of experimental time range
    analogWrite(motorPinL, 0); //set left motor to stop
    analogWrite(motorPinR, 0); //set right motor to stop
  }
}
