#include <Encoder.h> //library used for encoder
#include <Wire.h> //allows communication between arduino & pi

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
int recieve[32] = {};
void receiveData(int);

Encoder motorEnc(2, 8); //creates encoder object using pin 2 as A output and pin 8 as B

const int resetPin = 3; //pin for the switch that can be used to reset the wheel to 0
const int directionPin = 7; //pin used to control direction of motor
const int motorPin = 9; //pin used to control magnitude of motor speed using PWM
int resetFlag = 0; //flag used by reset interrupt, gets set to 1 if switch is pressed
float theta;
double diff = 0;
double newPosition = 0; //desired input

//control parameters & variables
double integral = 0.0;
double error = 0.0;
double Kp = 4.84;
double Ki = 0.2;

void setup() {
  Serial.begin(115200); //set baud rate
  Serial.println("Encoder start: "); 
  pinMode(motorPin, INPUT); //pin 9
  pinMode(directionPin, INPUT); //pin 7
  attachInterrupt(digitalPinToInterrupt(resetPin), reset, RISING); //define reset interrupt
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  Serial.println("Ready!");
}

double oldPosition  = -999; //initialize prev counter value for encoder
double currentPosition = 0; //intialize counter for encoder

void loop() {
  int timestart = millis(); //read time the loop begins


 //Controller code:
  error = (newPosition - theta); //calculates error based on difference between current and desired position
  if (abs(error) >= PI) {  //if error is off by pi, it is actually off by ~0
    error = 0;
  }
  if ((abs(error) < 0.05) && (abs(error) >= 0)) {
    integral = 0;
  }
  integral = error*(.005)+integral; //integral calculation uses 5 ms as time between each iteration
  int voltage_sign = 0; //intialize var that controls motor direction
  int voltage_send = 255*(Kp*error + Ki*integral)/7.2; //calculates PWM value sent to controller, as a ratio of 255 (max value) to 7.2 (voltage input)
  if (voltage_send < 0){ //if voltage_send is negative, go clockwise 
      voltage_sign = 0;
  } else { //else, go CCW
      voltage_sign = 1;
  }
  if (abs(voltage_send) > 255){ //rails max PWM value to 255
    voltage_send = 255;
  }

  if (resetFlag == 0) { //if flag = 0, code runs normally
    
    currentPosition = motorEnc.read(); //reads encoder counts value
    analogWrite(motorPin, abs(voltage_send)); //writes motor based on current PWM value
    digitalWrite(directionPin, voltage_sign); //tells motor which direction to rotate
  
    if (currentPosition != oldPosition) { //updates encoder counts if position changed
      oldPosition = currentPosition;
  
      //one full rotation of the motor shaft is equivalent to 3200 counts, which is 64 times the gear ratio (50:1)
      theta = 2 * PI * (double) currentPosition / 3200.00; //converts counts to radians
  
      while ((theta > PI) || (theta < -1 * PI)) { //restricts theta to range of -pi/2 < theta < pi/2
          if (theta > PI) {
          theta = theta - 2 * PI;
        } else if (theta < -1 * PI) {
          theta = theta + 2 * PI;
        }
      }
      
      //Serial.println((String)theta + "  " + (String)millis());
    }
  } else { //if flag = 1, motor doesn't run
    analogWrite(motorPin, 0);
  }
  while( (double) (millis() - timestart)/ (double) 1000 < 0.005){
    
  }
}

void reset() { //if switch is pressed, flip reset flag
  resetFlag = !resetFlag;
  error = 0; //resets error if switch is pressed so wheel doesn't keep moving
}

//code that reads in the pi's output and outputs corresponding desired angle
void receiveData(int byteCount) {
  int i = 0;
  while (Wire.available()) {
    recieve[i] = Wire.read()*2;
    if (recieve[i] == 0){
      newPosition = 0;
    } else if (recieve[i] == 90){
      newPosition = PI / 2;
    } else if(recieve[i] == 180){
      newPosition = PI;
    } else if(recieve[i] == 270){
      newPosition = -1 * PI / 2;
    }
  }
}
