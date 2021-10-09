//Global Variable Declarations
double integral = 0.0; //Variable storing current integral value utilized in the integral portion of the PI controller, in units of [radian/second]
double error = 0.0; //Variable storing error used in proportional component of the PI controller, in units of [radian]
double Kp = 4.84; //Proportional Gain of the PI controller, in units of [volts/radian]
double Ki = 0.2; //Integral Gain of the PI controller, in units of [volts*seconds/radian]

void loop(){
  error = (newPosition - theta); //calculates error based on difference between current and desired position
  if (abs(error) >= PI) {  //if error is off by pi, it is actually off by ~0
    error = 0;
  }
  if ((abs(error) < 0.05) && (abs(error) >= 0)) {
    integral = 0;
  }
  integral = error*(.005)+integral; //integral calculation uses 5 ms as time between each iteration
  int voltage_sign = 0; //intialize var that controls motor direction
  int voltage_send = 255*(Kp*error + Ki*integral)/7.2; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
  if (voltage_send < 0){ //if voltage_send is negative, go clockwise 
      voltage_sign = 0;
  } else { //else, go CCW
      voltage_sign = 1;
  }
  if (abs(voltage_send) > 255){ //rails max PWM value to 255
    voltage_send = 255;
  }
}
