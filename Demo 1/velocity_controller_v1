//Global Variable Declarations
double integral = 0.0; //Variable storing current integral value utilized in the integral portion of the PI controller, in units of [radian/second]
double error = 0.0; //Variable storing error used in proportional component of the PI controller, in units of [radian]
double rho_Kp = 5; //Proportional Gain of the PI controller, in units of [volts/radian]
double rho_Ki = 0.1; //Integral Gain of the PI controller, in units of [volts*seconds/radian]


void loop(){
  error = (rho_d - rho); //calculates error based on difference between current and desired position
  integral = error*(.005)+integral; //integral calculation uses 5 ms as time between each iteration
  int voltage_send = 255*(Kp*error + Ki*integral)/7.2; //implementing the tuned controller gains in the control system and then multiplyling the final result by a ratio to convert from volts to PWM counts.
  if (abs(voltage_send) > 255){ //rails max PWM value to 255
    voltage_send = 255;
  }
}
