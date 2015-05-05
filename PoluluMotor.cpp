/************************************************************************************************************
 * PoluluMotor.cpp - Arduino library for interfacing Polulu metal gearmotors with/without encoders.         *
 *																											*
 * Copyright 2015 Abhishek N. Kulkarni (abhi.bp1993@gmail.com)		                                        *
 * The library is available at https://github.com/abhibp1993/PoluluMotor/									*
 * Last update: 03.05.2015                                                                                  *
 ************************************************************************************************************
 
 ************************************************************************************************************
 * This program is free software: you can redistribute it and/or modify										*
 * it under the terms of the GNU General Public License as published by										*
 * the Free Software Foundation, either version 3 of the License, or										*
 * (at your option) any later version.																		*
 *																											*
 * This program is distributed in the hope that it will be useful,											*
 * but WITHOUT ANY WARRANTY; without even the implied warranty of											*
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the											*
 * GNU General Public License for more details.																*
 *																											*
 * You should have received a copy of the GNU General Public License										*
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.			                        *
 ***********************************************************************************************************/
 

#include "PoluluMotor.h"


PololuWheelEncoders encoders;

int PoluluMotor::instances = 0;
int PoluluMotor::m1_encA = 255;
int PoluluMotor::m1_encB = 255;
int PoluluMotor::m2_encA = 255;
int PoluluMotor::m2_encB = 255;


/* [HELPER]**********************************************************************
Function: map
Parameters: 
  1. x: parameter whose value is to be mapped
  2. in_min: from value (lower end)
  2. in_max: from value (upper end)
  2. out_min: to value (lower end)
  2. out_min: to value (upper end)

Description:
  The function overloads the internal 'map' function which maps
  only integer values for floats and doubles.
  
**********************************************************************************/
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* [HELPER]**********************************************************************
Function: setPWMFrequency 
Parameters: 
  1. pin: PWM pin on which the frequency is to be set. 
  2. divisor: prescaling factor to set the frequency.

Description:
  The function changes the frequency on pwm pin. 

  $$ Currently only pin 9, 10 frequency can be changed. $$
  $$ Changing frequency on either pin changes the pwm-frequency on other also $$
**********************************************************************************/
void setPWMFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}



/* [HELPER]**********************************************************************
Function: getDivisor 
Parameters: 
  1. duty: desired duty cycle on the pwm pin. [Value in range 0-1]

Description:
  The function returns an appropriate PWM-frequency so as to deliver enough current
  on pwm pin to drive the motor. (a good choice in case of high torque motors)
  
  $$ The slabs are chosen according to Polulu 25D metal gearmotor 6V/6A $$
  $$ Appropriate slabs might be chosen according to motor to be interfaced $$
**********************************************************************************/
int getDivisor(double duty){
  if       (duty < 0.08) {}
  else if  (duty >= 0.08  && duty < 0.30)   { return 1024;}
  else if  (duty >= 0.30  && duty < 0.50)   { return 256 ;}  
  else if  (duty >= 0.50  && duty < 1.00)   { return 64  ;}
  else if  (duty >= 1.00)                   { return 64  ;}
}


/* [HELPER]**********************************************************************
Function: setPWM
Parameters: 
  1. pin_pwm: the pwm pin on which the duty cycle is to be changed/set
  2. duty: the value of duty cycle in range 0 to 1.

Description:
  The function internally adjusts the pwm frequency and generates the pwm 
  according to given duty cycle. (The variable frequency drive is enabled
  if the VAR_FREQ_ENABLE is set to 'true')
  
  $$ Currently the function works only for pin_pwm = 9 AND pin_pwm = 10. $$ 
  
**********************************************************************************/
void setPWM(uint8_t pin_pwm, double duty){
  
  if (VAR_FREQ_ENABLE){
	  static int div9, div10;
	  
	  /* Get the required divisor for given duty */
	  int reqdDivisor = getDivisor(duty);
	  
	  
	  /* Set the appropriate frequency for each pin */
	  if (pin_pwm == 9) {  div9 = reqdDivisor; }
	  if (pin_pwm == 10){ div10 = reqdDivisor; }
	  
	  
	  /* Choose minimum of required frequencies of both motors */
	  reqdDivisor = max(div9, div10);
	  
	  
	  /* Set appropriate frequency of PWM */
	  setPWMFrequency(pin_pwm, reqdDivisor);
	}	  
  
  /* Set PWM duty */
  analogWrite(pin_pwm, (uint8_t)(duty * 255));
  
}


/* [HELPER]**********************************************************************
Function: PIDCompute
Parameters: 
	1. Input: feedback input received
	2. Setpoint: The desired setpoint value
	3. Kp: Proportional gain
	4. Ki: Integral gain
	5. Kd: Derivative gain

Description:
  Applies PID Correction to input according to setpoint and returns a corrected
  output value.
  
  $$ The internal values to be stored for different motors are not handled yet. $$
  
**********************************************************************************/
double outMax = 1.0, outMin = 0.0;
double PIDCompute(double input, double setpoint, double kp, double ki, double kd, double* lastInput, double* ITerm)
{   
	/*Compute all the working error variables*/
	double error = setpoint - input;

	*(ITerm) += (ki * error);
	if(*ITerm > outMax) *ITerm= outMax;
	else if(*ITerm < outMin) *ITerm= outMin;

	double dInput = (input - *lastInput);

	/*Compute PID Output*/
	double output = kp * error + *ITerm - kd * dInput;

	if(output > outMax) output = outMax;
	else if(output < outMin) output = outMin;

	/*Remember some variables for next time*/
	*lastInput = input;
	return output;

}




/**********************************************************************************
Function: PoluluMotor (constructor)
Parameters: 
  1. pinPWM: pin which would generate PWM for speed control of motor.
  2. pinIN1: Direction control pin 1
  3. pinIN2: Direction control pin 2

Description:
  The function checks for how many instances of PoluluMotor class have been
  are created. If it's more than 2, it returns the instance without any 
  pin assignments. 
  
  If instance-number validation is successful, it assigns the given pin 
  numbers to internal variables.
**********************************************************************************/
PoluluMotor::PoluluMotor(uint8_t pinPWM, uint8_t pinIN1, uint8_t pinIN2){
  
  if (instances == 0){ this->me_instance = 1; instances++; }
  else if (instances == 1){ this->me_instance = 2; instances++; }

  this->pwm = pinPWM;
  this->in1 = pinIN1;
  this->in2 = pinIN2;
  
  this->lastInput = 0;
  this->ITerm = 0;
}



/**********************************************************************************
Function: attachEncoder
Parameters: 
  1. pinENCA: Encoder channel A pin

Description:
  The function attaches the encoder and enables the speed sensing. Use this 
  function when single channel of encoder will be used.
**********************************************************************************/
void PoluluMotor::attachEncoder(uint8_t pinENCA){
  if (this->me_instance == 1){ m1_encA = pinENCA;}   
  if (this->me_instance == 2){ m2_encA = pinENCA;}
  
  this->feedback_enable = true;
  encoders.init(m1_encA, m1_encB, m2_encA, m2_encB);
}



/**********************************************************************************
Function: attachEncoder
Parameters: 
  1. pinENCA: Encoder channel A pin
  2. pinENCA: Encoder channel B pin  

Description:
  The function attaches the encoder and enables the speed sensing. Use this 
  function when both channels of quadrature encoder will be used.
**********************************************************************************/
void PoluluMotor::attachEncoder(uint8_t pinENCA, uint8_t pinENCB){
  if (this->me_instance == 1){ m1_encA = pinENCA; m1_encB = pinENCB;}
  if (this->me_instance == 2){m2_encA = pinENCA; m2_encB = pinENCB;}
  
  this->feedback_enable = true;
  encoders.init(m1_encA, m1_encB, m2_encA, m2_encB);  
}



/**********************************************************************************
Function: detachEncoder
Parameters: None

Description:
  The function detaches the encoder and speed sensing is disabled. Might be used 
  in open-loop system.
**********************************************************************************/
void PoluluMotor::detachEncoder(){
  if (this->me_instance == 1){ m1_encA = 255; m1_encB = 255;}
  if (this->me_instance == 2){ m2_encA = 255; m2_encB = 255;}
  
  this->feedback_enable = false;
  encoders.init(m1_encA, m1_encB, m2_encA, m2_encB);  
}



/**********************************************************************************
Function: setDuty
Parameters: 
  1. perDuty: percentage duty cycle to be applied to pwm pin. Range: 0-1
  
Description:
  Directly applies the percentage PWM to the pin. Note, the PWM frequency is 
  adjusted automatically to suit the duty cycle.
**********************************************************************************/
void PoluluMotor::setDuty(double perDuty){
  setPWM(this->pwm, perDuty);
}



/**********************************************************************************
Function: setRefSpeed
Parameters: 
  1. rpm: desired reference speed (generally when using PID)

Description:
  The function updates the value of internal setpoint for speed.
**********************************************************************************/
void PoluluMotor::setRefSpeed(double rpm){
  this->refSpeed = map(rpm, 0.0, 300.0, 0.0, 1.0);
}



/**********************************************************************************
Function: setGains
Parameters: 
  1. Kp: Proportional Gain for PID
  2. Ki: Integral Gain for PID
  3. Kd: Derivative Gain for PID

Description:
  The function updates the value of internal gains for PID algorithm. 
**********************************************************************************/
void PoluluMotor::setGains(double Kp, double Ki, double Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}



/**********************************************************************************
Function: setPIDEngage
Parameters: 
  1. value: boolean -- True sets and engages the PID controller. False - disengages it
Description:
  The function sets whether PID controller shall be activated or not.
  
  $$ The PID currently uses the AUTOMATIC mode, i.e. the gains will be tweaked
     so as to achieve smooth control $$
  $$ The aforementioned point needs to be evaluated as per requirement and
     necessary changes made in this code. $$
**********************************************************************************/
void  PoluluMotor::setPIDEngage(boolean value){
  this->pid_engage = value;
	
  //myPID->SetMode(AUTOMATIC);   // automatic PID tunings
  //myPID.SetMode(MANUAL);      // manual PID tunings  
}



/**********************************************************************************
Function: getSpeed
Parameters: None
Returns: Speed in RPM (double)

Description:
  The function computes the current speed of motor based on encoder counts and 
  returns the speed in RPM.
  
  $$ The caibration is not yet done. The value is theoretical $$
**********************************************************************************/
void PoluluMotor::motorEnable(boolean val){
	if (val == true){
		this->motor_enable = true;
		
		digitalWrite(this->in1, HIGH);
		digitalWrite(this->in2, LOW);
		pinMode(this->pwm, OUTPUT);
	}
	else{
		this->motor_enable = false;
		
		digitalWrite(this->in1, LOW);
		digitalWrite(this->in2, LOW);
		pinMode(this->pwm, INPUT);
	}
}


/**********************************************************************************
Function: getSpeed
Parameters: None
Returns: Speed in RPM (double)

Description:
  The function computes the current speed of motor based on encoder counts and 
  returns the speed in RPM.
  
  $$ The caibration is not yet done. The value is theoretical $$
**********************************************************************************/
double PoluluMotor::getSpeed(){
  if (feedback_enable == true){
    
    long int count;
  
    static long int _lastRunTime;
    long int _now = millis() - _lastRunTime;
  
    if (this->me_instance == 1) {count = encoders.getCountsAndResetM1();}
    if (this->me_instance == 2) {count = encoders.getCountsAndResetM2();}
    
    this->currSpeed = (count/1636.8) * (1000 / _now) * 60;  //in RPM
    return this->currSpeed;
  }
  
//  Serial.println(analogRead(A0));
//  this->currSpeed = map(analogRead(A0), 0.0, 1024.0, 0.0, 1.0);
//  return this->currSpeed;
}


/**********************************************************************************
Function: getDirection
Parameters: None
Returns: 1: Clockwise rotation, 0: Not moving, -1: counter-clockwise rotation

Description:
  The function determines whether motor is moving by it's speed. If it's moving,
  then it determines the clockwise and counter-clockwise sense of rotation
  based on custom implementation. 
  
  $$ The function is NOT YET IMPLEMENTED $$
**********************************************************************************/
uint8_t PoluluMotor::getDirection(){
  return 0;
}
  
 
 
/**********************************************************************************
Function: brake
Parameters: 
  1. duration: duration (in ms) for which the brakes are to be applied. 
      [duration = -1 implies indefinite braking.]
  
Description:
  Applies brake to motor for given duration. After given duration, the brakes are
  released by resetting the direction to forward.
  
**********************************************************************************/
void PoluluMotor::brake(uint8_t duration_ms){
  digitalWrite(this->in1, LOW);
  digitalWrite(this->in2, LOW);
  
  if (duration_ms > 0) {
    delay(duration_ms);
    digitalWrite(this->in1, HIGH);
  }
}


/**********************************************************************************
Function: reverse
Parameters: None

Description:
  Reverses the direction of motor. Initially brakes and then reverses.

  $$ Currently braking duration is chosen randomly as 2ms $$
  
**********************************************************************************/
void PoluluMotor::reverse(){
  
  this->brake(2);
  if (this->isClockwise == true) {digitalWrite(this->in1, LOW); digitalWrite(this->in2, HIGH); isClockwise = false; }
  else                           {digitalWrite(this->in1, HIGH); digitalWrite(this->in2, LOW); isClockwise = true ; }  
}



/**********************************************************************************
Function: applyUpdate
Parameters: None

Description:
  Applies the speed update to motor. If PID controller is engaged, the pid 
  correction is computed and applied. Else, the reference speed is proportionally
  converted to pwm and applied.
  
**********************************************************************************/
void PoluluMotor::applyUpdate(){
  
  if (this->pid_engage == false && this->motor_enable == true){ 
    setPWM(this->pwm, this->refSpeed); 
  }
  else if (this->pid_engage == true && this->motor_enable == true) { 
    this->output = PIDCompute(this->currSpeed, this->refSpeed, this->Kp, this->Ki, this->Kd, &(this->lastInput), &(this->ITerm)); 
	double constrnVal = constrain(this->output, 0, 1);
    setPWM(this->pwm, constrnVal); 
  }
  
}