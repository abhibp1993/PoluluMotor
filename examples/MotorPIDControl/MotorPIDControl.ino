/********************************************************
 * Library: PoluluMotor
 * Example: MotorPIDControl
 * Author: Abhishek N. Kulkarni
 * 
 * The example demonstrates how to control speed of 
 * motor using PID controller. 
 *
 * For more details refer to 
 * https://github.com/abhibp1993/PoluluMotor/
 ********************************************************/
#include <PololuWheelEncoders.h>
#include <PoluluMotor.h>

#define M1_PWM   9
#define M1_IN1   3
#define M1_IN2   4


// Instantiate the motors
PoluluMotor m1(M1_PWM, M1_IN1, M1_IN2);


void setup() {
  Serial.begin(9600);			// Initialize serial communication
  
  m1.attachEncoder(5, 6);		// Both channel of encoder will be used
  m1.setGains(0.3, 0.1, 0.05);	// Set the PID gains
  m1.setPIDEngage(true);		// Enable PID
  m1.motorEnable(true);			// Enable the motor
  
  m1.setRefSpeed(125);			// Motor Reference Speed set to 125RPM
}


void loop() {  
  
  // Sense the current speeds
  Serial.println("----------");
  Serial.println(m1.getSpeed());
  
  // Apply update to speed
  m1.applyUpdate();
  
  // Wait for some time between PID corrections
  delay(100);
  
}
