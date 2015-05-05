/********************************************************
 * Library: PoluluMotor
 * Example: MotorWithFeedback
 * Author: Abhishek N. Kulkarni
 * 
 * The example demonstrates how to control speeds of two
 * motors using this library using encoder feedback.
 * We shall attach the encoders and read the speed of 
 * motors.
 *
 * The two methods of attaching encoders are demonstrated.
 * 
 * For more details refer to 
 * https://github.com/abhibp1993/PoluluMotor/
 ********************************************************/

#include <PololuWheelEncoders.h>
#include <PoluluMotor.h>

#define M1_PWM   9
#define M1_IN1   3
#define M1_IN2   4

#define M2_PWM   10
#define M2_IN1   11
#define M2_IN2   12

// Instantiate the motors
PoluluMotor m1(M1_PWM, M1_IN1, M1_IN2);
PoluluMotor m2(M2_PWM, M2_IN1, M2_IN2);


void setup() {
  Serial.begin(9600);		// Initialize serial communication
  
  m1.attachEncoder(5);		// Single channel of encoder will be used
  m2.attachEncoder(6, 7);	// Both channels of encoder will be used
  
  m1.motorEnable(true);		// Enable the motor
  m2.motorEnable(true);		// Enable the motor
}


unsigned long int m1_speed = 100; 	   // Next reference speed of Motor 1
unsigned long int m2_speed = 100;      // Next reference speed of Motor 2

void loop() {  
  
  // Sense the current speeds
  Serial.println("----------");
  Serial.println(m1.getSpeed());
  Serial.println(m2.getSpeed());
  
  
  // Update reference speeds
  m1.setRefSpeed(m1_speed);
  m2.setRefSpeed(m2_speed);
  
  // Apply update to speed
  m1.applyUpdate();
  m2.applyUpdate();
  
  // Change speed
  m1_speed += 25;
  if (m1_speed > MAX_SPEED) {m2_speed = 100;}

  m2_speed += 25;
  if (m2_speed > MAX_SPEED) {m2_speed = 100;}
  
  // Wait for some time to observe the change.
  delay(1000);
  
}
