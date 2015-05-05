/********************************************************
 * Library: PoluluMotor
 * Example: MotorWithoutFeedback
 * Author: Abhishek N. Kulkarni
 * 
 * The example demonstrates how to control speeds of two
 * motors using this library without using any encoder
 * feedback. In other words, this is a open-loop system.
 * 
 * We demonstrate two different methods in which the speed
 * may be controlled using available functionality. The 
 * motor 1 is controlled using direct PWM duty cycle, 
 * while motor 2 is controlled by setting the reference
 * speed. 
 *
 * For more details refer to 
 * https://github.com/abhibp1993/PoluluMotor/
 ********************************************************/

#include <PoluluMotor.h>

#define M1_PWM   9
#define M1_IN1   3
#define M1_IN2   4

#define M2_PWM   10
#define M2_IN1   5
#define M2_IN2   6

PoluluMotor m1(M1_PWM, M1_IN1, M1_IN2);
PoluluMotor m2(M2_PWM, M2_IN1, M2_IN2);


void setup() {
  m2.motorEnable(true);
}


double m1_duty = 0.0;                // Next duty cycle for Motor 1
unsigned long int m2_speed = 0;      // Next reference speed of Motor 2

void loop() {  
  
  // Directly apply duty for Motor 1. 
  m1.setDuty(m1_duty);

  // Increment m1_duty to see increasing speed of motor
  m1_duty += 0.1;
  if (m1_duty > 1.0) {m1_duty = 0.0;}
  
  // Use refSpeed to change speed of Motor 2
  m2.setRefSpeed(m2_speed);
  m2.applyUpdate();
  
  // Change speed
  m2_speed += 10;
  if (m2_speed > MAX_SPEED) {m2_speed = 10;}
  
  // Wait for some time to observe the change.
  delay(1000);
  
}
