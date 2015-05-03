/************************************************************************************************************
 * PoluluMotor.h - Arduino library for interfacing Polulu metal gearmotors with/without encoders.           *
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



#ifndef POLULUMOTOR_h
#define POLULUMOTOR_h

#include <PID_v1.h>
#include <PololuWheelEncoders.h>
#include "Arduino.h"


#define MAX_SPEED 300

class PoluluMotor{
  
  static int instances;                                               // Count of instances. At max 2 Instances allowed.
  static int m1_encA, m1_encB, m2_encA, m2_encB;                      // 4 Encoder pins from 2 motors.
  
  
  
  public:  // Functions
    PoluluMotor(uint8_t pinPWM, uint8_t pinIN1, uint8_t pinIN2);      // Constructor
    void attachEncoder(uint8_t pinENCA);                              // If single encoder channel will be used
    void attachEncoder(uint8_t pinENCA, uint8_t pinENCB);             // If both encoder channels will be used
    void detachEncoder();                                             // Disengages the encoder.
    
    void setDuty(double perDuty);                                     // Sets the duty cycle, perDuty in range (0, 1)
    void setRefSpeed(double rpm);                                     // Set the reference speed (if PID engaged, pid shall be used)
    void setGains(double Kp, double Ki, double Kd);                   // PID gains, if engaged
    void setPIDEngage(boolean value);                                 // Set the pid_engage variable value
    
    double getSpeed();                                                // Returns the speed in RPM
    uint8_t getDirection();                                           // Returns the direction of rotation (1: Clockwise, 0: Stopped, -1: Anticlockwise)
    
    void brake(uint8_t duration_ms);                                  // Explicit braking with GND/Vs
    void reverse();                                                   // Safely reverses the direction of motor
    
    void applyUpdate();                                               // Applies the speed corrections (manual pwm or PID correction, whichever applicable)
    
    
  public:  // Variables
    int me_instance;
    boolean pid_engage, motor_enable, feedback_enable;
    boolean isClockwise;
    
    uint8_t pwm, in1, in2;
    double currSpeed, refSpeed, output;
    double Kp, Ki, Kd;
  
    PID* myPID;    
    
};

#endif