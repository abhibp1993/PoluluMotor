# PoluluMotor
Arduino Library for Interfacing Polulu Motor.

This library allows to interface upto 2 Polulu metal gearmotors with 48CPR encoders with Arduino. In general, any PMDC motor with it's respective H-bridge driver with 3 control signals `PWM`, `IN1`, `IN2` can be controlled using `PoluluMotor` Class. 

## Table of Contents
- Installation
- Usage
  - Motor without Encoder
  - Motor with Encoder
- PID Algorithm
- Limitations


## 1. Installation
Download the ZIP archive from the git or clone the repository on your desktop. Move the complete folder in Arduino Library folder.
In your sketch, include the library with
```cpp
#include <PoluluMotor.h>
```


## 2. Usage
To start off, you need to instantiate `PoluluMotor` Class. The instantiation requires 3 parameters; namely `PWM` representing the Arduino pin on which you would generate PWM for speed control of motor, `IN1` and `IN2` representing the direction control pins.
```cpp
PoluluMotor m1(9, 11, 12);
```

### a. Motor without Encoder
In case you wish to run a motor without encoder, you would only require to control the speed. This can be done in 2 ways;
```cpp
m1.setDuty(0.9);   // Sets 90% duty cycle
```
or
```cpp
m1.setRefSpeed(100);  // Sets reference speed to 100 RPM
m1.applyUpdate();     // Computes proportional PWM and applies it.
```

If you somehow know the model of motor, second method might be useful. *The code for model encompassing is not yet completed. Currently, only proportional duty cycle is computed and applied.*

### b. Motor with Encoder
The library assumes 48CPR quadrature encoder is available and is based on PoluluWheelEncoders library. Refer to [documentation](https://www.pololu.com/docs/0j18/18) for more details.

The library allows to interface encoder in 2 ways
- Single Channel: This can be done using `m1.attachEncoder(3)` to attach Channel A of encoder to pin 3.
- Dual Channel: This can be done using `m1.attachEncoder(3, 4)` to attach Channel A of encoder to pin 3 and Channel B to pin 4.

To detach the encoders use `m1.detachEncoders()`.

If the encoders are attached, the speed can be sensed in RPM using `double speed = m1.getSpeed()`. 


## 3. PID Algorithm
The library also provides an option to use PID algorithm for speed control. The implementation is inspired from PID library by [Brett Beauregard](http://playground.arduino.cc/Code/PIDLibrary). To use this feature, you will require to set the gains then engage PID algorithm. This can be done using

```cpp 
m1.setGains(0.5, 0.1, 0.05);    // Set the gains for motor1
m1.setPIDEngage(true);          // enable PID corrections for motor1
```

Finally, in the loop, update the current speed and call `applyUpdate` to apply single PID correction to speed. Optionally you may update the reference speed using `m1.setRefSpeed(...)` in case the speed set-point varies with time.

```cpp
m1.getSpeed();          // Update speed value for motor1
m1.applyUpdate();       // Apply PID Correction to motor1 speed
```


## 4. Limitations
- Only `two` instances of PoluluMotor are permitted due to constraints from `PoluluWheelEncoders` library.
- No gain autotuning feature available for PID. (refer library by [Brett Beauregard](http://playground.arduino.cc/Code/PIDLibrary) for autotuning).


