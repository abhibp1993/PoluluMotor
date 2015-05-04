# PoluluMotor
Arduino Library for Interfacing Polulu Motor.

This library allows to interface upto 2 Polulu metal gearmotors with 48CPR encoders with Arduino. In general, any PMDC motor with it's respective driver with 3 control signals `PWM`, `DIR_1`, `DIR_2` can be controlled using `PoluluMotor` Class. 

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





