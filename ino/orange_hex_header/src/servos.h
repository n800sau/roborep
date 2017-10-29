#ifndef __SERVOS_H

#define __SERVOS_H

#include <Arduino.h>
#include <Servo.h>

extern Servo head_pan_servo;
extern Servo head_tilt_servo;

void setup_servos();

void center_servos();

void detach_servos();

#endif //__SERVOS_H
