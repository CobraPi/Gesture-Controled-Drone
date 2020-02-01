#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <MathUtils.h>
#include <Messages.h>
#include <Joystick.h>
#include <IMU.h>
#include <LocoDrone.h>

#define MAX_PITCH_ANGLE 40
#define MAX_ROLL_ANGLE 40

#define PWM_MIN 0
#define PWM_MAX 255

extern LocoDrone drone;

void controller_setController(ControlSourceType type);

void controller_controlLoop();

#endif // CONTROLLER_H
