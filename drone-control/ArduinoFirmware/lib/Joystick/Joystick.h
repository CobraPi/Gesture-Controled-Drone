#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <MathUtils.h>
#include <inttypes.h>
#include <Messages.h>

#define THROTTLE_PIN A3
#define YAW_PIN A2
#define PITCH_PIN A1
#define ROLL_PIN A0

typedef struct JoystickData {
    int16_t rawThrottle, rawYaw, rawPitch, rawRoll;
    int16_t throttle, yaw, pitch, roll;
} JoystickData;

typedef struct JoystickResponse {
    float throttleAlpha = 0.2;
    float yawAlpha = 1;
    float rollAlpha = 1;
    float pitchAlpha = 1;

    int exponentialSensitivity = 100;
} JoystickResponse;

typedef struct JoystickCalibration {
    int throttleMin = 1021;
    int throttleMax = 0;

    int yawMin = 1018;
    int yawMax = 4;

    int rollMin = 4;
    int rollMax = 1018;

    int pitchMin = 1;
    int pitchMax = 1020;


} JoystickCalibration;

void joystick_init();

void joystick_getData(JoystickData *d);

void joystick_getResponse(JoystickResponse *d);

void joystick_getCalibration(JoystickCalibration *d);

void joystick_read();


#endif // JOYSTICK_H
