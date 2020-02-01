#include <Arduino.h>
#include <MathUtils.h>
#include "Joystick.h"


static JoystickData _joystickData;// = {0, 0, 0, 0, 0, 0, 0, 0};
static JoystickResponse _joystickResponse;// = {0.2, 1, 1, 1, 100};
static JoystickCalibration _joystickCalibration;// = {1021, 0, 1018, 4, 4, 1018, 1, 1020};

void joystick_init() {

    pinMode(THROTTLE_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
}

void joystick_getData(JoystickData *d) {
    d->rawThrottle = _joystickData.rawThrottle;
    d->rawYaw = _joystickData.rawYaw;
    d->rawRoll = _joystickData.rawRoll;
    d->rawPitch = _joystickData.rawPitch;

    d->throttle = _joystickData.throttle;
    d->yaw = _joystickData.yaw;
    d->roll = _joystickData.roll;
    d->pitch = _joystickData.pitch;
}

void joystick_getResponse(JoystickResponse *d) {
    d-> throttleAlpha = _joystickResponse.throttleAlpha;
    d-> yawAlpha = _joystickResponse.yawAlpha;
    d-> pitchAlpha = _joystickResponse.pitchAlpha;
    d-> rollAlpha = _joystickResponse.rollAlpha;
    d-> exponentialSensitivity =  _joystickResponse.exponentialSensitivity;
}

void joystick_getCalibration(JoystickCalibration *d) {
    d->throttleMin = _joystickCalibration.throttleMin;
    d->pitchMax = _joystickCalibration.throttleMax;

    d->yawMin = _joystickCalibration.yawMin;
    d->yawMax = _joystickCalibration.yawMax;

    d->rollMin = _joystickCalibration.rollMin;
    d->rollMax = _joystickCalibration.rollMax;

    d->pitchMin = _joystickCalibration.pitchMin;
    d->pitchMax = _joystickCalibration.pitchMax;
}

void joystick_read() {

    _joystickData.rawThrottle = analogRead(THROTTLE_PIN);
    _joystickData.rawYaw = analogRead(YAW_PIN);
    _joystickData.rawRoll = analogRead(ROLL_PIN);
    _joystickData.rawPitch = analogRead(PITCH_PIN);


    // Apply an alpha filter to the control values
    _joystickData.throttle = alphaFilter(_joystickData.rawThrottle, // Current value
                                         _joystickData.throttle, // Previous value
                                         _joystickResponse.throttleAlpha); // Alpha constant


    _joystickData.yaw = alphaFilter(_joystickData.rawYaw, // Current value
                                    _joystickData.yaw, // Previous value
                                    _joystickResponse.yawAlpha); // Alpha constant


    _joystickData.roll = alphaFilter(_joystickData.rawRoll, // Current value
                                     _joystickData.roll, // Previous value
                                     _joystickResponse.rollAlpha); // Alpha constant


    _joystickData.pitch = alphaFilter(_joystickData.rawPitch, // Current value
                                      _joystickData.pitch, // Previous value
                                      _joystickResponse.pitchAlpha); // Alpha constant

}



