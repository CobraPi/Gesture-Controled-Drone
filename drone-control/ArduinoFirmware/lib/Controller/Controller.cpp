#include "Controller.h"

ControlSourceType currentType = ControlSourceSerial;

JoystickData _joystickData;
JoystickResponse _joystickResponse;
JoystickCalibration _joystickCalibration;

AccelerometerData _accelerometer;

void controller_setController(ControlSourceType type) {
    currentType = type;
}


void processJoystick() {
    joystick_read();
    joystick_getData(&_joystickData);
    joystick_getResponse(&_joystickResponse);
    joystick_getCalibration(&_joystickCalibration);


    long throttle = map(_joystickData.throttle,
                                    _joystickCalibration.throttleMin,
                                    _joystickCalibration.throttleMax,
                                    PWM_MIN,
                                    PWM_MAX);

    long yaw = map(_joystickData.yaw,
                               _joystickCalibration.yawMin,
                               _joystickCalibration.yawMax,
                               PWM_MIN,
                               PWM_MAX);


    long roll = map(_joystickData.roll,
                                _joystickCalibration.rollMin,
                                _joystickCalibration.rollMax,
                                PWM_MIN,
                                PWM_MAX);

    long pitch = map(_joystickData.pitch,
                                 _joystickCalibration.pitchMin,
                                 _joystickCalibration.pitchMax,
                                 PWM_MIN,
                                 PWM_MAX);


    // Constrain the values to the range [0, 255]
    throttle = constrain(throttle, PWM_MIN, PWM_MAX);
    yaw = constrain(yaw, PWM_MIN, PWM_MAX);
    roll = constrain(roll, PWM_MIN, PWM_MAX);
    pitch = constrain(pitch, PWM_MIN, PWM_MAX);

    // Update the drone payload
    drone.setThrottle(uint8_t(throttle));
    drone.setYaw(uint8_t(yaw));
    drone.setRoll(uint8_t(roll));
    drone.setPitch(uint8_t(pitch));

}

void processAccelerometer() {
    // Get Roll and Pitch angles from accelerometer
    imu_read();
    imu_getAccelerometer(&_accelerometer);
    // Read Joystick Values
    joystick_read();
    joystick_getData(&_joystickData);
    joystick_getResponse(&_joystickResponse);
    joystick_getCalibration(&_joystickCalibration);

    // Get the control values from the joystick
    long throttle = _joystickData.throttle;
    long yaw = _joystickData.yaw;

    // Calculate Accelerometer attitude in radians
    double pitchRadians = atan(_accelerometer.y / sqrt(pow(_accelerometer.x, 2) + pow(_accelerometer.z, 2)));
    double rollRadians = atan(_accelerometer.x / sqrt(pow(_accelerometer.y, 2) + pow(_accelerometer.z, 2)));

    // Convert Accelerometer attitude to degrees
    int pitchDegrees = pitchRadians * 180 / M_PI;
    int rollDegrees = rollRadians * 180 / M_PI;


    //Serial.println(pitchDegrees);
    //Serial.println(rollDegrees);

    // Apply an exponential sensitivity curve to accelerometer data
    yaw = inputExponential(_joystickResponse.exponentialSensitivity,
                           yaw,
                           _joystickCalibration.yawMin,
                           _joystickCalibration.yawMax);

    rollDegrees = inputExponential(ACC_EXPO,
                            rollDegrees,
                            MAX_ROLL_ANGLE,
                            -MAX_ROLL_ANGLE);

    pitchDegrees = inputExponential(ACC_EXPO,
                             pitchDegrees,
                             MAX_PITCH_ANGLE,
                             -MAX_PITCH_ANGLE);

    // Map the PWM values to the range [0,255]

    throttle = map(throttle,
                   _joystickCalibration.throttleMin,
                   _joystickCalibration.throttleMax,
                   PWM_MIN,
                   PWM_MAX);

    yaw = map(yaw,
              _joystickCalibration.yawMin,
              _joystickCalibration.yawMax,
              PWM_MIN,
              PWM_MAX);

    rollDegrees = constrain(map(rollDegrees,
               MAX_ROLL_ANGLE,
               -MAX_ROLL_ANGLE,
               0,
               255), 0, 255);

    pitchDegrees = constrain(map(pitchDegrees,
                MAX_PITCH_ANGLE,
                -MAX_PITCH_ANGLE,
                0,
                255), 0, 255);


    // Constrain the values to the range [0, 255]
    throttle = constrain(throttle, PWM_MIN, PWM_MAX);
    yaw = constrain(yaw, PWM_MIN, PWM_MAX);
    //rollDegrees = constrain(rollDegrees, PWM_MIN, PWM_MAX);
    //pitchDegrees = constrain(pitchDegrees, PWM_MIN, PWM_MAX);


    //Serial.println("");
    //Serial.println(rollDegrees);
    //Serial.println(pitchDegrees);
    //Serial.println("");



    // Update the drone payload
    drone.setThrottle(uint8_t(throttle));
    drone.setYaw(uint8_t(yaw));
    drone.setRoll(uint8_t(rollDegrees));
    drone.setPitch(uint8_t(pitchDegrees));
}


void controller_controlLoop() {
    switch(currentType) {
        case ControlSourceJoystick:
            processJoystick();
            break;
        case ControlSourceAccelerometer:
            processAccelerometer();
            break;
        case ControlSourceSerial:
            break;
    }
}
