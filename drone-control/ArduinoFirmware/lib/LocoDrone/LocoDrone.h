//
// Created by joey on 8/8/16.
//

#ifndef ACCCONTROL_LOCODRONE_H
#define ACCCONTROL_LOCODRONE_H

// Libraries
#include <HCD.h> // RF Library
#include <I2Cdev.h> // I2C communication library
#include <Wire.h> // Arduino I2C library
#include <MPU6050.h> // Accelerometer library

#include <Messages.h>

#define PWM_MIN 0
#define PWM_MAX 255

#define TRIM_MIN 0
#define TRIM_MAX 128

extern HCD rfm;


class LocoDrone {

public:
    LocoDrone();

    uint8_t getThrottle();
    uint8_t getYaw();
    uint8_t getYawTrim();
    uint8_t getPitch();
    uint8_t getRoll();
    uint8_t getPitchTrim();
    uint8_t getRollTrim();
    uint8_t getFlyDrive();

    bool is_active();

    void setThrottle(uint8_t throttle);
    void setYaw(uint8_t yaw);
    void setYawTrim(uint8_t yawTrim);
    void setPitch(uint8_t pitch);
    void setRoll(uint8_t roll);
    void setPitchTrim(uint8_t pitchTrim);
    void setRollTrim(uint8_t rollTrim);
    void setFlyDrive(uint8_t flyDrive);

    void bind();
    void unbind();
    void reconnect();

    void resetPayload();
    void sendPayload();

private:
    uint8_t throttle_;
    uint8_t yaw_;
    uint8_t yawTrim_;
    uint8_t pitch_;
    uint8_t roll_;
    uint8_t pitchTrim_;
    uint8_t rollTrim_;
    uint8_t flyDrive_;

    unsigned char ID_[4];
    bool active_;

    ControlSourceType controller_;
};


#endif //ACCCONTROL_LOCODRONE_H
