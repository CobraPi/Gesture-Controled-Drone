//
// Created by joey on 8/8/16.
//

#include <Arduino.h>
#include "LocoDrone.h"

LocoDrone::LocoDrone() {
    throttle_ = 0;

    yaw_ = 127;
    yawTrim_ = 64;

    pitch_ = 127;
    pitchTrim_ = 64;

    roll_ = 127;
    rollTrim_ = 64;

    flyDrive_ = OperatingModeFly;

    ID_[0] = 0x16;
    ID_[1] = 0x01;
    ID_[2] = 0x55;
    ID_[3] = 0x11;

    active_ = false;
}

uint8_t LocoDrone::getThrottle() {
    return throttle_;
}

uint8_t LocoDrone::getYaw() {
    return yaw_;
}

uint8_t LocoDrone::getYawTrim() {
    return yawTrim_;
}

uint8_t LocoDrone::getPitch() {
    return pitch_;
}

uint8_t LocoDrone::getRoll() {
    return roll_;
}

uint8_t LocoDrone::getPitchTrim() {
    return pitchTrim_;
}

uint8_t LocoDrone::getRollTrim() {
    return rollTrim_;
}

uint8_t LocoDrone::getFlyDrive() {
    return flyDrive_;
}

bool LocoDrone::is_active() {
    return !rfm.inactive();
}

void LocoDrone::setThrottle(uint8_t throttle) {
    throttle_ = constrain(throttle, PWM_MIN, PWM_MAX);
}

void LocoDrone::setYaw(uint8_t yaw) {
    yaw_ = constrain(yaw, PWM_MIN, PWM_MAX);
}

void LocoDrone::setYawTrim(uint8_t yawTrim) {
    yawTrim_ = constrain(yawTrim, TRIM_MIN, TRIM_MAX);
}

void LocoDrone::setPitch(uint8_t pitch) {
    pitch_ = constrain(pitch, PWM_MIN, PWM_MAX);
}

void LocoDrone::setRoll(uint8_t roll) {
    roll_ = constrain(roll, PWM_MIN, PWM_MAX);
}

void LocoDrone::setPitchTrim(uint8_t pitchTrim) {
    pitchTrim_ = constrain(pitchTrim, TRIM_MIN, TRIM_MAX);
}

void LocoDrone::setRollTrim(uint8_t rollTrim) {
    rollTrim_ = constrain(rollTrim, TRIM_MIN, TRIM_MAX);
}

void LocoDrone::setFlyDrive(uint8_t flyDrive) {
    flyDrive_ = flyDrive;
}

void LocoDrone::bind() {
    rfm.bind(ID_);
    sendPayload();
}

void LocoDrone::unbind() {
    rfm.unbind();
    sendPayload();
}

void LocoDrone::reconnect() {
    rfm.reconnect(ID_);
    sendPayload();
}

void LocoDrone::resetPayload() {
   throttle_ = 0;
   yaw_ = 127;
   yawTrim_ = 64;
   pitch_ = 127;
   roll_ = 127;
   pitchTrim_ = 64;
   rollTrim_ = 64;
   flyDrive_ = 0;
}

void LocoDrone::sendPayload() {
    unsigned char payload[] = {
        (unsigned char) throttle_,
        (unsigned char) yaw_,
        (unsigned char) yawTrim_,
        (unsigned char) pitch_,
        (unsigned char) roll_,
        (unsigned char) pitchTrim_,
        (unsigned char) rollTrim_,
        (unsigned char) flyDrive_
    };

    rfm.update(payload);
}

LocoDrone drone;
