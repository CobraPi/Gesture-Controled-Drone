//
// Created by joey on 8/23/16.
//

#include "IMU.h"

// Accelerometer Object and attributes
LSM9DS1 mpu;

IMU::IMU() {

    // Initialize class attributes
    _rawAx = 0;
    _rawAy = 0;
    _rawAz = 0;

    _rawGx = 0;
    _rawGy = 0;
    _rawGz = 0;

    _rawMx = 0;
    _rawMy = 0;
    _rawMz = 0;

    _ax = 0;
    _ay = 0;
    _az = 0;

    _gx = 0;
    _gy = 0;
    _gz = 0;

    _mx = 0;
    _my = 0;
    _mz = 0;

    _axG = 0;
    _ayG = 0;
    _azG = 0;

    _pitchRadians = 0;
    _rollRadians = 0;

    _pitchDegrees = 0;
    _rollDegrees = 0;

    _heading = 0;

    _temperature = 0;

    _accAlpha = 0.2;
    _gyroAlpha = 0.2;
    _magAlpha = 0.3;

};

void IMU::get_acc(int16_t *acc) {

    acc[X] = _ax;
    acc[Y] = _ay;
    acc[Z] = _az;
}

void IMU::get_accG(float *accG) {

    accG[X] = _axG;
    accG[Y] = _ayG;
    accG[Z] = _azG;
}

void IMU::get_gyro(int16_t *gyro) {

    gyro[X] = _gx;
    gyro[Y] = _gy;
    gyro[Z] = _gz;
}

void IMU::get_mag(int16_t *mag) {

    mag[X] = _mx;
    mag[Y] = _my;
    mag[Z] = _mz;
}

void IMU::get_pitchRadians(float &pitch) {

    pitch = _pitchRadians;
}

void IMU::get_rollRadians(float &roll) {

    roll = _rollRadians;
}

void IMU::get_pitchDegrees(int16_t &pitch) {

    pitch = _pitchDegrees;
}

void IMU::get_rollDegrees(int16_t &roll) {

    roll = _rollDegrees;
}

void IMU::get_heading(int16_t &heading) {

    heading = _heading;
}

void IMU::get_chipTemp(int16_t &temp) {

    temp = _temperature;
}

void IMU::set_accAlpha(float accAlpha) {

    _accAlpha = accAlpha;
}

void IMU::set_gyroAlpha(float gyroAlpha) {

    _gyroAlpha = gyroAlpha;
}

void IMU::set_magAlpha(float magAlpha) {

    _magAlpha = magAlpha;
}

void IMU::read_acc() {

    mpu.readAccel();

    _rawAx = mpu.ax;
    _rawAy = mpu.ay;
    _rawAz = mpu.az;

    _ax = alphaFilter(_rawAx, _ax, _accAlpha);
    _ay = alphaFilter(_rawAy, _ay, _accAlpha);
    _az = alphaFilter(_rawAz, _az, _accAlpha);

    _axG = mpu.calcAccel(_ax);
    _ayG = mpu.calcAccel(_ay);
    _azG = mpu.calcAccel(_az);
}

void IMU::read_gyro() {

    mpu.readGyro();

    _rawGx = mpu.gx;
    _rawGy = mpu.gy;
    _rawGz = mpu.gz;

    _gx = alphaFilter(_rawGx, _gx, _gyroAlpha);
    _gy = alphaFilter(_rawGy, _gy, _gyroAlpha);
    _gz = alphaFilter(_rawGz, _gz, _gyroAlpha);
}

void IMU::read_mag() {

    mpu.readMag();

    _rawMx = mpu.mx;
    _rawMy = mpu.my;
    _rawMz = mpu.mz;

    _mx = alphaFilter(_rawMx, _mx, _magAlpha);
    _my = alphaFilter(_rawMy, _my, _magAlpha);
    _mz = alphaFilter(_rawMz, _mz, _magAlpha);
}

void IMU::read() {

    read_acc();
    read_gyro();
    read_mag();
}

void IMU::init() {

    // Before initializng the IMU, ther are a few settings
    // we may need to adjust. Use the settings struct to set
    // the device's communication mode and addresses:
    mpu.settings.device.commInterface = IMU_MODE_I2C;
    mpu.settings.device.mAddress = LSM9DS1_M;
    mpu.settings.device.agAddress = LSM9DS1_AG;
    // The above lines will only take effect AFTER calling
    // imu.begin(), which verifies communication witht he IMU
    // and turns it on.
    mpu.begin();

}

void IMU::calibrate() {

    mpu.calibrate();
}

void IMU::calibrate_mag() {

    mpu.calibrateMag();
}

/*
void IMU::cal_attitude() {

    _pitchRadians = atan(_ax / sqrt(pow(_ay, 2) + pow(_az, 2)));
    _rollRadians = atan(_ay / sqrt(pow(_ax, 2) + pow(_az, 2)));

    _pitchDegrees = int16_t(_pitchRadians * 180 / M_PI);
    _rollDegrees = int16_t(_rollRadians * 180 / M_PI);

    _pitchDegrees *= -1;
    _rollDegrees *= -1;
}
*/

void IMU::cal_attitude() {

    _rollRadians = atan(_ax / sqrt(pow(_ay, 2) + pow(_az, 2)));
    _pitchRadians = atan(_ay / sqrt(pow(_ax, 2) + pow(_az, 2)));

    _rollDegrees = int16_t(_rollRadians * 180 / M_PI);
    _pitchDegrees = int16_t(_pitchRadians * 180 / M_PI);

    _rollDegrees *= -1;
}

void IMU::cal_heading() {

    float heading;
    if (_my == 0) {
        heading = (_mx < 0) ? 180.0 : 0;
    }
    else {
        heading = atan2(_mx, _my);
    }

    heading -= DECLINATION * M_PI / 180.0;

    if (heading > M_PI) {

        heading -= 2 * M_PI;
    }
    else if (heading < -M_PI) {

        heading += 2 * M_PI;
    }
    else if (heading < 0) {

        heading += 2 * M_PI;
    }
    _heading = int16_t(heading * (180.0/M_PI));
}

void IMU::cal_absoluteOrientation() {

    cal_attitude();
    cal_heading();
}






