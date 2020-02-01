//
// Created by joey on 8/23/16.
//

#ifndef IMU_H
#define IMU_H


#include <avr/io.h>
#include <SparkFunLSM9DS1.h>
#include <Utils.h>


// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading.
// Calculate yours here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 12.7 // Declination (degrees) in Philadelphia, PA

enum AXIS {X, Y, Z};

class IMU {

private:

    // Raw IMU component values
    int16_t _rawAx, _rawAy, _rawAz;
    int16_t _rawGx, _rawGy, _rawGz;
    int16_t _rawMx, _rawMy, _rawMz;

    // Filtered IMU component values
    int16_t _ax, _ay, _az;
    int16_t _gx, _gy, _gz;
    int16_t _mx, _my, _mz;

    // Gravity in G's
    float _axG, _ayG, _azG;

    // Attitude
    float _pitchRadians;
    float _rollRadians;

    int16_t _pitchDegrees;
    int16_t _rollDegrees;


    // Orientation
    int16_t _heading;

    // Temperature
    int16_t _temperature;

    // Filter Values
    float _accAlpha;
    float _gyroAlpha;
    float _magAlpha;

public:

    IMU();

    void get_acc(int16_t *acc);
    void get_accG(float *accG);
    void get_gyro(int16_t *gyro);
    void get_mag(int16_t *mag);

    void get_pitchRadians(float &pitch);
    void get_rollRadians(float &roll);

    void get_pitchDegrees(int16_t &pitch);
    void get_rollDegrees(int16_t &roll);

    void get_heading(int16_t &heading);

    void get_chipTemp(int16_t &temp);

    void set_accAlpha(float accAlpha);
    void set_gyroAlpha(float gyroAlpha);
    void set_magAlpha(float magAlpha);

    void read_acc();
    void read_gyro();
    void read_mag();
    void read();

    void init();

    void calibrate();
    void calibrate_mag();

    // Must call [read_acc()]
    void cal_attitude();
    // Must call [read_mag()]
    void cal_heading();
    // Must call [read()]
    void cal_absoluteOrientation();

};


#endif //IMU_H
