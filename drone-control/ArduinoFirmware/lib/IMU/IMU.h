#ifndef IMU_H
#define IMU_H

#define ONE_G 16384
#define TWO_G 32768

#define ACC_EXPO 100

#define ACC_ALPHA 0.2
#define GYRO_ALPHA 0.2

#include <MPU6050.h>
#include <inttypes.h>
#include <Messages.h>
#include <MathUtils.h>

typedef struct AccelerometerCalibration {
    // Accelerometer / Gyroscope offsets
    int16_t axOffset, ayOffset, azOffset;
    int16_t gxOffset, gyOffset, gzOffset;

    // Accelerometer / Gyroscope means
    int16_t axMean, ayMean, azMean;
    int16_t gxMean, gyMean, gzMean;

    // Amount of readings used to average, higher for more precision but slower
    int16_t bufferSize = 500;
} AccelerometerCalibration;

// Units in milli-g's
typedef struct AccelerometerData {
    int16_t rawX, rawY, rawZ;
    int16_t x, y, z;
} AccelerometerData;

typedef struct GyroscopeData {
    int16_t rawX, rawY, rawZ;
    int16_t x, y, z;
} GyroscopeData;

typedef struct AttitudeData {
    float pitchRadians;
    float rollRadians;

    int pitchDegrees;
    int rollDegrees;

} AttitudeData;

void imu_init();

void imu_read();

void imu_calibrateAccelerometer();

void imu_getAccelerometer(AccelerometerData *d);

void imu_getGyroscope(GyroscopeData *d);

void imu_getAttitude(AttitudeData *d);

#endif // IMU_H
