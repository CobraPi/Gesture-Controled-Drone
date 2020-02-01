#include "IMU.h"


MPU6050 mpu;

static AccelerometerCalibration _accelerometerCalibration;

static AccelerometerData _accelerometer = {0, 0, 0, 0, 0, 0};
static GyroscopeData _gyroscope = {0, 0, 0, 0, 0, 0};

void imu_init() {
    mpu.initialize();

    if (!mpu.testConnection()) {
        uint8_t packet[3];
        packet[0] = OutgoingMessageInitializationFailure;
        packet[1] = 1;
        packet[2] = InitializationFailureAccelerometer;

        Serial.write(packet, sizeof(packet));
    }
}


void imu_read() {
    mpu.getMotion6(&_accelerometer.rawX, &_accelerometer.rawY, &_accelerometer.rawZ,
                   &_gyroscope.rawX, &_gyroscope.rawY, &_gyroscope.rawZ);

    // apply filtering here
    _accelerometer.x = alphaFilter(_accelerometer.rawX, _accelerometer.x, ACC_ALPHA);
    _accelerometer.y = alphaFilter(_accelerometer.rawY, _accelerometer.y, ACC_ALPHA);
    _accelerometer.z = alphaFilter(_accelerometer.rawZ, _accelerometer.z, ACC_ALPHA);

    _gyroscope.x = alphaFilter(_gyroscope.rawX, _gyroscope.x, GYRO_ALPHA);
    _gyroscope.y = alphaFilter(_gyroscope.rawY, _gyroscope.y, GYRO_ALPHA);
    _gyroscope.z = alphaFilter(_gyroscope.rawZ, _gyroscope.z, GYRO_ALPHA);
}


void imu_calibrateAccelerometer() {
    // Do calibration
    long i = 0;
    long ax = 0;
    long ay = 0;
    long az = 0;

    long gx = 0;
    long gy = 0;
    long gz = 0;


    while (i<(_accelerometerCalibration.bufferSize+101)){
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&_accelerometer.rawX,
                       &_accelerometer.rawY,
                       &_accelerometer.rawZ,
                       &_gyroscope.rawX,
                       &_gyroscope.rawY,
                       &_gyroscope.rawZ);

        if (i>100 && i<=(_accelerometerCalibration.bufferSize+100)){ //First 100 measures are discarded
            ax = ax + _accelerometer.rawX;
            ay = ay + _accelerometer.rawY;
            az = az + _accelerometer.rawZ;
            gx = gx + _gyroscope.rawX;
            gy = gy + _gyroscope.rawY;
            gz = gz + _gyroscope.rawZ;
        }
        if (i==(_accelerometerCalibration.bufferSize+100)){
            _accelerometerCalibration.axMean = ax / _accelerometerCalibration.bufferSize;
            _accelerometerCalibration.ayMean = ay / _accelerometerCalibration.bufferSize;
            _accelerometerCalibration.azMean = az / _accelerometerCalibration.bufferSize;
            _accelerometerCalibration.gxMean = gx / _accelerometerCalibration.bufferSize;
            _accelerometerCalibration.gyMean = gy / _accelerometerCalibration.bufferSize;
            _accelerometerCalibration.gzMean = gz / _accelerometerCalibration.bufferSize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }


    _accelerometerCalibration.axMean = ax / _accelerometerCalibration.bufferSize;
    _accelerometerCalibration.ayMean = ay / _accelerometerCalibration.bufferSize;
    _accelerometerCalibration.azMean = az / _accelerometerCalibration.bufferSize;
    _accelerometerCalibration.gxMean = gx / _accelerometerCalibration.bufferSize;
    _accelerometerCalibration.gyMean = gy / _accelerometerCalibration.bufferSize;
    _accelerometerCalibration.gzMean = gz / _accelerometerCalibration.bufferSize;

    // Calculate offsets
    _accelerometerCalibration.axOffset = -_accelerometerCalibration.axMean / 8;
    _accelerometerCalibration.ayOffset = -_accelerometerCalibration.ayMean / 8;
    _accelerometerCalibration.azOffset = (ONE_G - _accelerometerCalibration.azMean) / 8;
    _accelerometerCalibration.gxOffset = -_accelerometerCalibration.gxMean / 4;
    _accelerometerCalibration.gyOffset = -_accelerometerCalibration.gyMean / 4;
    _accelerometerCalibration.gzOffset = -_accelerometerCalibration.gzMean / 4;

    // apply calibration here
    mpu.setXAccelOffset(_accelerometerCalibration.axOffset);
    mpu.setYAccelOffset(_accelerometerCalibration.ayOffset);
    mpu.setZAccelOffset(_accelerometerCalibration.azOffset);

    mpu.setXGyroOffset(_accelerometerCalibration.gxOffset);
    mpu.setYGyroOffset(_accelerometerCalibration.gyOffset);
    mpu.setZGyroOffset(_accelerometerCalibration.gzOffset);
}


void imu_getAccelerometer(AccelerometerData *d) {
    d->x = _accelerometer.x;
    d->y = _accelerometer.y;
    d->z = _accelerometer.z;

    d->rawX = _accelerometer.rawX;
    d->rawY = _accelerometer.rawY;
    d->rawZ = _accelerometer.rawZ;
}


void imu_getGyroscope(GyroscopeData *d) {
    d->x = _gyroscope.x;
    d->y = _gyroscope.y;
    d->z = _gyroscope.z;

    d->rawX = _gyroscope.rawX;
    d->rawY = _gyroscope.rawY;
    d->rawZ = _gyroscope.rawZ;
}




