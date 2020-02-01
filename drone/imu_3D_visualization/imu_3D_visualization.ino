// Include Required Libraries
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// Create Define For Data Type Output From Library
#define OUTPUT_READABLE_ACCELGYRO
// Include the Wire.h Arduino I2C Library
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 12.7 // Declination (degrees) in Philadelphia, PA.

void setup() {
 // Matlab/Arduino Handshake
 Serial.begin(115200);  
 Serial.println('a');
 char a = 'b';
 // Wait for a specific character from the PC
 while (a != 'a')
  a = Serial.read();
 // Initialize Arduino Wire.h I2C Library
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
 #endif

 // Before initializing the IMU, there are a few settings
 // we may need to adjust. Use the settings struct to set
 // the device's communication mode and addresses:
 imu.settings.device.commInterface = IMU_MODE_I2C;
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
 // The above lines will only take effect AFTER calling
 // imu.begin(), which verifies communication with the IMU
 // and turns it on.
 if (!imu.begin()){
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(-ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.println(pitch);
  Serial.println(roll);
  Serial.println(heading);
}

void loop() {  
}
// Triggered When Detects Serial Data
void serialEvent() {
 // Wait for Command Character
 char incomingByte;
 incomingByte = Serial.read();
 // Read raw accel/gyro measurements from device
 imu.readAccel();
 imu.readGyro();
 imu.readMag();
 
 
 
 // Switch Based on Read in Command Character
 switch(incomingByte) {
  case 'a': // Accelereometer Data Requested
    Serial.println(imu.ax);
    Serial.println(imu.ay);
    Serial.println(imu.az);
      break;
  case 'g': // Gyroscope Data Requested
    Serial.println(imu.gx);
    Serial.println(imu.gy);
    Serial.println(imu.gz);
      break;
  case 'm': // Magnetometer Data Requested
    Serial.println(imu.mx);
    Serial.println(imu.my);
    Serial.println(imu.mz);
      break;
  case 'p': // Processed Orientation Values
    printAttitude(imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);   
       break;
  case 'c': // Calibrate imu
    imu.calibrate(true);
      break;
                        
    }
}

