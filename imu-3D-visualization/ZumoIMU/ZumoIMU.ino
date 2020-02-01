/* This example reads the raw values from the L3GD20H gyro and
LSM303D accelerometer and magnetometer on the Zumo 32U4, and
prints those raw values to the serial monitor.

The accelerometer readings can be converted to units of g using
the conversion factors specified in the "Sensor characteristics"
table in the LSM303 datasheet.  The default full-scale (FS)
setting is +/- 2 g, so the conversion factor is 0.61 mg/LSB
(least-significant bit).  A raw reading of 16384 would correspond
to 1 g.

The gyro readings can be converted to degrees per second (dps)
using the "Mechanical characteristics" table in the L3GD20H
datasheet.  The default sensitivity is 8.75 mdps/LSB
(least-significant bit).  A raw reading of 10285 would correspond
to 90 dps.

The magnetometer readings are more difficult to interpret and
will usually require calibration. */

#include <Wire.h>
#include <Zumo32U4.h>
#define ONE_G 16384
LSM303 compass;
L3G gyro;
bool flag = true;
char report[120];

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 12.7 // Declination (degrees) in Philadelphia, PA.

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  if (!gyro.init())
  {
    // Failed to detect the gyro.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }

  gyro.enableDefault();

}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  ax /= ONE_G;
  ay /= ONE_G;
  az /= ONE_G;
  
  float roll = (atan2(-ay, az));
  float pitch = (atan2(-ax, sqrt(ay * ay + az * az)));
  
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
  roll  *= 180.0 / PI * -1;
  
  
  Serial.println(pitch);
  Serial.println(roll);
  Serial.println(heading);
}

void loop()
{
  while(flag){
    char a = 'b';
    while (a != 'a')
      a = Serial.read();
    flag = false;
    Serial.println('a');
    delay(1000);
  }
  
  compass.read();
  gyro.read();

  char incomingByte = Serial.read();
  switch(incomingByte) {
  case 'a': // Accelereometer Data Requested
    Serial.println(compass.a.x);
    Serial.println(compass.a.y);
    Serial.println(compass.a.z);
      break;
  case 'g': // Gyroscope Data Requested
    Serial.println(gyro.g.x);
    Serial.println(gyro.g.y);
    Serial.println(gyro.g.z);
      break;
  case 'm': // Magnetometer Data Requested
    Serial.println(compass.m.x);
    Serial.println(compass.m.y);
    Serial.println(compass.m.z);
      break;
  case 'p': // Processed Orientation Values
    printAttitude(compass.a.x, compass.a.y, compass.a.z, compass.m.x, compass.m.y, compass.m.z); 
       break;
  case 'c': // Calibrate imu
    
      break;
                        
    }
}
