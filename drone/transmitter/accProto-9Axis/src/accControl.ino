//************************************************************************
//  LocoDrone Intuitive Transmitter
//  
//  Author: Joey Orduna
//
//
// LED RING CONNECTIONS
//
// Signal: 3 ---> Middle Pin (Signal)
//
// JOYSTICK CONNECTIONS
//
// Throttle: A0 ---> VERT (left joystick)
// Yaw     : A1 ---> HORZ (left joystick)
// Pitch   : A2 ---> VERT (right joystick)
// Roll    : A3 ---> HORZ (right joystick)
//
// ACCELEROMETER CONNECTIONS
//
// Data Line : A4 ---> SDA
// Clock Line: A5 ---> SCL
// Interrupt :  2 ---> INT
//
// RF CHIP CONNEC:IONS
//
// Chip Enable :        8 ----> CE
// Chip Select Not:     10 ---> CSN
// SPI Clock:           13 ---> SCK
// Master Out Slave In: 11 ---> MOSI
// Master In Slave Out: 12 ---> MISO
//************************************************************************

// TODO - Implement aggregate_payload() function to consolidate PWM values

// Joystick Control Pins
#define THROTTLE_PIN A3 // A2
#define YAW_PIN A7// A3
#define PITCH_PIN A6//A0
#define ROLL_PIN A1
#define THUMB_PIN A0
#define INDEX_PIN A2
#define MIDDLE_PIN A1
// Joystick Button Pins
#define LEFT_BUTTON_PIN 4
#define RIGHT_BUTTON_PIN 9

// Analog Joystick Input Range
#define JOY_MIN 0
#define JOY_MAX 1023
// PWM Range
#define PWM_MIN 0
#define PWM_MAX 255
// Flight Modes
#define FLY 0x00
#define RUN 0x0F

// Control Interface
#define JOYSTICKS 0
#define ACCELEROMETER 1

// Libraries
#include <HCD.h> // RF Library
#include <I2Cdev.h> // I2C communication library
#include <Wire.h> // Arduino I2C library
#include <SPI.h> //
#include <SparkFunLSM9DS1.h> // Accelerometer library
// LED lights
extern "C"{
#include <ws2812.h>
};

// Instance of drone RF object
HCD drone0;
// Drone ID's (pick 4 random numbers)
unsigned char ID0[]={0x16,0x01,0x55,0x11};

struct control_parameters { // Parameters for joystick/accelerometer control 
    
    // Control interface flag - determines input interface
    uint8_t control = ACCELEROMETER;
    
    // Analog joystick input value range
    int throttleMin = JOY_MIN,  throttleMax = JOY_MAX;
    int yawMin      = JOY_MIN,  yawMax      = JOY_MAX;
    int pitchMin    = JOY_MIN,  pitchMax    = JOY_MAX;
    int rollMin     = JOY_MIN,  rollMax     = JOY_MAX;
    
    // PWM range
    unsigned int pwmMin = PWM_MIN;
    unsigned int pwmMax = PWM_MAX;
    
    // Tilt-angle range in degrees for accelerometer control
    int rollLimit = 30, pitchLimit = 30;
    // Roll and pitch angle values in radians
    double rollRad;
    double pitchRad;
    // Roll and pitch angle values in degrees
    int rollDeg;
    int pitchDeg;
} controller;


// Analog joystick max and min range
enum CONTROL_RANGE {MIN, MAX};
unsigned int throttleStickRange[] = {2, 1023},
             yawStickRange[] = {0, 1023},
             pitchStickRange[] = {0, 1023},
             rollStickRange[] = {0, 1023},
             pwmRange[] = {0, 255};

unsigned int thumbFlexRange[] = {0, 0},
             indexFlexRange[] = {0, 0},
             middleFlexRange[] = {0, 0};

// Maximum tilt angle for accelerometer control
int rollAngleLimit = 40, pitchAngleLimit = 40;
int rollAngleRange[] = {-rollAngleLimit, rollAngleLimit},
    pitchAngleRange[] = {-pitchAngleLimit, pitchAngleLimit};

struct flexSensors {
    unsigned int thumb;
    unsigned int index;
    unsigned int middle;
} flex;

struct joystick {
    unsigned int throttle;
    unsigned int yaw;
    unsigned int pitch;
    unsigned int roll;
    bool leftButton;
    bool rightButton;
} joy;

struct accelerometer {
    const int ONE_G = 16384;
    const int TWO_G = 32768;
    // Raw values from the accelerometer
    int    rawX;
    int    rawY;
    int    rawZ;
    // Processed/filtered values
    int    x;
    int    y;
    int    z;
    // Axis values in metric units of g (1g = 9.81 m/s^2)
    double xG;
    double yG;
    double zG;

} acc;
struct gyroscope {
    // Raw values from the gyroscope
    int rawX, rawY, rawZ;
    // Processed/filtered values
    int x, y, z;
    // Gyro values in radians/second
    double xRadSec, yRadSec, zRadSec;
} gyro;

struct magnetometer {
    // Raw values from magnetometer
    int rawX, rawY, rawZ;
    // Processed/filtered values
    int x, y, z;
    // Magnetometer values in Gauss
    double xGauss, yGauss, zGauss;
} mag;

struct imu_calibration {
    // Accelerometer / Gyroscope offsets
    int axOffset, ayOffset, azOffset;
    int gxOffset, gyOffset, gzOffset;
    // Accelerometer / Gyroscope means
    int axMean, ayMean, azMean;
    int gxMean, gyMean, gzMean;
    // Amount of readings used to average, higher for more precision but slower
    int buffersize = 300;
} imu_cal;

struct filter_constants {
    // Alpha constant and exponential sensitivity for accelerometer
    double accAlpha = 0.2;
    int accExpo = 100;
    // Alpha constant for gyroscope
    double gyrAlpha = 0.2;
    // Alpha constant for Magnetometer
    double magAlpha = 0.3;
    // Alpha constant for joystick throttle
    double throttleAlpha = 0.2;
    // Alpha constnant for flex sensor
    double flexAlpha = 0.4;
    // Exponential sensitivity for flex Yaw
    int flexYawExpo = 100;
    // Exponential sensitivity for joystick control
    int joyExpo = 100;

} filter;

//// Analog joystick values
//int throttleStick = 0, yawStick = 0, pitchStick = 0, rollStick = 0;
//// Trim center values
//uint8_t throttleTrim = 0, yawTrim = 64, pitchTrim = 66, rollTrim = 62;
// Trim increment value
uint8_t trimIncrement = 2;

struct pwm_payload {
    // Payload Values
    unsigned int throttle = 0x00;
    unsigned int yaw = 0x00;
    unsigned int yawTrim = 64;
    unsigned int pitch = 0x00;
    unsigned int roll = 0x00;
    unsigned int pitchTrim = 66;
    unsigned int rollTrim = 62;
    unsigned int flyRun = FLY;
    // Throttle Offset
    unsigned char throttleTrim;
} payload;
// PWM control value payload
//enum PAYLOAD {THROTTLE, YAW, YAW_TRIM, PITCH, ROLL, PITCH_TRIM, ROLL_TRIM, FLY_RUN};
//unsigned char pwmPayload[] = {0x00, 0x80, yawTrim, 0x80, 0x80, pitchTrim, rollTrim, FLY};

// Accelerometer Object and attributes
LSM9DS1 imu;

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. 
// Calculate yours here: 
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 12.7 // Declination (degrees) in Philadelphia, PA


//// Raw Accelerometer and Gyroscope Data
//int16_t ax = 0, ay = 0, az = 0,
//        gx = 0, gy = 0, gz = 0;

// Accelerometer calibration

//int accDeadband = 8;   // Accelerometer error deadband
//int gyrDeadband = 1;   // Gyroscope error deadband

// Accelerometer/Gyroscope Offsets
//int16_t axOffset = 0, ayOffset = 0, azOffset = 0,
//        gxOffset = 0, gyOffset = 0, gzOffset = 0;
// Accelerometer/Gyroscope Means
//int16_t axMean = 0, ayMean = 0, azMean = 0,
//        gxMean = 0, gyMean = 0, gzMean = 0;

// Alpha Filter Variables
//double accAlpha = 1.68,
//        gyrAlpha = 0.75,
//        joyAlpha = 2.15;

//double accAlpha = 0.2,
//       gyrAlpha = 0.2,
//       joyAlpha = 0.2;

//Roll and Pitch angles in degrees
//int rollAngle = 0, pitchAngle = 0;

// Control interface flag - determines the input routine
//uint8_t control = JOYSTICKS; //Defaults to Joystick control

// Use a lookup table for angle-to-PWM mapping, set to false for one-to-one angle to PWM mapping
bool lookupPWM = false;

// Exponential sensitivity adjustment [0 - 100], set to 0 for a linear response
int expo = 100;

// Time variable for payload update
unsigned long timer=0;


struct led_color_values{
    // Raw 8-bit rgb color values for dynamic animations 
    uint8_t r,
            g,
            b;
    // Set led colors for static lighting

} led;

//****************************** FUNCTION DECLARATIONS ************************************************ 

// Joystick Control
void read_joystick();
void print_joystick_values();
void print_flex_values();
// Accelerometer Control
void printOffsets();
//void applyOffsets();
//void calibrateMPU();
void read_flex();
void read_acc();
void print_acc_values();
void angle_to_pwm();
// Universal Control
int alpha_filter(int currentInput, int previousInput, double alpha);
int input_exponential(long int value, int inputMin, int inputMax);
void print_payload();
void aggregate_payload();
void serial_control();

//******************************** SETUP CODE ************************************************

void setup(){
    // Initialize joystick pins
    //pinMode(THROTTLE_PIN, INPUT);
    //pinMode(YAW_PIN, INPUT);
    //pinMode(PITCH_PIN, INPUT);
    //pinMode(ROLL_PIN, INPUT);
    pinMode(LEFT_BUTTON_PIN, INPUT);
    pinMode(RIGHT_BUTTON_PIN, INPUT);
    //pinMode(YAW_TRIM_PIN, INPUT);
    //pinMode(PITCH_ROLL_TRIM_PIN, INPUT); 
    // Initialize serial
    Serial.begin(19200);
    delay(5);
    // Before initializng the IMU, ther are a few settings
    // we may need to adjust. Use the settings struct to set
    // the device's communication mode and addresses:
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;
    // The above lines will only take effect AFTER calling
    // imu.begin(), which verifies communication witht he IMU
    // and turns it on.
    imu.begin();
    
    init_leds();
    set_leds(0, 0, 10); //0,35,10
    sync_leds();

    Serial.print("Open your hand and press 'c' when done...");
    while(Serial.read() != 'c'){
        thumbFlexRange[MAX] = analogRead(THUMB_PIN);
        indexFlexRange[MAX] = analogRead(INDEX_PIN);
        middleFlexRange[MAX] = analogRead(MIDDLE_PIN);
    }
    Serial.println("Done");
    Serial.print("Close your hand and press 'c' when done...");
    while(Serial.read() != 'c'){
        thumbFlexRange[MIN] = analogRead(THUMB_PIN);
        indexFlexRange[MIN] = analogRead(INDEX_PIN);
        middleFlexRange[MIN] = analogRead(MIDDLE_PIN);
    }
    Serial.println("Done");
}

//******************************** MAIN RUN LOOP ************************************************ 

void loop()
{
    serial_control();
    //read_joystick();
    read_flex();
    read_acc();
    aggregate_payload();
    // Accelerometer calibration routine
   // if(digitalRead(LEFT_BUTTON_PIN) == 0 && drone0.inactive()){
     //   Serial.println("Calibrating MPU...");
       // calibrateMPU();
       // Serial.println("Calibration Done!");
       // printOffsets();
   // }
   

    // Payload update
    if(millis()>=timer){
        timer += 20;
        drone0.update(payload.throttle,
                      127,//  payload.yaw,
                      payload.yawTrim,
                      payload.pitch,
                      payload.roll,
                      payload.pitchTrim,
                      payload.rollTrim,
                      payload.flyRun);
    }
}
//****************************** FUNCTION IMPLEMENTATIONS ************************************************

void read_joystick(){
    // Reads the raw analog control values from the joysticks, converts to PWM,

    // Read in raw values from the analog joystick
    unsigned int rawYaw = analogRead(YAW_PIN),
            rawThrottle = analogRead(THROTTLE_PIN),
            rawRoll = analogRead(ROLL_PIN),
            rawPitch = analogRead(PITCH_PIN);

    // Apply and alpha filter to the throttle value
    joy.throttle = alpha_filter(rawThrottle, // Current value
                                joy.throttle, // Previous value
                                filter.throttleAlpha); // Alpha constant

    // Apply exponential sensitivity to joystick controls
    joy.yaw = input_exponential(rawYaw, yawStickRange[MIN], yawStickRange[MAX]);
    joy.roll = input_exponential(rawRoll, rollStickRange[MIN], rollStickRange[MAX]);
    joy.pitch = input_exponential(rawPitch, pitchStickRange[MIN], pitchStickRange[MAX]);
}
void read_flex(){
    unsigned int rawThumb = analogRead(THUMB_PIN),
                 rawIndex = analogRead(INDEX_PIN),
                 rawMiddle = analogRead(MIDDLE_PIN);
    
    //flex.thumb = input_exponential(rawThumb, thumbFlexRange[MIN], thumbFlexRange[MAX]); 
    flex.thumb = alpha_filter(rawThumb, flex.thumb, filter.flexAlpha);
    flex.index = alpha_filter(rawIndex, flex.index, filter.flexAlpha);
    flex.middle = alpha_filter(rawMiddle, flex.middle, filter.flexAlpha);
    
}
void print_joystick_values(){
    // Prints out the analog values of the Joysticks
    Serial.print("JOYSTICK VALUES: "); Serial.print('\t');
    Serial.print(" Throttle: ");
    Serial.print(analogRead(THROTTLE_PIN)); Serial.print('\t');
    Serial.print(" Yaw: ");
    Serial.print(analogRead(YAW_PIN)); Serial.print('\t');
    Serial.print(" Pitch: ");
    Serial.print(analogRead(PITCH_PIN)); Serial.print('\t');
    Serial.print(" Roll: ");
    Serial.println(analogRead(ROLL_PIN));
}
void print_flex_values(){
    // Prints out the analog values of the Joysticks
    Serial.print("FlEX VALUES: "); Serial.print('\t');
    Serial.print(" Thumb: ");
    Serial.print(analogRead(THUMB_PIN)); Serial.print('\t');
    Serial.print(" Index: ");
    Serial.print(analogRead(INDEX_PIN)); Serial.print('\t');
    Serial.print(" Middle: ");
    Serial.println(analogRead(MIDDLE_PIN));
}    
    
void printOffsets(){
    Serial.print("MPU OFFSETS: "); Serial.print('\t');
    Serial.print(" Acc X: "); Serial.print(imu_cal.axOffset);
    Serial.print(" Acc Y: "); Serial.print(imu_cal.ayOffset);
    Serial.print(" Acc Z: "); Serial.print(imu_cal.azOffset); Serial.print('\t');
    Serial.print(" Gyr X: "); Serial.print(imu_cal.gxOffset);
    Serial.print(" Gyr Y: "); Serial.print(imu_cal.gyOffset);
    Serial.print(" Gyr Z: "); Serial.println(imu_cal.gzOffset);
}

/*
void applyOffsets(){
    mpu.setXAccelOffset(imu_cal.axOffset);
    mpu.setYAccelOffset(imu_cal.ayOffset);
    mpu.setZAccelOffset(imu_cal.azOffset);
    mpu.setXGyroOffset(imu_cal.gxOffset);
    mpu.setYGyroOffset(imu_cal.gyOffset);
    mpu.setZGyroOffset(imu_cal.gzOffset);
}

void calibrateMPU(){
    long i=0,
         buff_ax=0, buff_ay=0, buff_az=0,
         buff_gx=0, buff_gy=0, buff_gz=0;

    while (i<(imu_cal.buffersize+101)){
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&acc.rawX,
                       &acc.rawY,
                       &acc.rawZ,
                       &gyro.rawX,
                       &gyro.rawY,
                       &gyro.rawZ);

        if (i>100 && i<=(imu_cal.buffersize+100)){ //First 100 measures are discarded
            buff_ax = buff_ax + acc.rawX;
            buff_ay = buff_ay + acc.rawY;
            buff_az = buff_az + acc.rawZ;
            buff_gx = buff_gx + gyro.rawX;
            buff_gy = buff_gy + gyro.rawY;
            buff_gz = buff_gz + gyro.rawZ;
        }
        if (i==(imu_cal.buffersize+100)){
            imu_cal.axMean = buff_ax / imu_cal.buffersize;
            imu_cal.ayMean = buff_ay / imu_cal.buffersize;
            imu_cal.azMean = buff_az / imu_cal.buffersize;
            imu_cal.gxMean = buff_gx / imu_cal.buffersize;
            imu_cal.gyMean = buff_gy / imu_cal.buffersize;
            imu_cal.gzMean = buff_gz / imu_cal.buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }

    // Calculate offsets
    imu_cal.axOffset =- imu_cal.axMean / 8;
    imu_cal.ayOffset =- imu_cal.ayMean / 8;
    imu_cal.azOffset = (acc.ONE_G - imu_cal.azMean) / 8;
    imu_cal.gxOffset =- imu_cal.gxMean / 4;
    imu_cal.gyOffset =- imu_cal.gyMean / 4;
    imu_cal.gzOffset =- imu_cal.gzMean / 4;
    // Apply calculated offsets
    applyOffsets();
}
*/
void read_acc(){
    // Reads accelerometer and converts data into degrees

    // Populate sample data with unfiltered values
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
    // Populate accelerometer data struct with read values
    acc.rawX = imu.ax; acc.rawY = imu.ay; acc.rawZ = imu.az;
    // Populate gyro data struct with read values
    gyro.rawX = imu.gx; gyro.rawY = imu.gy; gyro.rawZ = imu.gz;
    // Populate magnetometer struct with read values
    mag.rawX = imu.mx; mag.rawY = imu.my; mag.rawZ = imu.mz;
    // Filter the raw accelerometer values
    acc.x = alpha_filter(acc.rawX, acc.x, filter.accAlpha);
    acc.y = alpha_filter(acc.rawY, acc.y, filter.accAlpha);
    acc.z = alpha_filter(acc.rawZ, acc.z, filter.accAlpha);
    // Filter the raw gyroscope values
    gyro.x = alpha_filter(gyro.rawX, gyro.x, filter.gyrAlpha);
    gyro.y = alpha_filter(gyro.rawY, gyro.y, filter.gyrAlpha);
    gyro.z = alpha_filter(gyro.rawZ, gyro.z, filter.gyrAlpha);
    // Filter the raw magnetometer values
    mag.x = alpha_filter(mag.rawX, mag.x, filter.magAlpha);
    mag.y = alpha_filter(mag.rawY, mag.y, filter.magAlpha);
    mag.z = alpha_filter(mag.rawZ, mag.z, filter.magAlpha);
    // Convert raw data to g - Sensitivity Constants: (32768 = 2g, 16384 = 1g)
    acc.xG = imu.calcAccel(acc.x);
    acc.yG = imu.calcAccel(acc.y);
    acc.zG = imu.calcAccel(acc.z);
    
    // Convert g into radians
    controller.pitchRad = atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
    controller.rollRad  = atan(acc.x / sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
    // Convert radians into degrees
    controller.rollDeg = constrain(controller.pitchRad * 180 / M_PI, pitchAngleRange[MIN], pitchAngleRange[MAX]);
    controller.pitchDeg = constrain(controller.rollRad * 180 / M_PI, rollAngleRange[MIN], rollAngleRange[MAX]);
    // Reverse the range of the accelerometer axes
    controller.pitchDeg *= -1;
    controller.rollDeg *= -1;
}
void print_acc_values(){
    // Prints out accelerometer values to serial

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(acc.x); Serial.print('\t');
    Serial.print(acc.y); Serial.print('\t');
    Serial.print(acc.z); Serial.print('\t');
    Serial.print(gyro.x); Serial.print('\t');
    Serial.print(gyro.x); Serial.print('\t');
    Serial.print(gyro.x); Serial.print('\t');
    Serial.print("Pitch (degrees): ");
    Serial.print(controller.pitchDeg); Serial.print('\t');
    Serial.print("Roll (degrees): ");
    Serial.println(controller.rollDeg);
}
void angle_to_pwm(){
    // Convert angle value to PWM

    // Convert angle values to PWM with one-to-one mapping

    // Apply exponential curve to angle inputs
    controller.rollDeg = input_exponential(controller.rollDeg,
                                          rollAngleRange[MIN],
                                          rollAngleRange[MAX]);
    controller.pitchDeg = input_exponential(controller.pitchDeg,
                                           pitchAngleRange[MIN],
                                           pitchAngleRange[MAX]);

    // Update the PWM payload values
    payload.roll = constrain(map(controller.rollDeg,
                       rollAngleRange[MIN],
                       rollAngleRange[MAX],
                       pwmRange[MIN],
                       pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
    payload.pitch = constrain(map(controller.pitchDeg,
                        pitchAngleRange[MIN],
                        pitchAngleRange[MAX],
                        pwmRange[MIN],
                        pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
}

// Filters out deviations in input data according to supplied alpha constant
int alpha_filter(int currentInput, int previousValue, double alpha){
        return ((1 - alpha) * previousValue) + (alpha * currentInput);
}
// Applies an exponential curve to the control input response
int input_exponential(long int value, int inputMin, int inputMax){
    
    // Return the exact input value for a linear response
    if(expo == 0){
        return value;
    }
    // Calculate exponent magnitude
    double exponent = 3 * (expo/100.00);
    // Store endpoints of exponential range
    long int min = pow( -   100,        3);
    long int max = pow(100, 3);
    // Map the input value into the exponential range
    value = map(value,
            inputMin,
            inputMax,
            -100,
            100);
    // Apply the exponential function to the input value
    value = pow(value, exponent);
    // Return the modified value and map it back into it's original range
    return map(value,
            min,
            max,
            inputMin,
            inputMax);
}
// Consolidates the PWM data from various sources and updates the payload[] array
void aggregate_payload(){
    switch(controller.control){
        case JOYSTICKS:
            payload.yaw = constrain(map(joy.yaw,
                    yawStickRange[MIN],
                    yawStickRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            payload.throttle = constrain(map(joy.throttle,
                    throttleStickRange[MAX],
                    throttleStickRange[MIN],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            payload.roll = constrain(map(joy.roll,
                    rollStickRange[MIN],
                    rollStickRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            payload.pitch = constrain(map(joy.pitch,
                    pitchStickRange[MIN],
                    pitchStickRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            break;

        case ACCELEROMETER:
            /*
            // Update yaw and throttle from joysticks
            payload.yaw = constrain(map(joy.yaw,
                    yawStickRange[MIN],
                    yawStickRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            payload.throttle = constrain(map(joy.throttle,
                    throttleStickRange[MIN],
                    throttleStickRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            // Use lookup table for angle to PWM conversion            
            */
            payload.yaw = constrain(map(flex.thumb,
                    thumbFlexRange[MAX],
                    thumbFlexRange[MIN],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);
            payload.throttle = constrain(map(flex.index,
                    indexFlexRange[MIN],
                    indexFlexRange[MAX],
                    pwmRange[MIN],
                    pwmRange[MAX]), pwmRange[MIN], pwmRange[MAX]);

            angle_to_pwm();
            break;
    }
}
// Prints out the payload control values to the serial port
void print_payload(){    
    Serial.print("PAYLOAD:"); Serial.print("\t");
    for(int i = 0; i < 8; i++){
        switch(i){
            case 0:
                Serial.print(" Throttle:  ");
                Serial.print(payload.throttle);
                break;
            case 1:
                Serial.print(" Yaw: ");
                Serial.print(payload.yaw);
                break;
            case 2:
                Serial.print(" Yaw Trim: ");
                Serial.print(payload.yawTrim);
                break;
            case 3:
                Serial.print(" Pitch: ");
                Serial.print(payload.pitch);
                break;
            case 4:
                Serial.print(" Roll: ");
                Serial.print(payload.roll);
                break;
            case 5:
                Serial.print(" Pitch Trim: ");
                Serial.print(payload.pitchTrim);
                break;
            case 6:
                Serial.print(" Roll Trim: ");
                Serial.print(payload.roll);
                break;
            case 7:
                Serial.print(" Fly Run: ");
                Serial.println(payload.flyRun);
                break;
        }
    }
    Serial.println(" ");
}
// Serial control protocol
void serial_control(){
    if(Serial.available())
    {
        unsigned char c=Serial.read();

        switch(c){
            // Bind to drone
            case '1':               
                Serial.print("Connecting to drone... ");
                if(drone0.inactive())
                    drone0.bind(ID0);
                delay(2000);
                if(!drone0.inactive())
                    Serial.println("SUCCESS!");
                else
                    Serial.println("BINDING ERROR");
                break;
                // Unbind from drone
            case '2':
                Serial.print("Unbinding from drone... ");
                drone0.unbind();
                if(drone0.inactive())
                    Serial.println("SUCCESS!");
                else
                    Serial.println("UNBIND ERROR");
                break;
                // Reconnect to drone
            case '3':
                Serial.print("Reconnecting to drone... ");
                drone0.reconnect(ID0);
                if(drone0.inactive())
                    Serial.println("RE-CONNECT ERROR");
                else
                    Serial.println("SUCCESS!");
                break;
                // Toggle between fly and run modes
            case '4':
                if(payload.flyRun == FLY){
                    payload.flyRun = RUN;}
                else{
                    payload.flyRun = FLY;}
                break;
                // Print out analog joystick values
            case '8':
                //print_joystick_values();
                print_flex_values();
                break;  
                // Print out accelerometer values to serial
            case '9':
                print_acc_values();
                break;
                // Print payload values to serial
            case '0':
                print_payload();
                break;  
                // Toggle between Joystick and Accelerometer control
            case '5':
                if(controller.control == JOYSTICKS){
                    controller.control = ACCELEROMETER;
                    Serial.println("ACCELEROMETER CONTROL SELECTED");
                }
                else{
                    controller.control = JOYSTICKS;
                    Serial.println("JOYSTICK CONTROL SELECTED");
                }
                break;
                // Increase trim increment value
            case '.': // '>'
                if(trimIncrement >= 20){break;}
                else{trimIncrement++;}
                break;
                // Decrease trim increment value
            case ',': // '<"
                if(trimIncrement <= 1){break;}
                else{trimIncrement--;}
                break;
                // Throttle trim UP
            case 'e':
                payload.throttleTrim = constrain(payload.throttleTrim + trimIncrement, 0, 128);
                break;
                /* Calibration of input controls
                   case 'c':
                   calibrate_input();
                 */
                // Print out calibration offsets
            case 'o':
                printOffsets();
                break;
                // Throttle trim DOWN
            case 'c':
                Serial.println("Calibrating MPU...");
                imu.calibrate(true);
                Serial.println("Calibration Done!");
                printOffsets();
                break;

            case 'd':
                payload.throttleTrim = constrain(payload.throttleTrim - trimIncrement, 0, 128);
                break;
                // Yaw trim RIGHT
            case 'f':
                payload.yawTrim = constrain(payload.yawTrim + trimIncrement, 0, 128);
                break;
                // Yaw trim LEFT
            case 's':
                payload.yawTrim = constrain (payload.yawTrim - trimIncrement, 0, 128);
                break;
                // Pitch trim UP
            case 'i':
                payload.pitchTrim = constrain(payload.pitchTrim + trimIncrement, 0, 128);
                break;
                // Pitch trim DOWN
            case 'k':
                payload.pitchTrim = constrain(payload.pitchTrim - trimIncrement, 0, 128);
                break;
                // Roll trim RIGHT
            case 'l':
                payload.rollTrim = constrain(payload.rollTrim + trimIncrement, 0, 128);
                break;
                // Roll trim LEFT 
            case 'j':
                payload.rollTrim = constrain(payload.rollTrim - trimIncrement, 0, 128);

                break;
                // Default case, do nothing
            default:
                break;
        }
    }
}































