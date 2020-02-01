//
// Created by joey on 8/21/16.
//
#include <Arduino.h>
#include <HCD.h>    // RF library
//#include <I2Cdev.h> // I2C Communication library
#include <Wire.h>   // Arduino I2C library
#include <SPI.h>
//#include <SparkFunLSM9DS1.h> // 9-DOF IMU library
#include <IMU.h>
#include <Utils.h>

// LED Lights
extern "C" {
#include <ws2812.h>
};

// LED Signal = D9
// Triac Control = D6

// PWM Range
#define PWM_MIN 0
#define PWM_MAX 255
// Flight Modes
#define FLY 0x00
#define RUN 0x0F

// Comment out for right handed use
#define LEFT_HANDED 1

#define THUMB_PIN A2
#define INDEX_PIN A1
#define MIDDLE_PIN A0

#define TRIAC_PIN 6

#define PITCH_ANGLE_RANGE 30
#define ROLL_ANGLE_RANGE 40

// Instance of drone object
HCD drone0;

// Instance of 9-dof IMU
IMU imu;

//Drone ID's (pick 4 random numbers)
unsigned char ID0[] = {0x16, 0x01, 0x55, 0x11};

// Payload data:
// THROTTLE,YAW,YAW_TRIM,PITCH,ROLL,PITCH_TRIM,ROLL_TRIM,FLY/RUN
// throttle 0-255
// pitch,roll,yaw 0-255 128=middle
// trims 64=center
// fly/run 0=fly 15=run

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
} payload;

struct flexSensors {
    int thumb = 0;
    int index = 0;
    int middle = 0;

    int raw_thumb = 0;
    int raw_index = 0;
    int raw_middle = 0;

    int thumb_range_min = 0;
    int thumb_range_max = 1023;

    int index_range_min = 0;
    int index_range_max = 1023;

    int middle_range_min = 0;
    int middle_range_max = 1023;

} flex;

struct response_constants {
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

} response;



void read_flex();
void calibrate_flex();
void print_flex_values();
void print_payload();
void angle_to_pwm();
void aggregate_payload();
void print_accValues();
void flash_triac(int speed, int iterations);

// LED color enum
enum RGB {R, G, B};

// LED colors
uint8_t off[] = {0, 0, 0};
uint8_t white[] = {40, 40, 40};
uint8_t red[] = {40, 0, 0};
uint8_t green[] = {0, 40, 0};
uint8_t blue[] = {0, 0, 40};

uint8_t amber[] = {40, 80, 0};
uint8_t dark_orchid[] = {153, 50, 204};

uint8_t green_blue[] = {0, 35, 10};

uint8_t pitch_color[] = {0, 10, 35};
uint8_t roll_color[] = {35, 10, 0};

// LED Functions
void leds_setColorAll(uint8_t *color);
void leds_pulse(uint8_t *color, uint32_t speed);
void leds_cycleOn(uint8_t *color, uint32_t speed, bool direction);
void leds_tiltMeter(uint8_t *pitchColor, uint8_t *rollColor);
void rainbow_wheel(int led_num, int wheel_val);
void leds_rainbow(int rotation_speed);

bool serial_open = true;

enum YAW_MODE {NORMAL, LOCKOUT, DEADBAND};

YAW_MODE current_yaw_mode = LOCKOUT;

void setup() {

    // Initialize LED's
    init_leds();
    delay(1000);
    leds_rainbow(10);

    Serial.begin(115200);
    // Keep checking for the Serial object
    // to be created
    int serial_time = 0;

    leds_setColorAll(off);

    while(!Serial) {
        if (serial_time >= 5) {
            serial_open = false;
            for (int i = 0; i < 16; i++) {
                leds_pulse(dark_orchid, 80);
            }
            break;
        }
        serial_time++;
        leds_setColorAll(off);
    }


    pinMode(THUMB_PIN, INPUT);
    pinMode(INDEX_PIN, INPUT);
    pinMode(MIDDLE_PIN, INPUT);

    pinMode(TRIAC_PIN, OUTPUT);

    calibrate_flex();

    digitalWrite(TRIAC_PIN, HIGH);

    imu.init();
    delay(10);

    if(!serial_open) {

        drone0.bind(ID0);
    }
    else {

        for (int i = 0; i < 16; i++) {
            leds_pulse(amber, 80);
        }
        leds_setColorAll(off);

        // Serial Welcome Message
        Serial.println(""); Serial.println("");
        Serial.println("------------- Welcome to AURA --------------"); Serial.println("");
        Serial.println("Press: ");
        Serial.println("1 to bind");
        Serial.println("2 to re-connect");
        Serial.println("3 to un-bind");
        Serial.println("4 to toggle standard yaw mode (default)");
        Serial.println("5 to toggle deadband yaw mode");
        Serial.println("6 to toggle lockout yaw mode");
    }

}

unsigned long timer = 0;

void loop() {


    if (Serial.available()) {
        unsigned char c = Serial.read();

        switch (c) {

            case '1':
                Serial.println("Binding...");
                drone0.bind(ID0);
                break;

            case '2':
                Serial.println("Re-Connecting...");
                drone0.reconnect(ID0);
                break;

            case '3':
                Serial.println("Un-Binding...");
                drone0.unbind();
                break;

            case '4':
                current_yaw_mode = NORMAL;
                break;

            case '5':
                current_yaw_mode = DEADBAND;
                break;

            case '6':
                current_yaw_mode = LOCKOUT;
                break;
            case '8':
                print_flex_values();
                break;

            case '9':
                print_accValues();
                break;

            case '0':
                print_payload();
                break;
        }
    }


    imu.read_acc();
    imu.cal_absoluteOrientation();
    leds_tiltMeter(pitch_color, roll_color);
    read_flex();
    aggregate_payload();


    if (millis() >= timer) {
        timer += 20;
        drone0.update(payload.throttle,
                      payload.yaw,
                      payload.yawTrim,
                      payload.pitch,
                      payload.roll,
                      payload.pitchTrim,
                      payload.rollTrim,
                      payload.flyRun);
    }

}


void print_accValues() {
    imu.read();
    imu.cal_absoluteOrientation();

    int16_t pitch;
    int16_t roll;
    int16_t heading;
    float accG[3];

    imu.get_pitchDegrees(pitch);
    imu.get_rollDegrees(roll);
    imu.get_heading(heading);
    imu.get_accG(accG);

    Serial.print("Acceleration in G's: ");
    Serial.print("X[ "); Serial.print(accG[X]); Serial.print(" ] ");
    Serial.print("Y[ "); Serial.print(accG[Y]); Serial.print(" ] ");
    Serial.print("Z[ "); Serial.print(accG[Z]); Serial.print(" ] ");
    Serial.print("    ");
    Serial.print("Pitch: "); Serial.print(pitch); Serial.print("  ");
    Serial.print("Roll: "); Serial.print(roll); Serial.print("  ");
    Serial.print("Heading: "); Serial.println(heading);
}

void read_flex(){
    flex.raw_thumb = analogRead(THUMB_PIN);
    flex.raw_index = analogRead(INDEX_PIN);
    flex.raw_middle = analogRead(MIDDLE_PIN);

    flex.thumb = alphaFilter(flex.raw_thumb, flex.thumb, response.flexAlpha);
    flex.index = alphaFilter(flex.raw_index, flex.index, response.flexAlpha);
    flex.middle = alphaFilter(flex.raw_middle, flex.middle, response.flexAlpha);
}

void calibrate_flex() {
    int thumb = 0;
    int index = 0;
    int middle = 0;

    leds_setColorAll(off);

    for (int i = 0; i < 8; i++) {
    leds_pulse(green, 80);
    }

    leds_setColorAll(off);

    //Serial.print("Open your hand and press 'c' when done...");
    for (int i = 0; i < 8; i++) {
        thumb += analogRead(THUMB_PIN);
        index += analogRead(INDEX_PIN);
        middle += analogRead(MIDDLE_PIN);
        set_led(i, 0, 0, 40);
        sync_leds();
        delay(1000);
    }

    flex.thumb_range_max = thumb / 8;
    flex.index_range_max = index / 8;
    flex.middle_range_max = middle / 8;

    //Serial.println("Done");
    thumb = 0;
    index = 0;
    middle = 0;

    leds_setColorAll(off);

    for (int i = 0; i < 8; i++) {
    leds_pulse(green, 80);
    }

    leds_setColorAll(off);

    //Serial.print("Close your hand and press 'c' when done...");
    set_led(0, 40, 0, 0);
    sync_leds();
    for (int i = 8; i > 0; i--) {
        thumb += analogRead(THUMB_PIN);
        index += analogRead(INDEX_PIN);
        middle += analogRead(MIDDLE_PIN);
        set_led(i, 40, 0, 0);
        sync_leds();
        delay(1000);
    }
    flex.thumb_range_min = thumb / 8;
    flex.index_range_min = index / 8;
    flex.middle_range_min = middle / 8;
    //Serial.println("Done");
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
                Serial.print(payload.rollTrim);
                break;
            case 7:
                Serial.print(" Fly Run: ");
                Serial.println(payload.flyRun);
                break;
        }
    }
    Serial.println(" ");
}

void angle_to_pwm(){
    // Convert angle value to PWM
    int16_t pitch;
    int16_t roll;

    imu.get_pitchDegrees(roll);
    imu.get_rollDegrees(pitch);

    // Apply exponential curve to angle inputs


    pitch = inputExponential(response.accExpo,
                             pitch,
                             -PITCH_ANGLE_RANGE,
                             PITCH_ANGLE_RANGE);

    roll = inputExponential(response.accExpo,
                            roll,
                            -ROLL_ANGLE_RANGE,
                            ROLL_ANGLE_RANGE);

    // Update the PWM payload values


    payload.pitch = constrain(map(pitch,
                                 -PITCH_ANGLE_RANGE,
                                 PITCH_ANGLE_RANGE,
                                 PWM_MIN, PWM_MAX),
                                 PWM_MIN, PWM_MAX);

    payload.roll = constrain(map(roll,
                                 -ROLL_ANGLE_RANGE,
                                 ROLL_ANGLE_RANGE,
                                 PWM_MAX, PWM_MIN),
                                 PWM_MIN, PWM_MAX);
}

void aggregate_payload(){



    payload.throttle = constrain(map(flex.index,
                                flex.index_range_min,
                                flex.index_range_max,
                                PWM_MIN, PWM_MAX),
                                PWM_MIN, PWM_MAX);

    switch(current_yaw_mode) {

        // Standard yaw control w/ no expo
        case NORMAL:

            payload.yaw = constrain(map(flex.thumb,
                                    flex.thumb_range_max,
                                    flex.thumb_range_min,
                                    PWM_MIN, PWM_MAX),
                                    PWM_MIN, PWM_MAX);
            break;

        // Apply exponential curve to the yaw control
        case DEADBAND:
             payload.yaw = inputExponential(response.flexYawExpo,
                                            payload.yaw,
                                            flex.thumb_range_max,
                                            flex.thumb_range_min);

             payload.yaw = constrain(map(flex.thumb,
                                    flex.thumb_range_max,
                                    flex.thumb_range_min,
                                    PWM_MIN, PWM_MAX),
                                    PWM_MIN, PWM_MAX);


            break;

        // Lockout the yaw control and set it to center
        case LOCKOUT:
             int yaw_mode = constrain(map(flex.middle,
                                flex.middle_range_min,
                                flex.middle_range_max,
                                PWM_MIN, PWM_MAX),
                                PWM_MIN, PWM_MAX);

            if(yaw_mode > 127) {
                payload.yaw = 127;
            }
            else{
            // Set yaw to center PWM value
            payload.yaw = inputExponential(response.flexYawExpo,
                                            payload.yaw,
                                            flex.thumb_range_max,
                                            flex.thumb_range_min);

            payload.yaw = constrain(map(flex.thumb,
                                    flex.thumb_range_max,
                                    flex.thumb_range_min,
                                    PWM_MIN, PWM_MAX),
                                    PWM_MIN, PWM_MAX);
            }
            break;
    }

    angle_to_pwm();

}

void leds_setColorAll(uint8_t *color) {

    set_leds(color[R], color[G], color[B]);
    sync_leds();
}


void leds_pulse(uint8_t *color, uint32_t speed) {
   leds_setColorAll(off);
   delay(speed);
   leds_setColorAll(color);
   delay(speed);
}

void leds_cycleOn(uint8_t *color, uint32_t speed, bool direction) {

    leds_setColorAll(off);

    uint8_t min = 0;
    uint8_t max = ws2812_count;

    if(direction) {
        for (int i = min; i < max; i++) {
            set_led(i, color[R], color[G], color[B]);
            sync_leds();
            delay(speed);
        }
    }
    else {
        for (int i = max; i > min; i--) {
            set_led(0, color[R], color[G], color[B]);
            set_led(i, color[R], color[G], color[B]);
            sync_leds();
            delay(speed);
        }
    }
}

void leds_tiltMeter(uint8_t *pitchColor, uint8_t *rollColor) {

    float axis_init = 10;
    int16_t pitch;
    int16_t roll;

    imu.get_pitchDegrees(pitch);
    imu.get_rollDegrees(roll);

    uint8_t number_of_lights = 0;

    // Roll Case
    if (abs(roll) > axis_init) {
        number_of_lights = map(abs(roll),
                               0,
                               ROLL_ANGLE_RANGE,
                               0, 8);
        number_of_lights = constrain(number_of_lights, 0, 8);
        if (roll > 0) {
            for (int i = 0; i < number_of_lights; i++) {
                set_led(i, rollColor[R], rollColor[G], rollColor[B]);
            }
        }
        else {

            set_led(0, rollColor[R], rollColor[G], rollColor[B]);
            for (int i = ws2812_count; i > ws2812_count - number_of_lights; i--) {
                 set_led(i, rollColor[R], rollColor[G], rollColor[B]);
            }
        }
    }


    // Pitch Case
    else if (abs(pitch) > axis_init) {
        number_of_lights = map(abs(pitch),
                               0,
                               PITCH_ANGLE_RANGE,
                               0, 8);
        number_of_lights = constrain(number_of_lights, 0, 8);
         if (pitch > 0) {
            for (int i = 0; i < number_of_lights; i++) {
                set_led(i, pitchColor[R], pitchColor[G], pitchColor[B]);
            }
        }
        else {
            set_led(0, pitchColor[R], pitchColor[G], pitchColor[B]);
            for (int i = ws2812_count; i > ws2812_count - number_of_lights; i--) {
                 set_led(i, pitchColor[R], pitchColor[G], pitchColor[B]);
            }
        }
    }

    else {
        leds_setColorAll(off);
        set_leds(green_blue[R], green_blue[G], green_blue[B]);
    }

    sync_leds();
}

void rainbow_wheel(int led_num, int wheel_val) {

  wheel_val = 255 - wheel_val;

  if (wheel_val < 170) {
  set_led(led_num, 255 - wheel_val * 3, 0, wheel_val * 3);
  }
  else if (wheel_val < 170) {
  wheel_val -= 85;
  set_led(led_num, 0, wheel_val * 3, 255 - wheel_val * 3);
  }
  else {
    wheel_val -= 170;
    set_led(led_num, wheel_val * 3, 255 - wheel_val * 3, 0);
  }
}

void leds_rainbow(int rotation_speed) {

  for (int j = 0; j < 256 * 5; j+=20) {

    for (int i = 0; i < 8; i++) {

      int color_value = int(floor((i * 256 / 8) + j)) & 255;

      rainbow_wheel(i, color_value);

      sync_leds();

      delay(rotation_speed);
    }

    delay(rotation_speed);
  }
}


void flash_triac(int speed, int iterations) {
    int time = 0;

    while(iterations > 0) {
        digitalWrite(TRIAC_PIN, HIGH);
        delay(speed);
        digitalWrite(TRIAC_PIN, LOW);
        delay(speed);
        iterations--;
    }

    digitalWrite(TRIAC_PIN, HIGH);
}



