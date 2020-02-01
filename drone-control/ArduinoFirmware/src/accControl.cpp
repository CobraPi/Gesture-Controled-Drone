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

// Libraries
#include <Arduino.h>

#include <MathUtils.h>
#include <HCD.h> // RF Library
#include <I2Cdev.h> // I2C communication library
#include <Wire.h> // Arduino I2C library
#include <IMU.h> // Accelerometer library
#include <Joystick.h> // Accelerometer library
#include <LocoDrone.h>
#include <Controller.h>
#include <Messages.h> // Serial communication codes
#include "../lib/Controller/Controller.h"

// LED lights
extern "C"{
#include <ws2812.h>
};

extern LocoDrone drone;
extern HCD rfm;
void debug(char *message);

unsigned long timer = 0;
//extern "C"{
//#include <IMU.h>
//};

extern ControlSourceType currentType;

// Serial control protocol
void processSerial();
// Payload debugger
void debugPayload();
void debug(char* msg);

void setup(){
    Serial.begin(115200);
    Serial.setTimeout(10000);
    imu_init();
    //delay(10u00);
    //imu_calibrateAccelerometer();
    joystick_init();

}


void loop(){
//    char c = Serial.read();
//    if(c == 'c'){
//        imu_calibrateAccelerometer();
//    }
//    delay(100);

    processSerial();
    controller_controlLoop();

    if(millis() >= timer && currentType != ControlSourceSerial){
        timer += 20;
        drone.sendPayload();
    }
}

void debugPayload() {
    uint8_t payload[] = {drone.getThrottle(),
                         drone.getYaw(),
                         drone.getYawTrim(),
                         drone.getPitch(),
                         drone.getRoll(),
                         drone.getPitchTrim(),
                         drone.getRollTrim(),
                         drone.getFlyDrive()};

    Serial.write(OutgoingMessageDebug);
    Serial.write(8);
    Serial.write(payload, 8);
}

void handleBind(uint8_t *data, uint8_t _length) {
    drone.bind();

    for (int i = 0; i < 10000; i++) {
        delay(10);

        if (drone.is_active()) {
            break;
        }

        drone.reconnect();
    }

    uint8_t packet[2];

    if(drone.is_active()){
        packet[0] = OutgoingMessageBindSuccess;
    }
    else{
        packet[0] = OutgoingMessageBindFailure;
    }

    packet[1] = 0;
    Serial.write(packet, sizeof(packet));
}

void handleUnbind(uint8_t *data, uint8_t _length) {
    drone.unbind();

    uint8_t packet[2];

    if(drone.is_active()){
        packet[0] = OutgoingMessageUnbindFailure;
    }
    else{
        packet[0] = OutgoingMessageUnbindSuccess;
    }

    packet[1] = 0;
    Serial.write(packet, sizeof(packet));
}

void handleReconnect(uint8_t *data, uint8_t _length) {
    drone.reconnect();

    uint8_t packet[2];

    if(drone.is_active()){
        packet[0] = OutgoingMessageReconnectSuccess;
    }
    else{
        packet[0] = OutgoingMessageReconnectFailure;
    }

    packet[1] = 0;
    Serial.write(packet, sizeof(packet));
}

void handleOperatingMode(uint8_t *data, uint8_t _length) {
    uint8_t mode = data[0];

    switch(mode) {
        case OperatingModeFly:
            drone.setFlyDrive(OperatingModeFly);
            break;
        case OperatingModeDrive:
            drone.setFlyDrive(OperatingModeDrive);
            break;
    }
}

void handlePing(uint8_t *data, uint8_t _length) {
    uint8_t msg[2];
    msg[0] = OutgoingMessagePing;
    msg[1] = 0;

    Serial.write(msg, sizeof(msg));

}

void handleControlPayload(uint8_t *data, uint8_t _length) {
    if (_length == 8) {
        rfm.update(data);
    }
}

void handleControlSource(uint8_t *data, uint8_t _length) {
    uint8_t mode = data[0];

    switch (mode) {
        case ControlSourceJoystick:
            controller_setController(ControlSourceJoystick);
            break;
        case ControlSourceAccelerometer:
            controller_setController(ControlSourceAccelerometer);
            break;
        case ControlSourceSerial:
            controller_setController(ControlSourceSerial);
            break;
    }
}




// Serial control protocol
void processSerial(){
    if(Serial.available() >= 2) {
        uint8_t code = (uint8_t) Serial.read();
        uint8_t length = (uint8_t) Serial.read();

        uint8_t data[length];

        if (length > 0) {
            Serial.readBytes(data, length);
        }

        switch(code){
            case IncomingMessageBind:
                // Bind to drone
                handleBind(data, length);
                break;

            case IncomingMessageUnbind:
                // Unbind from drone
                handleUnbind(data, length);
                break;

            case IncomingMessageReconnect:
                // Reconnect to drone
                handleReconnect(data, length);
                break;

            case IncomingMessageOperatingMode:
                // Toggle FlyDrive Mode
                handleOperatingMode(data, length);
                break;

            case IncomingMessageControlSource:
                // Parse payload values from python
                handleControlSource(data, length);
                break;

            case IncomingMessageControlPayload:
                //Parse payload values from python
                handleControlPayload(data, length);
                //debugPayload();
                break;

            case IncomingMessageEcho:
                // Verify communication
                handlePing(data, length);
                break;

            case IncomingMessageCalibrateAccelerometer:
                // Calibrate Accelerometer
                imu_calibrateAccelerometer();
                break;

        }
    }
}

void debug(char *message) {
    uint8_t len = strlen(message);
    uint8_t packet[2 + len];

    packet[0] = OutgoingMessageDebug;
    packet[1] = len;

    for (int i = 0; i < len; i++) {
        packet[i + 2] = (uint8_t) message[i];
    }

    Serial.write(packet, sizeof(packet));
}
