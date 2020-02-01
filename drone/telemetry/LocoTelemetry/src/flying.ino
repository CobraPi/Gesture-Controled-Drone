#include <HCD.h>
#include <arduino.h>


#define BIND 1
#define UNBIND 2
#define RECONNECT 3
#define PAYLOAD_UPDATE 4


#define FLY 0x00
#define RUN 0x10




// Drone Object
HCD drone0;

//Drone ID's (pick 4 random numbers)
unsigned char ID0[]={
  0x16,0x01,0x55,0x11};

// Payload data:
// THROTTLE,YAW,YAW_TRIM,PITCH,ROLL,PITCH_TRIM,ROLL_TRIM,FLY/RUN
// throttle 0-255
// pitch,roll,yaw 0-255 128=middle
// trims 64=center
// fly/run 0=fly 16=run

unsigned char payload0[]={
  0x00, // Throttle
  0x7F, // Yaw
  0x40, // Yaw Trim
  0x7F, // Pitch
  0x7F, // Roll
  0x40, // Pitch Trim
  0x40, // Roll Trim
  0x00}; // FlyRun



unsigned long timer=0;




void setup() {
  Serial.begin(19200);
  Serial.write('a');
  Serial.setTimeout(1000);

  char a = 'b';
  while (a != 'a'){
    a = Serial.read();
  }
}

void parse_payload(unsigned char *payload){
    uint8_t size = Serial.parseInt();
    Serial.readBytes(payload, size);
    Serial.println(*payload);
}


void loop() {
  if(Serial.available()) {

    unsigned char c = Serial.read();

    switch(c) {

        case BIND:
            if(drone0.inactive()){
                drone0.bind(ID0);
            }
            delay(2000);
            if(!drone0.inactive()){
                Serial.write('1');
            }
            else{
                Serial.write('0');
            }
            break;

        case UNBIND:
            drone0.unbind();
            if(drone0.inactive()){
                Serial.write('1');
            }
            else{
                Serial.write('0');
            }
            break;
        case RECONNECT:
            drone0.reconnect(ID0);
            if(!drone0.inactive()){
                Serial.write('1');
            }
            else{
                Serial.write('0');
            }
            break;
        case PAYLOAD_UPDATE:
            if(!drone0.inactive()){
                parse_payload(payload0); }
            break;

        default:
            break;
    }
  }

  if(millis()>=timer) {
    timer+=20;
    drone0.update(payload0);
  }
}


































