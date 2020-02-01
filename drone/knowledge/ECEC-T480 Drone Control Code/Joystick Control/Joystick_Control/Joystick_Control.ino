/*
 * By: Joseph Orduna
 * Joystick Control 
 * 
 *
 *
*/

// RF communciation library
#include <HCD.h>

// Create drone object
HCD drone0; 

// Left joystick pins
#define LEFT_VERTICAL_PIN A0
#define LEFT_HORIZONTAL_PIN A1
#define LEFT_BUTTON_PIN A2

// Right joystick pins
#define RIGHT_VERTICAL_PIN A3
#define RIGHT_HORIZONTAL_PIN A4
#define RIGHT_BUTTON_PIN A5

// Trim Switches
#define YAW_TRIM_PIN 5
#define PITCH_ROLL_TRIM_PIN 6

// Serial speed
#define SERIAL_SPEED 19200

#define PRINT_LOGS 0


// Joystick analog input parameters
// MIN = 0, MID = 512, MAX = 1023
#define JOYSTICK_MIN = 0
#define JOYSTICK_MID = 512
#define JOYSTICK_MAX = 1023

// Joystick pin array
const unsigned int JOYSTICK_PINS[] = {
  // Left joystick
  LEFT_VERTICAL_PIN, LEFT_HORIZONTAL_PIN, 
  // Yaw trim pin
  YAW_TRIM_PIN,
  // Right joystick
  RIGHT_VERTICAL_PIN, RIGHT_HORIZONTAL_PIN,
  // Pitch and Roll trim
  PITCH_ROLL_TRIM_PIN,
  // Left and Right buttons
  LEFT_BUTTON_PIN, RIGHT_BUTTON_PIN
};

// Payload enumeration
enum PAYLOAD {THROTTLE, YAW, YAW_TRIM, PITCH, ROLL, PITCH_TRIM, ROLL_TRIM, FLY_RUN};

// Trim center values
unsigned int yawTrimCenter = 64, pitchTrimCenter = 57, rollTrimCenter = 64;

unsigned long timer = 0;
// Execution counter aray used to limit the duration of a pitch or yaw
unsigned long exe_ctr[8];

const unsigned int THROTTLE_INC = 1;
const unsigned int CENTER       = 128;
const unsigned int MAX          = 255;
const unsigned int MIN          = 0;

unsigned int YAW_OFFSET_LOW = 60;
unsigned int YAW_OFFSET_MED = 85;
unsigned int YAW_OFFSET_HI  = 110;

unsigned int PITCH_OFFSET_LOW = 23;
unsigned int PITCH_OFFSET_MED = 42;
unsigned int PITCH_OFFSET_HI  = 90;

float YAW_DURR = 0.02;
float PITCH_DURR = 0.02;

unsigned int FOLLOW_PITCH = 15;
unsigned int FOLLOW_YAW   = 125;

// Drone ID's (pick 4 random numbers)
unsigned char ID0[] = {
  0x16, 0x22, 0x55, 0x01
};

/* Payload data:
//  payload[0]=throttle;        throttle 0-255
//  payload[1]=yaw;             yaw      0-255 128=middle
//  payload[2]=yawtrim;         trims    0-128 64=center
//  payload[3]=pitch;           pitch    0-255 128=middle
//  payload[4]=roll;            roll     0-255 128=middle
//  payload[5]=pitchtrim;       trims    0-128 64=center
//  payload[6]=rolltrim;        trims    0-128 64=center
//  payload[7]=flyrun;          fly/run  0=fly 16=run
*/
unsigned char payloadx[] = {
  0x00, 0x80, yawTrimCenter, 0x80, 0x80, pitchTrimCenter, rollTrimCenter, 0x00
};

void setup(){
 
 // Fill the execution counter arrays with 0's
 for(int i = 0; i < 8; i++){
  exe_ctr[i] = 0;
 }
 
 // Set the serial speed
  Serial.begin(SERIAL_SPEED);
  delay(1000);

  // Set the joystick pins to INPUT
  for(int i = 0; i < 8; i++){
    pinMode(JOYSTICK_PINS[i], INPUT); 
  }
  
  Serial.println("Press 0 to bind to HCD");

}

// Increment a flight control parameter to maximum
void inc255(int idx) 
{
  if (payloadx[idx] < 255) {
    payloadx[idx] += 0x01;
  }
}
// Decrement a flight control parameter to minimum
void dec255(int idx)
{
  if (payloadx[idx] > 0) {
    payloadx[idx] -= 0x01;
  }
}
// Decrement a flight control paramter to center
void inc128(int idx)
{
  if (payloadx[idx] < 128) {
    payloadx[idx] += 0x01;
  }
}
// Decrement a flight control paramter to center
void dec128(int idx)
{
  if (payloadx[idx] > 0) {
    payloadx[idx] -= 0x01;
  }
}
// Set a flight control paramter to a desired value
void Change_Payload(int idx, int new_speed)
{
  payloadx[idx] = new_speed;
}
// Set a flight control paramter to a desired value, 
// for a given duration
void Change_Payload(int idx, int new_speed, float durr)
{
  payloadx[idx] = new_speed;
  exe_ctr[idx] = (durr * 50);
}
// Center a flight control value
void Make_Center( int cmd )
{
  Change_Payload( cmd,   128 );
}

// process the raw value outputted by the analog potentiometer
// Increase/ Decrease sensitivity w/ exponential
// MAX = 1023, MIN = 0


// Reads and converts joystick values and stores them in the payload array
void readJoystick(){
  // Raw value read from joystick
  long int value = 0;
  // Interpreted value to be fed into payload array
  long int result = 0;
  for(int i = 0; i < 5; i++){
    // 0 = Throttle, 3 = Pitch
    if(i == 0 || i == 3){
      //
      value = analogRead(JOYSTICK_PINS[i]) - 516;
     value = exp(
     result = map(value, -512, 512, 0, 255); 
     Change_Payload(i, result);
    }
    //      1 = Yaw,  4 = Roll
    else if(i == 1 || i == 4){
     value = analogRead(JOYSTICK_PINS[i]) - 520;
     result = map(value, 512, -512, 0, 255);
     Change_Payload(i, result); 
    }
    
    
//    switch(i){  
//      // Read throttle
//      case 0:
//        value = map(analogRead(JOYSTICK_PINS[i]), 0, 1023, 0, 255);
//        // Convert value into [0-255] range
//        Change_Payload(i, value);
//        
//        break;
//      
//      // Read yaw 
//      case 1:
//        value = map(analogRead(JOYSTICK_PINS[i]) - JOYSTICK_MID, 1023, 0, 0, 255);
//        // Convert value into [0-255] range
//        Change_Payload(i, value);
//        break;
//      
//      // Read pitch
//      case 3:
//        value = map(analogRead(JOYSTICK_PINS[i]) - JOYSTICK_MID, 0, 1023, 0, 255);
//        // Convert value into [0-255] range
//        Change_Payload(i, value);
//        break;
//      
//      // Read roll
//      case 4: 
//        value = map(analogRead(JOYSTICK_PINS[i]) - JOYSTICK_MID, 1023, 0, 0, 255);
//        // Convert value into [0-255] range
//        Change_Payload(i, value);
//        break;  
//      
//      // Default case - do nothing
//      default:
//        break;
//     }
  }
}

// Sets the trim of the flight controls: 
void adjust_trim(){
  unsigned int left_button = analogRead(LEFT_BUTTON_PIN);
  unsigned int right_button = analogRead(RIGHT_BUTTON_PIN);
  bool yawPin = digitalRead(YAW_TRIM_PIN);
  bool rollPitchPin = digitalRead(PITCH_ROLL_TRIM_PIN);

  // ********************** YAW TRIM ***********************
  // If the yaw switch is set to the left, modify yaw trim
  if(!yawPin){
    // Decrement the yaw trim center with left joystick button
    if(left_button == 0){
      Change_Payload(YAW_TRIM, yawTrimCenter--);
    }
    // Increment the yaw trim center with right joystick button
    else if(right_button == 0){
      Change_Payload(YAW_TRIM, yawTrimCenter++);  
    }
  }
  // If the yaw switch is set to the right, modify roll and pitch trim
  else{
    
    // ********************** ROLL TRIM ************************
    // If the roll-pitch switch is set to the left, modify roll
    if(!rollPitchPin){     
      // Decrement the roll trim center with left joystick button
      if(left_button == 0){
        Change_Payload(ROLL_TRIM, rollTrimCenter--);
      }
      // Increment the roll trim center with right joystick button 
      else if(right_button == 0){
        Change_Payload(ROLL_TRIM, rollTrimCenter++);
      }
    }

    // ********************** PITCH TRIM ************************
    // If the roll-pitch switch is set to the right, modify pitch
    else if(rollPitchPin){
      // Decrement the pitch trim center with left joystick button
      if(left_button == 0){
        Change_Payload(PITCH_TRIM, pitchTrimCenter--);
      }
      // Increment the roll pitch trim center with right joystick button
      else if(right_button == 0){
        Change_Payload(PITCH_TRIM, pitchTrimCenter++);
      }  
    }
  }
}
  


// Prints out the analog values for both joysticks
void printJoystickValues(int offset = 0){
  Serial.println("----------------------------------------------------------");
  // Left and right vertical values printed across the same line
  Serial.print("Left Vertical Value:   ");
  Serial.print(analogRead(LEFT_VERTICAL_PIN) - offset);
  Serial.print("    |   "); // 3 spaces
  Serial.print("Right Vertical Value:   ");
  Serial.println(analogRead(RIGHT_VERTICAL_PIN) - offset);
  // Left and right horizontal values printed across the same line
  Serial.print("Left Horizontal Value: ");
  Serial.print(analogRead(LEFT_HORIZONTAL_PIN) - offset);
  Serial.print("    |   "); // spaces
  Serial.print("Right Horizontal Value: ");
  Serial.println(analogRead(RIGHT_HORIZONTAL_PIN) - offset);
  // Left and right vertical values printed across the same line
  Serial.print("Left Button Value:     ");
  Serial.print(analogRead(LEFT_BUTTON_PIN) - offset);
  Serial.print("     |   "); // spaces
  Serial.print("Right Button Value:     ");
  Serial.println(analogRead(RIGHT_BUTTON_PIN) - offset);
  Serial.println("-------------------------------------------------------------");
}

// Print converted joystick values
void printPayloadValues(unsigned char payload[]){
  Serial.println("-------------------------------------------------------------");
  for(int i = 0; i < 8; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.println(payload[i]);
  }
}

///** MAIN LOOP **///
void loop(){

  if (Serial.available())
  {
    unsigned char c = Serial.read();
    switch (c)
    {
      case '0':              //-Press zero to bind
        if (drone0.inactive())
        {
          drone0.bind(ID0);
        }
        break;
     
      case '1':
        drone0.unbind();
        
      default:
      break;
    }
  }
  
  
  // Read the joystick values and update the payload
  readJoystick();
  
  
  // Adjust trim
  adjust_trim();

  // Update payload data
  //drone0.update(payloadx);
  
  // Print the joystick and payload values through serial
  printJoystickValues();
  //printPayloadValues(payloadx);
  delay(500);
  //Serial.println(digitalRead(YAW_TRIM_PIN));
  //Serial.println(digitalRead(PITCH_ROLL_TRIM_PIN));

////////////////////////////////
//////// Payload Update ////////
////////////////////////////////
  if (millis() >= timer)
  {
    timer += 20;
    drone0.update(payloadx);
    
//    for (int i = 0; i < 8; i++)   // Runs through execution timer array and  
//    {                             // decrements every 20 miliseconds if necessary.
//      if ( exe_ctr[i] > 0 )       // 
//      {                           // - Usefull if phone connection is lost so the 
//        exe_ctr[i]--;             // - drone does not continue to pitch/yaw out 
//        if ( exe_ctr[i] == 0 )    // - of control.
//        {                         //
//          Make_Center( i );       // As each individual timer reaches '0' 
//        }                         // that payload element returns to center.
//      }                           //
//    }                             //

  }
  
}


