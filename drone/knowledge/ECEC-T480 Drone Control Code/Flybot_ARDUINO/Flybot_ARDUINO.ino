#include <bk2421.h>
#include <blocks.h>
#include <HCD.h>

/* By Matt Fairfield
 * - mcf62@drexel.edu
 * - 3/1/2016
 * -- make sure to add my version of HCD.zip to Arduino libraries as there have been some minor changes from the online version
 * -- code adapted from http://dzlsevilgeniuslair.blogspot.com/2013/11/more-toy-quadcopter-hacking.html
 */



HCD drone0;

enum PAYLOAD {THROTTLE, YAW, YAW_TRIM, PITCH, ROLL, PITCH_TRIM, ROLL_TRIM, FLY_RUN};
unsigned int yTrimCenter = 64, pTrimCenter = 57;
unsigned long timer = 0;   
unsigned long exe_ctr[8];  /* Execution Counter array used to limit the druation of a pitch or yaw*/ 
 const unsigned int THROTTLE_INC = 1;
 const unsigned int CENTER       = 128;
 const unsigned int MAX          = 255;
 const unsigned int MIN          = 0;
unsigned int YAW_OFFSET_low   = 60;/* 0-255 */
unsigned int YAW_OFFSET_Med   = 85;/* 0-255 */
unsigned int YAW_OFFSET_HI    = 110;/* 0-255 */
unsigned int PITCH_OFFSET_low = 23; /* 0-255 */
unsigned int PITCH_OFFSET_Med = 42; /* 0-255 */
unsigned int PITCH_OFFSET_HI  = 90; /* 0-255 */
       float YAW_DURR         = 0.02;/* seconds */
       float PITCH_DURR       = 0.02;/* seconds */
unsigned int FOLLOW_PITCH     = 15; /* 0-255 */
unsigned int FOLLOW_YAW       = 125; /* 0-255 */

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
  0x00, 0x80, yTrimCenter, 0x80, 0x80, pTrimCenter, 0x40, 0x00
};

void setup()
{
  for (int i  = 0; i < 8; i++)
  {
    exe_ctr[i] = 0;
  }
  Serial.begin(19200);
  delay(1000);
}
void inc255(int idx) 
{
  if (payloadx[idx] < 255) {
    payloadx[idx] += 0x01;
  }
}
void dec255(int idx)
{
  if (payloadx[idx] > 0) {
    payloadx[idx] -= 0x01;
  }
}
void inc128(int idx)
{
  if (payloadx[idx] < 128) {
    payloadx[idx] += 0x01;
  }
}
void dec128(int idx)
{
  if (payloadx[idx] > 0) {
    payloadx[idx] -= 0x01;
  }
}
void Change_Payload(int idx, int new_speed)
{
  payloadx[idx] = new_speed;
}
void Change_Payload(int idx, int new_speed, float durr)
{
  payloadx[idx] = new_speed;
  exe_ctr[idx] = (durr * 50);
}

void Make_Center( int cmd )
{
  Change_Payload( cmd,   128 );
}

//////** MAIN LOOP **//////
void loop()
{
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

      ///////////////////////////////////////
      /////////* THROTTLE CONTROLS */////////
      ///////////////////////////////////////
      case '9':              //Throttle Shutoff
        Change_Payload( THROTTLE, MIN );
        break;

      case '8':              //Throttle Launch 
        Change_Payload( THROTTLE, 200 );
        break;

      case '7':              //Throttle Up Specific Amount 'THROTTLE_INC'
        if (payloadx[0] != MAX)
        {
          for ( int x = THROTTLE_INC ; x > 0; --x )
          {
            inc255(0);
          }
        }
        break;

      case '6':              //Throttle Down Specific Amount 'THROTTLE_INC'
        if (payloadx[0] != MIN)
        {
          for ( int x = THROTTLE_INC ; x > 0; --x )
          {
            dec255(0);
          }
        }
        break;
      ///////////////////////////////////////

      //////////////////////////////////////////
      /////////* MAKE CENTER CONTROLS */////////
      //////////////////////////////////////////
      case '5':              // Yaw and Pitch Return To Center
        Make_Center(YAW);
        Make_Center(PITCH);
        break;
        
      case '3':              // Yaw Center
        Make_Center(YAW);
        break;

      case '2':              // Pitch Center
        Make_Center(PITCH);
        break;
      //////////////////////////////////////////


      //////////////////////////////////////////
      ///////////* FOLLOW CONTROLS *///////////
      //////////////////////////////////////////
      case 'o':                             // Pitch 
        Change_Payload( PITCH, ( CENTER + FOLLOW_PITCH ), PITCH_DURR );
        break;
      case 'p':                             // Yaw
        Change_Payload( YAW, ( CENTER + FOLLOW_YAW ), PITCH_DURR );
        break;
      //////////////////////////////////////////

      
      //////////////////////////////////////////
      ////////* TIMED MOVMENT CONTROLS *////////
      //////////////////////////////////////////
      case 'X':                             // Pitch Forward HI
        Change_Payload( PITCH, ( CENTER + PITCH_OFFSET_HI ), PITCH_DURR );
        break;
      case 'W':                             // Pitch Forward Medium
        Change_Payload( PITCH, ( CENTER + PITCH_OFFSET_Med ), PITCH_DURR );
        break;
      case 'w':                             // Pitch Forward Low
        Change_Payload( PITCH, ( CENTER + PITCH_OFFSET_low ), PITCH_DURR );
        break;
      case 'x':                             // Pitch Backward HI
        Change_Payload( PITCH, ( CENTER - PITCH_OFFSET_HI ), PITCH_DURR );
        break;
      case 'S':                             // Pitch Backward Medium
        Change_Payload( PITCH, ( CENTER - PITCH_OFFSET_Med ), PITCH_DURR );
        break;
      case 's':                             // Pitch Backward Low
        Change_Payload( PITCH, ( CENTER - PITCH_OFFSET_low ), PITCH_DURR );
        break;

      case 'Z':                             // Yaw Right HI
        Change_Payload( YAW, ( CENTER + YAW_OFFSET_HI ), YAW_DURR );
        break;
      case 'D':                             // Yaw Right Medium
        Change_Payload( YAW, ( CENTER + YAW_OFFSET_Med ), YAW_DURR );
        break;
      case 'd':                             // Yaw Right Low
        Change_Payload( YAW, ( CENTER + YAW_OFFSET_low ), YAW_DURR );
        break;
      case 'z':                             // Yaw Left HI
        Change_Payload( YAW, ( CENTER - YAW_OFFSET_HI ), YAW_DURR );
        break;
      case 'A':                             // Yaw Left Medium
        Change_Payload( YAW, ( CENTER - YAW_OFFSET_Med ), YAW_DURR );
        break;
      case 'a':                             // Yaw Left Low
        Change_Payload( YAW, ( CENTER - YAW_OFFSET_low ), YAW_DURR );
        break;
      /////////////////////////////////////////


      /////////////////////////////////////
      ///* CONSTANT MOVMENT ADJUSTMENT *///
      /////////////////////////////////////
      case 'i':        // Pitch Forward  //
        inc255(PITCH);
        break;
      case 'k':        // Pitch Backward //
        dec255(PITCH);
        break;
      case 'j':        // Yaw Left       //
        inc255(YAW);
        break;
      case 'l':        // Yaw Right      // 
        dec255(YAW);
        break;
      case 'm':        // Roll Left      //
        inc255(ROLL);
        break;
      case ',':        // Roll Right     //
        dec255(ROLL);                    //
        break;                           //
      /////////////////////////////////////

      //////////////////////////////////////////////
      ///////////* TRIM ADJUSTMENT *////////////////
      //////////////////////////////////////////////
      case 't':                    // Pitch Forward  
        inc255(PITCH_TRIM);
        break;                                                                                   
      case 'g':                    // Pitch Backward 
        dec255(PITCH_TRIM);                          
        break;                                                                                 
      case 'f':                    // Yaw Left       
        inc255(YAW_TRIM);                            
        break;                                                                                   
      case 'h':                    // Yaw Right      
        dec255(YAW_TRIM);                            
        break;                                       
      case 'v':                    // Roll Left      
        inc255(ROLL_TRIM);                           
        break;                                                                              
      case 'b':                    // Roll Right     
        dec255(ROLL_TRIM);                          
        break;
      ////////////////////////////////////////////

    }
  }


////////////////////////////////
//////// Payload Update ////////
////////////////////////////////
  if (millis() >= timer)
  {
    timer += 20;
    drone0.update(payloadx);
    
    for (int i = 0; i < 8; i++)   // Runs through execution timer array and  
    {                             // decrements every 20 miliseconds if necessary.
      if ( exe_ctr[i] > 0 )       // 
      {                           // - Usefull if phone connection is lost so the 
        exe_ctr[i]--;             // - drone does not continue to pitch/yaw out 
        if ( exe_ctr[i] == 0 )    // - of control.
        {                         //
          Make_Center( i );       // As each individual timer reaches '0' 
        }                         // that payload element returns to center.
      }                           //
    }                             //

  }
  
}
