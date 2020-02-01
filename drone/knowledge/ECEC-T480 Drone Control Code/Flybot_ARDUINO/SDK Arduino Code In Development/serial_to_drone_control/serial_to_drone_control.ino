// serial to caged drone controller
// coding from William Keeley, Chris Banski, Matt Fairfield

//-------------------- PURPOSE --------------------
//
// this code will take up to 8 numbers (from 0 to 255) separated by spaces as a continuous string and convert
// them into 8 unsigned characters that the drone uses for flight control. It will then connect to and transmit these commands
// to the drone. This code's intention is to black box drone control so that users can control the drone using any
// language that supports serialization.
//
//-------------------- PURPOSE --------------------

// *************ATTENTION*************
//
// before you connect the arduino to your program be sure to set the com timeout appropriately.
// you will want the timeout to be faster than the program sends sets of strings so the arduino knows
// when your program is done sending data.
//
// *************ATTENTION*************

//---------------INSTRUCTIONS---------------
//
// start with drone already powered on, then start up the arduino and PC program
// the drone will attempt to connect when there is data in the arduino Tx buffer (when a command is sent)
// start by sending dummy data '0 0 0 0 0 0 0 0\r\n' to start connection
// once drone lights go solid red the drone is ready to accept commands.
//
// ---------------INSTRUCTIONS---------------

//---------------TROUBLESHHOTING---------------
//
// throughout the testing process arduino and python were not easy to get to communicate properly when
// it came to strings. Try ensuring your arduino can echo back what you send it with a small test script
// on both your PC and on the arduino.
//
// always make sure your arduino and your code have the same baud rate and com port settings
// if you change the parity on one change it on the other.
//
// you can always use an oscilloscope on the arduino Tx port to see what is being sent to the arduino.
// at the end of this program is a python test script that will allow you to send commands to the drone
//
//---------------TROUBLESHHOTING---------------


#include <SoftwareSerial.h>

// drone object from Dzl's Lair
// http://dzlsevilgeniuslair.blogspot.com/2013/11/more-toy-quadcopter-hacking.html
#include <HCD.h>
HCD drone0;

// serial communication variables
// by default arduino serial objects are set to 8 bits with no parity and 1 stop bit
// see arduino documentation on begin() function for serial communication settings
const int Tx_Com = 1;
const int Rx_Com = 0;
const int serialBaud = 19200;
SoftwareSerial com = SoftwareSerial(Rx_Com, Tx_Com);

// input string from com port
char line[20];
const int line_size = 20;
// drone packet
unsigned char packet[8];
const int packet_size = 8;
// timer for sending packets
unsigned long timer = 0;

// Drone ID's (pick 4 random numbers)
unsigned char ID0[] = {
  0x16, 0x01, 0x55, 0x01
};


void setup() {
   // com port tx rx setup
   com.begin(serialBaud);
   // packet initialization
   memset(packet,'\0',sizeof(packet));
   delay(100);// com port setup delay
}

// -------------chars_to_packet-------------
//
// char array to drone packet function.
// takes pointer to character array and pointer to unsigned character array which is the packet for the drone.
// expects character arrays that can be converted to integers separated by spaces and null terminated.
// this function returns nothing.

void chars_to_packet(char* input_str,unsigned char* payload){
  int str_ptr = 0;
  int packet_ptr = 0;
  int temp_int = 0;
  String input = "";
  while(input_str[str_ptr] != '\0'){
    while((input_str[str_ptr] != ' ') & (input_str[str_ptr] != '\0')){
      input += input_str[str_ptr];
      str_ptr++;
    }
    temp_int = input.toInt();
    input = "";
    payload[packet_ptr] = (unsigned char)temp_int;
    packet_ptr++;
    str_ptr++;
  }

}

// same as chars_to_packet but accepts strings instead of char*
void string_to_packet(String input_str,unsigned char* payload){
  int str_ptr = 0;
  int packet_ptr = 0;
  int temp_int = 0;
  String input = "";
  while(input_str[str_ptr] != '\0'){
    while((input_str[str_ptr] != ' ') & (input_str[str_ptr] != '\0')){
      input += input_str[str_ptr];
      str_ptr++;
    }
    temp_int = input.toInt();
    input = "";
    payload[packet_ptr] = (unsigned char)temp_int;
    packet_ptr++;
    str_ptr++;
  }

}

//-------------print_unsigned_char-------------
//
// for printing an array of unsigned chars.
// useful for debugging.
// prints to com object.

void print_unsigned_char(unsigned char* to_print,const int sizeOf){
  int i = 0;
  while(i < sizeOf){
    if(to_print[i] == 0x00){return;}
    com.write(to_print[i]);
    i++;
  }  
}
// -------------read_string-------------
//
// takes a pointer to a character array and that array's size
// reads string in and null terminates it when a \r is detected
// and writes that to the array deleting the return carriage character and replacing it with null.
// this function returns nothing.
// useful for non timeout debugging

void read_string(char* buff, const int buff_len){
  int i = 0;
  while(1){
    if(com.available() > 0){
      buff[i] = (char)com.read();
      if( (buff[i++] == '\r') || (i >= (buff_len - 1))){
        buff[i-1] = '\0';
        return;
      }
    }//end if
  }//end while
  
}//end read_string




void loop() {
    
   String input_str;
   
   if (com.available() > 0) {

    if (drone0.inactive()) {
        drone0.bind(ID0);
    }//end drone if
    
    input_str = com.readString();
    string_to_packet(input_str,packet);
    
   }//end com available if

   if (millis() >= timer) {
    timer += 20;
    drone0.update(packet);

   }//end update if

}//end loop

//------------PYTHON EXAMPLE CODE------------
//
// below is example code for typing in commands to the drone
// the code requires pyserial to be installed which can be done easily using pip command
//
//------------PYTHON EXAMPLE CODE------------

/*
# written for python version 3.4.3
import serial
import sys
import time
ser = serial.Serial(port='COM7',
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
# wait for serial to finish starting
time.sleep(2)
a = ""
while a != "quit\r\n\0": # enter in quit in cmd to stop python and close port
    a = input("enter what to send:\r\n")
    a = a + "\r\n\0"
    ser.write(serial.to_bytes(bytes(a,"utf-8")))
    

ser.close()
*/
