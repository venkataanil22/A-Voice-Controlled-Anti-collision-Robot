
#include <Servo.h> 
//
// serial_binary.ino - Arduino file to convert serial data into binary
//
// 
// Created By:	Randon Stasney, Aditya Pawar, 
//				Venkata Anil Viswanadha ,Brandon Biodrowski
// Date:		3/21/2017
// Version:		1.0
//
// Description:
// ------------
// This module takes the serial info from the wireless wixel and creates
// a binary representation that is read by the Nexys4 GPIO pins
//////////////////////////////////////////////////////////////////////


#define rx_pin  0           	// Arduino RX pin to read input from Wixel Tx
#define tx_pin 1 				// Tx pin to send input to Wixel Rx
#define llsb_pin  4 			// least significant bit
#define lsb_pin   7				// 2nd least significant bit
#define msb_pin   8				// 3rd least significant bit
#define mmsb_pin  12			// most significant bit


char go_direction = '0' ;  		// string serial read


void setup() 
{   
  Serial.begin(9600);
  // setup pins
  pinMode(rx_pin, INPUT);    
  pinMode(llsb_pin, OUTPUT);
  pinMode(lsb_pin, OUTPUT);
  pinMode(msb_pin, OUTPUT);
  pinMode(mmsb_pin, OUTPUT);
  pinMode(tx_pin, OUTPUT);
} 

void loop() // forever loop
{ 

  while(Serial.available() <=0); 
  go_direction = Serial.read();  
  motor(go_direction);

}



char motor(char dir){
  switch (dir) { //case to decode serial string to binary

  case '0':  
    digitalWrite(llsb_pin, LOW);
    digitalWrite(lsb_pin, LOW);
    digitalWrite(msb_pin, LOW);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '1':  
    digitalWrite(llsb_pin, HIGH);
    digitalWrite(lsb_pin, LOW);
    digitalWrite(msb_pin, LOW);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '2':  
    digitalWrite(llsb_pin, LOW);
    digitalWrite(lsb_pin, HIGH);
    digitalWrite(msb_pin, LOW);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '3':  
    digitalWrite(llsb_pin, HIGH);
    digitalWrite(lsb_pin, HIGH);
    digitalWrite(msb_pin, LOW);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '4':  
    digitalWrite(llsb_pin, LOW);
    digitalWrite(lsb_pin, LOW);
    digitalWrite(msb_pin, HIGH);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '5':  
    digitalWrite(llsb_pin, HIGH);
    digitalWrite(lsb_pin, LOW);
    digitalWrite(msb_pin, HIGH);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '6':  
    digitalWrite(llsb_pin, LOW);
    digitalWrite(lsb_pin, HIGH);
    digitalWrite(msb_pin, HIGH);
    digitalWrite(mmsb_pin, LOW);
    break;
  case '7':  
    digitalWrite(llsb_pin, HIGH);
    digitalWrite(lsb_pin, HIGH);
    digitalWrite(msb_pin, HIGH);
    digitalWrite(mmsb_pin, LOW);
    break;
  }
}


