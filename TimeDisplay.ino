#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(4,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE8E8F0F0E1LL, 0xF0F0F0F0D2LL };

boolean running = false;
unsigned long SlideTime = 0;
unsigned long StartTime = 0;
unsigned long LastChange = 0;
int no = 0;

//Pin Assignments (You should change these)
const int CLK       = 9;           //Connected to TPIC pin 13: SRCLK (aka Clock)
const int LATCH     = 7;          //Connected to TPIC pin 12: RCLK (aka Latch/load/CS/SS...)
const int OE        = 6;          //Connected to TPIC pin 9: OE (Output Enable)
const int DOUT      = 8;          //Connected to TPIC pin 3: SER (aka MOSI)

//Number Patterns (0-9)
//***Drains 0-7 must be connected to segments A-DP respectively***
const byte numTable[] =
{
  B01111110,
  B00011000,
  B10110110,
  B10111100,
  B11011000,
  B11101100,
  B11101110,
  B00111000,
  B11111110,
  B11111100,
  B01111111,
  B00011001,
  B10110111,
  B10111101,
  B11011001,
  B11101101,
  B11101111,
  B00111001,
  B11111111,
  B11111101
};

byte number[3];

void setup()
{
  Serial.begin(9600);

  //Set pin modes
  pinMode(CLK,OUTPUT);
  pinMode(LATCH,OUTPUT);
  pinMode(DOUT, OUTPUT);
  pinMode(OE, OUTPUT);

  //7-Segment Display Init
  digitalWrite(OE,LOW);        //Enables SR Operation
  
  StartTime=millis(); 
  
  radio.begin();
  radio.setChannel(125);
  radio.setAutoAck(false);  
  radio.setRetries(15,15);
  //radio.setPayloadSize(16);  
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);  
  radio.startListening(); 
 
  clearDisplay(); 
}

void initializeSRData()
{
  //Display Scanner (Iterates through each display module)
  digitalWrite(LATCH,LOW);      //Tells all SRs that uController is sending data

    //Digit Scanner (Iterates through each SR (digit) in a display module)
    for(int digit = 0; digit < 3; digit++)
    {
      //Clears any garbage on the serial line
      shiftOut(DOUT,CLK,LSBFIRST,0);          //Shift out 0s to all displays
      //number[digit] = 0;              //Stores a 0 for each digit so its completely off
    }
  
  digitalWrite(LATCH,HIGH);      //Tells all SRs that uController is done sending data
}

void clearDisplay()
{
  initializeSRData();
  refreshDisplay();
}

void refreshDisplay()
{
  //Digit Scanner  
  //int digit = no%3;
  for(int digit = 0; digit < 3; digit++)
  {  
    //Display Scanner
    digitalWrite(LATCH,LOW);
   
    //Pre-Digit blanker (shifts out 0s to correct digits before sending segment data to desired digit)
    for(int blanks = (3 - 1 - digit); blanks > 0; blanks--)
      shiftOut(DOUT,CLK,LSBFIRST,0);

    shiftOut(DOUT,CLK,LSBFIRST,number[digit]);

    //Post-Digit blanker (shifts out 0s to remaining digits)
    for(int blanks = digit; blanks > 0; blanks--)
      shiftOut(DOUT,CLK,LSBFIRST,0);
   
    digitalWrite(LATCH,HIGH);
  }
}

unsigned long timeOfStop = 0;
unsigned long lastDisplayShow = 0;
void loop()
{ 
  if(millis()-lastDisplayShow>4)
  {
    refreshDisplay();
    lastDisplayShow=millis();
  }
  else
  {
    initializeSRData();
  }    
  if(radio.available())
  {
    Serial.println("GOT DATA!");
    unsigned long got_time1[2];
        
    bool done = radio.read(got_time1, sizeof(got_time1) );   
    
    if(got_time1[0]==1||got_time1[0]==0)
    {
      unsigned long send_time[2];
      send_time[0]=got_time1[0];
      send_time[1]=got_time1[1];
    
      radio.stopListening();  
      bool ok = radio.write(send_time, sizeof(send_time) );    
      if(ok)
      {
        Serial.println("SENT REPLY!");
      }    
      radio.startListening();
  
      if(got_time1[0]==1)
      {
        Serial.println("START!");
        running=true;
        timeOfStop = millis();
        StartTime=millis()-got_time1[1];
      }
      else if(got_time1[0]==0)
      {
        Serial.println("STOP!");
        running = false;
        timeOfStop = millis();
        SlideTime=got_time1[1];
      }   
    }
  }
    
  if(running)
  {
    SlideTime=millis()-StartTime;
  }
   
  //clearDisplay();
  
  if(millis()-LastChange>10)
  {
     LastChange=millis();
     if((!running || SlideTime>5000 || SlideTime<1000) && millis()-timeOfStop<30000)   
     {     
     long sec1 = SlideTime/10000;  
     if(sec1<1)
     {   
       number[0] = numTable[(SlideTime/1000)%10+10];
       number[1] = numTable[(SlideTime/100)%10];
       if(running){
         number[2]= 0;
       }
       else{
         number[2] = numTable[(SlideTime/10)%10];
       }
     }
     else{
       number[0] = numTable[sec1%10];
       number[1] = numTable[(SlideTime/1000)%10+10];
       number[2] = numTable[(SlideTime/100)%10];
     }
    }
    else
    {
      number[0] = 0;
      number[1] = 0;
      number[2] = 0;
    }
  }
}
