#include <RunningAverage.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

unsigned long TripStartTime = 0;
unsigned long LastChangeStartTime = 0;
unsigned long LedOnStartTime = 0;
boolean IsTripped = false;

unsigned long ReadTimeTime = 0;
int CalibrationLedState = HIGH;
int LaserState = LOW;
int prevLaserValue = 500;
RunningAverage ReadAverage = RunningAverage(50);

const int LaserOutPins = A2;
const int LaserInPins = A3;
const int CalibrationLeds = 3;

RF24 radio(4,10);
//Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE8E8F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
  Serial.begin(9600);

  pinMode(CalibrationLeds, OUTPUT);
  pinMode(LaserOutPins, OUTPUT);

  radio.begin();
  radio.setChannel(125);
  radio.setAutoAck(false);
  radio.setRetries(15,15);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);  
  radio.stopListening();
}

unsigned long startCalibration = 0;

void detectTrip(){
  if(micros()-ReadTimeTime>=4000)
  {
    ReadTimeTime= micros();          
    int reading = analogRead(LaserInPins);
    
     ReadAverage.addValue(reading*1.0); 
     float margin = ReadAverage.getAverage();       
    /* Serial.print("AVG: ");  
     Serial.print(margin);  
     Serial.print(" VAL: ");  
     Serial.println(reading);  */
           
    if(millis()-startCalibration<202)
    {
      ReadAverage.addValue(reading*1.0);        
    }
    else
    {
      float margin = ReadAverage.getAverage();  
      boolean change=false;
          
      if(prevLaserValue>= margin + 1 && reading<margin-1)
      {
        change=true;    
      }
      else if(prevLaserValue<margin-1 && reading>=margin+1)
      {
        change=true; 
      }      
      else if(reading>=prevLaserValue+40)
      {
        change=true; 
        Serial.println("Reason 3");
      }
            
      if(change)
      { 
        //change 
        LastChangeStartTime=millis();  
        
        //wire stays tripped for 500ms
        if(millis()-LedOnStartTime>=500) 
        {      
          if(IsTripped)
          {                 
            Serial.println("UN-TRIPPED");
          }    
          IsTripped=false;
          TripStartTime=millis()+2;
          digitalWrite(CalibrationLeds, HIGH);  
          ReadAverage.addValue(reading*1.0);       
        } 
      }   
      else
      {    
        //No change
        if(millis()-LastChangeStartTime>=35)
        {         
          //if no change for 30ms then the laser is trippeds
          digitalWrite(CalibrationLeds, LOW);
          if(!IsTripped)
          {          
            Serial.println("TRIPPED"); 
            Serial.print("AVG: ");  
            Serial.print(margin);  
            Serial.print(" Prev: ");  
            Serial.print(prevLaserValue);  
            Serial.print(" Cur: ");  
            Serial.println(reading);        
          }
          IsTripped=true;
          LedOnStartTime=millis(); 
        }
        
        if(millis()-LastChangeStartTime>=2000)
        { 
          startCalibration=millis();
          ReadAverage.clear();
          Serial.println("CALLIBRATING");
        }
        
      }      
      prevLaserValue=reading;
    }
  }
}

unsigned long laserChangeTime = 0;
void blinkLasers()
{
  //change state ca. every 5ms
  if(millis()-laserChangeTime>=5)
  {
    if(LaserState==LOW)
    {
      //Serial.println("LASER ON!");
      LaserState = HIGH;
    }
    else
    {
      //Serial.println("LASER OFF!");
      LaserState = LOW;
    }
    digitalWrite(LaserOutPins, LaserState);
    
    laserChangeTime=millis();
  }
}

unsigned long started_waiting_at=0;
unsigned long prevTripStart = 0;
boolean waitForResponse = false;
boolean noReply = false;
int timeoutCount = 0;
boolean newTrip = false;
void loop() {  
  blinkLasers();  
  boolean trippedBefore = IsTripped;
  detectTrip(); 
    
  if(IsTripped && !trippedBefore)
  {
      newTrip=true;
      timeoutCount = 0;
      prevTripStart = TripStartTime;
      Serial.println("TRIP DETECTED!");
  }
  
  if(newTrip)
  {        
    unsigned long times[2];
    times[0] = 5;
    times[1] = millis()-prevTripStart; 
      
    radio.stopListening();  
    bool ok = radio.write(times, sizeof(times));
    
    if(ok){
      waitForResponse=true;
      started_waiting_at=millis();
      newTrip = false;
      Serial.println("TRIP SENT!");
    }
    else{
      noReply=true;
    }    
    radio.startListening();    
  }
    
   bool timeout = false;
   bool doRead = false;
   if(waitForResponse)
   {
     if(radio.available())
     {
       doRead=true;
     }
     else if (millis() - started_waiting_at > 50 )
     {
       timeout = true;
     }
    }      
    
    if ( timeout || doRead )
    {
      waitForResponse=false;  
      
      if(doRead)
      {        
        unsigned long got_time[2];
        radio.read( got_time, sizeof(got_time) );   
        
        if(got_time[0]!=5)
        {
           waitForResponse=true;  
        }     
        else
        {
          Serial.println("TRIP REPLY!");
          Serial.print("Data: ");               
          Serial.println(got_time[1]);
          Serial.print("round-trip delay: ");          
          unsigned long roundtripDelay = millis()-prevTripStart-got_time[1];          
          Serial.println(roundtripDelay);
          
          waitForResponse=false; 
          noReply = false;
          radio.stopListening();
        }
      }
      if(timeout)
      {
        timeoutCount++;        
        waitForResponse=false;         
        noReply = false;
        if(timeoutCount<100m )
        {
          Serial.println("TRIP TIMEOUT!");
          newTrip = true;          
        }        
        else{
          Serial.println("TRIP TIMEOUT! -- GAVE UP");
        }
      }          
    }
}


