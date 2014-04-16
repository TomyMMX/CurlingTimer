#include <RunningAverage.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

const long minTime = 5000;
const long maxTime = 99990;

unsigned long TeeLineTime=0;
unsigned long SlideTime=0;
unsigned long LastChangeStartTime = 0;
unsigned long LedOnStartTime = 0;

//data for both sensors
unsigned long TripStartTime[] = {0,0};
boolean IsTripped[] = {false, false};

byte displayOut[3];

unsigned long ReadTimeTime = 0;
int CalibrationLedState = HIGH;
unsigned long startCalibration = 0;
int LaserState = LOW;
int prevLaserValue = 500;
RunningAverage ReadAverage = RunningAverage(100);

const int LaserOutPin = A0;
const int LaserInPin = A1;
const int CalibrationLed = 2;

//display pins
const int latchPin[] = {5, 8, 9};  
const int dataPin  = 6;
const int clockPin = 7; 

RF24 radio(4,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE8E8F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
  Serial.begin(9600);

  pinMode(CalibrationLed, OUTPUT);
  pinMode(LaserOutPin, OUTPUT);
  
  radio.begin();
  radio.setChannel(125);
  radio.setAutoAck(false);
  radio.setRetries(15,15);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);  
  radio.startListening();
}

void detectLocalTrip(){
  if(micros()-ReadTimeTime>=4000)
  {
    ReadTimeTime= micros();          
    int reading = analogRead(LaserInPin);
           
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
      else if(reading>=prevLaserValue+20)
      {
        change=true;
      }
      
      if(change)
      {   
        //change 
        LastChangeStartTime=millis();          
        ReadAverage.addValue(reading*1.0);
        
        //wire stays tripped for 500ms
        if(millis()-LedOnStartTime>=500){      
          if(IsTripped[0]){          
            Serial.println("UN-TRIPPED");
          }    
          IsTripped[0]=false;
          TripStartTime[0]=millis()+2; //we look only every 4ms.. lets say the trip always starts in the middle of the two windows
          digitalWrite(CalibrationLed, HIGH);     
          ReadAverage.addValue(reading*1.0);     
        } 
      }   
      else
      {            
        //No change
        if(millis()-LastChangeStartTime>=35){         
          //if no change for 35ms then the laser is tripped
          digitalWrite(CalibrationLed, LOW);
          if(!IsTripped[0]){ 
            Serial.println("TRIPPED");       
          }     
          IsTripped[0]=true;
          LedOnStartTime=millis(); 
        }   
        if(millis()-LastChangeStartTime>=2000){     
          Serial.println("CALLIBRATING"); 
          startCalibration = millis();
          ReadAverage.clear();
        }
      }
       prevLaserValue = reading; 
    }
  }
}

unsigned long laserChangeTime = 0;
void blinkLaser()
{
  //change state every 5ms
  if(millis()-laserChangeTime>=5){
    if(LaserState==LOW){      
      LaserState = HIGH;
    }
    else{      
      LaserState = LOW;
    }
    digitalWrite(LaserOutPin, LaserState);     
    laserChangeTime = millis();
  }
}

boolean isTimeRunning = true;
boolean doSendTime = false;
unsigned long lastTeeBrake=0;

void calculateCurrentTime()
{
  if((TeeLineTime==0 || SlideTime>minTime) && millis()-lastTeeBrake>1000)
  {
    if(IsTripped[1]){
      TeeLineTime=TripStartTime[1];
      lastTeeBrake=millis();
      isTimeRunning=true;
      doSendTime=true;
    }
  }
  
  if(TeeLineTime!=0)
  {
    SlideTime=millis()-TeeLineTime;
    if(IsTripped[0]||SlideTime>maxTime){
      if(IsTripped[0]){
        SlideTime=TripStartTime[0]-TeeLineTime;
      }
      if(SlideTime>maxTime){
        SlideTime=0;
      }
      TeeLineTime=0;
      isTimeRunning=false;
      doSendTime=true;
    }    
  }   
}

unsigned long lastRemoteSend = 0;
boolean waitForResponse = false;
void sendTimeToRemoteDisplay()
{     
  unsigned long times[2];
  if(isTimeRunning)
    times[0] = 1;
  else
    times[0] = 0;
     
  times[1] = SlideTime;        
  
  radio.stopListening();
  bool ok = radio.write(times, sizeof(times));   
  if(ok){
    Serial.println("SENT TIME!");
  }
  radio.startListening();
  
  doSendTime=false;
  waitForResponse=true;
  lastRemoteSend = millis();
}

void loop() {  
  blinkLaser();
  detectLocalTrip();    
  
  if(radio.available()){
    unsigned long got_time[2];
    radio.read(got_time, sizeof(got_time) );   
   
    if(got_time[0]==5){
      Serial.println("DATA: Remote TRIP!");
      IsTripped[1]=true;
      TripStartTime[1] = millis()-got_time[1];
     
      radio.stopListening();
      bool ok = radio.write(got_time, sizeof(got_time) );    
      if(ok){
        Serial.println("SENT Trip Reply!");
      }
      radio.startListening();     
    }  
   
    if(got_time[0]==1 || got_time[0]==0){
      Serial.println("DATA: Clock Reply!");  
      
      waitForResponse = false;     
    }  
  }
    
  calculateCurrentTime();
  
  //so the romote does not have to send the trip end signal
  IsTripped[1]=false;
  
  //if timeout while sending to remote display
  if(millis()-lastRemoteSend>50 && waitForResponse){
    doSendTime=true;    
  }  

  if(doSendTime){ 
    sendTimeToRemoteDisplay();
  }
}


