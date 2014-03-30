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

const byte numbers[20] = {
  0b11111100,
  0b01100000,
  0b11011010,
  0b11110010,
  0b01100110,
  0b10110110,
  0b10111110,
  0b11100000,
  0b11111110,
  0b11100110,
  0b11111101,
  0b01100001,
  0b11011011,
  0b11110011,
  0b01100111,
  0b10110111,
  0b10111111,
  0b11100001,
  0b11111111,
  0b11100111
};

RF24 radio(4,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE8E8F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
  Serial.begin(9600);

  pinMode(CalibrationLed, OUTPUT);
  pinMode(LaserOutPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  for(int i=0;i<3;i++){    
    pinMode(latchPin[i], OUTPUT);     
  }

  displayOut[0]= numbers[10];
  displayOut[1]= numbers[0];  
  displayOut[2]= numbers[0]; 
  
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

void show(byte number[])
{
  for(int z=0;z<3;z++){
    //chariplexing
    for(int j = 0; j <= 7; j++)
     {
       byte toWrite = number[z] & (0b10000000 >> j);
       shiftIt(toWrite, z);      
     }
     
    //multiplexing
    //shiftIt(number[z], z);
  }  
}

void shiftIt (byte data, int loc)
{
  // Set latchPin LOW while clocking these 8 bits in to the register
  digitalWrite(latchPin[loc], LOW);

  for (int k=0; k <= 7; k++)
  {
    // clockPin LOW prior to sending a bit
    digitalWrite(clockPin, LOW); 
         
    if ( data & (1 << k) )
    {
      digitalWrite(dataPin, LOW); // turn “On”
    }
    else
    {
      digitalWrite(dataPin, HIGH); // turn “Off”
    }
    // and clock the bit in
    digitalWrite(clockPin, HIGH);
  }

  //stop shifting out data
  digitalWrite(clockPin, LOW); 

  //set latchPin to high to lock and send data
  digitalWrite(latchPin[loc], HIGH);
}

void blankDisplay()
{
  displayOut[0]= 0;
  displayOut[1]= 0;
  displayOut[2]= 0; 
  show(displayOut);  
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

bool showTimeLocal=true;
void doShowTime()
{
  long sec1 = SlideTime/10000;  
  if(sec1<1)
  {   
    displayOut[0]= numbers[(SlideTime/1000)%10+10];
    displayOut[1]= numbers[(SlideTime/100)%10];  
    if(!isTimeRunning)
      displayOut[2]= numbers[(SlideTime/10)%10];
    else
      displayOut[2] = 0;
  }
  else{
    displayOut[0]=numbers[sec1];
    displayOut[1]=numbers[(SlideTime/1000)%10+10];
    displayOut[2]=numbers[(SlideTime/100)%10];   
  }
      
  show(displayOut);
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
      showTimeLocal=false;    
      blankDisplay();   
    }  
  }
    
  calculateCurrentTime();
  
  //so the romote does not have to send the trip end signal
  IsTripped[1]=false;
  
  //if timeout while sending to remote display
  if(millis()-lastRemoteSend>50 && waitForResponse){
    doSendTime=true;    
    if(millis()-lastRemoteSend>50)
      showTimeLocal=true;
  }  

  if(doSendTime){ 
    sendTimeToRemoteDisplay();
  }
  
  if(showTimeLocal){
    doShowTime();
  }
}


