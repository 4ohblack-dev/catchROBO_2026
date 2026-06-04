#include <Arduino.h>
#include<Wire.h>
#include<Adafruit_AS5600.h>
#include<ESP32Servo.h>
#include<Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
Adafruit_AS5600 as5600;
int16_t loopCount = 0;
uint16_t lastRawAngle = 0;
bool isfirstRead = true;

void setup(){
  Wire.begin(21,22);
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setPWMFreq(50);

  if (as5600.begin()==false){
    Serial.println("AS5600 is not detected");
  }

  Serial.println("AS5600 is detected");
  if(as5600.isMagnetDetected()){
    Serial.println("good magnet_Position");
  }
  if(as5600.isAGCminGainOverflow()){
    Serial.println("magnet is too strong");
  }
  if(as5600.isAGCmaxGainOverflow()){
    Serial.println("magnet is too weak");
  }

  Serial.println("as5600 PERFECT");
  delay(100);
}


void loop(){
  uint16_t currentRawAngle = as5600.getRawAngle();//連続して回るようなところ
  if(isfirstRead){
    lastRawAngle=currentRawAngle;
    isfirstRead=false;
  }
  int16_t diff = currentRawAngle - lastRawAngle;
  if(diff<-2048)loopCount++;
  else if (diff>2048)loopCount--;

  lastRawAngle=currentRawAngle;

  int32_t totalsteps=((int32_t)loopCount*4096)+currentRawAngle;
  float totalDegree = totalsteps*360/4096;
  float degrees = currentRawAngle*360/4096;
  Serial.println(degrees);
  delay(5);
}