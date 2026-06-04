#include <Arduino.h>
#include<Wire.h>
#include<Adafruit_AS5600.h>

Adafruit_AS5600 as5600;

void setup(){
  Wire.begin(21,22);
  Serial.begin(115200);
  if (as5600.begin()==false){
    Serial.print("AS5600 is not detected");
  }
  Serial.print("AS5600 is detected");
  delay(100);
}
