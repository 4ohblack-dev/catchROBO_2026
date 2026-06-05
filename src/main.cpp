#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_AS5600.h>
#include<ESP32Servo.h>
#include<Adafruit_PWMServoDriver.h>
#include<cmath>
#include<iostream>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
Adafruit_AS5600 as5600;

int16_t loopCount = 0;
uint16_t lastRawAngle = 0;
bool isfirstRead = true;

const int theta_pin = 13;
const int theta_pwm = 12;
const int theta_ch = 0;

const int length_pin = 25;
const int length_pwm = 26;
const int length_ch =1;

int theta = 0;
float L1 = 100;
float h = 100;


class MotorDrive{
public:
  int dirpin;
  int motorpwm;
  int pwmch;

  MotorDrive(int pin1,int pin2,int ch){
    dirpin=pin1;
    motorpwm=pin2;
    pwmch=ch;
  }

  void setup(){
    pinMode(dirpin,OUTPUT);
    pinMode(motorpwm,OUTPUT);
    ledcAttachPin(motorpwm,pwmch);
    ledcSetup(pwmch,12800,8);
  }
  void drive(int val){
    val = constrain(val,-255,255);
    if(val<0){
      digitalWrite(dirpin,HIGH);
      ledcWrite(pwmch,-val);
    }
    else if(val>0){
      digitalWrite(dirpin,LOW);
      ledcWrite(pwmch,val);
    }
    else{
      digitalWrite(dirpin,LOW);
      ledcWrite(pwmch,0);
    }
  }
};


struct calcMoved{
  double d_theta;
  double d_length;
  bool success;
};

//今のthetaとL1を取得する関数、更新する関数が必要

calcMoved calcuratedy(double dy,double theta){
  calcMoved result={0.0,0.0,false};
  if(theta>90.0 || theta<0.0){
    return result;//false
  }
  theta= theta*PI/180.0;
  if (theta == 0.0) {
        result.d_theta = 0.0;
        result.d_length = 0.0;//要変更
        result.success = true;
        return result;//とりあえずtrue
  }
  result.d_theta= std::atan2((dy*std::sin(theta)),(L1 + dy*std::cos(theta)));
  double tan1 = std::tan(result.d_theta);
  double tan2 = std::tan(theta-result.d_theta);

  if(std::abs(tan1)<1e-6 || std::abs(tan2)<1e-6){
    return result;//false
  }
  result.d_length=L1*(std::sin(theta)/tan1 + std::sin(theta)/tan2 -1);
  result.success = true;
  return result;//true,戻り値はラジアンになってる
}
calcMoved calcuratedY(double dy,double theta){
  calcMoved result={0.0,0.0,false};
  if(theta>90.0 || theta<0.0){
    return result;//false
  }
  theta= theta*PI/180.0;
  if (theta == 0.0) {
        result.d_theta = 0.0;
        result.d_length = 0.0;//要変更
        result.success = true;
        return result;//とりあえずtrue
  }
  result.d_theta= std::atan2((dy*std::sin(theta)),(L1 + dy*std::cos(theta)));
  double tan1 = std::tan(result.d_theta);
  double tan2 = std::tan(theta-result.d_theta);

  if(std::abs(tan1)<1e-6 || std::abs(tan2)<1e-6){
    return result;//false
  }
  result.d_length=L1*(std::sin(theta)/tan1 + std::sin(theta)/tan2 -1);
  result.success = true;
  return result;//true,戻り値はラジアンになってる
}

void setup(){
  Wire.begin(21,22);
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  MotorDrive theta_M{theta_pin,theta_pwm,theta_ch};
  MotorDrive length_M{length_pin,length_pwm,length_ch};
  theta_M.setup();
  length_M.setup();

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