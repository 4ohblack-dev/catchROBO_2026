#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_AS5600.h>
#include<ESP32Servo.h>
#include<Adafruit_PWMServoDriver.h>
#include<cmath>
#include<iostream>

#define SDA2_pin 25
#define SCL2_pin 32

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
Adafruit_AS5600 theta_as5600,length_as5600;
Adafruit_AS5600* as5600[] = { &theta_as5600, &length_as5600 };
const int the_enc=0;
const int len_enc=1;

const int theta_as=0;
const int length_as=1;

int16_t loopCount[2] = {0, 0};
uint16_t lastRawAngle[2] = {0, 0};
bool isfirstRead[2] = {true, true};

const int theta_pin = 13;
const int theta_pwm = 12;
const int theta_ch = 0;

const int length_pin = 27;
const int length_pwm = 26;
const int length_ch =1;

const int height_pin = 17;
const int height_pwm = 18;
const int height_ch = 2;

//プルアップ抵抗をつける（4.7kΩ〜10kΩ）

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

MotorDrive theta_M{theta_pin,theta_pwm,theta_ch};
MotorDrive length_M{length_pin,length_pwm,length_ch};
MotorDrive height_M{height_pin,height_pwm,height_ch};

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

float getCulculatedDeg(int id){
  uint16_t currentRawAngle = as5600[id]->getRawAngle();//連続して回るようなところ
  if(isfirstRead[id]){
    lastRawAngle[id]=currentRawAngle;
    isfirstRead[id]=false;
  }
  int16_t diff = currentRawAngle - lastRawAngle[id];
  if(diff<-2048)loopCount[id]++;
  else if (diff>2048)loopCount[id]--;

  lastRawAngle[id]=currentRawAngle;

  int32_t totalsteps=((int32_t)loopCount[id]*4096)+currentRawAngle;
  float totalDegree = totalsteps*360.0/4096.0;
  return totalDegree;//度数表記で返す
}

void setup(){
  Serial.begin(115200);
  I2C_1.begin(21,22,400000);
  I2C_2.begin(SDA2_pin,SCL2_pin,400000);
  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  theta_M.setup();
  length_M.setup();
  height_M.setup();

  if (as5600[the_enc]->begin(AS5600_DEFAULT_ADDR,&I2C_1) == false) {
    Serial.println("AS5600 (Theta) is not detected");
  } else {
    Serial.println("AS5600 (Theta) is detected");
  }

  if (as5600[len_enc]->begin(AS5600_DEFAULT_ADDR,&I2C_2) == false) {
    Serial.println("AS5600 (Length) is not detected");
  } else {
    Serial.println("AS5600 (Length) is detected");
  }

  for(int id=0;id<2;id++){
    if(as5600[id]->isMagnetDetected()){
      Serial.print(id);
      Serial.println("good magnet_Position");
    }
    else if(as5600[id]->isAGCminGainOverflow()){
      Serial.print(id);
      Serial.println("magnet is too strong");
    }
    else if(as5600[id]->isAGCmaxGainOverflow()){
      Serial.print(id);
      Serial.println("magnet is too weak");
    }
  }

  Serial.println("as5600 PERFECT");
  delay(100);
}

void loop(){
  float theta_degree=getCulculatedDeg(the_enc);
  float length_degree=getCulculatedDeg(len_enc);

  // シリアル出力
  Serial.print("Theta: ");   Serial.print(theta_degree);
  Serial.print("\tLength: "); Serial.println(length_degree);

  delay(5);
}