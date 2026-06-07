#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_AS5600.h>
#include<ESP32Servo.h>
#include<Adafruit_PWMServoDriver.h>
#include<cmath>
#include<iostream>

#define SDA2_pin 25
#define SCL2_pin 32
#define Length 100.0    //Y軸のデフォの長さ
#define theta 90.0      //thetaのデフォ

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
Adafruit_AS5600 theta_as5600,length_as5600;
Adafruit_AS5600* as5600[] = { &theta_as5600, &length_as5600 };
const int theta_as=0;
const int length_as=1;

uint16_t loopCount = 0;
double lastRawAngle = 0.0;
bool isfirstRead = true;

const int theta_pin = 13;
const int theta_pwm = 12;
const int theta_ch = 0;

const int length_pin = 27;
const int length_pwm = 26;
const int length_ch =1;

const int height_pin = 17;//z方向は360サーボ
const int hand_pin = 18;

const float alpha = 2.0;// Y軸のギア比
const float beta = 2.0;// thetaのギア比

//プルアップ抵抗をつける（4.7kΩ〜10kΩ）

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
Servo height_M, hand_servo;

struct currentState{
  double current_X;
  double current_Y;
  double current_theta;
  double current_L;
};
struct calcMoved{
  double d_theta;
  double d_length;
  bool success;
};

//今のthetaとL1を取得する関数、更新する関数
double getCurrentState(){
  currentState state;
  uint16_t current_theta = as5600[theta_as]->getAngle();
  current_theta = (float)current_theta*360.0/4096.0;

  uint16_t current_L_angle = as5600[length_as]->getRawAngle();
  if(isfirstRead){
    lastRawAngle=current_L_angle;
    isfirstRead=false;
  }
  uint16_t diff = (uint32_t)current_L_angle - (uint32_t)lastRawAngle;
  if(diff<-2048)loopCount++;
  else if (diff>2048)loopCount--;

  lastRawAngle=current_L_angle;

  uint32_t totalsteps=((int32_t)loopCount*4096) + current_L_angle;
  float totalDegree = totalsteps*360.0/4096.0;


  state.current_theta=current_theta;
  state.current_L= Length + alpha * totalDegree;
  state.current_X=state.current_L*std::cos(state.current_theta);
  state.current_Y=state.current_L*std::sin(state.current_theta);
}

//theta,Lの差分を計算して返す関数
calcMoved calculateIK(double dx,double dy){
  calcMoved result = {0.0,0.0,false};
  currentState state;

  double target_X=state.current_X + dx;
  double target_Y=state.current_Y + dy;
  double target_L=std::sqrt(state.current_X*state.current_X + state.current_Y*state.current_Y);

  if(target_L<20||target_L>150){//要変更
    return result;
  }

  double target_theta=std::atan2(target_Y,target_X);
  double delta_theta_deg=(target_theta-state.current_theta)*180/PI;
  if(delta_theta_deg>90||delta_theta_deg<-90){//要変更
    return result;
  }

  result.d_length=target_L - std::sqrt(state.current_X*state.current_X+state.current_Y*state.current_Y);
  result.d_theta=delta_theta_deg;
  result.success=true;
}

void setup(){
  Serial.begin(115200);
  I2C_1.begin(21,22,400000);
  I2C_2.begin(SDA2_pin,SCL2_pin,400000);
  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  theta_M.setup();
  length_M.setup();
  height_M.attach(height_pin);
  hand_servo.attach(hand_pin);

  if (as5600[theta_as]->begin(AS5600_DEFAULT_ADDR,&I2C_1) == false) {
    Serial.println("AS5600 (Theta) is not detected");
  } else {
    Serial.println("AS5600 (Theta) is detected");
  }

  if (as5600[length_as]->begin(AS5600_DEFAULT_ADDR,&I2C_2) == false) {
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
  theta_M.drive(0);
  length_M.drive(0);
  height_M.write(90);
  hand_servo.write(90);
  delay(100);
}

void loop(){

  if (Serial.available()){
    String inputstring = Serial.readStringUntil('\n');
    inputstring.trim();
    if(inputstring.length() >0){
      int commaIndex = inputstring.indexOf(',');
      if(commaIndex!=-1){
        String dx_str = inputstring.substring(0,commaIndex);
        String dy_str = inputstring.substring(commaIndex+1);

        double dx = dx_str.toFloat();
        double dy = dy_str.toFloat();
        
        Serial.println("----------------------------------------");
        Serial.print("[入力受信] dx = "); Serial.print(dx);
        Serial.print(" , dy = "); Serial.println(dy);

        Serial.println(">>> 計算成功 <<<");
      }
    }
  }


  delay(5);
}