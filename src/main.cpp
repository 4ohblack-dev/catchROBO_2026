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
#define THETA 90.0      //thetaのデフォ
#define  pinion_circle 9.6*PI //ピニオンの円周 
#define theta_parcent 10.0//thetaのサイズ比

#define I2C_1 Wire
TwoWire I2C_2 = TwoWire(1);


Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
Adafruit_AS5600 theta_as5600,length_as5600;
Adafruit_AS5600* as5600[] = { &theta_as5600, &length_as5600 };
const int theta_as=0;
const int length_as=1;

uint16_t loopCount = 0;
double lastRawAngle = 0.0;
bool isfirstRead = true;

const int theta_pin = 18;
const int theta_pwm = 19;
const int theta_ch = 0;

const int length_pin = 26;
const int length_pwm = 27;
const int length_ch =1;

const int height_pin = 13;//z方向は360サーボ
const int hand_pin = 14;


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
  double current_theta;//radian
  double current_L;
};
struct calcMoved{
  double d_theta;//radian
  double d_length;
  bool success;
};
struct __attribute__((packed)) DeltaData{
  float deltaX,deltaY,angle;
};

const uint8_t HEADER = 0xAA;
const size_t DATA_SIZE = sizeof(DeltaData);
const size_t PACKET_SIZE = 1 + DATA_SIZE +1;

uint8_t calculateCRC(const uint8_t *data,size_t len){
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07; // 多項式 0x07
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void sendPacket(const DeltaData& data) {
  uint8_t buffer[PACKET_SIZE];
  
  buffer[0] = HEADER; 

  memcpy(&buffer[1], &data, DATA_SIZE); 
  
  buffer[9] = calculateCRC(&buffer[1], DATA_SIZE); 
  
  Serial.write(buffer, PACKET_SIZE);
  Serial.flush();
}


//今のthetaとL1を取得する関数、更新する関数
currentState getCurrentState(){
  currentState state;
  uint16_t current_theta = as5600[theta_as]->getAngle();
  current_theta = (float)current_theta*360.0/4096.0;//degreeに変換

  uint16_t current_L_angle = as5600[length_as]->getRawAngle();
  if(isfirstRead){
    lastRawAngle=current_L_angle;
    isfirstRead=false;
  }
  uint16_t diff = (int32_t)current_L_angle - (int32_t)lastRawAngle;
  if(diff<-2048)loopCount++;
  else if (diff>2048)loopCount--;

  lastRawAngle=current_L_angle;

  uint32_t totalsteps=((int32_t)loopCount*4096) + current_L_angle;
  float totalDegree = totalsteps*360.0/4096.0;//degreeに変換
  double current_theta_rad = current_theta * M_PI / 180.0; //radianに変換

  state.current_theta=current_theta_rad;
  state.current_L= Length + totalDegree;//totaldegreeはradian。係数が必要
  state.current_X=state.current_L*std::cos(state.current_theta);
  state.current_Y=state.current_L*std::sin(state.current_theta);

  return state;
}

//theta,Lの差分を計算して返す関数
calcMoved calculateIK(double dx,double dy,currentState state){
  calcMoved result = {0.0,0.0,false};

  double target_X=state.current_X + dx;
  double target_Y=state.current_Y + dy;
  double target_L=std::sqrt(target_X*target_X + target_Y*target_Y);

  if(target_L<20||target_L>150){//要変更
    return result;
  }

  double target_theta=std::atan2(target_Y,target_X);
  double delta_theta=(target_theta-state.current_theta);
  if(delta_theta>PI/2||delta_theta<-PI/2){//要変更
    return result;
  }

  result.d_length=target_L - state.current_L;
  result.d_theta=delta_theta;//radian
  result.success=true;

  return result;
}

/*
受信側
[ 開始: loop() が回る ]
         │
         ▼
 1. バッファ量チェック (31バイト以上あるか)
         │
         ▼
 2. ヘッダーの頭出し (0xAA, 0xBB を探す)
         │
         ▼
 3. データ本体とCRCの分離読み込み
         │
         ▼
 4. 受信データからCRCを再計算して検証
         │
         ├──────── (エラー: 不一致) ──┐
  (判定: 一致)                        │
         ▼                            ▼
 5. 構造体への展開 (復元)         [ パケット破棄 ]
         │                            │
         ▼                            │
[ 終了: ロボットのモータ制御等へ ] ◄────┘


*/

void setup(){
  Serial.begin(1152000);
  Serial.setTimeout(0);
  I2C_1.begin(21, 22, 400000);
  I2C_2.begin(SDA2_pin, SCL2_pin, 400000);  
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
  Serial.print("DATA_SIZE_CHECK: "); Serial.println(sizeof(DeltaData));
  Serial.print("PACKET_SIZE_CHECK: "); Serial.println(1 + sizeof(DeltaData) + 1);
  theta_M.drive(0);
  length_M.drive(0);
  height_M.write(90);
  hand_servo.write(90);
  delay(100);
}

void loop() {
  while (Serial.available() >= PACKET_SIZE) {
    
    if (Serial.peek() != HEADER) {
      Serial.read();
      continue;
    }

    uint8_t rawPacket[PACKET_SIZE];
    size_t readLen = Serial.readBytes(rawPacket, PACKET_SIZE);

    if (readLen == PACKET_SIZE) {
      uint8_t *dataBuffer = &rawPacket[1];              // データの先頭ポインタ
      uint8_t receivedCRC = rawPacket[PACKET_SIZE - 1]; // 末尾のCRC

      if (calculateCRC(dataBuffer, DATA_SIZE) == receivedCRC) {
        DeltaData receivedData;
        memcpy(&receivedData, dataBuffer, DATA_SIZE);

        sendPacket(receivedData);
      }
    }
  }
}