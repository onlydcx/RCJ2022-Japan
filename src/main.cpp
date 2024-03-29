// ライブラリの読み込み
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <CytronMotorDriver.h>
#include <EEPROM.h>
#include <Servo.h>

// ドリブラーの制御周波数など
#define MAX_SIGNAL 1600
#define MIN_SIGNAL 1000
#define ESC_PIN 13

// ドリブラー用の変数・コンストラクタ
int volume = 0;
char message[50];
Servo esc;

// ディスプレイ関連の定義
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SIZE_OF_ARRAY(array) (sizeof(array) / sizeof(array[0]))
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C
#define w display.width()
#define h display.height()

// ボタンのピン関連
#define MOTOR_SWITCH 32
#define LeftPin 33
#define RightPin 27
#define LeftPush isPush(31)
#define CenterPush isPush(25)
#define OnOffPush isPush(30)
#define RightPush isPush(27)
#define AnyPush (LeftPush || CenterPush || OnOffPush || RightPush)


// ラインセンサーの真偽
#define isOnFront (isOnLine(1,0) || isOnLine(1,1)) // 前 (中,外)
#define isOnRight (isOnLine(0,0) || isOnLine(0,1)) // 右 (外,中)
#define isOnBack (isOnLine(2,0) || isOnLine(2,1)) // 後 (外,中)
#define isOnLeft (isOnLine(3,0) || isOnLine(3,1)) // 左 (外,中)

// 外側のラインセンサが反応してるか
#define isOutFront isOnLine(1,1)
#define isOutRight isOnLine(0,0)
#define isOutBack isOnLine(2,0)
#define isOutLeft isOnLine(3,0)

// 内側のラインセンサが反応してるか
#define isInFront isOnLine(1,0)
#define isInRight isOnLine(0,1)
#define isInBack isOnLine(2,1)
#define isInLeft isOnLine(3,1)

// いずれかの外側のラインセンサが反応しているか
#define isOnOut (isOnLine(1,1) || isOnLine(0,0) || isOnLine(2,0) || isOnLine(3,0))
// いずれかの内側の...
#define isOnIn (isOnLine(1,0) || isOnLine(0,1) || isOnLine(2,1) || isOnLine(3,1))

// ラインセンサがどれか1つでも反応しているか
#define isOnAny (isOnOut || isOnIn)

// モータードライバの定義
CytronMD motor1(PWM_DIR, 5, 4);
CytronMD motor2(PWM_DIR, 3, 2);
CytronMD motor3(PWM_DIR, 9, 8);
CytronMD motor4(PWM_DIR, 12, 6);

// ディスプレイの初期化
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ジャイロ関連
MPU6050 mpu;
static uint8_t mpuIntStatus;
static bool dmpReady = false;
static uint16_t packetSize;
int16_t Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int Gyro_X, Gyro_Y, Gyro_Z, Accel_Z;

// ロボットのスピード 0~255まで
int speed = 200;

// その他 グローバル変数
bool isFirstSetSpeed = true;
int ave_motor_power[4][10] = {0};
int ave_mpPlus = 0;
int prevIR, dirPlus, cnt;
int dirIR = 0;

// ドリブラーを回す関数
void dribler(int pulse) {
   esc.writeMicroseconds(pulse);
}

// IRセンサと通信して値を取得
int getVah(int f) {
   byte val = 0;
   Wire.beginTransmission(0x0E);
   Wire.write(f);
   Wire.endTransmission();
   // Wire.endTransmission(false); // falseの方が良いらしい...?
   Wire.requestFrom(0x0E, 1);
   while (Wire.available()) {
      val = Wire.read();
   }
   return (int)val;
}

// カメラと通信する
int getCam() {
   byte re = 0;
   if (Serial7.available()) {
      byte a = Serial7.read();
      if (a >= 0 && a <= 70) {
         re = a;
      }
      else {
         re = 0;
      }
   }
   return (int)re;
}

// 文字列の中心x座標を返す(ディスプレイ用)
int getDx(String txt) {
   int char_len = txt.length();
   return (display.width() / 2) - ((char_len / 2) * 11);
}

// ボールの距離or角度を返す
int IRval(int i) {
   int a = getVah(0x04);
   int b = getVah(0x05);
   int c = getVah(0x06);
   int d = getVah(0x07);
   int re_angle;
   int re_strength;

   if (d < 10) {
      re_angle = a;
      re_strength = b;
   }
   else {
      re_angle = c;
      re_strength = d;
   }

   if(i != 1) {
      return re_strength;
   }
   else {
      return re_angle*5;
   }
}

// IRセンサの精度向上らしい
int getdirIR() {
   dirIR = IRval(1);
   if (abs(prevIR - dirIR) > 30) {
      cnt++; 
      if (cnt == 5) {
         cnt = 0;
         prevIR = dirIR;
      }
      else {
         dirIR = prevIR;
      }
   }
   else {
      cnt = 0;
      prevIR = dirIR;
   }
   return dirIR;
}

// ジャイロセンサの値を返す
int GyroGet(void) {
   mpuIntStatus = false;
   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
   }
   else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) {
         fifoCount = mpu.getFIFOCount();
      }
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Gyro_Now = degrees(ypr[0]) + 180;
      Gyro = Gyro_Now + Gyro_Offset - 180;
      if (Gyro < 0) {
         Gyro += 360;
      }
      if (Gyro > 359) {
         Gyro -= 360;
      }
   }
   return Gyro;
}

// ジャイロセンサの初期化
void Gryo_init() {
   mpu.initialize();
   if (mpu.testConnection() != true) {
      Serial.println("MPU disconection");
   }
   if (mpu.dmpInitialize() != 0) {
      Serial.println("MPU break");
   }
   mpu.setXGyroOffset(Gyro_X);
   mpu.setYGyroOffset(Gyro_Y);
   mpu.setZGyroOffset(Gyro_Z);
   mpu.setZAccelOffset(Accel_Z);
   mpu.setDMPEnabled(true);
   mpuIntStatus = mpu.getIntStatus();
   dmpReady = true;
   packetSize = mpu.dmpGetFIFOPacketSize();
}

// ジャイロセンサの個体差調整
void selectGyro(int number) {
   switch (number) {
      case 0:
         Gyro_X = -93, Gyro_Y = 77, Gyro_Z = 10, Accel_Z = 1561;
         break;
      case 1:
         Gyro_X = 114, Gyro_Y = 61, Gyro_Z = 95, Accel_Z = 1719;
         break;
      case 2:
         Gyro_X = -118, Gyro_Y = 22, Gyro_Z = -61, Accel_Z = 797;
         break;
      case 3:
         Gyro_X = -364, Gyro_Y = -29, Gyro_Z = 9, Accel_Z = 1846;
         break;
      case 4:
         Gyro_X = 50, Gyro_Y = 22, Gyro_Z = 12, Accel_Z = 1347;
         break;
      default:
         Serial.println("error in selectGyro()");
         Gyro_X = 0, Gyro_Y = 0, Gyro_Z = 0, Accel_Z = 0;
         break;
   }
}

// num番のボタンが押されたかの真偽
bool isPush(int num) {
   if (digitalRead(num)) {
      while (digitalRead(num)) {
         if (!digitalRead(num)) {
            break;
         }
      }
      return 1;
   }
   else {
      return 0;
   }
}

// ディスプレイの初期化
void display_init() {
   if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
   }
}

// ディスプレイ用 車検時のキックデモ
void printKick() {
   String txt = "Kick Test";
   int char_len = txt.length();
   int drawX = (display.width() / 2) - ((char_len / 2) * 13);
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(drawX, 20);
   display.println(txt);
   display.display();
}

// ディスプレイ用 ボールの角度表示
void printIR() {
   int cx = w / 4, cy = h / 2, r = 18;
   float deg = IRval(1), Rad;
   display.clearDisplay();
   Rad = (deg - 90) / (180 / PI);
   display.drawCircle(cx, cy, w / 12, SSD1306_WHITE);
   display.drawCircle(cx, cy, w / 5, SSD1306_WHITE);
   display.fillCircle(cx + r * cos(Rad), cy + r * sin(Rad), 3, 1);
   display.drawLine(cx, 0, cx, h, SSD1306_WHITE);
   display.drawLine(0, cy, cx * 2, cy, SSD1306_WHITE);
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(w / 2 + 10, 10);
   display.println("Ball");
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(w / 2 + 10, h / 2 + 5);
   display.println((int)deg);
   display.display();
}

// ラインセンサーのピン
int analogPins[4][2] = {
   {0, 1}, {2, 3}, {6, 7}, {8, 9}
};

// ラインセンサーの閾値
int thresholds[4][2] = {
   {800, 800},
   {800, 800},
   {800, 800},
   {800, 800}
};

// ラインセンサの時期位置自動調整用の一時的な変数
int LineMin[4][2] = {1023};
int LineMax[4][2] = {0};

// ラインセンサの時期位置自動調整
void LineThUpdate() {
   String txt = "Update";
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(getDx(txt), 30);
   display.println(txt);
   display.display();
   if(OnOffPush) {
      txt = "Updating";
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(getDx(txt), 30);
      display.println(txt);
      display.display();
      while(!OnOffPush) {
         for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 2; j++) {
               int val = analogRead(analogPins[i][j]);
               if(val < LineMin[i][j]) {
                  LineMin[i][j] = val;
               }
               if(val > LineMax[i][j]) {
                  LineMax[i][j] = val;
               }
            }
         }
      }
      for(int i = 0; i < 4; i++) {
         for(int j = 0; j < 2; j++) {
            int border = 3;
            int add = 50;
            int diff = abs(LineMax[i][j] - LineMin[i][j]);
            int threshold = LineMax[i][j] - (diff / border);
            thresholds[i][j] = threshold + add;
         }
      }
      txt = "done";
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(getDx(txt), 30);
      display.println(txt);
      display.display();
      delay(500);
   }
}

// ラインセンサーの読み取った値を返す
int getLine(int i, int j) {
   return analogRead(analogPins[i][j]);
}

// 白線上かを判定
bool isOnLine(int i, int j) {
   return (analogRead(analogPins[i][j]) > thresholds[i][j])? true: false;
}

// ボールをキャッチしているか
bool isCatch() {
  int th = 50;
  int val = analogRead(A12);
  if(val < th) {
      return true;
  }
  else {
   return false;
  }
}

// ディスプレイ用 ラインセンサーの表示
void printLine() {
   int cx = w/4, cy = h/2;
   display.clearDisplay();
   int paddingC = 10, paddingI = 3;
   int r = w / 5;
   display.drawCircle(cx, cy, r, SSD1306_WHITE);
   display.drawLine(cx, paddingC, cx, cy - paddingI, SSD1306_WHITE);
   display.drawLine(cx, cy + paddingI, cx, h - paddingC, SSD1306_WHITE);
   display.drawLine(paddingC, cy, cx - paddingI, cy, SSD1306_WHITE);
   display.drawLine(cx + paddingI, cy, cx * 2 - paddingC, cy, SSD1306_WHITE);
   String isOnTXT = "";
   if(isOnFront) {
      display.fillRect(cx-2, 10, 5, r-5, SSD1306_WHITE);
      isOnTXT += "F";
   }
   if(isOnRight) {
      display.fillRect(cx+3, cy-2, r-5, 5, SSD1306_WHITE);
      isOnTXT += "R";
   }
   if(isOnBack) {
      display.fillRect(cx-2, cy+3, 5, r-5, SSD1306_WHITE);
      isOnTXT += "B";
   }
   if(isOnLeft) {
      display.fillRect(10, cy-2, r-5, 5, SSD1306_WHITE);
      isOnTXT += "L";
   }
   display.setTextColor(SSD1306_WHITE);
   display.setTextSize(2);
   display.setCursor(w/2 + 10, 10);
   display.println("Line");
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(w / 2 + 10, h / 2 + 5);
   display.println(isOnTXT);
   display.display();
}

// ディスプレイ用のグローバル変数
int speed_status = 0;
bool speed_isPushed = false;
String speed_mode[] = {"0", "10", "50", "100", "150", "170", "200", "220", "250"};
int speed_mode_len = SIZE_OF_ARRAY(speed_mode);

// ディスプレイ用 スピードを変える
void changeSpeed() {
   if (LeftPush) {
      speed_isPushed = true;
      if ((speed_status <= speed_mode_len - 1) && (speed_status >= 1)) {
         speed_status--;
      }
   }
   if (RightPush) {
      speed_isPushed = true;
      if ((speed_status <= speed_mode_len - 2) && (speed_status >= 0)) {
         speed_status++;
      }
   }
   if (speed_isPushed) {
      display.clearDisplay();
      if (speed_status <= speed_mode_len - 2) {
         display.drawTriangle(90, 45, 90, 55, 100, 50, SSD1306_WHITE);
      } 
      if (speed_status >= 1) {
         display.drawTriangle(34, 45, 34, 55, 24, 50, SSD1306_WHITE);
      }
      int char_len = speed_mode[speed_status].length();
      int drawX = (display.width() / 2) - ((char_len / 2) * 11);
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(drawX, 7);
      display.println(speed_mode[speed_status]);
      display.drawLine(0, 32, 128, 32, SSD1306_WHITE);
      display.display();
      speed = atoi((speed_mode[speed_status]).c_str());
   }
}

// ディスプレイ用 ジャイロの値を表示
void printIMU() {
   int cx = w / 4, cy = h / 2, r = 21;
   float deg = GyroGet(), Rad;
   display.clearDisplay();
   Rad = (deg - 90) / (180 / PI);
   display.drawCircle(cx, cy, w / 6, SSD1306_WHITE);
   display.fillCircle(cx + r * cos(Rad), cy + r * sin(Rad), 3, 1);
   display.drawLine(cx, 0, cx, h, SSD1306_WHITE);
   display.drawLine(0, cy, cx * 2, cy, SSD1306_WHITE);
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(w / 2 + 10, 10);
   display.println("Gyro");
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(w / 2 + 10, h / 2 + 5);
   display.println((int)deg);
   display.display();
}

// ディスプレイ用 ジャイロの0度のリセット
void RST_Gy() {
   String txt = "RST Gyro";
   int char_len = txt.length();
   int drawX = (display.width() / 2) - ((char_len / 2) * 11);
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(drawX, 30);
   display.println(txt);
   display.display();
   if(OnOffPush) {
      String txt = "Reseting";
      int char_len = txt.length();
      int drawX = (display.width() / 2) - ((char_len / 2) * 11);
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(drawX, 30);
      display.println(txt);
      display.display();
      Gryo_init();
      delay(100);
   }
}

// モーター停止
void motorStop() {
   motor1.setSpeed(0);
   motor2.setSpeed(0);
   motor3.setSpeed(0);
   motor4.setSpeed(0);
}

// ロボットが正面を向いていなければ前を向く
void turnFront() {
   int diff = 45;
   int S = 50;
   int MAX = 130;
   int GY = GyroGet();
   while(GY >= diff || GY < (360 - diff)) {
   if(GY >= diff && GY < 90) {
      motor1.setSpeed(S);
      motor2.setSpeed(S);
      motor3.setSpeed(-S);
      motor4.setSpeed(S);
   }
   else if(GY >= 90 && GY < 180) {
      motor1.setSpeed(MAX);
      motor2.setSpeed(MAX);
      motor3.setSpeed(-MAX);
      motor4.setSpeed(MAX);
   }
   else if(GY >= 180 && GY < 270) {
      motor1.setSpeed(-MAX);
      motor2.setSpeed(-MAX);
      motor3.setSpeed(MAX);
      motor4.setSpeed(-MAX);
   }
   else if(GY >= 270 && GY < 315) {
      motor1.setSpeed(-S);
      motor2.setSpeed(-S);
      motor3.setSpeed(S);
      motor4.setSpeed(-S);
   }
   else {
      break;
   }
      GY = GyroGet();
   }
} 

// モーターを0~360度、任意の方へ動かす 平均値化アリ 通常移動用
void motor(int angle) {
   double motor_power[4];
   double max_power;
   motor_power[0] = cos((45 - angle) / 180.0 * PI);
   motor_power[1] = cos((135 - angle) / 180.0 * PI);
   motor_power[2] = cos((-45 - angle) / 180.0 * PI);
   motor_power[3] = cos((-135 - angle) / 180.0 * PI);
   for (int i = 0; i < 4; i++) {
      if (abs(motor_power[i]) > max_power) {
         max_power = abs(motor_power[i]);
      }
   }
   for (int i = 0; i < 4; i++) {
      motor_power[i] = speed * motor_power[i] / max_power;
      for (int j = 9; j > 0; j--) {
         ave_motor_power[i][j] = ave_motor_power[i][j - 1];
      }
      ave_motor_power[i][0] = motor_power[i];
      ave_mpPlus = 0;
      for (int k = 0; k < 10; k++) {
         ave_mpPlus = ave_mpPlus + ave_motor_power[i][k];
      }
      motor_power[i] = ave_mpPlus / 10;
   }
   int gy = GyroGet();
   int addP = 0;
   if(gy > 3 && gy < 180) {
      addP = 30;
   }
   else if (gy >= 180 && gy < 358) {
      addP = -30;
   }
   if(gy > 90 && gy < 270) {
      turnFront();
   }
   motor1.setSpeed(-motor_power[1] + addP);
   motor2.setSpeed(motor_power[0] + addP);
   motor3.setSpeed(motor_power[2] + addP);
   motor4.setSpeed(motor_power[3] + addP);
}

// ラインセンサーを踏んだ時用のモーター制御 平均値化なし
void lightMotor(int angle) {
   double motor_power[4];
   double max_power;
   motor_power[0] = cos((45 - angle) / 180.0 * PI);
   motor_power[1] = cos((135 - angle) / 180.0 * PI);
   motor_power[2] = cos((-45 - angle) / 180.0 * PI);
   motor_power[3] = cos((-135 - angle) / 180.0 * PI);
   for (int i = 0; i < 4; i++) {
      if (abs(motor_power[i]) > max_power) {
         max_power = abs(motor_power[i]);
      }
   }
   for (int i = 0; i < 4; i++) {
      motor_power[i] = speed * motor_power[i] / max_power;
   }
   int gy = GyroGet();
   int addP = 0;
   if(gy > 30 && gy < 330) {
      turnFront();
   }
   if(gy > 5 && gy < 180) {
      addP = 0;
   }
   else if (gy >= 180 && gy < 355) {
      addP = -50;
   }

   motor1.setSpeed(-motor_power[1] + addP);
   motor2.setSpeed(motor_power[0] + addP);
   motor3.setSpeed(motor_power[2] + addP);
   motor4.setSpeed(motor_power[3] + addP);
}

// キッカー
void kick() {
   digitalWrite(10, HIGH);
   delay(100);
   digitalWrite(10, LOW);
   delay(100);
}

// ラインセンサの値をメモリに保存
void writeEEPROM() {
  int cnt = 0;
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 2; j++) {
      EEPROM.write(cnt,thresholds[i][j] / 5);
      Serial.print(thresholds[i][j]);
      Serial.print(" ");
      cnt++;
    }
  }
  Serial.println("");
}

// ボールを追う関数 今は使ってない
void followBall2() {

  int IR = getdirIR();
  int dltime = 150;

   if(isOnAny) {
      
      if(isOnFront) {
         turnFront();
         lightMotor(180);
         delay(dltime*2);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR <= 90 || IR >= 270)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 90 && IR <= 270) || digitalRead(MOTOR_SWITCH)) {
               break;
            } 
         }
      }
      if(isOnRight) {
         turnFront();
         lightMotor(270);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR <= 180 && IR >= 0)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 180) || digitalRead(MOTOR_SWITCH)) {
               break;
            } 
         }
      }
      if(isOnBack) {
         turnFront();
         lightMotor(0);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR <= 270 && IR >= 90)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 270 || IR <= 90) || digitalRead(MOTOR_SWITCH)) {
               break;
            } 
         }
      }
      if(isOnLeft) {
         turnFront();
         lightMotor(90);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR >= 180)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 0 && IR <= 180) || digitalRead(MOTOR_SWITCH)) {
               break;
            }
         }
      }
   }
   else {
      IR = getdirIR();

      if (IR <= 35) {
         dirPlus = IR ;
      }
      else if (IR >= 325) {
         dirPlus = (360 - IR);
      } 
      else {
         dirPlus = 50;
      }
      dirPlus = dirPlus *3/4;

      if (getVah(0x05) <= 60 && getVah(0x07) <= 15) {
         motor(IR);
      }
      else if (getVah(0x05) >= 75 && getVah(0x07) >= 15) {
         if (IR <= 5 || IR >= 355) {
            while(isCatch() && !isOnFront) {
               lightMotor(0);
               if(isOnFront) {
                        lightMotor(180);
                        delay(dltime*2);
                        int time = millis(); 
                        while(((millis() - time) < 3000) && (IR <= 90 || IR >= 270)) {
                           motorStop();
                           IR = getdirIR(); 
                           if((IR >= 90 && IR <= 270) || digitalRead(MOTOR_SWITCH)) {
                              break;
                           } 
                        }
                        break;
               }
               if(!isCatch()) break;
            }
            motor(IR);
         }
         else {
            if (dirIR <= 180) {
               motor(IR + dirPlus * 2);
            }
            else {
               motor(IR - dirPlus * 2);
            }
         }
      } 
      else {
         if (dirIR <= 180) {
            motor(IR + dirPlus);
         } 
         else {
            motor(IR - dirPlus);
         }
      }
   }
}

// ボール追う関数 今はこっち使ってる
void followBall3() {
   // ball、IRにボールの角度を代入
   int ball = IRval(1);
   int IR = ball;

   // ラインセンサを踏んだ時に戻る時間
   int dltime = 200;

   if(isOnAny) {
      // いずれかの白線上なら

      if(isOnFront) { 
         // 前の白線を踏んだら
         lightMotor(180); // 180後ろへ行く
         delay(dltime*2); // dltime*2続ける
         int time = millis(); // 時間計測
         while(((millis() - time) < 3000) && (IR <= 90 || IR >= 270)) { 
            // 3秒以内かつ ボールが前 なら 待機
            motorStop();
            IR = getdirIR(); 
            if((IR >= 90 && IR <= 270) || digitalRead(MOTOR_SWITCH)) {
               // ボールが後ろに行ったら走り出す
               break;
            } 
         }
      }
      if(isOnRight) {
         lightMotor(270);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR <= 180 && IR >= 0)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 180) || digitalRead(MOTOR_SWITCH)) {
               break;
            } 
         }
      }
      if(isOnBack) {
         lightMotor(0);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR <= 270 && IR >= 90)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 270 || IR <= 90) || digitalRead(MOTOR_SWITCH)) {
               break;
            } 
         }
      }
      if(isOnLeft) {
         lightMotor(90);
         delay(dltime);
         int time = millis(); 
         while(((millis() - time) < 3000) && (IR >= 180)) {
            motorStop();
            IR = getdirIR(); 
            if((IR >= 0 && IR <= 180) || digitalRead(MOTOR_SWITCH)) {
               break;
            }
         }
      }
   }
   else {
      // 白線上でない時
      if(ball <= 5 || ball >= 350) {
         // ボールが前にある
         motor(ball);
         while(isCatch()) {
            // 課題：右に行く
            // dribler(1600);
            motor(0);
            if(!isCatch() || digitalRead(MOTOR_SWITCH)) {
               // dribler(1000);
               break;
            }
         }
      }
      else {
         // to do θの定数
         // dribler(1000);
         if(ball <= 180) {
            motor(ball+40);
         }
         else {
            motor(ball-40);
         }
      }
   }

}

// ディスプレイ関係の変数
String mode[] = {"Main", "Ball", "Gyro", "Kick", "Speed", "RST Gyro","LineCheck","LineThUp","EEPROM"};
int mode_len = SIZE_OF_ARRAY(mode);

void setup() {
   pinMode(MOTOR_SWITCH,INPUT);
   pinMode(27, INPUT);
   pinMode(25, INPUT);
   pinMode(34, INPUT);
   pinMode(35, INPUT);
   pinMode(10, OUTPUT);
   pinMode(32, INPUT);
   Serial.begin(19200);
   Wire.begin();
   Serial7.begin(19200);
   display_init();
   selectGyro(3);
   Gryo_init();
   // int cnt = 0;
   // for(int i = 0; i < 4; i++) {
   //    for(int j = 0; j < 2; j++) {
   //       thresholds[i][j] = EEPROM.read(cnt) * 5;
   //       // Serial.print(EEPROM.read(cnt) * 5);
   //       // Serial.print(" ");
   //       cnt++;
   //    }
   // }
   // Serial.println("");
   // int len = 8;
   // int drawX = (display.width() / 2) - ((len / 2) * 12);
   // display.clearDisplay();
   // display.setTextSize(2);
   // display.setTextColor(SSD1306_WHITE);
   // display.setCursor(drawX, 10);
   // display.println("Main.cpp");

   // esc.attach(ESC_PIN);
   // esc.writeMicroseconds(MAX_SIGNAL);
   // delay(2000);
   // esc.writeMicroseconds(MIN_SIGNAL);
   // delay(2000);
   // dribler(1600);
}

int status = 0;
bool isPushed = false;

void loop() {
   while(digitalRead(MOTOR_SWITCH)) {
      followBall2();
      if(!digitalRead(MOTOR_SWITCH)) {
         // motorStop();
         // dribler(1000); // モーター止まらん ドリブラー止まる
         
         // motorStop();
         // dribler(0); //　モーター止まる ドリブラーバグる
         motorStop();
         delay(10);
         // dribler(0);
         break;
      }
   }
   // if (LeftPush) {
   //    isPushed = true;
   //    if ((status <= mode_len - 1) && (status >= 1)) {
   //       status--;
   //    }
   // }
   // if (RightPush) {
   //    isPushed = true;
   //    if ((status <= mode_len - 2) && (status >= 0)) {
   //       status++;
   //    }
   // }
   // if (CenterPush) {
   //    switch (status) {
   //       case 0:
   //          display.clearDisplay();
   //          display.display();
   //          while(!CenterPush) {
   //             followBall3();
   //          }
   //          dribler(1000);
   //          motorStop();
   //          break;
   //       case 1:
   //          while (!CenterPush) {
   //             printIR();
   //          }
   //          break;
   //       case 2:
   //          while (!CenterPush) {
   //             printIMU();
   //          }
   //          break;
   //       case 3:
   //          while (!CenterPush) {
   //             printKick();
   //             if (digitalRead(31)) {
   //                kick();
   //             }
   //          }
   //          break;
   //       case 4:
   //          if (isFirstSetSpeed) {
   //             int char_len = ((String)speed).length();
   //             int drawX = (display.width() / 2) - ((char_len / 2) * 11);
   //             display.clearDisplay();
   //             display.setTextSize(2);
   //             display.setTextColor(SSD1306_WHITE);
   //             display.setCursor(drawX, 7);
   //             display.println(speed);
   //             display.drawLine(0, 32, 128, 32, SSD1306_WHITE);
   //             if (atoi((speed_mode[0]).c_str()) < speed) {
   //                display.drawTriangle(90, 45, 90, 55, 100, 50, SSD1306_WHITE);
   //             }
   //             if (atoi((speed_mode[speed_mode_len - 1]).c_str()) > speed) {
   //                display.drawTriangle(34, 45, 34, 55, 24, 50, SSD1306_WHITE);
   //             }
   //             for (int i = 0; i < speed_mode_len; i++) {
   //                if (atoi((speed_mode[i]).c_str()) == speed) {
   //                   speed_status = i + 1;
   //                   break;
   //                }
   //                if (atoi((speed_mode[i]).c_str()) > speed) {
   //                   speed_status = i;
   //                   break;
   //                }
   //             }
   //             speed = atoi((speed_mode[speed_status]).c_str());
   //             display.display();
   //             isFirstSetSpeed = !isFirstSetSpeed;
   //          }
   //          while (!CenterPush) {
   //             changeSpeed();
   //          }
   //          break;
   //       case 5:
   //          while (!CenterPush) {
   //             RST_Gy();
   //          }
   //          break;
   //       case 6:
   //          while (!CenterPush) {
   //             printLine();
   //          }
   //          break;
   //       case 7:
   //          while (!CenterPush) {
   //             LineThUpdate();
   //          }
   //          break;
   //       case 8:
   //          writeEEPROM();
   //          break;
   //       default:
   //          break;
   //    }
   // }
   // if (isPushed) {
   //    int char_len = mode[status].length();
   //    int drawX = (display.width() / 2) - ((char_len / 2) * 12);
   //    display.clearDisplay();
   //    display.setTextSize(2);
   //    display.setTextColor(SSD1306_WHITE);
   //    display.setCursor(drawX, 7);
   //    display.println(mode[status]);
   //    display.drawLine(0, 32, 128, 32, SSD1306_WHITE);
   //    if (status <= mode_len - 2) {
   //       display.drawTriangle(90, 45, 90, 55, 100, 50, SSD1306_WHITE);
   //    }
   //    if (status >= 1) {
   //       display.drawTriangle(34, 45, 34, 55, 24, 50, SSD1306_WHITE);
   //    }
   //    display.display();
   // }
}
