#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_6Axis_MotionApps20.h>
// #include "ssd1306.h"

#define LeftPin 33
#define RightPin 27

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define SIZE_OF_ARRAY(array) (sizeof(array) / sizeof(array[0]))

#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C

#define w display.width()
#define h display.height()

#define LeftPush isPush(30)
#define CenterPush isPush(33)
#define OnOffPush isPush(31)
#define RightPush isPush(27)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

int speed = 150;
bool isFirstSetSpeed = true;

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

int getVah(int f) {
   byte val = 0;
   Wire.beginTransmission(0x0E);
   Wire.write(f);
   Wire.endTransmission();
   Wire.requestFrom(0x0E, 1);
   while (Wire.available()) {
      val = Wire.read();
   }
   return (int)val;
}

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

   if (i != 1) {
      return re_strength;
   }
   else {
      return re_angle * 5;
   }
}

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

void display_init() {
   if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      // for (;;) ;
   }
}

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

void LineThUpdate() {
   String txt = "Updating..";
   int char_len = txt.length();
   int drawX = (display.width() / 2) - ((char_len / 2) * 11);
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(drawX, 30);
   display.println(txt);
   display.display();
}

int speed_status = 0;
bool speed_isPushed = false;

String speed_mode[] = {"0", "10", "50", "100", "150", "170", "200", "220", "250"};
int speed_mode_len = SIZE_OF_ARRAY(speed_mode);

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

bool Kicking = false;

void kick() {
   digitalWrite(10, HIGH);
   Kicking = true;
   delay(60);
   digitalWrite(10, LOW);
   Kicking = !Kicking;
   delay(60);
}

String mode[] = {"Main", "Ball", "Gyro", "Kick", "Speed", "LineUpdate", "GK"};
int mode_len = SIZE_OF_ARRAY(mode);

void setup() {
   pinMode(27, INPUT);
   pinMode(33, INPUT);
   pinMode(34, INPUT);
   pinMode(35, INPUT);
   pinMode(10, OUTPUT);
   Serial.begin(9600);
   Wire.begin();
   display_init();
   selectGyro(3);
   Gryo_init();
   int len = 8;
   int drawX = (display.width() / 2) - ((len / 2) * 12);
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(drawX, 10);
   display.println("Main.cpp");
   display.display();
}

int status = 0;
bool isPushed = false;

void loop() {
   if (LeftPush) {
      isPushed = true;
      if ((status <= mode_len - 1) && (status >= 1)) {
         status--;
      }
   }
   if (RightPush) {
      isPushed = true;
      if ((status <= mode_len - 2) && (status >= 0)) {
         status++;
      }
   }
   if (CenterPush) {
      switch (status) {
         case 1:
            while (!CenterPush) {
               printIR();
            }
            break;
         case 2:
            while (!CenterPush) {
               printIMU();
            }
            break;
         case 3:
            while (!CenterPush) {
               printKick();
               if (!Kicking && digitalRead(31)) {
                  kick();
               }
            }
            break;
         case 4:
            if (isFirstSetSpeed) {
               int char_len = ((String)speed).length();
               int drawX = (display.width() / 2) - ((char_len / 2) * 11);
               display.clearDisplay();
               display.setTextSize(2);
               display.setTextColor(SSD1306_WHITE);
               display.setCursor(drawX, 7);
               display.println(speed);
               display.drawLine(0, 32, 128, 32, SSD1306_WHITE);
               if (atoi((speed_mode[0]).c_str()) < speed) {
                  display.drawTriangle(90, 45, 90, 55, 100, 50, SSD1306_WHITE);
               }
               if (atoi((speed_mode[speed_mode_len - 1]).c_str()) > speed) {
                  display.drawTriangle(34, 45, 34, 55, 24, 50, SSD1306_WHITE);
               }
               for (int i = 0; i < speed_mode_len; i++) {
                  if (atoi((speed_mode[i]).c_str()) == speed) {
                     speed_status = i + 1;
                     break;
                  }
                  if (atoi((speed_mode[i]).c_str()) > speed) {
                     speed_status = i;
                     break;
                  }
               }
               speed = atoi((speed_mode[speed_status]).c_str());
               display.display();
               isFirstSetSpeed = !isFirstSetSpeed;
            }
            while (!CenterPush) {
               changeSpeed();
            }
         default:
            break;
      }
   }
   if (isPushed) {
      int char_len = mode[status].length();
      int drawX = (display.width() / 2) - ((char_len / 2) * 12);
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(drawX, 7);
      display.println(mode[status]);
      display.drawLine(0, 32, 128, 32, SSD1306_WHITE);
      if (status <= mode_len - 2) {
         display.drawTriangle(90, 45, 90, 55, 100, 50, SSD1306_WHITE);
      }
      if (status >= 1) {
         display.drawTriangle(34, 45, 34, 55, 24, 50, SSD1306_WHITE);
      }
      display.display();
   }
}