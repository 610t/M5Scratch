/*
    Scratch remote sensor protocol

    You need to get ssid , password and host

    http://tiisai.dip.jp/?p=3665
*/

/*
  Copyright 2016,2019-2020 Takeshi MUTOH

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
   This program is demonstration that Scrath Remote Sensor Protocol with M5Stack.
*/

#define DEBUG_SERIAL false

/*
  network.h contains network information below:

  const char* ssid     = "SSID";
  const char* password = "PASSWORD";
  const char* host     = "Scratch Host IP";
*/
#include "network.h"

// Scratch Remote Sensor TCP/IP port
const int Port = 42001;

// Setting file for Scratch Host IP
#define HOST_IP_FILE "/M5Scratch.txt"

// M5Scratch moving cat demo mode
#define M5SCRATCH_DEMO

#include <WiFi.h>
#include <Wire.h>

#if defined(ARDUINO_M5Stick_C)
#include <M5StickC.h>
#define ROTATION 0
#elif defined(ARDUINO_M5Stick_C_Plus)
#include <M5StickCPlus.h>
#define ROTATION 0
#elif defined(ARDUINO_M5Stack_Core_ESP32)
// #define M5STACK_MPU6886
#define M5STACK_MPU9250
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <M5Stack.h>
#include <M5StackUpdater.h>
#define ROTATION 1

#if defined(M5STACK_MPU9250)
#include "utility/MPU9250.h"
MPU9250 Imu;
#endif

#define FACES_KEYBOARD_I2C_ADDR 0x08
#elif defined(ARDUINO_M5STACK_TOUGH)
#include <M5Tough.h>
#elif defined(ARDUINO_M5STACK_Core2)
#include <M5Core2.h>
#define ROTATION 1
#define Imu IMU
#elif defined(ARDUINO_M5Stack_ATOM)
#include <M5Atom.h>

uint8_t DisBuff[2 + 5 * 5 * 3];

void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++)
  {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}
#endif

#if defined(M5SCRATCH_DEMO)
#include <LovyanGFX.hpp>

extern const unsigned short cat[];

static constexpr unsigned short catWidth = 32;
static constexpr unsigned short catHeight = 32;

static uint32_t sec, psec;
static size_t fps = 0, frame_count = 0;
static uint32_t lcd_width ;
static uint32_t lcd_height;

int_fast16_t x;
int_fast16_t y;
float r;
float z;
float t;

static LGFX lcd;
static LGFX_Sprite sprites[2];
static LGFX_Sprite icons;
static int_fast16_t sprite_height;
#endif

#if !defined(ARDUINO_M5STACK_TOUGH) && !defined(ARDUINO_M5Stick_C_Plus)
#include "utility/MahonyAHRS.h"
#endif

String ip = "";
WiFiClient client;

void setup() {
  // Init Serial
  Serial.begin(115200);
  Serial.print("Searial start");
  delay(10);

  // Init M5
#if defined(ARDUINO_M5Stack_ATOM)
  M5.begin(true, false, true);
#else
  M5.begin();
#endif
  delay(100);

#if defined(ARDUINO_M5Stack_Core_ESP32)
  // for LovyanLauncher
  if (digitalRead(BUTTON_A_PIN) == 0) {
    if (DEBUG_SERIAL) Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }
#endif

#if defined(M5SCRATCH_DEMO)
  lcd.init();

  lcd_width = lcd.width();
  lcd_height = lcd.height();

  uint32_t div = 2;
  for (;;) {
    sprite_height = (lcd_height + div - 1) / div;
    bool fail = false;
    for (std::uint32_t i = 0; !fail && i < 2; ++i)
    {
      sprites[i].setColorDepth(lcd.getColorDepth());
      sprites[i].setFont(&fonts::Font2);
      fail = !sprites[i].createSprite(lcd_width, sprite_height);
    }
    if (!fail) break;
    for (std::uint32_t i = 0; i < 2; ++i)
    {
      sprites[i].deleteSprite();
    }
    ++div;
  }

  icons.createSprite(catWidth, catHeight);

  icons.setSwapBytes(true);

  icons.pushImage(0, 0, catWidth,  catHeight, cat);

  lcd.startWrite();
  lcd.setAddrWindow(0, 0, lcd_width, lcd_height);
#endif
#if !defined(ARDUINO_M5Stack_ATOM)
  lcd.setRotation(ROTATION);
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
  lcd.setTextSize(1);
#elif defined(ARDUINO_M5Stack_Core_ESP32) || defined(ARDUINO_M5STACK_Core2)
  lcd.setTextSize(2);
#endif
  lcd.println("Welcome to Scratch Remote  Sensor!!");
#endif

#if !defined(M5SCRATCH_DEMO)
#if defined(ARDUINO_M5Stack_ATOM)
  setBuff(0x20, 0x20, 0x20);
  M5.dis.displaybuff(DisBuff);

  // for ATOM Lite
  // M5.dis.drawpix(0, 0xffffff);
#else
  lcd.println("WiFi connected.");
#endif
#endif
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
    lcd.print(".");
#endif
    Serial.print(".");
  }

  Serial.println("Wifi OK");

  // Wire setup
  Wire.begin();
  pinMode(5, INPUT);
  digitalWrite(5, HIGH);

  // Accel & gyro (& mag for M5Stack)
#if !defined(ARDUINO_M5STACK_TOUGH)
#if !defined(M5STACK_MPU9250)
  M5.Imu.Init();
#else
  Imu.MPU9250SelfTest(Imu.SelfTest);

  Imu.calibrateMPU9250(Imu.gyroBias, Imu.accelBias);
  Imu.initMPU9250();

  delay(500);
  Imu.initAK8963(Imu.magCalibration);
#endif
#endif

  // LED
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, LOW);
#endif

  // Button
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(M5_BUTTON_RST, INPUT);
#endif

#if defined(ARDUINO_M5Stack_Core_ESP3)
  // Scratch Host IP setting use /M5Scratch.txt at SD.
  File f = SD.open(HOST_IP_FILE);
  if (f) {
    host = "";
    Serial.println("File "HOST_IP_FILE" open successfully.");
    while (f.available()) {
      char chr = f.read();
      ip = ip + chr;
    }
    f.close();
  } else {
    Serial.println("File open error "HOST_IP_FILE);
  }
  ip.trim();
  if (ip.length() != 0) {
    host = const_cast<char*>(ip.c_str());
  }
#endif
  Serial.println("Scratch Host IP is {" + String(host) + "}");
  lcd.println("Scratch Host IP is " + String(host));

  delay(1000);
}

String getValue(char name, String msg) {
  msg.replace(String(name) + " ", "");
  if (DEBUG_SERIAL) Serial.println("str:\"" + String(name) + ":" + String(msg) + "\"");
  return msg;
}

void broadcast(String msg) {
  char scmd[32] = {0};
  char buf[100] = {0};
  String cmd = "broadcast \"" + msg + "\"";

  cmd.toCharArray(buf, cmd.length() + 1);
  strcpy(scmd + 4, buf);
  //scmd[3] = (uint8_t)strlen(scmd + 4);
  scmd[3] = cmd.length();
  if (DEBUG_SERIAL) Serial.println(">pre broadcast:" + String(scmd + 4));
  client.setTimeout(100);
  //  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
  if (client.write((const uint8_t*)scmd, 4 + cmd.length())) {
    if (DEBUG_SERIAL) Serial.println(">>broadcast:" + msg + " ok");
  } else {
    if (DEBUG_SERIAL) Serial.println(">>broadcast:" + msg + " err");
  }
}

void sensor_update(String varName, String varValue) {
  char scmd[32] = {0};
  char buf[100] = {0};
  String cmd = "sensor-update \"" + varName + "\" " + varValue + " ";

  cmd.toCharArray(buf, cmd.length() + 1);
  sprintf(scmd + 4, buf);
  scmd[3] = (uint8_t)strlen(scmd + 4);
  client.setTimeout(100);
  if (client.write((const uint8_t*)scmd, 4 + strlen(scmd + 4))) {
    if (DEBUG_SERIAL) Serial.println("sensor-update ok");
    return;
  } else {
    if (DEBUG_SERIAL) Serial.println("sensor-update err");
    return;
  }
}

void loop() {
  uint8_t buffer[128] = {0};
  int r = 0, g = 0, b = 0, led = 0;
  uint16_t rgb, old_rgb;
  int circle_r;
  String s;
  char* str;

  // Set circle radius
  if (ROTATION == 1 || ROTATION == 3) {
    circle_r = lcd_width / 12;
  } else if (ROTATION == 0 || ROTATION == 2) {
    circle_r = lcd_height / 12;
  }

  M5.update();

  if (DEBUG_SERIAL) Serial.println("Before client connect");
  while (!client.connect(host, Port)) {
    Serial.println("Scratch Host IP is {" + String(host) + "}");
    Serial.println("connection failed");
  }
  if (DEBUG_SERIAL) Serial.println("create tcp ok");

  while (!client.connected()) {
    Serial.println("Stop connection");
    client.stop();
    Serial.println("Before client.connect");
    //client.setTimeout(100);
    client.connect(host, Port);
    Serial.println("After client.connect");
  }
  if (DEBUG_SERIAL) Serial.println("Client connected");

  // Read all from server and print them to Serial.
  uint32_t len = 0;
  String msg = "";
  char *c;

  if (DEBUG_SERIAL) Serial.println("Let us go to read messages.");

  //// Receive msg
  len = 0;
  int av = client.available();
  if (DEBUG_SERIAL) Serial.println("available:" + String(av));
  //if (av > 0) {
  client.setTimeout(100);
  len = client.readBytes(buffer, sizeof(buffer));
  //}

  if (DEBUG_SERIAL) Serial.println("Get length:" + String(len));

  while (len > 0) {
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
    lcd.setCursor(0, 0);
#endif
    if (DEBUG_SERIAL) Serial.print("Received:[");
    // Skip 4 byte message header and get string.
    for (uint32_t i = 4; i < len; i++) {
      if (DEBUG_SERIAL) Serial.print((char)buffer[i]);
      msg += (char)buffer[i];
    }
    if (DEBUG_SERIAL) Serial.print("]\r\n");

    while ((!msg.startsWith("broadcast") && !msg.startsWith("sensor-update")) && msg.length() > 0 ) {
      msg = msg.substring(1);
    }

    if (msg.startsWith("broadcast") == true) {
      // message
      msg.replace("broadcast ", "");
      msg.replace("\"", "");
      if (DEBUG_SERIAL) Serial.println("broadcast:\"" + msg + "\"");
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
      lcd.println("broadcast:\"" + msg + "\"");
#endif
    } else if (msg.startsWith("sensor-update")) {
      // value
      msg.replace("sensor-update ", "");
      msg.replace("\"", "");
      msg.trim();
      //lcd.println("sensor-update\"" + msg + "\"");

      while (msg.length() > 0) {
        msg.trim();
        if (msg.startsWith("r") == true) {
          // RGB color red
          r = constrain(int(getValue('r', msg).toFloat()), 0, 255);
        } else if (msg.startsWith("g") == true) {
          // RGB color green
          g = constrain(int(getValue('g', msg).toFloat()), 0, 255);
        } else if (msg.startsWith("b") == true) {
          // RGB color blue
          b = constrain(int(getValue('b', msg).toFloat()), 0, 255);
        } else if (msg.startsWith("s") == true) {
          // Some sting
          s = getValue('s', msg);
        } else if (msg.startsWith("l") == true) {
          // LED on/off
          led =  int(getValue('l', msg).toFloat());
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
          digitalWrite(M5_LED, led);
#endif
        } else if (msg.startsWith("x") == true) {
          // Cat x axis location
          x = constrain(int(getValue('x', msg).toFloat()), -240, 240);
        } else if (msg.startsWith("y") == true) {
          // Cat y axis location
          y = constrain(int(getValue('y', msg).toFloat()), -180, 180);
        } else if (msg.startsWith("z") == true) {
          // Cat zoom value
          z = constrain(getValue('z', msg).toFloat(), 1, 10);
        } else if (msg.startsWith("t") == true) {
          // Cat direction theta
          t = constrain(int(getValue('t', msg).toFloat()), -180, 180);
        }
        //Serial.println("msg:\"" + msg + "\"");

        // Skip var_value
        while (msg.charAt(0) != ' ' && msg.length() > 0) {
          msg = msg.substring(1);
        }
        //Serial.println("msg2:\"" + msg + "\"");
      }
      rgb = uint16_t (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
      //// Output
#if defined(M5SCRATCH_DEMO)
      static uint8_t flip = 0;

      // Moving Cat
      for (int_fast16_t yy = 0; yy < lcd_height; yy += sprite_height) {
        flip = flip ? 0 : 1;
        sprites[flip].clear();

        if (DEBUG_SERIAL) Serial.println("(x,y)=(" + String(x) + "," + String(y) + ")");
        icons.pushRotateZoom(&sprites[flip]
                             , int((x + 240) / 480.0 * lcd_width)
                             , 240 - int((y + 180) / 360.0 * lcd_height) - yy
                             , t, z, z, 0);
        // icons.pushRotateZoom(&sprites[flip], x, y - yy, t, z, z, 0);

        if (yy == 0) {
          // Circle with RGB color
          sprites[flip].fillCircle(lcd_width - circle_r, circle_r, circle_r, rgb);
          // RGB line
          sprites[flip].drawGradientLine(0, 0, lcd_width, 0, old_rgb, rgb);

          // Print out variables
          sprites[flip].setFont(&fonts::Font4);
          sprites[flip].setTextColor(0xFFFFFFU);
          sprites[flip].setCursor(0, 0);
          sprites[flip].printf("fps:%d\n", fps);
          sprites[flip].printf("(x,y)=(%d,%d):t=%d,z=%d\n", int(x), int(y), int(t), int(z));
          sprites[flip].printf("(r,g,b)=(%d,%d,%d)\n", r, g, b);
          sprites[flip].printf("S:%s\n", s);
        }
        size_t len = sprite_height * lcd_width;
        if (yy + sprite_height > lcd_height) {
          len = (lcd_height - yy) * lcd_width;
        }
        lcd.pushPixelsDMA(sprites[flip].getBuffer(), len);
      }

      ++frame_count;
      sec = millis() / 1000;
      if (psec != sec) {
        psec = sec;
        fps = frame_count;
        frame_count = 0;
        lcd.setAddrWindow(0, 0, lcd.width(), lcd.height());
      }
#endif

      // RGB background
#if defined(ARDUINO_M5Stack_ATOM)
      // for ATOM Matrix
      // Conver raw rgb(0-255) to led rgb(0-0x20)
      int rl = constrain(int((r / 255.0) * 0x20), 0, 0x20);
      int gl = constrain(int((g / 255.0) * 0x20), 0, 0x20);
      int bl = constrain(int((b / 255.0) * 0x20), 0, 0x20);
      if (DEBUG_SERIAL) Serial.println("LED RGB:(" + String(rl) + ", " + String(gl) + ", " + String(bl) + ")");

      setBuff(rl, gl, bl);
      M5.dis.displaybuff(DisBuff);

      // for ATOM Lite
      // Conver raw rgb(0-255) to led rgb(0-0xf)
      //rl = constrain(int((r / 255.0) * 0xf), 0, 0xf);
      //gl = constrain(int((g / 255.0) * 0xf), 0, 0xf);
      //bl = constrain(int((b / 255.0) * 0xf), 0, 0xf);
      //M5.dis.drawpix(0, (((gl & 0xf) << 12) | ((rl & 0xf) << 8) | ((bl & 0xf) << 4)));
#elif defined(M5SCRATCH_DEMO)
      // Very slow
      // lcd.fillScreen(rgb);
#else
      lcd.fillScreen(rgb);
      lcd.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
#endif
      if (DEBUG_SERIAL) Serial.println("RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")");
      // msg
      //lcd.setCursor(0, 100);
      //lcd.setTextSize(5);
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
      lcd.println("s:\"" + s + "\"");
#endif
    } else {
      if (DEBUG_SERIAL) Serial.println("NOP");
    }

    old_rgb = rgb;
    len = msg.length();
  }

  // broadcast
  broadcast("test");

  // broadcast by button
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    broadcast("BtnA");
  }
  if (digitalRead(M5_BUTTON_RST) == LOW) {
    broadcast("BtnB");
  }
#elif defined(ARDUINO_M5Stack_Core_ESP32)
  if (M5.BtnA.isPressed()) {
    broadcast("BtnA");
  }
  if (M5.BtnB.isPressed()) {
    broadcast("BtnB");
  }
  if (M5.BtnC.isPressed()) {
    broadcast("BtnC");
  }
#elif defined(ARDUINO_M5Stack_ATOM)
  if (M5.Btn.wasPressed())
  {
    broadcast("BtnA");
  }
#endif

#if defined(ARDUINO_M5Stack_Core_ESP32)
  // keyboard input
  if (digitalRead(5) == LOW)
  {
    Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
    while (Wire.available())
    {
      char c = Wire.read(); // receive a byte as character
      if (DEBUG_SERIAL) Serial.print(c);        // print the character
      broadcast("Key_" + String(c));
    }
  }
#endif

  // sensor-update
  sensor_update("v", String(random(0, 255)));


  // define all sensor valiable.
  float ax = 0;
  float ay = 0;
  float az = 0;

  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;

  float mx = 0;
  float my = 0;
  float mz = 0;

  float pitch = 0;
  float roll = 0;
  float yaw = 0;

  float heading = 0;

  float temp = 0;

#if !defined(ARDUINO_M5STACK_TOUGH)
#if !defined(M5STACK_MPU9250)
  M5.Imu.getAccelData(&ax, &ay, &az);         // get accel
  M5.Imu.getGyroAdc(&gyroX, &gyroY, &gyroZ);  // get gyro
  M5.Imu.getTempData(&temp);                  // get temp
#else
  if (Imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    // get accel
    Imu.readAccelData(Imu.accelCount);  // Read the x/y/z adc values
    Imu.getAres();
    ax = (float)Imu.accelCount[0] * Imu.aRes; // - accelBias[0];
    ay = (float)Imu.accelCount[1] * Imu.aRes; // - accelBias[1];
    az = (float)Imu.accelCount[2] * Imu.aRes; // - accelBias[2];

    // get gyro
    Imu.readGyroData(Imu.gyroCount);
    Imu.getGres();

    gyroX = (float)Imu.gyroCount[0] * Imu.gRes;
    gyroY = (float)Imu.gyroCount[1] * Imu.gRes;
    gyroZ = (float)Imu.gyroCount[2] * Imu.gRes;

    // get magnetic
    Imu.readMagData(Imu.magCount);
    Imu.getMres();

    mx = (float)Imu.magCount[0] * Imu.mRes * Imu.magCalibration[0] - 470;
    my = (float)Imu.magCount[1] * Imu.mRes * Imu.magCalibration[1] - 120;
    mz = (float)Imu.magCount[2] * Imu.mRes * Imu.magCalibration[2] - 125;

    // get heading
    float offset_mx = 1944.21;
    float offset_my = 1204.83;
    heading = atan2(my - offset_my, mx - offset_mx) * 180.0 / M_PI;

    // get temp
    Imu.tempCount = Imu.readTempData();
    temp = ((float) Imu.tempCount) / 333.87 + 21.0;

    Imu.updateTime();
  }
#endif

#if !defined(ARDUINO_M5Stick_C_Plus)
  // Calculate pitch, roll, yaw
  MahonyAHRSupdateIMU(gyroX, gyroY, gyroZ, ax, ay, az, &pitch, &roll, &yaw);
#endif
#endif

  // send sensor-update

  // sensor-update accel
#if defined(ARDUINO_M5Stick_C) || defined(ARDUINO_M5Stick_C_Plus)
  // Rotation is different from landscape.
  sensor_update("ax", String(-1 * 240 * ay));
  sensor_update("ay", String(+1 * 180 * ax));
#elif defined(ARDUINO_M5Stack_Core_ESP32) || defined(ARDUINO_M5STACK_Core2)
  sensor_update("ax", String(-1 * 240 * ax));
  sensor_update("ay", String(-1 * 180 * ay));
#elif defined(ARDUINO_M5Stack_ATOM)
  sensor_update("ax", String(+1 * 240 * ax));
  sensor_update("ay", String(-1 * 180 * ay));
#endif
  sensor_update("az", String(1000 * az));
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("accel:(" + String(ax) + ", " + String(ay) + ", " + String(az) + ")");
#endif

  // sensor-update gyro
  sensor_update("gx", String(gyroX));
  sensor_update("gy", String(gyroY));
  sensor_update("gz", String(gyroZ));
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("gyro:(" + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + ")");
#endif

#if !defined(ARDUINO_M5STACK_TOUGH)
#if defined(M5STACK_MPU9250)
  // sensor-update magnetic
  sensor_update("mx", String(mx));
  sensor_update("my", String(my));
  sensor_update("mz", String(mz));
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("mag:(" + String(mx) + ", " + String(my) + ", " + String(mz) + ")");
#endif

  // sensor-update heading
  sensor_update("heading", String(heading));
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("heading:" + String(heading));
#endif
#endif
#endif

  // sensor-update pitch, roll, yaw
  sensor_update("pitch", String(pitch));
  sensor_update("roll", String(roll));
  sensor_update("yaw", String(yaw));
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("p,r,y:(" + String(pitch) + ", " + String(roll) + ", " + String(yaw) + ")");
#endif

  // sensor-update temp
#if !defined(ARDUINO_M5Stack_ATOM) && !defined(M5SCRATCH_DEMO)
  lcd.println("temp:" + String(temp));
#endif
  sensor_update("temp", String(temp));

  client.stop();
}

constexpr unsigned short cat[1024] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 0, 32 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 1, 64 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 2, 96 pixels
  0x0000, 0x0000, 0x0000, 0x39A6, 0x3165, 0x2945, 0x2945, 0x3186, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2925, 0x2925, 0x2945, 0x2124, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 3, 128 pixels
  0x0000, 0x0000, 0x0000, 0x39A6, 0x0000, 0x2104, 0x20E4, 0x0005, 0x0044, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x08A4, 0x0025, 0x20E4, 0x2104, 0x20E3, 0x2945, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 4, 160 pixels
  0x0000, 0x0000, 0x0000, 0x5269, 0x49A3, 0x3964, 0x6A63, 0x4184, 0x0864, 0x18E3, 0x000A, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0884, 0x0004, 0x3144, 0x7AA3, 0x49A3, 0x0006, 0x2945, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 5, 192 pixels
  0x0000, 0x0000, 0x0000, 0x2925, 0x4183, 0x3964, 0xC403, 0xBBE3, 0x5A03, 0x18C4, 0x0006, 0x0045, 0x0000, 0x0000, 0x0000, 0x0008, 0x2121, 0x0024, 0x3964, 0x9B43, 0xECC3, 0x6A63, 0x0024, 0x2104, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 6, 224 pixels
  0x0000, 0x0000, 0x0000, 0x3165, 0x3123, 0x2924, 0xBBE3, 0xFD23, 0xDC83, 0x82C3, 0x3144, 0x2104, 0x3964, 0x49C4, 0x51E4, 0x49C4, 0x4184, 0x5A03, 0xABA3, 0xF503, 0xFD03, 0x8AE3, 0x10A4, 0x2104, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 7, 256 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x2903, 0x2104, 0xABA3, 0xFD23, 0xFD23, 0xF4E3, 0xABA3, 0xA363, 0xCC23, 0xD463, 0xDC83, 0xDC63, 0xD443, 0xDC83, 0xF4E3, 0xFD03, 0xFD23, 0xAB83, 0x20E4, 0x2103, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 8, 288 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x2104, 0x18E4, 0xA363, 0xFD23, 0xF503, 0xFD03, 0xFD23, 0xF503, 0xFD23, 0xFD23, 0xFD23, 0xFD23, 0xFD23, 0xFD03, 0xFD03, 0xFD03, 0xFD03, 0xD443, 0x51E3, 0x10A4, 0x2103, 0x0085, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 9, 320 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x2104, 0x10A4, 0x9323, 0xFD23, 0xFD03, 0xFD03, 0xFD03, 0xF503, 0xF503, 0xFD03, 0xFD03, 0xFD03, 0xFD03, 0xFD03, 0xFD03, 0xE4C4, 0xC48A, 0xC4CB, 0xA3A5, 0x59E3, 0x18C4, 0x2104, 0x10E5, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 10, 352 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x2104, 0x0064, 0x82E3, 0xF503, 0xFD03, 0xF503, 0xD485, 0xC4AB, 0xC489, 0xDC84, 0xF503, 0xFD03, 0xF503, 0xFD03, 0xF503, 0xC4AA, 0xEF5D, 0xF7BF, 0xD657, 0xA3A7, 0x51E3, 0x10A4, 0x18E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 11, 384 pixels
  0x0000, 0x0000, 0x0000, 0x3165, 0x2903, 0x18E4, 0x9303, 0xFD03, 0xFD03, 0xD485, 0xD616, 0xF7BF, 0xF79E, 0xCDD4, 0xD466, 0xFD03, 0xF503, 0xFD03, 0xF4E3, 0xC52F, 0xFFFF, 0xFFFF, 0xFFFF, 0xD657, 0x9B45, 0x4183, 0x0009, 0x2924, 0x0000, 0x0000, 0x0000, 0x0000, // row 12, 416 pixels
  0x0000, 0x0000, 0x0000, 0x20E4, 0x0044, 0x5A03, 0xDC83, 0xFD03, 0xF503, 0xC4AB, 0xF7BE, 0xFFFF, 0xFFFF, 0xFFFF, 0xCDB3, 0xDC84, 0xFD03, 0xFD03, 0xF4E3, 0xC4CC, 0xFFDF, 0xFFFF, 0xFFFF, 0xFFDF, 0xBCED, 0x7263, 0x10A4, 0x2104, 0x0000, 0x0000, 0x0000, 0x0000, // row 13, 448 pixels
  0x0000, 0x0000, 0x2945, 0x3963, 0x2924, 0xAB83, 0xFD23, 0xFD03, 0xF503, 0xC4CB, 0xF7DF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEF7D, 0xC489, 0xF503, 0xFD03, 0xFD03, 0xCC66, 0xE6DA, 0xFFFF, 0xD69A, 0xDEDB, 0xD615, 0x9B43, 0x3144, 0x3143, 0x0000, 0x0000, 0x0000, 0x0000, // row 14, 480 pixels
  0x4163, 0x4143, 0x2104, 0x0004, 0x5A23, 0xE4A3, 0xFD03, 0xFD03, 0xFD03, 0xCC87, 0xEF3C, 0xFFFF, 0xE71C, 0xBDD7, 0xFFDF, 0xC50E, 0xF4E3, 0xFD03, 0xFD03, 0xECC3, 0xC4ED, 0xF79E, 0xB5B6, 0xBDF7, 0xD657, 0xBC04, 0x49A3, 0x000C, 0x10A2, 0x0000, 0x0000, 0x0000, // row 15, 512 pixels
  0x4143, 0x4143, 0x4143, 0x20E4, 0x9303, 0xFD03, 0xFD03, 0xFD03, 0xFD03, 0xE4A3, 0xC571, 0xFFFF, 0xDEDB, 0xAD55, 0xFFDF, 0xC4ED, 0xF4E3, 0xF503, 0xF503, 0xF503, 0xE4A4, 0xC551, 0xE6FB, 0xEF3C, 0xC5D6, 0xE653, 0x8349, 0x3164, 0x0000, 0x1061, 0x4143, 0x4163, // row 16, 544 pixels
  0x4143, 0x4163, 0x4163, 0x3123, 0xA343, 0xFD03, 0xFD03, 0xFD03, 0xF503, 0xFD03, 0xD465, 0xCD93, 0xF77D, 0xFFDF, 0xDEBA, 0xCDD4, 0xFE4F, 0xD4A8, 0x8B06, 0x8B26, 0x9B87, 0xDE98, 0xD679, 0xCE38, 0xEF3C, 0xFFFF, 0xF79D, 0xB553, 0x4A07, 0x20A2, 0x4143, 0x4143, // row 17, 576 pixels
  0x0000, 0x0000, 0x3943, 0x3144, 0xAB83, 0xF503, 0xFD03, 0xFD03, 0xF503, 0xFD24, 0xFDCB, 0xE633, 0xC5F7, 0xC5F7, 0xD679, 0xFFDF, 0xFFFF, 0xEF5C, 0x9C71, 0x6289, 0x8BEF, 0xF7BE, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEF5C, 0x8C0F, 0x3964, 0x4143, 0x4143, // row 18, 608 pixels
  0x4163, 0x4143, 0x4163, 0x3944, 0xAB83, 0xFD03, 0xFD03, 0xFD03, 0xFD24, 0xFEB2, 0xFFDE, 0xFFFF, 0xFFFF, 0xF7BE, 0xE6FB, 0xF79D, 0xFFFF, 0xFFFF, 0xFFDF, 0xDEBA, 0xEF5D, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEF3C, 0x5ACA, 0x0000, 0x2925, // row 19, 640 pixels
  0x4163, 0x4143, 0x4143, 0x2903, 0x8AE3, 0xFD03, 0xFD03, 0xFD03, 0xFD87, 0xFFBD, 0xFFFF, 0xFFFF, 0xFFFF, 0xE6FB, 0xBD95, 0xCE17, 0xC617, 0xDEBA, 0xEF7D, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xF7BE, 0xE6FB, 0x630B, 0x0000, 0x2124, // row 20, 672 pixels
  0x0000, 0x0000, 0x2104, 0x0004, 0x5A03, 0xDC83, 0xFD23, 0xFD03, 0xFD87, 0xFFBC, 0xFFFF, 0xFFFF, 0xFFFF, 0xD699, 0xE6FB, 0xFFFF, 0xF77D, 0xDEBA, 0xCE17, 0xC5F7, 0xCE58, 0xDEDA, 0xEF3C, 0xF7BE, 0xFFFF, 0xFFFF, 0xFFFF, 0xF7BE, 0xB575, 0x49E6, 0x3923, 0x4143, // row 21, 704 pixels
  0x0000, 0x0000, 0x3186, 0x2903, 0x20E4, 0x8AE3, 0xF503, 0xFD03, 0xFD24, 0xFF17, 0xFFFF, 0xFFFF, 0xFFFF, 0xDEDA, 0xD699, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xF7BE, 0xEF3C, 0xDEBA, 0xC5F7, 0xB534, 0xF7BE, 0xFFFF, 0xFFFF, 0xFFFF, 0xA4F3, 0x2904, 0x4943, 0x4143, // row 22, 736 pixels
  0x0000, 0x0000, 0x0000, 0x18E4, 0x0008, 0x3144, 0x9B43, 0xF503, 0xFD03, 0xFDCB, 0xFFBD, 0xFFFF, 0xFFFF, 0xF7BE, 0xC5D6, 0xEF5D, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEF5D, 0xC5F7, 0xE6FB, 0xFFFF, 0xFFFF, 0xFFFF, 0xC5F6, 0x4A07, 0x0000, 0x20E4, 0x0000, // row 23, 768 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x10A4, 0x0004, 0x3964, 0x9323, 0xECC3, 0xFD23, 0xFE0D, 0xFFBC, 0xFFFF, 0xFFFF, 0xEF5D, 0xC5F7, 0xD679, 0xEF3C, 0xF77D, 0xE71C, 0xCE38, 0xC5F7, 0xEF5D, 0xFFFF, 0xFFFF, 0xFFBE, 0xBD73, 0x5227, 0x1062, 0x18C2, 0x4209, 0x0000, // row 24, 800 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0064, 0x0005, 0x2904, 0x6A43, 0xBBE3, 0xECC3, 0xFDC9, 0xFF17, 0xFFFE, 0xFFFF, 0xFFDF, 0xE6FB, 0xCE58, 0xCE38, 0xD659, 0xE73C, 0xFFFF, 0xFFFF, 0xFFBE, 0xD637, 0x83AC, 0x3984, 0x0862, 0x18A2, 0x08A5, 0x0000, 0x0000, // row 25, 832 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x62C3, 0x0883, 0x3143, 0x6A63, 0xA363, 0xCC44, 0xE58C, 0xF6D6, 0xFF9C, 0xFFFF, 0xFFFF, 0xFFDF, 0xF79E, 0xE73C, 0xCE59, 0xA514, 0x736C, 0x41C5, 0x18A2, 0x0002, 0x0022, 0x0000, 0x0000, 0x0000, 0x0000, // row 26, 864 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0004, 0x0004, 0x0023, 0x20E3, 0x3963, 0x51E3, 0x6244, 0x7309, 0x7BAE, 0x83EF, 0x7BAE, 0x6B2C, 0x528A, 0x39A6, 0x20E3, 0x0820, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 27, 896 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0004, 0x0003, 0x0004, 0x0003, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 28, 928 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 29, 960 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 30, 992 pixels
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // row 31, 1024 pixels
};
