/*
    Scratch remote sensor protocol

    You need to get ssid , password and host

    http://tiisai.dip.jp/?p=3665
*/

/*
  Copyright 2016, 2021 Takeshi MUTOH

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
   This program is demonstration that Scrath Remote Sensor Protocol with ESP8266.

   My presentation is below:
   http://qml.610t.org/squeak/CodarDojoNara201612.html

*/

#define DEBUG_SERIAL false

// Edit these 3 lines for Network settings.
const char* ssid     = "YOUR SSID";
const char* password = "YOUR PASSWORD";
const char* host = "SCRATCH HOST IP ADDRESS";

/*
   Sensors & Actuators

   for WeMOS D1 mini / ESPr Developer (for D1 Mini ESP32)
   D0(,26):
   D1(GPIO5,22):  SCL
   D2(GPIO4,21):  SDA, RGB LED
   D3(GPIO0,17):  Button switch
   D4(GPIO2,16):  DHT sensor (DHT11), RGB LED
   D5(GPIO14,18): Matrix LED CLK, Buzzer
   D6(GPIO12,19)
   D7(GPIO13,23): Matrix LED DIN
   D8(GPIO15,05)
   D13():      Built-in LED
   A0(,SVP):

  I2C(D1:SCL, D2:SDA)
    OLED
    Barometric Pressure(HP303B)
    SHT30
    DHT
    Ambient light(BH1750)
*/
// #define BOARD_WEMOS

#if defined(ARDUINO_D1_MINI32)
// #define LED_BUILTIN 1
#define RGBLEDPIN   21
#define BUTTONPIN   17
#define DHTPIN      16
#elif defined(BOARD_WEMOS)
// #define LED_BUILTIN 1
#define RGBLEDPIN   D2
#define BUTTONPIN   D3
#define DHTPIN      D4
#endif

#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RGBLEDPIN, NEO_GRB + NEO_KHZ800);

#include "DHT.h"

#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

const int Port = 42001;

int r = 0, g = 0, b = 0;

WiFiClient client;

void setup() {
  // Set up I/O mode
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT);

  // Initialize Serial
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // This initializes the NeoPixel library.
  pixels.begin();

  // Initialize DHT sensor
  dht.begin();
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

int value = 0;

void loop() {
  uint8_t buffer[128] = {0};
  float t, t_old, h, h_old;

  ++value;
  // Blink Builtin LED
  if (value % 2 == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  Serial.print("connecting to ");
  Serial.println(host);

  if (!client.connect(host, Port)) {
    Serial.println("connection failed");
    return;
  }

  Serial.print("create tcp ok\r\n");

  for (uint32_t n = 0; n < 100000; n++) {
    int sw, stat;

    // Read all from server and print them to Serial
    client.setTimeout(100);
    uint32_t len = client.readBytes(buffer, sizeof(buffer));
    String msg = "";
    if (len > 0) {
      if (DEBUG_SERIAL) Serial.print("Received:[");
      for (uint32_t i = 0; i < len; i++) {
        if (DEBUG_SERIAL) Serial.print((char)buffer[i]);
        if (i >= 4) { // Skip 4 byte message header
          msg += (char)buffer[i];
        }
      }
      if (DEBUG_SERIAL) Serial.print("]\r\n");

      // Skip until broadcast or sensor-update
      while ((!msg.startsWith("broadcast") && !msg.startsWith("sensor-update")) && msg.length() > 0 )
      {
        msg = msg.substring(1);
      }

      if (msg.startsWith("broadcast") == true) {
        // message
        msg.replace("broadcast ", "");
        msg.replace("\"", "");
        Serial.print("{broadcast:" + msg + "}");
      } else if (msg.startsWith("sensor-update")) {
        // value
        msg.replace("sensor-update ", "");
        msg.replace("\"", "");
        msg.trim();

        while (msg.length() > 0) {
          msg.trim();
          switch (msg.charAt(0)) {
            case 'r':
              msg.replace("r ", "");
              r = int(msg.toFloat());
              Serial.println("{R:" + String(r) + "}");
              break;
            case 'g':
              msg.replace("g ", "");
              g = int(msg.toFloat());
              Serial.println("{G:" + String(g) + "}");
              break;
            case 'b':
              msg.replace("b ", "");
              b = int(msg.toFloat());
              Serial.println("{B:" + String(b) + "}");
              break;
          }
          Serial.println("{{msg:" + msg + "}}");

          // Skip var_value
          while (msg.charAt(0) != ' ' && msg.length() > 0) {
            msg = msg.substring(1);
          }
          Serial.println("{{msg2:" + msg + "}}");
        }
        Serial.println("{RGB:(" + String(r) + ", " + String(g) + ", " + String(b) + ")}");
      } else {
        if (DEBUG_SERIAL) Serial.println("NOP");
      }
      len = msg.length();
    }

    // RGB LED
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();

    // send broadcast message
    if (digitalRead(BUTTONPIN) == LOW) {
      broadcast("btn");
    }

    sensor_update("v", String(value));
    sensor_update("rand", String(random(255)));

    // Get DHT data
    t = NAN;
    h = NAN;
    while ( isnan(t) || isnan(h) ) {
      t = dht.readTemperature();
      h = dht.readHumidity();
    }

    if ( t != t_old ) {
      sensor_update("t", String(t));
    }

    if ( h != h_old ) {
      sensor_update("h", String(h));
    }

    // Store current h & t
    h_old = h;
    t_old = t;
  }
}
