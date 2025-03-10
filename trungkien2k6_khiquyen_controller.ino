// this is controller for trung kien 's project
// the other part of the project is in vscode platformio
// this is code for esp32-c3
// this code will be copy to arduino IDE

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <WiFiManager.h>
#include <SPI.h>
#include "RF24.h"
//=============================================================================
#define DEBUG
#define TIME_INTERVAL 2000
#define TOPIC "/esp/up-data"
#define SERVER "weatherinsight.xyz"
#define PORT 80
#define NRF_CE 8
#define NRF_CSN 7
#define WIND_PIN 3
//=============================================================================
// vairables declaration
uint8_t address[][6] = { "1Node", "2Node" };
unsigned long ulTime;
bool mode = 0;
bool isWifiConnected = true;
unsigned long lastSend = 0;

volatile int pulseCount = 0;
unsigned long lastCalWindSpeed = 0;
const float bladeCircumference = 2 * 3.14 * 0.09;  // in meters, example value

// bool isOLEDConnected = true;
struct SensorData {
  float fTempC;
  float fHumi;
  int iAQI;
  int iCO2;
  float fWindSpeed;
  float fPM1;
  float fPM2_5;
  float fPM10;
};
SensorData input;
//=============================================================================
// objects declaration
RF24 radio(NRF_CE, NRF_CSN);  // nrf also uses SPI : MISO(D6), MOSI(D7), SCK(D5)
SocketIOclient socketIO;
//=============================================================================
void readWindSpeed() {
  uint16_t data[6];
  while (1) {

    node_wind.begin(1, Serial1);
    delay(200);
    float result = node_wind.readHoldingRegisters(0x0000, 2);
    // float result = node_wind.readInputRegisters(0x0000, 2);
    if (result == node_wind.ku8MBSuccess) {
      uint16_t u16data[6];
      u16data[0] = node_wind.receive() / 10.0;
      input.fWindSpeed = (float)u16data[0];
  //     Serial.print("Wind speed: ");
  //     Serial.print(input.fWindSpeed);
  //     Serial.println("m/s");
  //   } else {
  //     Serial.println(result);
  //     Serial.println("Wind sensor not available!");
  //     break;
  //   }
  // }
}
//-----------------------------------------------------------------------------
void socketIOEvent(socketIOmessageType_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      Serial.printf("[IOc] get event: %s\n", payload);
      break;
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial.printf("[IOc] get binary: %u\n", length);
      break;
    case sIOtype_BINARY_ACK:
      Serial.printf("[IOc] get binary ack: %u\n", length);
      break;
  }
}
//-----------------------------------------------------------------------------
void sendDataToServer() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add(TOPIC);
  JsonObject data = array.createNestedObject();
  data["temp"] = input.fTempC;
  data["humi"] = input.fHumi;
  data["aqi"] = input.iAQI;
  data["co2"] = input.iCO2;
  data["wind"] = input.fWindSpeed;
  data["pm1"] = input.fPM1;
  data["pm25"] = input.fPM2_5;
  data["pm10"] = input.fPM10;
  String output;
  serializeJson(doc, output);
  Serial.println(output);
  socketIO.sendEVENT(output);
}
//=============================================================================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), countPulse, RISING);
  //---------------------------------------------------------------------------
  // init nrf24
  Serial.println("NRF24...");
  Serial.print(radio.begin() ? "done." : "failed!");
  radio.setPALevel(RF24_PA_MIN);
  // radio.setDataRate(RF24_250KBPS);
  // radio.setChannel(0x4c);
  // radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.startListening();
  //---------------------------------------------------------------------------
  // init wifi
  Serial.println("WiFi...");
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);  // wait 3 mins for config, or else use saved
  if (!wifiManager.autoConnect("ESP8266")) {
    Serial.println("failed to connect and hit timeout");
    isWifiConnected = false;
  } else {
    Serial.println("Connected.");
    isWifiConnected = true;
  }
  //---------------------------------------------------------------------------
  // Serial1.begin(4800, SERIAL_8O1, 3, 2);
  // node_wind.begin(1, Serial1);
  //---------------------------------------------------------------------------
  // server address, port and URL
  socketIO.begin(SERVER, PORT, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
}
//=============================================================================
void loop() {
  socketIO.loop();
  if (radio.available()) {
    radio.read(&input, sizeof(SensorData));
    Serial.println(input.fTempC);
    delay(100);
  }
  // printDebug(true, false, input);
  unsigned long currentTime = millis();
  if (currentTime - lastCalWindSpeed > 1000) {
    float rpm = (pulseCount / 20.0) * 60.0;
    input.fWindSpeed = (rpm * bladeCircumference) / 60.0;
    Serial.print("Wind Speed: ");
    Serial.print(input.fWindSpeed);
    Serial.println(" m/s");
    pulseCount = 0;          // Reset pulse count
    lastCalWindSpeed = currentTime;  // Update last time
  }
  if (currentTime - lastSend > 2000) {
    lastSend = millis();
    sendDataToServer();
  }
}

void countPulse() {
  pulseCount++;
}