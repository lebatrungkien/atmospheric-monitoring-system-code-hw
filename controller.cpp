// // this is controller for trung kien 's project
// // the other part of the project is in vscode platformio
// // this is code for nodeMCU ESP8266
// // this code will be copy to arduino IDE
// 
// #include <Arduino.h>
// #include <ESP8266WiFi.h>
// #include <ArduinoJson.h>
// #include <WebSocketsClient.h>
// #include <SocketIOclient.h>
// #include <WiFiManager.h>
// #include <SPI.h>
// #include "RF24.h"
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// 
// //=============================================================================
// #define DEBUG
// #define TIME_INTERVAL 2000
// #define TOPIC "/esp/up-data"
// #define SERVER ""
// #define PORT 80
// #define NRF_CE D4
// #define NRF_CSN D8
// #define BTN1_PIN D3
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define SCREEN_ADDRESS 0x3C
// #define OLED_RESET -1 // Reset pin
// //=============================================================================
// // vairables declaration
// uint8_t address[][6] = {"1Node", "2Node"};
// unsigned long ulTime;
// bool mode = 1;
// bool isWifiConnected = true;
// bool isOLEDConnected = true;
// struct SensorData
// {
//     float fTempC;
//     float fHumi;
//     int iAQI;
//     int iCO2;
//     float fWindSpeed;
//     float fPM1;
//     float fPM2_5;
//     float fPM10;
// };
// SensorData input;
// //=============================================================================
// // objects declaration
// RF24 radio(NRF_CE, NRF_CSN); // nrf also uses SPI : MISO(D6), MOSI(D7), SCK(D5)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// SocketIOclient socketIO;
// //=============================================================================
// // functions declaration
// void ISR_BTN1()
// {
//     mode = !mode;
//     if (mode == 1)
//     { // debug mode and calculate
//         Serial.println("Stopped listening....");
//         radio.stopListening();
//         radio.write("1", sizeof(unsigned char));
//     }
//     else if (mode == 0)
//     { // request data
//         radio.stopListening();
//         Serial.println("Request data from node 1....");
//         if (!radio.write("0", sizeof(unsigned char)))
//         {
//             Serial.println("Failed.");
//         }
//         radio.startListening();
//     }
// }
// //-----------------------------------------------------------------------------
// void printDebug(bool isSerial, bool isOLED, SensorData data)
// {
//     if (isOLED)
//     {
//         display.clearDisplay();
//         display.setCursor(0, 0);
//         display.print(data.fTempC);
//         display.print("C ");
//         display.print(data.fHumi);
//         display.print("% ");
//         display.setCursor(0, 10);
//         display.print(data.iAQI);
//         display.print(" ");
//         display.print(data.iCO2);
//         display.print("ppm ");
//         display.setCursor(0, 20);
//         display.print(data.fWindSpeed);
//         display.print("m/s ");
//         display.print(data.fPM1);
//         display.print("ug/m3 ");
//         display.setCursor(0, 30);
//         display.print(data.fPM2_5);
//         display.print("ug/m3 ");
//         display.print(data.fPM10);
//         display.print("ug/m3 ");
//         display.setCursor(0, 50);
//         display.print("Mode: ");
//         display.print(mode ? "Stop" : "Request");
//         display.display();
//     }
//     if (isSerial)
//     {
//         // print to serial
//         Serial.print("Temp: ");
//         Serial.print(data.fTempC);
//         Serial.print("C, Humi: ");
//         Serial.print(data.fHumi);
//         Serial.print("%, AQI: ");
//         Serial.print(data.iAQI);
//         Serial.print(", CO2: ");
//         Serial.print(data.iCO2);
//         Serial.print("ppm, Wind: ");
//         Serial.print(data.fWindSpeed);
//         Serial.print("m/s, PM1: ");
//         Serial.print(data.fPM1);
//         Serial.print("ug/m3, PM2.5: ");
//         Serial.print(data.fPM2_5);
//         Serial.print("ug/m3, PM10: ");
//         Serial.print(data.fPM10);
//         Serial.println("ug/m3");
//     }
// }
// //-----------------------------------------------------------------------------
// void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
// {
//     switch (type)
//     {
//     case sIOtype_DISCONNECT:
//         Serial.printf("[IOc] Disconnected!\n");
//         break;
//     case sIOtype_CONNECT:
//         Serial.printf("[IOc] Connected to url: %s\n", payload);
//         break;
//     case sIOtype_EVENT:
//         Serial.printf("[IOc] get event: %s\n", payload);
//         break;
//     case sIOtype_ACK:
//         Serial.printf("[IOc] get ack: %u\n", length);
//         break;
//     case sIOtype_ERROR:
//         Serial.printf("[IOc] get error: %u\n", length);
//         break;
//     case sIOtype_BINARY_EVENT:
//         Serial.printf("[IOc] get binary: %u\n", length);
//         break;
//     case sIOtype_BINARY_ACK:
//         Serial.printf("[IOc] get binary ack: %u\n", length);
//         break;
//     }
// }
// //-----------------------------------------------------------------------------
// void sendDataToServer () {
//     DynamicJsonDocument doc(1024);
//     JsonArray array = doc.to<JsonArray>();
//     array.add(TOPIC);
//     JsonObject data = array.createNestedObject();
//     data["temp"] = input.fTempC;
//     data["humi"] = input.fHumi;
//     data["aqi"] = input.iAQI;
//     data["co2"] = input.iCO2;
//     data["wind"] = input.fWindSpeed;
//     data["pm1"] = input.fPM1;
//     data["pm25"] = input.fPM2_5;
//     data["pm10"] = input.fPM10;
//     String output;
//     serializeJson(doc, output);
//     socketIO.sendEVENT(output);
// 
// }
// //=============================================================================
// void setup()
// {
//     // put your setup code here, to run once:
//     Serial.begin(115200);
//     //---------------------------------------------------------------------------
//     // init nrf24
//     Serial.println("NRF24...");
//     Serial.print(radio.begin() ? "done." : "failed!");
//     radio.setPALevel(RF24_PA_MIN);
//     radio.setDataRate(RF24_250KBPS);
//     radio.setChannel(0x4c);
//     radio.openWritingPipe(address[0]);
//     radio.openReadingPipe(1, address[1]);
//     radio.startListening();
//     //---------------------------------------------------------------------------
//     // init wifi
//     Serial.println("WiFi...");
//     WiFiManager wifiManager;
//     wifiManager.setTimeout(180); // wait 3 mins for config, or else use saved
//     if (!wifiManager.autoConnect("ESP8266"))
//     {
//         Serial.println("failed to connect and hit timeout");
//         isWifiConnected = false;
//     }
//     else
//     {
//         Serial.println("Connected.");
//         isWifiConnected = true;
//     }
//     //---------------------------------------------------------------------------
//     // init button
//     pinMode(BTN1_PIN, INPUT_PULLUP);
// 
//     //---------------------------------------------------------------------------
//     // int interrupt
//     attachInterrupt(digitalPinToInterrupt(BTN1_PIN), ISR_BTN1, FALLING);
//     //---------------------------------------------------------------------------
//     // init display
//     if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
//     {
//         Serial.println(F("SSD1306 allocation failed"));
//         isOLEDConnected = false;
//     }
//     // Clear the buffer.
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(WHITE);
//     //---------------------------------------------------------------------------
//     // server address, port and URL
//     socketIO.begin(SERVER, PORT, "/socket.io/?EIO=4"); // /socket.io/?EIO=4
//     socketIO.onEvent(socketIOEvent);
// }
// //=============================================================================
// void loop()
// {
//     // show current status
//     Serial.print("Mode: ");
//     Serial.println(mode ? "Stop" : "Request");
//     display.setCursor(0, 50);
//     display.print("Mode: ");
//     display.println(mode ? "Stop" : "Request");
//     if (mode == 0) // standard mode
//     {
//         unsigned long ulWaited = millis(); // timeout
//         Serial.println("Waiting for data...");
//         while (!radio.available())
//         {
//             if (millis() - ulWaited > 1000) // 1s timeout
//             {
//                 Serial.println("Timeout.");
//                 return;
//             }
//         }
//         Serial.println("Data received.");
//         radio.read(&input, sizeof(SensorData));
//         printDebug(true, false, input);
//         if (WiFi.status() == WL_CONNECTED)
//         {
//             sendDataToServer();
//         }
//     }
// }
