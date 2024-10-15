#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"
#include <SD_ZH03B.h>
#include <SPI.h>
#include "RF24.h"
// #include <ModbusMaster.h>
//=============================================================================
#define DEBUG
#define TIME_INTERVAL 2000
#define ZH03B_RX 5
#define ZH03B_TX 4
#define CE_PIN 9
#define CSN_PIN 10
#define RS485_RX 2
#define RS485_TX 3
//=============================================================================
// objects declaration
// ScioSense_ENS160      ens160(ENS160_I2CADDR_0); //0x52
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // 0x53..ENS160+AHT21
Adafruit_AHTX0 aht;
SoftwareSerial ZHSerial(ZH03B_RX, ZH03B_TX);    // RX, TX
// SoftwareSerial RS485Serial(RS485_RX, RS485_TX); // RX, TX
SD_ZH03B ZH03B(ZHSerial, SD_ZH03B::SENSOR_ZH03B);
RF24 radio(CE_PIN, CSN_PIN); // nrf also uses SPI : SS(10), MISO(12), MOSI(11), SCK(13)
// ModbusMaster node_wind;
//=============================================================================
// vairables declaration
uint8_t address[][6] = {"1Node", "2Node"};
unsigned long ulTime;
int mode = 0;

struct SensorData
{
  float fTempC;
  float fHumi;
  int iAQI;
  int iCO2;
  float fWindSpeed;
  float fPM1;
  float fPM2_5;
  float fPM10;
};

SensorData output;

//=============================================================================
// functions declaration
void readAHT()
{
  sensors_event_t humi, temp;
  aht.getEvent(&humi, &temp);
  output.fTempC = temp.temperature;
  output.fHumi = humi.relative_humidity;
#ifdef DEBUG
  Serial.print("temp: ");
  Serial.print(output.fTempC);
  Serial.print("C,\t humi: ");
  Serial.println(output.fHumi);
#endif
}
//-----------------------------------------------------------------------------
void readENS160()
{
  if (ens160.available())
  {
    output.iAQI = ens160.getAQI();
    output.iCO2 = ens160.geteCO2();
#ifdef DEBUG
    Serial.print("AQI: ");
    Serial.print(output.iAQI);
    Serial.print(",\t CO2: ");
    Serial.print(output.iCO2);
    Serial.println("ppm");
#endif
  }
  else
  {
    Serial.println("ENS160 not available!");
  }
}
//-----------------------------------------------------------------------------
void readZH03B()
{
  // ZH03B.wakeup();
  // ZH03B.setMode(SD_ZH03B::QA_MODE);
  if (ZH03B.readData())
  {
    output.fPM1 = ZH03B.getPM1_0();
    output.fPM2_5 = ZH03B.getPM2_5();
    output.fPM10 = ZH03B.getPM10_0();
    // ZH03B.sleep();
#ifdef DEBUG
    char printbuf1[80];
    char temp1[7];
    dtostrf(output.fPM1, 5, 2, temp1);
    char temp2[7];
    dtostrf(output.fPM2_5, 5, 2, temp2);
    char temp3[7];
    dtostrf(output.fPM10, 5, 2, temp3);
    sprintf(printbuf1, "PM1.0, PM2.5, PM10=[%s %s %s]", temp1, temp2, temp3);
    Serial.println(printbuf1);
#endif
  }
  else
  {
    Serial.println("ZH03B not available!");
  }
}
//-----------------------------------------------------------------------------
void sendDataToNRF()
{
  radio.stopListening();
  if (!radio.write(&output, sizeof(output)))
  {
    Serial.println("No ack...transmit failed!");
  }
  radio.startListening();
}
//-----------------------------------------------------------------------------
void toggleLed(byte ledPin, byte time, byte count)
{
  for (int i = 0; i < count; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
}
//-----------------------------------------------------------------------------
// void readWindSpeed()
// {
//   uint8_t result;
//   result = node_wind.readInputRegisters(0x0000, 2);
//   if (result == node_wind.ku8MBSuccess)
//   {
//     uint16_t u16data[6];
//     u16data[0] = node_wind.receive() / 10.0;
//     output.fWindSpeed = (float)u16data[0];
//     Serial.print("Wind speed: ");
//     Serial.print(output.fWindSpeed);
//     Serial.println("m/s");
//   }
//   else
//   {
//     Serial.println("Wind sensor not available!");
//   }
// }
//=============================================================================
void setup()
{
  Serial.begin(9600);
  //---------------------------------------------------------------------------
  // init led indicator
  // pinMode(LED_BUILTIN, OUTPUT);
  // for (int i = 0; i < 3; i++)
  // {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(100);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(100);
  // }
  //---------------------------------------------------------------------------
  // init ens160
  Serial.print("ENS160...");
  ens160.begin();
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available())
  {
    // Print ENS160 versions
    Serial.print("\tRev: ");
    Serial.print(ens160.getMajorRev());
    Serial.print(".");
    Serial.print(ens160.getMinorRev());
    Serial.print(".");
    Serial.println(ens160.getBuild());

    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
  }
  //---------------------------------------------------------------------------
  // init aht20
  Serial.print("AHT20...");
  Serial.println(aht.begin() ? "done." : "failed!");
  //---------------------------------------------------------------------------
  // init zh03b
  Serial.println("ZH03B...done.");
  ZHSerial.begin(9600);
  delay(100);
  ZH03B.setMode(SD_ZH03B::QA_MODE);
  Serial.println("-- Reading ZH03B --");
  delay(200);
  //---------------------------------------------------------------------------
  // init nrf24
  Serial.print("NRF24...");
  Serial.println(radio.begin() ? "done." : "failed!");
  // radio.setPALevel(RF24_PA_MIN);
  // radio.setDataRate(RF24_250KBPS);
  // radio.setChannel(0x4c);
  radio.openWritingPipe(address[1]);
  // radio.openReadingPipe(1, address[0]);
  radio.stopListening();
  //---------------------------------------------------------------------------
  // init modbus wind sensor
  // RS485Serial.begin(4800);
  // node_wind.begin(1, RS485Serial);
}
//=============================================================================
void loop()
{
  if (mode == 0) // standard mode
  {
    if (millis() - ulTime > TIME_INTERVAL)
    {
      ulTime = millis();
      readAHT();
      readENS160();
      readZH03B();
      // readWindSpeed();
      sendDataToNRF();
    }
  }
  if (mode == 1) // stop mode
  {
    // toggleLed(LED_BUILTIN, 100, 3);
  }

  // if (radio.available())
  // {
  //   unsigned char dataRx;
  //   radio.read(&dataRx, sizeof(unsigned char));
  //   Serial.print("Received: ");
  //   Serial.println(dataRx);
  //   switch (dataRx)
  //   {
  //   case '0':
  //     // toggleLed(LED_BUILTIN, 100, 3);
  //     mode = 0;
  //     break;
  //   case '1':
  //     // toggleLed(LED_BUILTIN, 100, 3);
  //     mode = 1;
  //     break;
  //   default:
  //     break;
  //   }
  // }
}
