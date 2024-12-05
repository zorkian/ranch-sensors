#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <RH_RF95.h>

// Battery measure
#define VBATPIN A7

// Defined for adafruit feather M0
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.1
#define RF95_POWER 23

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Ranch struct flags (0..31 flag positions max)
#define FLAG_CHARGER_PGOOD 1
#define FLAG_CHARGER_CHARGE 2

// data format, what's in the values depends on what type of sensor
// is sending it along
typedef struct
{
  uint8_t sensorType;
  uint16_t sensorId;
  ulong flags;
  ulong uptimeMillis;
  ulong txCounter;
  uint8_t txPower;
  uint16_t battMillivolts;
  uint16_t lastRssi;
  uint16_t valOne;
  uint16_t valTwo;
  uint16_t valThree;
  uint16_t valFour;
  uint16_t valFive;
  uint16_t valSix;
  uint16_t valSeven;
  uint16_t valEight;
  uint16_t valNine;
  uint16_t valTen;
} RanchSensorStruct;

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial)
    delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(RF95_POWER, false);
}

int16_t packetnum = 0; // packet counter, we increment per xmission

void loop()
{
  if (rf95.available())
  {
    RanchSensorStruct packet;
    uint8_t datalen = sizeof(packet);

    if (rf95.recv((uint8_t *)&packet, &datalen))
    {
      // Debugging output
      /*
      Serial.printf("Received (RSSI %d): type=%d/%d, f=%d, up=%d, batt=%d, tx(%d)=%d\n",
                    rf95.lastRssi(), packet.sensorType, packet.sensorId, packet.flags,
                    packet.uptimeMillis, packet.battMillivolts, packet.txPower, packet.txCounter);
      if (packet.sensorType == 10)
      {
        Serial.printf("-> Water Level: distance=%d cms\n", packet.valOne);
      }
      else
      {
        Serial.printf("-> Unknown type: %d", packet.sensorType);
      }
      */

      StaticJsonDocument<100> json;

      json["t"] = packet.sensorType;
      json["id"] = packet.sensorId;
      json["rssi"] = rf95.lastRssi();
      json["f"] = packet.flags;
      json["b"] = packet.battMillivolts;
      json["up"] = packet.uptimeMillis;
      json["txc"] = packet.txCounter;
      json["txp"] = packet.txPower;

      if (packet.sensorType == 10)
      {
        json["wl"] = packet.valOne;
      }

      serializeJson(json, Serial);
      Serial.println();
    }
    else
    {
      Serial.println("Receive failed!");
    }
  }
}