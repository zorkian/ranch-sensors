#include <Arduino.h>
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

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// data format, what's in the values depends on what type of sensor
// is sending it along
typedef struct
{
  uint8_t sensorType;
  uint16_t sensorId;
  ulong uptimeMillis;
  ulong txCounter;
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
  rf95.setTxPower(23, false);
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
      Serial.printf("Received (RSSI %d): type=%d, id=%d, uptime=%d, batt=%d, tx=%d\n",
                    rf95.lastRssi(), packet.sensorType, packet.sensorId, packet.uptimeMillis, packet.battMillivolts, packet.txCounter);
      if (packet.sensorType == 10)
      {
        Serial.printf("-> Water Level: distance=%d cms\n", packet.valOne);
      }
      else
      {
        Serial.printf("-> Unknown type: %d", packet.sensorType);
      }
    }
    else
    {
      Serial.println("Receive failed!");
    }
  }
}