#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <avr/dtostrf.h>

// Battery measure
#define VBATPIN A7

// Defined for adafruit feather M0
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Pins for ultrasonic sensor
const int trigPin = 5;
const int echoPin = 6;

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

  // for the ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
}

int16_t packetnum = 0; // packet counter, we increment per xmission

void loop()
{
  // only send every X timeframe, we should probably make this depend somewhat
  // on the power level as well as if the water is moving?
  delay(1000);

  // take a measurement
  float duration;
  int16_t distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // distance in... centimeters
  duration = pulseIn(echoPin, HIGH);
  distance = int16_t((duration * .0343) / 2);

  // get battery value
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  // now send it
  char radiopacket[25] = "WL                      ";
  itoa(packetnum++, radiopacket + 3, 10);
  for (int i = 0; i < 25; i++)
  {
    if (radiopacket[i] == 0)
    {
      radiopacket[i] = 0x20;
    }
  }
  itoa(distance, radiopacket + 9, 10);
  for (int i = 0; i < 25; i++)
  {
    if (radiopacket[i] == 0)
    {
      radiopacket[i] = 0x20;
    }
  }
  dtostrf(measuredvbat, 4, 2, radiopacket + 15);
  // sprintf(radiopacket, "WL %05d %05d %4.2f ss", packetnum++, distance, measuredvbat);
  Serial.print("Sending: ");
  Serial.println(radiopacket);
  radiopacket[24] = 0;

  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  delay(10);
  rf95.waitPacketSent();
}