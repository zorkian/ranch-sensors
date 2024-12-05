#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <avr/dtostrf.h>

// CHANGE THIS EVERY TIME YOU COMPILE FOR A SENSOR; IS THERE A NICER WAY?
#define SENSOR_ID 201

// How quickly to send reports; this might be fine
#define SEND_INTERVAL_MILLIS 5000
#define SEND_INTERVAL_LOWPOWER_MILLIS 300000
#define LOW_POWER_THRESHOLD_MILLIVOLTS 3600

// Battery measure
#define VBAT_PIN A7

// Defined for adafruit feather M0
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.1
#define RF95_POWER 23

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Pins for ultrasonic sensor
#define ULTRA_TRIG_PIN 5
#define ULTRA_ECHO_PIN 6

// Pins for checking on the charging system
#define CHARGER_PGOOD_PIN 10
#define CHARGER_CHARGE_PIN 11

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

ulong packetnum = 0; // packet counter, we increment per tx

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial)
    delay(1);
  delay(100);

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
  // TODO: have the receiver send back a power report so we can adjust how much power
  // we use to transmit
  rf95.setTxPower(RF95_POWER, false);

  // for the ultrasonic sensor
  pinMode(ULTRA_TRIG_PIN, OUTPUT); // Sets the ULTRA_TRIG_PIN as an Output
  pinMode(ULTRA_ECHO_PIN, INPUT);  // Sets the ULTRA_ECHO_PIN as an Input

  // turn off the LED; we flash it during xmit
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // configure input pins for charger
  pinMode(CHARGER_CHARGE_PIN, INPUT_PULLUP);
  pinMode(CHARGER_PGOOD_PIN, INPUT_PULLUP);
}

uint16_t getDistanceCentimeters()
{
  // take a measurement
  float duration;

  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  // distance in... centimeters
  duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
  return uint16_t((duration * .0343) / 2);
}

uint16_t getBattMillivolts()
{
  // get battery value
  float measuredvbat = analogRead(VBAT_PIN);
  measuredvbat *= 2;   // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  // TODO: should this really be 1024?
  measuredvbat /= 1024; // convert to voltage

  // convert up into millivolts and integerize
  return uint16_t(measuredvbat * 1000);
}

void loop()
{
  // record what time it is so we can delay later
  ulong startMillis = millis();
  digitalWrite(LED_BUILTIN, HIGH);

  // get structure
  RanchSensorStruct packet;
  packet.sensorType = 10;
  packet.sensorId = SENSOR_ID;
  packet.flags = 0;
  packet.uptimeMillis = startMillis;
  packet.battMillivolts = getBattMillivolts();
  // packet.lastRssi = ...
  packet.txCounter = packetnum++;
  packet.txPower = RF95_POWER;
  packet.valOne = getDistanceCentimeters();

  // Set flags based on charging system, this assumes the flags were 0 when we started
  packet.flags ^= (digitalRead(CHARGER_PGOOD_PIN) == LOW ? 1 : 0) << FLAG_CHARGER_PGOOD;
  packet.flags ^= (digitalRead(CHARGER_CHARGE_PIN) == LOW ? 1 : 0) << FLAG_CHARGER_CHARGE;

  // Debugging output
  Serial.printf("Sending: t=%d/id=%d/f=%d/up=%d/batt=%d/tx(%d)=%d/dist=%d\n",
                packet.sensorType, packet.sensorId, packet.flags, packet.uptimeMillis, packet.battMillivolts,
                packet.txPower, packet.txCounter, packet.valOne);

  // Write the packet and chill until it's sent
  delay(10);
  rf95.send((uint8_t *)&packet, sizeof(packet));

  delay(10);
  rf95.waitPacketSent();

  // LED off while we wait
  digitalWrite(LED_BUILTIN, LOW);

  // now relax a while, just so we don't blast the power transmitting way more than
  // we need to care about (it's not like the water level is going to move that
  // fast... and if it does, we're probably not there to deal with it anyway)
  ulong nowMillis = millis();
  long shouldDelay = (packet.battMillivolts > LOW_POWER_THRESHOLD_MILLIVOLTS ? SEND_INTERVAL_MILLIS : SEND_INTERVAL_LOWPOWER_MILLIS) - (nowMillis - startMillis);
  if (shouldDelay > 0)
  {
    // Serial.printf("Delay: %d = %d - (%d - %d)\n", shouldDelay, SEND_INTERVAL_MILLIS, nowMillis, startMillis);
    delay(shouldDelay);
  }
  else
  {
    Serial.println("Took too long to run a loop!");
  }
}