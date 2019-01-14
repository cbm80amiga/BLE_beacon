#include <SPI.h>
#include <RF24.h>
#include "BLE.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

// Code for the video: https://youtu.be/2ZlZVE3XjOw
// (C)2019 Pawel A. Hernik

/* PINOUT
  DHT11 pinout from left:
  VCC DATA NC GND

  nRF24L01 from pin side/top:
  -------------
  |1 3 5 7    |
  |2 4 6 8    |
  |           |
  |           |
  |           |
  |           |
  |           |
  -------------
  1 - GND  blk   GND
  2 - VCC  wht   3V3
  3 - CE   orng  9
  4 - CSN  yell  10
  5 - SCK  grn   13
  6 - MOSI blue  11
  7 - MISO viol  12
  8 - IRQ  gray  2

 More info about nRF24L01:
 http://arduinoinfo.mywikis.net/wiki/Nrf24L01-2.4GHz-HowTo#po1
*/

// def - DHT11 => Tm1
// ndef - internal ATMEGA temperature => Tm2
//#define BEACON_DH11

// def - lowpower mode
// ndef - regular delay
#define LOWPOWER

#define DHT11_PIN 8

RF24 radio(9,10);
BTLE btle(&radio);

// -------------------------

long readVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(4); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // calc AVcc in mV
  return result;
}

// -------------------------

float readIntTemp() 
{
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(4); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = (result - 125) * 1075;
  return result/10000.0;
}

// -------------------------

#define DHT_OK         0
#define DHT_CHECKSUM  -1
#define DHT_TIMEOUT   -2
int temp1,temp10;
int humidity;
float temperature;

int readDHT11(int pin)
{
  uint8_t bits[5];
  uint8_t bit = 7;
  uint8_t idx = 0;

  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT_PULLUP);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = 10000;
  while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

  loopCnt = 10000;
  while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loopCnt = 10000;
    while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

    unsigned long t = micros();
    loopCnt = 10000;
    while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

    if(micros() - t > 40) bits[idx] |= (1 << bit);
    if(bit == 0) {
      bit = 7;    // restart at MSB
      idx++;      // next byte!
    }
    else bit--;
  }

  humidity = bits[0];
  temp1    = bits[2];
  temp10   = bits[3];
  temperature = abs(temp1+temp10/10.0);

  if(bits[4] != bits[0]+bits[1]+bits[2]+bits[3]) return DHT_CHECKSUM;
  return DHT_OK;
}

// -------------------------
#ifdef LOWPOWER

enum wdt_time {
  SLEEP_15MS,
  SLEEP_30MS, 
  SLEEP_60MS,
  SLEEP_120MS,
  SLEEP_250MS,
  SLEEP_500MS,
  SLEEP_1S,
  SLEEP_2S,
  SLEEP_4S,
  SLEEP_8S,
  SLEEP_FOREVER
};

ISR(WDT_vect) { wdt_disable(); }

void powerDown(uint8_t time)
{
  ADCSRA &= ~(1 << ADEN);  // turn off ADC
  if(time != SLEEP_FOREVER) { // use watchdog timer
    wdt_enable(time);
    WDTCSR |= (1 << WDIE);  
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // most power saving
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  // ... sleeping here
  sleep_disable();
  ADCSRA |= (1 << ADEN); // turn on ADC
}
#endif
// -------------------------

void setup()
{
  Serial.begin(9600);
  Serial.println(F("BLE begin"));
#ifdef BEACON_DH11
  btle.begin("Tm1");
  btle.setMAC(0xDE,0xAD,0xBE,0xEF,0x00,0x01);
#else
  btle.begin("Tm2");
  btle.setMAC(0xDE,0xAD,0xBE,0xEF,0x00,0x02);
#endif
  Serial.println(F("BLE ok"));
}

// -------------------------
int cnt=0;
void loop() 
{
  long v = readVcc();
  battery_level_data battery_data;
  battery_data.service_uuid = NRF_BATTERY_SERVICE_UUID;
  //battery_data.battery_percentage = constrain(map(v,2900,4200,0,100),0,100);
  //Serial.print("Batt: "); Serial.println(v/1000.0);
  battery_data.battery_percentage = ((cnt++)/4)%100;
  Serial.print(F("Batt: ")); Serial.println(battery_data.battery_percentage);

#ifdef BEACON_DH11
  int ret = readDHT11(DHT11_PIN);
  //if(ret==DHT_TIMEOUT) Serial.println("DHT11 timeout error!"); else
  if(ret==DHT_CHECKSUM) Serial.println("DHT11 checksum error!");
#else
  temperature = readIntTemp();
#endif
  Serial.print(F("Temp: ")); Serial.println(temperature);
  nrf_service_data temp_data;
  temp_data.service_uuid = NRF_TEMPERATURE_SERVICE_UUID;
  temp_data.value = BTLE::to_nRF_Float(temperature);

  radio.powerUp();
  btle.preparePacket();
  if(!btle.addChunk(0x16, sizeof(battery_data), &battery_data)) {
    Serial.println(F("No room in packet for battery level"));
  }
  if(!btle.addChunk(0x16, sizeof(temp_data), &temp_data)) {
    Serial.println(F("No room in packet for temperature"));
  }
  btle.transmitPacket();
  btle.hopChannel();
  radio.powerDown();
#ifdef LOWPOWER
  Serial.flush();
  //powerDown(SLEEP_120MS);
  powerDown(SLEEP_250MS);
#else
  delay(250);
#endif
}

