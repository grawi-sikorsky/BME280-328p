#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <time.h>

#include <digitalWriteFast.h>
#include "LowPower.h"

#define TINYBME

#ifdef TINYBME
  #define TINY_BME280_I2C
  #include "TinyBME280.h"
  #define SENSE_VALUE 30
#else
  #include <Adafruit_Sensor.h>
  #include "Adafruit_BME280.h"
  #define SENSE_VALUE 1
#endif

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

#ifdef TINYBME
  tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)
#else
  Adafruit_BME280 bme; // I2C
#endif

float press, prev_press;
time_t current_positive, last_positive, // czas ostatniego dmuchniecia
          a;
bool was_whistled;                      // flaga dmuchniete czy nie
#define TIME_TO_WAIT_MS 100             // czas do nastepnego wyzwolenia
#define TIMEOUT_1       2000            // pierwszy timeiut
#define TIMEOUT_2       6000
void readValues();
void checkTimeout();
void wykonaj_transmisje(){};

void setup() {
  //Serial.begin(9600);
  //clock_prescale_set(clock_div_2);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER

  #ifdef TINYBME
    bme1.setI2CAddress(0x76);
    bme1.beginI2C();
  #endif

  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);    // changed as per below
    digitalWriteFast(i, HIGH);  //     ditto
  }
  digitalWriteFast(13, LOW);  // LED OFF
  digitalWriteFast(7,LOW);    // SPK

  readValues();               // pierwsze pobranie wartosci - populacja zmiennych

  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// Timer 0
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2
  //power_twi_disable(); // TWI (I2C)
}


void readValues() {
  prev_press = press;

  #ifdef TINYBME
    press = bme1.readFixedPressure();
  #else
    press = bme.readPressure();
  #endif
}

void checkPressure()
{
  if(press > prev_press + SENSE_VALUE )   // jeśli nowy odczyt jest wiekszy o SENSE_VALUE od poprzedniego ->
  {
    was_whistled = true;

    wykonaj_transmisje();

    digitalWriteFast(7,HIGH);   // SPK
    digitalWriteFast(13, HIGH);  // LED OFF
    delay(1);
    digitalWriteFast(7,LOW);    // SPK
    digitalWriteFast(13, LOW);  // LED OFF
  }    
}
void checkTimeout()
{
  if(current_positive - last_positive > TIME_TO_WAIT_MS) 
  {
    // save the last time you blinked the LED
    last_positive = current_positive;

    digitalWriteFast(7,HIGH);   // SPK
    digitalWriteFast(13, HIGH);  // LED OFF
    delay(100);
    digitalWriteFast(7,LOW);    // SPK
    digitalWriteFast(13, LOW);  // LED OFF
    delay(100);
    digitalWriteFast(7,HIGH);   // SPK
    digitalWriteFast(13, HIGH);  // LED OFF
    delay(100);
    digitalWriteFast(7,LOW);    // SPK
    digitalWriteFast(13, LOW);  // LED OFF

    was_whistled = false;
  }
  else if(current_positive - last_positive > TIMEOUT_1 )
  {
    // zmniejsz probkowanie 1x/s
  }
  else if(current_positive - last_positive > TIMEOUT_2 )
  {
    // przejsc w deep sleep
  }
}
// Brown-out disable // ->  avrdude -c usbtiny -p m328p -U efuse:w:0x07:m

void loop() {
  //Serial.begin(9600);

  #ifdef TINYBME
    bme1.beginI2C();
    //bme1.reset();
  #else
    bme.begin(0x76);
  #endif

  if(was_whistled == false)       // jeżeli poprzednio nie było dmuchnięcia
  {
    readValues();
    checkPressure();
  }
  else                            // jeżeli poprzednio było dmuchnięcie
  {
    current_positive = millis();  // pobierz czas.
      digitalWriteFast(7,HIGH);   // SPK
      digitalWriteFast(13, HIGH);  // LED OFF
      delay(1);
      digitalWriteFast(7,LOW);    // SPK
      digitalWriteFast(13, LOW);  // LED OFF
    if(current_positive - last_positive > TIME_TO_WAIT_MS) 
    {
      // save the last time you blinked the LED
      last_positive = current_positive;

      digitalWriteFast(7,HIGH);   // SPK
      digitalWriteFast(13, HIGH);  // LED OFF
      delay(100);
      digitalWriteFast(7,LOW);    // SPK
      digitalWriteFast(13, LOW);  // LED OFF
      delay(100);
      digitalWriteFast(7,HIGH);   // SPK
      digitalWriteFast(13, HIGH);  // LED OFF
      delay(100);
      digitalWriteFast(7,LOW);    // SPK
      digitalWriteFast(13, LOW);  // LED OFF

      was_whistled = false;
    }
  }


  //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
}