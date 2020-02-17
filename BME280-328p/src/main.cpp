#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <time.h>

#include <digitalWriteFast.h>
#include "LowPower.h"

#define TINYBME
//#define VWRF

#ifdef VWRF
  #include <VirtualWire.h>
#else
  #include <RH_ASK.h>
  #include <SPI.h> // Not actually used but needed to compile
#endif

#ifdef TINYBME
  #define TINY_BME280_I2C
  #include "TinyBME280.h"
  #define SENSE_VALUE 30
#else
  #include <Adafruit_Sensor.h>
  #include "Adafruit_BME280.h"
  #define SENSE_VALUE 1
#endif

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
#define LED_PIN         13
#define SPEAKER_PIN     7
#define TRANSMISION_PIN 4

RH_ASK rfsender;

void readValues();
void checkTimeout();

void wykonaj_transmisje()
{
  uint8_t temperature = 10;

  #ifdef VWRF
    digitalWrite(LED_PIN, HIGH); // Flash a light to show transmitting
    vw_send((uint8_t *)&temperature, sizeof(temperature));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(LED_PIN, LOW);
    delay(500);
  #else
    power_timer1_enable();
    digitalWriteFast(LED_PIN, HIGH); // Flash a light to show transmitting
    
    rfsender.send((uint8_t *)temperature, sizeof(temperature));
    rfsender.waitPacketSent();

    digitalWriteFast(LED_PIN, LOW);
    power_timer1_disable();
    delay(20);
  #endif
}

void setup_rf()
{
  #ifdef VWRF
    // Initialise the IO and ISR
    vw_set_tx_pin(TRANSMISION_PIN);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);       // Bits per sec
  #else
    if (!rfsender.init()){} //
  #endif
}


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
  digitalWriteFast(LED_PIN, LOW);  // LED OFF
  digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  digitalWriteFast(TRANSMISION_PIN, LOW);    // RF433

  readValues();               // pierwsze pobranie wartosci - populacja zmiennych
  setup_rf();

  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// Timer 0
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2
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

    digitalWriteFast(SPEAKER_PIN, HIGH);   // SPK
    delayMicroseconds(100);
    digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  }
}
void checkTimeout()
{
  if(current_positive - last_positive > TIME_TO_WAIT_MS) 
  {
    // odswiez last positive
    last_positive = current_positive;
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



/**************************
 *  LOOP
 * ************************/
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
      digitalWriteFast(LED_PIN, HIGH);  // LED OFF
      delayMicroseconds(100);
      digitalWriteFast(LED_PIN, LOW);  // LED OFF
    if(current_positive - last_positive > TIME_TO_WAIT_MS) 
    {
      // save the last time you blinked the LED
      last_positive = current_positive;

      //digitalWriteFast(SPEAKER_PIN, HIGH);   // SPK
      digitalWriteFast(LED_PIN, HIGH);  // LED OFF
      delayMicroseconds(100);
      //digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
      digitalWriteFast(LED_PIN, LOW);  // LED OFF

      was_whistled = false;
    }
  }


  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
}