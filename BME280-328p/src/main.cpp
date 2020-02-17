#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <time.h>

#include <digitalWriteFast.h>
#include "LowPower.h"

//#define VWRF

#ifdef VWRF
  #include <VirtualWire.h>
#else
  #include <RH_ASK.h>
  #include <SPI.h> // Not actually used but needed to compile
#endif

#define TINY_BME280_I2C
#include "TinyBME280.h"
#define SENSE_VALUE 30


tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

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

RH_ASK rfsender(2000,12,11,3,true);

void readValues();
void checkTimeout();


void wykonaj_transmisje()
{
  const char *msg = "JEB";
 
  #ifdef VWRF
    digitalWrite(LED_PIN, HIGH); // Flash a light to show transmitting
    vw_send((uint8_t *)&temperature, sizeof(temperature));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(LED_PIN, LOW);
    delay(500);
  #else
    power_timer1_enable();
    //digitalWriteFast(LED_PIN, HIGH); // Flash a light to show transmitting
    
    rfsender.send((uint8_t *)msg, sizeof(msg));
    rfsender.waitPacketSent();

    //digitalWriteFast(LED_PIN, LOW);
    power_timer1_disable();
    delay(2);
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

/*****************************************************
 * SETUP
 * ***************************************************/
void setup() {
  //clock_prescale_set(clock_div_2);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  //power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2
  
  bme1.setI2CAddress(0x76);
  bme1.beginI2C();

  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);    // changed as per below
    digitalWriteFast(i, LOW);  //     ditto
  }
  digitalWriteFast(LED_PIN, LOW);  // LED OFF
  digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  digitalWriteFast(TRANSMISION_PIN, LOW);    // RF433

  readValues();               // pierwsze pobranie wartosci - populacja zmiennych
  setup_rf();
}

/*****************************************************
 * Pobiera dane z czujnika BME280
 * ***************************************************/
void readValues() {
  prev_press = press;

  power_twi_enable();
  press = bme1.readFixedPressure();
  power_twi_disable();
}

/*****************************************************
 * Sprawdza czy cisnienie jest wieksze od zalozonego
 * ***************************************************/
void checkPressure(){
  if(press > prev_press + SENSE_VALUE )   // jeśli nowy odczyt jest wiekszy o SENSE_VALUE od poprzedniego ->
  {
    was_whistled = true;
    wykonaj_transmisje();
  }
}

/*****************************************************
 * TEST - TIMEOUT
 * ***************************************************/
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
    //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  } 
  else if(current_positive - last_positive > TIMEOUT_2 )
  {
    // przejsc w deep sleep
  }
}


/**************************
 *  LOOP
 * ************************/
void loop() {

  if(was_whistled == false)  // jeżeli poprzednio nie było dmuchnięcia
  {
    power_timer1_enable();
    power_twi_enable();
      bme1.beginI2C();
      readValues();
      checkPressure();
    power_twi_disable();
    power_timer1_disable();

    digitalWriteFast(I2C_SCL_PIN, LOW);
    digitalWriteFast(I2C_SDA_PIN, LOW);
    ADCSRA &= ~(1 << 7);
  }
  else                      // jeżeli poprzednio było dmuchnięcie
  {                         // TIME_TO_WAIT_MS okresla czas przerwy
    power_twi_disable();
    
    current_positive = millis();  // pobierz czas.

    if(current_positive - last_positive > TIME_TO_WAIT_MS) 
    {
      // save the last time you blinked the LED
      last_positive = current_positive;
      was_whistled = false;
    }
    power_timer1_disable();
  }

  checkTimeout();

  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
}
