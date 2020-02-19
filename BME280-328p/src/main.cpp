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

//#define TINY_BME280_I2C
#define TINY_BME280_SPI
#include "TinyBME280.h"
//#include "Wire.h"
#define SENSE_VALUE 30


tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

float press, prev_press;
time_t current_positive, last_positive, // czas ostatniego dmuchniecia
          current_timeout;
bool was_whistled;                      // flaga dmuchniete czy nie
period_t sleeptime = SLEEP_250MS;       // czas snu procesora
                                        // 4 - 250ms / 6 - 1s / 8 - 4s..
#define TIME_TO_WAIT_MS 100             // czas do nastepnego wyzwolenia
#define TIMEOUT_1       2000            // pierwszy timeiut
#define TIMEOUT_2       3000
#define LED_PIN         13
#define SPEAKER_PIN     7 //A2 // 7 minipro
#define TRANSMISION_PIN 4

RH_ASK rfsender(2000,5,TRANSMISION_PIN,3,true);

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
  //clock_prescale_set(clock_div_4);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  power_adc_disable(); // ADC converter
  //power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  //power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2
  
  //bme1.setI2CAddress(0x76);
  //bme1.beginI2C();

  bme1.begin();

  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);    // changed as per below
    digitalWriteFast(i, LOW);  //     ditto
  }
  //pinModeFast(0,OUTPUT);
  //pinModeFast(1,OUTPUT);
  //digitalWriteFast(0, HIGH);
  //digitalWriteFast(1, HIGH);

  pinModeFast(LED_PIN,OUTPUT);
  pinModeFast(SPEAKER_PIN,OUTPUT);
  pinModeFast(TRANSMISION_PIN,OUTPUT);
  digitalWriteFast(LED_PIN, LOW);  // LED OFF
  digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  digitalWriteFast(TRANSMISION_PIN, LOW);    // RF433

  pinModeFast(SS,OUTPUT);
  pinModeFast(MOSI,OUTPUT);
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);
  digitalWriteFast(SS,HIGH);
  digitalWriteFast(MOSI,HIGH);
  digitalWriteFast(MISO,HIGH);
  digitalWriteFast(SCK,HIGH);

  readValues();               // pierwsze pobranie wartosci - populacja zmiennych
  setup_rf();
}

/*****************************************************
 * Pobiera dane z czujnika BME280
 * ***************************************************/
void readValues() {
  prev_press = press;

  //power_spi_enable();
  //bme1.begin();
  bme1.setMode(11);
  press = bme1.readFixedPressure();
  bme1.setMode(00);
  //power_spi_disable();
}
// Brown-out disable // ->  avrdude -c usbasp -p m328p -U efuse:w:0x07:m

/*****************************************************
 * Sprawdza czy cisnienie jest wieksze od zalozonego
 * ***************************************************/
void checkPressure(){
  if(press > prev_press + SENSE_VALUE )   // jeśli nowy odczyt jest wiekszy o SENSE_VALUE od poprzedniego ->
  {
    was_whistled = true;
    wykonaj_transmisje();
    //digitalWriteFast(SPEAKER_PIN,HIGH);
    //delay(20);
    //digitalWriteFast(SPEAKER_PIN,LOW);
  }
}

/*****************************************************
 * TEST - TIMEOUT
 * ***************************************************/
void checkTimeout()
{
  current_positive = millis();  // pobierz czas.
  current_timeout = current_positive - last_positive;

  if(current_timeout > TIME_TO_WAIT_MS && current_timeout < TIMEOUT_1) // pierwszy prog
  { // jezeli 
    //last_positive = current_positive;
    sleeptime = SLEEP_250MS;

    //digitalWriteFast(SPEAKER_PIN,HIGH);
    //delay(20);
    //digitalWriteFast(SPEAKER_PIN,LOW);
  }
  else if(current_timeout > TIMEOUT_1 && current_timeout < TIMEOUT_2) // drugi prog
  {
    // zmniejsz probkowanie 1x/s
    sleeptime = SLEEP_1S; // 1S

    //digitalWriteFast(SPEAKER_PIN,HIGH);
    //delay(20);
    //digitalWriteFast(SPEAKER_PIN,LOW);
  } 
  else if(current_timeout > TIMEOUT_2 )
  { 
    //digitalWriteFast(SPEAKER_PIN,HIGH);
    //delay(20);
    //digitalWriteFast(SPEAKER_PIN,LOW);

    sleeptime = SLEEP_4S; // 4S
    //sleeptime = SLEEP_FOREVER;
    // setup ISR to WAKE UP!
    power_twi_disable();
    power_usart0_disable();
/*
    //wdt_disable();

    for (int i = 0; i < A5; i++) {
    if(i != 2)//just because the button is hooked up to digital pin 2
    pinMode(i, OUTPUT);
    }
    //attachInterrupt(0, digitalInterrupt, FALLING); //interrupt for waking up
      //SETUP WATCHDOG TIMER
    WDTCSR = (24);//change enable and WDE - also resets
    WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
    WDTCSR |= (1<<6);//enable interrupt mode

    //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
    ADCSRA &= ~(1 << 7);
    
    //ENABLE SLEEP - this enables the sleep mode
    SMCR |= (1 << 2); //power down mode
    SMCR |= 1;//enable sleep
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time

*/
  }  
}

/**************************
 *  LOOP
 * ************************/
void loop() {

  if(was_whistled == false)  // jeżeli poprzednio nie było dmuchnięcia
  {
    pinModeFast(SS,OUTPUT);
    pinModeFast(MOSI,OUTPUT);
    pinModeFast(MISO,OUTPUT);
    pinModeFast(SCK,OUTPUT);

    power_spi_enable();
    power_timer1_enable();
      bme1.begin();
      readValues();
      bme1.end();
    power_timer1_disable();
    power_spi_disable();

    pinModeFast(SS,INPUT);
    pinModeFast(MOSI,INPUT);
    pinModeFast(MISO,INPUT);
    pinModeFast(SCK,INPUT);

    //digitalWriteFast(SS,HIGH);
    //digitalWriteFast(MOSI,HIGH);
    //digitalWriteFast(MISO,HIGH);
    //digitalWriteFast(SCK,HIGH);
    ADCSRA &= ~(1 << 7);

    checkPressure();
  }
  else if(was_whistled == true) // jeżeli poprzednio było dmuchnięcie
  {                             // TIME_TO_WAIT_MS okresla czas przerwy
    //power_twi_disable();
    if(current_positive - last_positive > TIME_TO_WAIT_MS) 
    {
      last_positive = current_positive;
      was_whistled = false;
    }
    power_timer1_disable();
  }

  checkTimeout(); // przy poprzednim dmuchnieciu funkcja ruszy 2 razy.. do zrobienia.
  //Wire.end();
  //bme1.end();

  //pinModeFast(I2C_SCL_PIN,OUTPUT);
  //pinModeFast(I2C_SDA_PIN,OUTPUT);
  //digitalWriteFast(A5,LOW); // SCL
  //digitalWriteFast(A4,LOW); // SDA
  //pinModeFast(I2C_SCL_PIN,INPUT_PULLUP);
  //pinModeFast(I2C_SDA_PIN,INPUT_PULLUP);
  //pinModeFast(I2C_SCL_PIN,INPUT_PULLUP);
  //pinModeFast(I2C_SDA_PIN,INPUT_PULLUP);
  //delay(2000);

  LowPower.powerDown(sleeptime, ADC_OFF, BOD_OFF);
}
