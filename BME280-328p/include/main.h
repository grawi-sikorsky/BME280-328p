#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <time.h>
#include <digitalWriteFast.h>

// SLEEP LIB
#include "LowPower.h"

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"

// SENSING VAL
#define SENSE_VALUE 30

#define TIME_TO_WAIT_MS 50              // czas do nastepnego wyzwolenia
#define TIMEOUT_1       3000            // pierwszy timeiut
#define TIMEOUT_2       5000
#define LED_PIN         5
#define SPEAKER_PIN     7 //A2 // 7 minipro
#define TRANSMISION_PIN 0 // 4 w proto

tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

void makeMsg();       // przygotowanie ramki danych
void readValues();    // odczyt danych z czujnika
void checkTimeout();  // sprawdzenie czasu
void transmisjaCMT2110();
void transmisjaCMT2110Timer();

void setupTimer1();


// Transmisja 
u8 TxTbl[40];
u8 Prefix = 0xA5; // Dzien dobry
u8 AdrMsb = 0x00; // pierwszy bajt klucza
u8 AdrLsb = 0x00; // drugi bajt klucza
u8 Cmd = 0xA1;	  // Komenda
u8 Checksum = 0;  // Suma kontrolna // Razem 40 bit.
u8 BitNr;

volatile u8 HalfBit = 0;

enum uc_State {
    SLEEPING = 0,
    WAKE_AND_CHECK = 1,
    SENDING_DATA = 2,
  };
uc_State uc_state;