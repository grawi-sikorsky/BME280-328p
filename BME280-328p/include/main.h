#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <time.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>

// SLEEP LIB
#include "LowPower.h"

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"

// PINY
#define LED_PIN         5
#define SPEAKER_PIN     7 //A2 // 7 minipro
#define TRANSMISION_PIN 0 // 4 w proto
#define USER_SWITCH     2 // PD2 INT0

// KONFIGURACJA
#define PRESS_ATM_PROBE_COUNT 10
#define SENSE_VALUE     85
#define SENSE_LOW_VALUE 25      

#define SENSE_WHISTLED  15             // +/- widelki podczas wykrycia dmuchniecia
#define VACUUM_IGNORE   1     

#define SWITCH_TIMEOUT  3000          // czas przycisku nacisniecia
#define SW_RST_TIMEOUT  100          // czas w ktorym należy wykonac klikniecia dla RST
#define SW_RST_COUNT    5             // ilosc nacisniec do wykonania resetu
#define TIME_TO_WAIT_MS 50            // czas do nastepnego wyzwolenia
#define TIMEOUT_1       72000       // pierwszy timeiut // realnie wychodzi jakies (1 800 000 ms = 30 min) / 25 = 72000
#define TIMEOUT_2       144000       // drugi prog = 5 400 000 = 90 min // z uwagi na sleep-millis: 60 min
#define DATA_REPEAT_CNT 2               // ilosc powtorzen transmisji (min 3 lub wiecej)

tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

void makeMsg();       // przygotowanie ramki danych
void readValuesStartup();
void readValues();    // odczyt danych z czujnika
void checkPressure(); // sprawdza czy wzrost
void checkPressureTEST();
void checkTimeout();  // sprawdzenie czasu
void transmisjaCMT2110Timer();

void CheckButtonState();
void ButtonPressed();
void TurnOff();
void Blink();

void prepareToSleep();
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
  UC_GO_SLEEP = 0,
  UC_WAKE_AND_CHECK = 1,
  UC_SENDING_DATA = 2,
  UC_SENDING_DONE = 3,
  UC_BTN_CHECK  = 4,
};
enum transmission_State {
  TX_WAKEUP_CMT = 0,
  TX_SENDING_START = 1,
  TX_SENDING_PROGRESS = 2,
  TX_SENDING_REPEAT = 3,
};

uc_State uc_state;
transmission_State tx_state;