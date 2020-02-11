#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <digitalWriteFast.h>
#include "LowPower.h"

//#define TINYBME

#ifdef TINYBME
    #define TINY_BME280_I2C
    #include "TinyBME280.h"
#else
    #include <Adafruit_Sensor.h>
    #include "Adafruit_BME280.h"
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

unsigned long delayTime;
float press, prev_press;


void printValues() {
    prev_press = press;

    #ifdef TINYBME
        press = bme1.readFixedPressure();
    #else
        press = bme.readPressure();
    #endif

    Serial.println(press);
}

void checkPress()
{
    if(press > prev_press + 10 )
    {
        digitalWriteFast(7,HIGH);   // SPK
        delay(1);
        digitalWriteFast(7,LOW);    // SPK
    }
}

void setup() {
    Serial.begin(9600);
    #ifdef TINYBME
        bme1.setI2CAddress(0x76);
        bme1.beginI2C();
    #endif
    //clock_prescale_set(clock_div_1);
    
    for (byte i = 0; i <= A5; i++)
    {
        pinModeFast(i, OUTPUT);    // changed as per below
        digitalWriteFast(i, HIGH);  //     ditto
    }

    digitalWriteFast(13, LOW);  // LED OFF
    digitalWriteFast(7,LOW);    // SPK

    power_adc_disable(); // ADC converter
    power_spi_disable(); // SPI
    //power_usart0_disable();// Serial (USART)
    //power_timer0_disable();// Timer 0
    power_timer1_disable();// Timer 1
    power_timer2_disable();// Timer 2
    //power_twi_disable(); // TWI (I2C)
}

// Brown-out disable // ->  avrdude -c usbtiny -p m328p -U efuse:w:0x07:m


void loop() {
    Serial.begin(9600);
    //power_twi_enable();
    bme.begin(0x76);
    //bme1.beginI2C();
    //bme1.reset();
    printValues();
    checkPress();

    //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    //power_twi_disable(); // TWI (I2C)
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
}