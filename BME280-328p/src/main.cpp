#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "LowPower.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect
 

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running

    for (byte i = 0; i <= A5; i++)
    {
        pinMode (i, OUTPUT);    // changed as per below
        digitalWrite (i, LOW);  //     ditto
    }

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  

    if (!status) {
        Serial.println("Cant connect");
        while (1) delay(10);
    }
}


void loop() {
    Serial.begin(9600);
    bme.begin(0x76);
    delay(1);
    printValues();

    // disable ADC
    ADCSRA = 0;  
    // clear various "reset" flags
    MCUSR = 0;     
    // allow changes, disable reset
    WDTCSR = bit (WDCE) | bit (WDE);
    // set interrupt mode and an interval 
    WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 second delay
    wdt_reset();  // pat the dog
    
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
    noInterrupts ();           // timed sequence follows
    sleep_enable();
    
    // turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    interrupts ();             // guarantees next instruction executed
    sleep_cpu ();  
    
    // cancel sleep as a precaution
    sleep_disable();
}