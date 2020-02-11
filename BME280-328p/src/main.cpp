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
float press, prev_press;

    #define WDT_0125s()             WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP1)|(1<<WDP0)             //4
    #define WDT_025s()              WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)                       //5
    #define WDT_05s()               WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP0)             //6
    #define WDT_1s()                WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1)             //7
    #define WDT_2s()                WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0)   //8


void printValues() {
    prev_press = press;
    press = bme.readPressure();

    Serial.print("P= ");
    Serial.println(press / 100.0F);
}
void checkPress()
{
    if(press > prev_press + 10 )
    {
        digitalWrite(7,HIGH);
        delay(60/4);
        digitalWrite(7,LOW);
    }
}

 void gotosleep()
 {
    // disable ADC
    ADCSRA = 0;  
    // clear various "reset" flags
    MCUSR = 0;     
    // allow changes, disable reset
    WDTCSR = bit (WDCE) | bit (WDE);
    // set interrupt mode and an interval 
   // WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 second delay
    WDT_025s();
    wdt_reset();  // pat the dog
    
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
    noInterrupts ();           // timed sequence follows
    sleep_enable();
    
    // turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    interrupts ();             // guarantees next instruction executed
    sleep_cpu ();
 }

void setup() {
    Serial.begin(9600);
    clock_prescale_set(clock_div_4);
    
    for (byte i = 0; i <= A5; i++)
    {
        pinMode (i, OUTPUT);    // changed as per below
        digitalWrite (i, LOW);  //     ditto
    }

    digitalWrite(13, LOW); // LED OFF

    power_adc_disable(); // ADC converter
    power_spi_disable(); // SPI
    //power_usart0_disable();// Serial (USART)
    //power_timer0_disable();// Timer 0
    power_timer1_disable();// Timer 1
    //power_timer2_disable();// Timer 2
    //power_twi_disable(); // TWI (I2C)
}



void loop() {
    bme.begin(0x76);

    printValues();
    checkPress();

    //delay(1);
    digitalWrite(13, HIGH);
    delay(20/4);
    digitalWrite(13, LOW);
    //delay(50/8);

    power_adc_disable(); // ADC converter
    power_spi_disable(); // SPI
    //power_usart0_disable();// Serial (USART)
    //power_timer0_disable();// Timer 0
    power_timer1_disable();// Timer 1
    power_timer2_disable();// Timer 2
    //power_twi_disable(); // TWI (I2C)

    //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);

    //gotosleep();
    
    // cancel sleep as a precaution
    //sleep_disable();
}