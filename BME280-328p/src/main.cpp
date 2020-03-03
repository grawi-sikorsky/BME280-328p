#include "main.h"

float press_odczyt, prev_press;
float press_otoczenia;
float press_dmuch;

time_t current_positive, last_positive, current_timeout; // czas ostatniego dmuchniecia
bool was_whistled = false;              // flaga dmuchniete czy nie
bool startup = true;
period_t sleeptime = SLEEP_120MS;       // domyslny czas snu procesora
int data_repeat;


// Ustawia Timer1 na próbkowanie 6250 bps dla transmisji radiowej
// 160 us polbit / 320 us bit transmisji
void setupTimer1()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 6250 Hz (8000000/((4+1)*256))
  OCR1A = 4;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 256
  TCCR1B |= (1 << CS12);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

// Wylacza wszystko do spania
void prepareToSleep()
{
  clock_prescale_set(clock_div_16);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER

  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110
}

// Przerwanie Timer1 120 ms
ISR(TIMER1_COMPA_vect) 
{
  transmisjaCMT2110Timer();
}


// Przygotowuje RAMKE danych do odbiornika
void makeMsg()
{
  // const char *msg = "1010010100000000000000001010000101000110";
  /* RAMKA DLA PILOTA
    Checksum = (u8) ([0xA5] + [0] + [0] +  [0xA1]) = 0x46
    czyli sekwencja pilota dla adresu 0x0000 powinna być
    1010 0101 0000 0000 0000 0000 1010 0001 0100 0110
  */

  u8 i;
	u8 idx = 0;
  HalfBit = 0;

  Checksum = Prefix + AdrMsb + AdrLsb + Cmd; // Oblicz sume kontrolna

	i=0;
	while(i < 8)
		{
			if((Prefix & (0x80 >> i)) == (0x80 >> i))
				TxTbl[idx] = 1;
			else
				TxTbl[idx] = 0;
		i++;
		idx++;	
		}

	i=0;
	while(i < 8)
		{
			if((AdrMsb & (0x80 >> i)) == (0x80 >> i))
				TxTbl[idx] = 1;
			else
				TxTbl[idx] = 0;
		i++;
		idx++;	
		}
		
	i=0;
	while(i < 8)
		{
			if((AdrLsb & (0x80 >> i)) == (0x80 >> i))
				TxTbl[idx] = 1;
			else
				TxTbl[idx] = 0;
		i++;
		idx++;	
		}	
		
	i=0;
	while(i < 8)
		{
			if((Cmd & (0x80 >> i)) == (0x80 >> i))
				TxTbl[idx] = 1;
			else
				TxTbl[idx] = 0;
		i++;
		idx++;	
		}	
		
	i=0;
	while(i < 8)
		{
			if((Checksum & (0x80 >> i)) == (0x80 >> i))
				TxTbl[idx] = 1;
			else
				TxTbl[idx] = 0;
		i++;
		idx++;
		}
}

//  Transmisja danych z pilota do odbiornika zgodnie z dokumentacja ELMAK na ukladzie CMT2110A z predkoscia 6250bps
//  Kodowanie bifazowe, czas półbitu 160us
void transmisjaCMT2110Timer()
{
  if(BitNr <= 40) // jesli ramka do zrobienia
	{
		HalfBit++;
		HalfBit &= 0x01;
	
  	if(HalfBit == 1)
			BitNr++;
		
		if(HalfBit == 0)	// pierwszy polbit
		{
      if(TxTbl[BitNr] == 0)
        PORTD &= ~(1 << PD0);   // LOW
      else
        PORTD |= (1 << PD0);    // SUMA HIGH?
		}
		else
    {
      PORTD ^= (1 << PD0);      // TOGGLE
    }
  }
	else // jesli brak bitow w ramce danych
	{
		PORTD &= ~(1 << PD0);   // LOW
		
    tx_state = TX_SENDING_REPEAT;
    power_timer1_disable();
	}
}

/*****************************************************
 * SETUP
 * ***************************************************/
void setup() 
{
  //clock_prescale_set(clock_div_16);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  power_adc_disable(); // ADC converter
  //power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110

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

  power_timer1_enable();  // Timer 1 - I2C...
  readValuesStartup();        
  readValues();               // pierwsze pobranie wartosci - populacja zmiennych

  prev_press = press_odczyt; // jednorazowe na poczatku w setup

  makeMsg();                  // Przygotowuje ramke danych
  setupTimer1();              // Ustawia timer1
  power_timer1_disable(); // Timer 1 - I2C...

  startup = false;
  was_whistled = false;
  uc_state = UC_GO_SLEEP; // default uC state
}

/*****************************************************
 * Pobiera dane z czujnika BME280 na poczatku i przypisuje do ogolnego cisnienia
 * ***************************************************/
void readValuesStartup()
{
  pinModeFast(SS,OUTPUT);   // Ustaw piny SPI jako OUTPUT na czas pomiaru
  pinModeFast(MOSI,OUTPUT); // bardzo niewielka ale zdaje sie jednak oszczednosc prundu
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);

  power_spi_enable();
  bme1.begin();
  bme1.setMode(11);

  for(int i=1; i<=5; i++)
  {
    press_otoczenia += bme1.readFixedPressure();  // Odczyt z czujnika bme
    press_otoczenia = press_otoczenia/i;
    delay(200);
  }

  bme1.setMode(00);
  bme1.end();
  power_spi_disable();

  pinModeFast(SS,INPUT);    // Ustaw piny SPI jako INPUT po pomiarze
  pinModeFast(MOSI,INPUT);  // bardzo niewielka ale zdaje sie jednak oszczednosc prundu
  pinModeFast(MISO,INPUT);
  pinModeFast(SCK,INPUT);
}
/*****************************************************
 * Pobiera dane z czujnika BME280
 * ***************************************************/
void readValues() 
{
  prev_press = press_odczyt;       // Przypisz cisnienie do zmiennej przed kolejnym pomiarem 

  pinModeFast(SS,OUTPUT);   // Ustaw piny SPI jako OUTPUT na czas pomiaru
  pinModeFast(MOSI,OUTPUT); // bardzo niewielka ale zdaje sie jednak oszczednosc prundu
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);

  power_spi_enable();
  bme1.begin();
  bme1.setMode(11);

  press_odczyt = bme1.readFixedPressure();  // Odczyt z czujnika bme

  bme1.setMode(00);
  bme1.end();
  power_spi_disable();

  pinModeFast(SS,INPUT);    // Ustaw piny SPI jako INPUT po pomiarze
  pinModeFast(MOSI,INPUT);  // bardzo niewielka ale zdaje sie jednak oszczednosc prundu
  pinModeFast(MISO,INPUT);
  pinModeFast(SCK,INPUT);

  ADCSRA &= ~(1 << 7);  // TURN OFF ADC CONVERTER
}

/*****************************************************
 * Sprawdza czy cisnienie jest wieksze od zalozonego
 * ***************************************************/
void checkPressure()
{ 
  if (was_whistled == true) // gdy bylo dmuchane
  {
    if(press_odczyt > prev_press + SENSE_VALUE)   // jesli jest wiecej niz wartosc graniczna
    {
      was_whistled = true;
    }
    else if(press_odczyt >= press_dmuch - SENSE_WHISTLED)
    {
      was_whistled = true;
    }
    else
    {
      was_whistled = false;
    }
  }
  else  // gdy dmuchane nie było
  {
    if(press_odczyt > prev_press + SENSE_VALUE)   // jeśli nowy odczyt jest wiekszy o SENSE_VALUE od poprzedniego ->
    {
      was_whistled = true;
      press_dmuch = press_odczyt;
    }
    else
    {
      was_whistled = false;
    }
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
  {
    sleeptime = SLEEP_120MS;
  }
  else if(current_timeout > TIMEOUT_1 && current_timeout < TIMEOUT_2) // drugi prog
  {
    // zmniejsz probkowanie 2x/s
    sleeptime = SLEEP_500MS; // 1S
  } 
  else if(current_timeout > TIMEOUT_2 )
  { 
    sleeptime = SLEEP_1S; // 1S
    //sleeptime = SLEEP_FOREVER;
    // setup ISR to WAKE UP!
    power_twi_disable();
    power_usart0_disable();
  }  
}

/**************************
 *  LOOP
 * ************************/
void loop() 
{
  switch (uc_state)
  {
    // Idz spac w pizdu.
    case UC_GO_SLEEP:
    {
      digitalWriteFast(5, HIGH);
      prepareToSleep();
      LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
      digitalWriteFast(5,LOW);

      uc_state = UC_WAKE_AND_CHECK; // pokimał? to sprawdzić co sie dzieje->
      break;
    }

    // Pobudka i sprawdzamy czujnik
    case UC_WAKE_AND_CHECK:
    {
      clock_prescale_set(clock_div_1);
      readValues();                 // odczyt
      checkPressure();              // 
                
      if (was_whistled == true)     // zmiana -> wysylamy
      {
        uc_state = UC_SENDING_DATA;
        tx_state = TX_WAKEUP_CMT;
      }
      else                          // brak zmiany -> sio spać dalej
      {
        uc_state = UC_GO_SLEEP;
      }
      break;
    }

    // Wysylka pakietow danych do odbiornika -> do spania!
    case UC_SENDING_DATA:
    {
      //transmisjaCMT2110();
      if(tx_state == TX_WAKEUP_CMT)
      {
        digitalWriteFast(TRANSMISION_PIN,HIGH); // wybudzenie CMT2110
        delayMicroseconds(100); //
        digitalWriteFast(TRANSMISION_PIN,LOW); // wybudzenie CMT2110
        delay(4);
        tx_state = TX_SENDING_START;
      }
      else if(tx_state == TX_SENDING_START) // WYSYlAMY CALA RAMKE
      {
        BitNr = 0;
        HalfBit = 0;
        data_repeat = DATA_REPEAT_CNT;  // ilosc powtorzen
        //interrupts();                   // przerwania wlacz
        power_timer1_enable();          // wlacz timer 6250bps to send data.
        tx_state = TX_SENDING_PROGRESS; // czekaj na wyslanie 
      }
      else if(tx_state == TX_SENDING_PROGRESS)
      {
        // just wait for all bits to be sent..
      }
      else if(tx_state == TX_SENDING_REPEAT) // POWTARZAMY RAMKE (min 3x)
      {
        if(data_repeat > 0)
        {
          delay(5);
          data_repeat--; // jesli ramka cala poszla to zmniejsz ilosc powtorzen..
          BitNr = 0;
          //interrupts();           // przerwania wlacz
          power_timer1_enable();
          tx_state = TX_SENDING_PROGRESS;
        }
        else if(data_repeat == 0)
        {
          //delay(140); // -> delay powinien pojsc tam gdzie wykryte bedzie dalsze dmuchanie
          uc_state = UC_SENDING_DONE;
          power_timer1_disable();
        }
      }
      break;
    }

    case UC_SENDING_DONE:
    {
      uc_state = UC_GO_SLEEP;
      break;
    }
  }
  //checkTimeout(); // przy poprzednim dmuchnieciu funkcja ruszy 2 razy.. do zrobienia.
}
