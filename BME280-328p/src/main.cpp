#include "main.h"

float press_odczyt, prev_press;
float press_otoczenia;
float press_dmuch;

bool was_whistled = false;      // flaga dmuchniete czy nie
bool na_minusie = false;
int press_down_cnt = 0;         // licznik cykli odczyt�w [spadek/wzrost co najmniej 2 cykle]


time_t current_positive, last_positive, current_timeout; // czas ostatniego dmuchniecia
time_t btn_current, btn_pressed_at, btn_timeout;

time_t last_blink;      // czas ostatniego blinka diody led
time_t last_rst_click;
time_t current_time;    // aktualny czas tu� po wybudzeniu

bool btn_state = LOW;
bool btn_last_state = LOW;
int btn_rst_counter = 0;

bool device_is_off = false;
bool delegate_to_longsleep = false;

bool startup = true;
period_t sleeptime = SLEEP_120MS;       // domyslny czas snu procesora
int data_repeat;

//#define DEBUGMODE

/*****************************************************
 * Przerwania
 * ***************************************************/

// Ustawia Timer1 na pr�bkowanie 6250 bps dla transmisji radiowej
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

/*****************************************************
 * Przerwanie Timer1 - 120 ms
 * ***************************************************/
ISR(TIMER1_COMPA_vect) 
{
  transmisjaCMT2110Timer();
}

/*****************************************************
 * Przerwanie dla przycisku
 * ***************************************************/
void ISR_INT0_vect()
{
  ButtonPressed();
}

/*****************************************************
 * Obsluga przerwania przycisku
 * ***************************************************/
void ButtonPressed()
{
  if(device_is_off == true)  // jesli urzadzenie jest wylaczone
  { 
    btn_pressed_at = millis();

    // wlacz i przejdz do sprawdzenia stanu przycisku
    uc_state = UC_BTN_CHECK;
  }
}

/*****************************************************
 * Wylacza wszystko do spania
 * ***************************************************/
void prepareToSleep()
{
  //clock_prescale_set(clock_div_16);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER

  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  #ifdef DEBUGMODE
    power_usart0_disable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110
}

/*****************************************************
 * SW reset - dont reset peripherials
 * ***************************************************/
void softReset(){
  //asm volatile ("  jmp 0");
  cli(); //irq's off
  wdt_enable(WDTO_60MS); //wd on,15ms
  while(1); //loop
}

/*****************************************************
 * Przygotowuje RAMKE danych do odbiornika
 * ***************************************************/
void makeMsg()
{
  // const char *msg = "1010010100000000000000001010000101000110";
  /* RAMKA DLA PILOTA
    Checksum = (u8) ([0xA5] + [0] + [0] +  [0xA1]) = 0x46
    czyli sekwencja pilota dla adresu 0x0000 powinna by�
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

/*****************************************************
 * Transmisja danych z pilota do odbiornika zgodnie z 
 * dokumentacja ELMAK na ukladzie CMT2110A z predkoscia 6250bps
 * Kodowanie bifazowe, czas p�bitu 160us
 * ***************************************************/
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
  #ifdef DEBUGMODE
    Serial.begin(9600);	// Debugging only
  #endif

  clock_prescale_set(clock_div_1);
  
  // wylacz WDT
  MCUSR= 0 ;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

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

  pinModeFast(LED_PIN,OUTPUT);
  pinModeFast(SPEAKER_PIN,OUTPUT);
  pinModeFast(TRANSMISION_PIN,OUTPUT);
  pinModeFast(USER_SWITCH,INPUT);
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
  readValues();               // pierwsze pobranie wartosci - populacja zmiennych
  prev_press = press_odczyt; // jednorazowe na poczatku w setup
  setupTimer1();              // Ustawia timer1
  power_timer1_disable(); // Timer 1 - I2C...

  makeMsg();                  // Przygotowuje ramke danych

  last_blink = millis();

  startup = false;
  was_whistled = false;
  uc_state = UC_GO_SLEEP; // default uC state
}

/*****************************************************
 * Pobiera dane z czujnika BME280
 * ***************************************************/
void readValues() 
{

  // odczyt z BME do floata wyglada tak:
  // 94995.00
  // wobec tego SENSE val to tak na prawde dziesietne i setne
  // nacisniecie:
  // po nacisnieciu nastepuje wzrost -> spadek -> i powolny wzrost do normalnych odczytow
  // wzrost o ok 5-6 hpa, spadek o ok 7-8 (ok 3 ponizej wart sprzed nacisniecia) i wzrost trwajacy ok 10-15s - 30-45s.
  //
  // dmuchniecie:
  /*
    95051.00
    95051.00
    95168.00
    95848.00 dmuch
    94718.00 spadek
    94684.00
    94731.00
    94778.00  powolny wzrost
    94815.00
    94855.00
    94879.00
    94895.00
    94914.00
    94930.00
    94940.00
  */


  prev_press = press_odczyt; // nie potrzebne?

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
  if (was_whistled == true) // gdy w poprzednim cyklu bylo dmuchane
  {
    if(press_odczyt > prev_press + SENSE_VALUE)   // jesli jest wiecej niz wartosc graniczna
    { // tu jest blad bo jesli bedzie podcisnienie 800hpa i trafi do prev to zawsze
    // bedzie true-> wiec swieci!
      was_whistled = true;
    }
    else if(press_odczyt >= press_dmuch - SENSE_WHISTLED)
    {
      was_whistled = true;
    }
    else
    {
      prev_press = press_odczyt; // musi byc tutaj aby zapobiec 1000->800
                                // malo bezpieczne gdyby np. odczyt nie byl srednia atmosferyczna
      was_whistled = false;
    }
  }
  else  // gdy w poprzednim cyklu dmuchane nie by�o
  {
    if(press_odczyt > prev_press + SENSE_VALUE)   // je�li nowy odczyt jest wiekszy o SENSE_VALUE od poprzedniego ->
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

void checkPressureTEST()
{
  if(was_whistled == false) // nie by�o dmuchni�te
  {
    if(press_odczyt >= press_otoczenia + SENSE_VALUE)
    {
      was_whistled = true;
      last_positive = millis();
    }
    else 
    {
      was_whistled = false;
    }
  }
  else // by�o dmuchni�te
  {
    if(press_odczyt <= press_otoczenia + SENSE_LOW_VALUE )
    {
      was_whistled = false;
    }
    else
    {
      was_whistled = false;
    }
  }

}

void checkPressure3()
{
  if (was_whistled == true) // gdy w poprzednim cyklu bylo dmuchane
  {
    last_positive = millis();

    if(press_odczyt >= prev_press + SENSE_VALUE)   // jesli jest wiecej niz wartosc graniczna
    { // tu jest blad bo jesli bedzie podcisnienie 800hpa i trafi do prev to zawsze
    // bedzie true-> wiec swieci!
      was_whistled = true;
      press_down_cnt = 0;
      #ifdef DEBUGMODE
      Serial.print("isT setT; "); Serial.print("NOW: "); Serial.print(press_odczyt); Serial.print(" WAS: "); Serial.print(prev_press); Serial.print(" S: ");Serial.print(press_odczyt-prev_press); Serial.println(" laaa");
      #endif
    }
    else if(press_odczyt >= press_dmuch - SENSE_WHISTLED)
    {
      was_whistled = true;
      press_down_cnt = 0;
      #ifdef DEBUGMODE
      Serial.print("isT setT; "); Serial.print("NOW: "); Serial.print(press_odczyt); Serial.print(" DMU: "); Serial.print(press_dmuch); Serial.print(" S: ");Serial.print(press_odczyt-prev_press); Serial.println(" laaa");
      #endif
    }
    else
    {
     //prev_press = press_odczyt; // musi byc tutaj aby zapobiec 1000->800
                                // malo bezpieczne gdyby np. odczyt nie byl srednia atmosferyczna
      // nowe: podwojne sprawdzanie
      // wylaczy dopiero po drugim cyklu
      press_down_cnt++;
      if(press_down_cnt > 1)
      {
        was_whistled = false;
        press_down_cnt = 0;
      }
      
      #ifdef DEBUGMODE
      Serial.print("isT setF; "); Serial.print("NOW: "); Serial.print(press_odczyt); Serial.print(" WAS: "); Serial.print(prev_press); Serial.print(" S: ");Serial.print(press_odczyt-prev_press); Serial.println(" laaa");
      #endif
    }
  }
  else  // gdy w poprzednim cyklu dmuchane nie by�o
  {
    if((press_odczyt > prev_press + SENSE_VALUE) && na_minusie == false)
    {
      was_whistled = true;
      press_dmuch = press_odczyt;
      #ifdef DEBUGMODE
      Serial.print("isF setT; "); Serial.print("NOW: "); Serial.print(press_odczyt); Serial.print(" WAS: "); Serial.print(prev_press); Serial.print(" S: ");Serial.print(press_odczyt-prev_press); Serial.println(" laaa");
      #endif
    }
    else
    {
      was_whistled = false;
      #ifdef DEBUGMODE
      Serial.print("isF setF; "); Serial.print("NOW: "); Serial.print(press_odczyt); Serial.print(" WAS: "); Serial.print(prev_press); Serial.print(" S: ");Serial.print(press_odczyt-prev_press); Serial.println(" laaa");
      #endif
    }
  }

  if (press_odczyt < prev_press - SENSE_VALUE)
  {
    na_minusie = true;
  }
  else
  {
    na_minusie = false;
  }
}
/*****************************************************
 * TEST - TIMEOUT
 * ***************************************************/
void checkTimeout()
{
  //current_positive = millis();  // pobierz czas.
  //current_timeout = current_positive - last_positive;

  current_timeout = current_time - last_positive;

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
    delegate_to_longsleep = true;
    device_is_off= true;
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
      //delay(5);
      //digitalWriteFast(5, HIGH);

      // idzie w kimono kompletnie do czasu wybudzenia przyciskiem
      if(delegate_to_longsleep == true) 
      {
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(2), ISR_INT0_vect, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
      }
      else  // idziesz w krotkie kimonko
      {
        prepareToSleep();
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        interrupts();
        uc_state = UC_WAKE_AND_CHECK; // pokima�? to sprawdzi� co sie dzieje->
      }

      //digitalWriteFast(5,LOW);
      clock_prescale_set(clock_div_1); // podczas spania 1mhz -> po pobudce 8

      break;
    }

    // Pobudka i sprawdzamy czujnik
    // Blinki itp
    case UC_WAKE_AND_CHECK:
    {
      readValues();                 // odczyt
          //digitalWriteFast(LED_PIN, HIGH);
          //delay(10);
          //digitalWriteFast(LED_PIN, LOW);
      checkPressure3();             // compare
      current_time = millis();

      if (was_whistled == true)     // zmiana -> wysylamy
      {
        uc_state = UC_SENDING_DATA;
        tx_state = TX_WAKEUP_CMT;
      }
      else                          // brak zmiany -> sio spac dalej
      {
        uc_state = UC_BTN_CHECK;

        // blink
        if(current_time - last_blink >= 140)// 200ms = realnie jakies 5s przez deepsleep.
        {                                   // wychodzi ok 25x wolniej niz real
          last_blink = current_time;
          digitalWriteFast(LED_PIN, HIGH);
          delay(15);
          digitalWriteFast(LED_PIN, LOW);
        }
      }
      break;
    }

    // Wysylka pakietow danych do odbiornika -> do spania!
    case UC_SENDING_DATA:
    {
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
        data_repeat = DATA_REPEAT_CNT;    // ilosc powtorzen z conf.
        //interrupts();                   // przerwania wlacz
        power_timer1_enable();            // wlacz timer1 6250bps to send data.
        tx_state = TX_SENDING_PROGRESS;   // czekaj na wyslanie 
      }
      else if(tx_state == TX_SENDING_PROGRESS)
      {
        // just wait for all bits to be sent..
        // funkcja wysylajaca dane w przewaniu po wykonaniu serii 
        // ustawia flage TX_SENDING_REPEAT..
        #ifdef DEBUGMODE
          tx_state = TX_SENDING_REPEAT;
          power_timer1_disable();
        #endif
      }
      else if(tx_state == TX_SENDING_REPEAT) // POWTARZAMY RAMKE (min 3x)
      {
        if(data_repeat > 0)
        {
          delay(10);
          data_repeat--; // jesli ramka cala poszla to zmniejsz ilosc powtorzen..
          BitNr = 0;
          //interrupts();           // przerwania wlacz
          power_timer1_enable();
          tx_state = TX_SENDING_PROGRESS;
        }
        else// (data_repeat == 0)
        {
          //delay(140); // -> delay powinien pojsc tam gdzie wykryte bedzie 
          //dalsze dmuchanie
          // nie ma delay gdy sleeptime = 120MS -> robi jednoczesnie za delay.
          uc_state = UC_SENDING_DONE;
          power_timer1_disable();
        }
      }
      break;
    }

    case UC_SENDING_DONE:
    {
      uc_state = UC_BTN_CHECK;
      break;
    }

    case UC_BTN_CHECK: // sprawdza przycisk na koniec petli lub po wybudzeniu
    {
      delegate_to_longsleep = false;  // inaczej pojdzie spac long
                                      // device_is_off pozostaje dla ustalenia czy 
                                      // nadajnik był wybudzony czy pracowal normalnie

      btn_last_state = btn_state;     // do rst
      btn_state = digitalReadFast(USER_SWITCH); // odczyt stanu guzika
      
      current_time = millis();

      if(btn_state != btn_last_state) // jezeli stan przycisku sie zmienil
      {
        if(btn_state == HIGH)         // jezeli jest wysoki
        {
          btn_rst_counter++;
          last_rst_click = current_time;  // zeruj timeout
        }
        if(current_time - last_rst_click >= SW_RST_TIMEOUT)
        {
          btn_rst_counter = 0;
          last_rst_click = current_time;  // jezeli reset to reset...
        }
        if(btn_rst_counter >= SW_RST_COUNT)
        {
          for(int i=0; i<20; i++)
          {
            digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
            delay(100);
          }
          softReset();
        }
      }

      // jesli przycisk nie jest wcisniety lub zostal zwolniony
      if(btn_state == LOW)
      {
        btn_pressed_at = current_time; // ustaw obecny czas
        // jesli przycisk zostanie nacisniety ostatnia wartość stad nie bedzie nadpisywana
      }

      // jesli przycisk nie jest wcisniety gdy urzadzenie pracuje -> loop
      if(btn_state == false && device_is_off == false)
      {
        btn_pressed_at = current_time; // to wlasciwie mozna usunac na rzecz tego na gorze?
        // i od razu w krotka kime
        uc_state = UC_GO_SLEEP;
        break;
      }

      // jesli sie obudzi po przerwaniu a przycisk juz nie jest wcisniety -> deepsleep
      if(btn_state == false && device_is_off == true)
      {
        device_is_off = true;           // flaga off dla pewnosci
        delegate_to_longsleep = true;   // deleguj do glebokiego snu
        uc_state = UC_GO_SLEEP;
      }

      // jesli przycisk wcisniety gdy urzadzenie bylo wylaczone:
      if(btn_state == true && device_is_off == true) // jesli guzik + nadajnik off
      {
        if(current_time - btn_pressed_at >= SWITCH_TIMEOUT)
        {
          // pobudka
          btn_pressed_at = current_time;
          digitalWriteFast(LED_PIN,HIGH);
          delay(1000);
          digitalWriteFast(LED_PIN,LOW);

          device_is_off = false;

          // po dlugim snie moze przy checktimeout wpasc znow w deepsleep
          // dlatego last positice = teraz
          last_positive = current_time; 

          detachInterrupt(digitalPinToInterrupt(2));
          uc_state = UC_WAKE_AND_CHECK;
        }
        //uc_state = UC_GO_SLEEP;// nowe - przemyslec!
      }

      // jesli przycisk wcisniety a urzadzenie pracuje normalnie:
      else if(btn_state == true && device_is_off == false) // guzik + nadajnik ON
      {
        if(current_time - btn_pressed_at >= SWITCH_TIMEOUT)
        {
          // spij
          device_is_off = true;
          delegate_to_longsleep = true;          
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          delay(400);
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          uc_state = UC_GO_SLEEP;
        }
      }
      else
      {
        // yyyyy...
      }

      break;
    }
  }
  checkTimeout();
}