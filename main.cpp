/*
 * main.cpp
 *
 *  Created on: Jul 19, 2012
 *      Author: rasmus
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

/******************************************************************************
 * HT1632 Ports and pins
 ******************************************************************************/
#define HTport   PORTB
#define HTddr    DDRB
#define HTstrobe 3
#define HTclk    4
#define HTdata   5

/******************************************************************************
 * HT1632 Control Macros
 *****************************************************************************/

#define HTclk0    HTport&=~(1<<HTclk)           //Set HT clock pin low
#define HTclk1    HTport|= (1<<HTclk)           //Set HT clock pin high
#define HTstrobe0() HTport&=~(1<<HTstrobe)      //Set HT Strobe pin low
#define HTstrobe1() HTport|= (1<<HTstrobe)      //Set HT Strobe pin high
#define HTdata0   HTport&=~(1<<HTdata)          //Set HT Data pin low
#define HTdata1   HTport|= (1<<HTdata)          //Set HT Data pin high
//Set all pins on HT port to output and high
#define HTpinsetup() {  HTddr |=(1<<HTstrobe)|(1<<HTclk)|(1<<HTdata); HTport|=(1<<HTstrobe)|(1<<HTclk)|(1<<HTdata); }

/******************************************************************************
 * Key input setup and  macros
 *****************************************************************************/

#define keysetup() { DDRD&=0xff-(1<<7)-(1<<6)-(1<<5); PORTD|=(1<<7)+(1<<6)+(1<<5); }  //input, pull up
#define key1 ((PIND&(1<<7))==0)
#define key2 ((PIND&(1<<6))==0)
#define key3 ((PIND&(1<<5))==0)

/******************************************************************************
 * LED memory array
 *****************************************************************************/
volatile uint8_t leds[32]; //the screen array, 1 uint8_t = 1 column, left to right, lsb at top.
volatile uint8_t last_leds1[32]; //the screen array, 1 uint8_t = 1 column, left to right, lsb at top.
volatile uint8_t last_leds2[32]; //the screen array, 1 uint8_t = 1 column, left to right, lsb at top.
volatile uint8_t interim_blinks_left = 0;
volatile uint8_t TCCR2_Backup;
/******************************************************************************
 * HT 1632 commands
 *****************************************************************************/
#define HTstartsys   0b100000000010 //start system oscillator
#define HTstopsys    0b100000000000 //stop sytem oscillator and LED duty    <default
#define HTsetclock   0b100000110000 //set clock to master with internal RC  <default
#define HTsetlayout  0b100001000000 //NMOS 32*8 // 0b100-0010-ab00-0  a:0-NMOS,1-PMOS; b:0-32*8,1-24*16   default:ab=10
#define HTledon      0b100000000110 //start LEDs
#define HTledoff     0b100000000100 //stop LEDs    <default
#define HTsetbright  0b100101000000 //set brightness b=0..15  add b<<1  //0b1001010xxxx0 xxxx:brightness 0..15=1/16..16/16 PWM
#define HTblinkon    0b100000010010 //Blinking on
#define HTblinkoff   0b100000010000 //Blinking off  <default
/******************************************************************************
 * HT1632 Write operation
 * ADDRESS: MSB first
 * DATA: LSB first     transferring a uint8_t (msb first) fills one row of one 8*8-matrix, msb left, starting with the left matrix
 * timing: pull strobe LOW, bits evaluated at rising clock edge, strobe high
 * commands can be queued: 100-ccccccccc-ccccccccc-ccccccccc-... (ccccccccc: without 100 at front)
 *****************************************************************************/
#define HTwrite      0b1010000000   // 101-aaaaaaa-dddd-dddd-dddd-dddd-dddd-... aaaaaaa:nibble adress 0..3F   (5F for 24*16)
static inline void timersetup()
{
  //Timer 0 setup
  //This interrupt should happen at 8000000/256/256 = 122.0703125 Hz
  TCCR0 |= (1 << CS02); // Set timer0 prescaler to 256
  TIMSK |= (1 << TOIE0); // enable timer0 overflow interrupt

  //Timer 2 setup
  //This overflow interrupt should happen at 32768Hz/256/128 = 1 Hz
  ASSR |= (1 << AS2); // Set timer2 to use external 32768Hz clock
  TCCR2 |= (1 << CS20) | (1 << CS21); // Set timer2 prescaler to 32
  TIMSK |= (1 << TOIE2); // enable timer0 overflow interrupt

  sei();
  //enable interrupts
}

static inline void HTsetup()
{
  HTcommand(HTstartsys);
  HTcommand(HTledon);
  HTcommand(HTsetclock);
  HTcommand(HTsetlayout);
  HTbrightness(15);
  HTcommand(HTblinkoff);
}

static inline void ADCInit()
{
  // set analog to digital converter
  // Reference == VCC (5v), single ended input ADC2
  ADMUX = 0x42;

  // set analog to digital converter
  // to be enabled, with a clock prescale of 1/128
  // so that the ADC clock runs at 115.2kHz.
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // fire a conversion just to get the ADC warmed up
  ADCSRA |= (1 << ADSC);
}

void initrand()
{
  uint32_t state;
  static uint32_t EEMEM sstate;

  state = eeprom_read_dword(&sstate);

  srandom(state);
  eeprom_write_dword(&sstate, random());
}

void HTsend(uint16_t data, uint8_t n)
{
  // Send out 1 bit at a time.
  // Start at MSB of a bit array n bits long
  // bit has a single bit high at the bit that is about to be sent.
  uint16_t bit = ((uint16_t) 1) << (n - 1);
  while (bit) //send n bits
  {
    HTclk0; //Start by pulling write clock low
    if(data & bit) //If the data has a high bit at this bit position.
      HTdata1; //Pull data pin high
    else
      HTdata0; //Else pull it low.
    HTclk1; //Send the bit by pulling write clock high
    bit >>= 1; //Go to next bit in data... repeat
  }
}

void HTcommand(uint16_t data)
{
  HTstrobe0();
  HTsend(data, 12);
  HTstrobe1();
}

void HTsendscreen()
{
  HTstrobe0();
  HTsend(HTwrite, 10);
  for (uint8_t mtx = 0; mtx < 4; mtx++) //sending 8x8-matrices left to right, rows top to bottom, MSB left
    for (uint8_t row = 0; row < 8; row++)
    {
      //while leds[] is organized in columns for ease of use.
      uint8_t q = 0;
      for (uint8_t col = 0; col < 8; col++)
        q = (q << 1) | ((leds[col + (mtx << 3)] >> row) & 1);
      HTsend(q, 8);
    }
  HTstrobe1();
}

void HTbrightness(uint8_t b)
{
  HTcommand(HTsetbright + ((b & 15) << 1));
}

void UpdateLife()
{
//  Any live cell with fewer than two live neighbours dies, as if caused by under-population.
//  Any live cell with two or three live neighbours lives on to the next generation.
//  Any live cell with more than three live neighbours dies, as if by overcrowding.
//  Any dead cell with exactly three live neighbours becomes a live cell, as if by reproduction.

  if(interim_blinks_left)
  {
    interim_blinks_left--;

    if(interim_blinks_left)
    {
      if(interim_blinks_left == 4)
      {
        RandomLeds();
      }
      else if(interim_blinks_left > 4)
      {
        //Invert cell array
        for (uint8_t i = 0; i < 32; i++)
        {
          leds[i] = ~leds[i];
        }
      }

      return;
    }
    else
    {
      TCCR2 = TCCR2_Backup;
    }
  }

  uint8_t next_leds[32]; //the screen array, 1 uint8_t = 1 column, left to right, lsb at top.
  for (uint8_t i = 0; i < 32; i++)
  {

    next_leds[i] = leds[i];

  }

  uint8_t neigbours = 0;
  for (uint16_t x = 0; x < 32; x++)
  {
    for (uint16_t y = 0; y < 8; y++)
    {

      uint8_t left_x = x > 0 ? x - 1 : 31;
      uint8_t right_x = x < 31 ? x + 1 : 0;
      uint8_t over_y = y > 0 ? y - 1 : 7;
      uint8_t under_y = y < 7 ? y + 1 : 0;

      if(leds[left_x] & (1 << (y)))
      {
        neigbours++;
      }

      if(leds[right_x] & (1 << (y)))
      {
        neigbours++;
      }

      if(leds[x] & (1 << (over_y)))
      {
        neigbours++;
      }

      if(leds[x] & (1 << (under_y)))
      {
        neigbours++;
      }

      if(leds[left_x] & (1 << (over_y)))
      {
        neigbours++;
      }

      if(leds[right_x] & (1 << (under_y)))
      {
        neigbours++;
      }

      if(leds[right_x] & (1 << (over_y)))
      {
        neigbours++;
      }

      if(leds[left_x] & (1 << (under_y)))
      {
        neigbours++;
      }

      if((leds[x] & (1 << (y))))
      {
        if(neigbours < 2 || neigbours > 3) next_leds[x] &= ~(1 << (y));
      }
      else
      {
        if(neigbours == 3)
        {
          next_leds[x] |= (1 << (y));
        }
      }

      neigbours = 0;
    }
  }

  uint8_t still = 1;
  for (uint8_t i = 0; i < 32; i++)
  {
    //Copy out new led array
    leds[i] = next_leds[i];
    //Check if anything happened on the led array since last iteration.
    if((leds[i] != last_leds1[i]) && (leds[i] != last_leds2[i]))
    {
      still = 0;
    }
    //Copy frame to buffer to compare next time.
    last_leds2[i] = last_leds1[i];
    last_leds1[i] = leds[i];

  }

  //if nothing has happened in an iteration, raise flag
  if(still)
  {
    ResetGameOfLife();
  }
}

void RandomLeds()
{
  //Set cell array to random pattern
  for (uint8_t i = 0; i < 32; i++)
  {
    leds[i] = random() / (RAND_MAX / (256));
  };
}

void ResetGameOfLife()
{
  //Only allow reset if not already doing it.
  if(!interim_blinks_left)
  {
    interim_blinks_left = 10;
    TCCR2_Backup = TCCR2;
    TCCR2 = ((TCCR2 & 0b11111000) | 4);
  }
}
// Timer 0 overflow interrupt routine.
ISR(TIMER0_OVF_vect)
{

  static uint8_t key1_down = 0;
  static uint8_t key2_down = 0;
  static uint8_t key3_down = 0;

  static uint8_t brightness = 0;
  static uint8_t brightness_levels[4] =
    { 0, 3, 7, 15 }; //brightness levels

  static uint8_t update_prescaler = 1;
  static uint8_t update_prescalers[6] =
    { 2, 3, 4, 5, 6, 7 };

  //Poll keys
  switch(key1 + (key2 << 1) + (key3 << 2))
  {
    case 1: //Key 1 is pressed
    {
      if(!key1_down)
      {
        key1_down = 1;

        ResetGameOfLife();

      }
      break;
    }
    case 2: //key 2 is pressed
    {
      if(!key2_down)
      {
        key2_down = 1;
        if(!interim_blinks_left)
        {
          TCCR2 = ((TCCR2 & 0b11111000)
              | update_prescalers[update_prescaler++ % 6]);
        }
      }
      break;
    }
    case 4: //key 3 is pressed
    {
      if(!key3_down)
      {
        key3_down = 1;
        HTbrightness(brightness_levels[brightness++ % 4]);
      }
      break;
    }
  }

  //If keys are not pressed right now, clear the key_down flags
  if(!key1) key1_down = 0;
  if(!key2) key2_down = 0;
  if(!key3) key3_down = 0;

  //Update LED array with led values
  HTsendscreen();
}

// Timer 2 overflow interrupt routine.
ISR(TIMER2_OVF_vect)
{
  UpdateLife();
}

int main(void)
{ //==================================================================== main ==================

  HTpinsetup(); //Setup pins for controlling the HT1632
  HTsetup(); //Setup the initial configuration commands to HT1632
  keysetup(); //Setup pins for keys
  timersetup(); //Setup timers. Timer0 is display update and key poll. Timer2 is Game of Life
  initrand(); //Get random seed from EERPROM. New one is saved in EEPROM for next power cycle.

  while (1)
    ;
  return (0);
} //main
