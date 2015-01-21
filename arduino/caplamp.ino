/*
  zero crossing on pin 2
  ssr out on pin 3
  cap in on pin 4
*/

#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/delay.h"

#define ATTINY 1

#ifdef ATTINY

#include <TinyDebugSerial.h>
TinyDebugSerial mySerial = TinyDebugSerial();

#define SERIAL mySerial

#define PORT PORTB
#define PIN PINB
#define DDR DDRB

#define ADC_CLEAR 0b1111
#define ADC_GND 0b1101

#define CAP 4
#define CAP_DDR DDRB
#define CAP_PORT PORTB
#define CAP_CHAN 2
#define DEBUG 1
#define SSR 0

#else

#define SERIAL Serial

#define PORT PORTD
#define PIN PIND
#define DDR DDRD

#define ADC_CLEAR 0b1111
#define ADC_GND 0b1111

#define CAP 2
#define CAP_DDR DDRC
#define CAP_PORT PORTC
#define CAP_CHAN 2
#define DEBUG 6
#define SSR 3

#endif

#define ZC 2

#define STEPS 30
// this is in ZC cycles
#define CAP_DELAY 4
#define CAP_LEVEL 200
#define CAP_READS 4
#define TAP_COUNT 3

volatile int dim_level = 90;
volatile int pretrigger = 0;
volatile int should_measure = 0;
volatile int cycle_count = 0;

void debug_pulse(int us) {
  PORT |= (1 << DEBUG);
  _delay_us(us * 10);
  PORT &= ~(1 << DEBUG);
}

void debug_number(int num) {
  for(int i=15; i>=0; i--) {
    if(num & (1 << i)) {
      debug_pulse(5);
    } else {
      debug_pulse(1);
    }
  }
}

static inline void adc_channel(uint8_t channel){
  ADMUX &= ~(ADC_CLEAR);
  ADMUX |=   0b1111 & channel;
}

static inline uint16_t adc_get(void){
  ADCSRA |= (1 << ADSC); //start conversion
  loop_until_bit_is_clear(ADCSRA, ADSC); // wait for finish
  return ADC; //return value
}

uint16_t touch_measure(){
  uint8_t i;
  uint16_t retval;
  
  retval = 0;
 
  for (i=0 ; i<CAP_READS ; i++){
    CAP_PORT |= (1 << CAP); // pullup on
    _delay_ms(1);
    CAP_PORT &= ~(1 << CAP); // pullup off
    
    adc_channel(ADC_GND); //set ADC mux to ground;
    adc_get();            //do a measurement (to discharge the sampling cap)
    
    adc_channel(CAP_CHAN); //set mux to right channel
    retval +=  adc_get(); //do a conversion
  }
  return retval / CAP_READS;
}

static inline void enable_int0(volatile int delay) {
  // clear flag
}

static inline void disable_int0() {
}

/*
 * Dimmer interrupt - turn on SSR
 */
ISR(TIMER0_COMPA_vect) {
  debug_pulse(1);
  if(pretrigger == 1) {
    pretrigger = 2;
    PORT |= (1 << SSR);
    // retrigger for turn off
    TCNT0 = 0;
    OCR0A = 255;
#ifdef ATTINY
    TIMSK &= ~(1 << OCIE0A);
#else
    TIMSK0 &= ~(1 << OCIE0A);
#endif
  }
}

/*
 * On zero crossing, reset the dimmer timer.
 */
ISR(INT0_vect) {
  // increase cycle count and check if we should measure
  cycle_count++;

  if(cycle_count == CAP_DELAY) {
    cycle_count = 0;
    should_measure = 1;
  }
  
  // set up the timer for the dimming
  pretrigger = 1;
  TIFR |= (1 << OCF0A);
  TCNT0 = 0;
  OCR0A = dim_level;
  #ifdef ATTINY
  TIMSK |= (1 << OCIE0A);
  #else
  TIMSK0 |= (1 << OCIE0A);
  #endif
}

void setup() {
  SERIAL.begin(9600);

  DDR |= (1 << SSR) | (1 << DEBUG);

  cli();

  // INT0 change interrupt for ZC detection
  #ifdef ATTINY
  MCUCR |= (1 << ISC00);
  GIMSK |= (1 << INT0);
  #else
  EICRA |= (1 << ISC00);
  EIMSK |= (1 << INT0);
  #endif

  // timer interrupt for dim trigger
  TCCR0A = 0;
  TCCR0B = 0;

  // turn on CTC mode
  TCCR0A |= (1 << WGM01);

  #ifdef ATTINY
  // 64 prescalar for 1MHz
  TCCR0B |= (1 << CS01) | (1 << CS00);
  #else
  // 256 for 8Mhz
  TCCR0B |= (1 << CS02) | (1 << CS00);
  #endif


  // capsense
  #ifdef ATTINY
  // VCC reference
  ADMUX &= ~((1 << REFS1) | (1 << REFS0));
  // enable, prescalar is 8 
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
  #else
  // VCC ref
  ADMUX |= (1 << REFS0);
  // enable, 64 prescale
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
  #endif
  
  sei();
}


int consecutive_touches = 0;
int fade_dir = 1;
int level = 0;

void loop() {
  if(pretrigger == 2) {
    // turn off triac, ZC will setup timer again
    pretrigger = 0;
    PORT &= ~(1 << SSR);
  }
  
  if(should_measure) {
    should_measure = 0;

    int cap = touch_measure();
    
    if(cap > CAP_LEVEL) {
      level = level + fade_dir;
      if(consecutive_touches < TAP_COUNT) {
	consecutive_touches ++;
      }

      if(level >= STEPS || level <= 0) {
	fade_dir = -fade_dir;
      }
      dim_level = map(level, 0, STEPS, 90, 20);
    } else {
      // tap turns off
      if(consecutive_touches > 0 && consecutive_touches < TAP_COUNT) {
	level = 0;
	dim_level = 0;
	fade_dir = 1;
      }
      consecutive_touches = 0;
    }
  }
}

