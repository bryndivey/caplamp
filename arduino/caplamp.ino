/*
  zero crossing on PB3
  ssr out on PB0
  cap in on PB4
*/

#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/delay.h"

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

#define STEPS 30
// this is in ZC cycles
#define CAP_DELAY 4

// 200 for copper, 300 for lamp
#define CAP_LEVEL 30
#define CAP_READS 8
#define TAP_COUNT 3
#define BASELINE_READINGS 10

volatile int dim_level = 90;
volatile int pretrigger = 0;
volatile int should_measure = 0;
volatile int cycle_count = CAP_DELAY;
unsigned int baseline = 0;

void debug_pulse(int us) {
  PORT |= (1 << DEBUG);
  _delay_us(us * 10);
  PORT &= ~(1 << DEBUG);
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

/*
 * Dimmer interrupt - turn on SSR
 */
ISR(TIMER0_COMPA_vect) {
  if(pretrigger == 1) {
    pretrigger = 2;
    PORT |= (1 << SSR);
    TCNT0 = 0;
    OCR0A = 4;
  } else if(pretrigger == 2) {
    pretrigger = 0;
    PORT &= ~(1<< SSR);
    TCNT0 = 0;
    OCR0A = 255;
    TIMSK &= ~(1 << OCIE0A);
  }
}

/*
 * On zero crossing, reset the dimmer timer.
 */
ISR(PCINT0_vect) {
  if(cycle_count == 1) {
    should_measure = 1;
  } else if(cycle_count == 0) {
    cycle_count = CAP_DELAY;
  }
  // decrease cycle count and check if we should measure
  cycle_count--;
  
  // set up the timer for the dimming
  pretrigger = 1;
  TIFR |= (1 << OCF0A);
  TCNT0 = 0;
  OCR0A = dim_level;
  TIMSK |= (1 << OCIE0A);
}

void setup() {
  // get a baseline reading to compare the cap reading to

  for(int i=0; i<BASELINE_READINGS; i++) {
    baseline += touch_measure();
    _delay_ms(10);
  }

  baseline /= BASELINE_READINGS;

  DDR |= (1 << SSR) | (1 << DEBUG);

  cli();

  // pin change interrupts
  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PCINT3);

  // timer interrupt for dim trigger
  TCCR0A = 0;
  TCCR0B = 0;

  // turn on CTC mode
  TCCR0A |= (1 << WGM01);

  // 64 prescalar for 1MHz
  TCCR0B |= (1 << CS01) | (1 << CS00);

  // capsense
  // VCC reference
  ADMUX &= ~((1 << REFS1) | (1 << REFS0));
  // enable, prescalar is 8 
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
  
  sei();

}


int consecutive_touches = 0;
int fade_dir = 1;
int level = 0;

void loop() {
  if(should_measure) {
    should_measure = 0;

#ifndef CYCLE
    int cap = touch_measure() - baseline;
#else
    int cap = CAP_LEVEL + 1;
#endif

    if(cap > CAP_LEVEL) {
      level = level + fade_dir;
      if(consecutive_touches < TAP_COUNT) {
	consecutive_touches ++;
      }

      if(level >= STEPS || level <= 0) {
	fade_dir = -fade_dir;
      }
      dim_level = map(level, 0, STEPS, 110, 20);
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

