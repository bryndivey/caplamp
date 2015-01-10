/*
  zero crossing on pin 2
  ssr out on pin 3
  cap in on pin 4
*/

#import "avr/io.h"
#import "avr/interrupt.h"

#define ZC_PIN 2
#define SSR_PIN 3
#define CAP_PIN 4

#define STEPS 30
// this is in ZC cycles
#define CAP_DELAY 4


// 50Hz cycles are 312.5 counts, so timer will be triggered by ZC before this
#define INF 250

volatile int dim_level = 0;
volatile int pretrigger = 0;
volatile int should_measure = 0;
volatile int cycle_count = 0;

uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  //delay(1);
  uint8_t SREG_old = SREG; //back up the AVR Status Register
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;
  
  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}
  
  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
  SREG = SREG_old;
  
  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  
  return cycles;
}

/*
 * Dimmer interrupt - turn on SSR
 */
ISR(TIMER0_COMPA_vect) {
  volatile uint8_t* ssr_port = portOutputRegister(digitalPinToPort(SSR_PIN));
  int ssr_pin = digitalPinToBitMask(SSR_PIN);

  if(pretrigger == 1) {
    *ssr_port |= ssr_pin;
    pretrigger = 2;
    OCR0A = 5;
    TCNT0 = 0;
  } else if(pretrigger == 2) {
    // turn off triac, ZC will setup timer again
    *ssr_port &= ~(ssr_pin);
    pretrigger = 0;
    OCR0A = INF;
    TCNT0 = 0;
  }
}

/*
 * On zero crossing, reset the dimmer timer.
 */
void zc_interrupt() {
  // increase cycle count and check if we should measure
  cycle_count++;
  
  if(cycle_count == CAP_DELAY) {
    cycle_count = 0;
    should_measure = 1;
  }
  
  // set up the timer for the dimming
  pretrigger = 1;
  TCNT0 = 0;
  OCR0A = dim_level;
}

void setup() {
  Serial.begin(9600);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(5, OUTPUT);
  attachInterrupt(0, zc_interrupt, CHANGE);

  cli();
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;

  OCR0A = INF;

  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // 1024 prescalar (312.5 steps per 50Hz cycle)
  TCCR0B |= (1 << CS02) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
  sei();
}


int consecutive_touches = 0;
int fade_dir = 1;
int level = 0;

void loop() {
  if(should_measure) {
    Serial.print("Measured ");
    Serial.println(dim_level);
    should_measure = 0;
    int cap = readCapacitivePin(CAP_PIN);
    
    if(cap > 1) {
      level = level + fade_dir;
      if(consecutive_touches < 20) {
	consecutive_touches ++;
      }

      if(level >= STEPS || level <= 0) {
	fade_dir = -fade_dir;
      }
      dim_level = map(level, 0, STEPS, 120, 50);
    } else {
      // tap turns off
      if(consecutive_touches > 0 && consecutive_touches < 3) {
	level = 0;
	dim_level = 0;
	fade_dir = 1;
      }
      consecutive_touches = 0;
    }
  }
}

