#define SSR_PIN 3
#define STEPS 2
#define ZK_PIN 2
#define DIM_LEVEL OCR1A
// 50Hz cycles are 312.5 counts, so timer will be triggered by ZC before this
#define INF 65000

volatile int dim_level = 0;
volatile int pretrigger = 0;

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
  delay(1);
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
ISR(TIMER1_COMPA_vect) {
  if(pretrigger == 1) {
    digitalWrite(SSR_PIN, 1);
    pretrigger = 0;
    OCR1A = 1;
    TCNT1 = 0;
  } else {
    // turn off triac, ZC will setup timer again
    digitalWrite(SSR_PIN, 0);
    OCR1A = INF;
    TCNT1 = 0;
  }
}

/*
 * On zero crossing, reset the dimmer timer.
 */
void zc_interrupt() {
  pretrigger = 1;
  if(dim_level < 310) {
    OCR1A = dim_level;
    TCNT1 = 0;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SSR_PIN, OUTPUT);
  attachInterrupt(0, zc_interrupt, CHANGE);

  // triac switch interrupt
  cli();

  // reset
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // set target to infinity
  OCR1A = INF;

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // 1024 prescalar (312.5 steps per 50Hz cycle)
  TCCR1B |= (1 << CS12) | (1 << CS10);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void do_read() {
  int on_readings = 0;
  int cap1 = 0;
  int cap2 = 0;
  while(1) {
    cap1 = readCapacitivePin(2);
    if(cap1 > 1) {
      delay(200);
      cap2 = readCapacitivePin(2);
      if(cap1 == cap2 && cap1 > 1) {
	on_readings = (on_readings + 1) % STEPS;
	//analogWrite(TARGET_PIN, map(on_readings, 0, STEPS, 0, 256));
      }
      delay(200);
    }
  }
}

void loop() {
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    if(inChar == 'a') {
        dim_level = dim_level - 10;
    } else if(inChar == 's') {
        dim_level = dim_level + 10;
    } else if(inChar == 'j') {
        dim_level = dim_level - 1;
    } else if(inChar == 'k')  {
        dim_level = dim_level + 1;
    }
    Serial.println(dim_level);
  }
}

  
