/*
 * ================================================================
 * SMART PARKING SYSTEM
 * Arduino Uno / ATmega328P
 *
 * Pushbutton "sensors" (simulate a vehicle in each bay)
 *
 * Bay 1: Button -> D8  (PCINT0)
 * Bay 2: Button -> D9  (PCINT1)
 * Bay 3: Button -> D10 (PCINT2)
 *
 * Each button uses INPUT_PULLUP - no external resistor needed.
 * Wire one leg of each button to its pin above, and the diagonally
 * opposite leg to GND.
 *
 * Pressed  = OCCUPIED (pin reads LOW)
 * Released = EMPTY    (pin reads HIGH)
 *
 * LEDs:
 *   Green -> D5
 *   Red   -> D6
 *
 * ================================================================
 * INTERRUPT REQUIREMENTS
 *
 * PCI:
 *   D8, D9 and D10 belong to PORTB.
 *   Therefore they share PCINT0_vect.
 *
 * Timer:
 *   Timer1 generates an interrupt every 500 ms, used purely for the
 *   periodic status report - fully independent of the PCI side.
 *
 * Important:
 *   No delay() is used anywhere.
 *   ISRs are short and non-blocking.
 * ================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>


// ================================================================
// PIN CONFIGURATION
// ================================================================

const byte NUM_BAYS = 3;

const byte BUTTON_PIN[NUM_BAYS] = {8, 9, 10};   // PORTB - shared PCINT0 group

const byte GREEN_LED = 5;
const byte RED_LED   = 6;

// Bits corresponding to D8, D9 and D10 on PORTB
const byte SENSOR_BIT[NUM_BAYS] = {
  PCINT0,   // D8
  PCINT1,   // D9
  PCINT2    // D10
};


// ================================================================
// PARKING STATES
// ================================================================

enum BayState
{
  BAY_EMPTY,
  BAY_OCCUPIED
};

enum ParkingState
{
  PARKING_AVAILABLE,
  PARKING_FULL
};

BayState bayState[NUM_BAYS] =
{
  BAY_EMPTY,
  BAY_EMPTY,
  BAY_EMPTY
};

ParkingState parkingState = PARKING_AVAILABLE;


// ================================================================
// PCI VARIABLES
// ================================================================

// Previous PORTB state
volatile byte lastPortB = 0;

// Set true by the ISR whenever a button's pin changes;
// cleared once the main loop has processed it
volatile bool buttonEventPending[NUM_BAYS] =
{
  false,
  false,
  false
};


// ================================================================
// TIMER VARIABLES
// ================================================================

// Set by Timer1 ISR
volatile bool timerFlag = false;


// ================================================================
// FUNCTION: CONFIGURE PCI
// ================================================================

void configurePCI()
{
  /*
   * D8, D9 and D10 are configured as inputs with the internal
   * pull-up enabled, so each button only needs one leg wired
   * to GND - no external resistor required.
   *
   * All three pins are located on PORTB:
   *
   * D8  = PB0 = PCINT0
   * D9  = PB1 = PCINT1
   * D10 = PB2 = PCINT2
   *
   * These three pins therefore share the same PCINT0_vect
   * interrupt vector.
   */

  pinMode(BUTTON_PIN[0], INPUT_PULLUP);
  pinMode(BUTTON_PIN[1], INPUT_PULLUP);
  pinMode(BUTTON_PIN[2], INPUT_PULLUP);

  /*
   * Enable Pin Change Interrupt group 0.
   */
  PCICR |= (1 << PCIE0);

  /*
   * Enable D8, D9 and D10 individually.
   */
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);

  /*
   * Store the initial PORTB state.
   */
  lastPortB = PINB;
}


// ================================================================
// PCI INTERRUPT SERVICE ROUTINE
// ================================================================

ISR(PCINT0_vect)
{
  /*
   * Read current state of PORTB.
   */
  byte currentPortB = PINB;

  /*
   * XOR detects which pin(s) changed since last time.
   *
   * Example:
   *
   * lastPortB = 00000001
   * current   = 00000011
   * changed   = 00000010   -> D9 changed
   */
  byte changedPins = currentPortB ^ lastPortB;

  /*
   * Flag any button whose pin changed. No level reading, no
   * processing here - that happens safely in the main loop.
   */
  for (byte i = 0; i < NUM_BAYS; i++)
  {
    byte mask = (1 << SENSOR_BIT[i]);

    if (changedPins & mask)
    {
      buttonEventPending[i] = true;
    }
  }

  /*
   * Save current state for next interrupt.
   */
  lastPortB = currentPortB;
}


// ================================================================
// FUNCTION: CONFIGURE TIMER1
// ================================================================

void configureTimer1()
{
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  /*
   * Arduino Uno clock = 16 MHz
   * Prescaler = 256
   * 16,000,000 / 256 = 62,500 Hz
   * For 500 ms: 62,500 x 0.5 = 31,250 -> OCR1A = 31,249
   */
  OCR1A = 31249;

  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // prescaler /256

  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}


// ================================================================
// TIMER1 INTERRUPT SERVICE ROUTINE
// ================================================================

ISR(TIMER1_COMPA_vect)
{
  /*
   * Timer ISR only sets a flag - no Serial.print(), no heavy
   * work. The main loop handles the actual periodic task.
   */
  timerFlag = true;
}


// ================================================================
// FUNCTION: PROCESS SENSOR (BUTTON) EVENTS
// ================================================================

void processSensorEvents()
{
  for (byte i = 0; i < NUM_BAYS; i++)
  {
    if (buttonEventPending[i])
    {
      noInterrupts();
      buttonEventPending[i] = false;
      interrupts();

      /*
       * INPUT_PULLUP: LOW = pressed = OCCUPIED
       *               HIGH = released = EMPTY
       */
      bool pressed = (digitalRead(BUTTON_PIN[i]) == LOW);

      BayState newState = pressed ? BAY_OCCUPIED : BAY_EMPTY;

      if (newState != bayState[i])
      {
        bayState[i] = newState;

        Serial.print("Bay ");
        Serial.print(i + 1);
        Serial.println(newState == BAY_OCCUPIED ? " -> OCCUPIED" : " -> EMPTY");
      }
    }
  }
}


// ================================================================
// FUNCTION: UPDATE PARKING STATE
// ================================================================

void updateParkingState()
{
  byte availableSpaces = 0;

  for (byte i = 0; i < NUM_BAYS; i++)
  {
    if (bayState[i] == BAY_EMPTY)
    {
      availableSpaces++;
    }
  }

  parkingState = (availableSpaces == 0) ? PARKING_FULL : PARKING_AVAILABLE;
}


// ================================================================
// FUNCTION: UPDATE LEDS
// ================================================================

void updateLEDs()
{
  if (parkingState == PARKING_FULL)
  {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
  else
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
}


// ================================================================
// FUNCTION: PROCESS TIMER EVENT
// ================================================================

void processTimerEvent()
{
  if (!timerFlag)
  {
    return;
  }

  noInterrupts();
  timerFlag = false;
  interrupts();

  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("PERIODIC PARKING STATUS");
  Serial.println("--------------------------------");

  byte availableSpaces = 0;

  for (byte i = 0; i < NUM_BAYS; i++)
  {
    Serial.print("Bay ");
    Serial.print(i + 1);
    Serial.print(": ");

    if (bayState[i] == BAY_OCCUPIED)
    {
      Serial.println("OCCUPIED");
    }
    else
    {
      Serial.println("EMPTY");
      availableSpaces++;
    }
  }

  Serial.print("Available spaces: ");
  Serial.println(availableSpaces);

  Serial.println(parkingState == PARKING_FULL ? "Parking Status: FULL" : "Parking Status: AVAILABLE");
  Serial.println("--------------------------------");
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(9600);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  configurePCI();
  configureTimer1();

  updateParkingState();
  updateLEDs();

  Serial.println();
  Serial.println("================================");
  Serial.println("     SMART PARKING SYSTEM");
  Serial.println("================================");
  Serial.println("Bay 1: Button D8");
  Serial.println("Bay 2: Button D9");
  Serial.println("Bay 3: Button D10");
  Serial.println("--------------------------------");
  Serial.println("PCI Group: PCINT0");
  Serial.println("PCI Pins: D8, D9, D10");
  Serial.println("Timer1 Period: 500 ms");
  Serial.println("--------------------------------");
  Serial.println("System Ready.");
  Serial.println("================================");
}


// ================================================================
// MAIN LOOP
// ================================================================

void loop()
{
  processSensorEvents();   // event-driven side (PCINT - button presses)
  updateParkingState();    // grouped state logic across all 3 bays
  updateLEDs();             // conditional response

  processTimerEvent();      // time-driven side (Timer1) - fully independent
}
