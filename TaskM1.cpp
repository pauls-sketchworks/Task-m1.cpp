/*
 * ================================================================
 * SMART PARKING SYSTEM
 * Arduino Uno / ATmega328P
 *
 * Ultrasonic distance sensors (3-pin, PING)))-style: GND, 5V, SIG)
 *
 * Bay 1: SIG -> D8  (PCINT0)
 * Bay 2: SIG -> D9  (PCINT1)
 * Bay 3: SIG -> D10 (PCINT2)
 *
 * This sensor has ONE signal pin used for both the trigger pulse
 * AND the echo pulse, unlike the HC-SR04 (separate TRIG/ECHO pins).
 * We switch that pin's mode in software:
 *
 *   1. pinMode(pin, OUTPUT) - we briefly drive the pulse ourselves
 *   2. pinMode(pin, INPUT)  - the sensor then drives the same pin
 *                             back to time the echo
 *
 * While WE are driving the pin (step 1), its PCINT mask bit is
 * disabled so our own trigger pulse doesn't get mistaken for an
 * echo edge. The instant we switch back to INPUT (step 2), we
 * resync our "last known" copy of that bit and re-enable its
 * interrupt, so only genuine sensor-driven edges are timed.
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
 *   Timer1 generates an interrupt every 500 ms.
 *
 * Important:
 *   No delay() or delayMicroseconds() is used.
 *   ISRs are short and non-blocking.
 * ================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>


// ================================================================
// DEBUG
// ================================================================
// Set to true while calibrating OCCUPIED_DISTANCE against whatever
// range your simulator/hardware actually supports. Set to false
// for your final submission to keep Serial output clean.
const bool DEBUG_MODE = true;


// ================================================================
// PIN CONFIGURATION
// ================================================================

const byte NUM_BAYS = 3;

const byte SIG_PIN[NUM_BAYS] = {8, 9, 10};   // PORTB - shared PCINT0 group

const byte GREEN_LED = 5;
const byte RED_LED   = 6;

// Bits corresponding to D8, D9 and D10 on PORTB
const byte SENSOR_BIT[NUM_BAYS] = {
  PCINT0,   // D8
  PCINT1,   // D9
  PCINT2    // D10
};


// ================================================================
// PARKING SETTINGS
// ================================================================

const float OCCUPIED_MIN_DISTANCE = 50.0;    // cm - closer than this = not a valid reading (ignore)
const float OCCUPIED_MAX_DISTANCE = 300.0;   // cm - a bay is OCCUPIED between MIN and MAX


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

// Echo timing
volatile unsigned long echoStart[NUM_BAYS] =
{
  0, 0, 0
};

volatile unsigned long echoDuration[NUM_BAYS] =
{
  0, 0, 0
};

// Indicates that a complete echo measurement is ready
volatile bool echoReady[NUM_BAYS] =
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
// NON-BLOCKING SENSOR TRIGGER STATE
// ================================================================

enum TriggerStage
{
  STAGE_IDLE,     // not currently pulsing this sensor
  STAGE_SETTLE,   // pin held LOW briefly before the pulse
  STAGE_PULSE     // pin held HIGH for the trigger pulse itself
};

bool scanActive = false;
byte currentSensor = 0;

TriggerStage triggerStage = STAGE_IDLE;

unsigned long stageEndTime = 0;
unsigned long nextSensorTime = 0;


// ================================================================
// FUNCTION: CONFIGURE PCI
// ================================================================

void configurePCI()
{
  /*
   * D8, D9 and D10 start out as inputs. They will be switched to
   * OUTPUT briefly (and back) each time their sensor is triggered.
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

  pinMode(SIG_PIN[0], INPUT);
  pinMode(SIG_PIN[1], INPUT);
  pinMode(SIG_PIN[2], INPUT);

  PCICR |= (1 << PCIE0);

  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);

  lastPortB = PINB;
}


// ================================================================
// PCI INTERRUPT SERVICE ROUTINE
// ================================================================

ISR(PCINT0_vect)
{
  byte currentPortB = PINB;
  byte changedPins = currentPortB ^ lastPortB;

  for (byte i = 0; i < NUM_BAYS; i++)
  {
    byte mask = (1 << SENSOR_BIT[i]);

    if (changedPins & mask)
    {
      /*
       * RISING EDGE = echo pulse has started
       * FALLING EDGE = echo pulse has finished
       *
       * Only fires for genuine sensor-driven edges, since the
       * relevant mask bit is disabled while WE are driving the
       * pin during the trigger pulse (see updateSensorTrigger()).
       */
      if (currentPortB & mask)
      {
        echoStart[i] = micros();
      }
      else
      {
        echoDuration[i] = micros() - echoStart[i];
        echoReady[i] = true;
      }
    }
  }

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
  timerFlag = true;   // flag only - main loop does the actual work
}


// ================================================================
// FUNCTION: START SENSOR SCAN
// ================================================================

void startSensorScan()
{
  currentSensor = 0;
  scanActive = true;
  triggerStage = STAGE_IDLE;
  nextSensorTime = micros();   // first sensor can trigger immediately
}


// ================================================================
// FUNCTION: UPDATE SENSOR TRIGGER
// ================================================================
// Non-blocking state machine driving the single shared SIG pin
// through: settle LOW -> pulse HIGH -> hand control back to the
// sensor (switch to INPUT) and let the PCI ISR time the echo.
void updateSensorTrigger()
{
  if (!scanActive)
  {
    return;
  }

  unsigned long now = micros();
  byte pin = SIG_PIN[currentSensor];
  byte bit = SENSOR_BIT[currentSensor];

  switch (triggerStage)
  {
    case STAGE_IDLE:
      if ((long)(now - nextSensorTime) >= 0)
      {
        /*
         * Take over the pin: disable its PCI mask bit first so our
         * own edges aren't mistaken for an echo, THEN drive it.
         */
        noInterrupts();
        PCMSK0 &= ~(1 << bit);
        interrupts();

        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);

        stageEndTime = now + 2;     // brief LOW settle time
        triggerStage = STAGE_SETTLE;
      }
      break;

    case STAGE_SETTLE:
      if ((long)(now - stageEndTime) >= 0)
      {
        digitalWrite(pin, HIGH);
        stageEndTime = now + 5;     // trigger pulse width
        triggerStage = STAGE_PULSE;
      }
      break;

    case STAGE_PULSE:
      if ((long)(now - stageEndTime) >= 0)
      {
        digitalWrite(pin, LOW);
        pinMode(pin, INPUT);        // hand the pin back to the sensor

        /*
         * Resync our "last known" copy of this bit to the pin's
         * actual current level BEFORE re-enabling its interrupt,
         * so the OUTPUT->INPUT handover itself can't be mistaken
         * for an echo edge.
         */
        noInterrupts();
        byte currentPortB = PINB;
        lastPortB = (lastPortB & ~(1 << bit)) | (currentPortB & (1 << bit));
        PCMSK0 |= (1 << bit);
        interrupts();

        triggerStage = STAGE_IDLE;
        nextSensorTime = now + 30000;   // 30 ms gap before next sensor
        currentSensor++;

        if (currentSensor >= NUM_BAYS)
        {
          scanActive = false;
          currentSensor = 0;
        }
      }
      break;
  }
}


// ================================================================
// FUNCTION: PROCESS SENSOR EVENTS
// ================================================================

void processSensorEvents()
{
  for (byte i = 0; i < NUM_BAYS; i++)
  {
    if (echoReady[i])
    {
      unsigned long duration;

      noInterrupts();
      duration = echoDuration[i];
      echoReady[i] = false;
      interrupts();

      /*
       * distance(cm) = duration(us) / 58   (standard round-trip
       * ultrasonic conversion, matches this sensor's datasheet).
       */
      float distance = duration / 58.0;

      if (DEBUG_MODE)
      {
        Serial.print("[debug] Bay ");
        Serial.print(i + 1);
        Serial.print(" raw distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      }

      bool vehicleDetected = (distance >= OCCUPIED_MIN_DISTANCE && distance <= OCCUPIED_MAX_DISTANCE);
      BayState newState = vehicleDetected ? BAY_OCCUPIED : BAY_EMPTY;

      if (newState != bayState[i])
      {
        bayState[i] = newState;

        Serial.print("Bay ");
        Serial.print(i + 1);
        Serial.print(newState == BAY_OCCUPIED ? " -> OCCUPIED" : " -> EMPTY");
        Serial.print(" | Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
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

  if (!scanActive)
  {
    startSensorScan();
  }

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
  Serial.println("Bay 1: SIG D8");
  Serial.println("Bay 2: SIG D9");
  Serial.println("Bay 3: SIG D10");
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
  processSensorEvents();   // event-driven side (PCINT - echo timing)
  updateSensorTrigger();   // non-blocking trigger pulse generator
  updateParkingState();    // grouped state logic across all 3 bays
  updateLEDs();             // conditional response

  processTimerEvent();      // time-driven side (Timer1) - fully independent
}
