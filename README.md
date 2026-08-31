# Smart Parking System

A three-bay embedded parking system built on an Arduino Uno, demonstrating interrupt-driven
sense-think-act design for Module 1 (QP4 — Distinction).

## Overview

Three pushbuttons simulate per-bay occupancy sensors (pressed = vehicle present). A green LED
indicates at least one bay is free; a red LED indicates the parking area is full. All sensor
input is handled through a shared Pin Change Interrupt (PCI) vector, and a separate Timer
Interrupt drives an independent periodic status report over Serial.

## Hardware

| Component | Pin |
|---|---|
| Button — Bay 1 | D8 |
| Button — Bay 2 | D9 |
| Button — Bay 3 | D10 |
| Green LED (available) | D5 |
| Red LED (full) | D6 |

Buttons use `INPUT_PULLUP` — wire one leg to its Arduino pin above and the diagonally opposite
leg to GND. No external resistor is needed on the button side. Each LED needs a series resistor
(e.g. 220Ω) between the Arduino pin and the LED anode; the cathode goes to GND.

See `circuit-diagram.png` for the full wiring diagram.

## Interrupt architecture

- **Pin Change Interrupt (PCINT0):** D8, D9 and D10 all sit on PORTB, so they share a single
  interrupt vector, `ISR(PCINT0_vect)`. The ISR identifies which pin changed by XOR-ing the
  current `PINB` register against its previous value, then flags the corresponding bay for
  the main loop to process — no blocking logic runs inside the ISR itself.
- **Timer Interrupt (Timer1):** configured in CTC mode (prescaler /256, `OCR1A = 31249`) to
  fire every exactly 0.5 seconds, completely independent of the button interrupts. Its ISR
  only sets a flag; the periodic status report is printed from the main loop.

## Running the simulation

1. Open the project in [TinkerCad Circuits](https://www.tinkercad.com/).
2. Wire the circuit per the table above (or import `circuit-diagram.png` as reference).
3. Paste the contents of `TaskM1.cpp` into the `sketch.ino` code editor tab.
4. Click **Start Simulation** and open the **Serial Monitor**.
5. Click each button to toggle that bay between EMPTY and OCCUPIED, and watch the LEDs and
   Serial output respond. A full status report prints automatically every 0.5 seconds.

## Files in this repository

- `TaskM1.cpp` — full Arduino sketch
- `circuit-diagram.png` — wiring schematic
- `reflection_report.md` — architecture, interrupt design, and issues encountered (300–500 words)
- `README.md` — this file
