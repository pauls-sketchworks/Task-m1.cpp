# Smart Parking System

A three-bay embedded parking system built on an Arduino Uno, demonstrating interrupt-driven
sense-think-act design for Module 1 (QP4 — Distinction).

## Overview

Three 3-pin ultrasonic distance sensors (PING)))-style: GND, 5V, SIG) measure distance in each
bay — a bay is marked OCCUPIED when its reading falls between 50 cm and 300 cm, and EMPTY
otherwise. Each sensor uses a single shared pin for both the trigger pulse and the echo,
switched in software between OUTPUT (sending the pulse) and INPUT (timing the echo). A green
LED indicates at least one bay is free; a red LED indicates the parking area is full. All
sensor input is handled through a shared Pin Change Interrupt (PCI) vector, and a separate
Timer Interrupt drives an independent periodic status report and triggers the next sensor scan.

## Hardware

| Component | Pin |
|---|---|
| Sensor 1 — SIG | D8 |
| Sensor 2 — SIG | D9 |
| Sensor 3 — SIG | D10 |
| Green LED (available) | D5 |
| Red LED (full) | D6 |

Each sensor needs 3 connections: `GND`→GND, `5V`→5V, `SIG`→its pin above. Each LED needs a
series resistor (e.g. 220Ω) between the Arduino pin and the LED anode; the cathode goes to GND.

See `circuit-diagram.png` for the full wiring diagram.

## Interrupt architecture

- **Pin Change Interrupt (PCINT0):** the three sensors' SIG pins (D8, D9, D10) all sit on
  PORTB, so they share a single interrupt vector, `ISR(PCINT0_vect)`. Since each sensor uses
  one pin for both trigger and echo, that pin's interrupt mask bit is disabled while the code
  is driving it (sending the trigger pulse) and re-enabled the instant it's switched back to
  INPUT, so only genuine sensor-driven edges are timed — the ISR identifies which pin changed
  by XOR-ing the current `PINB` register against its previous value, then timestamps the
  rising edge (echo start) or computes the pulse duration on the matching falling edge (echo
  end), with no `pulseIn()` and no blocking.
- **Timer Interrupt (Timer1):** configured in CTC mode (prescaler /256, `OCR1A = 31249`) to
  fire every exactly 0.5 seconds, completely independent of the PCINT side. Its ISR only sets
  a flag; the main loop uses that flag to print the periodic status report and start the next
  non-blocking sensor scan (sensors are triggered one at a time, 30 ms apart, to avoid
  acoustic cross-talk).

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
