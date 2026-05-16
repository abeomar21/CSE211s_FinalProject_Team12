# CSE211_T12FinalProject — Object-Following Robot Car

Mbed OS 6 project for an STM32 Nucleo board (e.g. F401RE) that
uses an HC-SR04 ultrasonic sensor to follow an object moving in
a straight line and stop safely when the object stops.

## How to build (Mbed Studio)
1. *File → New Program → Pristine Mbed OS program* (name it
   whatever you like) so Mbed Studio fetches `mbed-os`.
2. Drop these four files into the project root:
   `main.cpp`, `ultrasonic.h`, `ultrasonic.cpp`,
   `motor.h`, `motor.cpp`.
3. Select your target board and `Build & Run`.

## Wiring (NUCLEO-F401RE MB1136 rev C - labels as printed on the right header)

| Function          | Silkscreen label | Notes                      |
|-------------------|------------------|----------------------------|
| HC-SR04 TRIG      | `D8`             | digital out                |
| HC-SR04 ECHO      | `D2`             | EXTI10, via 5V->3.3V divider |
| Left motor PWM    | `PWM/D5`         | TIM3_CH1                   |
| Left motor IN1    | `D4`             | digital out                |
| Left motor IN2    | `D7`             | digital out                |
| Right motor PWM   | `PWM/D6`         | TIM2_CH3                   |
| Right motor IN3   | `PWM/D9`         | digital out (PWM not used) |
| Right motor IN4   | `PWM/CS/D10`     | digital out (PWM not used) |
| GND               | `GND`            | sensor & driver GND        |
| 5 V (sensor)      | `5V`             | HC-SR04 VCC                |

> **ECHO is 5 V, the Nucleo GPIO is 3.3 V.**  Use a simple
> resistor divider (1 kΩ + 2 kΩ) on the ECHO line.

The L298N gets its motor supply (`VS`) from the battery pack;
its logic 5 V can either come from the on-board regulator
(jumper ON) or from the Nucleo's 5 V.

## File overview

| File              | Role                                       |
|-------------------|--------------------------------------------|
| `main.cpp`        | startup, controller loop, telemetry        |
| `ultrasonic.h/cpp`| HC-SR04 driver — ISR-based, non-blocking   |
| `motor.h/cpp`     | L298N differential-drive driver            |

## Tuning

All behavioural parameters live at the top of `main.cpp`:

```
TARGET_CM  = 20 cm  // distance to hold
DEAD_BAND  =  3 cm  // tolerance around the target
MAX_RANGE  = 60 cm  // ignore objects past this
K_P        = 0.035  // proportional gain
MIN_SPEED  = 0.40   // PWM duty needed to start moving
MAX_SPEED  = 0.80   // upper duty limit
```

If the car oscillates around the target, lower `K_P`.
If it cannot start moving, raise `MIN_SPEED`.
