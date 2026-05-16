/* ============================================================
 *  ultrasonic.h
 *  Non-blocking HC-SR04 driver for Mbed OS 6.
 *
 *  - A Ticker fires every SAMPLE_PERIOD_MS and sends a 10us
 *    trigger pulse on the TRIG pin.
 *  - The ECHO pin is wired to an InterruptIn. The rising edge
 *    starts a Timer; the falling edge stops it and converts the
 *    elapsed microseconds into a distance in centimetres.
 *  - A Timeout guards against a missing echo (object out of
 *    range): if no falling edge arrives within ECHO_TIMEOUT_MS
 *    the measurement is marked invalid.
 * ============================================================ */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "mbed.h"

class Ultrasonic {
public:
    Ultrasonic(PinName trig_pin, PinName echo_pin);

    /* Start / stop the periodic measurement cycle. */
    void  start();
    void  stop();

    /* Latest readings (safe to call from the main thread). */
    float distance_cm() const;   // last measured distance
    bool  valid()       const;   // false if last measurement timed out

private:
    /* --- HC-SR04 specs -------------------------------------- */
    static constexpr uint32_t SAMPLE_PERIOD_MS = 60;   // >= 50 ms between pulses
    static constexpr uint32_t ECHO_TIMEOUT_MS  = 30;   // ~4 m round-trip max
    static constexpr float    US_PER_CM        = 58.0f;// (2 cm)/(343 m/s)
    static constexpr float    OUT_OF_RANGE_CM  = 400.0f;

    /* --- I/O & timing --------------------------------------- */
    DigitalOut   _trig;
    InterruptIn  _echo;
    Timer        _timer;
    Ticker       _sampler;
    Timeout      _echo_guard;

    /* --- shared state (written in ISR, read in main) -------- */
    volatile float _distance_cm;
    volatile bool  _valid;
    volatile bool  _measuring;

    /* --- ISRs ----------------------------------------------- */
    void _send_trigger();   // Ticker  ISR : start a measurement
    void _on_echo_rise();   // EXTI    ISR : echo went HIGH
    void _on_echo_fall();   // EXTI    ISR : echo went LOW
    void _on_echo_timeout();// Timeout ISR : no echo received
};

#endif /* ULTRASONIC_H */
