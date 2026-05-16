/* ============================================================
 *  ultrasonic.cpp
 *  Implementation of the HC-SR04 driver declared in ultrasonic.h
 * ============================================================ */
#include "ultrasonic.h"

using namespace std::chrono;

/* ------------------------------------------------------------
 *  Construction
 * ----------------------------------------------------------- */
Ultrasonic::Ultrasonic(PinName trig_pin, PinName echo_pin)
    : _trig(trig_pin, 0),    // TRIG idle LOW
      _echo(echo_pin),       // ECHO as InterruptIn
      _distance_cm(OUT_OF_RANGE_CM),
      _valid(false),
      _measuring(false)
{
    /* Hook the two edge interrupts once; they stay active for the
       whole lifetime of the object. */
    _echo.rise(callback(this, &Ultrasonic::_on_echo_rise));
    _echo.fall(callback(this, &Ultrasonic::_on_echo_fall));
}

/* ------------------------------------------------------------
 *  Public control
 * ----------------------------------------------------------- */
void Ultrasonic::start()
{
    _sampler.attach(callback(this, &Ultrasonic::_send_trigger),
                    milliseconds(SAMPLE_PERIOD_MS));
}

void Ultrasonic::stop()
{
    _sampler.detach();
    _echo_guard.detach();
    _trig = 0;
}

float Ultrasonic::distance_cm() const { return _distance_cm; }
bool  Ultrasonic::valid()       const { return _valid; }

/* ------------------------------------------------------------
 *  ISR : Ticker - emit a 10us trigger pulse and arm the timeout
 *  Busy-waiting for 10us inside an ISR is acceptable; the HC-SR04
 *  datasheet requires the pulse to be >= 10us.
 * ----------------------------------------------------------- */
void Ultrasonic::_send_trigger()
{
    _measuring = false;          // any in-flight measurement is stale
    _timer.stop();
    _timer.reset();

    _trig = 1;
    wait_us(10);
    _trig = 0;

    _echo_guard.attach(callback(this, &Ultrasonic::_on_echo_timeout),
                       milliseconds(ECHO_TIMEOUT_MS));
}

/* ------------------------------------------------------------
 *  ISR : ECHO rising edge - the sound burst has just left,
 *  start the high-resolution timer.
 * ----------------------------------------------------------- */
void Ultrasonic::_on_echo_rise()
{
    _timer.reset();
    _timer.start();
    _measuring = true;
}

/* ------------------------------------------------------------
 *  ISR : ECHO falling edge - the burst has returned, compute
 *  the round-trip distance.
 * ----------------------------------------------------------- */
void Ultrasonic::_on_echo_fall()
{
    if (!_measuring) return;     // spurious edge, ignore

    _timer.stop();
    _echo_guard.detach();

    const auto us = duration_cast<microseconds>(_timer.elapsed_time()).count();
    const float cm = static_cast<float>(us) / US_PER_CM;

    if (cm > 2.0f && cm < OUT_OF_RANGE_CM) {
        _distance_cm = cm;
        _valid       = true;
    } else {
        _valid = false;
    }
    _measuring = false;
}

/* ------------------------------------------------------------
 *  ISR : echo never came back -> target is out of range
 * ----------------------------------------------------------- */
void Ultrasonic::_on_echo_timeout()
{
    _timer.stop();
    _measuring   = false;
    _valid       = false;
    _distance_cm = OUT_OF_RANGE_CM;
}
