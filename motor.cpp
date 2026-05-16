/* ============================================================
 *  motor.cpp
 *  Implementation of the differential-drive H-bridge controller.
 * ============================================================ */
#include "motor.h"

Motor::Motor(PinName ena, PinName in1, PinName in2,
             PinName enb, PinName in3, PinName in4)
    : _ena(ena), _enb(enb),
      _in1(in1, 0), _in2(in2, 0),
      _in3(in3, 0), _in4(in4, 0)
{
    _ena.period_us(PWM_PERIOD_US);
    _enb.period_us(PWM_PERIOD_US);
    stop();
}

/* ------------------------------------------------------------
 *  Public commands
 * ----------------------------------------------------------- */
void Motor::forward(float speed)
{
    _set_left ( +1, speed);
    _set_right( +1, speed);
}

void Motor::reverse(float speed)
{
    _set_left ( -1, speed);
    _set_right( -1, speed);
}

void Motor::pivot_left(float speed)
{
    _set_left ( -1, speed);
    _set_right( +1, speed);
}

void Motor::pivot_right(float speed)
{
    _set_left ( +1, speed);
    _set_right( -1, speed);
}

void Motor::stop()
{
    _set_left (0, 0.0f);
    _set_right(0, 0.0f);
}

/* ------------------------------------------------------------
 *  Helpers
 * ----------------------------------------------------------- */
float Motor::_clamp(float v)
{
    if (v < 0.0f)     return 0.0f;
    if (v > MAX_DUTY) return MAX_DUTY;
    return v;
}

void Motor::_set_left(int dir, float speed)
{
    if (dir > 0)      { _in1 = 1; _in2 = 0; }
    else if (dir < 0) { _in1 = 0; _in2 = 1; }
    else              { _in1 = 0; _in2 = 0; }
    _ena.write(_clamp(speed));
}

void Motor::_set_right(int dir, float speed)
{
    if (dir > 0)      { _in3 = 1; _in4 = 0; }
    else if (dir < 0) { _in3 = 0; _in4 = 1; }
    else              { _in3 = 0; _in4 = 0; }
    _enb.write(_clamp(speed));
}
