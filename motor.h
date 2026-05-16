/* ============================================================
 *  motor.h
 *  Differential-drive controller for a 2-wheel chassis driven
 *  by an L298N / L293D style H-bridge.
 *
 *  Each motor needs 3 lines:
 *      ENx  - PWM, controls speed (0.0 .. 1.0 duty cycle)
 *      INx1 - direction bit 1
 *      INx2 - direction bit 2
 *  (IN_a, IN_b) = (1,0) -> forward, (0,1) -> reverse, (0,0) -> coast.
 * ============================================================ */
#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class Motor {
public:
    Motor(PinName ena, PinName in1, PinName in2,    // left  motor
          PinName enb, PinName in3, PinName in4);   // right motor

    /* High-level commands. speed is 0.0 (stop) .. 1.0 (full). */
    void forward(float speed);
    void reverse(float speed);
    void pivot_left (float speed);   // left wheel back , right wheel fwd
    void pivot_right(float speed);   // left wheel fwd  , right wheel back
    void stop();

private:
    /* PWM frequency for the H-bridge enable pins. */
    static constexpr int  PWM_PERIOD_US = 50;    // 20 kHz - silent, smooth
    static constexpr float MAX_DUTY     = 1.0f;

    PwmOut     _ena, _enb;
    DigitalOut _in1, _in2, _in3, _in4;

    /* Helpers - clamp + drive a single side. */
    static float _clamp(float v);
    void _set_left (int dir, float speed);   // dir: +1 fwd, -1 rev, 0 stop
    void _set_right(int dir, float speed);
};

#endif /* MOTOR_H */
