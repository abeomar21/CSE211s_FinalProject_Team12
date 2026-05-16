/* ============================================================
 *  CSE211s - Introduction to Embedded Systems - Spring 2026
 *  Final Project : Object-following robot car
 *
 *  Platform : ST Nucleo (e.g. F401RE / L476RG) + Mbed OS 6
 *  Sensor   : HC-SR04 ultrasonic ranger
 *  Actuator : 2-wheel differential drive + front caster,
 *             driven by an L298N H-bridge
 *
 *  Behaviour
 *  ---------
 *  The car keeps a TARGET_CM distance to the object in front.
 *  A proportional law converts the distance error into a wheel
 *  duty cycle:
 *      - object farther than TARGET + DEAD_BAND  -> drive forward
 *      - object closer  than TARGET - DEAD_BAND  -> drive backward
 *      - inside the dead band                    -> stop (object
 *                                                   is held at a
 *                                                   "suitable
 *                                                   distance")
 *      - object out of range or no echo          -> stop safely
 *
 *  Software structure
 *  ------------------
 *  Startup  : Mbed reset handler -> SystemInit() -> _start()
 *             -> global ctors (Ultrasonic, Motor) -> main()
 *  main()   : configures the controller and loops at 20 Hz,
 *             reading the latest distance and updating the
 *             motors. No blocking calls.
 *  ISRs     : 1) Ticker          -> Ultrasonic::_send_trigger
 *             2) ECHO rising     -> Ultrasonic::_on_echo_rise
 *             3) ECHO falling    -> Ultrasonic::_on_echo_fall
 *             4) Echo Timeout    -> Ultrasonic::_on_echo_timeout
 * ============================================================ */

#include "mbed.h"
#include "ultrasonic.h"
#include "motor.h"

/* ============================================================
 *  Hardware pin map  (NUCLEO-F401RE MB1136 rev C)
 *
 *  Pin names below match the silkscreen labels printed on the
 *  right header of the board (CN5/CN9):
 *      RX/D0   TX/D1   D2   PWM/D3   D4   PWM/D5   PWM/D6   D7
 *      D8   PWM/D9   PWM/CS/D10   PWM/MOSI/D11   ...
 *
 *  Constraints honoured for this board:
 *   - ECHO is on D2  -> EXTI10, supports InterruptIn
 *   - PWMs are on D5 (TIM3_CH1) and D6 (TIM2_CH3) - independent
 *     timer instances, so the two PwmOut channels never collide.
 *   - Direction pins are taken from non-PWM silkscreen labels
 *     (D4, D7, D8) or from PWM pins used in plain DigitalOut mode
 *     (D9, D10) - this is allowed and frees the timers for the
 *     real speed pins.
 * ============================================================ */
#define TRIG_PIN     D8        /* HC-SR04 TRIG                       */
#define ECHO_PIN     D2        /* HC-SR04 ECHO  (silkscreen: D2)     */

#define LEFT_PWM     D5        /* L298N ENA  (silkscreen: PWM/D5)    */
#define LEFT_DIR_A   D4        /* L298N IN1  (silkscreen: D4)        */
#define LEFT_DIR_B   D7        /* L298N IN2  (silkscreen: D7)        */

#define RIGHT_PWM    D6        /* L298N ENB  (silkscreen: PWM/D6)    */
#define RIGHT_DIR_A  D9        /* L298N IN3  (silkscreen: PWM/D9,    */
#define RIGHT_DIR_B  D10       /* L298N IN4   PWM/CS/D10 - used as   */
                                /*             plain DigitalOut)     */

/* ============================================================
 *  Behavioural parameters
 * ============================================================ */
static constexpr float TARGET_CM   = 20.0f; // suitable following distance
static constexpr float DEAD_BAND   =  3.0f; // tolerance around target
static constexpr float MAX_RANGE   = 60.0f; // ignore objects past this
static constexpr float K_P         = 0.035f;// gain  (1/cm)
static constexpr float MIN_SPEED   = 0.40f; // overcome motor stiction
static constexpr float MAX_SPEED   = 0.80f; // upper duty cycle limit
static constexpr auto  LOOP_PERIOD = 50ms;  // controller tick

/* ============================================================
 *  Global hardware objects (constructed before main())
 * ============================================================ */
static Ultrasonic sonar(TRIG_PIN, ECHO_PIN);
static Motor      drive(LEFT_PWM,  LEFT_DIR_A,  LEFT_DIR_B,
                        RIGHT_PWM, RIGHT_DIR_A, RIGHT_DIR_B);

/* ============================================================
 *  Controller : map distance error to a signed wheel command
 *  Returns a duty cycle in [-MAX_SPEED, +MAX_SPEED]
 *    sign > 0 : drive forward
 *    sign < 0 : drive reverse
 *    sign = 0 : stop
 * ============================================================ */
static float compute_command(float distance_cm)
{
    const float error = distance_cm - TARGET_CM;   // + = too far

    if (fabsf(error) < DEAD_BAND) {
        return 0.0f;                               // we are at the sweet spot
    }

    float speed = MIN_SPEED + K_P * (fabsf(error) - DEAD_BAND);
    if (speed > MAX_SPEED) speed = MAX_SPEED;

    return (error > 0.0f) ? +speed : -speed;
}

/* ============================================================
 *  Apply a signed command to the H-bridge.
 * ============================================================ */
static void apply_command(float cmd)
{
    if (cmd > 0.0f)      drive.forward(cmd);
    else if (cmd < 0.0f) drive.reverse(-cmd);
    else                 drive.stop();
}

/* ============================================================
 *  main : the supervisor loop
 * ============================================================ */
int main()
{
    printf("\n[CSE211s] Object-follower starting...\r\n");

    drive.stop();
    sonar.start();

    while (true) {

        /* ----- 1. read latest measurement ------------------- */
        const bool  ok = sonar.valid();
        const float d  = sonar.distance_cm();

        /* ----- 2. decide -------------------------------------*/
        float cmd;
        if (!ok || d > MAX_RANGE) {
            cmd = 0.0f;                  // lost the target -> stop
        } else {
            cmd = compute_command(d);
        }

        /* ----- 3. actuate ---------------------------------- */
        apply_command(cmd);

        /* ----- 4. telemetry (optional) --------------------- */
        printf("d=%6.1f cm  ok=%d  cmd=%+.2f\r\n", d, ok, cmd);

        ThisThread::sleep_for(LOOP_PERIOD);
    }
}
