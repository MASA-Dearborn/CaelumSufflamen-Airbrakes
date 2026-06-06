#include "actuator.h"

#include "config.h"
#include "math_utils.h"

#if defined(ARDUINO_TEENSY41) || defined(TEENSYDUINO)
#include <PWMServo.h>
static PWMServo servo;
#else
#include <Servo.h>
static Servo servo;
#endif

/*
actuator.cpp
===============================================================================
PURPOSE
  Own servo attachment, pulse mapping, and idle forcing.

SAFETY
  ACTUATION_ENABLED gates non-idle command writes inside this module even if
  upstream logic accidentally calls actuator_write_command01().
===============================================================================
*/

static ActuatorConfig g_cfg = {
  SERVO_US_MIN_DEFAULT,
  SERVO_US_MAX_DEFAULT,
  SERVO_US_IDLE_DEFAULT
};

static bool g_attached = false;
static int g_last_us = SERVO_US_IDLE_DEFAULT;

/*
actuator_begin(...)
------------------------------------------------------------------------------
ROLE
  Attach the servo backend and immediately force the idle pulse.

INPUT CONTRACT
  cfg defines the pulse map. Invalid calibration is not repaired here; calibration
  validation should occur before flight enable.
*/
void actuator_begin(const ActuatorConfig &cfg) {
  g_cfg = cfg;
  servo.attach(PIN_AIRBRAKE_SERVO);
  g_attached = true;

  actuator_force_idle();
}

/*
actuator_force_idle(...)
------------------------------------------------------------------------------
ROLE
  Write the configured idle pulse.

SAFETY
  This function is the default actuator sink for any invalid policy or safety
  condition.
*/
void actuator_force_idle(void) {
  if (!g_attached) return;

  servo.writeMicroseconds(g_cfg.servo_us_idle);
  g_last_us = g_cfg.servo_us_idle;
}

/*
actuator_write_command01(...)
------------------------------------------------------------------------------
ROLE
  Map normalized policy command to servo microseconds.

MECHANISM
  command01 is clamped to [0,1], then linearly mapped from servo_us_min to
  servo_us_max.

COMPILE-TIME SAFETY
  If ACTUATION_ENABLED is false, the command is ignored and idle is forced.
*/
void actuator_write_command01(float command01) {
#if ACTUATION_ENABLED
  if (!g_attached) return;

  const float u = clamp01(command01);
  const int us =
    (int)((float)g_cfg.servo_us_min +
          u * (float)(g_cfg.servo_us_max - g_cfg.servo_us_min));

  servo.writeMicroseconds(us);
  g_last_us = us;
#else
  (void)command01;
  actuator_force_idle();
#endif
}

/*
actuator_last_us(...)
------------------------------------------------------------------------------
ROLE
  Report last pulse command for telemetry/debugging.
*/
int actuator_last_us(void) {
  return g_last_us;
}