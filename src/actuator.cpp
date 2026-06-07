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
  Own servo attachment, command mapping, and idle forcing.

IMPLEMENTATION NOTE
  PWMServo installations may not expose writeMicroseconds(). This implementation
  maps configured pulse-equivalent values into Servo/PWMServo degrees. g_last_us
  remains the telemetry representation of the equivalent pulse command.
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
us_to_degrees(...)
------------------------------------------------------------------------------
ROLE
  Convert pulse-equivalent microseconds into Servo/PWMServo angle.

INPUT CONTRACT
  g_cfg must contain a positive min/max span. us may be outside span.

OUTPUT CONTRACT
  Returns angle in [0,180] for valid span. Returns zero for invalid span.

MECHANISM
  1. Clamp input pulse to configured min/max.
  2. Compute span.
  3. Reject non-positive span.
  4. Map relative pulse position to degrees.

FAILURE BEHAVIOR
  Invalid calibration span returns zero degrees.

DETERMINISM
  Constant-time integer math. No loops. No hardware I/O. No dynamic allocation.
*/
static int us_to_degrees(int us) {
  if (us < g_cfg.servo_us_min) us = g_cfg.servo_us_min;
  if (us > g_cfg.servo_us_max) us = g_cfg.servo_us_max;

  const long span_us = (long)g_cfg.servo_us_max - (long)g_cfg.servo_us_min;
  if (span_us <= 0L) return 0;

  const long rel_us = (long)us - (long)g_cfg.servo_us_min;
  return (int)((rel_us * 180L) / span_us);
}

/*
command01_to_us(...)
------------------------------------------------------------------------------
ROLE
  Convert normalized actuator command into equivalent pulse width.

INPUT CONTRACT
  command01 may be finite, non-finite, or outside [0,1]. clamp01 handles it.

OUTPUT CONTRACT
  Returns equivalent pulse within configured span.

MECHANISM
  1. Clamp normalized command to [0,1].
  2. Multiply by configured pulse span.
  3. Add configured minimum pulse.

FAILURE BEHAVIOR
  Non-finite command maps to zero command through clamp01.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
static int command01_to_us(float command01) {
  const float u = clamp01(command01);

  return (int)(
    (float)g_cfg.servo_us_min +
    u * (float)(g_cfg.servo_us_max - g_cfg.servo_us_min)
  );
}

/*
actuator_begin(...)
------------------------------------------------------------------------------
ROLE
  Attach servo backend and immediately force idle output.

INPUT CONTRACT
  cfg should contain safe servo min, max, and idle settings.

OUTPUT CONTRACT
  Module configuration is stored, servo is attached, and idle is commanded.

MECHANISM
  1. Store actuator configuration.
  2. Attach servo backend to PIN_AIRBRAKE_SERVO.
  3. Mark backend attached.
  4. Force idle.

FAILURE BEHAVIOR
  Servo attach status is not explicitly reported by this backend.

DETERMINISM
  Bounded servo attach call. No loops. No dynamic allocation by project code.
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
  Write configured idle output to the servo backend.

INPUT CONTRACT
  actuator_begin(...) should have been called.

OUTPUT CONTRACT
  If attached, servo output is set to idle and g_last_us records idle pulse.

MECHANISM
  1. Reject unattached backend.
  2. Convert idle pulse-equivalent to degrees.
  3. Write servo angle.
  4. Store idle equivalent pulse.

FAILURE BEHAVIOR
  Unattached backend returns without hardware write.

DETERMINISM
  Constant-time plus one servo write. No loops. No dynamic allocation.
*/
void actuator_force_idle(void) {
  if (!g_attached) return;

  servo.write(us_to_degrees(g_cfg.servo_us_idle));
  g_last_us = g_cfg.servo_us_idle;
}

/*
actuator_write_command01(...)
------------------------------------------------------------------------------
ROLE
  Apply normalized actuator command if compile-time actuation is enabled.

INPUT CONTRACT
  command01 may be any float.

OUTPUT CONTRACT
  If actuation is compiled in and backend is attached, servo command is written.
  Otherwise idle is forced.

MECHANISM
  1. Enforce compile-time actuation gate.
  2. Reject unattached backend.
  3. Convert normalized command to pulse-equivalent.
  4. Convert pulse-equivalent to degrees.
  5. Write servo output and record telemetry pulse.

FAILURE BEHAVIOR
  Safe builds force idle. Non-finite commands clamp to zero.

DETERMINISM
  Constant-time plus one servo write. No loops. No dynamic allocation.
*/
void actuator_write_command01(float command01) {
#if ACTUATION_ENABLED
  if (!g_attached) return;

  const int us = command01_to_us(command01);
  servo.write(us_to_degrees(us));
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
  Report last equivalent pulse command for telemetry.

INPUT CONTRACT
  None.

OUTPUT CONTRACT
  Returns g_last_us.

MECHANISM
  1. Return stored pulse-equivalent value.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
int actuator_last_us(void) {
  return g_last_us;
}