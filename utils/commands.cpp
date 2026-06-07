#include "commands.h"

#include <string.h>

#include "config.h"
#include "math_utils.h"
#include "telemetry.h"
#include "estimation.h"
#include "sd_logger.h"
#include "sensors.h"
#include "airbrake_policy.h"


/*
commands.cpp
===============================================================================
PURPOSE
  Provide a bounded Serial command parser.

DETERMINISM
  commands_service(...) consumes only bytes already available in Serial. It never
  waits for a complete command line. CAL_BASELINE is a deliberate bounded blocking
  ground calibration command.
===============================================================================
*/

static char cmd_buf[CMD_BUF_N];
static size_t cmd_len = 0;

/*
commands_print_help(...)
------------------------------------------------------------------------------
ROLE
  Emit the supported command surface.

INPUT CONTRACT
  Serial should already be initialized.

OUTPUT CONTRACT
  A help line is printed. Runtime state is not modified.

MECHANISM
  1. Print the supported command list.
  2. Keep this list synchronized with handle_command(...).

FAILURE BEHAVIOR
  Serial output is best-effort.

DETERMINISM
  Bounded Serial output. No loops. No hardware I/O. No dynamic allocation.
*/
void commands_print_help(void) {
  Serial.println(
    F("HELP,CMDS=HELP|STATUS|HDR 0|1|CAP_BASELINE|CAL_BASELINE|SET_SLP <hpa>|SIM_APOGEE <h_m> <v_mps>"));
}


/*
parse_two_float_args(...)
------------------------------------------------------------------------------
ROLE
  Parse two whitespace-separated floating-point command arguments.

INPUT CONTRACT
  s must point to a null-terminated argument string. out_a and out_b must be
  valid output pointers.

OUTPUT CONTRACT
  Returns true only when exactly two floating-point values are parsed and the
  trailing suffix contains only whitespace.

MECHANISM
  1. Parse first number with strtod.
  2. Parse second number with strtod.
  3. Reject missing numbers.
  4. Reject non-whitespace trailing text.
  5. Store parsed values.

FAILURE BEHAVIOR
  Invalid input returns false and does not rely on partial parse results.

DETERMINISM
  Bounded string parse. No heap allocation. No hardware I/O.
*/
static bool parse_two_float_args(const char *s, float *out_a, float *out_b) {
  if (!s || !out_a || !out_b) return false;

  char *end_a = NULL;
  const double a = strtod(s, &end_a);

  if (end_a == s) return false;

  char *end_b = NULL;
  const double b = strtod(end_a, &end_b);

  if (end_b == end_a) return false;

  while (*end_b) {
    if (!isspace((unsigned char)*end_b)) {
      return false;
    }

    ++end_b;
  }

  *out_a = (float)a;
  *out_b = (float)b;

  return true;
}

/*
handle_command(...)
------------------------------------------------------------------------------
ROLE
  Execute one complete null-terminated command line.

INPUT CONTRACT
  s must refer to the shared SystemState object. line must point to a mutable
  null-terminated command string.

OUTPUT CONTRACT
  The command may print help/status, change configuration, capture/calibrate
  baseline pressure, or reset estimator/logger references.

MECHANISM
  1. Trim whitespace.
  2. Split command token from optional argument.
  3. Uppercase command token.
  4. Dispatch command.
  5. Reset estimator and SD reference state after pressure-reference changes.

FAILURE BEHAVIOR
  Unknown command emits ERR,UNKNOWN_CMD. Invalid arguments leave existing
  configuration unchanged.

DETERMINISM
  Bounded string parsing and Serial output. CAL_BASELINE is the only command that
  intentionally performs a bounded blocking calibration routine.
*/
static void handle_command(SystemState &s, char *line) {
  trim_spaces(line);

  if (line[0] == '\0') return;

  char *arg = strchr(line, ' ');
  if (arg) {
    *arg++ = '\0';
    trim_spaces(arg);
  }

  upper_inplace(line);

  if (strcmp(line, "HELP") == 0) {
    commands_print_help();
    return;
  }

  if (strcmp(line, "STATUS") == 0) {
    telemetry_print_status(s);
    return;
  }

  if (strcmp(line, "HDR") == 0) {
    s.cfg.serial_header_enable = (!arg || strcmp(arg, "0") != 0);

    if (s.cfg.serial_header_enable) {
      telemetry_print_header();
    }

    Serial.println(F("ACK,HDR"));
    return;
  }

  if (strcmp(line, "SET_SLP") == 0) {
    float v = NAN;

    if (arg && parse_float_arg(arg, &v) && is_finite_f(v) && v > 0.0f) {
      s.cfg.sea_level_hpa = v;
      s.cfg.valid = true;

      estimation_reset(s);
      sd_logger_reset_reference_and_kf(s);

      Serial.println(F("ACK,SET_SLP"));
    } else {
      Serial.println(F("ERR,SET_SLP"));
    }

    return;
  }

  if (strcmp(line, "CAP_BASELINE") == 0) {
    if (s.baro.valid && is_finite_f(s.baro.press_hpa)) {
      s.cfg.baro_baseline_hpa = s.baro.press_hpa;
      s.cfg.valid = true;

      estimation_reset(s);
      sd_logger_reset_reference_and_kf(s);

      Serial.println(F("ACK,CAP_BASELINE"));
    } else {
      Serial.println(F("ERR,CAP_BASELINE"));
    }

    return;
  }

  if (strcmp(line, "CAL_BASELINE") == 0) {
    const bool ok = sensors_calibrate_baro_base(s, BARO_CALIB_SAMPLES);

    if (ok) {
      estimation_reset(s);
      sd_logger_reset_reference_and_kf(s);

      Serial.print(F("ACK,CAL_BASELINE,"));
      Serial.println(s.cfg.baro_baseline_hpa);
    } else {
      Serial.println(F("ERR,CAL_BASELINE"));
    }

    #if AIRBRAKE_POLICY_TEST_API
        if (strcmp(line, "SIM_APOGEE") == 0) {
          float h_m = NAN;
          float v_mps = NAN;

          if (!arg || !parse_two_float_args(arg, &h_m, &v_mps)) {
            Serial.println(F("ERR,SIM_APOGEE"));
            return;
          }

          const float u_max = clamp01(POLICY_MAX_COMMAND01);

          const float apogee0 =
            airbrake_policy_predict_apogee_m(h_m, v_mps, 0.0f);

          const float apogee1 =
            airbrake_policy_predict_apogee_m(h_m, v_mps, u_max);

          const float cmd =
            airbrake_policy_solve_command01(
              h_m,
              v_mps,
              POLICY_TARGET_APOGEE_M);

          Serial.print(F("SIM_APOGEE,h="));
          Serial.print(h_m);

          Serial.print(F(",v="));
          Serial.print(v_mps);

          Serial.print(F(",apogee0="));
          Serial.print(apogee0);

          Serial.print(F(",apogee1="));
          Serial.print(apogee1);

          Serial.print(F(",target="));
          Serial.print(POLICY_TARGET_APOGEE_M);

          Serial.print(F(",cmd="));
          Serial.println(cmd);

          return;
        }
    #endif
    return;
  }

  Serial.println(F("ERR,UNKNOWN_CMD"));
}

/*
commands_service(...)
------------------------------------------------------------------------------
ROLE
  Consume available Serial bytes and execute complete command lines.

INPUT CONTRACT
  s must refer to the shared SystemState object. Serial should already be
  initialized. This function should be called frequently from loop().

OUTPUT CONTRACT
  Complete command lines are dispatched. Partial command text remains buffered.

MECHANISM
  1. Read only bytes already available in Serial.
  2. Treat CR/LF as command terminators.
  3. Null-terminate and dispatch non-empty command lines.
  4. Append ordinary bytes while buffer capacity remains.
  5. Discard overlong commands and emit an error.

FAILURE BEHAVIOR
  Overlong commands are discarded so a malformed line cannot corrupt later input.

DETERMINISM
  Bounded by available Serial bytes. No dynamic allocation. No blocking wait.
*/
void commands_service(SystemState &s) {
  while (Serial.available() > 0) {
    int ch = Serial.read();

    if (ch < 0) {
      break;
    }

    if (ch == '\r' || ch == '\n') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = '\0';
        handle_command(s, cmd_buf);
        cmd_len = 0;
      }

      continue;
    }

    if (cmd_len + 1 < sizeof(cmd_buf)) {
      cmd_buf[cmd_len++] = (char)ch;
    } else {
      cmd_len = 0;
      Serial.println(F("ERR,CMD_TOO_LONG"));
    }
  }
}
