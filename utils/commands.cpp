#include "commands.h"

#include <string.h>

#include "config.h"
#include "math_utils.h"
#include "telemetry.h"
#include "estimation.h"
#include "sd_logger.h"
#include "sensors.h"

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
  Serial.println(F("HELP,CMDS=HELP|STATUS|HDR 0|1|CAP_BASELINE|CAL_BASELINE|SET_SLP <hpa>"));
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