#include "commands.h"

#include <string.h>

#include "config.h"
#include "math_utils.h"
#include "telemetry.h"
#include "estimation.h"
#include "sd_logger.h"

/*
commands.cpp
===============================================================================
PURPOSE
  Provide a bounded, non-blocking Serial command parser.

DETERMINISM
  The service function consumes only bytes already available in Serial. It never
  waits for a complete command line.

COMMANDS
  HELP
  STATUS
  HDR 0|1
  CAP_BASELINE
  SET_SLP <hPa>
===============================================================================
*/

static char cmd_buf[CMD_BUF_N];
static size_t cmd_len = 0;

/*
commands_print_help(...)
------------------------------------------------------------------------------
ROLE
  Emit the supported command surface.
*/
void commands_print_help(void) {
  Serial.println(F("HELP,CMDS=HELP|STATUS|HDR 0|1|CAP_BASELINE|SET_SLP <hpa>"));
}

/*
handle_command(...)
------------------------------------------------------------------------------
ROLE
  Execute one complete, null-terminated command line.

MECHANISM
  The command token is uppercased. The argument string is kept as text for the
  specific command parser.
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

  if (strcmp(line, "SET_SLP") == 0) 
  {
    float v = NAN;

    if (arg && parse_float_arg(arg, &v) && is_finite_f(v) && v > 0.0f) 
    {
      s.cfg.sea_level_hpa = v;
      s.cfg.valid = true;
      estimation_reset(s);
      sd_logger_reset_reference_and_kf(s);
      Serial.println(F("ACK,SET_SLP"));
    } 
    else 
    {
      Serial.println(F("ERR,SET_SLP"));
    }

    return;
  }

  if (strcmp(line, "CAP_BASELINE") == 0) {
    if (s.baro.valid && is_finite_f(s.baro.press_hpa)) 
    {
      s.cfg.baro_baseline_hpa = s.baro.press_hpa;
      s.cfg.valid = true;
      estimation_reset(s);
      sd_logger_reset_reference_and_kf(s);
      Serial.println(F("ACK,CAP_BASELINE"));
    } 
    else 
    {
      Serial.println(F("ERR,CAP_BASELINE"));
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

OVERFLOW BEHAVIOR
  If the line buffer fills before a newline arrives, the partial command is
  discarded and an error is emitted. This prevents buffer overrun and prevents
  a malformed long command from contaminating the next command.
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