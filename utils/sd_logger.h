#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
sd_logger.h
===============================================================================
ROLE
  SD-card CSV logging subsystem interface.

OWNERSHIP
  sd_logger.cpp owns:
    - SD initialization status,
    - log-file creation,
    - file write/flush cadence,
    - runtime SD failure state.

DESIGN REASON
  The SD logger is separated from telemetry. Telemetry is a live Serial
  observation channel. The SD logger is a persistent flight/test record channel.

FAILURE MODEL
  SD failure must never stop flight runtime execution. If SD initialization or a
  runtime write fails, the logger disables itself and records the fault in
  SdLoggerState so telemetry can report it.
===============================================================================
*/

void sd_logger_init(SystemState &state);
void sd_logger_reset_reference_and_kf(SystemState &state);
void sd_logger_service(SystemState &state, uint32_t now_us, uint32_t now_ms);
bool sd_logger_ok(const SystemState &state);