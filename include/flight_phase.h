#pragma once

#include "data_types.h"

/*
flight_phase.h
===============================================================================
ROLE
  Conservative stateful flight-phase detector interface.

PURPOSE
  The detector classifies the current vehicle state into broad phases used by
  safety and policy logic:

    IDLE
    BOOST
    COAST
    BRAKE
    DESCENT

STATEFUL CONTRACT
  The implementation is intentionally stateful. It uses private launch, burnout,
  and descent latches plus dwell timers to prevent noisy instantaneous sensor
  samples from causing phase chatter. flight_phase_reset(...) is therefore the
  required way to return the detector to IDLE before a new ground test or flight.

SAFETY CONTRACT
  Phase classification is advisory. The safety module and actuator module remain
  responsible for final actuation permission.
===============================================================================
*/

void flight_phase_reset(SystemState &state);
void flight_phase_update(SystemState &state, uint32_t now_ms);
const char *flight_phase_name(FlightPhase phase);
