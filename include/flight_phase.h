#pragma once

#include "data_types.h"

/*
flight_phase.h
===============================================================================
ROLE
  Conservative flight-phase detector interface.

PURPOSE
  The detector classifies the current vehicle state into broad phases used by
  safety and policy logic:

    IDLE
    BOOST
    COAST
    BRAKE
    DESCENT

SAFETY CONTRACT
  Phase classification is advisory. The safety module and actuator module remain
  responsible for final actuation permission.
===============================================================================
*/

void flight_phase_reset(SystemState &state);
void flight_phase_update(SystemState &state, uint32_t now_ms);
const char *flight_phase_name(FlightPhase phase);