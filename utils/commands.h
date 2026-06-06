#pragma once

#include "data_types.h"

/*
commands.h
===============================================================================
ROLE
  Non-blocking Serial command interface.
===============================================================================
*/

void commands_service(SystemState &state);
void commands_print_help(void);