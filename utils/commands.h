#pragma once

#include "data_types.h"

/*
commands.h
===============================================================================
ROLE
  Non-blocking Serial command interface.
===============================================================================
*/
void commands_print_help(void);
static void handle_command(SystemState &s, char *line);
static bool parse_two_float_args(const char *s, float *out_a, float *out_b);
void commands_service(SystemState &state);
