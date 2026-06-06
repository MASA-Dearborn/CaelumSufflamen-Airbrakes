#include "config.h"

/*
config.cpp
===============================================================================
ROLE
  Placeholder translation unit for compile-time configuration.

RATIONALE
  The finalized modular branch uses RuntimeConfig inside SystemState and
  constants from config.h. The earlier EEPROM-backed Config/EepromBlob branch is
  intentionally not used here because it belongs to a different API generation.

ENGINEERING NOTE
  Keeping this file present preserves the requested module structure and gives a
  safe location for future persistence helpers without changing sketch layout.
===============================================================================
*/