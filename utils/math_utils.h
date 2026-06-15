#pragma once

#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

/*
math_utils.h
===============================================================================
ROLE
  Stateless helper functions shared by modules.

DESIGN RULE
  Helpers here are deterministic and side-effect free except explicit in-place
  string normalization functions. No helper performs hardware I/O.
===============================================================================
*/

/*
is_finite_f(...)
------------------------------------------------------------------------------
ROLE
  Central float finiteness test.

MECHANISM
  Wraps isfinite() so project code uses one naming convention.
*/
static inline bool is_finite_f(float x) {
  return isfinite(x);
}

/*
age_ms(...)
------------------------------------------------------------------------------
ROLE
  Compute snapshot age while preserving invalid-snapshot semantics.

MECHANISM
  A valid snapshot returns unsigned elapsed time. An invalid snapshot returns a
  sentinel so telemetry and diagnostics can distinguish unavailable data from a
  true zero-age sample.
*/
static inline uint32_t age_ms(uint32_t now_ms, uint32_t t_ms, bool valid) {
  if (!valid) return 0xFFFFFFFFUL;
  return now_ms - t_ms;
}

/*
pressure_to_altitude_m(...)
------------------------------------------------------------------------------
ROLE
  Convert pressure to altitude using the standard barometric approximation.

INPUT CONTRACT
  press_hpa and reference pressure must be finite and positive.

FAILURE BEHAVIOR
  Invalid inputs return NAN. This prevents invalid pressure/reference values from
  silently becoming plausible-looking altitude estimates.
*/
static inline float pressure_to_altitude_m(float press_hpa, float sea_level_hpa) {
  if (!is_finite_f(press_hpa) || !is_finite_f(sea_level_hpa)) return NAN;
  if (press_hpa <= 0.0f || sea_level_hpa <= 0.0f) return NAN;

  // This is the standard-atmosphere inversion:
  //   h = 44330 * (1 - (p/p0)^(1/5.255))
  //
  // It is an approximation, but it is consistent across baseline capture,
  // telemetry, and estimator updates, which is more important here than having
  // an inconsistent mix of atmospheric models.
  return 44330.0f * (1.0f - powf(press_hpa / sea_level_hpa, 0.19029495718f));
}

/*
clamp01(...)
------------------------------------------------------------------------------
ROLE
  Bound normalized actuator/policy commands to [0,1].

FAILURE BEHAVIOR
  Non-finite input returns 0.0f, the fail-safe lower command.
*/
static inline float clamp01(float x) {
  if (!is_finite_f(x)) return 0.0f;
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

/*
trim_spaces(...)
------------------------------------------------------------------------------
ROLE
  Remove leading and trailing ASCII whitespace from a mutable C string.

MECHANISM
  Leading whitespace is removed by memmove(). Trailing whitespace is removed by
  overwriting with null terminators.

SAFETY
  A null pointer is accepted as a no-op.
*/
static inline void trim_spaces(char *s) {
  if (!s) return;

  char *p = s;
  while (*p && isspace((unsigned char)*p)) {
    ++p;
  }

  if (p != s) {
    memmove(s, p, strlen(p) + 1);
  }

  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[n - 1] = '\0';
    --n;
  }
}

/*
upper_inplace(...)
------------------------------------------------------------------------------
ROLE
  Normalize command tokens to uppercase for case-insensitive command parsing.

SAFETY
  Cast to unsigned char before toupper() avoids undefined behavior for bytes with
  the high bit set.
*/
static inline void upper_inplace(char *s) {
  if (!s) return;

  while (*s) {
    *s = (char)toupper((unsigned char)*s);
    ++s;
  }
}

/*
parse_float_arg(...)
------------------------------------------------------------------------------
ROLE
  Parse a complete floating-point command argument.

MECHANISM
  strtod() parses the numeric prefix. The remaining suffix must contain only
  whitespace, otherwise the argument is rejected.
*/
static inline bool parse_float_arg(const char *s, float *out_v) {
  if (!s || !out_v) return false;

  char *endp = NULL;
  double v = strtod(s, &endp);

  if (endp == s) return false;

  while (*endp) {
    if (!isspace((unsigned char)*endp)) return false;
    ++endp;
  }

  *out_v = (float)v;
  return true;
}
