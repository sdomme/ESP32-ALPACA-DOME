/*
  config.h - Selects the configuration: config_local.h if present (personal settings,
  ignored by git), otherwise config_default.h. Edit one of those, not this file.
*/
#pragma once

// Park sensor types for PARK_SENSOR_TYPE
#define PARK_SENSOR_REED  1
#define PARK_SENSOR_BEAM  2

#if __has_include("config_local.h")
#include "config_local.h"
#else
#include "config_default.h"
#endif
