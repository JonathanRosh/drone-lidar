#pragma once

#include "domain/Units.h"

namespace config {

struct DroneConfig {
  struct MinPass {
    domain::Length width = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length length = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length height = 0.0 * mp_units::si::unit_symbols::cm;
  };
  MinPass min_pass;

  struct MaxCommand {
    domain::Angle rotate = 0.0 * mp_units::si::unit_symbols::deg;
    domain::Length advance = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length elevate = 0.0 * mp_units::si::unit_symbols::cm;
  };
  MaxCommand max_command;

  struct LidarConfig {
    domain::Length z_min = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length z_max = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length d = 0.0 * mp_units::si::unit_symbols::cm;
    int fovc = 1;
  };
  LidarConfig lidar_config;
};

} // namespace config
