#pragma once

#include <mp-units/systems/si/unit_symbols.h>
#include <string>
#include <vector>

namespace parser {

using Length = decltype(1.0 * mp_units::si::unit_symbols::cm);
using Angle = decltype(1.0 * mp_units::si::unit_symbols::deg);

struct DroneConfig {
  struct MinPass {
    Length width = 0.0 * mp_units::si::unit_symbols::cm;
    Length length = 0.0 * mp_units::si::unit_symbols::cm;
    Length height = 0.0 * mp_units::si::unit_symbols::cm;
  };

  MinPass min_pass;

  struct MaxCommand {
    Angle rotate = 0.0 * mp_units::si::unit_symbols::deg;
    Length advance = 0.0 * mp_units::si::unit_symbols::cm;
    Length elevate = 0.0 * mp_units::si::unit_symbols::cm;
  };
  MaxCommand max_command;

  struct LidarConfig {
    Length z_min = 0.0 * mp_units::si::unit_symbols::cm;
    Length z_max = 0.0 * mp_units::si::unit_symbols::cm;
    Length d = 0.0 * mp_units::si::unit_symbols::cm;
    int fovc = 1;
  };

  LidarConfig lidar_config;
};

struct MissionConfig {

  struct MapBoundary {
    Length x_min = 0.0 * mp_units::si::unit_symbols::cm;
    Length x_max = 0.0 * mp_units::si::unit_symbols::cm;
    Length y_min = 0.0 * mp_units::si::unit_symbols::cm;
    Length y_max = 0.0 * mp_units::si::unit_symbols::cm;
    Length height_min = 0.0 * mp_units::si::unit_symbols::cm;
    Length height_max = 0.0 * mp_units::si::unit_symbols::cm;
  };
  MapBoundary map_boundary;

  struct StartPosition {
    Length x = 0.0 * mp_units::si::unit_symbols::cm;
    Length y = 0.0 * mp_units::si::unit_symbols::cm;
    Length height = 0.0 * mp_units::si::unit_symbols::cm;
    Angle angle = 0.0 * mp_units::si::unit_symbols::deg;
  };
  StartPosition start_position;

  struct Resolution {
    int xy = 0;
    int height = 0;
  };
  Resolution resolution;
};

struct InputMap {
  struct OccupiedPoint {
    Length x = 0.0 * mp_units::si::unit_symbols::cm;
    Length y = 0.0 * mp_units::si::unit_symbols::cm;
    Length height = 0.0 * mp_units::si::unit_symbols::cm;
  };

  std::vector<OccupiedPoint> occupied_points;
};

DroneConfig parseDroneConfigFile(const std::string &path);
MissionConfig parseMissionConfigFile(const std::string &path);
InputMap parseInputMapFile(const std::string &path);

std::string buildPath(const std::string &base_path,
                      const std::string &filename);

void writeInputErrorsFile(const std::string &base_path,
                          const std::vector<std::string> &errors);
} // namespace parser