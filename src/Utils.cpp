#include "Utils.h"

#include "MapUtils.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

constexpr double kGridEpsilon = 1e-9;

} // namespace

namespace Utils {

double cmValue(Distance distance) {
  return distance.force_numerical_value_in(cm);
}

double degValue(HorizontalAngle angle) {
  return angle.force_numerical_value_in(deg);
}

double degValue(Altitude angle) {
  return angle.force_numerical_value_in(deg);
}

int lowerGridIndex(Distance value, Distance resolution) {
  return static_cast<int>(
      std::ceil(cmValue(value) / cmValue(resolution) - kGridEpsilon));
}

int upperGridIndex(Distance value, Distance resolution) {
  return static_cast<int>(
      std::floor(cmValue(value) / cmValue(resolution) + kGridEpsilon));
}

GridBounds gridBoundsForMission(const MissionConfig &mission_config) {
  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const auto &boundary = mission_config.map_boundry;

  return {
      {
          lowerGridIndex(boundary.minX, res_xy),
          lowerGridIndex(boundary.minY, res_xy),
          lowerGridIndex(boundary.minHeight, res_z),
      },
      {
          upperGridIndex(boundary.maxX, res_xy),
          upperGridIndex(boundary.maxY, res_xy),
          upperGridIndex(boundary.maxHeight, res_z),
      },
  };
}

ProgramOptions parseProgramOptions(int argc, char **argv) {
  ProgramOptions options{
      .input_output_path = std::filesystem::current_path(),
      .log_enabled = false,
  };

  bool path_set = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--log") {
      options.log_enabled = true;
      continue;
    }

    if (path_set) {
      throw std::runtime_error("Unexpected command line argument: " + arg);
    }

    options.input_output_path = std::filesystem::path(arg);
    path_set = true;
  }

  return options;
}

} // namespace Utils
