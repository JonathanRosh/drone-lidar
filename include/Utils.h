#pragma once

#include "Configs.h"
#include "GridCoord.h"
#include "Units.h"

#include <filesystem>

namespace Utils {

struct GridBounds {
  GridCoord min;
  GridCoord max;
};

struct ProgramOptions {
  std::filesystem::path input_output_path;
  bool log_enabled = false;
};

double cmValue(Distance distance);
double degValue(HorizontalAngle angle);
double degValue(Altitude angle);
int lowerGridIndex(Distance value, Distance resolution);
int upperGridIndex(Distance value, Distance resolution);
GridBounds gridBoundsForMission(const MissionConfig &mission_config);
ProgramOptions parseProgramOptions(int argc, char **argv);

} // namespace Utils
