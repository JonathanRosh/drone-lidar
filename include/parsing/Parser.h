#pragma once

#include "config/DroneConfig.h"
#include "config/MapInput.h"
#include "config/MissionConfig.h"
#include <string>
#include <vector>

namespace parsing {

config::DroneConfig parseDroneConfigFile(const std::string &path);
config::MissionConfig parseMissionConfigFile(const std::string &path);
config::InputMap parseInputMapFile(const std::string &path);

std::string buildPath(const std::string &base_path, const std::string &filename);

void writeInputErrorsFile(const std::string &base_path,
                          const std::vector<std::string> &errors);

} // namespace parsing
