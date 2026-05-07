#include "Configs.h"
#include "TrueMap.h"
#include "Units.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string trim(const std::string &s) {
  const std::size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const std::size_t end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

bool parseKeyValue(const std::string &line, std::string &key,
                   std::string &value) {
  const std::size_t sep = line.find(':');
  if (sep == std::string::npos) {
    return false;
  }
  key = trim(line.substr(0, sep));
  value = trim(line.substr(sep + 1));
  return !key.empty() && !value.empty();
}

} // namespace

DroneConfig parseDroneConfig(const std::string &filename) {

  DroneConfig drone_config{};

  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  std::string line;
  while (std::getline(file, line)) {
    const std::string cleaned_line = trim(line);
    if (cleaned_line.empty() || cleaned_line[0] == '#')
      continue;

    std::string key;
    std::string value;
    if (parseKeyValue(cleaned_line, key, value)) {
      if (key == "min_pass_width") {
        drone_config.minPass.width = Length(std::stod(value), cm);
      } else if (key == "min_pass_height") {
        drone_config.minPass.height = Length(std::stod(value), cm);
      } else if (key == "min_pass_length") {
        drone_config.minPass.length = Length(std::stod(value), cm);
      } else if (key == "max_advance") {
        drone_config.maxCommand.maxAdvance = Distance(std::stod(value), cm);
      } else if (key == "max_elevate") {
        drone_config.maxCommand.maxElevate = Distance(std::stod(value), cm);
      } else if (key == "max_rotate") {
        drone_config.maxCommand.maxRotate = Angle(std::stod(value), deg);
      } else if (key == "lidar_z_min") {
        drone_config.lidarConfig.zMin = Distance(std::stod(value), cm);
      } else if (key == "lidar_z_max") {
        drone_config.lidarConfig.zMax = Distance(std::stod(value), cm);
      } else if (key == "lidar_d") {
        drone_config.lidarConfig.d = Distance(std::stod(value), cm);
      } else if (key == "lidar_fovc") {
        drone_config.lidarConfig.fovc = std::stoi(value);
      }
    }
  }

  return drone_config;
}

MissionConfig parseMissionConfig(const std::string &filename) {
  MissionConfig mission_config{};

  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  std::string line;
  while (std::getline(file, line)) {
    const std::string cleaned_line = trim(line);
    if (cleaned_line.empty() || cleaned_line[0] == '#')
      continue;

    std::string key;
    std::string value;
    if (parseKeyValue(cleaned_line, key, value)) {
      if (key == "map_boundary_x_min") {
        mission_config.map_boundry.minX = Distance(std::stod(value), cm);
      } else if (key == "map_boundary_y_min") {
        mission_config.map_boundry.minY = Distance(std::stod(value), cm);
      } else if (key == "map_boundary_x_max") {
        mission_config.map_boundry.maxX = Distance(std::stod(value), cm);
      } else if (key == "map_boundary_y_max") {
        mission_config.map_boundry.maxY = Distance(std::stod(value), cm);
      } else if (key == "map_boundary_height_max") {
        mission_config.map_boundry.maxHeight = Distance(std::stod(value), cm);
      } else if (key == "map_boundary_height_min") {
        mission_config.map_boundry.minHeight = Distance(std::stod(value), cm);
      } else if (key == "resolution_xy" || key == "map_resolution_xy") {
        mission_config.map_resolution.xy_resolution = std::stoi(value);
      } else if (key == "resolution_height" || key == "map_resolution_height") {
        mission_config.map_resolution.height_resolution = std::stoi(value);
      }
    }
  }

  return mission_config;
}

// TODO after deciding data sturcture for TrueMap

/*TrueMap parseTrueMap(const std::string &filename) {

  TrueMap trueMap;
  return trueMap;

}*/