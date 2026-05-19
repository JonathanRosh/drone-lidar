#ifndef DRONE_LIDAR_PARSER_H
#define DRONE_LIDAR_PARSER_H

#include "Configs.h"
#include "TrueMap.h"
#include <string>

class Parser {
public:
  static DroneConfig parseDroneConfig(const std::string &filename);
  static MissionConfig parseMissionConfig(const std::string &filename);
  static TrueMap parseTrueMap(const std::string &filename,
                              const MissionConfig &mission_config);
};

#endif // DRONE_LIDAR_PARSER_H
