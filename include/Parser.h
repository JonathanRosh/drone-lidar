#ifndef DRONE_LIDAR_PARSER_H
#define DRONE_LIDAR_PARSER_H

#include "Configs.h"
#include "TrueMap.h"
#include <string>

DroneConfig parseDroneConfig(const std::string &filename);
MissionConfig parseMissionConfig(const std::string &filename);
TrueMap parseTrueMap(const std::string &filename,
                     const MissionConfig &mission_config);

#endif // DRONE_LIDAR_PARSER_H