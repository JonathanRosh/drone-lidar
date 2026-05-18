#ifndef DRONE_LIDAR_LIDARSENSORMOCKTESTER_H
#define DRONE_LIDAR_LIDARSENSORMOCKTESTER_H

#include "Units.h"
#include "LidarSensorMock.h"

#include <optional>

class LidarSensorMock;

class LidarSensorMockTester {
public:
  static std::optional<Distance> traceBeam(const LidarSensorMock& lidar ,const Orientation& beam_orientation);
};

#endif