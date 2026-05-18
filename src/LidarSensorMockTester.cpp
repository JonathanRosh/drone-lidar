#include "../include/LidarSensorMockTester.h"

std::optional<Distance> LidarSensorMockTester::traceBeam(const LidarSensorMock& lidar, const Orientation& beam_orientation) {
  return lidar.traceBeam(beam_orientation);
}