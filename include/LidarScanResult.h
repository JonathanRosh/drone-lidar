#ifndef DRONE_LIDAR_SCANRESULT_H
#define DRONE_LIDAR_SCANRESULT_H

#include "Units.h"

struct LidarHit {
  Distance distance;
  Orientation orientation;
  bool detected = true;
};

typedef std::vector<LidarHit> LidarScanResult;

#endif // DRONE_LIDAR_SCANRESULT_H
