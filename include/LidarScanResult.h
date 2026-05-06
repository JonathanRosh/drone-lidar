#ifndef DRONE_LIDAR_SCANRESULT_H
#define DRONE_LIDAR_SCANRESULT_H

#include "Units.h"

struct LidarHit {
    Distance distance;
    Orientation orientation;
};

struct  LidarScanResult {
    std::vector<LidarHit> hits;
};

#endif //DRONE_LIDAR_SCANRESULT_H
