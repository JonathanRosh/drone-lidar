#ifndef DRONE_LIDAR_ILIDARSENSOR_H
#define DRONE_LIDAR_ILIDARSENSOR_H

#include "LidarScanResult.h"

class ILidarSensor {
public:
    //virtual ~ILidarSensor();
    virtual LidarScanResult getScan(const Orientation& drone_orientation) const = 0;
};

#endif //DRONE_LIDAR_ILIDARSENSOR_H
