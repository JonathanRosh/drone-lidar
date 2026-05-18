#ifndef DRONE_LIDAR_IDRONE_H
#define DRONE_LIDAR_IDRONE_H

#include "Units.h"
#include "LidarScanResult.h"
#include "IMap3D.h"

class IDrone {
public:
    virtual void rotateLeft(HorizontalAngle angle) const = 0;
    virtual void rotateRight(HorizontalAngle angle) const = 0;
    virtual void advance(Distance distance) const = 0;
    virtual void elevate(Distance distance) const = 0;

    virtual LidarScanResult scan(Angle xy_angle, Angle height_angle) const = 0;
    virtual Position3D getPosition() const = 0;

    // virtual IMap3D& finish() const = 0;
};

#endif //DRONE_LIDAR_IDRONE_H
