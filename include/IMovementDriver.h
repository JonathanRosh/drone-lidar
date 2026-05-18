#ifndef DRONE_LIDAR_IMOVEMENTDRIVER_H
#define DRONE_LIDAR_IMOVEMENTDRIVER_H

#include "Units.h"

class IMovementDriver {
public:
    virtual void rotateLeft(HorizontalAngle angle) const = 0;
    virtual void rotateRight(HorizontalAngle angle) const = 0;
    virtual void advance(Distance distance) const = 0;
    virtual void elevate(Distance distance) const = 0;
};

#endif //DRONE_LIDAR_IMOVEMENTDRIVER_H
