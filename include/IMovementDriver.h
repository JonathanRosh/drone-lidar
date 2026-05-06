#ifndef DRONE_LIDAR_IMOVEMENTDRIVER_H
#define DRONE_LIDAR_IMOVEMENTDRIVER_H

#include "Units.h"

class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    virtual void rotateLeft(Angle angle) const = 0;
    virtual void rotateRight(Angle angle) const = 0;
    virtual void advance(Distance distance) const = 0;
    virtual void elevate(Distance distance) const = 0;
};

#endif //DRONE_LIDAR_IMOVEMENTDRIVER_H
