#ifndef DRONE_LIDAR_IPOSITIONSENSOR_H
#define DRONE_LIDAR_IPOSITIONSENSOR_H

#include "Units.h"

class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    virtual Position3D& getPosition() const = 0;
    virtual Orientation getOrientation() const = 0;
};

#endif //DRONE_LIDAR_IPOSITIONSENSOR_H
