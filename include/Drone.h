#ifndef DRONE_LIDAR_DRONE_H
#define DRONE_LIDAR_DRONE_H

#include "IDrone.h"
#include "ILidarSensor.h"
#include "IMovementDriver.h"
#include "IPositionSensor.h"

class Drone : public IDrone {

    IMovementDriver& movement_driver;
    IPositionSensor& position_sensor;
    ILidarSensor& lidar_sensor;

public:
  Drone(IMovementDriver &movement_driver, IPositionSensor &position_sensor,
        ILidarSensor &lidar_sensor);

    void rotateRight(Angle angle) const override;
    void rotateLeft(Angle angle) const override;
    void advance(Distance distance) const override;
    void elevate(Distance distance) const override;

  LidarScanResult scan(Angle xy_angle, Angle height_angle) const override;
  Position3D getPosition() const override;

    // IMap3D& finish() const override;
};

#endif // DRONE_LIDAR_DRONE_H
