#include "../include/Drone.h"

Drone::Drone(
    IMovementDriver& movement_driver,
    IPositionSensor& position_sensor,
    ILidarSensor& lidar_sensor)
    : movement_driver(movement_driver),
      position_sensor(position_sensor),
      lidar_sensor(lidar_sensor)
{}

//TODO: fix types to avoid this conversion
void Drone::rotateRight(Angle angle) const {
    movement_driver.rotateRight(HorizontalAngle{angle.force_numerical_value_in(deg) * deg});
}

void Drone::rotateLeft(Angle angle) const {
    movement_driver.rotateLeft(HorizontalAngle{angle.force_numerical_value_in(deg) * deg});
}

void Drone::advance(Distance distance) const {
    movement_driver.advance(distance);
}

void Drone::elevate(Distance distance) const {
    movement_driver.elevate(distance);
}

Position3D Drone::getPosition() const {
    return position_sensor.getPosition();
}

Orientation Drone::getOrientation() const {
    return position_sensor.getOrientation();
}

LidarScanResult Drone::scan(Angle xy_angle, Angle height_angle) const {
    Orientation scan_orientation{
        HorizontalAngle{xy_angle.force_numerical_value_in(deg) * deg},
        Altitude{height_angle.force_numerical_value_in(deg) * deg}
    };

    return lidar_sensor.getScan(scan_orientation);
}
