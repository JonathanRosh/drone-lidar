#ifndef DRONE_LIDAR_CONFIG_H
#define DRONE_LIDAR_CONFIG_H

#include "Units.h"

struct MinPass {
    Length width;
    Length height;
    Length length;
};

struct MaxCommand {
    Distance maxAdvance;
    Distance maxElevate;
    Angle maxRotate;
};

struct DroneConfig {
    MinPass minPass;
    MaxCommand maxCommand;
};

struct LidarConfig {
    Distance zMin;
    Distance zMax;
    Distance d;
    unsigned int fovc;
};

struct MapBoundry {
    Distance minX, minY, maxX, maxY, maxHeight, minHeight;
};

struct MapResolution {
    unsigned int xy_resolution;
    unsigned int height_resolution;
};

struct MissionConfig {
    MapBoundry map_boundry;
    MapResolution map_resolution;
    // std::set<Position3D> recharging_stations; TODO
};

#endif //DRONE_LIDAR_CONFIG_H
