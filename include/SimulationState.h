#ifndef DRONE_LIDAR_SIMULATIONSTATE_H
#define DRONE_LIDAR_SIMULATIONSTATE_H

#include "Units.h"

struct SimulationState {
    Position3D drone_position;
    Orientation drone_orientation;
};

#endif //DRONE_LIDAR_SIMULATIONSTATE_H
