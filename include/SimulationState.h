#ifndef DRONE_LIDAR_SIMULATIONSTATE_H
#define DRONE_LIDAR_SIMULATIONSTATE_H

#include "Units.h"

#include <string>

struct SimulationState {
    Position3D drone_position;
    Orientation drone_orientation;

    bool failed = false;
    std::string failure_reason;
};

#endif //DRONE_LIDAR_SIMULATIONSTATE_H
