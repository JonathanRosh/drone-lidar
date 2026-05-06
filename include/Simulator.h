#ifndef DRONE_LIDAR_ISIMULATOR_H
#define DRONE_LIDAR_ISIMULATOR_H

#include "IDrone.h"
#include "IMap3D.h"
#include "TrueMap.h"

class Simulator {

    IDrone drone;

public:
    Simulator(DroneConfig lidar_config, MissionConfig mission_config, TrueMap true_map);

    IMap3D& simulate();
};

#endif //DRONE_LIDAR_ISIMULATOR_H
