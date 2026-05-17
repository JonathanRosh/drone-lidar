#ifndef DRONE_LIDAR_ISIMULATOR_H
#define DRONE_LIDAR_ISIMULATOR_H

#include "IDrone.h"
#include "IMap3D.h"
#include "TrueMap.h"

class Simulator {

  IDrone& drone;

public:
  Simulator(DroneConfig drone_config, MissionConfig mission_config,
            TrueMap true_map); // Should it take them as reference?

  IMap3D &simulate();
};

#endif // DRONE_LIDAR_ISIMULATOR_H
