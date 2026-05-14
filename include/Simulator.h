#pragma once

#include "Configs.h"
#include "IDrone.h"
#include "ILidarSensor.h"
#include "IMap3D.h"
#include "IMovementDriver.h"
#include "IPositionSensor.h"
#include "SimulationState.h"
#include "TrueMap.h"

class Simulator {

  IDrone *drone;
  TrueMap &true_map;
  IMap3D *map;
  ILidarSensor *lidar_sensor;
  IPositionSensor *position_sensor;
  IMovementDriver *movement_driver;
  SimulationState simulation_state;
  const MissionConfig &mission_config_;

  void setInitialPosition();

public:
  Simulator(IDrone &drone, TrueMap &true_map, IMap3D &map,
            ILidarSensor &lidar_sensor, IPositionSensor &position_sensor,
            IMovementDriver &movement_driver,
            const MissionConfig &mission_config);

  const MissionConfig &missionConfig() const noexcept {
    return mission_config_;
  }

  IMap3D &simulate();
};
