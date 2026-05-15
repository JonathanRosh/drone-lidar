#pragma once

#include "Configs.h"
#include "IDrone.h"
#include "ILidarSensor.h"
#include "IMap3D.h"
#include "IMovementDriver.h"
#include "IPositionSensor.h"
#include "GridCoord.h"
#include "SimulationState.h"
#include "TrueMap.h"

#include <unordered_map>

class Simulator {

  IDrone *drone;
  TrueMap &true_map;
  IMap3D *map;
  ILidarSensor *lidar_sensor;
  IPositionSensor *position_sensor;
  IMovementDriver *movement_driver;
  SimulationState simulation_state;
  const MissionConfig &mission_config_;
  const DroneConfig &drone_config_;

  /** Failed attempts per frontier grid cell; cooled cells are skipped in BFS. */
  std::unordered_map<GridCoord, int, GridCoordHash> frontier_failure_counts_;

  void setInitialPosition();

public:
  Simulator(IDrone &drone, TrueMap &true_map, IMap3D &map,
            ILidarSensor &lidar_sensor, IPositionSensor &position_sensor,
            IMovementDriver &movement_driver,
            const MissionConfig &mission_config,
            const DroneConfig &drone_config);

  const MissionConfig &missionConfig() const noexcept {
    return mission_config_;
  }

  const DroneConfig &droneConfig() const noexcept { return drone_config_; }

  IMap3D &simulate();
};
