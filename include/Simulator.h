#pragma once

#include "Configs.h"
#include "Drone.h"
#include "IMap3D.h"
#include "LidarSensorMock.h"
#include "MovementDriverMock.h"
#include "PositionSensorMock.h"
#include "SimulationState.h"
#include "TrueMap.h"

class Simulator {
  const DroneConfig &drone_config_;
  const MissionConfig &mission_config_;
  const TrueMap &true_map_;

  SimulationState sim_state_;
  TrueMap mapped_map_;
  PositionSensorMock position_sensor_;
  MovementDriverMock movement_driver_;
  LidarSensorMock lidar_sensor_;
  Drone drone_;

public:
  Simulator(const DroneConfig &drone_config,
            const MissionConfig &mission_config,
            const TrueMap &true_map);

  IMap3D& simulate();
};
