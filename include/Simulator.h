#pragma once

#include "Configs.h"
#include "Drone.h"
#include "DroneMap.h"
#include "GridCoord.h"
#include "IMap3D.h"
#include "LidarSensorMock.h"
#include "MovementDriverMock.h"
#include "PositionSensorMock.h"
#include "SimulationState.h"
#include "TrueMap.h"

#include <optional>
#include <vector>

class Simulator {
  const DroneConfig &drone_config_;
  const MissionConfig &mission_config_;
  const TrueMap &true_map_;

  SimulationState sim_state_;
  DroneMap mapped_map_;
  PositionSensorMock position_sensor_;
  MovementDriverMock movement_driver_;
  LidarSensorMock lidar_sensor_;
  Drone drone_;

  Distance res_xy_;
  Distance res_z_;

  GridCoord worldToGrid(const Position3D &position) const;
  Position3D gridToWorld(const GridCoord &grid) const;
  std::vector<GridCoord> neighbors(const GridCoord &grid) const;
  bool isFrontier(const GridCoord &grid) const;
  std::optional<std::vector<GridCoord>> pathToNearestFrontier() const;
  void scanCurrentPosition();
  void integrateHit(const Position3D &origin, const Orientation &drone_orientation,
                    const LidarHit &hit);
  void moveAlongPath(const std::vector<GridCoord> &path);
  void moveToAdjacent(const GridCoord &from, const GridCoord &to);
  void rotateTo(HorizontalAngle target);

public:
  Simulator(const DroneConfig &drone_config,
            const MissionConfig &mission_config,
            const TrueMap &true_map);

  IMap3D& simulate();
};
