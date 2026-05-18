#include "../include/DroneMap.h"
#include "../include/MapGrid.h"

DroneMap::DroneMap(const MissionConfig &mission_config)
    : config_(mission_config),
      res_xy_(map_grid::xyResolution(mission_config)),
      res_height_(map_grid::zResolution(mission_config)) {}

GridCoord DroneMap::worldToGrid(const Position3D &pos) const {
  return map_grid::worldToGrid(pos, res_xy_, res_height_);
}

Position3D DroneMap::gridToWorld(const GridCoord &grid) const {
  return map_grid::gridToWorld(grid, res_xy_, res_height_);
}

bool DroneMap::isInsideBounds(const Position3D &pos) const {
  return map_grid::insideMissionBounds(pos, config_);
}

void DroneMap::set(const Position3D &pos, Mapping mapping) {
  if (!isInsideBounds(pos)) {
    return;
  }
  cells_[worldToGrid(pos)] = mapping;
}

Mapping DroneMap::get(const Position3D &pos) const {
  if (!isInsideBounds(pos)) {
    return OUTSIDE_BOUNDARY;
  }
  const auto it = cells_.find(worldToGrid(pos));
  if (it == cells_.end()) {
    return NOT_MAPPED;
  }
  return it->second;
}
