#include "../include/MapCore.h"
#include "../include/MapUtils.h"

MapCore::MapCore(const MissionConfig &mission_config, Mapping default_in_bounds)
    : config_(mission_config),
      res_xy_(MapUtils::xyResolution(mission_config)),
      res_height_(MapUtils::zResolution(mission_config)),
      default_in_bounds_(default_in_bounds) {}

GridCoord MapCore::worldToGrid(const Position3D &pos) const {
  return MapUtils::worldToGrid(pos, res_xy_, res_height_);
}

Position3D MapCore::gridToWorld(const GridCoord &grid) const {
  return MapUtils::gridToWorld(grid, res_xy_, res_height_);
}

void MapCore::setCell(const Position3D &pos, Mapping mapping) {
  if (!isInsideBounds(pos)) {
    return;
  }
  cells_[worldToGrid(pos)] = mapping;
}

Mapping MapCore::get(const Position3D &pos) const {
  if (!isInsideBounds(pos)) {
    return OUTSIDE_BOUNDARY;
  }

  const auto it = cells_.find(worldToGrid(pos));
  if (it == cells_.end()) {
    return default_in_bounds_;
  }
  return it->second;
}

bool MapCore::isInsideBounds(const Position3D &pos) const {
  return MapUtils::insideMissionBounds(pos, config_);
}
