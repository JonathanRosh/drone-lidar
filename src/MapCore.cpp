#include "../include/MapCore.h"

#include <cmath>
#include <mp-units/framework.h>

namespace mp = mp_units;

namespace {

Distance resolutionFromDecimalPlaces(unsigned int decimal_places_after_point) {
  return std::pow(10.0, -static_cast<int>(decimal_places_after_point)) * cm;
}

Distance xyResolution(const MissionConfig &mission_config) {
  return resolutionFromDecimalPlaces(
      mission_config.map_resolution.xy_resolution);
}

Distance zResolution(const MissionConfig &mission_config) {
  return resolutionFromDecimalPlaces(
      mission_config.map_resolution.height_resolution);
}

int toGridIndex(Distance value, Distance resolution) {
  return static_cast<int>(
      std::round((value / resolution).numerical_value_in(mp::one)));
}

bool insideMissionBounds(const Position3D &position,
                         const MissionConfig &mission_config) {
  const auto &b = mission_config.map_boundry;
  return position.x >= b.minX && position.x <= b.maxX &&
         position.y >= b.minY && position.y <= b.maxY &&
         position.z >= b.minHeight && position.z <= b.maxHeight;
}

} // namespace

MapCore::MapCore(const MissionConfig &mission_config)
    : config_(mission_config),
      res_xy_(xyResolution(mission_config)),
      res_height_(zResolution(mission_config)) {}

GridCoord MapCore::worldToGrid(const Position3D &pos) const {
  return {toGridIndex(pos.x, res_xy_), toGridIndex(pos.y, res_xy_),
          toGridIndex(pos.z, res_height_)};
}

Position3D MapCore::gridToWorld(const GridCoord &grid) const {
  const double x = static_cast<double>(grid.x) * res_xy_.numerical_value_in(cm);
  const double y = static_cast<double>(grid.y) * res_xy_.numerical_value_in(cm);
  const double z =
      static_cast<double>(grid.z) * res_height_.numerical_value_in(cm);

  return {x * cm, y * cm, z * cm};
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
    return NOT_MAPPED;
  }
  return it->second;
}

bool MapCore::isInsideBounds(const Position3D &pos) const {
  return insideMissionBounds(pos, config_);
}
