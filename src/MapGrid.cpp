#include "../include/MapGrid.h"

#include <cmath>
#include <mp-units/framework.h>

namespace mp = mp_units;

namespace map_grid {

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

GridCoord worldToGrid(const Position3D &position, Distance res_xy,
                      Distance res_z) {
  return {toGridIndex(position.x, res_xy), toGridIndex(position.y, res_xy),
          toGridIndex(position.z, res_z)};
}

Position3D gridToWorld(const GridCoord &grid, Distance res_xy, Distance res_z) {
  const double fx = static_cast<double>(grid.x) * res_xy.numerical_value_in(cm);
  const double fy = static_cast<double>(grid.y) * res_xy.numerical_value_in(cm);
  const double fz = static_cast<double>(grid.z) * res_z.numerical_value_in(cm);

  Position3D p{};
  p.x = fx * cm;
  p.y = fy * cm;
  p.z = fz * cm;
  return p;
}

bool insideMissionBounds(const Position3D &position,
                         const MissionConfig &mission_config) {
  const auto &b = mission_config.map_boundry;
  return position.x >= b.minX && position.x <= b.maxX &&
         position.y >= b.minY && position.y <= b.maxY &&
         position.z >= b.minHeight && position.z <= b.maxHeight;
}

} // namespace map_grid
