#pragma once

#include "Configs.h"
#include "GridCoord.h"
#include "Units.h"

namespace MapUtils {

Distance resolutionFromDecimalPlaces(unsigned int decimal_places_after_point);
Distance xyResolution(const MissionConfig &mission_config);
Distance zResolution(const MissionConfig &mission_config);
int toGridIndex(Distance value, Distance resolution);
GridCoord worldToGrid(const Position3D &position, Distance res_xy,
                      Distance res_z);
Position3D gridToWorld(const GridCoord &grid, Distance res_xy, Distance res_z);
bool insideMissionBounds(const Position3D &position,
                         const MissionConfig &mission_config);

} // namespace MapUtils
