#ifndef DRONE_LIDAR_MAPUTILS_H
#define DRONE_LIDAR_MAPUTILS_H

#include "Units.h"
#include "GridCoord.h"

namespace MapUtils {

int toIndex(Distance value, Distance resolution);

GridCoord worldToGrid(const Position3D& pos, const Distance res_xy, Distance res_height);

}

#endif //DRONE_LIDAR_MAPUTILS_H