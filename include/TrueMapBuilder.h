#ifndef DRONE_LIDAR_TRUEMAPBUILDER_H
#define DRONE_LIDAR_TRUEMAPBUILDER_H

#include "Units.h"
#include "IMap3D.h"

class TrueMap;

class TrueMapBuilder {
public:
  static void set(TrueMap& map, Position3D pos, Mapping val); 
};

#endif // DRONE_LIDAR_TRUEMAPBUILDER_H