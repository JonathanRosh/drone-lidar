#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "MapCore.h"
#include "TrueMapBuilder.h"

class TrueMap : public MapCore {
  friend class TrueMapBuilder;
    
public:
  explicit TrueMap(const MissionConfig &mission_config);
};

#endif //DRONE_LIDAR_TRUEMAP_H
