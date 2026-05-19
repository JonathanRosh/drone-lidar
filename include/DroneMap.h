#pragma once

#include "IMap3D.h"
#include "MapCore.h"

class DroneMap : public MapCore, public IWritableMap3D {
public:
  explicit DroneMap(const MissionConfig &mission_config);

  void set(const Position3D &pos, Mapping val) override;
};
