#pragma once

#include "Configs.h"
#include "GridCoord.h"
#include "IMap3D.h"

#include <unordered_map>

class MapCore : public virtual IMap3D {
  std::unordered_map<GridCoord, Mapping, GridCoordHash> cells_;
  const MissionConfig &config_;
  const Distance res_xy_;
  const Distance res_height_;

protected:
  GridCoord worldToGrid(const Position3D &pos) const;
  Position3D gridToWorld(const GridCoord &grid) const;
  void setCell(const Position3D &pos, Mapping mapping);

public:
  explicit MapCore(const MissionConfig &mission_config);

  Mapping get(const Position3D &pos) const override;
  bool isInsideBounds(const Position3D &pos) const override;
};
