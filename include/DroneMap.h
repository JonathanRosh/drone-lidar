#pragma once

#include "Configs.h"
#include "GridCoord.h"
#include "IMap3D.h"

#include <ostream>
#include <unordered_map>

class DroneMap : public IMap3D {
  std::unordered_map<GridCoord, Mapping, GridCoordHash> cells_;
  const MissionConfig &config_;
  const Distance res_xy_;
  const Distance res_height_;

  GridCoord worldToGrid(const Position3D &pos) const;
  Position3D gridToWorld(const GridCoord &grid) const;

public:
  explicit DroneMap(const MissionConfig &mission_config);

  Mapping get(const Position3D &pos) const override;
  void set(const Position3D &pos, Mapping val) override;
  bool isInsideBounds(const Position3D &pos) const override;

  void writeOccupied(std::ostream &out) const;
  std::size_t occupiedCount() const;
};
