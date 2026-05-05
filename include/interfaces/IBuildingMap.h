#pragma once

#include "domain/MapCellState.h"
#include "domain/Point3D.h"

namespace interfaces {

class IBuildingMap {
public:
  virtual ~IBuildingMap() = default;

  virtual void setCell(const domain::Point3D &point, domain::MapCellState state) = 0;
  virtual domain::MapCellState getCell(const domain::Point3D &point) const = 0;
};

} // namespace interfaces
