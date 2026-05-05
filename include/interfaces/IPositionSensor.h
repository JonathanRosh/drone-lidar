#pragma once

#include "domain/Position.h"

namespace interfaces {

class IPositionSensor {
public:
  virtual ~IPositionSensor() = default;
  virtual domain::Position getPosition() const = 0;
};

} // namespace interfaces
