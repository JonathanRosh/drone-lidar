#pragma once

#include "domain/Units.h"

namespace interfaces {

class IMovementDriver {
public:
  virtual ~IMovementDriver() = default;

  virtual void rotateLeft(domain::Angle angle) = 0;
  virtual void rotateRight(domain::Angle angle) = 0;
  virtual void advance(domain::Length distance) = 0;
  virtual void elevate(domain::Length distance) = 0;
};

} // namespace interfaces
