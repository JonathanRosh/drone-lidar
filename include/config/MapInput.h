#pragma once

#include "domain/Point3D.h"
#include <vector>

namespace config {

struct InputMap {
  std::vector<domain::Point3D> occupied_points;
};

} // namespace config
