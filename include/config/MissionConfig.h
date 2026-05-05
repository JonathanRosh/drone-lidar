#pragma once

#include "domain/Position.h"

namespace config {

struct MissionConfig {
  struct MapBoundary {
    domain::Length x_min = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length x_max = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length y_min = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length y_max = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length height_min = 0.0 * mp_units::si::unit_symbols::cm;
    domain::Length height_max = 0.0 * mp_units::si::unit_symbols::cm;
  };
  MapBoundary map_boundary;

  domain::Position start_position;

  struct Resolution {
    int xy = 0;
    int height = 0;
  };
  Resolution resolution;
};

} // namespace config
