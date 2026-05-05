#pragma once

#include "domain/Units.h"

namespace domain {

struct Position {
  Length x = 0.0 * mp_units::si::unit_symbols::cm;
  Length y = 0.0 * mp_units::si::unit_symbols::cm;
  Length height = 0.0 * mp_units::si::unit_symbols::cm;
  Angle xy_angle = 0.0 * mp_units::si::unit_symbols::deg;
};

} // namespace domain
