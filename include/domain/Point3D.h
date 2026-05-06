#pragma once

#include "domain/Units.h"

namespace domain {

struct Point3D {
  Length x = 0.0 * mp_units::si::unit_symbols::cm;
  Length y = 0.0 * mp_units::si::unit_symbols::cm;
  Length height = 0.0 * mp_units::si::unit_symbols::cm;
};

} // namespace domain
