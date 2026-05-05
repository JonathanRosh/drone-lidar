#pragma once

#include <mp-units/systems/si/unit_symbols.h>

namespace domain {

using Length = decltype(1.0 * mp_units::si::unit_symbols::cm);
using Angle = decltype(1.0 * mp_units::si::unit_symbols::deg);

} // namespace domain
