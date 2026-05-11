#ifndef DRONE_LIDAR_UNITS_H
#define DRONE_LIDAR_UNITS_H

#include <mp-units/framework.h>
#include <mp-units/systems/si/unit_symbols.h>

namespace mp = mp_units;

using mp::si::unit_symbols::deg;
using mp::si::unit_symbols::cm;

using Distance =  mp::quantity<cm, double>;
using Length = mp::quantity<cm, double>;
using Angle = mp::quantity<deg, double>;

struct Orientation {
    Angle azimuth;
    Angle elevation;
};

// TODO: add strong types for x, y, z like in the example
struct Position3D {
    Distance x;
    Distance y;
    Distance z;
};

#endif //DRONE_LIDAR_UNITS_H
