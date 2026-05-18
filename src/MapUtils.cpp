#include "../include/MapUtils.h"

#include <iostream>

namespace MapUtils {

int toIndex(Distance value, Distance resolution) {
  return static_cast<int>(
        std::round(
            (value / resolution)
                .numerical_value_in(mp::one)
        )
    );
};

GridCoord worldToGrid(const Position3D& pos, const Distance res_xy, Distance res_height) {
  return {
        toIndex(pos.x, res_xy),
        toIndex(pos.y, res_xy),
        toIndex(pos.z, res_height)
    };
}

}