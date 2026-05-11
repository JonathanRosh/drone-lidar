#include "../include/TrueMap.h"

#include <iostream>

int toIndex(Distance value, Distance resolution) {
    return static_cast<int>(
        std::floor(
            (value / resolution)
                .numerical_value_in(mp::one)
        )
    );
}

TrueMap::TrueMap(MissionConfig& mission_config) : config(mission_config) {};

void TrueMap::set(const Position3D& pos, Mapping mapping) {
    if (!isInsideBounds(pos)) {
        std::cerr << "cannot set position outside of map boundry \n" << std::endl;
        return;
    }

    auto grid = worldToGrid(pos);

    cells[grid] = mapping;
}

GridCoord TrueMap::worldToGrid(const Position3D& pos) const {
    return {
        toIndex(pos.x, resolutionToDistance(config.map_resolution.xy_resolution)),
        toIndex(pos.y, resolutionToDistance(config.map_resolution.xy_resolution)),
        toIndex(pos.z, resolutionToDistance(config.map_resolution.height_resolution))
    };
}

bool TrueMap::isInsideBounds(const Position3D& pos) const {
    const auto& b = config.map_boundry;

    return
        pos.x >= b.minX &&
        pos.x <= b.maxX &&
        pos.y >= b.minY &&
        pos.y <= b.maxY &&
        pos.z >= b.minHeight &&
        pos.z <= b.maxHeight;
}

Distance TrueMap::resolutionToDistance(const unsigned int res) const {
    double meters_value =
        std::pow(10.0, -static_cast<int>(res));

    return meters_value * mp::si::metre;
}

Mapping TrueMap::get(const Position3D& pos) const {
    if (!isInsideBounds(pos)) {
        return OUTSIDE_BOUNDARY;
    }

    auto grid = worldToGrid(pos);

    auto it = cells.find(grid);

    if (it == cells.end()) {
        return NOT_MAPPED;
    }

    return it->second;
}