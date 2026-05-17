#include "../include/TrueMap.h"

#include <iostream> // TODO: maybe remove this?

TrueMap::TrueMap(const MissionConfig& mission_config) : 
    config(mission_config), 
    res_xy(resolutionToDistance(config.map_resolution.xy_resolution)),
    res_height(resolutionToDistance(config.map_resolution.height_resolution))
    {};

void TrueMap::set(const Position3D& pos, Mapping mapping) {
    if (!isInsideBounds(pos)) {
        std::cerr << "cannot set position outside of map boundry \n" << std::endl;
        return;
    }

    auto grid = worldToGrid(pos);

    cells[grid] = mapping;
}

GridCoord TrueMap::worldToGrid(const Position3D& pos) const {
    return MapUtils::worldToGrid(pos, res_xy, res_height);
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
    return std::pow(10.0, -static_cast<int>(res)) * cm;
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