#include "../include/TrueMap.h"
#include "../include/MapGrid.h"

#include <iostream>
#include <cmath>

TrueMap::TrueMap(const MissionConfig& mission_config) : 
    config(mission_config), 
    res_xy(map_grid::xyResolution(mission_config)),
    res_height(map_grid::zResolution(mission_config))
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

Position3D TrueMap::gridToWorld(const GridCoord& grid) const {
    return map_grid::gridToWorld(grid, res_xy, res_height);
}

bool TrueMap::isInsideBounds(const Position3D& pos) const {
    return map_grid::insideMissionBounds(pos, config);
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

std::optional<Position3D> TrueMap::firstUnoccupiedPositionLexOrder() const {
    const auto& b = config.map_boundry;

    const double rx = res_xy.numerical_value_in(cm);
    const double rz = res_height.numerical_value_in(cm);

    const double xmin = b.minX.numerical_value_in(cm);
    const double xmax = b.maxX.numerical_value_in(cm);
    const double ymin = b.minY.numerical_value_in(cm);
    const double ymax = b.maxY.numerical_value_in(cm);
    const double zmin = b.minHeight.numerical_value_in(cm);
    const double zmax = b.maxHeight.numerical_value_in(cm);

    const int ix_lo = static_cast<int>(std::ceil(xmin / rx - 1e-12));
    const int ix_hi = static_cast<int>(std::floor(xmax / rx + 1e-12));
    const int iy_lo = static_cast<int>(std::ceil(ymin / rx - 1e-12));
    const int iy_hi = static_cast<int>(std::floor(ymax / rx + 1e-12));
    const int iz_lo = static_cast<int>(std::ceil(zmin / rz - 1e-12));
    const int iz_hi = static_cast<int>(std::floor(zmax / rz + 1e-12));

    for (int ix = ix_lo; ix <= ix_hi; ++ix) {
        for (int iy = iy_lo; iy <= iy_hi; ++iy) {
            for (int iz = iz_lo; iz <= iz_hi; ++iz) {
                const Position3D p = gridToWorld({ix, iy, iz});
                if (!isInsideBounds(p)) {
                    continue;
                }
                if (get(p) != OCCUPIED) {
                    return p;
                }
            }
        }
    }

    return std::nullopt;
}