#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "Configs.h"
#include "GridCoord.h"
#include "IMap3D.h"
#include "TrueMapBuilder.h"
#include "MapUtils.h"

#include <functional>

class TrueMap : public IMap3D {

    
    std::unordered_map<GridCoord, Mapping, GridCoordHash> cells;
    const MissionConfig& config;
    const Distance res_xy;
    const Distance res_height;
    
    GridCoord worldToGrid(const Position3D& pos) const;
    Position3D gridToWorld(const GridCoord& grid) const;

    friend class TrueMapBuilder;
    
public:
    TrueMap(const MissionConfig& mission_config);
    Mapping get(const Position3D& pos) const override;
    void set(const Position3D& pos, Mapping mapping) override;
    bool isInsideBounds(const Position3D& pos) const override;

    const MissionConfig& missionConfig() const noexcept { return config; }

    /** First in-bounds cell in lex order (x, then y, then z) whose mapping is not OCCUPIED. */
    std::optional<Position3D> firstUnoccupiedPositionLexOrder() const;

    /** Start pose near occupied structure (for simulation bootstrap). */
    std::optional<Position3D> startPositionNearStructure() const;

    /** Invokes fn for each explicitly stored cell (sparse occupied points). */
    void forEachStoredCell(
        const std::function<void(const Position3D &, Mapping)> &fn) const;
};

#endif //DRONE_LIDAR_TRUEMAP_H
