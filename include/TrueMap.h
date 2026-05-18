#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "Configs.h"
#include "GridCoord.h"
#include "IMap3D.h"
#include "TrueMapBuilder.h"
#include "MapUtils.h"

struct GridCoordHash {
    std::size_t operator()(const GridCoord& c) const {
        std::size_t h1 = std::hash<int>{}(c.x);
        std::size_t h2 = std::hash<int>{}(c.y);
        std::size_t h3 = std::hash<int>{}(c.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

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
};

#endif //DRONE_LIDAR_TRUEMAP_H
