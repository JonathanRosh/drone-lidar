#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "IMap3D.h"
#include "Configs.h"
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
    bool isInsideBounds(const Position3D& pos) const;
    Distance resolutionToDistance(const unsigned int res) const; // the res is number of places after the number of decimal places after the point
    void set(const Position3D& pos, const Mapping mapping);

    friend class TrueMapBuilder;
    
public:
    TrueMap(const MissionConfig& mission_config);
    Mapping get(const Position3D& pos) const override;
};

#endif //DRONE_LIDAR_TRUEMAP_H
