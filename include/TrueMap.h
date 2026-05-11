#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "IMap3D.h"
#include "Configs.h"

struct GridCoord {
    int x;
    int y;
    int z;

    bool operator==(const GridCoord&) const = default;
};

struct GridCoordHash {
    std::size_t operator()(const GridCoord& c) const {
        std::size_t h1 = std::hash<int>{}(c.x);
        std::size_t h2 = std::hash<int>{}(c.y);
        std::size_t h3 = std::hash<int>{}(c.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class TrueMap : public IMap3D {

    TrueMap(MissionConfig& mission_config);

    std::unordered_map<GridCoord, Mapping, GridCoordHash> cells;
    MissionConfig& config;

    void set(const Position3D& pos, const Mapping mapping);
    GridCoord worldToGrid(const Position3D& pos) const;
    bool isInsideBounds(const Position3D& pos) const;
    Distance resolutionToDistance(const unsigned int res) const; // the res is number of places after the number of decimal places after the point

public:
    Mapping get(const Position3D& pos) const override;
};

#endif //DRONE_LIDAR_TRUEMAP_H
