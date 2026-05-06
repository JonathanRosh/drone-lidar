#ifndef DRONE_LIDAR_DRONEMAP_H
#define DRONE_LIDAR_DRONEMAP_H

#include "IMap3D.h"

class DroneMap : public IMap3D {

    // TODO

public:
    Mapping get(const Position3D& pos) const override;
    void set(const Position3D& pos, Mapping val);
    IMap3D& getMap();
};

#endif //DRONE_LIDAR_DRONEMAP_H
