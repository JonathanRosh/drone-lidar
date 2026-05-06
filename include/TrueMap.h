#ifndef DRONE_LIDAR_TRUEMAP_H
#define DRONE_LIDAR_TRUEMAP_H

#include "IMap3D.h"

class TrueMap : public IMap3D {

    // TODO

public:
    Mapping get(const Position3D& pos) const override;
};

#endif //DRONE_LIDAR_TRUEMAP_H
