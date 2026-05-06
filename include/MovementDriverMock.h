#ifndef DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
#define DRONE_LIDAR_MOVEMENTDRIVERMOCK_H

#include "IMovementDriver.h"
#include "SimulationState.h"
#include "Units.h"

class MovementDriverMock : public IMovementDriver {

    SimulationState& sim_state;

public:
    MovementDriverMock(SimulationState& sim_state);

    void rotateLeft(Angle angle) const override;
    void rotateRight(Angle angle) const override;
    void advance(Distance distance) const override;
    void elevate(Distance distance) const override;
};


#endif //DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
