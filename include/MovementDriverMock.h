#ifndef DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
#define DRONE_LIDAR_MOVEMENTDRIVERMOCK_H

#include "IMovementDriver.h"
#include "SimulationState.h"
#include "Units.h"
#include "Configs.h"

class MovementDriverMock : public IMovementDriver {

    SimulationState& sim_state;
    MaxCommand& limits;

public:
    MovementDriverMock(SimulationState& sim_state, MaxCommand& limits);

    void rotateLeft(HorizontalAngle angle) const override;
    void rotateRight(HorizontalAngle angle) const override;
    void advance(Distance distance) const override;
    void elevate(Distance distance) const override;
};


#endif //DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
