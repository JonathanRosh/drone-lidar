#ifndef DRONE_LIDAR_POSITIONSENSORMOCK_H
#define DRONE_LIDAR_POSITIONSENSORMOCK_H

#include "IPositionSensor.h"
#include "SimulationState.h"

class PositionSensorMock : public IPositionSensor {

    SimulationState& sim_state;

public:
    PositionSensorMock(SimulationState& sim_state);

    Position3D getPosition() const override;
    Orientation getOrientation() const override;

};

#endif //DRONE_LIDAR_POSITIONSENSORMOCK_H
