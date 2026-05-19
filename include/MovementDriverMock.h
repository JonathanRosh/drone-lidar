#ifndef DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
#define DRONE_LIDAR_MOVEMENTDRIVERMOCK_H

#include "IMap3D.h"
#include "IMovementDriver.h"
#include "SimulationState.h"
#include "Units.h"
#include "Configs.h"

class MovementDriverMock : public IMovementDriver {

    SimulationState& sim_state;
    const MaxCommand& limits;
    const IMap3D& simulation_map;
    const MissionConfig& mission_config;
    Distance clearance_radius;

    bool validatePath(const Position3D& from, const Position3D& to) const;
    bool validatePosition(const Position3D& position) const;
    void fail(const char* reason) const;

public:
    MovementDriverMock(SimulationState& sim_state, const MaxCommand& limits,
                       const IMap3D& simulation_map,
                       const MissionConfig& mission_config,
                       const MinPass& min_pass);

    void rotateLeft(HorizontalAngle angle) const override;
    void rotateRight(HorizontalAngle angle) const override;
    void advance(Distance distance) const override;
    void elevate(Distance distance) const override;
};


#endif //DRONE_LIDAR_MOVEMENTDRIVERMOCK_H
