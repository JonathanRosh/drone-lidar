#include "../include/MovementDriverMock.h"
#include "../include/IMap3D.h"

#include <gtest/gtest.h>
#include <string>

class MovementTestMap : public IMap3D {
    bool occupied_from_five_;

public:
    explicit MovementTestMap(bool occupied_from_five = false)
        : occupied_from_five_(occupied_from_five) {}

    Mapping get(const Position3D& pos) const override {
        if (!isInsideBounds(pos)) {
            return OUTSIDE_BOUNDARY;
        }
        if (occupied_from_five_ && pos.x >= 5.0 * x_extent[cm]) {
            return OCCUPIED;
        }
        return EMPTY;
    }

    bool isInsideBounds(const Position3D& pos) const override {
        return pos.x >= -1000.0 * x_extent[cm] && pos.x <= 1000.0 * x_extent[cm] &&
               pos.y >= -1000.0 * y_extent[cm] && pos.y <= 1000.0 * y_extent[cm] &&
               pos.z >= -1000.0 * z_extent[cm] && pos.z <= 1000.0 * z_extent[cm];
    }
};

const MissionConfig wide_mission_config{
    .map_boundry = {
        .minX = -1000.0 * cm,
        .minY = -1000.0 * cm,
        .maxX = 1000.0 * cm,
        .maxY = 1000.0 * cm,
        .maxHeight = 1000.0 * cm,
        .minHeight = -1000.0 * cm
    },
    .map_resolution = {.xy_resolution = 0, .height_resolution = 0}
};

const MinPass no_clearance{
    .width = 0.0 * cm,
    .height = 0.0 * cm,
    .length = 0.0 * cm
};

TEST(MovementDriverMockTest, AdvanceMovesForwardAlongXAxis)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.advance(10.0 * cm);

    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 10.0, 1e-6);
    EXPECT_NEAR(state.drone_position.y.force_numerical_value_in(cm), 0.0, 1e-6);
}

TEST(MovementDriverMockTest, AdvanceRespectsYawRotation)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {90.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.advance(10.0 * cm);

    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 0.0, 1e-6);
    EXPECT_NEAR(state.drone_position.y.force_numerical_value_in(cm), 10.0, 1e-6);
}

TEST(MovementDriverMockTest, AdvanceIsClampedToMax)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 5.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.advance(20.0 * cm); // exceeds max

    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 5.0, 1e-6);
}

TEST(MovementDriverMockTest, RotateLeftChangesHeading)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.rotateLeft(20.0 * deg);

    EXPECT_NEAR(state.drone_orientation.horizontal.force_numerical_value_in(deg), 20.0, 1e-6);
}

TEST(MovementDriverMockTest, RotateRightChangesHeading)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.rotateRight(20.0 * deg);

    EXPECT_NEAR(state.drone_orientation.horizontal.force_numerical_value_in(deg), 340.0, 1e-6);
}

TEST(MovementDriverMockTest, RotateLeftIsClamped)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 30.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.rotateLeft(90.0 * deg); // request exceeds limit

    EXPECT_NEAR(
        state.drone_orientation.horizontal.force_numerical_value_in(deg),
        30.0,
        1e-6
    );
}

TEST(MovementDriverMockTest, ElevateMovesUpwards)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.elevate(10.0 * cm);

    EXPECT_NEAR(state.drone_position.z.force_numerical_value_in(cm), 10.0, 1e-6);
}

TEST(MovementDriverMockTest, ElevateIsClamped)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 5.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.elevate(20.0 * cm);

    EXPECT_NEAR(state.drone_position.z.force_numerical_value_in(cm), 5.0, 1e-6);
}

TEST(MovementDriverMockTest, NegativeAdvanceIsClampedToMax)
{
    SimulationState state{
        .drone_position = {10.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 5.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.advance(-20.0 * cm);

    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 5.0, 1e-6);
}

TEST(MovementDriverMockTest, AdvanceFailsWhenPathWouldCollide)
{
    const MovementTestMap map(true);
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    MovementDriverMock driver(state, limits, map, wide_mission_config, no_clearance);

    driver.advance(10.0 * cm);

    EXPECT_TRUE(state.failed);
    EXPECT_NE(std::string::npos, state.failure_reason.find("collide"));
    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 0.0, 1e-6);
}

TEST(MovementDriverMockTest, AdvanceFailsWhenLeavingMissionBoundary)
{
    SimulationState state{
        .drone_position = {19.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MissionConfig mission_config{
        .map_boundry = {
            .minX = 0.0 * cm,
            .minY = -10.0 * cm,
            .maxX = 20.0 * cm,
            .maxY = 10.0 * cm,
            .maxHeight = 10.0 * cm,
            .minHeight = -10.0 * cm
        },
        .map_resolution = {.xy_resolution = 0, .height_resolution = 0}
    };
    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, mission_config, no_clearance);

    driver.advance(5.0 * cm);

    EXPECT_TRUE(state.failed);
    EXPECT_NE(std::string::npos, state.failure_reason.find("boundaries"));
    EXPECT_NEAR(state.drone_position.x.force_numerical_value_in(cm), 19.0, 1e-6);
}

TEST(MovementDriverMockTest, ElevateFailsWhenDroneClearanceWouldCrossCeiling)
{
    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 8.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg}
    };

    MissionConfig mission_config{
        .map_boundry = {
            .minX = -10.0 * cm,
            .minY = -10.0 * cm,
            .maxX = 10.0 * cm,
            .maxY = 10.0 * cm,
            .maxHeight = 10.0 * cm,
            .minHeight = 0.0 * cm
        },
        .map_resolution = {.xy_resolution = 0, .height_resolution = 0}
    };
    const MinPass min_pass{
        .width = 4.0 * cm,
        .height = 4.0 * cm,
        .length = 4.0 * cm
    };

    MaxCommand limits{
        .maxAdvance = 100.0 * cm,
        .maxElevate = 100.0 * cm,
        .maxRotate  = 180.0 * deg
    };

    const MovementTestMap map;
    MovementDriverMock driver(state, limits, map, mission_config, min_pass);

    driver.elevate(1.0 * cm);

    EXPECT_TRUE(state.failed);
    EXPECT_NE(std::string::npos, state.failure_reason.find("boundaries"));
    EXPECT_NEAR(state.drone_position.z.force_numerical_value_in(cm), 8.0, 1e-6);
}
