#include "../include/MovementDriverMock.h"

#include <gtest/gtest.h>

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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

    driver.rotateRight(20.0 * deg);

    EXPECT_NEAR(state.drone_orientation.horizontal.force_numerical_value_in(deg), -20.0, 1e-6);
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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

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

    MovementDriverMock driver(state, limits);

    driver.elevate(20.0 * cm);

    EXPECT_NEAR(state.drone_position.z.force_numerical_value_in(cm), 5.0, 1e-6);
}