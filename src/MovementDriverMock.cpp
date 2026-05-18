#include "../include/MovementDriverMock.h"

#include <mp-units/systems/si/math.h>

Distance clampDistance(Distance value, Distance max) {
    const auto v = value.force_numerical_value_in(cm);
    const auto m = max.force_numerical_value_in(cm);

    return Distance{std::min(v, m) * cm};
}

HorizontalAngle clampAngle(HorizontalAngle value, Angle max)
{
    const auto v = value.force_numerical_value_in(deg);
    const auto m = max.force_numerical_value_in(deg);

    return HorizontalAngle{std::min(v, m) * deg};
}

MovementDriverMock::MovementDriverMock(SimulationState& sim_state, MaxCommand& limits)
    : sim_state(sim_state), limits(limits) {}

// TODO: maybe will need to normalize the angles
void MovementDriverMock::rotateLeft(HorizontalAngle angle) const {
    const auto limited = clampAngle(angle, limits.maxRotate);
    sim_state.drone_orientation.horizontal += limited;
}

void MovementDriverMock::rotateRight(HorizontalAngle angle) const {
    const auto limited = clampAngle(angle, limits.maxRotate);
    sim_state.drone_orientation.horizontal -= limited;
}

void MovementDriverMock::advance(Distance distance) const
{
    const auto limited = clampDistance(distance, limits.maxAdvance);

    const auto heading = sim_state.drone_orientation.horizontal;

    const auto dx = si::cos(heading);
    const auto dy = si::sin(heading);

    sim_state.drone_position.x += XLength{
        dx * limited.force_numerical_value_in(cm) * cm
    };

    sim_state.drone_position.y += YLength{
        dy * limited.force_numerical_value_in(cm) * cm
    };
}

void MovementDriverMock::elevate(Distance distance) const
{
    const auto limited = clampDistance(distance, limits.maxElevate);

    sim_state.drone_position.z += ZLength{
        limited.force_numerical_value_in(cm) * cm
    };
}