#include "../include/MovementDriverMock.h"

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si/math.h>

namespace {

Distance clampDistance(Distance value, Distance max) {
    const auto v = value.force_numerical_value_in(cm);
    const auto m = std::abs(max.force_numerical_value_in(cm));
    const auto limited = std::clamp(v, -m, m);

    return Distance{limited * cm};
}

HorizontalAngle clampAngle(HorizontalAngle value, Angle max)
{
    const auto v = value.force_numerical_value_in(deg);
    const auto m = std::abs(max.force_numerical_value_in(deg));
    const auto limited = std::clamp(v, -m, m);

    return HorizontalAngle{limited * deg};
}

HorizontalAngle normalizeHorizontal(HorizontalAngle angle) {
    double value = std::fmod(angle.force_numerical_value_in(deg), 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return HorizontalAngle{value * deg};
}

Distance clearanceRadius(const MinPass& min_pass) {
    const double width = min_pass.width.force_numerical_value_in(cm);
    const double height = min_pass.height.force_numerical_value_in(cm);
    const double length = min_pass.length.force_numerical_value_in(cm);
    const double diameter = std::max({width, height, length});
    return Distance{(diameter / 2.0) * cm};
}

} // namespace

MovementDriverMock::MovementDriverMock(SimulationState& sim_state,
                                       const MaxCommand& limits,
                                       const IMap3D& simulation_map,
                                       const MissionConfig& mission_config,
                                       const MinPass& min_pass)
    : sim_state(sim_state),
      limits(limits),
      simulation_map(simulation_map),
      mission_config(mission_config),
      clearance_radius(clearanceRadius(min_pass)) {}

void MovementDriverMock::fail(const char* reason) const {
    sim_state.failed = true;
    sim_state.failure_reason = reason;
}

bool MovementDriverMock::validatePosition(const Position3D& position) const {
    const double r = clearance_radius.force_numerical_value_in(cm);
    const double offsets[] = {-r, 0.0, r};

    for (double dx : offsets) {
        for (double dy : offsets) {
            for (double dz : offsets) {
                const Position3D sample{
                    position.x + dx * x_extent[cm],
                    position.y + dy * y_extent[cm],
                    position.z + dz * z_extent[cm],
                };

                const auto& b = mission_config.map_boundry;
                if (sample.x < b.minX || sample.x > b.maxX ||
                    sample.y < b.minY || sample.y > b.maxY ||
                    sample.z < b.minHeight || sample.z > b.maxHeight) {
                    fail("movement would leave the mission boundaries");
                    return false;
                }

                if (!simulation_map.isInsideBounds(sample)) {
                    fail("movement would place the drone outside map bounds");
                    return false;
                }

                if (simulation_map.get(sample) == OCCUPIED) {
                    fail("movement would collide with an occupied cell");
                    return false;
                }
            }
        }
    }

    return true;
}

bool MovementDriverMock::validatePath(const Position3D& from, const Position3D& to) const {
    const double dx = to.x.force_numerical_value_in(cm) - from.x.force_numerical_value_in(cm);
    const double dy = to.y.force_numerical_value_in(cm) - from.y.force_numerical_value_in(cm);
    const double dz = to.z.force_numerical_value_in(cm) - from.z.force_numerical_value_in(cm);
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int steps = std::max(1, static_cast<int>(std::ceil(length)));

    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Position3D sample{
            from.x + (dx * t) * x_extent[cm],
            from.y + (dy * t) * y_extent[cm],
            from.z + (dz * t) * z_extent[cm],
        };

        if (!validatePosition(sample)) {
            return false;
        }
    }

    return true;
}

void MovementDriverMock::rotateLeft(HorizontalAngle angle) const {
    if (sim_state.failed) {
        return;
    }

    const auto limited = clampAngle(angle, limits.maxRotate);
    sim_state.drone_orientation.horizontal =
        normalizeHorizontal(sim_state.drone_orientation.horizontal + limited);
}

void MovementDriverMock::rotateRight(HorizontalAngle angle) const {
    if (sim_state.failed) {
        return;
    }

    const auto limited = clampAngle(angle, limits.maxRotate);
    sim_state.drone_orientation.horizontal =
        normalizeHorizontal(sim_state.drone_orientation.horizontal - limited);
}

void MovementDriverMock::advance(Distance distance) const
{
    if (sim_state.failed) {
        return;
    }

    const auto limited = clampDistance(distance, limits.maxAdvance);

    const auto heading = sim_state.drone_orientation.horizontal;

    const auto dx = si::cos(heading);
    const auto dy = si::sin(heading);

    const Position3D next_position{
        sim_state.drone_position.x + XLength{
            dx * limited.force_numerical_value_in(cm) * cm
        },
        sim_state.drone_position.y + YLength{
            dy * limited.force_numerical_value_in(cm) * cm
        },
        sim_state.drone_position.z
    };

    if (validatePath(sim_state.drone_position, next_position)) {
        sim_state.drone_position = next_position;
    }
}

void MovementDriverMock::elevate(Distance distance) const
{
    if (sim_state.failed) {
        return;
    }

    const auto limited = clampDistance(distance, limits.maxElevate);

    const Position3D next_position{
        sim_state.drone_position.x,
        sim_state.drone_position.y,
        sim_state.drone_position.z + ZLength{
            limited.force_numerical_value_in(cm) * cm
        }
    };

    if (validatePath(sim_state.drone_position, next_position)) {
        sim_state.drone_position = next_position;
    }
}
