#include "../include/LidarSensorMock.h"
#include "../include/Units.h"

#include <mp-units/systems/si/math.h>

LidarSensorMock::LidarSensorMock(const LidarConfig &lidar_config,
                                 const IMap3D &simulation_map,
                                 IPositionSensor &pos_sensor)
    : lidar_config(lidar_config), map(simulation_map),
      pos_sensor(pos_sensor) {};

// TODO: move semantics
// 3D DDA Voxel traversal algorithm
std::optional<Distance> LidarSensorMock::traceBeam(const Orientation& beam_orientation) const {

    const Position3D& origin = pos_sensor.getPosition();

    const auto cos_altitude = si::cos(beam_orientation.altitude);

    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const Distance step = Distance{0.1 * cm};
    const Distance min_distance = std::min(lidar_config.zMin, step);

    for (Distance distance = min_distance; distance <= lidar_config.zMax; distance += step) {
       
        const Position3D sample{
            origin.x + dx.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * x_extent[cm],
            origin.y + dy.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * y_extent[cm],
            origin.z + dz.force_numerical_value_in(mp::one) * distance.force_numerical_value_in(cm) * z_extent[cm],
        };

        if (!map.isInsideBounds(sample)) {
            return std::nullopt;
        }

        // TODO: map.get is expensive right now because of world to grid conversion
        if (map.get(sample) == OCCUPIED) {
            if (distance < lidar_config.zMin){
                return Distance{0*cm};
            }
            return distance;
        }
    }

    return std::nullopt;
}

std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

HorizontalAngle horizontal_delta(Distance offset, Distance distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

Altitude altitude_delta(Distance offset, Distance distance) {
    return Altitude{si::atan2(offset, distance)};
}

// TODO: move semantics for this
LidarScanResult LidarSensorMock::getScan(const Orientation& drone_orientation) const {

    LidarScanResult results;
    if (lidar_config.fovc == 0) {
        return results;
    }

    const Orientation& sensor_heading = pos_sensor.getOrientation();
    const Orientation& beam_0 = drone_orientation; // alias

    const Orientation beam_0_abs{
        beam_0.horizontal + sensor_heading.horizontal,
        beam_0.altitude + sensor_heading.altitude,
    };

    const auto center_distance = traceBeam(beam_0_abs);
    results.push_back(LidarHit{
        center_distance.value_or(lidar_config.zMax),
        beam_0,
        center_distance.has_value(),
    });

    for (std::size_t circle = 1; circle < lidar_config.fovc; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const Distance radius = static_cast<double>(circle) * lidar_config.d / 2.0;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const Distance horizontal_offset = radius * si::cos(theta);
            const Distance altitude_offset = radius * si::sin(theta);
            
            const Orientation offset{
                horizontal_delta(horizontal_offset, lidar_config.zMin),
                altitude_delta(altitude_offset, lidar_config.zMin),
            };

            // TODO: this is not efficient
            const Orientation abs_circle_beam{
                beam_0.horizontal + offset.horizontal + sensor_heading.horizontal,
                beam_0.altitude + offset.altitude + sensor_heading.altitude,
            };
            
            const Orientation circle_beam{
                beam_0.horizontal + offset.horizontal,
                beam_0.altitude + offset.altitude,
            };
            const auto distance = traceBeam(abs_circle_beam);
            results.push_back(LidarHit{
                distance.value_or(lidar_config.zMax),
                circle_beam,
                distance.has_value(),
            });
        }
    }

    return results;
}
