#include "../include/LidarSensorMock.h"

#include <cmath>

LidarSensorMock::LidarSensorMock(
        const LidarConfig& lidar_config, 
        IMap3D& simulation_map, 
        IPositionSensor& pos_sensor
    ) : lidar_config(lidar_config), map(simulation_map), pos_sensor(pos_sensor) {};

// TODO: move semantics
// 3D DDA Voxel traversal algorithm
LidarHit LidarSensorMock::traceBeam(const Orientation& beam_orientation) const {

    (void)beam_orientation;

    Distance distance = 10.0 * cm;;
    Orientation orientation;

    return LidarHit{
        distance,
        orientation
    };
}

// TODO: move semantics for this
LidarScanResult LidarSensorMock::getScan(const Orientation& drone_orientation) const {

    (void)drone_orientation;
    std::vector<LidarHit> hits;

    return LidarScanResult{
        hits
    };
}
