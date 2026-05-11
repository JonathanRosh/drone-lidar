#include "../include/LidarSensorMock.h"

LidarSensorMock::LidarSensorMock(
        const LidarConfig& lidar_config, 
        IMap3D& simulation_map, 
        IPositionSensor& pos_sensor
    ) : lidar_config(lidar_config), map(simulation_map), pos_sensor(pos_sensor) {};

// LidarScanResult LidarSensorMock::getScan(const Orientation& drone_orientation) {

// }
