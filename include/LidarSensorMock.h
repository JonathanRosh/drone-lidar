#ifndef DRONE_LIDAR_LIDARSENSORMOCK_H
#define DRONE_LIDAR_LIDARSENSORMOCK_H

#include "ILidarSensor.h"
#include "IMap3D.h"
#include "IPositionSensor.h"
#include "Configs.h"
#include "LidarScanResult.h"

#include <optional>

class LidarSensorMock : public ILidarSensor {

    const LidarConfig& lidar_config;
    const IMap3D& map;
    const IPositionSensor& pos_sensor;
    
    std::optional<Distance> traceBeam(const Orientation& beam_orientation) const;

    friend class LidarSensorMockTester;
    
public:
    LidarSensorMock(const LidarConfig& lidar_config, IMap3D& simulation_map, IPositionSensor& pos_sensor);
    LidarScanResult getScan(const Orientation& drone_orientation) const override;
};

#endif //DRONE_LIDAR_LIDARSENSORMOCK_H
