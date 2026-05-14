#ifndef DRONE_LIDAR_LIDARSENSORMOCK_H
#define DRONE_LIDAR_LIDARSENSORMOCK_H

#include "Configs.h"
#include "ILidarSensor.h"
#include "IMap3D.h"
#include "IPositionSensor.h"

namespace mp = mp_units;

using mp::si::unit_symbols::cm;
using mp::si::unit_symbols::deg;

class LidarSensorMock : public ILidarSensor {

  const LidarConfig &lidar_config;
  const IMap3D &map;
  const IPositionSensor &pos_sensor;

public:
  LidarSensorMock(const LidarConfig &lidar_config, IMap3D &simulation_map,
                  IPositionSensor &pos_sensor);
  // LidarScanResult getScan(const Orientation& drone_orientation) override;
};

#endif // DRONE_LIDAR_LIDARSENSORMOCK_H
