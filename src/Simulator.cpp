#include "../include/Simulator.h"

#include "../include/Units.h"

Simulator::Simulator(const DroneConfig &drone_config,
                     const MissionConfig &mission_config,
                     const TrueMap &true_map)
    : drone_config_(drone_config),
      mission_config_(mission_config),
      true_map_(true_map),
      sim_state_{
          .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
          .drone_orientation = {0.0 * deg, 0.0 * deg},
          .failure_reason = "",
      },
      mapped_map_(mission_config_),
      position_sensor_(sim_state_),
      movement_driver_(sim_state_,
                       drone_config_.maxCommand,
                       true_map_,
                       mission_config_,
                       drone_config_.minPass),
      lidar_sensor_(drone_config_.lidarConfig, true_map_, position_sensor_),
      drone_(movement_driver_, position_sensor_, lidar_sensor_) {}

IMap3D &Simulator::simulate() {
  return mapped_map_;
}
