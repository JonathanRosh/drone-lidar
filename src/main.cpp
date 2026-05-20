#include "Drone.h"
#include "DroneMap.h"
#include "LidarSensorMock.h"
#include "MovementDriverMock.h"
#include "Parser.h"
#include "PositionSensorMock.h"
#include "Simulator.h"
#include "TrueMap.h"
#include "Units.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

/** Fraction of true occupied cells also marked occupied by the drone. */
double computeMappingScore(const TrueMap &true_map, const IMap3D &drone_map) {
  int true_occupied = 0;
  int matched = 0;

  true_map.forEachStoredCell([&](const Position3D &pos, Mapping mapping) {
    if (mapping != OCCUPIED) {
      return;
    }
    ++true_occupied;
    if (drone_map.get(pos) == OCCUPIED) {
      ++matched;
    }
  });

  if (true_occupied == 0) {
    return 100.0;
  }
  return 100.0 * static_cast<double>(matched) /
         static_cast<double>(true_occupied);
}

} // namespace

int main(int argc, char **argv) {
  namespace fs = std::filesystem;

  try {
    const fs::path input_output_path =
        (argc >= 2) ? fs::path(argv[1]) : fs::current_path();

    const fs::path drone_config_path = input_output_path / "drone_config.txt";
    const fs::path mission_config_path =
        input_output_path / "mission_config.txt";
    const fs::path true_map_path = input_output_path / "map_input.txt";
    const fs::path map_output_path = input_output_path / "map_output.txt";

    const DroneConfig drone_config =
        parseDroneConfig(drone_config_path.string());
    const MissionConfig mission_config =
        parseMissionConfig(mission_config_path.string());
    TrueMap true_map = parseTrueMap(true_map_path.string(), mission_config);

    SimulationState state{
        .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
        .drone_orientation = {0.0 * deg, 0.0 * deg},
    };

    PositionSensorMock position_sensor(state);
    MovementDriverMock movement_driver(state, drone_config.maxCommand);
    LidarSensorMock lidar_sensor(drone_config.lidarConfig, true_map,
                                 position_sensor);
    Drone drone(movement_driver, position_sensor, lidar_sensor);

    DroneMap drone_map(mission_config);
    Simulator simulator(drone, true_map, drone_map, lidar_sensor,
                        position_sensor, movement_driver, state,
                        mission_config, drone_config);

    std::cout << "Starting mapping simulation..." << std::endl;
    simulator.simulate();
    std::cout << "Simulation finished. Drone occupied cells: "
              << drone_map.occupiedCount() << std::endl;

    std::ofstream map_out(map_output_path);
    if (!map_out) {
      throw std::runtime_error("Failed to open output map: " +
                               map_output_path.string());
    }
    drone_map.writeOccupied(map_out);

    const double score = computeMappingScore(true_map, drone_map);
    std::cout << "Mapping score: " << score << " / 100\n";
    std::cout << "Wrote " << map_output_path << '\n';

  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
