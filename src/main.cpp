#include "Parser.h"
#include "TrueMap.h"
#include "Units.h"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char **argv) {

  namespace fs = std::filesystem;

  fs::path input_output_path =
      (argc >= 2) ? fs::path(argv[1]) : fs::current_path();

  fs::path drone_config_path = input_output_path / "drone_config.txt";
  fs::path mission_config_path = input_output_path / "mission_config.txt";
  fs::path true_map_path = input_output_path / "map_input.txt";

  const DroneConfig drone_config = parseDroneConfig(drone_config_path.string());
  const MissionConfig mission_config =
      parseMissionConfig(mission_config_path.string());
  TrueMap true_map = parseTrueMap(true_map_path.string(), mission_config);

  (void)drone_config;
  (void)mission_config;
  (void)true_map;

  // TODO:
  //  Instanciate drone and all other objects needed for the simulator

  // Run simulator.simulate() and get the final map

  // Calculate score and print it

  return 0;
}
