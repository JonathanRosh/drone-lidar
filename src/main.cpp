#include "Configs.h"
#include "Parser.h"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char **argv) {

  // parse txt files
  namespace fs = std::filesystem;

  fs::path input_output_path =
      (argc >= 2) ? fs::path(argv[1]) : fs::current_path();

  fs::path drone_config_path = input_output_path / "drone_config.txt";
  fs::path mission_config_path = input_output_path / "mission_config.txt";
  fs::path true_map_path = input_output_path / "map_input.txt";

  try {
    const DroneConfig drone_config =
        parseDroneConfig(drone_config_path.string());
    const MissionConfig mission_config =
        parseMissionConfig(mission_config_path.string());
    // const TrueMap true_map = parseTrueMap("true_map.txt");

    (void)drone_config;
    (void)mission_config;

  } catch (const std::exception &ex) {

    std::cerr << "Error parsing input files: " << ex.what() << std::endl;
    return 1;
  }

  // DroneConfig, LidarConfig, MissionConfig, TrueMap

  // // init Simulator
  return 0;
}