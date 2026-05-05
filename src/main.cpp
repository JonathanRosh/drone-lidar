#include "parsing/Parser.h"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  const std::string base_path = (argc > 1) ? argv[1] : ".";

  try {
    const auto drone_config =
        parsing::parseDroneConfigFile(parsing::buildPath(base_path, "drone_config.txt"));
    const auto mission_config = parsing::parseMissionConfigFile(
        parsing::buildPath(base_path, "mission_config.txt"));
    const auto input_map =
        parsing::parseInputMapFile(parsing::buildPath(base_path, "map_input.txt"));

    std::cout << "Parsing succeeded.\n";
    std::cout << "Drone FOVC: " << drone_config.lidar_config.fovc << "\n";
    std::cout << "Mission XY resolution: " << mission_config.resolution.xy << "\n";
    std::cout << "Occupied points: " << input_map.occupied_points.size() << "\n";
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Parsing failed: " << ex.what() << "\n";
    return 1;
  }
}