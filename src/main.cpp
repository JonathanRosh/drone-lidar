#include "Parser.h"
#include "Simulator.h"
#include "TrueMap.h"

#include <filesystem>
#include <string>

int main(int argc, char **argv) {

  namespace fs = std::filesystem;

  fs::path input_output_path =
      (argc >= 2) ? fs::path(argv[1]) : fs::current_path();

  fs::path drone_config_path = input_output_path / "drone_config.txt";
  fs::path mission_config_path = input_output_path / "mission_config.txt";
  fs::path true_map_path = input_output_path / "map_input.txt";

  const DroneConfig drone_config =
      Parser::parseDroneConfig(drone_config_path.string());
  const MissionConfig mission_config =
      Parser::parseMissionConfig(mission_config_path.string());
  TrueMap true_map = Parser::parseTrueMap(true_map_path.string(), mission_config);

  Simulator simulator(drone_config, mission_config, true_map);
  IMap3D &mapped_map = simulator.simulate();
  (void)mapped_map;

  // TODO:
  // Calculate score and print it

  return 0;
}
