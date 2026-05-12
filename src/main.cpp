#include "Parser.h"
#include "Simulator.h"
#include "Units.h"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

const char *mappingName(Mapping m) {
  switch (m) {
  case EMPTY:
    return "EMPTY(0)";
  case OCCUPIED:
    return "OCCUPIED(1)";
  case NOT_MAPPED:
    return "NOT_MAPPED(-1)";
  case OUTSIDE_BOUNDARY:
    return "OUTSIDE_BOUNDARY(-2)";
  default:
    return "?";
  }
}

void printProbe(const TrueMap &map, const Position3D &pos, const char *label) {
  const Mapping m = map.get(pos);
  std::cout << "  " << label << " (" << pos.x.numerical_value_in(cm) << ", "
            << pos.y.numerical_value_in(cm) << ", "
            << pos.z.numerical_value_in(cm) << ") cm -> " << mappingName(m)
            << '\n';
}

} // namespace

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
  const TrueMap true_map = parseTrueMap(true_map_path.string(), mission_config);

  Simulator simulator(drone_config, mission_config, true_map);
  IMap3D &map = simulator.simulate();

  return 0;
}
