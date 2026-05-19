#include "ApplicationLogger.h"
#include "MapOutputWriter.h"
#include "Parser.h"
#include "Score.h"
#include "Simulator.h"
#include "Utils.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  namespace fs = std::filesystem;

  try {
    const Utils::ProgramOptions options = Utils::parseProgramOptions(argc, argv);
    const fs::path input_output_path = options.input_output_path;

    fs::path drone_config_path = input_output_path / "drone_config.txt";
    fs::path mission_config_path = input_output_path / "mission_config.txt";
    fs::path true_map_path = input_output_path / "map_input.txt";
    fs::path output_map_path = input_output_path / "map_output.txt";
    fs::path log_path = input_output_path / "log.txt";

    std::ofstream log_file;
    std::ostream *log = nullptr;
    if (options.log_enabled) {
      log_file.open(log_path);
      if (!log_file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " +
                                 log_path.string());
      }
      log = &log_file;
    }
    const ApplicationLogger logger(log);
    logger.started(input_output_path);

    const DroneConfig drone_config =
        Parser::parseDroneConfig(drone_config_path.string());
    const MissionConfig mission_config =
        Parser::parseMissionConfig(mission_config_path.string());
    TrueMap true_map =
        Parser::parseTrueMap(true_map_path.string(), mission_config);

    Simulator simulator(drone_config, mission_config, true_map, log);
    IMap3D &mapped_map = simulator.simulate();

    const MappingStats stats =
        Score::calculate(true_map, mapped_map, mission_config);
    Score::print(stats, std::cout);
    logger.scoreSummary(stats);
    MapOutputWriter::writeOccupiedCells(mapped_map,
                                        mission_config,
                                        output_map_path);
    logger.mapOutputWritten(output_map_path);
  } catch (const std::exception &e) {
    std::cerr << "Unrecoverable error: " << e.what() << '\n';
  } catch (...) {
    std::cerr << "Unrecoverable error: unknown exception\n";
  }

  return 0;
}
