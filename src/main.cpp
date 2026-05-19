#include "IMap3D.h"
#include "MapUtils.h"
#include "Parser.h"
#include "Simulator.h"
#include "TrueMap.h"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// TODO: this is duplicate from simulator
double cmValue(Distance distance) {
  return distance.force_numerical_value_in(cm);
}

// TODO: move score logic and map output to its own class 
int lowerGridIndex(Distance value, Distance resolution) {
  return static_cast<int>(
      std::ceil(cmValue(value) / cmValue(resolution) - 1e-9));
}

int upperGridIndex(Distance value, Distance resolution) {
  return static_cast<int>(
      std::floor(cmValue(value) / cmValue(resolution) + 1e-9));
}

struct GridBounds {
  GridCoord min;
  GridCoord max;
};

struct ProgramOptions {
  std::filesystem::path input_output_path;
  bool log_enabled = false;
};

ProgramOptions parseProgramOptions(int argc, char **argv) {
  ProgramOptions options{
      .input_output_path = std::filesystem::current_path(),
      .log_enabled = false,
  };

  bool path_set = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--log") {
      options.log_enabled = true;
      continue;
    }

    if (path_set) {
      throw std::runtime_error("Unexpected command line argument: " + arg);
    }

    options.input_output_path = std::filesystem::path(arg);
    path_set = true;
  }

  return options;
}

GridBounds gridBoundsForMission(const MissionConfig &mission_config) {
  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const auto &boundary = mission_config.map_boundry;

  return {
      {
          lowerGridIndex(boundary.minX, res_xy),
          lowerGridIndex(boundary.minY, res_xy),
          lowerGridIndex(boundary.minHeight, res_z),
      },
      {
          upperGridIndex(boundary.maxX, res_xy),
          upperGridIndex(boundary.maxY, res_xy),
          upperGridIndex(boundary.maxHeight, res_z),
      },
  };
}

struct MappingStats {
  std::size_t total_voxels = 0;
  std::size_t correct_voxels = 0;
  std::size_t incorrect_voxels = 0;
  std::size_t true_occupied = 0;
  std::size_t true_empty = 0;
  std::size_t mapped_occupied = 0;
  std::size_t mapped_empty = 0;
  std::size_t mapped_not_mapped = 0;
  std::size_t occupied_found = 0;
  std::size_t occupied_missed = 0;
  std::size_t false_occupied = 0;
  std::size_t empty_found = 0;
  double score = 0.0;
};

MappingStats calculateStats(const IMap3D &true_map,
                            const IMap3D &mapped_map,
                            const MissionConfig &mission_config) {
  MappingStats stats;
  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const GridBounds bounds = gridBoundsForMission(mission_config);

  for (int z = bounds.min.z; z <= bounds.max.z; ++z) {
    for (int y = bounds.min.y; y <= bounds.max.y; ++y) {
      for (int x = bounds.min.x; x <= bounds.max.x; ++x) {
        const Position3D position = MapUtils::gridToWorld({x, y, z},
                                                          res_xy,
                                                          res_z);
        if (!MapUtils::insideMissionBounds(position, mission_config)) {
          continue;
        }

        const Mapping true_value = true_map.get(position);
        const Mapping mapped_value = mapped_map.get(position);

        ++stats.total_voxels;
        if (true_value == mapped_value) {
          ++stats.correct_voxels;
        } else {
          ++stats.incorrect_voxels;
        }

        if (true_value == OCCUPIED) {
          ++stats.true_occupied;
          if (mapped_value == OCCUPIED) {
            ++stats.occupied_found;
          } else {
            ++stats.occupied_missed;
          }
        } else {
          ++stats.true_empty;
          if (mapped_value == EMPTY) {
            ++stats.empty_found;
          }
        }

        if (mapped_value == OCCUPIED) {
          ++stats.mapped_occupied;
          if (true_value != OCCUPIED) {
            ++stats.false_occupied;
          }
        } else if (mapped_value == EMPTY) {
          ++stats.mapped_empty;
        } else if (mapped_value == NOT_MAPPED) {
          ++stats.mapped_not_mapped;
        }
      }
    }
  }

  if (stats.total_voxels != 0) {
    stats.score = 100.0 * static_cast<double>(stats.correct_voxels) /
                  static_cast<double>(stats.total_voxels);
  }

  return stats;
}

void printStats(const MappingStats &stats) {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Mapping score: " << stats.score << " / 100\n";
  std::cout << "Total in-bound voxels: " << stats.total_voxels << '\n';
  std::cout << "Correct voxels: " << stats.correct_voxels << '\n';
  std::cout << "Incorrect voxels: " << stats.incorrect_voxels << '\n';
  std::cout << "True occupied cells: " << stats.true_occupied << '\n';
  std::cout << "True empty cells: " << stats.true_empty << '\n';
  std::cout << "Mapped occupied cells: " << stats.mapped_occupied << '\n';
  std::cout << "Mapped empty cells: " << stats.mapped_empty << '\n';
  std::cout << "Mapped not-mapped cells: " << stats.mapped_not_mapped << '\n';
  std::cout << "True occupied cells found: " << stats.occupied_found << '\n';
  std::cout << "True occupied cells missed: " << stats.occupied_missed << '\n';
  std::cout << "False occupied cells: " << stats.false_occupied << '\n';
  std::cout << "True empty cells found: " << stats.empty_found << '\n';
}

void writeMapOutput(const IMap3D &mapped_map,
                    const MissionConfig &mission_config,
                    const std::filesystem::path &output_path) {
  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error("Failed to open file for writing: " +
                             output_path.string());
  }

  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const GridBounds bounds = gridBoundsForMission(mission_config);

  output << std::setprecision(10);
  for (int z = bounds.min.z; z <= bounds.max.z; ++z) {
    for (int y = bounds.min.y; y <= bounds.max.y; ++y) {
      for (int x = bounds.min.x; x <= bounds.max.x; ++x) {
        const Position3D position = MapUtils::gridToWorld({x, y, z},
                                                          res_xy,
                                                          res_z);
        if (!MapUtils::insideMissionBounds(position, mission_config)) {
          continue;
        }
        if (mapped_map.get(position) != OCCUPIED) {
          continue;
        }

        output << position.x.force_numerical_value_in(cm) << ' '
               << position.y.force_numerical_value_in(cm) << ' '
               << position.z.force_numerical_value_in(cm) << '\n';
      }
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  namespace fs = std::filesystem;

  try {
    const ProgramOptions options = parseProgramOptions(argc, argv);
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
      *log << "drone_mapper log enabled\n";
      *log << "input_output_path: " << input_output_path << '\n';
    }

    const DroneConfig drone_config =
        Parser::parseDroneConfig(drone_config_path.string());
    const MissionConfig mission_config =
        Parser::parseMissionConfig(mission_config_path.string());
    TrueMap true_map =
        Parser::parseTrueMap(true_map_path.string(), mission_config);

    Simulator simulator(drone_config, mission_config, true_map, log);
    IMap3D &mapped_map = simulator.simulate();

    const MappingStats stats =
        calculateStats(true_map, mapped_map, mission_config);
    printStats(stats);
    if (log != nullptr) {
      *log << "score: " << stats.score << '\n';
      *log << "stats: total=" << stats.total_voxels
           << " correct=" << stats.correct_voxels
           << " incorrect=" << stats.incorrect_voxels
           << " mapped_occupied=" << stats.mapped_occupied
           << " mapped_empty=" << stats.mapped_empty
           << " mapped_not_mapped=" << stats.mapped_not_mapped
           << " occupied_found=" << stats.occupied_found
           << " occupied_missed=" << stats.occupied_missed
           << " false_occupied=" << stats.false_occupied << '\n';
    }
    writeMapOutput(mapped_map, mission_config, output_map_path);
    if (log != nullptr) {
      *log << "wrote map output: " << output_map_path << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << "Unrecoverable error: " << e.what() << '\n';
  } catch (...) {
    std::cerr << "Unrecoverable error: unknown exception\n";
  }

  return 0;
}
