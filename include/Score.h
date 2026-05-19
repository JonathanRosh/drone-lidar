#pragma once

#include "Configs.h"
#include "IMap3D.h"

#include <cstddef>
#include <iosfwd>

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

class Score {
public:
  static MappingStats calculate(const IMap3D &true_map,
                                const IMap3D &mapped_map,
                                const MissionConfig &mission_config);
  static void print(const MappingStats &stats, std::ostream &output);
  static void writeLogSummary(const MappingStats &stats, std::ostream &output);
};
