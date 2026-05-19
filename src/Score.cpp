#include "Score.h"

#include "MapUtils.h"
#include "Utils.h"

#include <iomanip>
#include <ostream>

MappingStats Score::calculate(const IMap3D &true_map,
                              const IMap3D &mapped_map,
                              const MissionConfig &mission_config) {
  MappingStats stats;
  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const Utils::GridBounds bounds = Utils::gridBoundsForMission(mission_config);

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

void Score::print(const MappingStats &stats, std::ostream &output) {
  output << std::fixed << std::setprecision(2);
  output << "Mapping score: " << stats.score << " / 100\n";
  output << "Total in-bound voxels: " << stats.total_voxels << '\n';
  output << "Correct voxels: " << stats.correct_voxels << '\n';
  output << "Incorrect voxels: " << stats.incorrect_voxels << '\n';
  output << "True occupied cells: " << stats.true_occupied << '\n';
  output << "True empty cells: " << stats.true_empty << '\n';
  output << "Mapped occupied cells: " << stats.mapped_occupied << '\n';
  output << "Mapped empty cells: " << stats.mapped_empty << '\n';
  output << "Mapped not-mapped cells: " << stats.mapped_not_mapped << '\n';
  output << "True occupied cells found: " << stats.occupied_found << '\n';
  output << "True occupied cells missed: " << stats.occupied_missed << '\n';
  output << "False occupied cells: " << stats.false_occupied << '\n';
  output << "True empty cells found: " << stats.empty_found << '\n';
}

void Score::writeLogSummary(const MappingStats &stats, std::ostream &output) {
  output << "score: " << stats.score << '\n';
  output << "stats: total=" << stats.total_voxels
         << " correct=" << stats.correct_voxels
         << " incorrect=" << stats.incorrect_voxels
         << " mapped_occupied=" << stats.mapped_occupied
         << " mapped_empty=" << stats.mapped_empty
         << " mapped_not_mapped=" << stats.mapped_not_mapped
         << " occupied_found=" << stats.occupied_found
         << " occupied_missed=" << stats.occupied_missed
         << " false_occupied=" << stats.false_occupied << '\n';
}
