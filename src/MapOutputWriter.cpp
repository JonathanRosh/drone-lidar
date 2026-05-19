#include "MapOutputWriter.h"

#include "MapUtils.h"
#include "Utils.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>

void MapOutputWriter::writeOccupiedCells(
    const IMap3D &mapped_map,
    const MissionConfig &mission_config,
    const std::filesystem::path &output_path) {
  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error("Failed to open file for writing: " +
                             output_path.string());
  }

  const Distance res_xy = MapUtils::xyResolution(mission_config);
  const Distance res_z = MapUtils::zResolution(mission_config);
  const Utils::GridBounds bounds = Utils::gridBoundsForMission(mission_config);

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
