#pragma once

#include "Configs.h"
#include "IMap3D.h"

#include <filesystem>

class MapOutputWriter {
public:
  static void writeOccupiedCells(const IMap3D &mapped_map,
                                 const MissionConfig &mission_config,
                                 const std::filesystem::path &output_path);
};
