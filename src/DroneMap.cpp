#include "../include/DroneMap.h"

DroneMap::DroneMap(const MissionConfig &mission_config)
    : MapCore(mission_config, NOT_MAPPED) {}

void DroneMap::set(const Position3D &pos, Mapping mapping) {
  setCell(pos, mapping);
}
