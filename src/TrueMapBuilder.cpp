#include "../include/TrueMapBuilder.h"
#include "../include/TrueMap.h"

void TrueMapBuilder::set(TrueMap& map, Position3D pos, Mapping val) {
  map.setCell(pos, val);
}
