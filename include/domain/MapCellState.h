#pragma once

namespace domain {

enum class MapCellState {
  Empty = 0,
  Occupied = 1,
  UnknownUnmapped = -1,
  OutOfBounds = -2
};

} // namespace domain
