#pragma once

#include <cstddef>
#include <functional>

struct GridCoord {
  int x;
  int y;
  int z;

  bool operator==(const GridCoord &) const = default;
};

struct GridCoordHash {
  std::size_t operator()(const GridCoord &c) const {
    const std::size_t h1 = std::hash<int>{}(c.x);
    const std::size_t h2 = std::hash<int>{}(c.y);
    const std::size_t h3 = std::hash<int>{}(c.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};
