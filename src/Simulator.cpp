#include "../include/Simulator.h"

#include "../include/MapUtils.h"
#include "../include/Units.h"
#include "../include/Utils.h"

#include <algorithm>
#include <cmath>
#include <mp-units/framework.h>
#include <mp-units/systems/si/math.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace mp = mp_units;

namespace {

double normalizeDeg(double value) {
  const double normalized = std::fmod(value, 360.0);
  return normalized < 0.0 ? normalized + 360.0 : normalized;
}

Distance minResolution(Distance xy, Distance z) {
  return Utils::cmValue(xy) < Utils::cmValue(z) ? xy : z;
}

GridCoord gridDelta(const GridCoord &from, const GridCoord &to) {
  return {to.x - from.x, to.y - from.y, to.z - from.z};
}

std::size_t inBoundsVoxelCount(const MissionConfig &mission_config,
                               Distance res_xy, Distance res_z) {
  const auto &b = mission_config.map_boundry;
  const auto x_count = static_cast<std::size_t>(
      std::max(1.0, std::floor(Utils::cmValue(b.maxX - b.minX) /
                                Utils::cmValue(res_xy)) +
                        1.0));
  const auto y_count = static_cast<std::size_t>(
      std::max(1.0, std::floor(Utils::cmValue(b.maxY - b.minY) /
                                Utils::cmValue(res_xy)) +
                        1.0));
  const auto z_count = static_cast<std::size_t>(
      std::max(1.0,
               std::floor(Utils::cmValue(b.maxHeight - b.minHeight) /
                          Utils::cmValue(res_z)) +
                   1.0));
  return x_count * y_count * z_count;
}

} // namespace

Simulator::Simulator(const DroneConfig &drone_config,
                     const MissionConfig &mission_config,
                     const TrueMap &true_map,
                     std::ostream *log)
    : drone_config_(drone_config),
      mission_config_(mission_config),
      true_map_(true_map),
      sim_state_{
          .drone_position = {0.0 * cm, 0.0 * cm, 0.0 * cm},
          .drone_orientation = {0.0 * deg, 0.0 * deg},
          .failure_reason = "",
      },
      mapped_map_(mission_config_),
      position_sensor_(sim_state_),
      movement_driver_(sim_state_,
                       drone_config_.maxCommand,
                       true_map_,
                       mission_config_,
                       drone_config_.minPass),
      lidar_sensor_(drone_config_.lidarConfig, true_map_, position_sensor_),
      drone_(movement_driver_, position_sensor_, lidar_sensor_),
      res_xy_(MapUtils::xyResolution(mission_config_)),
      res_z_(MapUtils::zResolution(mission_config_)),
      logger_(log) {}

GridCoord Simulator::worldToGrid(const Position3D &position) const {
  return MapUtils::worldToGrid(position, res_xy_, res_z_);
}

Position3D Simulator::gridToWorld(const GridCoord &grid) const {
  return MapUtils::gridToWorld(grid, res_xy_, res_z_);
}

std::vector<GridCoord> Simulator::neighbors(const GridCoord &grid) const {
  return {
      {grid.x + 1, grid.y, grid.z},
      {grid.x - 1, grid.y, grid.z},
      {grid.x, grid.y + 1, grid.z},
      {grid.x, grid.y - 1, grid.z},
      {grid.x, grid.y, grid.z + 1},
      {grid.x, grid.y, grid.z - 1},
  };
}

Distance Simulator::clearanceRadius() const {
  const double width = drone_config_.minPass.width.force_numerical_value_in(cm);
  const double height = drone_config_.minPass.height.force_numerical_value_in(cm);
  const double length = drone_config_.minPass.length.force_numerical_value_in(cm);
  const double diameter = std::max({width, height, length});
  return Distance{(diameter / 2.0) * cm};
}

bool Simulator::canOccupyDiscovered(const Position3D &position) const {
  const double r = clearanceRadius().force_numerical_value_in(cm);
  const double offsets[] = {-r, 0.0, r};

  for (double dx : offsets) {
    for (double dy : offsets) {
      for (double dz : offsets) {
        const Position3D sample{
            position.x + dx * x_extent[cm],
            position.y + dy * y_extent[cm],
            position.z + dz * z_extent[cm],
        };

        if (!mapped_map_.isInsideBounds(sample)) {
          return false;
        }

        if (mapped_map_.get(sample) == OCCUPIED) {
          return false;
        }
      }
    }
  }

  return true;
}

bool Simulator::isFrontier(const GridCoord &grid) const {
  const Position3D position = gridToWorld(grid);
  if (mapped_map_.get(position) != EMPTY) {
    return false;
  }
  if (!canOccupyDiscovered(position)) {
    logger_.frontierRejected(grid);
    return false;
  }

  for (const GridCoord &neighbor : neighbors(grid)) {
    const Position3D neighbor_position = gridToWorld(neighbor);
    if (mapped_map_.isInsideBounds(neighbor_position) &&
        mapped_map_.get(neighbor_position) == NOT_MAPPED) {
      return true;
    }
  }

  return false;
}

std::optional<std::vector<GridCoord>> Simulator::pathToNearestFrontier() const {
  const GridCoord start = worldToGrid(drone_.getPosition());
  std::queue<GridCoord> pending;
  std::unordered_set<GridCoord, GridCoordHash> visited;
  std::unordered_map<GridCoord, GridCoord, GridCoordHash> parent;

  pending.push(start);
  visited.insert(start);

  while (!pending.empty()) {
    const GridCoord current = pending.front();
    pending.pop();

    if (current != start && isFrontier(current)) {
      std::vector<GridCoord> path;
      GridCoord step = current;
      path.push_back(step);
      while (step != start) {
        step = parent.at(step);
        path.push_back(step);
      }
      std::reverse(path.begin(), path.end());
      logger_.frontierPath(path.size(), visited.size(), current);
      return path;
    }

    for (const GridCoord &neighbor : neighbors(current)) {
      if (visited.contains(neighbor)) {
        continue;
      }

      const Position3D neighbor_position = gridToWorld(neighbor);
      if (mapped_map_.get(neighbor_position) != EMPTY ||
          !canOccupyDiscovered(neighbor_position)) {
        continue;
      }

      visited.insert(neighbor);
      parent.emplace(neighbor, current);
      pending.push(neighbor);
    }
  }

  logger_.frontierStopped(visited.size());
  return std::nullopt;
}

void Simulator::scanCurrentPosition() {
  const Position3D origin = drone_.getPosition();
  const Orientation orientation = drone_.getOrientation();
  mapped_map_.set(origin, EMPTY);

  const struct ScanDirection {
    Angle xy;
    Angle z;
  } scan_directions[] = {
      {0.0 * deg, 0.0 * deg},   {90.0 * deg, 0.0 * deg},
      {180.0 * deg, 0.0 * deg}, {270.0 * deg, 0.0 * deg},
      {0.0 * deg, 90.0 * deg},  {0.0 * deg, -90.0 * deg},
  };

  for (const auto &scan_direction : scan_directions) {
    const LidarScanResult scan =
        drone_.scan(scan_direction.xy, scan_direction.z);
    std::size_t detected = 0;
    for (const LidarHit &hit : scan) {
      if (hit.detected) {
        ++detected;
      }
    }
    logger_.scanSummary(scan_direction.xy,
                        scan_direction.z,
                        scan.size(),
                        detected);

    for (const LidarHit &hit : scan) {
      if (hit.detected) {
        logger_.lidarHit(hit);
      }
      integrateBeam(origin, orientation, hit);
    }
  }
}

void Simulator::integrateBeam(const Position3D &origin,
                             const Orientation &drone_orientation,
                             const LidarHit &hit) {
  const Orientation absolute{
      drone_orientation.horizontal + hit.orientation.horizontal,
      drone_orientation.altitude + hit.orientation.altitude,
  };

  const auto cos_altitude = si::cos(absolute.altitude);
  const auto dx = cos_altitude * si::cos(absolute.horizontal);
  const auto dy = cos_altitude * si::sin(absolute.horizontal);
  const auto dz = si::sin(absolute.altitude);

  const Distance step = minResolution(res_xy_, res_z_);
  const double step_cm = Utils::cmValue(step);
  const double hit_cm = Utils::cmValue(hit.distance);
  const double ray_end_cm = hit.detected && hit_cm == 0.0 ? step_cm : hit_cm;

  for (double distance_cm = step_cm; distance_cm < ray_end_cm;
       distance_cm += step_cm) {
    const Position3D sample{
        origin.x + dx.force_numerical_value_in(mp::one) * distance_cm *
                       x_extent[cm],
        origin.y + dy.force_numerical_value_in(mp::one) * distance_cm *
                       y_extent[cm],
        origin.z + dz.force_numerical_value_in(mp::one) * distance_cm *
                       z_extent[cm],
    };
    if (mapped_map_.isInsideBounds(sample)) {
      mapped_map_.set(sample, EMPTY);
    }
  }

  if (!hit.detected) {
    const Position3D ray_end{
        origin.x + dx.force_numerical_value_in(mp::one) * ray_end_cm *
                       x_extent[cm],
        origin.y + dy.force_numerical_value_in(mp::one) * ray_end_cm *
                       y_extent[cm],
        origin.z + dz.force_numerical_value_in(mp::one) * ray_end_cm *
                       z_extent[cm],
    };
    if (mapped_map_.isInsideBounds(ray_end)) {
      mapped_map_.set(ray_end, EMPTY);
    }
    return;
  }

  const Position3D occupied{
      origin.x + dx.force_numerical_value_in(mp::one) * ray_end_cm *
                     x_extent[cm],
      origin.y + dy.force_numerical_value_in(mp::one) * ray_end_cm *
                     y_extent[cm],
      origin.z + dz.force_numerical_value_in(mp::one) * ray_end_cm *
                     z_extent[cm],
  };
  if (mapped_map_.isInsideBounds(occupied)) {
    mapped_map_.set(occupied, OCCUPIED);
  }
}

void Simulator::rotateTo(HorizontalAngle target) {
  const double max_rotate =
      std::abs(drone_config_.maxCommand.maxRotate.force_numerical_value_in(deg));
  if (max_rotate <= 0.0) {
    sim_state_.failed = true;
    sim_state_.failure_reason = "cannot rotate: max rotate is not positive";
    logger_.failure("rotate", sim_state_.failure_reason);
    return;
  }

  while (!sim_state_.failed) {
    const double current = Utils::degValue(drone_.getOrientation().horizontal);
    const double target_deg = normalizeDeg(Utils::degValue(target));
    const double left_delta = normalizeDeg(target_deg - current);
    if (left_delta < 1e-9 || std::abs(left_delta - 360.0) < 1e-9) {
      return;
    }

    const bool rotate_left = left_delta <= 180.0;
    const double remaining = rotate_left ? left_delta : 360.0 - left_delta;
    const double command = std::min(remaining, max_rotate);
    logger_.rotateCommand(rotate_left, command, current, target_deg);
    if (rotate_left) {
      drone_.rotateLeft(command * deg);
    } else {
      drone_.rotateRight(command * deg);
    }
    logger_.failure("rotate", sim_state_.failure_reason);
  }
}

void Simulator::moveToAdjacent(const GridCoord &from, const GridCoord &to) {
  const GridCoord delta = gridDelta(from, to);

  logger_.moveStep(from, to);

  if (delta.z != 0) {
    const double step = static_cast<double>(delta.z) * Utils::cmValue(res_z_);
    const double max_elevate =
        Utils::cmValue(drone_config_.maxCommand.maxElevate);
    if (max_elevate <= 0.0) {
      sim_state_.failed = true;
      sim_state_.failure_reason = "cannot elevate: max elevate is not positive";
      logger_.failure("elevate", sim_state_.failure_reason);
      return;
    }

    double remaining = step;
    while (!sim_state_.failed && std::abs(remaining) > 1e-9) {
      const double command =
          std::clamp(remaining, -max_elevate, max_elevate);
      logger_.elevateCommand(command);
      drone_.elevate(command * cm);
      logger_.failure("elevate", sim_state_.failure_reason);
      remaining -= command;
    }
    return;
  }

  HorizontalAngle target_heading{0.0 * deg};
  if (delta.x > 0) {
    target_heading = HorizontalAngle{0.0 * deg};
  } else if (delta.x < 0) {
    target_heading = HorizontalAngle{180.0 * deg};
  } else if (delta.y > 0) {
    target_heading = HorizontalAngle{90.0 * deg};
  } else if (delta.y < 0) {
    target_heading = HorizontalAngle{270.0 * deg};
  } else {
    return;
  }

  rotateTo(target_heading);
  if (sim_state_.failed) {
    return;
  }

  double remaining = Utils::cmValue(res_xy_);
  const double max_advance =
      Utils::cmValue(drone_config_.maxCommand.maxAdvance);
  if (max_advance <= 0.0) {
    sim_state_.failed = true;
    sim_state_.failure_reason = "cannot advance: max advance is not positive";
    logger_.failure("advance", sim_state_.failure_reason);
    return;
  }

  while (!sim_state_.failed && remaining > 1e-9) {
    const double command = std::min(remaining, max_advance);
    logger_.advanceCommand(command);
    drone_.advance(command * cm);
    logger_.failure("advance", sim_state_.failure_reason);
    remaining -= command;
  }
}

void Simulator::moveAlongPath(const std::vector<GridCoord> &path) {
  logger_.followPath(path.size());
  for (std::size_t i = 1; i < path.size() && !sim_state_.failed; ++i) {
    moveToAdjacent(path[i - 1], path[i]);
    mapped_map_.set(drone_.getPosition(), EMPTY);
    logger_.state("after move", drone_.getPosition(), drone_.getOrientation());
  }
}

/*
 * The simulator uses a deliberately simple occupancy-grid exploration
 * algorithm. The key rule is that all sensing and movement decisions go
 * through the IDrone interface: the simulator may command scans, rotations,
 * advances, and elevation changes, and it may read the drone position and
 * orientation, but it does not inspect the true map directly to decide what
 * to map. The true map remains behind the lidar and movement-driver mocks.
 *
 * Mapping is done as a sparse voxel occupancy grid in mapped_map_. Every
 * position starts as NOT_MAPPED because mapped_map_ is a DroneMap. At the
 * current drone position we scan six deterministic directions: four horizontal
 * compass directions and up/down. Each lidar result describes one emitted beam.
 * For that beam, the simulator combines the drone orientation with the
 * beam-relative orientation, steps along the ray at mission resolution, and
 * marks traversed voxels EMPTY. If the beam detected an obstacle, the endpoint
 * voxel is marked OCCUPIED. If the lidar reports distance 0, the obstacle is
 * too close to measure accurately, so only the nearest voxel in that beam
 * direction is marked OCCUPIED. If the beam did not detect an obstacle, the ray
 * is still useful: cells along the beam up to the reported max range are marked
 * EMPTY, but no endpoint is marked OCCUPIED.
 *
 * Exploration uses a frontier strategy over the discovered map. A frontier is
 * a known EMPTY voxel that has at least one in-bounds NOT_MAPPED 6-neighbor.
 * After scanning, the simulator runs BFS through known EMPTY voxels only, using
 * a fixed neighbor order (+X, -X, +Y, -Y, +Z, -Z), and chooses the nearest
 * reachable frontier. It then follows the BFS path one grid step at a time:
 * horizontal steps rotate toward the target neighbor and advance one XY
 * resolution, while vertical steps use elevate by one height resolution. Each
 * command is split into chunks that respect the configured max movement and
 * rotation limits. If the movement mock reports simulation failure, exploration
 * stops immediately.
 *
 * The loop terminates when there is no reachable frontier, when the simulation
 * fails, or when a conservative iteration cap is reached. The cap prevents an
 * infinite loop in cases where the lidar cannot make progress because of blind
 * spots or because all remaining unknown space is unreachable. Unreachable or
 * unobservable cells may legitimately remain NOT_MAPPED in the returned map.
 */
IMap3D &Simulator::simulate() {
  logger_.simulationStart(res_xy_, res_z_, drone_config_);
  logger_.state("initial state", drone_.getPosition(), drone_.getOrientation());

  mapped_map_.set(drone_.getPosition(), EMPTY);

  const std::size_t max_iterations =
      std::max<std::size_t>(1, inBoundsVoxelCount(mission_config_, res_xy_, res_z_) * 6);

  for (std::size_t iteration = 0;
       iteration < max_iterations && !sim_state_.failed;
       ++iteration) {
    logger_.iteration(iteration);
    logger_.state("before scan", drone_.getPosition(), drone_.getOrientation());
    scanCurrentPosition();
    const auto path = pathToNearestFrontier();
    if (!path.has_value()) {
      logger_.writeLine("simulation stop: no path to frontier");
      break;
    }
    moveAlongPath(*path);
  }

  if (sim_state_.failed) {
    logger_.stopFailed(sim_state_.failure_reason);
  }
  logger_.writeLine("simulation end");
  logger_.state("final state", drone_.getPosition(), drone_.getOrientation());

  return mapped_map_;
}
