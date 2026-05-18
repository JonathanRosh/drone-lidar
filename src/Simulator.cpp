#include "Simulator.h"
#include "Configs.h"
#include "GridCoord.h"
#include "LidarScanResult.h"
#include "MapGrid.h"
#include "Units.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using std::vector;

namespace {

constexpr double kPi = 3.14159265358979323846;

/** Rotate heading when path progress in one iteration is below safe_scan_step. */
constexpr double kStuckRecoveryRotateDeg = 30.0;

/** After this many failed tries, BFS skips the frontier cell. */
constexpr int kFrontierCooldownAttempts = 3;

struct ScanAngles {
  double horizontal_offset_deg;
  double altitude_offset_deg;
};

struct PathAdvanceResult {
  std::size_t waypoints_reached;
  double distance_cm;
};

double degToRad(double deg) { return deg * (kPi / 180.0); }

double radToDeg(double rad) { return rad * (180.0 / kPi); }

// --- Lidar / map integration ---

/**
 * Fixed survey boresight (body frame) for mapping iterations.
 *
 * - zMin / zMax (cm): vertical extents used at far range to set pitch center.
 * - zMax (cm): planning range for vertical angles (long view).
 * - d (cm): near clip; far range is at least d.
 * - fovc (deg): horizontal fan width; boresight stays centered at 0° so the
 *   sensor sweeps ±fovc/2 (wide scan). Wire fovc into the lidar mock when built.
 */
ScanAngles computeOptimalSurveyScan(const LidarConfig &lidar) {
  const double z_min_cm = lidar.zMin.numerical_value_in(cm);
  const double z_max_cm = lidar.zMax.numerical_value_in(cm);
  const double d_cm = lidar.d.numerical_value_in(cm);
  const double far_cm = std::max(z_max_cm, d_cm);

  const double elev_min_deg = radToDeg(std::atan2(z_min_cm, far_cm));
  const double elev_max_deg = radToDeg(std::atan2(z_max_cm, far_cm));

  ScanAngles angles{};
  angles.horizontal_offset_deg = 0.0;
  angles.altitude_offset_deg = 0.5 * (elev_min_deg + elev_max_deg);
  (void)lidar.fovc;
  return angles;
}

/** Wrap to (-180, 180] degrees. */
double normalizeAngleDeg(double degrees) {
  while (degrees <= -180.0) {
    degrees += 360.0;
  }
  while (degrees > 180.0) {
    degrees -= 360.0;
  }
  return degrees;
}

// --- Movement / orientation ---

/** Current drone yaw in world XY (degrees). */
double currentHeadingDeg(const SimulationState &sim_state) {
  return sim_state.drone_orientation.horizontal.numerical_value_in(deg);
}

/** Writes yaw into simulation_state (used after rotate commands). */
void setHeadingDeg(SimulationState &sim_state, double heading_deg) {
  sim_state.drone_orientation.horizontal = heading_deg * deg;
}

/** Heading for body +X forward in world XY (degrees, +X = 0, +Y = 90). */
double headingTowardDeg(double dx_cm, double dy_cm) {
  return radToDeg(std::atan2(dy_cm, dx_cm));
}

/**
 * Rotates in place toward target_heading using maxRotate chunks.
 * Updates simulation_state and issues rotate on drone + movement driver.
 */
void rotateTowardHeading(double target_heading_deg, IDrone &drone,
                         IMovementDriver &driver, SimulationState &sim_state,
                         const MaxCommand &max_command) {
  const double max_rot_deg = max_command.maxRotate.numerical_value_in(deg);
  if (max_rot_deg <= 0.0) {
    return;
  }

  double delta =
      normalizeAngleDeg(target_heading_deg - currentHeadingDeg(sim_state));

  while (std::abs(delta) > 1e-6) {
    const double step_deg = std::min(std::abs(delta), max_rot_deg);
    const Angle step = step_deg * deg;

    if (delta > 0.0) {
      drone.rotateRight(step);
      driver.rotateRight(step);
    } else {
      drone.rotateLeft(step);
      driver.rotateLeft(step);
    }

    setHeadingDeg(sim_state, currentHeadingDeg(sim_state) +
                                (delta > 0.0 ? step_deg : -step_deg));
    delta = normalizeAngleDeg(target_heading_deg -
                              currentHeadingDeg(sim_state));
  }
}

/** Builds a Position3D from raw cm values. */
Position3D positionFromCm(double x, double y, double z) {
  Position3D p{};
  p.x = x * cm;
  p.y = y * cm;
  p.z = z * cm;
  return p;
}

/**
 * Unit ray in body frame: +X forward, horizontal angle measured from +X
 * toward +Y (right-handed about +Z), altitude measured from the horizontal
 * plane toward +Z.
 */
void hitRayBodyUnit(double horiz_deg, double alt_deg, double *ux, double *uy,
                    double *uz) {
  const double th = degToRad(horiz_deg);
  const double ta = degToRad(alt_deg);
  const double ch = std::cos(th);
  const double sh = std::sin(th);
  const double ca = std::cos(ta);
  const double sa = std::sin(ta);
  *ux = ca * ch;
  *uy = ca * sh;
  *uz = sa;
}

/** Body direction to world: yaw about +Z, then pitch about +Y (radians). */
void bodyDirToWorld(double body_x, double body_y, double body_z, double yaw_rad,
                    double pitch_rad, double *wx, double *wy, double *wz) {
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);

  const double x1 = cp * body_x + sp * body_z;
  const double y1 = body_y;
  const double z1 = -sp * body_x + cp * body_z;

  *wx = cy * x1 - sy * y1;
  *wy = sy * x1 + cy * y1;
  *wz = z1;
}

/** World position of a lidar return from range + beam angles + drone pose. */
Position3D hitToPosition(const LidarHit &hit, const Position3D &sensor_position,
                         const Orientation &drone_orientation) {
  double bx = 0;
  double by = 0;
  double bz = 0;
  hitRayBodyUnit(hit.orientation.horizontal.numerical_value_in(deg),
                 hit.orientation.altitude.numerical_value_in(deg), &bx, &by,
                 &bz);

  const double yaw =
      degToRad(drone_orientation.horizontal.numerical_value_in(deg));
  const double pitch =
      degToRad(drone_orientation.altitude.numerical_value_in(deg));

  double wx = 0;
  double wy = 0;
  double wz = 0;
  bodyDirToWorld(bx, by, bz, yaw, pitch, &wx, &wy, &wz);

  const double L = hit.distance.numerical_value_in(cm);
  const double ox = sensor_position.x.numerical_value_in(cm) + L * wx;
  const double oy = sensor_position.y.numerical_value_in(cm) + L * wy;
  const double oz = sensor_position.z.numerical_value_in(cm) + L * wz;

  return positionFromCm(ox, oy, oz);
}

/**
 * Integrates each lidar ray into the drone map.
 *
 * This is **not** a full 3D DDA over every voxel the ray crosses: it samples
 * points along the segment from the sensor to the reported range at a fixed
 * step (half the smaller map cell size). That is usually enough to carve
 * NOT_MAPPED → EMPTY along line of sight; tighten the step if you need
 * thinner obstacle fidelity.
 *
 * Endpoint: reported range is assumed to be a surface return; the endpoint
 * cell is marked OCCUPIED when the ray was not blocked earlier.
 */
void updateMap(const LidarScanResult &scan_result, IMap3D &map,
               const Position3D &sensor_position,
               const Orientation &drone_orientation,
               const MissionConfig &mission_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const double step_cm = 0.5 * std::min(res_xy.numerical_value_in(cm),
                                        res_z.numerical_value_in(cm));
  if (step_cm <= 0.0) {
    return;
  }

  for (const LidarHit &hit : scan_result.hits) {
    const Position3D end =
        hitToPosition(hit, sensor_position, drone_orientation);

    const double sx = sensor_position.x.numerical_value_in(cm);
    const double sy = sensor_position.y.numerical_value_in(cm);
    const double sz = sensor_position.z.numerical_value_in(cm);
    const double ex = end.x.numerical_value_in(cm);
    const double ey = end.y.numerical_value_in(cm);
    const double ez = end.z.numerical_value_in(cm);

    double dx = ex - sx;
    double dy = ey - sy;
    double dz = ez - sz;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-12) {
      continue;
    }
    dx /= len;
    dy /= len;
    dz /= len;

    bool blocked = false;
    for (double dist = step_cm; dist < len - 1e-9; dist += step_cm) {
      const Position3D p =
          positionFromCm(sx + dx * dist, sy + dy * dist, sz + dz * dist);
      if (!map.isInsideBounds(p)) {
        blocked = true;
        break;
      }
      const Mapping cell = map.get(p);
      if (cell == OCCUPIED) {
        blocked = true;
        break;
      }
      if (cell == NOT_MAPPED) {
        map.set(p, EMPTY);
      }
    }

    if (!blocked && map.isInsideBounds(end) && map.get(end) != OCCUPIED) {
      map.set(end, OCCUPIED);
    }
  }
}

/** Drone orientation used when integrating a scan aimed at body-frame offsets. */
Orientation orientationForScan(const Orientation &base, double xy_offset_deg,
                               double altitude_offset_deg) {
  Orientation o = base;
  o.horizontal =
      (base.horizontal.numerical_value_in(deg) + xy_offset_deg) * deg;
  o.altitude = (base.altitude.numerical_value_in(deg) + altitude_offset_deg) *
               deg;
  return o;
}

/** Body-frame scan offsets to point the lidar at a world target from `from`. */
ScanAngles scanAnglesToward(const Position3D &from, const Position3D &target,
                            const Orientation &drone_orientation) {
  const double dx =
      target.x.numerical_value_in(cm) - from.x.numerical_value_in(cm);
  const double dy =
      target.y.numerical_value_in(cm) - from.y.numerical_value_in(cm);
  const double dz =
      target.z.numerical_value_in(cm) - from.z.numerical_value_in(cm);
  const double horiz = std::sqrt(dx * dx + dy * dy);

  const double world_yaw_deg = headingTowardDeg(dx, dy);
  const double world_pitch_deg =
      horiz > 1e-9 ? radToDeg(std::atan2(dz, horiz)) : 0.0;

  ScanAngles angles{};
  angles.horizontal_offset_deg = normalizeAngleDeg(
      world_yaw_deg - drone_orientation.horizontal.numerical_value_in(deg));
  angles.altitude_offset_deg =
      world_pitch_deg - drone_orientation.altitude.numerical_value_in(deg);
  return angles;
}

/** Runs scan at body offsets and merges hits into the drone map. */
void performScan(IDrone &drone, IMap3D &map, const SimulationState &sim_state,
                 const MissionConfig &mission_config, double body_xy_offset_deg,
                 double body_altitude_offset_deg) {
  const LidarScanResult scan =
      drone.scan(body_xy_offset_deg * deg, body_altitude_offset_deg * deg);
  const Orientation scan_orientation = orientationForScan(
      sim_state.drone_orientation, body_xy_offset_deg, body_altitude_offset_deg);
  updateMap(scan, map, sim_state.drone_position, scan_orientation,
            mission_config);
}

/** Wide survey scan for each mapping iteration (angles from lidar config). */
void scanSurvey(IDrone &drone, IMap3D &map, const SimulationState &sim_state,
                const MissionConfig &mission_config,
                const DroneConfig &drone_config) {
  const ScanAngles survey = computeOptimalSurveyScan(drone_config.lidarConfig);
  performScan(drone, map, sim_state, mission_config, survey.horizontal_offset_deg,
              survey.altitude_offset_deg);
}

/** Turns the drone in place by a fixed offset (stuck recovery). */
void rotateDroneRightBy(double degrees, IDrone &drone, IMovementDriver &driver,
                        SimulationState &sim_state,
                        const MaxCommand &max_command) {
  const double target =
      normalizeAngleDeg(currentHeadingDeg(sim_state) + degrees);
  rotateTowardHeading(target, drone, driver, sim_state, max_command);
}

/** Scan aimed at a world point (used before each path micro-step). */
void scanTowardTarget(IDrone &drone, IMap3D &map,
                      const SimulationState &sim_state,
                      const MissionConfig &mission_config,
                      const Position3D &target) {
  const ScanAngles angles =
      scanAnglesToward(sim_state.drone_position, target,
                       sim_state.drone_orientation);
  performScan(drone, map, sim_state, mission_config,
                angles.horizontal_offset_deg, angles.altitude_offset_deg);
}

// --- Frontier / pathfinding ---

/** Six face-adjacent grid cell centers within mission bounds. */
vector<Position3D> getNeighbors(const Position3D &position,
                                const MissionConfig &mission_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const GridCoord g = map_grid::worldToGrid(position, res_xy, res_z);

  static constexpr int kDx[] = {1, -1, 0, 0, 0, 0};
  static constexpr int kDy[] = {0, 0, 1, -1, 0, 0};
  static constexpr int kDz[] = {0, 0, 0, 0, 1, -1};

  vector<Position3D> out;
  out.reserve(6);
  for (int i = 0; i < 6; ++i) {
    const Position3D p = map_grid::gridToWorld(
        {g.x + kDx[i], g.y + kDy[i], g.z + kDz[i]}, res_xy, res_z);
    if (map_grid::insideMissionBounds(p, mission_config)) {
      out.push_back(p);
    }
  }
  return out;
}

/** True if EMPTY and touching at least one NOT_MAPPED neighbor. */
bool isFrontierCell(const Position3D &pos, IMap3D &map,
                    const MissionConfig &mission_config) {
  if (map.get(pos) != EMPTY) {
    return false;
  }
  for (const Position3D &n : getNeighbors(pos, mission_config)) {
    if (!map.isInsideBounds(n)) {
      continue;
    }
    if (map.get(n) == NOT_MAPPED) {
      return true;
    }
  }
  return false;
}

/** True if this frontier cell failed too many times and should be skipped. */
bool isFrontierCooled(
    const GridCoord &cell,
    const std::unordered_map<GridCoord, int, GridCoordHash> &failure_counts) {
  const auto it = failure_counts.find(cell);
  return it != failure_counts.end() &&
         it->second >= kFrontierCooldownAttempts;
}

/** Frontier cell that BFS may still target (not on cooldown). */
bool isUsableFrontier(const Position3D &pos, IMap3D &map,
                      const MissionConfig &mission_config,
                      const std::unordered_map<GridCoord, int, GridCoordHash>
                          &frontier_failure_counts) {
  if (!isFrontierCell(pos, map, mission_config)) {
    return false;
  }
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const GridCoord cell = map_grid::worldToGrid(pos, res_xy, res_z);
  return !isFrontierCooled(cell, frontier_failure_counts);
}

/** Increments failure count on miss; clears entry when the cell is reached. */
void recordFrontierAttempt(
    std::unordered_map<GridCoord, int, GridCoordHash> &failure_counts,
    const GridCoord &frontier_cell, bool reached_goal) {
  if (reached_goal) {
    failure_counts.erase(frontier_cell);
    return;
  }
  failure_counts[frontier_cell]++;
}

/** Rebuilds world positions along the BFS parent chain (start → frontier). */
vector<Position3D> reconstructPath(
    GridCoord frontier, const GridCoord &start,
    const std::unordered_map<GridCoord, GridCoord, GridCoordHash> &parent,
    Distance res_xy, Distance res_z) {
  vector<Position3D> rev;
  GridCoord cur = frontier;
  for (;;) {
    rev.push_back(map_grid::gridToWorld(cur, res_xy, res_z));
    if (cur == start) {
      break;
    }
    const auto it = parent.find(cur);
    if (it == parent.end()) {
      break;
    }
    cur = it->second;
  }
  std::reverse(rev.begin(), rev.end());
  return rev;
}

/**
 * BFS over known EMPTY cells to the nearest usable frontier.
 * Skips frontier cells on cooldown.
 */
vector<Position3D> getPathToFrontier(
    const Position3D &drone_position, IMap3D &map,
    const MissionConfig &mission_config,
    const std::unordered_map<GridCoord, int, GridCoordHash>
        &frontier_failure_counts) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);

  const GridCoord start = map_grid::worldToGrid(drone_position, res_xy, res_z);
  const Position3D start_world = map_grid::gridToWorld(start, res_xy, res_z);

  if (!map.isInsideBounds(start_world)) {
    return {};
  }
  const Mapping start_mapping = map.get(start_world);
  if (start_mapping == OCCUPIED || start_mapping == OUTSIDE_BOUNDARY) {
    return {};
  }

  if (start_mapping == EMPTY &&
      isUsableFrontier(start_world, map, mission_config,
                       frontier_failure_counts)) {
    return {start_world};
  }

  if (start_mapping != EMPTY) {
    return {};
  }

  std::queue<GridCoord> q;
  std::unordered_map<GridCoord, GridCoord, GridCoordHash> parent;

  q.push(start);

  while (!q.empty()) {
    const GridCoord cur = q.front();
    q.pop();
    const Position3D cur_world = map_grid::gridToWorld(cur, res_xy, res_z);

    if (isUsableFrontier(cur_world, map, mission_config,
                         frontier_failure_counts)) {
      return reconstructPath(cur, start, parent, res_xy, res_z);
    }

    for (const Position3D &nworld : getNeighbors(cur_world, mission_config)) {
      if (!map.isInsideBounds(nworld)) {
        continue;
      }
      if (map.get(nworld) != EMPTY) {
        continue;
      }
      const GridCoord next = map_grid::worldToGrid(nworld, res_xy, res_z);
      if (parent.contains(next)) {
        continue;
      }
      if (next == start) {
        continue;
      }
      parent.emplace(next, cur);
      q.push(next);
    }
  }

  return {};
}

// --- Safe movement along a path ---

/** Euclidean distance between two positions (cm). */
double distanceCm(const Position3D &a, const Position3D &b) {
  const double dx = a.x.numerical_value_in(cm) - b.x.numerical_value_in(cm);
  const double dy = a.y.numerical_value_in(cm) - b.y.numerical_value_in(cm);
  const double dz = a.z.numerical_value_in(cm) - b.z.numerical_value_in(cm);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/** True if both positions map to the same grid cell. */
bool sameGridCell(const Position3D &a, const Position3D &b,
                  const MissionConfig &mission_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  return map_grid::worldToGrid(a, res_xy, res_z) ==
         map_grid::worldToGrid(b, res_xy, res_z);
}

/** Max distance per micro-move before the next scan (trust lidar range + command limits). */
Distance computeSafeScanStep(const DroneConfig &drone_config) {
  const Distance z_max = drone_config.lidarConfig.zMax;
  const Distance max_adv = drone_config.maxCommand.maxAdvance;
  const Distance max_el = drone_config.maxCommand.maxElevate;
  const double z_cm = z_max.numerical_value_in(cm);
  const double adv_cm = max_adv.numerical_value_in(cm);
  const double el_cm = max_el.numerical_value_in(cm);
  const double step_cm = std::min({z_cm, adv_cm, el_cm});
  return step_cm * cm;
}

/** Half-extent in cells for minPass clearance checks. */
int clearanceCells(Distance half_extent, Distance resolution) {
  if (half_extent.numerical_value_in(cm) <= 0.0) {
    return 0;
  }
  const double res = resolution.numerical_value_in(cm);
  return static_cast<int>(
      std::ceil(half_extent.numerical_value_in(cm) / res - 1e-12));
}

/** True if every cell in the drone minPass box at center is known-empty. */
bool fitsMinPassAt(const Position3D &center, IMap3D &map,
                   const MissionConfig &mission_config,
                   const DroneConfig &drone_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const GridCoord c = map_grid::worldToGrid(center, res_xy, res_z);

  const MinPass &mp = drone_config.minPass;
  const int rx = clearanceCells(mp.length / 2, res_xy);
  const int ry = clearanceCells(mp.width / 2, res_xy);
  const int rz = clearanceCells(mp.height / 2, res_z);

  for (int ix = -rx; ix <= rx; ++ix) {
    for (int iy = -ry; iy <= ry; ++iy) {
      for (int iz = -rz; iz <= rz; ++iz) {
        const Position3D p = map_grid::gridToWorld(
            {c.x + ix, c.y + iy, c.z + iz}, res_xy, res_z);
        if (!map.isInsideBounds(p)) {
          return false;
        }
        const Mapping m = map.get(p);
        if (m != EMPTY) {
          return false;
        }
      }
    }
  }
  return true;
}

/** Point move_cm along the segment from → to (capped at the target). */
Position3D lerpToward(const Position3D &from, const Position3D &to,
                      double move_cm) {
  const double dist = distanceCm(from, to);
  if (dist < 1e-12) {
    return from;
  }
  const double t = std::min(1.0, move_cm / dist);
  const double fx = from.x.numerical_value_in(cm);
  const double fy = from.y.numerical_value_in(cm);
  const double fz = from.z.numerical_value_in(cm);
  const double tx = to.x.numerical_value_in(cm);
  const double ty = to.y.numerical_value_in(cm);
  const double tz = to.z.numerical_value_in(cm);
  return positionFromCm(fx + t * (tx - fx), fy + t * (ty - fy),
                        fz + t * (tz - fz));
}

/**
 * Commands rotation (XY), then horizontal advance along body forward, then
 * vertical elevate. Elevation does not change heading.
 */
void applyCommandedMove(const Position3D &from, const Position3D &to,
                        IDrone &drone, IMovementDriver &driver,
                        SimulationState &sim_state,
                        const MaxCommand &max_command) {
  const double dx = to.x.numerical_value_in(cm) - from.x.numerical_value_in(cm);
  const double dy = to.y.numerical_value_in(cm) - from.y.numerical_value_in(cm);
  const double dz = to.z.numerical_value_in(cm) - from.z.numerical_value_in(cm);
  const double horiz = std::sqrt(dx * dx + dy * dy);

  const double max_adv = max_command.maxAdvance.numerical_value_in(cm);
  const double max_el = max_command.maxElevate.numerical_value_in(cm);

  if (horiz > 1e-9) {
    rotateTowardHeading(headingTowardDeg(dx, dy), drone, driver, sim_state,
                        max_command);

    double remaining_h = horiz;
    while (remaining_h > 1e-9) {
      const double chunk = std::min(remaining_h, max_adv);
      drone.advance(chunk * cm);
      driver.advance(chunk * cm);
      remaining_h -= chunk;
    }
  }

  double remaining_z = std::abs(dz);
  while (remaining_z > 1e-9) {
    const double chunk = std::min(remaining_z, max_el);
    drone.elevate(chunk * cm);
    driver.elevate(chunk * cm);
    remaining_z -= chunk;
  }
}

/**
 * Moves toward target in steps of at most safe_scan_step. After each step,
 * runs scan + map update. Stops when target is reached, passage is unsafe,
 * or a step cannot be taken. Returns false if movement was aborted early.
 */
bool followPathTowardWaypoint(const Position3D &waypoint, Distance safe_scan_step,
                              IDrone &drone, IMap3D &map,
                              IMovementDriver &movement_driver,
                              SimulationState &sim_state,
                              const MissionConfig &mission_config,
                              const DroneConfig &drone_config) {
  const double max_step_cm = safe_scan_step.numerical_value_in(cm);
  const MaxCommand &max_cmd = drone_config.maxCommand;

  while (distanceCm(sim_state.drone_position, waypoint) > 1e-6) {
    if (!fitsMinPassAt(sim_state.drone_position, map, mission_config,
                       drone_config)) {
      return false;
    }

    const double leg_cm = distanceCm(sim_state.drone_position, waypoint);
    const double move_cm = std::min(max_step_cm, leg_cm);

    const Position3D next =
        lerpToward(sim_state.drone_position, waypoint, move_cm);

    if (!fitsMinPassAt(next, map, mission_config, drone_config)) {
      return false;
    }

    scanTowardTarget(drone, map, sim_state, mission_config, next);

    applyCommandedMove(sim_state.drone_position, next, drone, movement_driver,
                       sim_state, max_cmd);
    sim_state.drone_position = next;

    if (!fitsMinPassAt(sim_state.drone_position, map, mission_config,
                       drone_config)) {
      return false;
    }
  }

  return true;
}

/**
 * Walks grid waypoints on the frontier path using safe scan steps between
 * waypoints. Returns how many waypoints were reached and total distance moved.
 */
PathAdvanceResult advanceAlongFrontierPath(const vector<Position3D> &path,
                                           Distance safe_scan_step,
                                           IDrone &drone, IMap3D &map,
                                           IMovementDriver &driver,
                                           SimulationState &sim_state,
                                           const MissionConfig &mission_config,
                                           const DroneConfig &drone_config) {
  if (path.empty()) {
    return {0, 0.0};
  }

  const Position3D path_start = sim_state.drone_position;

  std::size_t waypoint_index = 0;
  if (path.size() > 1 &&
      sameGridCell(path.front(), sim_state.drone_position, mission_config)) {
    waypoint_index = 1;
  }

  for (; waypoint_index < path.size(); ++waypoint_index) {
    if (!followPathTowardWaypoint(path[waypoint_index], safe_scan_step, drone,
                                  map, driver, sim_state, mission_config,
                                  drone_config)) {
      break;
    }
  }

  return {waypoint_index,
          distanceCm(path_start, sim_state.drone_position)};
}

/** Places the drone at the lexicographically first free cell in the true map. */
void setInitialPosition(TrueMap &true_map, SimulationState &sim_state) {
  const std::optional<Position3D> start =
      true_map.firstUnoccupiedPositionLexOrder();
  if (!start) {
    throw std::runtime_error(
        "TrueMap has no in-bounds cell that is not OCCUPIED");
  }
  sim_state.drone_position = *start;
}

} // namespace

Simulator::Simulator(IDrone &drone, TrueMap &true_map, IMap3D &map,
                     ILidarSensor &lidar_sensor,
                     IPositionSensor &position_sensor,
                     IMovementDriver &movement_driver,
                     const MissionConfig &mission_config,
                     const DroneConfig &drone_config)
    : drone(&drone), true_map(true_map), map(&map), lidar_sensor(&lidar_sensor),
      position_sensor(&position_sensor), movement_driver(&movement_driver),
      mission_config_(mission_config), drone_config_(drone_config) {}

/**
 * Autonomous mapping loop: survey → plan frontier → move safely → repeat.
 * Returns the drone's built map when no frontier remains (or all are cooled).
 */
IMap3D &Simulator::simulate() {
  setInitialPosition(true_map, simulation_state);

  // Per-run frontier cooldown (lives only inside simulate, not part of class API).
  std::unordered_map<GridCoord, int, GridCoordHash> frontier_failure_counts;

  const Distance safe_scan_step = computeSafeScanStep(drone_config_);
  const double step_threshold_cm = safe_scan_step.numerical_value_in(cm);
  const Distance res_xy = map_grid::xyResolution(mission_config_);
  const Distance res_z = map_grid::zResolution(mission_config_);

  while (true) {
    // 1. Survey scan from current pose (wide cone, angles from lidar config).
    scanSurvey(*drone, *map, simulation_state, mission_config_, drone_config_);

    // 2. BFS to the nearest frontier cell not on cooldown.
    const vector<Position3D> frontier_path = getPathToFrontier(
        simulation_state.drone_position, *map, mission_config_,
        frontier_failure_counts);

    if (frontier_path.empty()) {
      break; // No frontier left (mapping complete or all cooled).
    }

    const Position3D &frontier_goal = frontier_path.back();
    const GridCoord frontier_cell =
        map_grid::worldToGrid(frontier_goal, res_xy, res_z);

    // 3. Already sitting on a frontier with nowhere else to go → done.
    if (frontier_path.size() == 1 &&
        sameGridCell(frontier_goal, simulation_state.drone_position,
                     mission_config_) &&
        isFrontierCell(simulation_state.drone_position, *map,
                       mission_config_)) {
      break;
    }

    // 4. Follow path in safe steps (scan toward each step, minPass checks).
    const PathAdvanceResult advance = advanceAlongFrontierPath(
        frontier_path, safe_scan_step, *drone, *map, *movement_driver,
        simulation_state, mission_config_, drone_config_);

    const bool reached_goal =
        sameGridCell(simulation_state.drone_position, frontier_goal,
                     mission_config_);
    recordFrontierAttempt(frontier_failure_counts, frontier_cell, reached_goal);

    if (reached_goal) {
      continue; // Replan from the frontier cell we reached.
    }

    if (advance.waypoints_reached == 0) {
      continue; // Blocked; failure recorded → try another frontier next iter.
    }

    // 5. Little progress this iteration → rotate and replan (avoid corner lock).
    if (advance.distance_cm < step_threshold_cm - 1e-9) {
      rotateDroneRightBy(kStuckRecoveryRotateDeg, *drone, *movement_driver,
                         simulation_state, drone_config_.maxCommand);
      continue;
    }
  }

  return *map;
}
