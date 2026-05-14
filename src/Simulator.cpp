#include "Simulator.h"
#include "Configs.h"
#include "GridCoord.h"
#include "LidarScanResult.h"
#include "MapGrid.h"
#include "Units.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using std::vector;

namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg) { return deg * (kPi / 180.0); }

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
  const double step_cm =
      0.5 * std::min(res_xy.numerical_value_in(cm), res_z.numerical_value_in(cm));
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

vector<Position3D> getPathToFrontier(const Position3D &drone_position,
                                     IMap3D &map,
                                     const MissionConfig &mission_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);

  const GridCoord start =
      map_grid::worldToGrid(drone_position, res_xy, res_z);
  const Position3D start_world =
      map_grid::gridToWorld(start, res_xy, res_z);

  if (!map.isInsideBounds(start_world)) {
    return {};
  }
  const Mapping start_mapping = map.get(start_world);
  if (start_mapping == OCCUPIED || start_mapping == OUTSIDE_BOUNDARY) {
    return {};
  }

  if (start_mapping == EMPTY && isFrontierCell(start_world, map, mission_config)) {
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

    if (isFrontierCell(cur_world, map, mission_config)) {
      return reconstructPath(cur, start, parent, res_xy, res_z);
    }

    for (const Position3D &nworld :
         getNeighbors(cur_world, mission_config)) {
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

} // namespace


Simulator::Simulator(IDrone &drone, TrueMap &true_map, IMap3D &map,
                     ILidarSensor &lidar_sensor,
                     IPositionSensor &position_sensor,
                     IMovementDriver &movement_driver,
                     const MissionConfig &mission_config)
    : drone(&drone), true_map(true_map), map(&map), lidar_sensor(&lidar_sensor),
      position_sensor(&position_sensor), movement_driver(&movement_driver),
      mission_config_(mission_config) {}

void Simulator::setInitialPosition() {
  const std::optional<Position3D> start =
      true_map.firstUnoccupiedPositionLexOrder();
  if (!start) {
    throw std::runtime_error(
        "TrueMap has no in-bounds cell that is not OCCUPIED");
  }
  simulation_state.drone_position = *start;
}


IMap3D &Simulator::simulate() {
  setInitialPosition();

  while (true) {
    const LidarScanResult scan_result = drone->scan(0. * deg, 0. * deg);

    
    updateMap(scan_result, *map, position_sensor->getPosition(),
              position_sensor->getOrientation(), mission_config_);

    const vector<Position3D> frontier_path =
        getPathToFrontier(drone->getPosition(), *map, mission_config_);


    if (frontier_path.empty()) {
      break;
    }
    // TODO: move the drone along frontier_path with safe scanning zone
    (void)frontier_path;
    break;
  }

  return *map;
}
