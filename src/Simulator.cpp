#include "Simulator.h"
#include "Configs.h"
#include "LidarScanResult.h"
#include "MapGrid.h"
#include "TrueMap.h"
#include "Units.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

using std::vector;

namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg) { return deg * (kPi / 180.0); }

double radToDeg(double rad) { return rad * (180.0 / kPi); }

/** Wrap to [-180, 180) degrees. */
double normalizeAngleDeg(double degrees) {
  while (degrees < -180.0) {
    degrees += 360.0;
  }
  while (degrees >= 180.0) {
    degrees -= 360.0;
  }
  return degrees;
}

double headingTowardDeg(double dx_cm, double dy_cm) {
  return radToDeg(std::atan2(dy_cm, dx_cm));
}

void rotateTowardHeading(double target_heading_deg, IDrone &drone,
                         IPositionSensor &position_sensor,
                         const MaxCommand &max_command) {
  const double max_rot_deg = max_command.maxRotate.numerical_value_in(deg);
  if (max_rot_deg <= 0.0) {
    return;
  }

  double delta = normalizeAngleDeg(
      target_heading_deg -
      position_sensor.getOrientation().horizontal.numerical_value_in(deg));

  for (int guard = 0; guard < 16 && std::abs(delta) > 1e-6; ++guard) {
    const double prev = std::abs(delta);
    const double step_deg = std::min(std::abs(delta), max_rot_deg);
    const Angle step = step_deg * deg;

    // MovementDriverMock: rotateLeft increases heading, rotateRight decreases.
    if (delta > 0.0) {
      drone.rotateLeft(step);
    } else {
      drone.rotateRight(step);
    }

    delta = normalizeAngleDeg(
        target_heading_deg -
        position_sensor.getOrientation().horizontal.numerical_value_in(deg));
    if (std::abs(delta) >= prev - 1e-6) {
      break;
    }
  }
}

Position3D positionFromCm(double x, double y, double z) {
  Position3D p{};
  p.x = x * cm;
  p.y = y * cm;
  p.z = z * cm;
  return p;
}

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

Position3D hitToPosition(const LidarHit &hit, const Position3D &sensor_position,
                         const Orientation &sensor_orientation) {
  const double horiz_deg =
      sensor_orientation.horizontal.numerical_value_in(deg) +
      hit.orientation.horizontal.numerical_value_in(deg);
  const double alt_deg = sensor_orientation.altitude.numerical_value_in(deg) +
                         hit.orientation.altitude.numerical_value_in(deg);

  double wx = 0;
  double wy = 0;
  double wz = 0;
  hitRayBodyUnit(horiz_deg, alt_deg, &wx, &wy, &wz);

  const double L = hit.distance.numerical_value_in(cm);
  const double ox = sensor_position.x.numerical_value_in(cm) + L * wx;
  const double oy = sensor_position.y.numerical_value_in(cm) + L * wy;
  const double oz = sensor_position.z.numerical_value_in(cm) + L * wz;

  return positionFromCm(ox, oy, oz);
}

void updateMap(const LidarScanResult &scan_result, IMap3D &map,
               const Position3D &sensor_position,
               const Orientation &sensor_orientation,
               const MissionConfig &mission_config) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const double step_cm = 0.5 * std::min(res_xy.numerical_value_in(cm),
                                        res_z.numerical_value_in(cm));
  if (step_cm <= 0.0) {
    return;
  }

  for (const LidarHit &hit : scan_result) {
    LidarHit effective = hit;
    if (effective.distance.numerical_value_in(cm) < 1e-6) {
      effective.distance = Distance{2.0 * cm};
    }
    const Position3D end =
        hitToPosition(effective, sensor_position, sensor_orientation);

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

void performScan(IDrone &drone, IMap3D &map, IPositionSensor &position_sensor,
                 const MissionConfig &mission_config, double body_xy_offset_deg,
                 double body_altitude_offset_deg) {
  const LidarScanResult scan =
      drone.scan(body_xy_offset_deg * deg, body_altitude_offset_deg * deg);
  updateMap(scan, map, position_sensor.getPosition(),
            position_sensor.getOrientation(), mission_config);
}

void scanSurvey(IDrone &drone, IMap3D &map, IPositionSensor &position_sensor,
                const MissionConfig &mission_config,
                const LidarConfig &lidar_config) {
  const unsigned int fovc = std::max(1u, lidar_config.fovc);
  const double pitch_step = (fovc <= 2) ? 10.0 : 15.0;

  // Boresight: wide vertical ladder so floor/ceiling and tube Z-samples get
  // EMPTY.
  for (double alt = -30.0; alt <= 30.0 + 1e-6; alt += 10.0) {
    performScan(drone, map, position_sensor, mission_config, 0.0, alt);
  }

  // Axis-aligned body headings: layered pitch.
  for (double yaw = 0.0; yaw < 360.0; yaw += 90.0) {
    for (double alt :
         {-26.0, -18.0, -pitch_step * 1.6, pitch_step * 1.6, 18.0, 26.0}) {
      performScan(drone, map, position_sensor, mission_config, yaw, alt);
    }
    const double diag_alt = -((fovc <= 2) ? 12.0 : 15.0);
    performScan(drone, map, position_sensor, mission_config, yaw, diag_alt);
  }

  // 45° headings: LiDAR only sends a few beams per pose; diagonals cover voxels
  // between the ±X/±Y passes without moving the drone.
  for (double yaw : {45.0, 135.0, 225.0, 315.0}) {
    for (double alt : {-25.0, -15.0, -5.0, 5.0, 15.0, 25.0}) {
      performScan(drone, map, position_sensor, mission_config, yaw, alt);
    }
  }
}

void applyCommandedMove(const Position3D &from, const Position3D &to,
                        IDrone &drone, IPositionSensor &position_sensor,
                        const MaxCommand &max_command) {
  const double dx = to.x.numerical_value_in(cm) - from.x.numerical_value_in(cm);
  const double dy = to.y.numerical_value_in(cm) - from.y.numerical_value_in(cm);
  const double dz = to.z.numerical_value_in(cm) - from.z.numerical_value_in(cm);
  const double horiz = std::sqrt(dx * dx + dy * dy);

  const double max_adv = max_command.maxAdvance.numerical_value_in(cm);
  const double max_el = max_command.maxElevate.numerical_value_in(cm);

  if (horiz > 1e-9) {
    rotateTowardHeading(headingTowardDeg(dx, dy), drone, position_sensor,
                        max_command);

    double remaining_h = horiz;
    while (remaining_h > 1e-9) {
      const double chunk = std::min(remaining_h, max_adv);
      drone.advance(chunk * cm);
      remaining_h -= chunk;
    }
  }

  if (std::abs(dz) > 1e-9) {
    const double sign = dz >= 0.0 ? 1.0 : -1.0;
    double remaining_z = std::abs(dz);
    while (remaining_z > 1e-9) {
      const double chunk = std::min(remaining_z, max_el);
      drone.elevate(sign * chunk * cm);
      remaining_z -= chunk;
    }
  }
}

// Survey → DroneMap → forward steps; row end → U-turn (90°→sidestep≤
// min_pass.length→90°) or 4× fallback 90° + rescan; cage / 100 row-blocks →
// next Z. Planning uses DroneMap only; lidar mock still reads TrueMap.

constexpr int kMaxRowBlocksPerLevel = 100;
constexpr double kStepEpsCm = 1e-3;
constexpr int kMaxRescansBeforeRowBlock = 24;

struct LevelGeometry {
  double zmin_interior;
  double zmax_interior;
  double layer_spacing_cm;
};

LevelGeometry computeLevelGeometry(const MissionConfig &mission_config,
                                   const DroneConfig &drone_config) {
  const auto &b = mission_config.map_boundry;
  const Distance res_z = map_grid::zResolution(mission_config);
  const double res_z_cm = res_z.numerical_value_in(cm);

  const double margin_xy =
      std::max(0.5 * drone_config.minPass.width.numerical_value_in(cm),
               0.5 * drone_config.minPass.length.numerical_value_in(cm));
  const double margin_z =
      std::max(0.5 * drone_config.minPass.height.numerical_value_in(cm),
               drone_config.lidarConfig.zMin.numerical_value_in(cm));

  const double zmin_interior = b.minHeight.numerical_value_in(cm) + margin_z;
  const double zmax_interior = b.maxHeight.numerical_value_in(cm) - margin_z;
  const unsigned int fovc = std::max(1u, drone_config.lidarConfig.fovc);
  const double z_range = std::max(0.0, zmax_interior - zmin_interior);
  const double layer_spacing =
      std::max(res_z_cm, z_range / static_cast<double>(fovc + 1));

  (void)margin_xy;
  return LevelGeometry{zmin_interior, zmax_interior, layer_spacing};
}

vector<double> sweepHeightLevels(const LevelGeometry &geom) {
  vector<double> levels;
  if (geom.zmax_interior < geom.zmin_interior - 1e-6) {
    return levels;
  }
  for (double z = geom.zmin_interior; z <= geom.zmax_interior + 1e-6;
       z += geom.layer_spacing_cm) {
    levels.push_back(z);
  }
  if (levels.empty()) {
    levels.push_back(geom.zmin_interior);
  }
  return levels;
}

void forwardUnitXY(double heading_deg, double *ux, double *uy) {
  const double th = degToRad(heading_deg);
  *ux = std::cos(th);
  *uy = std::sin(th);
}

void rotateRightBy(IDrone &drone, const MaxCommand &max_command,
                   double degrees_total) {
  const double max_rot = max_command.maxRotate.numerical_value_in(deg);
  if (max_rot <= 1e-9 || degrees_total <= 1e-9) {
    return;
  }
  double done = 0.0;
  for (int guard = 0; guard < 64 && done < degrees_total - 1e-6; ++guard) {
    const double step = std::min(max_rot, degrees_total - done);
    drone.rotateRight(step * deg);
    done += step;
  }
}

void rotateRightQuarterTurn(IDrone &drone, const MaxCommand &max_command) {
  const double max_rot =
      std::max(1e-6, max_command.maxRotate.numerical_value_in(deg));
  const int n = std::max(1, static_cast<int>(std::ceil(90.0 / max_rot)));
  const double chunk = 90.0 / static_cast<double>(n);
  for (int i = 0; i < n; ++i) {
    rotateRightBy(drone, max_command, chunk);
  }
}

bool cellAllowsPassage(const IMap3D &map, const Position3D &p) {
  if (!map.isInsideBounds(p)) {
    return false;
  }
  const Mapping m = map.get(p);
  if (m == NOT_MAPPED || m == OUTSIDE_BOUNDARY || m == OCCUPIED) {
    return false;
  }
  return m == EMPTY;
}

/**
 * Min-pass tube along body XY: centerline at (cx,cy,cz), forward (ux,uy),
 * lateral half-width and vertical half-height from minPass. Sampled on grid.
 */
bool bodyTubeClearAt(const IMap3D &map, const MissionConfig &mission_config,
                     const DroneConfig &drone_config, double cx, double cy,
                     double cz, double ux, double uy) {
  const auto &b = mission_config.map_boundry;
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const Distance res_z = map_grid::zResolution(mission_config);
  const double rx = std::max(1e-6, res_xy.numerical_value_in(cm));
  const double rz = std::max(1e-6, res_z.numerical_value_in(cm));

  const double half_w = 0.5 * drone_config.minPass.width.numerical_value_in(cm);
  const double half_h =
      0.5 * drone_config.minPass.height.numerical_value_in(cm);

  const double px = -uy;
  const double py = ux;

  const double z_mn = b.minHeight.numerical_value_in(cm);
  const double z_mx = b.maxHeight.numerical_value_in(cm);
  const double z0 = std::max(z_mn, cz - half_h);
  const double z1 = std::min(z_mx, cz + half_h);

  const double x_mn = b.minX.numerical_value_in(cm);
  const double x_mx = b.maxX.numerical_value_in(cm);
  const double y_mn = b.minY.numerical_value_in(cm);
  const double y_mx = b.maxY.numerical_value_in(cm);

  for (double w = -half_w; w <= half_w + 1e-9; w += rx) {
    double wx = cx + px * w;
    double wy = cy + py * w;
    if (wx < x_mn || wx > x_mx || wy < y_mn || wy > y_mx) {
      return false;
    }
    for (double z = z0; z <= z1 + 1e-9; z += rz) {
      if (!cellAllowsPassage(map, positionFromCm(wx, wy, z))) {
        return false;
      }
    }
  }
  return true;
}

/**
 * Furthest distance t in [0, plan_horizon] such that the whole forward segment
 * [res_xy, t] passes the min-pass tube (DroneMap only; NOT_MAPPED blocks).
 */
double maxClearForwardDistance(const IMap3D &map,
                               const MissionConfig &mission_config,
                               const DroneConfig &drone_config,
                               const Position3D &pos, double heading_deg) {
  const Distance res_xy = map_grid::xyResolution(mission_config);
  const double rx = std::max(1e-6, res_xy.numerical_value_in(cm));

  const double sx = pos.x.numerical_value_in(cm);
  const double sy = pos.y.numerical_value_in(cm);
  const double sz = pos.z.numerical_value_in(cm);

  double ux = 0;
  double uy = 0;
  forwardUnitXY(heading_deg, &ux, &uy);

  const double z_max = drone_config.lidarConfig.zMax.numerical_value_in(cm);
  const double min_pass_len =
      drone_config.minPass.length.numerical_value_in(cm);
  const double plan_horizon = std::max(z_max, min_pass_len);
  const double max_adv =
      drone_config.maxCommand.maxAdvance.numerical_value_in(cm);
  const double cap = std::min(plan_horizon, max_adv);

  double max_good = 0.0;
  for (double t = rx; t <= cap + 1e-9; t += rx) {
    const double cx = sx + ux * t;
    const double cy = sy + uy * t;
    if (!bodyTubeClearAt(map, mission_config, drone_config, cx, cy, sz, ux,
                         uy)) {
      break;
    }
    max_good = t;
  }
  return max_good;
}

void advanceForwardHorizontal(IDrone &drone, IPositionSensor &position_sensor,
                              const MaxCommand &max_command, double dist_cm) {
  if (dist_cm <= kStepEpsCm) {
    return;
  }
  const Position3D from = position_sensor.getPosition();
  const double hdg =
      position_sensor.getOrientation().horizontal.numerical_value_in(deg);
  double ux = 0;
  double uy = 0;
  forwardUnitXY(hdg, &ux, &uy);
  const Position3D to =
      positionFromCm(from.x.numerical_value_in(cm) + ux * dist_cm,
                     from.y.numerical_value_in(cm) + uy * dist_cm,
                     from.z.numerical_value_in(cm));
  applyCommandedMove(from, to, drone, position_sensor, max_command);
}

bool forwardClearAfterSurvey(IDrone &drone, IMap3D &map,
                             IPositionSensor &position_sensor,
                             const MissionConfig &mission_config,
                             const DroneConfig &drone_config) {
  scanSurvey(drone, map, position_sensor, mission_config,
             drone_config.lidarConfig);
  const double step = maxClearForwardDistance(
      map, mission_config, drone_config, position_sensor.getPosition(),
      position_sensor.getOrientation().horizontal.numerical_value_in(deg));
  return step > kStepEpsCm;
}

/** Right turn → sidestep (min_pass_length cap) → right turn → new row heading.
 */
bool recoverFromRowBlock(IDrone &drone, IMap3D &map,
                         IPositionSensor &position_sensor,
                         const MissionConfig &mission_config,
                         const DroneConfig &drone_config) {
  rotateRightQuarterTurn(drone, drone_config.maxCommand);
  const bool can_sidestep_begin = forwardClearAfterSurvey(
      drone, map, position_sensor, mission_config, drone_config);

  if (can_sidestep_begin) {
    const double side_cap = drone_config.minPass.length.numerical_value_in(cm);
    const double side_step = std::min(
        side_cap,
        maxClearForwardDistance(
            map, mission_config, drone_config, position_sensor.getPosition(),
            position_sensor.getOrientation().horizontal.numerical_value_in(
                deg)));
    if (side_step > kStepEpsCm) {
      advanceForwardHorizontal(drone, position_sensor, drone_config.maxCommand,
                               side_step);
      rotateRightQuarterTurn(drone, drone_config.maxCommand);
      if (forwardClearAfterSurvey(drone, map, position_sensor, mission_config,
                                  drone_config)) {
        return true;
      }
    }
  }

  for (int i = 0; i < 4; ++i) {
    rotateRightQuarterTurn(drone, drone_config.maxCommand);
    if (forwardClearAfterSurvey(drone, map, position_sensor, mission_config,
                                drone_config)) {
      return true;
    }
  }
  return false;
}

void runUTurnLevelSweep(IDrone &drone, IMap3D &map,
                        IPositionSensor &position_sensor,
                        const MissionConfig &mission_config,
                        const DroneConfig &drone_config, double layer_z_cm) {
  {
    const Position3D p = position_sensor.getPosition();
    const Position3D to = positionFromCm(
        p.x.numerical_value_in(cm), p.y.numerical_value_in(cm), layer_z_cm);
    applyCommandedMove(p, to, drone, position_sensor, drone_config.maxCommand);
  }

  int row_block_count = 0;

  for (;;) {
    for (;;) {
      double step = 0.0;
      for (int rescan = 0; rescan < kMaxRescansBeforeRowBlock; ++rescan) {
        scanSurvey(drone, map, position_sensor, mission_config,
                   drone_config.lidarConfig);
        step = maxClearForwardDistance(
            map, mission_config, drone_config, position_sensor.getPosition(),
            position_sensor.getOrientation().horizontal.numerical_value_in(
                deg));
        if (step > kStepEpsCm) {
          break;
        }
      }
      if (step <= kStepEpsCm) {
        break;
      }
      advanceForwardHorizontal(drone, position_sensor, drone_config.maxCommand,
                               step);
    }

    ++row_block_count;
    if (row_block_count > kMaxRowBlocksPerLevel) {
      return;
    }

    if (!recoverFromRowBlock(drone, map, position_sensor, mission_config,
                             drone_config)) {
      return;
    }
  }
}

void runLevelMapping(IDrone &drone, IMap3D &map,
                     IPositionSensor &position_sensor,
                     const MissionConfig &mission_config,
                     const DroneConfig &drone_config) {
  const LevelGeometry lg = computeLevelGeometry(mission_config, drone_config);
  const vector<double> height_levels = sweepHeightLevels(lg);
  for (const double z : height_levels) {
    std::cout << "Running U-turn level sweep at height: " << z << std::endl;
    runUTurnLevelSweep(drone, map, position_sensor, mission_config,
                       drone_config, z);
  }
}

void setInitialPosition(TrueMap &true_map, SimulationState &sim_state) {
  const std::optional<Position3D> start =
      true_map.firstUnoccupiedPositionLexOrder();
  if (!start) {
    throw std::runtime_error(
        "No unoccupied spawn cell inside mission boundaries");
  }
  sim_state.drone_position = *start;
  sim_state.drone_orientation = {0.0 * deg, 0.0 * deg};
}

void seedDroneMapAtStart(IMap3D &map, const Position3D &start) {
  if (map.isInsideBounds(start)) {
    map.set(start, EMPTY);
  }
}

} // namespace

Simulator::Simulator(IDrone &drone, TrueMap &true_map, IMap3D &map,
                     ILidarSensor &lidar_sensor,
                     IPositionSensor &position_sensor,
                     IMovementDriver &movement_driver,
                     SimulationState &simulation_state,
                     const MissionConfig &mission_config,
                     const DroneConfig &drone_config)
    : drone(&drone), true_map(true_map), map(&map), lidar_sensor(&lidar_sensor),
      position_sensor(&position_sensor), movement_driver(&movement_driver),
      simulation_state_(simulation_state), mission_config_(mission_config),
      drone_config_(drone_config) {}

IMap3D &Simulator::simulate() {
  setInitialPosition(true_map, simulation_state_);
  seedDroneMapAtStart(*map, simulation_state_.drone_position);
  scanSurvey(*drone, *map, *position_sensor, mission_config_,
             drone_config_.lidarConfig);

  runLevelMapping(*drone, *map, *position_sensor, mission_config_,
                  drone_config_);

  return *map;
}
