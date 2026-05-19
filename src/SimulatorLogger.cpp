#include "SimulatorLogger.h"

#include "Utils.h"

#include <ostream>
#include <string>

SimulatorLogger::SimulatorLogger(std::ostream *output)
    : output_(output) {}

bool SimulatorLogger::enabled() const {
  return output_ != nullptr;
}

void SimulatorLogger::writeLine(const char *line) const {
  if (!enabled()) {
    return;
  }
  *output_ << line << '\n';
}

void SimulatorLogger::simulationStart(
    Distance res_xy,
    Distance res_z,
    const DroneConfig &drone_config) const {
  if (!enabled()) {
    return;
  }

  *output_ << "simulation start\n";
  *output_ << "resolution: xy_cm=" << Utils::cmValue(res_xy)
           << " z_cm=" << Utils::cmValue(res_z) << '\n';
  *output_ << "movement limits: max_advance_cm="
           << Utils::cmValue(drone_config.maxCommand.maxAdvance)
           << " max_elevate_cm="
           << Utils::cmValue(drone_config.maxCommand.maxElevate)
           << " max_rotate_deg="
           << drone_config.maxCommand.maxRotate.force_numerical_value_in(deg)
           << '\n';
  *output_ << "lidar: z_min_cm="
           << Utils::cmValue(drone_config.lidarConfig.zMin)
           << " z_max_cm="
           << Utils::cmValue(drone_config.lidarConfig.zMax)
           << " d_cm=" << Utils::cmValue(drone_config.lidarConfig.d)
           << " fovc=" << drone_config.lidarConfig.fovc << '\n';
}

void SimulatorLogger::state(const char *label,
                            const Position3D &position,
                            const Orientation &orientation) const {
  if (!enabled()) {
    return;
  }

  *output_ << label
           << ": position_cm=(" << position.x.force_numerical_value_in(cm)
           << ',' << position.y.force_numerical_value_in(cm)
           << ',' << position.z.force_numerical_value_in(cm)
           << ") orientation_deg=(" << Utils::degValue(orientation.horizontal)
           << ',' << Utils::degValue(orientation.altitude) << ")\n";
}

void SimulatorLogger::iteration(std::size_t iteration) const {
  if (!enabled()) {
    return;
  }
  *output_ << "iteration " << iteration << '\n';
}

void SimulatorLogger::scanSummary(Angle xy,
                                  Angle altitude,
                                  std::size_t beams,
                                  std::size_t detected) const {
  if (!enabled()) {
    return;
  }

  *output_ << "scan: relative_xy=" << xy.force_numerical_value_in(deg)
           << " relative_altitude=" << altitude.force_numerical_value_in(deg)
           << " beams=" << beams
           << " detected=" << detected
           << " missed=" << (beams - detected) << '\n';
}

void SimulatorLogger::lidarHit(const LidarHit &hit) const {
  if (!enabled()) {
    return;
  }

  *output_ << "lidar hit: distance_cm=" << Utils::cmValue(hit.distance)
           << " relative_xy_deg=" << Utils::degValue(hit.orientation.horizontal)
           << " relative_altitude_deg="
           << Utils::degValue(hit.orientation.altitude) << '\n';
}

void SimulatorLogger::frontierRejected(const GridCoord &grid) const {
  if (!enabled()) {
    return;
  }
  *output_ << "frontier rejected by clearance: grid=(" << grid.x << ','
           << grid.y << ',' << grid.z << ")\n";
}

void SimulatorLogger::frontierPath(std::size_t length,
                                   std::size_t visited,
                                   const GridCoord &target) const {
  if (!enabled()) {
    return;
  }
  *output_ << "frontier path found: length=" << length
           << " visited=" << visited
           << " target_grid=(" << target.x << ',' << target.y << ','
           << target.z << ")\n";
}

void SimulatorLogger::frontierStopped(std::size_t visited) const {
  if (!enabled()) {
    return;
  }
  *output_ << "frontier search stopped: no reachable frontier, visited="
           << visited << '\n';
}

void SimulatorLogger::followPath(std::size_t path_size) const {
  if (!enabled()) {
    return;
  }
  *output_ << "follow path: steps=" << (path_size == 0 ? 0 : path_size - 1)
           << '\n';
}

void SimulatorLogger::moveStep(const GridCoord &from,
                               const GridCoord &to) const {
  if (!enabled()) {
    return;
  }
  *output_ << "move step: from_grid=(" << from.x << ',' << from.y << ','
           << from.z << ") to_grid=(" << to.x << ',' << to.y << ','
           << to.z << ")\n";
}

void SimulatorLogger::rotateCommand(bool rotate_left,
                                    double command_deg,
                                    double from_deg,
                                    double target_deg) const {
  if (!enabled()) {
    return;
  }
  *output_ << "command: rotate_" << (rotate_left ? "left" : "right")
           << " angle_deg=" << command_deg
           << " from_deg=" << from_deg
           << " target_deg=" << target_deg << '\n';
}

void SimulatorLogger::elevateCommand(double distance_cm) const {
  if (!enabled()) {
    return;
  }
  *output_ << "command: elevate distance_cm=" << distance_cm << '\n';
}

void SimulatorLogger::advanceCommand(double distance_cm) const {
  if (!enabled()) {
    return;
  }
  *output_ << "command: advance distance_cm=" << distance_cm << '\n';
}

void SimulatorLogger::failure(const char *context,
                              const std::string &reason) const {
  if (!enabled() || reason.empty()) {
    return;
  }
  *output_ << "simulation failed during " << context << ": " << reason << '\n';
}

void SimulatorLogger::stopFailed(const std::string &reason) const {
  if (!enabled()) {
    return;
  }
  *output_ << "simulation stop: failed: " << reason << '\n';
}
