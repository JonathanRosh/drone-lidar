#pragma once

#include "Configs.h"
#include "GridCoord.h"
#include "LidarScanResult.h"
#include "Units.h"

#include <cstddef>
#include <iosfwd>
#include <string>

class SimulatorLogger {
  std::ostream *output_;

public:
  explicit SimulatorLogger(std::ostream *output = nullptr);

  bool enabled() const;
  void writeLine(const char *line) const;
  void simulationStart(Distance res_xy,
                       Distance res_z,
                       const DroneConfig &drone_config) const;
  void state(const char *label,
             const Position3D &position,
             const Orientation &orientation) const;
  void iteration(std::size_t iteration) const;
  void scanSummary(Angle xy,
                   Angle altitude,
                   std::size_t beams,
                   std::size_t detected) const;
  void lidarHit(const LidarHit &hit) const;
  void frontierRejected(const GridCoord &grid) const;
  void frontierPath(std::size_t length,
                    std::size_t visited,
                    const GridCoord &target) const;
  void frontierStopped(std::size_t visited) const;
  void followPath(std::size_t path_size) const;
  void moveStep(const GridCoord &from, const GridCoord &to) const;
  void rotateCommand(bool rotate_left,
                     double command_deg,
                     double from_deg,
                     double target_deg) const;
  void elevateCommand(double distance_cm) const;
  void advanceCommand(double distance_cm) const;
  void failure(const char *context, const std::string &reason) const;
  void stopFailed(const std::string &reason) const;
};
