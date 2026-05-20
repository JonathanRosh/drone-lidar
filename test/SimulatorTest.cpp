#include "../include/Simulator.h"
#include "../include/TrueMapBuilder.h"

#include <gtest/gtest.h>

namespace {

MissionConfig lineMissionConfig() {
  return MissionConfig{
      .map_boundry = {
          .minX = 0.0 * cm,
          .minY = 0.0 * cm,
          .maxX = 5.0 * cm,
          .maxY = 0.0 * cm,
          .maxHeight = 0.0 * cm,
          .minHeight = 0.0 * cm,
      },
      .map_resolution = {
          .xy_resolution = 0,
          .height_resolution = 0,
      },
  };
}

MissionConfig offsetStartMissionConfig() {
  return MissionConfig{
      .map_boundry = {
          .minX = 5.0 * cm,
          .minY = 0.0 * cm,
          .maxX = 9.0 * cm,
          .maxY = 0.0 * cm,
          .maxHeight = 0.0 * cm,
          .minHeight = 0.0 * cm,
      },
      .map_resolution = {
          .xy_resolution = 0,
          .height_resolution = 0,
      },
      .initial_pose = {
          .position = {6.0 * cm, 0.0 * cm, 0.0 * cm},
          .orientation = {0.0 * deg, 0.0 * deg},
      },
  };
}

DroneConfig testDroneConfig() {
  return DroneConfig{
      .minPass = {
          .width = 0.0 * cm,
          .height = 0.0 * cm,
          .length = 0.0 * cm,
      },
      .maxCommand = {
          .maxAdvance = 1.0 * cm,
          .maxElevate = 1.0 * cm,
          .maxRotate = 90.0 * deg,
      },
      .lidarConfig = {
          .zMin = 0.1 * cm,
          .zMax = 10.0 * cm,
          .d = 1.0 * cm,
          .fovc = 1,
      },
  };
}

} // namespace

TEST(SimulatorTest, SimulateMapsSimpleOccupiedVoxelAndFreeRayCells) {
  const MissionConfig mission_config = lineMissionConfig();
  TrueMap true_map(mission_config);
  TrueMapBuilder::set(true_map, {3.0 * cm, 0.0 * cm, 0.0 * cm}, OCCUPIED);

  Simulator simulator(testDroneConfig(), mission_config, true_map);
  IMap3D &mapped = simulator.simulate();

  EXPECT_EQ(mapped.get({0.0 * cm, 0.0 * cm, 0.0 * cm}), EMPTY);
  EXPECT_EQ(mapped.get({1.0 * cm, 0.0 * cm, 0.0 * cm}), EMPTY);
  EXPECT_EQ(mapped.get({2.0 * cm, 0.0 * cm, 0.0 * cm}), EMPTY);
  EXPECT_EQ(mapped.get({3.0 * cm, 0.0 * cm, 0.0 * cm}), OCCUPIED);
}

TEST(SimulatorTest, SimulateStartsAtMissionInitialPosition) {
  const MissionConfig mission_config = offsetStartMissionConfig();
  TrueMap true_map(mission_config);
  TrueMapBuilder::set(true_map, {8.0 * cm, 0.0 * cm, 0.0 * cm}, OCCUPIED);

  Simulator simulator(testDroneConfig(), mission_config, true_map);
  IMap3D &mapped = simulator.simulate();

  EXPECT_EQ(mapped.get({6.0 * cm, 0.0 * cm, 0.0 * cm}), EMPTY);
  EXPECT_EQ(mapped.get({7.0 * cm, 0.0 * cm, 0.0 * cm}), EMPTY);
  EXPECT_EQ(mapped.get({8.0 * cm, 0.0 * cm, 0.0 * cm}), OCCUPIED);
  EXPECT_EQ(mapped.get({0.0 * cm, 0.0 * cm, 0.0 * cm}), OUTSIDE_BOUNDARY);
}
