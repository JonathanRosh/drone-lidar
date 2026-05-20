#include "../include/Parser.h"

#include <filesystem>
#include <gtest/gtest.h>
#include <string>

namespace {

std::filesystem::path inputPath(const std::string &filename) {
  return std::filesystem::path(__FILE__).parent_path() / "map_test_inputs" /
         filename;
}

MissionConfig testMissionConfig() {
  return MissionConfig{
      .map_boundry = {
          .minX = 0.0 * cm,
          .minY = 0.0 * cm,
          .maxX = 10.0 * cm,
          .maxY = 10.0 * cm,
          .maxHeight = 10.0 * cm,
          .minHeight = 0.0 * cm,
      },
      .map_resolution = {
          .xy_resolution = 0,
          .height_resolution = 0,
      },
  };
}

} // namespace

TEST(ParserTest, ParseDroneConfigValidFile) {
  const auto path = inputPath("drone_config_valid.txt");

  const DroneConfig config = Parser::parseDroneConfig(path.string());

  EXPECT_DOUBLE_EQ(config.minPass.width.force_numerical_value_in(cm), 30.0);
  EXPECT_DOUBLE_EQ(config.minPass.height.force_numerical_value_in(cm), 20.0);
  EXPECT_DOUBLE_EQ(config.minPass.length.force_numerical_value_in(cm), 25.0);
  EXPECT_DOUBLE_EQ(config.maxCommand.maxAdvance.force_numerical_value_in(cm),
                   100.0);
  EXPECT_DOUBLE_EQ(config.maxCommand.maxElevate.force_numerical_value_in(cm),
                   40.0);
  EXPECT_DOUBLE_EQ(config.maxCommand.maxRotate.force_numerical_value_in(deg),
                   90.0);
  EXPECT_DOUBLE_EQ(config.lidarConfig.zMin.force_numerical_value_in(cm), 5.0);
  EXPECT_DOUBLE_EQ(config.lidarConfig.zMax.force_numerical_value_in(cm), 120.0);
  EXPECT_DOUBLE_EQ(config.lidarConfig.d.force_numerical_value_in(cm), 2.5);
  EXPECT_EQ(config.lidarConfig.fovc, 3u);
}

TEST(ParserTest, ParseDroneConfigThrowsOnMissingFile) {
  EXPECT_THROW(Parser::parseDroneConfig("/missing/drone_config.txt"),
               std::runtime_error);
}

TEST(ParserTest, ParseDroneConfigThrowsOnUnknownKey) {
  const auto path = inputPath("drone_config_unknown_key.txt");

  EXPECT_THROW(Parser::parseDroneConfig(path.string()), std::runtime_error);
}

TEST(ParserTest, ParseDroneConfigThrowsOnMalformedLineWithoutColon) {
  const auto path = inputPath("drone_config_malformed.txt");

  EXPECT_THROW(Parser::parseDroneConfig(path.string()), std::runtime_error);
}

TEST(ParserTest, ParseMissionConfigValidFile) {
  const auto path = inputPath("mission_config_valid.txt");

  const MissionConfig config = Parser::parseMissionConfig(path.string());

  EXPECT_DOUBLE_EQ(config.map_boundry.minX.force_numerical_value_in(cm), 1.0);
  EXPECT_DOUBLE_EQ(config.map_boundry.minY.force_numerical_value_in(cm), 2.0);
  EXPECT_DOUBLE_EQ(config.map_boundry.maxX.force_numerical_value_in(cm), 30.0);
  EXPECT_DOUBLE_EQ(config.map_boundry.maxY.force_numerical_value_in(cm), 40.0);
  EXPECT_DOUBLE_EQ(config.map_boundry.minHeight.force_numerical_value_in(cm),
                   3.0);
  EXPECT_DOUBLE_EQ(config.map_boundry.maxHeight.force_numerical_value_in(cm),
                   50.0);
  EXPECT_EQ(config.map_resolution.xy_resolution, 2u);
  EXPECT_EQ(config.map_resolution.height_resolution, 3u);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.x.force_numerical_value_in(cm),
                   4.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.y.force_numerical_value_in(cm),
                   5.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.z.force_numerical_value_in(cm),
                   6.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.orientation.horizontal
                       .force_numerical_value_in(deg),
                   90.0);
}

TEST(ParserTest, ParseMissionConfigAcceptsResolutionAliases) {
  const auto path = inputPath("mission_config_resolution_aliases.txt");

  const MissionConfig config = Parser::parseMissionConfig(path.string());

  EXPECT_EQ(config.map_resolution.xy_resolution, 4u);
  EXPECT_EQ(config.map_resolution.height_resolution, 5u);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.x.force_numerical_value_in(cm),
                   0.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.y.force_numerical_value_in(cm),
                   0.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.position.z.force_numerical_value_in(cm),
                   0.0);
  EXPECT_DOUBLE_EQ(config.initial_pose.orientation.horizontal
                       .force_numerical_value_in(deg),
                   0.0);
}

TEST(ParserTest, ParseMissionConfigThrowsOnMissingFile) {
  EXPECT_THROW(Parser::parseMissionConfig("/missing/mission_config.txt"),
               std::runtime_error);
}

TEST(ParserTest, ParseMissionConfigThrowsOnUnknownKey) {
  const auto path = inputPath("mission_config_unknown_key.txt");

  EXPECT_THROW(Parser::parseMissionConfig(path.string()), std::runtime_error);
}

TEST(ParserTest, ParseMissionConfigThrowsOnMalformedLineWithoutColon) {
  const auto path = inputPath("mission_config_malformed.txt");

  EXPECT_THROW(Parser::parseMissionConfig(path.string()), std::runtime_error);
}

TEST(ParserTest, ParseTrueMapValidFile) {
  const MissionConfig mission_config = testMissionConfig();
  const auto path = inputPath("map_input_valid.txt");

  const TrueMap map = Parser::parseTrueMap(path.string(), mission_config);

  EXPECT_EQ(map.get({1.0 * cm, 2.0 * cm, 3.0 * cm}), OCCUPIED);
  EXPECT_EQ(map.get({4.0 * cm, 5.0 * cm, 6.0 * cm}), OCCUPIED);
  EXPECT_EQ(map.get({7.0 * cm, 8.0 * cm, 9.0 * cm}), EMPTY);
  EXPECT_EQ(map.get({11.0 * cm, 0.0 * cm, 0.0 * cm}), OUTSIDE_BOUNDARY);
}

TEST(ParserTest, ParseTrueMapThrowsOnMissingFile) {
  EXPECT_THROW(Parser::parseTrueMap("/missing/map_input.txt",
                                    testMissionConfig()),
               std::runtime_error);
}

TEST(ParserTest, ParseTrueMapThrowsOnMalformedLine) {
  const auto path = inputPath("map_input_malformed.txt");

  EXPECT_THROW(Parser::parseTrueMap(path.string(), testMissionConfig()),
               std::runtime_error);
}

TEST(ParserTest, ParseTrueMapThrowsOnPointOutsideBoundaries) {
  const auto path = inputPath("map_input_outside.txt");

  EXPECT_THROW(Parser::parseTrueMap(path.string(), testMissionConfig()),
               std::runtime_error);
}
