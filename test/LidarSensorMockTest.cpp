#include "../include/LidarSensorMock.h"
#include "../include/LidarSensorMockTester.h"

#include <gtest/gtest.h>

class FakeMap : public IMap3D {
public:
    std::function<Mapping(const Position3D&)> handler;

    Mapping get(const Position3D& pos) const override {
        return handler(pos);
    }
};

class FakePositionSensor : public IPositionSensor {
public:
    Position3D position;
    Orientation orientation;

    const Position3D& getPosition() const override {
        return position;
    }

    const Orientation& getOrientation() const override {
        return orientation;
    }
};

TEST(MockLidarSensorTest, TraceBeamReturnsNulloptWhenNoObstacle)
{
    FakeMap map;
    map.handler = [](const Position3D&) {
        return EMPTY;
    };

    FakePositionSensor sensor;
    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 10.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation beam{
        0.0 * deg,
        0.0 * deg
    };

    auto result = LidarSensorMockTester::traceBeam(lidar, beam);

    EXPECT_FALSE(result.has_value());
}

TEST(MockLidarSensorTest, TraceBeamHitsObstacleAhead)
{
    FakeMap map;

    map.handler = [](const Position3D& pos) {

        if (pos.x >= 5.0 * x_extent[cm]) {
            return OCCUPIED;
        }

        return EMPTY;
    };

    FakePositionSensor sensor;
    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 20.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation beam{
        0.0 * deg,
        0.0 * deg
    };

    auto result = LidarSensorMockTester::traceBeam(lidar, beam);

    ASSERT_TRUE(result.has_value());

    EXPECT_NEAR(
        result->force_numerical_value_in(cm),
        5.0,
        0.11
    );
}

TEST(MockLidarSensorTest, TraceBeamReturnsZeroWhenObstacleTooClose)
{
    FakeMap map;

    map.handler = [](const Position3D& pos) {

        if (pos.x >= 2.0 * x_extent[cm]) {
            return OCCUPIED;
        }

        return EMPTY;
    };

    FakePositionSensor sensor;
    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 5.0 * cm,
        .zMax = 20.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation beam{
        0.0 * deg,
        0.0 * deg
    };

    auto result = LidarSensorMockTester::traceBeam(lidar, beam);

    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(
        result->force_numerical_value_in(cm),
        0.0
    );
}

TEST(MockLidarSensorTest, TraceBeamFollowsHorizontalAngle)
{
    FakeMap map;

    map.handler = [](const Position3D& pos) {

        if (pos.y >= 5.0 * y_extent[cm]) {
            return OCCUPIED;
        }

        return EMPTY;
    };

    FakePositionSensor sensor;
    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 20.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation beam{
        90.0 * deg,
        0.0 * deg
    };

    auto result = LidarSensorMockTester::traceBeam(lidar, beam);

    ASSERT_TRUE(result.has_value());

    EXPECT_NEAR(
        result->force_numerical_value_in(cm),
        5.0,
        0.11
    );
}

TEST(MockLidarSensorTest, GetScanReturnsCenterHit)
{
    FakeMap map;

    map.handler = [](const Position3D& pos) {

        if (pos.x >= 5.0 * x_extent[cm]) {
            return OCCUPIED;
        }

        return EMPTY;
    };

    FakePositionSensor sensor;
    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 20.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation scan_dir{
        0.0 * deg,
        0.0 * deg
    };

    auto results = lidar.getScan(scan_dir);

    ASSERT_EQ(results.size(), 1);

    EXPECT_NEAR(
        results[0].distance.force_numerical_value_in(cm),
        5.0,
        0.11
    );
}

TEST(MockLidarSensorTest, SensorHeadingRotatesBeam)
{
    FakeMap map;

    map.handler = [](const Position3D& pos) {

        if (pos.y >= 5.0 * y_extent[cm]) {
            return OCCUPIED;
        }

        return EMPTY;
    };

    FakePositionSensor sensor;

    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        90.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 20.0 * cm,
        .d = 1.0 * cm,
        .fovc = 1
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation relative_scan{
        0.0 * deg,
        0.0 * deg
    };

    auto results = lidar.getScan(relative_scan);

    ASSERT_EQ(results.size(), 1);

    EXPECT_NEAR(
        results[0].distance.force_numerical_value_in(cm),
        5.0,
        0.11
    );
}

TEST(MockLidarSensorTest, GetScanGeneratesAllCircleBeams)
{
    FakeMap map;

    // Every beam immediately hits
    map.handler = [](const Position3D&) {
        return OCCUPIED;
    };

    FakePositionSensor sensor;

    sensor.position = {
        0.0 * cm,
        0.0 * cm,
        0.0 * cm
    };

    sensor.orientation = {
        0.0 * deg,
        0.0 * deg
    };

    LidarConfig config{
        .zMin = 1.0 * cm,
        .zMax = 10.0 * cm,
        .d = 1.0 * cm,
        .fovc = 3
    };

    LidarSensorMock lidar(config, map, sensor);

    Orientation scan_direction{
        0.0 * deg,
        0.0 * deg
    };

    auto results = lidar.getScan(scan_direction);

    // Expected:
    // beam_0      -> 1
    // circle 1    -> 4
    // circle 2    -> 16
    // total       -> 21

    ASSERT_EQ(results.size(), 21);
}