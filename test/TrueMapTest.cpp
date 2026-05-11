#include "../include/TrueMap.h"

#include <gtest/gtest.h>

Distance minX = 10.0 * cm; 
Distance minY = 10.0 * cm;
Distance maxX = 20.0 * cm; 
Distance maxY = 20.0 * cm;
Distance maxHeight = 20.0 * cm;
Distance minHeight = 10.0 * cm;

const MapBoundry map_boundry = {
    minX,
    minY,
    maxX,
    maxY,
    maxHeight,
    minHeight,
};

const MapResolution map_resolution = {
    2,
    3,
};

const MissionConfig mission_config = {
    map_boundry,
    map_resolution
};

TrueMap tm = TrueMap(mission_config);

TEST(TrueMapTest, OutsideBoundaryReturnsExpectedValue)
{
    Position3D outside_boundary = {
        21.0 * cm,
        10.0 * cm,
        10.0 * cm
    };

    Mapping result = tm.get(outside_boundary);

    EXPECT_EQ(result, OUTSIDE_BOUNDARY);
}

TEST(TrueMapTest, NotMappedCell)
{
  Position3D pos = { 
        15.0 * cm,            
        15.0 * cm,
        15.0 * cm
    };

    Mapping non_mapped = tm.get(pos);
}

TEST(TrueMapTest, SetAndGetCell)
{
    Position3D pos = {
        15.0 * cm,
        15.0 * cm,
        15.0 * cm
    };

    tm.set(pos, OCCUPIED);

    EXPECT_EQ(tm.get(pos), OCCUPIED);
}

TEST(TrueMapTest, XYResolutionDistinguishesCells)
{
    Position3D xy1 = {
        15.01 * cm,
        15.0 * cm,
        15.0 * cm
    };

    Position3D xy2 = {
        15.02 * cm,
        15.0 * cm,
        15.0 * cm
    };

    tm.set(xy1, OCCUPIED);
    tm.set(xy2, EMPTY);

    EXPECT_EQ(tm.get(xy1), OCCUPIED);
    EXPECT_EQ(tm.get(xy2), EMPTY);
}

TEST(TrueMapTest, HeightResolutionDistinguishesCells)
{
    Position3D h1 = {
        15.0 * cm,
        15.0 * cm,
        16.001 * cm
    };

    Position3D h2 = {
        15.0 * cm,
        15.0 * cm,
        16.002 * cm
    };

    tm.set(h1, OCCUPIED);
    tm.set(h2, EMPTY);

    EXPECT_EQ(tm.get(h1), OCCUPIED);
    EXPECT_EQ(tm.get(h2), EMPTY);
}