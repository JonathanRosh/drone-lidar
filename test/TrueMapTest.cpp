#include "../include/TrueMap.h"
#include "../include/TrueMapBuilder.h"

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

    TrueMapBuilder::set(tm ,pos, OCCUPIED);

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

    TrueMapBuilder::set(tm, xy1, OCCUPIED);
    TrueMapBuilder::set(tm, xy2, EMPTY);

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

    TrueMapBuilder::set(tm, h1, OCCUPIED);
    TrueMapBuilder::set(tm, h2, EMPTY);

    EXPECT_EQ(tm.get(h1), OCCUPIED);
    EXPECT_EQ(tm.get(h2), EMPTY);
}

TEST(TrueMapTest, FirstUnoccupiedLexOrderFreshMap)
{
    TrueMap fresh(mission_config);
    const auto p = fresh.firstUnoccupiedPositionLexOrder();
    ASSERT_TRUE(p.has_value());
    EXPECT_NEAR(p->x.numerical_value_in(cm), 10.0, 1e-6);
    EXPECT_NEAR(p->y.numerical_value_in(cm), 10.0, 1e-6);
    EXPECT_NEAR(p->z.numerical_value_in(cm), 10.0, 1e-6);
}

TEST(TrueMapTest, FirstUnoccupiedLexOrderSkipsOccupiedCorner)
{
    TrueMap fresh(mission_config);
    const Position3D corner{10.0 * cm, 10.0 * cm, 10.0 * cm};
    TrueMapBuilder::set(fresh, corner, OCCUPIED);

    const auto p = fresh.firstUnoccupiedPositionLexOrder();
    ASSERT_TRUE(p.has_value());
    EXPECT_NE(fresh.get(*p), OCCUPIED);
    // Next cell in z at same (x,y) for this resolution
    EXPECT_GT(p->z.numerical_value_in(cm), corner.z.numerical_value_in(cm));
}