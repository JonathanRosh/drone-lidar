cmake_minimum_required(VERSION 3.10)

project(drone-lidar)

find_package(mp-units CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

add_library(drone_core
    src/parser.cpp
    src/TrueMap.cpp
    src/LidarSensorMock.cpp
    src/TrueMapBuilder.cpp
    src/MapUtils.cpp
)

target_include_directories(
        drone_core
        PUBLIC include
)

target_link_libraries(drone_core PUBLIC mp-units::mp-units)

target_compile_features(drone_core PUBLIC cxx_std_20)

target_compile_options(drone_core PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_executable(drone_mapper
        src/main.cpp
)

target_compile_features(drone_mapper PRIVATE cxx_std_20)

target_compile_options(drone_mapper PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

target_link_libraries(drone_mapper PRIVATE drone_core)

enable_testing()

add_executable(
    trueMapTest
    test/TrueMapTest.cpp
)

target_link_libraries(trueMapTest
    PRIVATE drone_core
    PRIVATE GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(trueMapTest)