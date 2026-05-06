cmake_minimum_required(VERSION 3.10)

project(drone-lidar)

find_package(mp-units CONFIG REQUIRED)

add_executable(drone_mapper
        src/main.cpp
        src/parser.cpp
)

target_include_directories(drone_mapper
        PRIVATE ${CMAKE_SOURCE_DIR}/include
)

target_compile_features(drone_mapper PRIVATE cxx_std_20)

target_compile_options(drone_mapper PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

target_link_libraries(drone_mapper PUBLIC mp-units::mp-units)