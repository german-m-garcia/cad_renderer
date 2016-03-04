cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cad_renderer)

# Find Catkin
find_package(catkin REQUIRED)
# Declare this project as a catkin package
catkin_package()
 
find_package(PCL 1.7 REQUIRED)
SET(CMAKE_CXX_FLAGS "-O4 -std=c++11 -g -Wall -fopenmp")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_snapshot_creator_rgb_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(snapshotCreatorRGB ${PCL_snapshot_creator_rgb_SRC})
 
target_link_libraries(snapshotCreatorRGB ${PCL_LIBRARIES})
