cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cad_renderer)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
#Include Required Directories
include_directories("${PROJECT_INCLUDE_DIR}")

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
 
add_library(RENDERER SHARED

    ${PROJECT_SOURCE_DIR}/render_views_tesselated_sphere_rgb.cpp
    ${PROJECT_SOURCE_DIR}/Renderer.cpp
)

add_executable(snapshotCreatorRGB ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(snapshotCreatorRGB ${PCL_LIBRARIES} RENDERER)

