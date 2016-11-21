cmake_minimum_required(VERSION 2.8)
project(particle_filter)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -g -O3 -fopenmp")

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND LIBRARIES_LIST ${Boost_LIBRARIES} )

find_package(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBRARIES_LIST ${OpenCV_LIBRARIES} )

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(include)

add_library(particleFilter
	    src/test.cpp
            src/log.cpp
            src/map.cpp
            src/sensor_model.cpp
            src/motion_model.cpp
            src/robot_state.cpp
            src/particle_state.cpp
            src/util_functions.cpp
            src/test_raytrace.cpp
            src/sampler.cpp)

target_link_libraries(particleFilter ${LIBRARIES_LIST})

add_executable(runTest src/run_test.cpp)
add_executable(runTestSingle src/run_test_single_particle.cpp)
add_executable(testRayTrace src/test_raytrace.cpp)

target_link_libraries(runTest particleFilter)
target_link_libraries(testRayTrace particleFilter)
target_link_libraries(runTestSingle particleFilter)


	
