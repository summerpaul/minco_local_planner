cmake_minimum_required(VERSION 3.0.2)
project(minco_local_planner)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RELEASE")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -O3 -march=native ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -O3 -fopenmp ")

else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -fsanitize=leak -g")
endif()



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES minco_local_planner
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
find_package(Eigen3)
include_directories(core
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

add_subdirectory(core)
add_subdirectory(app)
add_subdirectory(visualizer)
add_subdirectory(plugin)
