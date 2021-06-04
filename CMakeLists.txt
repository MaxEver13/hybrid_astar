cmake_minimum_required(VERSION 3.5)
project(smac_planner)

set(CMAKE_BUILD_TYPE Release) #Debug, Release

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs 
  tf
  tf2
  tf2_ros
  tf2_eigen
  rospy
  dynamic_reconfigure
  costmap_2d
  nav_core
  grid_map_core
  grid_map_ros
  grid_map_costmap_2d
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs nav_msgs tf tf2 tf2_ros tf2_eigen rospy dynamic_reconfigure 
  costmap_2d nav_core grid_map_core grid_map_ros grid_map_costmap_2d
)

find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${OMPL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(library_name smac_planner_core)

# SE2 plugin
add_library(${library_name} SHARED
  src/smac_planner.cpp
  src/a_star.cpp
  src/node_se2.cpp
  src/costmap_downsampler.cpp
  src/footprint_collision_checker.cpp
)

target_link_libraries(${library_name} ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES}  OpenMP::OpenMP_CXX)
target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})



## Declare a cpp executable
add_executable(smac_planner 
  test/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(smac_planner 
  ${library_name}
  ${catkin_LIBRARIES}  
)
