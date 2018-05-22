################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(heroehs_online_walking_pattern_generator)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  robotis_math
  scilab_optimization
)

find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES heroehs_online_walking_pattern_generator
  CATKIN_DEPENDS roscpp robotis_math scilab_optimization
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(heroehs_online_walking_pattern_generator
  src/online_pelvis_xy_calculator.cpp
  src/online_endpoint_calculator.cpp
  src/online_walking_pattern_generator.cpp
)

add_dependencies(heroehs_online_walking_pattern_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(heroehs_online_walking_pattern_generator
  ${catkin_LIBRARIES}
)