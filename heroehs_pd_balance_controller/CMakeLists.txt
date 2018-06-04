################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(heroehs_pd_balance_controller)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  robotis_math
  heroehs_math
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
  LIBRARIES heroehs_pd_balance_controller
  CATKIN_DEPENDS roscpp robotis_math heroehs_math
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(heroehs_pd_balance_controller
  src/heroehs_pd_balance_controller.cpp
)

add_dependencies(heroehs_pd_balance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(heroehs_pd_balance_controller
  ${catkin_LIBRARIES}
)
