cmake_minimum_required(VERSION 2.8.3)
project(rviz_3d_view_controller)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  rviz
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz_3d_view_controller
#  CATKIN_DEPENDS eigen_conversions roscpp rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(rviz_3d_view_controller
  src/rviz_orbit3d_view_controller.cpp
)

add_dependencies(rviz_3d_view_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rviz_3d_view_controller
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

