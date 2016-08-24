cmake_minimum_required(VERSION 2.8.11)
project(rviz_cloud_annotation)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  pcl_conversions
  roscpp
  tf
  visualization_msgs
  cmake_modules
  std_msgs
  rviz
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz_cloud_annotation
#  CATKIN_DEPENDS interactive_markers pcl_conversions roscpp tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(rviz_cloud_annotation
#   src/${PROJECT_NAME}/rviz_cloud_annotation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rviz_cloud_annotation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rviz_cloud_annotation_node
  src/rviz_cloud_annotation.cpp
  src/rviz_cloud_annotation_points.cpp
  )

add_dependencies(rviz_cloud_annotation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rviz_cloud_annotation_node
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES}
  )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(rviz_cloud_annotation_plugin
  src/rviz_cloud_annotation_plugin.cpp
)

add_dependencies(rviz_cloud_annotation_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rviz_cloud_annotation_plugin
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

#############
## Install ##
#############

