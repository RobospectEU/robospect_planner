cmake_minimum_required(VERSION 2.8.3)
project(robospect_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs 
  tf
  diagnostic_updater
  actionlib_msgs
  ackermann_msgs
  interactive_markers
  visualization_msgs
  std_srvs
  self_test
  actionlib
  message_generation
  robotnik_msgs
)

find_package(PCL REQUIRED)

add_message_files(
   FILES goal.msg 
   State.msg
)

add_action_files(DIRECTORY action FILES GoTo.action)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs ackermann_msgs robotnik_msgs)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(  
  INCLUDE_DIRS 
     include
     ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    roscpp 
    tf
    self_test
    diagnostic_updater
    sensor_msgs 
    std_srvs
    actionlib
    actionlib_msgs
    nav_msgs
    std_msgs 
    geometry_msgs
    ackermann_msgs
    message_runtime
   DEPENDS
    PCL
)

# Library definition
add_library(tunnel_planner
  src/tunnel_planner.cpp
  src/tunnel_map.cpp
  src/Path.cpp
  src/Component.cc
)

add_dependencies(tunnel_planner geometry_msgs_gencpp)
target_link_libraries(tunnel_planner
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(robospect_planner_node src/robospect_planner_node.cpp)
add_dependencies(robospect_planner_node robotnik_msgs_generate_messages_cpp)
add_dependencies(robospect_planner_node robospect_planner_generate_messages_cpp)
add_dependencies(tunnel_planner robospect_planner_generate_messages_cpp robotnik_msgs_generate_messages_cpp)
target_link_libraries(robospect_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} tunnel_planner)

##Install

install(PROGRAMS
   scripts/robospect_planner/path_marker.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

