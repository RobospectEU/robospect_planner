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
  robotnik_pp_msgs
  ackermann_msgs
  interactive_markers
  visualization_msgs
  std_srvs
  self_test
  actionlib
)

find_package(PCL REQUIRED)

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
    nav_msgs
    std_msgs 
    geometry_msgs
    robotnik_pp_msgs 
    ackermann_msgs
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
add_dependencies(robospect_planner_node roscpp sensor_msgs robotnik_pp_msgs_generate_messages_cpp)
target_link_libraries(robospect_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} tunnel_planner)

