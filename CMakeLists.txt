cmake_minimum_required(VERSION 2.8.3)
project(main_strategy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  krabi_msgs
  geometry_msgs
  tf
  nav_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf krabi_msgs 
)

include_directories(
 include
 ../lidar_strategy/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_strategy_node 
src/main.cpp
../lidar_strategy/src/lidarStrat.cpp
src/helpers_broker.c
src/Krabi/position.cpp
src/Krabi/positionPlusAngle.cpp
src/Krabi/distance.cpp
src/Krabi/vec2d.cpp
src/Krabi/vec3d.cpp
src/Krabi/angle.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)