cmake_minimum_required(VERSION 2.8.3)
project(main_strategy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             krabi_msgs
             geometry_msgs
             tf
             tf2
             nav_msgs
             goal_strategy
             lidar_strategy)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  krabi_msgs
  goal_strategy
  lidar_strategy)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/main_strategy_node.cpp src/helpers_broker.c src/core.cpp)

add_executable(odometry_node src/odometry_node.cpp src/odometry/lightOdometry.cpp)


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(odometry_node ${catkin_LIBRARIES})

