cmake_minimum_required(VERSION 3.5)
project(main_strategy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D USE_ROS)
add_definitions(-D USE_IOSTREAM)
add_definitions(-D STANDALONE_STRATEGIE)


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(krabi_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(goal_strategy REQUIRED)
#find_package(lidar_strategy REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)


set(PLOT_NEURONS FALSE)

if(PLOT_NEURONS)
include_directories(include "/usr/include/python3.8")
include_directories(include "/usr/include/python3.6")
add_definitions(-DPLOT_NEURONS)
endif()

add_executable(${PROJECT_NAME}_node
  src/main_strategy_node.cpp 
  src/helpers_broker.c 
  src/core.cpp)

add_executable(odometry_node 
  src/odometry_node.cpp 
  src/odometry/odomTFPublisher.cpp)

ament_target_dependencies(${PROJECT_NAME}_node std_msgs geometry_msgs tf2 sensor_msgs krabi_msgs nav_msgs goal_strategy
 #lidar_strategy
 tf2_eigen
tf2_ros
tf2_geometry_msgs
  rclcpp)

ament_target_dependencies(odometry_node std_msgs geometry_msgs tf2 sensor_msgs krabi_msgs nav_msgs goal_strategy
# lidar_strategy
 rclcpp)


if(PLOT_NEURONS)
ament_target_dependencies(${PROJECT_NAME}_node -lpython3.8)
ament_target_dependencies(${PROJECT_NAME}_node -lpython3.6)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)
ament_package()