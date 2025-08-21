cmake_minimum_required(VERSION 3.0.2)
project(imu_algorithm_visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES imu_algorithm_visualization
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_visualizer_node
  src/imu_visualizer_node.cpp
  src/imu_visualizer.cpp
)
target_link_libraries(imu_visualizer_node
  ${catkin_LIBRARIES}
)

add_executable(dummy_imu_publisher
  src/dummy_imu_publisher_node.cpp
  src/dummy_imu_publisher.cpp
)
target_link_libraries(dummy_imu_publisher
  ${catkin_LIBRARIES}
)

add_executable(ground_truth_visualizer_node
  src/ground_truth_visualizer_node.cpp
  src/ground_truth_visualizer.cpp
)
target_link_libraries(ground_truth_visualizer_node
  ${catkin_LIBRARIES}
)

add_executable(gravity_measurement_visualizer_node
  src/gravity_measurement_visualizer_node.cpp
  src/gravity_measurement_visualizer.cpp
)
target_link_libraries(gravity_measurement_visualizer_node
  ${catkin_LIBRARIES}
)