cmake_minimum_required(VERSION 3.0.2)
project(imu_translate_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES imu_translate_node
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs tf
)

include_directories(
 # include
  ${catkin_INCLUDE_DIRS}
)

add_library(pid src/pid.cpp)

add_executable(imu_translate_node src/imu_translate_node.cpp)
add_executable(heading_lock_node src/heading_lock_node.cpp)
add_executable(demo_yaw_publisher_node src/demo_yaw_publisher_node.cpp)
add_executable(rotate_90_degrees_node src/rotate_90_degrees_node.cpp src/pid.cpp)

target_link_libraries(imu_translate_node pid ${catkin_LIBRARIES})
target_link_libraries(heading_lock_node pid ${catkin_LIBRARIES})
target_link_libraries(demo_yaw_publisher_node ${catkin_LIBRARIES})
target_link_libraries(rotate_90_degrees_node ${catkin_LIBRARIES})
install(TARGETS imu_translate_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS heading_lock_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS demo_yaw_publisher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS rotate_90_degrees_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
