cmake_minimum_required(VERSION 3.8)
project(lidar_angle_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_angle_filter_node src/lidar_angle_filter_node.cpp)
target_compile_features(lidar_angle_filter_node PUBLIC cxx_std_17)
target_include_directories(lidar_angle_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lidar_angle_filter_node rclcpp sensor_msgs)

install(TARGETS lidar_angle_filter_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_angle_filter test/test_angle_filter.cpp)
  target_compile_features(test_angle_filter PUBLIC cxx_std_17)
  target_include_directories(test_angle_filter PRIVATE include)
endif()

ament_package()
