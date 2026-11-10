cmake_minimum_required(VERSION 3.8)
project(kratos_traversability)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(traversability_layer SHARED src/traversability_layer.cpp)
target_compile_features(traversability_layer PUBLIC cxx_std_17)
target_link_libraries(traversability_layer PUBLIC
  rclcpp::rclcpp nav2_costmap_2d::nav2_costmap_2d_core pluginlib::pluginlib tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs tf2_ros::tf2_ros
  ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})
pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer.xml)

install(TARGETS traversability_layer
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(PROGRAMS scripts/traversability_mapper.py DESTINATION lib/${PROJECT_NAME})

ament_package()
