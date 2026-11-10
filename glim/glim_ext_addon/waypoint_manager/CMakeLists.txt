cmake_minimum_required(VERSION 3.22)
project(waypoint_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glim REQUIRED)
find_package(glim_ros REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_interfaces REQUIRED)

add_library(waypoint_manager SHARED
  src/glim_ext/waypoint_manager_module_ros2.cpp
)

target_include_directories(waypoint_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${glim_ros_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

ament_target_dependencies(waypoint_manager
  rclcpp
  visualization_msgs
  geometry_msgs
  waypoint_interfaces
)

target_link_libraries(waypoint_manager
  glim::glim
  ${glim_ros_LIBRARIES}
  ${GTSAM_LIBRARIES}
  Eigen3::Eigen
  spdlog::spdlog
  yaml-cpp
)

install(TARGETS waypoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_manager test/test_waypoint_manager.cpp)
  target_include_directories(test_waypoint_manager PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${glim_INCLUDE_DIRS}
    ${glim_ros_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
  )
  ament_target_dependencies(test_waypoint_manager
    rclcpp
    visualization_msgs
    geometry_msgs
    waypoint_interfaces
  )
  target_link_libraries(test_waypoint_manager
    waypoint_manager
    glim::glim
    ${glim_ros_LIBRARIES}
    ${GTSAM_LIBRARIES}
    Eigen3::Eigen
    spdlog::spdlog
    yaml-cpp
  )
endif()

ament_package()
