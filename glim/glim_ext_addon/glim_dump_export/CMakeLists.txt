cmake_minimum_required(VERSION 3.22)
project(glim_dump_export)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(glim_dump_export src/glim_dump_export.cpp)
target_include_directories(glim_dump_export PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(glim_dump_export
  glim::glim
  spdlog::spdlog
  ${PCL_LIBRARIES}
)

install(TARGETS glim_dump_export DESTINATION lib/${PROJECT_NAME})

ament_package()
