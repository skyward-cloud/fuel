cmake_minimum_required(VERSION 3.8)
project(active_perception)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  plan_env
  bspline
  path_searching
)

add_library(active_perception SHARED
  src/traj_visibility.cpp
  src/heading_planner.cpp
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/perception_utils.cpp
)
target_include_directories(active_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(active_perception
  ${PCL_LIBRARIES}
  bspline::bspline_lib
)
ament_target_dependencies(active_perception ${dependencies})

install(TARGETS active_perception
  EXPORT export_active_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_active_perception HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
