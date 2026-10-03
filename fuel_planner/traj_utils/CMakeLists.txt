cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  pcl_conversions
  bspline
  path_searching
  poly_traj
  plan_env
  active_perception
)

add_library(traj_utils SHARED
  src/planning_visualization.cpp
)
target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(traj_utils ${dependencies})
target_link_libraries(traj_utils
  bspline::bspline_lib
  ${PCL_LIBRARIES}
)

add_executable(process_msg
  src/process_msg.cpp
)
target_include_directories(process_msg PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(process_msg
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(process_msg
  ${PCL_LIBRARIES}
)

install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS process_msg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
