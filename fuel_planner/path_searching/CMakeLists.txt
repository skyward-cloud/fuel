cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  plan_env
)

add_library(path_searching SHARED
  src/kinodynamic_astar.cpp
  # src/astar.cpp
  src/astar2.cpp
  src/topo_prm.cpp
)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(path_searching
  ${PCL_LIBRARIES}
)
ament_target_dependencies(path_searching ${dependencies})

install(TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
