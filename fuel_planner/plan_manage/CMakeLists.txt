cmake_minimum_required(VERSION 3.8)
project(plan_manage)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_bridge_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# bspline_opt exports NLopt::nlopt on its INTERFACE; locate it for consumers
list(APPEND CMAKE_PREFIX_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/nlopt"
  "$ENV{HOME}/app/ws_fule/third_party/nlopt"
)
find_package(NLopt QUIET)
if(NOT NLopt_FOUND AND NOT TARGET NLopt::nlopt)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    set(ENV{PKG_CONFIG_PATH} "$ENV{HOME}/app/ws_fule/third_party/nlopt/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    pkg_check_modules(NLOPT QUIET nlopt)
  endif()
  if(NLOPT_FOUND AND NOT TARGET NLopt::nlopt)
    add_library(NLopt::nlopt UNKNOWN IMPORTED)
    set_target_properties(NLopt::nlopt PROPERTIES
      IMPORTED_LOCATION "${NLOPT_LINK_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${NLOPT_INCLUDE_DIRS}")
  endif()
endif()

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  quadrotor_msgs
  px4_bridge_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(plan_manage SHARED
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)
target_include_directories(plan_manage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(plan_manage ${dependencies})
target_link_libraries(plan_manage
  bspline::bspline_lib
  ${PCL_LIBRARIES}
)

add_executable(fast_planner_node
  src/fast_planner_node.cpp
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)
target_include_directories(fast_planner_node PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fast_planner_node ${dependencies})
target_link_libraries(fast_planner_node
  bspline::bspline_lib
  ${PCL_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(traj_server ${dependencies})
target_link_libraries(traj_server
  bspline::bspline_lib
  ${PCL_LIBRARIES}
)

install(TARGETS plan_manage
  EXPORT export_plan_manage
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fast_planner_node traj_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_plan_manage HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
