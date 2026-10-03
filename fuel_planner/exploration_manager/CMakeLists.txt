cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

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
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(plan_manage REQUIRED)
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
  visualization_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  plan_manage
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
)
target_include_directories(exploration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(exploration_node ${dependencies})
# Link library target only — lkh_tsp_solver export also contains executable lkh_tsp
target_link_libraries(exploration_node
  bspline::bspline_lib
  lkh_tsp_solver::lkh_tsp_solver
  ${PCL_LIBRARIES}
)

install(TARGETS exploration_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch resource
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
