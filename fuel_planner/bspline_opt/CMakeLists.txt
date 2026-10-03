cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# Prefer CMake config / pkg-config over hardcoded NLopt paths
list(APPEND CMAKE_PREFIX_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/nlopt"
  "$ENV{HOME}/app/ws_fule/third_party/nlopt"
)
find_package(NLopt QUIET)
if(NLopt_FOUND)
  if(TARGET NLopt::nlopt)
    set(NLOPT_TARGET NLopt::nlopt)
  elseif(TARGET nlopt)
    set(NLOPT_TARGET nlopt)
  endif()
endif()
if(NOT NLOPT_TARGET)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    set(ENV{PKG_CONFIG_PATH} "$ENV{HOME}/app/ws_fule/third_party/nlopt/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    pkg_check_modules(NLOPT QUIET nlopt)
  endif()
  if(NOT NLOPT_FOUND)
    find_path(NLOPT_INCLUDE_DIRS NAMES nlopt.hpp nlopt.h
      PATHS
        /usr/include
        /usr/local/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/nlopt/include
        $ENV{HOME}/app/ws_fule/third_party/nlopt/include
    )
    find_library(NLOPT_LIBRARIES NAMES nlopt
      PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/nlopt/lib
        $ENV{HOME}/app/ws_fule/third_party/nlopt/lib
    )
    if(NLOPT_INCLUDE_DIRS AND NLOPT_LIBRARIES)
      set(NLOPT_FOUND TRUE)
    endif()
  endif()
  if(NOT NLOPT_FOUND AND NOT NLOPT_TARGET)
    message(FATAL_ERROR "NLopt not found. Install libnlopt-cxx-dev or build third_party/nlopt.")
  endif()
endif()

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  plan_env
  active_perception
)

add_library(bspline_opt SHARED
  src/bspline_optimizer.cpp
)
target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
if(NLOPT_TARGET)
  target_link_libraries(bspline_opt ${NLOPT_TARGET})
  get_target_property(_nlopt_loc ${NLOPT_TARGET} IMPORTED_LOCATION)
  if(NOT _nlopt_loc)
    get_target_property(_nlopt_loc ${NLOPT_TARGET} IMPORTED_LOCATION_RELEASE)
  endif()
  if(_nlopt_loc)
    get_filename_component(NLOPT_LIB_DIR "${_nlopt_loc}" DIRECTORY)
  endif()
else()
  target_include_directories(bspline_opt PUBLIC ${NLOPT_INCLUDE_DIRS})
  target_link_libraries(bspline_opt ${NLOPT_LIBRARIES})
  get_filename_component(NLOPT_LIB_DIR "${NLOPT_LIBRARIES}" DIRECTORY)
endif()
if(NLOPT_LIB_DIR)
  set_target_properties(bspline_opt PROPERTIES
    BUILD_RPATH "${NLOPT_LIB_DIR}"
    INSTALL_RPATH "${NLOPT_LIB_DIR}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
endif()
ament_target_dependencies(bspline_opt ${dependencies})

# Environment hook so dependents find private NLopt install at runtime
if(NLOPT_LIB_DIR)
  set(NLOPT_LIB_DIR_HOOK "${NLOPT_LIB_DIR}")
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/nlopt.sh.in")
endif()

install(TARGETS bspline_opt
  EXPORT export_bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_bspline_opt HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
