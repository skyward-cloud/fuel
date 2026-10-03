cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# LKH defines globals in headers; allow multiple tentative definitions (pre-GCC10 behavior)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcommon")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcommon")

find_package(ament_cmake REQUIRED)

file(GLOB SRCS src/*.c)

add_library(lkh_tsp_solver SHARED
  ${SRCS}
  src/lkh_interface.cpp
)
target_include_directories(lkh_tsp_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lkh_tsp_solver m)

add_executable(lkh_tsp
  ${SRCS}
)
target_include_directories(lkh_tsp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lkh_tsp m)

install(TARGETS lkh_tsp_solver lkh_tsp
  EXPORT export_lkh_tsp_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_lkh_tsp_solver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
