cmake_minimum_required(VERSION 3.8)
project(bspline)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(bspline_lib SHARED src/non_uniform_bspline.cpp)
target_include_directories(bspline_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(bspline_lib rclcpp)
set_target_properties(bspline_lib PROPERTIES OUTPUT_NAME bspline)

install(TARGETS bspline_lib
  EXPORT export_bspline_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_bspline_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rosidl_default_runtime std_msgs geometry_msgs builtin_interfaces)

ament_package()
