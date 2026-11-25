cmake_minimum_required(VERSION 3.8)
project(sc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPS controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools geometry_msgs std_msgs kdl_parser orocos_kdl urdf)

add_library(sc_control SHARED src/cartesian_impedance_controller.cpp)
target_compile_features(sc_control PUBLIC cxx_std_17)
target_include_directories(sc_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(sc_control ${DEPS})
pluginlib_export_plugin_description_file(controller_interface sc_control_plugins.xml)

install(TARGETS sc_control LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_libraries(sc_control)
ament_export_dependencies(${DEPS})
ament_package()
