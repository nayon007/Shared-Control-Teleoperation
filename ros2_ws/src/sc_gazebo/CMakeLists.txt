cmake_minimum_required(VERSION 3.8)
project(sc_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(sc_weld_plugin SHARED src/weld_plugin.cpp)
target_compile_features(sc_weld_plugin PUBLIC cxx_std_17)
ament_target_dependencies(sc_weld_plugin
  gazebo_dev gazebo_ros rclcpp std_msgs std_srvs geometry_msgs)

install(TARGETS sc_weld_plugin LIBRARY DESTINATION lib)
install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_startup.py scripts/weld_check.py scripts/reachability.py
  scripts/baseline_direct.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_start_pose test/test_start_pose.py TIMEOUT 300)
endif()

ament_package()
