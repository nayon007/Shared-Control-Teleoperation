cmake_minimum_required(VERSION 3.8)
project(sc_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config rviz models
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/go_to_start.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py TIMEOUT 120)
endif()

ament_package()
