cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch rviz 
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/control/path_generator.py
  src/control/tf_controller.py
  DESTINATION lib/control
)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

install(
  DIRECTORY src/control
  DESTINATION lib/python3.12/site-packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpklint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
