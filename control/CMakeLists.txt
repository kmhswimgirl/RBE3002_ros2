cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY
  launch rviz src
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/lab2_controller.py # my individual lab 2 node
  src/controller.py # my lab 3 node
  src/cb_controller.py # reworked lab 2 node (for B25 RBE3002)
  src/lab2_topic_only.py # topic only verion of lab 2
  src/lab2_path.py # test path node for lab 2
  src/tf_controller.py # TF lookup bullshit implemented
  DESTINATION lib/control
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
