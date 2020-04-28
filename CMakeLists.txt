cmake_minimum_required(VERSION 3.5)
project(ros2_timer_latency_measurement)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rttest REQUIRED)
find_package(tlsf_cpp REQUIRED)
find_package(rcl REQUIRED)

include_directories(include ${rcl_INCLUDE_DIRS} )

add_executable(init_thread src/init_thread.cpp src/util.cpp)
ament_target_dependencies(init_thread rclcpp std_msgs rttest tlsf_cpp)
install(TARGETS init_thread DESTINATION lib/${PROJECT_NAME})

add_executable(init_and_node_thread src/init_and_node_thread.cpp src/util.cpp)
ament_target_dependencies(init_and_node_thread rclcpp rttest tlsf_cpp)
install(TARGETS init_and_node_thread DESTINATION lib/${PROJECT_NAME})

add_executable(timer_cb_latency_spin src/timer_cb_latency_spin.cpp src/util.cpp)
ament_target_dependencies(timer_cb_latency_spin rclcpp rttest tlsf_cpp)
install(TARGETS timer_cb_latency_spin DESTINATION lib/${PROJECT_NAME})

add_executable(timer_cb_latency_spin_some src/timer_cb_latency_spin_some.cpp src/util.cpp)
ament_target_dependencies(timer_cb_latency_spin_some rclcpp rttest tlsf_cpp)
install(TARGETS timer_cb_latency_spin_some DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
