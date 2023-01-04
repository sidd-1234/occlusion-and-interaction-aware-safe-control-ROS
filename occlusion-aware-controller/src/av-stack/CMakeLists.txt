cmake_minimum_required(VERSION 3.5)
project(av-stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -c -MMD -pipe -std=gnu++17 -g -flto -fPIC -fno-math-errno -O3 -march=armv8-a -mcpu=cortex-a57)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jlb_pid)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

include_directories(/usr/local/include/eigen3/)

include_directories(/usr/local/include/libInterpolate/)

add_executable(joy_code src/joystick_teleop_control_test.cpp)
ament_target_dependencies(joy_code rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs nav2_msgs)

add_executable(joy_map src/joy_save_map.cpp)
ament_target_dependencies(joy_map rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs nav2_msgs slam_toolbox)

add_executable(interrupt src/interrupt_test.cpp)
ament_target_dependencies(interrupt rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs nav2_msgs Eigen3)

add_executable(stanley src/stanley_controller.cpp)
ament_target_dependencies(stanley rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs Eigen3)

add_executable(safecontrol src/occlusion_safe_controller.cpp)
ament_target_dependencies(safecontrol rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs Eigen3)

add_executable(tf_publish src/tf_publisher.cpp)
ament_target_dependencies(tf_publish rclcpp nav_msgs tf2_ros geometry_msgs)

add_executable(tf_publish_slam src/tf_publisher_mapping.cpp)
ament_target_dependencies(tf_publish_slam rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS
  joy_code
  tf_publish
  tf_publish_slam
  stanley
  safecontrol
  interrupt
  DESTINATION lib/${PROJECT_NAME})
  
 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
