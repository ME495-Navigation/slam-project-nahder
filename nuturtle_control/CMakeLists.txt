cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${rclcpp_INCLUDE_DIRS})

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(catch_ros2 REQUIRED)

add_executable(turtle_control src/turtle_control.cpp)
add_executable(turtle_control_test test/turtle_control_test.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

ament_target_dependencies(turtle_control rclcpp  std_msgs geometry_msgs sensor_msgs nuturtlebot_msgs turtlelib)
ament_target_dependencies(turtle_control_test rclcpp  std_msgs geometry_msgs sensor_msgs nuturtlebot_msgs turtlelib)
ament_target_dependencies(odometry rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)
ament_target_dependencies(circle rclcpp geometry_msgs std_srvs)

target_include_directories(turtle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(turtle_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS turtle_control turtle_control_test odometry circle
  DESTINATION lib/${PROJECT_NAME})


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

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(turtle_control_test catch_ros2::catch_ros2_with_node_main turtlelib::turtlelib "${cpp_typesupport_target}")

target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle "${cpp_typesupport_target}")

install(DIRECTORY
  srv
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()