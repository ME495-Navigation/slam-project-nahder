cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

ament_target_dependencies(turtle_control rclcpp  std_msgs geometry_msgs sensor_msgs nuturtlebot_msgs turtlelib)
ament_target_dependencies(odometry rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)
ament_target_dependencies(circle rclcpp geometry_msgs std_srvs)

target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")

install(TARGETS turtle_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)

# integration testing
find_package(catch_ros2 REQUIRED)
add_executable(turtle_control_test test/turtle_control_test.cpp)
# add_executable(turtle_odom_test test/turtle_odom_test_node.cpp)
# add_executable(turtle_circle_test test/turtle_circle_test.cpp)

target_link_libraries(turtle_control_test catch_ros2::catch_ros2_with_node_main)
# target_link_libraries(turtle_odom_test catch_ros2::catch_ros2_with_node_main "${cpp_typesupport_target}")
# target_link_libraries(turtle_circle_test catch_ros2::catch_ros2_with_node_main "${cpp_typesupport_target}")

ament_target_dependencies(
  turtle_control_test
  rclcpp
  nuturtlebot_msgs
  geometry_msgs
  catch_ros2
  sensor_msgs
)

# ament_target_dependencies(
#   turtle_odom_test
#   rclcpp
#   tf2
#   tf2_ros
# )
# ament_target_dependencies(
#   turtle_circle_test
#   rclcpp
#   geometry_msgs
#   catch_ros2
# )

install(TARGETS turtle_control_test DESTINATION lib/${PROJECT_NAME})
# install(TARGETS turtle_odom_test DESTINATION lib/${PROJECT_NAME})
# install(TARGETS turtle_circle_test DESTINATION lib/${PROJECT_NAME})

install(FILES
  test/turtle_control_test.launch.py
  # test/turtle_odom_test.launch.xml
  # test/turtle_circle_test.launch.xml
  DESTINATION share/${PROJECT_NAME}
)

catch_ros2_add_integration_test(turtle_control_test
LAUNCH_FILE turtle_control_test.launch.py
)
# catch_ros2_add_integration_test(turtle_odom_test
# LAUNCH_FILE turtle_odom_test.launch.xml
# )
# catch_ros2_add_integration_test(turtle_circle_test
# LAUNCH_FILE turtle_circle_test.launch.xml
# )
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()