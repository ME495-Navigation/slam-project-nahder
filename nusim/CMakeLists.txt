cmake_minimum_required(VERSION 3.8)
project(nusim)

include_directories(${rclcpp_INCLUDE_DIRS})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(nusim src/nusim.cpp)
ament_target_dependencies(nusim turtlelib rclcpp std_msgs std_srvs tf2 tf2_ros)

install(TARGETS
  nusim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces 
  "srv/Teleport.srv" 
)
#TODO: cite https://robotics.stackexchange.com/questions/23171/creating-a-custom-msg-in-ros2
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces 
  "rosidl_typesupport_cpp"
)

target_link_libraries(nusim "${cpp_typesupport_target}")

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
