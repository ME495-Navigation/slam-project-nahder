cmake_minimum_required(VERSION 3.8)
include_directories(${rclcpp_INCLUDE_DIRS})

project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlelib REQUIRED)

# add_library(nusim_lib SHARED src/nusim.cpp)
add_executable(nusim src/nusim.cpp)
ament_target_dependencies(nusim turtlelib rclcpp std_msgs std_srvs)

install(TARGETS
  nusim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
