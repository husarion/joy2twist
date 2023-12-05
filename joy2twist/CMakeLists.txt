cmake_minimum_required(VERSION 3.5)
project(joy2twist)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(GCC_COVERAGE_COMPILE_FLAGS "-pthread -DBOOST_ERROR_CODE_HEADER_ONLY -BOOST_SYSTEM_NO_DEPRECATED")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(joy_linux REQUIRED)
include_directories(include)

add_executable(
  joy2twist
  src/main.cpp
  src/joy2twist_node.cpp
)

ament_target_dependencies(joy2twist
  ament_cmake
  ament_index_cpp
  rclcpp
  geometry_msgs  
  sensor_msgs
  std_msgs
  std_srvs
  joy_linux
)

install(TARGETS joy2twist
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
