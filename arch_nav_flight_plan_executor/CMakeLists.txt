cmake_minimum_required(VERSION 3.8)
project(arch_nav_flight_plan_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arch_nav CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arch_nav_ros2_interfaces REQUIRED)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

add_executable(arch_nav_flight_plan_executor
  src/flight_plan_parser.cpp
  src/flight_plan_executor.cpp
  src/flight_plan_executor_node.cpp
  src/main.cpp
)

target_include_directories(arch_nav_flight_plan_executor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(arch_nav_flight_plan_executor
  rclcpp
  rclcpp_action
  arch_nav_ros2_interfaces
)

target_link_libraries(arch_nav_flight_plan_executor
  arch_nav::arch_nav
  nlohmann_json::nlohmann_json
)

install(TARGETS arch_nav_flight_plan_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

ament_package()
