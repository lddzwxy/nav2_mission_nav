cmake_minimum_required(VERSION 3.8)
project(mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mission_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
# yaml_cpp_vendor 只提供头文件路径与 yaml-cpp 的 CMake config，库要单独 find + link
find_package(yaml-cpp REQUIRED)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
  src/mission_state_machine.cpp
  src/waypoint_loader.cpp
)
target_include_directories(mission_manager_node PRIVATE include)
ament_target_dependencies(mission_manager_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs visualization_msgs
  std_msgs std_srvs mission_msgs yaml_cpp_vendor
)
# 不显式链接 yaml-cpp 会出现大量 undefined reference to YAML::...
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(mission_manager_node yaml-cpp::yaml-cpp)
else()
  target_link_libraries(mission_manager_node yaml-cpp)
endif()

install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
