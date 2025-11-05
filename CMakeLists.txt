cmake_minimum_required(VERSION 3.5.1)

project(asv_helm)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(project11 REQUIRED)
find_package(project11_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} src/asv_helm_node.cpp)

ament_target_dependencies(${PROJECT_NAME} geometry_msgs nav_msgs project11 project11_msgs rclcpp)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
