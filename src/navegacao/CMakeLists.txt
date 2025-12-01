cmake_minimum_required(VERSION 3.8)
project(navegacao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(include)

add_executable(path_planner src/path_planner_node.cpp)
ament_target_dependencies(path_planner rclcpp cg_interfaces)

add_executable(mapper src/mapper_node.cpp)
ament_target_dependencies(mapper rclcpp cg_interfaces)

install(TARGETS
  path_planner
  mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

