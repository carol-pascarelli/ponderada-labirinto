cmake_minimum_required(VERSION 3.8)
project(ponderada_navegacao)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(include)

add_executable(pathfinder src/pathfinder.cpp)
ament_target_dependencies(pathfinder rclcpp std_msgs cg_interfaces)

add_executable(mapper src/mapper.cpp)
ament_target_dependencies(mapper rclcpp std_msgs cg_interfaces)

install(TARGETS
  pathfinder
  mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
