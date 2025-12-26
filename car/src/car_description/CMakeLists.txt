cmake_minimum_required(VERSION 3.5)
project(car_description)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY urdf launch config worlds car_world meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS teleop/simple_car_teleop.py
  PROGRAMS teleop/car_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
