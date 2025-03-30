cmake_minimum_required(VERSION 3.5)
project(integratedPackage)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlInstructions.msg"
)

add_executable(receiever src/coordinatesreceiver.cpp)
ament_target_dependencies(receiever 
  rclcpp 
  geometry_msgs
  rosidl_typesupport_cpp
)

install(TARGETS
  receiever
  DESTINATION lib/${PROJECT_NAME})

ament_package()