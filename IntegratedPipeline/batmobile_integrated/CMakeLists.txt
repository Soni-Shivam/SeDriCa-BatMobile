cmake_minimum_required(VERSION 3.5)
project(batmobile_integrated)
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

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlInstructions.msg"
  DEPENDENCIES geometry_msgs
)

# Get the include directories for the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add the receiver executable
add_executable(coordinates_receiver src/coordinatesreceiver.cpp)
ament_target_dependencies(coordinates_receiver 
  rclcpp 
  geometry_msgs
)

# Link against the generated messages
target_link_libraries(coordinates_receiver "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  coordinates_receiver
  DESTINATION lib/${PROJECT_NAME})

# Install message interfaces
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  src/publisher_node.py
  DESTINATION lib/${PROJECT_NAME}/src
)

ament_package()