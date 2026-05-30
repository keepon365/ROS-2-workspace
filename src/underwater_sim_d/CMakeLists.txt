cmake_minimum_required(VERSION 3.8)
project(underwater_sim_d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Thrust.msg"
  "msg/DVL.msg"
  "msg/IMU.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS
  scripts/sim_bridge.py
  scripts/control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
