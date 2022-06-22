cmake_minimum_required(VERSION 3.8)
project(omni_move_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(ROS_LIBS
  rclcpp;
  rclcpp_components;
  geometry_msgs;
  nav_msgs;
  std_msgs;
)
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# ros2 packages required
foreach(lib IN LISTS ROS_LIBS)
  find_package(${lib} REQUIRED)
endforeach()

# ros2 components
add_library(${PROJECT_NAME} SHARED
  src/omni3wd_driver.cpp
  src/omniVelDumyPub.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${ROS_LIBS})
rclcpp_components_register_nodes(${PROJECT_NAME} "OmniMoveRobot::Omni3wdRobot")
rclcpp_components_register_nodes(${PROJECT_NAME} "OmniMoveRobot::OmniVelDumyPub")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_package()
