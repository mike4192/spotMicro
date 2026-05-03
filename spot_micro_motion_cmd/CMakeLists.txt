cmake_minimum_required(VERSION 3.8)
project(spot_micro_motion_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2cpwm_board REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(libs/spot_micro_kinematics_cpp)

include_directories(
  include/spot_micro_motion_cmd
  src/smfsm
  src/rate_limited_first_order_filter
)

add_library(smfsm
  src/smfsm/spot_micro_state.cpp
  src/smfsm/spot_micro_idle.cpp
  src/smfsm/spot_micro_stand.cpp
  src/smfsm/spot_micro_transition_stand.cpp
  src/smfsm/spot_micro_transition_idle.cpp
  src/smfsm/spot_micro_walk.cpp
)

target_link_libraries(smfsm
  spot_micro_kinematics
)

ament_target_dependencies(smfsm
  rclcpp
  std_msgs
  geometry_msgs
  i2cpwm_board
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

add_executable(${PROJECT_NAME}_node
  src/spot_micro_motion_cmd.cpp
  src/spot_micro_motion_cmd_node.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  smfsm
  spot_micro_kinematics
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  i2cpwm_board
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
