cmake_minimum_required(VERSION 3.8)
project(spot_micro_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/spotMicroPlot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/spot_micro_kinematics_python
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
