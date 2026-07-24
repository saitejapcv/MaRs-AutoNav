cmake_minimum_required(VERSION 3.8)
project(ares_nova_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  description
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_detector.py
  scripts/cone_detector.py
  scripts/master_vision.py
  scripts/cone_hunter.py
  scripts/swerve_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()