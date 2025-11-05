cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(ros_gz_interfaces REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/fox_detector.py
  scripts/navigation.py
  scripts/send_goal.py
  scripts/GUI.py
  DESTINATION lib/${PROJECT_NAME}
)

# Rename fox_detector for ros2 run (remove .py extension)
install(PROGRAMS scripts/fox_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fox_detector
)

install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/foxManagerNode.py
  scripts/box.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
