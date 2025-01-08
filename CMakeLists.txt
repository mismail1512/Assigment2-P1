cmake_minimum_required(VERSION 3.10)

project(robot_urdf)

# Find ament_cmake and ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/robot_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and URDF files
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# If there are any C++ components, include them in the build
# You can use add_executable() to add C++ executables and link libraries as necessary.
# For example, if there were a C++ file (e.g., robot_mover.cpp), you could add:
# add_executable(robot_mover_cpp src/robot_mover.cpp)
# target_link_libraries(robot_mover_cpp ${rclpy_LIBRARIES})

ament_package()

